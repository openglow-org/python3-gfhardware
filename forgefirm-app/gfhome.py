#!/usr/bin/python3
"""
gfhome - one-shot Glowforge web-service homing for ForgeFIRM

Connects the machine to the Glowforge web service just long enough for
the service to run its camera-based homing sequence (settings report ->
hunt -> lid image -> move to the home corner -> lid image). The hunt is
answered as done without moving the lens or playing its file: the
session borrows the service for its camera homing only. After the
service goes quiet the lens takes its one reference, on the hall
sensor's rising edge, moves the half-steps the controller asked for
(GFHOME_PARK_HALF_STEPS, from the edge to the park height, never onto a
stop), and the run exits. X/Y end at the back-left home corner; the
controller sets Z from the edge's focal height (the focus card's number)
and the park.

The grblHAL-glowforge controller invokes this for $H when
homing_mode = gfcloud is set in /data/forgefirm/forgefirm.conf, releasing
/dev/glowforge for the duration of the run. It can also be run by hand
(with the controller stopped or its homing session active). The same
shared config supplies optional identity overrides (gf_serial /
gf_password; the fuse identity is the fallback), managed from the
forgectrl UI. The service hostname is always derived from whichever
serial is in effect - it is never set independently.

The service ends the sequence silently - there is no completion
message - so the run is considered homed once a hunt and at least one
motion have completed and the service has been quiet for --quiet
seconds, AND the session showed physical motion. Quiet alone is not
success: against wedged stepper drivers (playback and counters run,
motors dead) the service repeats the same visual correction until it
gives up and goes silent. Two independent guards catch that: a run of
near-identical motion corrections aborts the session, and the head
accelerometer must have seen real motion at least once (it rides the
gantry; bench-characterized thresholds) before quiet counts as homed.
Nor does a service that went quiet after a motion that did not run
whole (cancelled, stopped short): the head is short of the home, and
the session fails at that motion's end.

Exit codes: 0 = homed, 1 = configuration/connection failure,
2 = homing did not complete.

Copyright 2026 514 LLC d/b/a OpenGlow
Written by Scott Wiederhold
SPDX-License-Identifier: MIT
"""
import argparse
import json
import logging
import math
import os
import queue
import shutil
import signal
import sys
import threading
import time
from pathlib import Path
from queue import Queue

from gfutilities.configuration import parse, get_cfg
from gfutilities.service.authentication import authenticate_machine
from gfutilities.service.dispatch import dispatch_action, PULS_ACTIONS
from gfutilities.service.websocket import get_session, ws_connect

import ffmachine

CONF = '/data/forgefirm/gfhome.conf'
CONF_SAMPLE = '/etc/gfhome.conf.sample'

logger = logging.getLogger('openglow')

# SIGTERM (the controller's $H budget ran out, a soft reset, a supervisor
# stopping the controller) ends the session from the handler itself: the
# running action is canceled, so a motion still downloading never starts,
# and the kernel is stopped and the latch locked at once. The handler runs
# on the main thread between bytecodes, wherever that thread is, so it takes
# no lock the main thread can be holding: _stop is only read there, and a
# SIGTERM that lands inside the handler, where _stop's set holds its lock,
# returns at once. The session then leaves through the same shutdown as
# every other exit; a lens move under way stops at its next step.
_stop = threading.Event()
_machine = None             # the session's machine, once built, for the handler
_in_handler = False         # the handler is running, on the main thread


def _on_sigterm(*_):
    global _in_handler
    if _in_handler:
        return              # the handler this one interrupted stops the machine
    _in_handler = True
    try:
        _stop.set()
        machine = _machine
        if machine is not None:
            try:
                machine.stop_now()
            except Exception:
                logger.exception('could not stop the machine from the handler')
    finally:
        _in_handler = False


def _deadline(timeout: int) -> float:
    """When the session must end, on the monotonic clock. The controller
    that spawns the runner hands over GFHOME_DEADLINE: its share of the $H
    budget, counted from the session's start, which leaves the lens moves
    and the shutdown time before the controller's own SIGTERM. Run by hand,
    or handed a value that is not a finite number, `timeout` seconds from
    now."""
    try:
        deadline = float(os.environ['GFHOME_DEADLINE'])
    except (KeyError, ValueError):
        deadline = math.nan
    return deadline if math.isfinite(deadline) else time.monotonic() + timeout


def _until(fn, deadline: float, what: str):
    """fn() on a helper thread, waited for while SIGTERM and the deadline are
    watched: the sign-in and the socket's connect block for seconds at a
    time and read neither. (True, fn's result), or (False, None) once the
    session must end, leaving the helper to the process's exit; an
    exception in fn is raised here."""
    box = {}

    def run():
        try:
            box['result'] = fn()
        except BaseException as e:
            box['error'] = e
    helper = threading.Thread(target=run, daemon=True)
    helper.start()
    while helper.is_alive():
        if _stop.is_set():
            logger.warning('stopped (SIGTERM) during %s - not homed', what)
            return False, None
        if time.monotonic() >= deadline:
            logger.error('%s ran past the session deadline - not homed', what)
            return False, None
        helper.join(0.1)
    if 'error' in box:
        raise box['error']
    return True, box.get('result')


def load_config(path: str) -> bool:
    if path == CONF and not Path(CONF).is_file() and Path(CONF_SAMPLE).is_file():
        Path(CONF).parent.mkdir(parents=True, exist_ok=True)
        # The config may carry credentials; keep the sample's owner-only
        # mode (copyfile does not preserve permissions).
        shutil.copyfile(CONF_SAMPLE, CONF)
        os.chmod(CONF, 0o600)
    if not Path(path).is_file():
        logger.error('config file %s not found', path)
        return False
    parse(path)
    if not get_cfg('SERVICE.SERVER_URL'):
        logger.error('config %s has no SERVICE section', path)
        return False
    ffmachine.setup_captures('gfhome')
    return True


# Head accelerometer (glowforge.dts head-accel, i2c-3 @0x1e), resolved
# by bus address - iio numbering follows probe order. Motion thresholds
# bench-characterized: real motion >= ~1000 counts p2p in a window,
# wedged drivers stay under ~210.
HEAD_ACCEL_BUS = '3-001e'
ACCEL_P2P_MOVING = 500
# Consecutive motion corrections within this many microsteps of each
# other count as the service repeating itself against a machine that is
# not physically moving.
REPEAT_TOL_STEPS = 60
REPEAT_LIMIT = 3


def _head_accel_dir():
    base = '/sys/bus/iio/devices'
    try:
        for node in sorted(os.listdir(base)):
            p = os.path.join(base, node)
            if HEAD_ACCEL_BUS in os.path.realpath(p):
                return p
    except OSError:
        pass
    return None


class _AccelWatch(threading.Thread):
    """Session-long physical-motion witness: counts ~2 s windows in
    which the head accelerometer's peak-to-peak cleared the motion
    threshold. Zero windows across a session that 'completed' motion
    actions means the gantry never moved."""

    def __init__(self):
        self.stop = False
        self.motion_windows = 0
        self._dir = _head_accel_dir()
        threading.Thread.__init__(self, daemon=True)

    def _read(self, axis: str) -> int:
        with open('%s/in_accel_%s_raw' % (self._dir, axis)) as f:
            return int(f.read())

    def run(self):
        if self._dir is None:
            logger.warning('head accelerometer not found - '
                           'motion witness disabled')
            return
        while not self.stop:
            lo = {'x': None, 'y': None}
            hi = {'x': None, 'y': None}
            t0 = time.monotonic()
            while time.monotonic() - t0 < 2.0 and not self.stop:
                for ax in ('x', 'y'):
                    try:
                        v = self._read(ax)
                    except (OSError, ValueError):
                        continue
                    if lo[ax] is None or v < lo[ax]:
                        lo[ax] = v
                    if hi[ax] is None or v > hi[ax]:
                        hi[ax] = v
                # ~100 Hz per axis: each read is an I2C transaction on the
                # bus the laser head sits on - an unpaced loop saturates it
                # (and pins a core) for the whole session, while ~200
                # samples per axis per window still captures the vibration
                # envelope the thresholds were built on.
                time.sleep(0.01)
            for ax in ('x', 'y'):
                if lo[ax] is not None and hi[ax] - lo[ax] >= ACCEL_P2P_MOVING:
                    self.motion_windows += 1
                    break


def home(machine, args) -> int:
    """One session, ended by args.deadline (the monotonic clock) at the
    latest, and by SIGTERM in every phase."""
    q_rx: Queue = Queue()
    q_tx: Queue = Queue()

    session = get_session()
    ok, signed_in = _until(lambda: authenticate_machine(session), args.deadline, 'the sign-in')
    if not ok:
        return 2
    if not signed_in:
        logger.error('sign-in to %s failed', get_cfg('SERVICE.SERVER_URL'))
        return 1

    # The session goes with the socket, so a reconnect signs in again for
    # a fresh single-use token rather than offering the spent one.
    ok, ws = _until(lambda: ws_connect(q_rx, q_tx, session), args.deadline, 'the socket connect')
    if not ok:
        return 2
    if not ws:
        logger.error('web socket connection failed')
        return 1

    result = 2
    try:
        try:
            machine.start(session, q_tx)
        except Exception:
            logger.exception('the machine did not start - not homed')
            return 2

        from gfhardware._common import InputSwitch
        switches = machine._sw_thread.all_switches()
        if not switches[InputSwitch.SW_DOORS]:
            logger.error('lid is open - close it and re-home')
            return 2

        t0 = time.monotonic()
        last_activity = t0
        in_flight = ''
        done = set()
        accel = _AccelWatch()
        accel.start()
        last_delta = None
        repeats = 0

        while True:
            now = time.monotonic()
            if _stop.is_set():
                logger.warning('stopped (SIGTERM) - not homed')
                return 2
            if now >= args.deadline:
                logger.error('homing ran past the session deadline (completed: %s)',
                             sorted(done) or 'nothing')
                return 2
            if 'hunt' not in done and now - t0 > args.start_timeout:
                logger.error('service did not start homing within %ds',
                             args.start_timeout)
                return 2

            busy = bool(machine.running_action_id)
            if busy:
                last_activity = now
            elif in_flight:
                if in_flight == 'motion':
                    # A motion that did not run whole (cancelled, stopped
                    # short, refused, or crashed before its end) left the
                    # head short of where the service sent it; the service
                    # goes quiet all the same, and quiet would pass for homed.
                    cancelled = getattr(machine, '_running_action_cancelled', False)
                    if cancelled or not getattr(machine, '_motion_stats', {}).get('stats'):
                        logger.error('a homing motion did not run whole (%s) - the head '
                                     'is short of the home, NOT homed',
                                     'cancelled' if cancelled else 'no end on record')
                        return 2
                logger.info('%s completed', in_flight)
                done.add(in_flight)
                if in_flight == 'motion':
                    st = getattr(machine, '_motion_stats', {}).get('stats') or {}
                    delta = (int(st.get('XEND', 0)), int(st.get('YEND', 0)))
                    if (last_delta is not None
                            and abs(delta[0] - last_delta[0]) <= REPEAT_TOL_STEPS
                            and abs(delta[1] - last_delta[1]) <= REPEAT_TOL_STEPS):
                        repeats += 1
                        if repeats >= REPEAT_LIMIT:
                            logger.error(
                                'service repeated the same correction %d '
                                'times - the machine is not physically '
                                'moving (wedged stepper drivers?)',
                                repeats + 1)
                            return 2
                    else:
                        repeats = 0
                    last_delta = delta
                in_flight = ''

            if ('hunt' in done and 'motion' in done and not busy
                    and now - last_activity >= args.quiet):
                if accel.motion_windows == 0:
                    logger.error('service went quiet but the head '
                                 'accelerometer never saw motion - NOT homed')
                    return 2
                logger.info('homing complete (service quiet %.0fs, '
                            '%d motion windows)', args.quiet,
                            accel.motion_windows)
                result = 0
                break

            try:
                msg = json.loads(q_rx.get(timeout=0.5))
            except queue.Empty:
                continue
            except ValueError:
                logger.warning('unparseable service message')
                continue
            last_activity = time.monotonic()
            logger.info('service action: %s (%s)',
                        msg.get('action_type'), msg.get('status'))
            # Homing borrows the service only for camera homing; a print must
            # never run inside a homing session (allow_print=False). The
            # answer is the action's name; the session's result stays its own.
            answer = dispatch_action(machine, msg, allow_print=False)
            if answer in PULS_ACTIONS:
                in_flight = answer
    finally:
        try:
            accel.stop = True
        except NameError:
            pass
        result = finish(machine, ws, result)
    return result


def finish(machine, ws, result: int) -> int:
    """The end of every session, whatever ended it.

    A homed run (result 0) takes its lens moves first: reference the lens
    on the hall sensor's rising edge (the controller knows its focal
    height from the focus card), then park it the half-steps the
    controller handed over; the hunt inside the session moved nothing.
    Both moves read SIGTERM before every step: one stops there, the motor
    rested, and the run is not homed. Then
    the machine is stopped and the service socket shut, in that order -
    always: nothing the lens moves raise, an exit included, may skip the
    stop that locks the laser and releases the device, and the socket's
    close, which can wait seconds for the service, never comes before it.
    Returns the result, 2 when the lens moves failed or were skipped.
    """
    if result == 0:
        try:
            from gfhardware.z_axis import LensStopped, ZAxis
            park = int(os.environ.get('GFHOME_PARK_HALF_STEPS', '0'))
            try:
                ZAxis.home(stop=_stop.is_set)
                ffmachine.park_lens(park, stop=_stop.is_set)
            except LensStopped:
                logger.warning('stopped (SIGTERM) during the lens moves - not homed')
                result = 2
        except BaseException:
            logger.exception('final Z reference or park failed')
            result = 2
    try:
        machine.stop()
    except BaseException:
        logger.exception('machine shutdown failed')
    try:
        ws.shutdown()
    except BaseException:
        logger.exception('service shutdown failed')
    return result


def main() -> int:
    ap = argparse.ArgumentParser(description='ForgeFIRM one-shot Glowforge cloud homing')
    ap.add_argument('-c', '--config', default=CONF, help='config file (default %s)' % CONF)
    ap.add_argument('--timeout', type=int, default=240,
                    help='overall time budget in seconds when run by hand; the controller '
                         'hands its own deadline over (default 240)')
    ap.add_argument('--start-timeout', type=int, default=120,
                    help='max seconds to wait for the service to begin homing (default 120)')
    ap.add_argument('--quiet', type=int, default=10,
                    help='silence after the last action that means done (default 10)')
    args = ap.parse_args()
    args.deadline = _deadline(args.timeout)

    signal.signal(signal.SIGTERM, _on_sigterm)

    # Logging first: syslog under the gfhome program name, level from
    # /data/forgefirm/forgefirm.conf (log_gfhome_disk / _remote).
    ffmachine.setup_logging('gfhome')
    if not load_config(args.config):
        return 1

    ffmachine.apply_identity_overrides()
    ffmachine.apply_user_agent()
    ffmachine.acknowledge_hunts()

    # Machine() reads the OCOTP identity and head info; it fails cleanly
    # when the controller still owns /dev/glowforge.
    try:
        machine = ffmachine.build_machine()
    except Exception:
        logger.exception('machine init failed (is the motion controller '
                         'still holding /dev/glowforge?)')
        return 1
    # A camera home under the GRBL controller: the session leaves the
    # button, the lid lamp and the rail to their owners, and locks the latch
    # as its first write.
    machine.homing_session = True
    global _machine
    _machine = machine

    rc = home(machine, args)
    logger.info('exit %d (%s)', rc, 'homed' if rc == 0 else 'not homed')
    return rc


if __name__ == '__main__':
    sys.exit(main())

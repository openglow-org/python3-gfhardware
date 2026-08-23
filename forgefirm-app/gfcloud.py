#!/usr/bin/python3
"""
gfcloud - full Glowforge web-service controller for ForgeFIRM.

Runs the machine under the Glowforge web service (the factory cloud
experience): the phone/web app drives homing, framing, and printing.
Started by the gfcloud init service when controller_mode = cloud in
/data/forgefirm.conf, which keeps grblHAL down so this daemon owns
/dev/glowforge exclusively.

Reconnects (fresh single-use ws_token) and 401 re-auth are handled in
gfutilities. On SIGTERM the service loop stops and the machine is shut
down safe (laser latched, steppers disabled, deadman released).

Offline: with --offline, or while the marker file OFFLINE_MARKER exists
at start, the machine runs under gfutilities' OfflineService instead: no
account, no network, a local socket (OFFLINE_SOCKET) that takes the
service's action messages and hands back the machine's events. The
acceptance tests use it to run the machine's print behavior without a
job from the app. The marker lives under /run, so a reboot never comes
up offline by accident.

Emulate: with --emulate, or while EMULATE_MARKER exists at start, the
real service is driven by gfutilities' Emulator in this machine's
identity instead of the hardware: it answers the service with canned
frames (EMULATOR_DIR, the dev image's gfutilities fixtures) and completes
prints without moving anything, which proves the service protocol with
nobody at the machine. Same marker rule.

No hunt: with --no-hunt, or while NOHUNT_MARKER exists at start, the
first settings report goes out in the reconnect form (no values), which
the service answers by keeping the head position it already has instead
of sending its connect-time hunt. For a restart where the service
already knows the head (the acceptance tests' restarts between tests
that do not home); a start that needs the hunt, and every fresh boot,
reports the values. Same marker rule.

Every marker is read and taken down first thing, before the imports
that take seconds on this board, by the client that starts: it applies
to that one start, never to a respawn after it, and whoever wrote it can
move on as soon as the supervisor reports the client up.

(C) Copyright 2026
Scott Wiederhold, s.e.wiederhold@gmail.com
SPDX-License-Identifier: MIT
"""
import argparse
import logging
import os
import shutil
import signal
import sys
import time
from pathlib import Path

OFFLINE_MARKER = '/run/gfcloud-offline'
OFFLINE_SOCKET = '/run/gfcloud-offline.sock'
EMULATE_MARKER = '/run/gfcloud-emulate'
NOHUNT_MARKER = '/run/gfcloud-nohunt'


def take_markers(paths=(OFFLINE_MARKER, EMULATE_MARKER, NOHUNT_MARKER)) -> dict:
    """Which one-start markers this start found, each taken down as it is
    read. Called before anything slow is imported, so the marker is gone
    within the interpreter's own start-up. A marker that cannot be
    removed is reported (and honored) once logging is up."""
    found = {}
    for path in paths:
        p = Path(path)
        if not p.exists():
            found[path] = False
            continue
        try:
            p.unlink()
            found[path] = True
        except OSError as e:
            found[path] = e
    return found


MARKERS = take_markers()

from gfutilities.configuration import parse, get_cfg   # noqa: E402
from gfutilities import GFUIService                    # noqa: E402

import ffmachine                                       # noqa: E402

CONF = '/data/etc/gfhome.conf'
CONF_SAMPLE = '/etc/gfhome.conf.sample'
EMULATOR_DIR = '/usr/share/gfutilities/emulator'
EMULATOR_WORK = '/tmp/gfcloud-emulate'

logger = logging.getLogger('openglow')


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
    ffmachine.setup_captures('gfcloud')
    return True


def main() -> int:
    ap = argparse.ArgumentParser(description='ForgeFIRM full Glowforge cloud controller')
    ap.add_argument('-c', '--config', default=CONF, help='config file (default %s)' % CONF)
    ap.add_argument('--offline', action='store_true',
                    help='no web service: take actions on %s (also when %s exists)'
                         % (OFFLINE_SOCKET, OFFLINE_MARKER))
    ap.add_argument('--emulate', action='store_true',
                    help="the real service driven by the emulator in this machine's identity, "
                         'no hardware (also when %s exists)' % EMULATE_MARKER)
    ap.add_argument('--no-hunt', action='store_true',
                    help='no connect-time hunt: the service keeps the head position it has '
                         '(also when %s exists)' % NOHUNT_MARKER)
    args = ap.parse_args()
    offline = args.offline or bool(MARKERS[OFFLINE_MARKER])
    emulate = args.emulate or bool(MARKERS[EMULATE_MARKER])
    nohunt = args.no_hunt or bool(MARKERS[NOHUNT_MARKER])

    # Logging first: syslog under the gfcloud program name, level from
    # /data/forgefirm.conf (log_gfcloud_disk / _remote).
    ffmachine.setup_logging('gfcloud')
    for marker, found in MARKERS.items():
        if found is True:
            logger.info('marker %s read and taken down: this start only', marker)
        elif found:
            logger.warning('marker %s read but not taken down (%s): a respawn would read it too',
                           marker, found)
    if not load_config(args.config):
        return 1

    ffmachine.apply_identity_overrides()

    # Machine() reads the OCOTP identity and head info; it fails cleanly if
    # grblHAL still holds /dev/glowforge (controller_mode must be cloud).
    try:
        machine = ffmachine.build_emulator(EMULATOR_DIR, EMULATOR_WORK) if emulate             else ffmachine.build_machine()
    except Exception:
        logger.exception('machine init failed (is grblHAL still running? '
                         'controller_mode must be cloud)')
        return 1

    if nohunt and not offline and not emulate:
        ffmachine.inhibit_connect_hunt()
    elif nohunt:
        logger.info('no-hunt ignored: the %s has no connect-time hunt to skip',
                    'offline service' if offline else 'emulator keeps its hunt')

    if offline:
        from gfutilities.service.offline import OfflineService
        logger.info('offline: actions from %s, no web service', OFFLINE_SOCKET)
        service = OfflineService(machine, path=OFFLINE_SOCKET)
    else:
        service = GFUIService(machine)

    def _shutdown(*_):
        logger.info('shutdown requested')
        service.request_stop()

    signal.signal(signal.SIGTERM, _shutdown)
    signal.signal(signal.SIGINT, _shutdown)

    # Run for the life of the daemon. connect() can fail if the network or
    # service is briefly unavailable (e.g. at boot); retry until stopped.
    # Once connected, run() stays up across WS drops (gfutilities reconnects
    # with a fresh token) and returns only when a stop is requested, having
    # shut the machine down safe.
    while not service.stop:
        if service.connect():
            service.run()
            break
        logger.error('connect failed; retrying in 10s')
        for _ in range(100):
            if service.stop:
                break
            time.sleep(0.1)

    logger.info('gfcloud exit')
    return 0


if __name__ == '__main__':
    sys.exit(main())

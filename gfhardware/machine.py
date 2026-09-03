"""
(C) Copyright 2020
Scott Wiederhold, s.e.wiederhold@gmail.com
https://community.openglow.org
SPDX-License-Identifier:    MIT
"""
import fcntl
import logging
import os
from threading import Event
from time import monotonic, sleep
from typing import Union

from gfutilities import BaseMachine
from gfutilities.configuration import get_cfg, set_cfg
from gfutilities.puls import generate_linear_puls
from gfutilities.service.websocket import (PULSE_REJECT_BYTES, PULSE_WARN_BYTES,
                                           fetch_motion, img_upload,
                                           motion_run_time, send_wss_event,
                                           send_wss_progress)
from gfutilities.device.settings import MACHINE_SETTINGS, update_settings

from gfhardware import id
from gfhardware._common import *
from gfhardware.cnc import *
from gfhardware.cooling import *
from gfhardware.feeder import CHUNK as FEED_CHUNK, PulseFeeder
from gfhardware.coolsvc import cooling_svc, limits_from_header, LIMIT_TAGS, INERT_LIMIT_TAGS
from gfhardware.leds import *
from gfhardware.switches import *
from gfhardware.z_axis import ZAxis

from gfhardware import cam

logger = logging.getLogger(LOGGER_NAME)

# Under the forgectrl device broker the pulse device arrives as an
# inherited fd (GF_PULSE_FD): the broker holds /dev/glowforge open for
# its lifetime, so this process must never close it - handovers and
# job boundaries no longer cycle the 40 V rail, and the broker (not
# the kernel close) is the dead-man for a writer crash. Standalone,
# the per-job open/flock/close below keeps the original semantics.
_pulse_stream = None

# The free counter is two SDMA channel-0 transactions per read, on the
# engine that clocks the steps: the watchdog asks at most this often.
ROOM_POLL_S = 5.0
_room_at = 0.0
_room_last = False


def _ring_has_room() -> bool:
    """Whether the ring could take another chunk right now.

    What tells a stalled feed apart from a full ring: a feeder with nothing
    to do because the window is full is healthy, and the same feeder with
    room in front of it and no progress behind it is not. Fails closed, so
    an unreadable ring never trips the watchdog. The answer is cached for
    ROOM_POLL_S so a quiet feed does not turn into continuous polling.
    """
    global _room_at, _room_last
    now = monotonic()
    if now - _room_at < ROOM_POLL_S:
        return _room_last
    _room_at = now
    try:
        _room_last = cnc.free > FEED_CHUNK
    except (OSError, ValueError):
        _room_last = False
    return _room_last


def _ring_room_reset() -> None:
    """Forget the cached answer: every job asks afresh."""
    global _room_at
    _room_at = 0.0


def _inherited_pulse_dev():
    global _pulse_stream
    fd = os.getenv('GF_PULSE_FD')
    if fd is None:
        return None
    if _pulse_stream is None:
        _pulse_stream = os.fdopen(int(fd), 'wb', buffering=0)
    return _pulse_stream


# How long a print may be held on the cooling verdict (WARMUP, COLD, a
# hot loop, a suspected flow fault) before it is canceled instead: the
# engine offers no resume on its fail tiers, and a hold with no end is
# not a state a job can be left in. The forgefirm.conf key
# cloud_hold_max_s overrides it (60 to 7200 s).
HOLD_MAX_DEFAULT_S = 1800.0

# The shared machine config: the same trivial "key = value" file the
# GRBL controller and forgectrl read, so both controller modes honor
# the same operator-facing tunables.
MACHINE_CONF = os.environ.get('GFHOME_CONF', '/data/forgefirm.conf')

# Feed watchdog. A live-fed run whose feeder stops making progress while the
# ring has room for a chunk is a feed that has wedged. Left alone it ends the
# same way every time: the ring plays out what it holds, tens of minutes at
# the print tick, then goes dry, and a dry ring is an underrun - an instant
# stop, a position no longer trusted, and a job that cannot be picked back up.
# The watchdog stops the machine cleanly long before that and retraces the way
# a pause does, so the feed has room to catch up and the seam is hidden if it
# does. These are supervision timeouts rather than preferences, so they live
# here rather than in the machine config.
FEED_STALL_S = 30.0        # no progress, with room to write, is a stalled feed
FEED_RECOVER_S = 60.0      # how long a held job waits for the feed to move
FEED_MAX_HOLDS = 3         # a feed that keeps stalling is sawing the material

# A print's warm-up and its rest, in seconds. The factory does both: a print
# holds 3.05 s between configuring the run and starting it, and rests about
# 10.35 s after its park before it goes idle. That is equipment protection,
# not ceremony - the warm-up is what gets air and coolant moving before the
# first fire, and the rest is what purges the enclosure and the tube after
# the last one - and the service assumes both have happened. The pulse header
# carries two keys that correlate with the periods (CCwp, CCrp), and the
# factory does nothing with them, so the numbers come from the config with the
# factory's measured behavior as the default.
WARM_UP_DEFAULT_S = 3.0
COOL_DOWN_DEFAULT_S = 10.0

# Header keys that speak to a job's lifecycle rather than to its motion: a
# park flag and the two periods above, plus one print-only flag whose meaning
# is unknown. Nothing drives behavior off them, and nothing should: the
# factory acts on none of the four either, and its own warm-up and rest come
# from somewhere other than the job. They are named in the log of every job as
# a record of what the service sends.
LIFECYCLE_KEYS = ('CFrh', 'CCwp', 'CCrp', 'CCup')

# Header keys this client acts on outside the settings table: the serial the
# job is locked to and the pulse-data format, both checked before a byte
# reaches the ring.
HEADER_CHECKED_KEYS = ('MCsn', 'PDfm')

# Header keys the service fills in per job that this machine does not act
# on, each by decision, with the decision beside it. A key in neither this
# set nor any applier is undecided, and the per-job gap line counts those
# separately: that count is the one that should be zero.
DECLARED_IGNORED = {
    # Warm-up phase fan duties and tach windows: the run profile covers the
    # warm-up hold, and every captured header sets them equal to the run
    # values anyway.
    'AAwd': 'warm-up fan profile', 'AAwn': 'warm-up fan profile', 'AAwx': 'warm-up fan profile',
    'EFwd': 'warm-up fan profile', 'EFwn': 'warm-up fan profile', 'EFwx': 'warm-up fan profile',
    'IFwd': 'warm-up fan profile', 'IFwn': 'warm-up fan profile', 'IFwx': 'warm-up fan profile',
    # Power-supply temperature window: the service sends the whole ADC range
    # (a window that cannot trip) and the factory binds it to nothing; the
    # supply's raw reading is watched per job by the cooling engine.
    'PTmn': 'supply window, unbound in the factory', 'PTmx': 'supply window, unbound in the factory',
    # Board, head, lid, interconnect and fused temperature ceilings: sent in a
    # unit that is not millidegrees and not established; the chassis (board)
    # sensor is watched per job, the others have no sensor on this platform.
    'BTrn': 'board temperature, unit unknown', 'BTrx': 'board temperature, unit unknown',
    'BTcx': 'board temperature, unit unknown',
    'HTrn': 'head temperature, no sensor', 'HTrx': 'head temperature, no sensor',
    'HTcx': 'head temperature, no sensor',
    'LTrn': 'lid temperature, no sensor', 'LTrx': 'lid temperature, no sensor',
    'LTcx': 'lid temperature, no sensor',
    'ITrn': 'interconnect temperature, no sensor', 'ITrx': 'interconnect temperature, no sensor',
    'ITcx': 'interconnect temperature, no sensor',
    'FTrn': 'fused temperature, no sensor', 'FTrx': 'fused temperature, no sensor',
    'FTcx': 'fused temperature, no sensor',
    # The coolant window in raw counts: older files only; the millidegree
    # window (CMrn/CMrx) is the one passed through.
    'CTrn': 'coolant window in raw counts (CMrn/CMrx carry it)',
    'CTrx': 'coolant window in raw counts (CMrn/CMrx carry it)',
    # Head accelerometer thresholds: unit and filter unknown; the
    # accelerometer is the motion-liveness probe here, and a crash detector
    # is its own bench-measured item.
    'HAxr': 'accelerometer threshold, unit unknown', 'HAyr': 'accelerometer threshold, unit unknown',
    'HAar': 'accelerometer threshold, unit unknown', 'HAxa': 'accelerometer threshold, unit unknown',
    'HAya': 'accelerometer threshold, unit unknown', 'HAaa': 'accelerometer threshold, unit unknown',
    # Lid IR flame thresholds: absolute per-quartile values; the lid IR
    # channels read the lid lamp, so these are the prior for the lamp-aware
    # fire watch, not a gate today.
    'IRwx': 'lid IR threshold (fire-watch prior)', 'IRwc': 'lid IR threshold (fire-watch prior)',
    'IRxx': 'lid IR threshold (fire-watch prior)', 'IRxc': 'lid IR threshold (fire-watch prior)',
    'IRyx': 'lid IR threshold (fire-watch prior)', 'IRyc': 'lid IR threshold (fire-watch prior)',
    'IRzx': 'lid IR threshold (fire-watch prior)', 'IRzc': 'lid IR threshold (fire-watch prior)',
    # High-voltage current caps: the sampled LASER_ON witness covers the idle
    # case, and HV current is ranged in every job's log line.
    'HIix': 'HV current cap (emission witness covers it)',
    'HIrx': 'HV current cap (emission witness covers it)',
    # Thermal report upload conditions: a reporting knob for the factory's
    # telemetry, which is out of scope here.
    'TRuc': 'thermal report upload conditions',
    # The pump: the cooling engine holds it on as part of its idle posture;
    # per-job pump control would belong in its per-job profile.
    'WPon': 'pump on (the engine holds it on)',
}

# How often a running print tells the service where it has gotten to. The
# factory's progress_update_interval_ms is 30 s and the app is built around
# that pace, so this is protocol parity rather than a preference and lives
# here rather than in the machine config. Phase changes report at once,
# whatever it says.
PROGRESS_INTERVAL_S = 30.0

# The driver state as the wire numbers it (CCst), which is the kernel's own
# order: a running machine reports 1, which is what a factory capture shows.
CCST_STATE = {
    MachineState.IDLE: 0,
    MachineState.RUNNING: 1,
    MachineState.DISABLED: 2,
    MachineState.FAULT: 3,
    MachineState.UNDERRUN: 4,
}


def _conf_float(key: str, default: float) -> float:
    try:
        with open(MACHINE_CONF) as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#') or '=' not in line:
                    continue
                k, v = line.split('=', 1)
                if k.strip() == key:
                    try:
                        default = float(v.strip())  # last occurrence wins
                    except ValueError:
                        pass
    except OSError:
        pass
    return default


class _JobProgress:
    """The moving bar the app shows while a job runs.

    A print is the one action long enough to need one, and the factory
    reports exactly that one: a progress frame at every phase change and
    one every 30 s in between, carrying how far the program has played
    against how long it is.

    The denominator is the whole point. The kernel's byte total counts what
    has been *enqueued*, and under a live feed that number climbs all job
    long, which is why the factory's own bar divides by a moving figure. The
    job's length is fixed and known before the first byte plays, so it is
    what this divides by, and the bar means what it says.
    """

    def __init__(self, q_tx, action_id, label: str, total: Union[int, None],
                 interval: float = None):
        self._q_tx = q_tx
        self._action_id = action_id
        self._label = label
        # Frozen for the life of the job: a denominator that moves is the
        # defect this exists to avoid.
        self._total = total if total and total > 0 else None
        self._interval = PROGRESS_INTERVAL_S if interval is None else interval
        self._last = 0.0
        self._over = False
        if self._total is None:
            logger.info('%s: job length unknown; reporting position without '
                        'a total', label)
        else:
            # The denominator, once per job: it is what every frame of this
            # job divides by, and the one number a bar that misbehaves is
            # worth checking first.
            logger.info('%s: reporting against %d bytes every %.0f s',
                        label, self._total, self._interval)

    def send(self, force: bool = False) -> None:
        """Report now if a phase changed, or if the interval has come round."""
        if self._interval <= 0 or self._q_tx is None:
            return
        now = monotonic()
        if not force and now - self._last < self._interval:
            return
        self._last = now
        try:
            pos = cnc.position
            state = cnc.state
        except (OSError, ValueError) as e:
            # Reporting must never be what ends a job.
            logger.debug('progress not reported: %s', e)
            return
        played = pos.bytes.processed
        current = played
        if self._total is not None and current > self._total:
            # The job outran the length it declared. Report it finished
            # rather than past its own end, and say so once: a bar that
            # overshoots means the declared length was wrong.
            if not self._over:
                logger.warning('%s: played %d bytes against a declared %d; '
                               'reporting the job complete',
                               self._label, current, self._total)
                self._over = True
            current = self._total
        # The frame doubles as the periodic settings report. Of the fifteen
        # tags the factory carries there, these are the four that describe
        # the job rather than the machine's sensors, plus the action they
        # belong to; the sensor readings stay excluded (cloud mode, "Scope":
        # https://docs.forgefirm.org/technical/forgefirm/cloud-mode/).
        # CCbp is the raw byte position, which is why it can sit past the
        # clamped bar rather than with it.
        send_wss_progress(self._q_tx, self._action_id, self._label, current,
                          units='steps', total=self._total,
                          values={'CAid': self._action_id or 0,
                                  'CCbp': played,
                                  'CCst': CCST_STATE.get(state, 0),
                                  'CCxp': pos.x.steps,
                                  'CCyp': pos.y.steps})
        logger.debug('%s: %d/%s', self._label, current, self._total)


class Machine(BaseMachine):
    """
    Operates the GF Hardware
    See parent class for method documentation
    """

    def __init__(self):
        # The thermal hardware (fans, pump, TEC, heater) is owned by the
        # forgectrl cooling engine; the pulse header's RUN fan duties go
        # to it as the per-job profile in the /cool/state reports. The
        # idle/cool-down duty keys are not mapped - those profiles are
        # the engine's.
        update_settings({
            'AArd': {'run': cooling_svc.profile_air_assist},
            'EFrd': {'run': cooling_svc.profile_exhaust},
            'IFrd': {'run': cooling_svc.profile_intake},
            'STfr': {'run': cnc.set_step_freq},
            'XSdm': {'run': cnc.set_x_decay},
            "XShc": {'idle': cnc.set_x_current},
            'XSmm': {'run': cnc.set_x_mode},
            'XSrc': {'run': cnc.set_x_current},
            'YSdm': {'run': cnc.set_y_decay},
            "YShc": {'idle': cnc.set_y_current},
            'YSmm': {'run': cnc.set_y_mode},
            'YSrc': {'run': cnc.set_y_current},
            'ZSmd': {'run': ZAxis.set_mode_from_puls},
        })

        self._button_pressed: bool = False
        self._motion_stats: dict = {}
        self._feeder = None
        # Set once the process is going down: a running action is told to
        # stop, and the park that would follow its cancel is skipped.
        self._shutting_down: bool = False
        self._sw_thread: SwitchMonitor = SwitchMonitor(SWITCH_DEVICE, self._switch_event)
        # Edge-to-run-loop signaling. The switch thread flags edges and
        # wakes the run loop; the run loop (the one owner of every cnc
        # write during a job) reacts on the wake instead of at its next
        # 100 ms tick, so a lid open reaches cnc/stop within a few ms.
        # _button_edges counts presses seen while a job runs (the pause /
        # resume toggle); _enclosure_edge latches a lid or interlock open
        # seen by the edge thread until the run loop consumes it.
        self._run_wake: Event = Event()
        self._button_edges: int = 0
        self._enclosure_edge: bool = False

        head = self.head_info()
        set_cfg('MACHINE.HEAD_FIRMWARE', head.version, True)
        set_cfg('MACHINE.HEAD_ID', head.hardware_id, True)
        set_cfg('MACHINE.HEAD_SERIAL', head.serial, True)

        set_cfg('MACHINE.SERIAL', id.serial(), True)
        set_cfg('MACHINE.HOSTNAME', id.hostname(), True)
        set_cfg('MACHINE.PASSWORD', id.password(), True)

        BaseMachine.__init__(self)

    # Switch polarity, everywhere below: truthy = circuit closed / OK for
    # the lid (SW_DOORS, the series chain the hardware safety chain itself
    # uses); the remote-interlock loop (SW_INTERLOCK) has the INVERTED
    # sense - it reads active only when the loop is OPEN (Basic/Plus ship
    # the 2-pin connector factory-jumpered, so it reads inactive =
    # satisfied there). SW_HV_ENABLE is the readback of the chain's
    # HV_ENABLE output and gates nothing. The hardware chain kills the
    # BEAM on lid or interlock by itself; what the checks below decide is
    # what MOTION and the job do.
    @staticmethod
    def _enclosure_open(switches: dict) -> Union[str, None]:
        """'lid opened' / 'interlock opened' when the enclosure is not
        safe, else None."""
        if not switches[InputSwitch.SW_DOORS]:
            return 'lid opened'
        if switches[InputSwitch.SW_INTERLOCK]:
            return 'interlock opened'
        return None

    @staticmethod
    def _hold_max_s() -> float:
        max_s = _conf_float('cloud_hold_max_s', HOLD_MAX_DEFAULT_S)
        if not 60.0 <= max_s <= 7200.0:
            max_s = HOLD_MAX_DEFAULT_S
        return max_s

    def _verdict_wait(self) -> None:
        """Wait out a cooling hold before the run starts.

        The armed session is open, so the engine is working on whatever
        holds it (the warm-up heater, the loop cooling). Bounded by
        cloud_hold_max_s; the lid, the interlock loop, a service cancel,
        and a verdict that goes missing end it the way they end the
        button wait: latch relocked, disarmed, job canceled.

        The engine's own armed flag is required before the verdict
        counts as clean. Until the engine has seen the armed report and
        applied the run airflow, the verdict on file is the one it
        computed for the idle session before the arm, and at idle
        nothing is wrong - so it says fire is fine while the fans are
        still at their idle duty.
        """
        max_s = self._hold_max_s()
        deadline = monotonic() + max_s
        named = None
        abort = None
        while True:
            v = cooling_svc.verdict()
            if v is None:
                abort = 'cooling verdict lost'
            elif (not v.get('hold', True) and v.get('fire_ok')
                    and v.get('armed')):
                break
            elif self._running_action_cancelled:
                abort = 'canceled'
            else:
                abort = self._enclosure_open(self._sw_thread.all_switches())
                if abort is None and monotonic() > deadline:
                    abort = 'held %.0f s' % max_s
            if abort is not None:
                break
            if v.get('verdict') != named:
                named = v.get('verdict')
                logger.info('waiting on the cooling engine: %s (%s)',
                            named, v.get('reason') or 'no reason given')
            self._run_wake.wait(.5)
            self._run_wake.clear()
        if abort is not None:
            logger.warning('cooling hold %s - relocking the laser', abort)
            cnc.laser_latch(1)
            cooling_svc.set_armed(False)
            self._running_action_cancelled = True
            set_button_color(ButtonColor.OFF)
        elif named is not None:
            logger.info('cooling verdict clean after %s; starting the run', named)

    def _button_wait(self, msg: dict) -> None:
        # The wait runs with the latch already unlocked, so it is
        # bounded and supervised the same way GRBL mode's arm window is:
        # the shared laser_button_timeout_s (default 300 s, clamped to
        # 1-3600 - out-of-range values fall back, never wait-forever)
        # bounds it, and an opened lid or interlock loop ends it (the
        # hardware button latch would ignore the press anyway). Timeout,
        # lid, interlock, and cloud cancel all relock the latch and disarm
        # before returning. Wakes on switch edges, so a press or a lid
        # open is seen within milliseconds.
        timeout_s = _conf_float('laser_button_timeout_s', 300.0)
        if not 1.0 <= timeout_s <= 3600.0:
            timeout_s = 300.0
        deadline = monotonic() + timeout_s
        self._button_pressed = False
        self._run_wake.clear()
        set_button_color(ButtonColor.WHITE)
        logger.info('waiting for button')
        abort = None
        # The press is delivered as an edge by the switch thread; the level
        # read here is the backstop should that thread be gone. A level
        # counts only after the button has been seen released during the
        # wait, so a button held (or stuck) before the wait never arms.
        seen_released = False
        while True:
            switches = self._sw_thread.all_switches()
            reason = self._enclosure_open(switches)
            if self._running_action_cancelled:
                abort = 'canceled'
                break
            if reason is not None:
                abort = reason
                break
            if self._button_pressed:
                break
            level = switches.get(InputSwitch.SW_BUTTON)
            if level is False:
                seen_released = True
            elif level and seen_released:
                logger.warning('button press seen as a level, not an edge '
                               '(is the switch thread alive?)')
                break
            if monotonic() > deadline:
                abort = 'timed out'
                break
            self._run_wake.wait(.1)
            self._run_wake.clear()
        if abort is not None:
            logger.warning('button wait %s - relocking the laser', abort)
            cnc.laser_latch(1)
            cooling_svc.set_armed(False)
            self._running_action_cancelled = True
            set_button_color(ButtonColor.OFF)

    @staticmethod
    def _config_from_pulse(state: str, header: dict):
        # Header values come from the service and go straight to motion
        # hardware (step frequency, stepper currents, microstep/decay
        # modes, fan duties): clamp each to its declared bounds before
        # applying.
        for key, setting in MACHINE_SETTINGS.items():
            val = header.get(key, None)
            if val is not None:
                func = getattr(setting, state)
                if func is not None:
                    if setting.min_value is not None and val < setting.min_value:
                        logger.warning('pulse header %s=%r below %s; clamped',
                                       key, val, setting.min_value)
                        val = setting.min_value
                    if setting.max_value is not None and val > setting.max_value:
                        logger.warning('pulse header %s=%r above %s; clamped',
                                       key, val, setting.max_value)
                        val = setting.max_value
                    func(val)

    def _head_image(self, msg: dict, settings: dict = None) -> None:
        logger.info('capturing Head Image')
        # settings is None for plain head-image requests (only lidar/hunt
        # requests carry HCil); unconditional indexing crashed those requests.
        if settings and settings.get('HCil') is not None:
            set_head_led_from_pulse(settings['HCil'])
        # exposure/gain come from the per-camera defaults in gfhardware.cam; the
        # cloud's HCex/HCga are factory-scale (1/16-line units differ) and would
        # under-expose on mainline.
        # illumination=0: the factory captured ALL head images with the white
        # torch off - added white light washes out the measure-laser dot and
        # can break the cloud's focus/hunt analysis.
        # try/finally: a failed capture must never leave the measure laser
        # lit with no owner (it was just armed from HCil above).
        try:
            img = cam.capture(cam.GFCAM_HEAD, illumination=0)
        finally:
            head_all_led_off()
        logger.info('uploading Head Image')
        img_upload(self._session, img, msg)
        if get_cfg('LOGGING.SAVE_SENT_IMAGES'):
            logger.info('saving Head Image')
            with open('%s/%s.jpeg' % (get_cfg('LOGGING.DIR'), msg['id']), 'wb') as f:
                f.write(img)

    def head_info(self) -> HeadInfo:
        (hw_id, serial, version, r5, r6) = read_file(SYSFS_GF_BASE + 'head/info').splitlines()
        return HeadInfo(
            int(hw_id.split('=')[1], 16),
            int(serial.split('=')[1]),
            int(version.split('=')[1], 16),
        )

    def _hunt(self, msg: dict) -> None:
        # A hunt is lens travel plus the service's XY hunt pattern; the
        # lid does not gate it (the factory runs a hunt with the lid open,
        # and the beam is blocked in hardware regardless).
        ZAxis.home()
        self._motion(msg, lid_gated=False)
        home_offset = int(get_cfg('MOTION.Z_HOME_OFFSET') or 0)
        if home_offset != 0:
            logger.debug('moving z to home offset %s half steps' % home_offset)
            offset_dir = Dir.Pos if home_offset > 0 else Dir.Neg
            for _ in range(abs(home_offset)):
                ZAxis.step(offset_dir)

    def _action_cleanup(self) -> None:
        """Post-action failsafe hook (BaseMachine runs it even when an action
        crashes): stop motion, lock the laser latch, extinguish the head
        emitters, and drop the pulse-device registration. The deadman fd
        itself is closed by _motion's with-block - when a crash happens
        mid-run, that close is what fires the kernel dead man's switch."""
        # cnc.stop() first: a crashed action must not leave the gantry
        # running the rest of the program unsupervised with only the beam
        # latched off. A controlled stop is a no-op when already idle, and
        # is refused in a kernel fault: every step here runs whatever the
        # ones before it did, so the latch relock and the idle report
        # never depend on the stop.
        self._safe_steps('cleanup', (
            ('stop motion', cnc.stop),
            ('lock the laser latch', lambda: cnc.laser_latch(1)),
            ('turn the head emitters off', head_all_led_off),
            ('stop the pulse feeder', self._stop_feeder),
            ('leave live-feed mode', lambda: cnc.set_streaming(False)),
            ('drop the pulse device', lambda: cnc.set_pulse_dev(None)),
            ('disarm the cooling engine', lambda: cooling_svc.set_armed(False)),
            ('report idle to the cooling engine', lambda: cooling_svc.set_mode('idle')),
            ('clear the job limits', cooling_svc.clear_limits),
        ))

    @staticmethod
    def _safe_steps(what: str, steps) -> None:
        """Run each (name, callable) in order; a step that raises is logged
        and the rest still run. For the paths that put the machine safe."""
        for name, step in steps:
            try:
                step()
            except Exception:
                logger.exception('%s: could not %s', what, name)

    def _initialize(self) -> None:
        logger.debug('initializing machine')
        self._sw_thread.start()
        # Setup machine. The thermal posture (pump, TEC, heater, fans)
        # belongs to the forgectrl cooling engine; this client only
        # starts reporting job state to it.
        cooling_svc.start()
        set_lid_led(MACHINE_SETTINGS['LLvl'].default)
        cnc.reset()
        ZAxis.reset()
        set_button_color(ButtonColor.OFF)
        cnc.enable()

    def _lid_image(self, msg: dict, settings: dict = None) -> None:
        # The flash is the service's to ask for, per shot, and a lid image is
        # the one capture it asks about: LCfl 0 means photograph the bed as it
        # is lit. Absent, the lamp comes on.
        lamp = self.lamp_level(settings)
        logger.info('capturing Lid Image (lamp %d)', lamp)
        img = cam.capture(cam.GFCAM_LID, illumination=lamp)
        logger.info('uploading Lid Image')
        img_upload(self._session, img, msg)
        if get_cfg('LOGGING.SAVE_SENT_IMAGES'):
            logger.info('saving Lid Image')
            with open('%s/%s.jpeg' % (get_cfg('LOGGING.DIR'), msg['id']), 'wb') as f:
                f.write(img)

    def _motion(self, msg: dict, lid_gated: bool = True) -> None:
        logger.info('start motion')
        self._motion_stats = {}
        if not self._safe_to_move(lid_gated):
            # Refused before anything moved. The service dead-reckons from
            # the events it gets back, so a job that never ran must end
            # ':cancelled', never ':completed'. (The service itself will
            # not print with the lid open - the app requires the lid
            # closed and imaged first - so this is a backstop.)
            self._running_action_cancelled = True
        else:
            # The job runs against a flock(LOCK_EX)'d pulse device fd:
            # the lock arms the kernel dead man's switch on the open
            # file description, and every pulse write and seek routes
            # through the one fd (cnc.set_pulse_dev() points the seek
            # helpers at it; the feeder and generate_linear_puls write to
            # the open file). Broker mode reuses the inherited, never-closed
            # fd; standalone opens and closes per job, the close being
            # what fires the dead-man if this process dies mid-print.
            inherited = _inherited_pulse_dev()
            if inherited is not None:
                fcntl.flock(inherited, fcntl.LOCK_EX)
                cnc.set_pulse_dev(inherited)
                try:
                    self._motion_locked(msg, inherited, lid_gated)
                finally:
                    cnc.set_pulse_dev(None)
            else:
                with open(PULS_DEVICE, 'wb', buffering=0) as pulse_dev:
                    fcntl.flock(pulse_dev, fcntl.LOCK_EX)
                    cnc.set_pulse_dev(pulse_dev)
                    try:
                        self._motion_locked(msg, pulse_dev, lid_gated)
                    finally:
                        cnc.set_pulse_dev(None)
        logger.info('end motion')

    def _motion_locked(self, msg: dict, pulse_dev, lid_gated: bool = True) -> None:
        """Body of a motion/print job; runs with the deadman fd held."""
        cnc.clear_all()
        # A job abandoned mid-feed could have left the device in live-feed
        # mode, where this job's ordinary end-of-data would read as a starved
        # ring. Start every job from the plain meaning.
        cnc.set_streaming(False)
        # Download puls file from service. It stays in memory: the service
        # compresses the stream tens to one, so even a job hours long is a
        # few MB held, nothing written to the eMMC, and the ring is fed from
        # it as it drains. Held in memory is why the size guards exist: they
        # bound this process, not the length of a job (which the feed no
        # longer caps). Both are forgefirm.conf keys; 0 lifts either.
        logger.info('loading motion file from %s' % msg['motion_url'])
        stats, source = fetch_motion(
            self._session, msg['motion_url'],
            warn_bytes=max(0, int(_conf_float('pulse_warn_threshold_bytes',
                                              PULSE_WARN_BYTES))),
            reject_bytes=max(0, int(_conf_float('pulse_reject_threshold_bytes',
                                                PULSE_REJECT_BYTES))))
        if not stats:
            # Rejected before anything reached the ring (bad magic, a short
            # or unusable header, or more body than this machine will hold;
            # the reason is logged where it was found): cancel cleanly
            # instead of subscripting False.
            logger.error('motion file rejected; canceling the action')
            self._running_action_cancelled = True
            return
        self._motion_stats = stats
        logger.info('motion header: %s' % self._motion_stats['header_data'])
        self._log_header_gaps(self._motion_stats['header_data'])
        # The job's limits go to the cooling engine with every report
        # from here to the end of the job; the engine applies each only
        # where it is stricter than its own.
        limits = limits_from_header(self._motion_stats['header_data'])
        cooling_svc.set_limits(limits)
        logger.info('job limits from the header: %s',
                    ' '.join('%s=%s' % kv for kv in sorted(limits.items())) or 'none')
        # Fill the ring before the operator is asked for the button, so a job
        # that cannot be loaded fails before the laser is ever armed.
        self._feeder = PulseFeeder(source, pulse_dev)
        self._feeder.start()
        try:
            self._feed_and_run(msg, source, lid_gated)
        finally:
            # On every way out, the cancel paths included. A job longer than
            # the ring leaves its feeder parked on a full ring with the rest
            # of the print in hand; left alive, it would refill the ring the
            # park clears, and the park would play the print's path.
            self._stop_feeder()

        # Cool down for prints
        if msg['action_type'] == 'print':
            self._return_home(pulse_dev)
            logger.info('start cool down')
            self._config_from_pulse('cool_down', self._motion_stats['header_data'])
            cooling_svc.set_mode('cooldown')
            self._dwell('cool_down')
            logger.info('end cool-down temps: %s' % str(temp_sensor.all))

        # Config for idle
        logger.info('start idle')
        self._config_from_pulse('idle', self._motion_stats['header_data'])
        cooling_svc.set_mode('idle')
        cooling_svc.clear_profile()
        cooling_svc.clear_limits()
        pos = cnc.position
        logger.info('end positions (%s, %s, %s)' % (pos.x.steps, pos.y.steps, pos.z.steps))

    def _stop_feeder(self) -> None:
        """Stop and drop the pulse feeder, if one is alive. Safe to repeat."""
        feeder, self._feeder = self._feeder, None
        if feeder is not None:
            feeder.stop()

    def _feed_and_run(self, msg: dict, source, lid_gated: bool) -> None:
        """The job from its first byte in the ring to the end of its run.

        Runs with the feeder alive. The caller stops the feeder on every
        exit, so this body only has to leave the cancel flag set when the
        job is not to run.
        """
        if not self._feeder.wait_primed():
            logger.error('could not load the job into the ring: %s',
                         self._feeder.error)
            self._feeder.stop()
            self._feeder = None
            self._running_action_cancelled = True
            return
        if self._feeder.finished:
            logger.info('job fits the ring: %d bytes enqueued',
                        self._feeder.written)
        else:
            logger.info('job is longer than the ring: %d bytes enqueued, '
                        'feeding the rest as it plays', self._feeder.written)
        if msg['action_type'] == 'print':
            send_wss_event(self._q_msg_tx, msg['id'], 'print:download:completed')
            # The cooling engine's verdict gates the armed window. An
            # absent engine (no verdict, or a stale one) blocks the print
            # outright: nothing would warm the loop or verify flow. A
            # standing hold (the coolant under the floor, a hot loop) is
            # waited out after the button instead, once the armed session
            # is open and the engine is working on it.
            if cooling_svc.verdict() is None:
                logger.error('no cooling verdict; the print cannot arm')
                self._running_action_cancelled = True
            else:
                cnc.laser_latch(0)
                cooling_svc.set_armed(True)
                self._button_wait(msg)
            if not self._running_action_cancelled:
                send_wss_event(self._q_msg_tx, msg['id'], 'print:warmup:starting')

        # Configure for print, and wait for warm up
        if not self._running_action_cancelled:
            self._config_from_pulse('run', self._motion_stats['header_data'])
            cooling_svc.set_mode('run')
            if msg['action_type'] == 'print':
                self._dwell('warm_up')

        # A print starts only on a clean verdict: a session that opened
        # under the warm-up gate or the floor is held here, with the
        # engine warming the loop, until it releases.
        if not self._running_action_cancelled and msg['action_type'] == 'print':
            self._verdict_wait()

        # Run motion job. Only a print pauses on the button (the factory's
        # print handler is the one that acts on the press); a motion or a
        # hunt runs straight through.
        if not self._running_action_cancelled:
            if msg['action_type'] == 'print':
                logger.info('start temps: %s' % str(temp_sensor.all))
                send_wss_event(self._q_msg_tx, msg['id'], 'print:running')
            if not self._feeder.finished:
                # End-of-data mid-run now means a starved ring, not a
                # finished job.
                self._feeder.declare_live_feed()
            progress = None
            if msg['action_type'] == 'print':
                # Only a print is long enough to need a bar, and only a
                # print is what the factory reports; a motion or a hunt is
                # over before a first frame would land.
                progress = _JobProgress(self._q_msg_tx, msg['id'],
                                        'print:progress',
                                        self._feeder.job_total)
            self._run_loop(lid_gated=lid_gated,
                           pausable=msg['action_type'] == 'print',
                           progress=progress)
            if self._feeder.finished:
                # The step totals are what the end position is checked
                # against, so let the accounting catch up before reading it.
                self._feeder.settle()
            self._feeder.stop()
            self._motion_stats['size'] = self._feeder.written
            self._motion_stats['stats'] = self._feeder.stats
            self._motion_stats['run_time'] = motion_run_time(
                self._motion_stats, self._feeder.written)
            # What the service's compression actually bought, per job. This
            # is the number the memory guards are sized against, so it is
            # worth having in the log rather than inferred from a capture.
            if source.body_size:
                logger.info('pulse data: %d bytes of body, %d bytes of program (%.1f:1)',
                            source.body_size, self._feeder.written,
                            self._feeder.written / source.body_size)
            # The job's feed is over. The park that may follow writes its own
            # small program and must not inherit this one's state.
            self._feeder = None
            cnc.laser_latch(1)
            cooling_svc.set_armed(False)
            pos = cnc.position
            expected = self._motion_stats['stats'] or {}
            logger.info('end positions (actual/expected): X (%s/%s), Y (%s/%s), Z (%s/%s)' % (
                pos.x.steps, expected.get('XEND'),
                pos.y.steps, expected.get('YEND'),
                pos.z.steps, expected.get('ZEND'),
            ))
            logger.info('motion bytes actual:%s, expected: %s' %
                        (pos.bytes.processed, self._motion_stats['size']))
            if msg['action_type'] == 'print':
                logger.info('end print temps: %s' % str(temp_sensor.all))

    def _return_home(self, pulse_dev) -> None:
        # The park is the response to an abort as much as to a finished
        # print, so it runs regardless of the cancel flag and regardless
        # of the lid: the factory parks with the lid open, and the service
        # dead-reckons from this move. Success is reported only if the
        # park ran to completion (a kernel fault is the one thing that
        # ends it early).
        logger.info('start return home')
        if self._shutting_down:
            # The process is going down: no new motion. No success is
            # reported; the service re-hunts on the next session.
            logger.warning('return home skipped: shutting down')
            return
        if self._feeder is not None:
            # Never with a feeder alive: it would refill the ring behind the
            # clear below, and the park would play the print's path. No
            # success is reported; the service re-hunts.
            logger.error('return home: the pulse feeder is still running; '
                         'stopping it and not parking')
            self._stop_feeder()
            return
        pos = cnc.position
        # The ring still holds whatever the job did not play: the rest of a
        # print aborted mid-run, or the whole print after a cancel at the
        # button wait. A run would play that first - the head walking the
        # job's path with the beam locked off - and only then the park.
        # Drop it (data and byte counters; the position counters are what
        # the park is computed from and stay), as the factory does before
        # its return home.
        try:
            cnc.clear_pulse_and_byte()
        except OSError as e:
            # Refused while the kernel still runs: never park on top of an
            # uncleared ring. No success is reported; the service re-hunts.
            logger.error('return home: ring not cleared (%s); not parking', e)
            return
        if pos.x.steps == 0 and pos.y.steps == 0:
            logger.info('return home: already at the job start')
            send_wss_event(self._q_msg_tx, self.running_action_id, 'print:return_to_home:succeeded')
            return
        generate_linear_puls(pos.x.steps * -1, pos.y.steps * -1, pulse_dev)
        # The park is the print's last leg and reports under the print, as
        # the factory reports it. Its whole program is in the ring before it
        # runs, so what the kernel counts as enqueued is the program itself
        # and is a denominator that stands still.
        try:
            park_total = cnc.position.bytes.total
        except (OSError, ValueError):
            park_total = None
        if self._run_loop(park=True,
                          progress=_JobProgress(self._q_msg_tx,
                                                self.running_action_id,
                                                'print:progress', park_total)):
            logger.warning('return home interrupted; not reporting success')
            return
        logger.info('return home complete')
        send_wss_event(self._q_msg_tx, self.running_action_id, 'print:return_to_home:succeeded')

    @staticmethod
    def _dwell(phase: str) -> float:
        """Hold before a print's first fire, or after its last.

        ``phase`` is 'warm_up' or 'cool_down'. Returns the seconds waited, so
        a caller can log what a job actually spent. Configurable to zero for
        anyone who wants the machine to skip it, which is how it shipped
        before the factory's own timings were measured.
        """
        keys = {'warm_up': ('MOTION.WARM_UP_DELAY', WARM_UP_DEFAULT_S),
                'cool_down': ('MOTION.COOL_DOWN_DELAY', COOL_DOWN_DEFAULT_S)}
        key, default = keys[phase]
        setting = get_cfg(key)
        try:
            seconds = default if setting is None else float(setting)
        except (TypeError, ValueError):
            logger.warning('%s is not a number (%r); holding %.1f s',
                           key, setting, default)
            seconds = default
        if seconds <= 0:
            # Worth a line rather than silence: a config that carries a zero
            # skips a period the service assumes was taken, and the log is
            # where that shows.
            logger.info('%s: skipped (%s = %r)', phase.replace('_', ' '),
                        key, setting)
            return 0.0
        logger.info('%s: holding %.1f s', phase.replace('_', ' '), seconds)
        sleep(seconds)
        return seconds

    @staticmethod
    def _log_header_gaps(header: dict) -> list:
        """Name what the job asked for that this machine does not act on.

        The factory takes a whole operating envelope from the pulse header;
        this client applies the motion keys and the three run fan duties. A
        header key with no applier is a decision, and it should be a recorded
        one rather than pass unremarked.
        Returns the unhandled keys, sorted.
        """
        applied = {k for k, s in MACHINE_SETTINGS.items()
                   if s.idle or s.run or s.cool_down}
        known = applied.union(HEADER_CHECKED_KEYS, LIFECYCLE_KEYS, LIMIT_TAGS, INERT_LIMIT_TAGS)
        logger.info('job lifecycle keys: %s',
                    ' '.join('%s=%s' % (k, header.get(k, '-')) for k in LIFECYCLE_KEYS))
        unhandled = sorted(k for k in header if k not in known)
        declared = [k for k in unhandled if k in DECLARED_IGNORED]
        gaps = [k for k in unhandled if k not in DECLARED_IGNORED]
        if unhandled:
            logger.info('%d of %d header keys have no applier here '
                        '(%d declared ignored, %d undecided)',
                        len(unhandled), len(header), len(declared), len(gaps))
            # Every job is then its own capture of what the service sends and
            # this machine ignores, which is what the disposition work needs.
            if gaps:
                logger.debug('header keys with no applier: %s',
                             ' '.join('%s=%s' % (k, header[k]) for k in gaps))
            if declared:
                logger.debug('header keys declared ignored: %s',
                             ' '.join('%s=%s' % (k, header[k]) for k in declared))
        return gaps

    def _retrace(self, backtrack: int) -> tuple:
        """Walk back over ground the job already cut, with the laser off.

        Called with the machine stopped and idle, by the button pause and by
        the feed watchdog alike. Returns ``(ok, retraced)``: ``ok`` is False
        only if the kernel faulted, and ``retraced`` is the ticks actually
        walked, which the ring's retained history bounds. A refusal is not
        fatal - the job holds where the deceleration left it - but the resume
        that follows has to lead by what was retraced rather than by what was
        asked for.
        """
        try:
            budget = max(0, cnc.max_backtrack)
        except (OSError, ValueError):
            # No readback: ask for the configured distance and let the kernel
            # refuse it if the history is short.
            budget = backtrack
        retraced = min(backtrack, budget)
        if retraced < backtrack:
            logger.info('retracing %d ticks of the %d asked for; that is what '
                        'the ring still holds', retraced, backtrack)
        if retraced <= 0:
            return True, 0
        try:
            cnc.resume(-retraced)
        except OSError as e:
            # Not fatal: hold where the decel stopped, and the resume then
            # brings the laser back on from the start.
            logger.warning('backtrack refused (%s); holding in place', e)
            return True, 0
        if not self._wait_kernel_idle():
            return False, retraced
        return True, retraced

    def _resume_retraced(self, retraced: int, overlap: int) -> bool:
        """Pick the program back up after a retraced hold.

        The lead is what was retraced less the overlap, so the beam returns
        over ground the job already cut. Never zero: zero is the kernel's
        "forward without re-enabling the laser", which would finish the job
        dark.
        """
        lead = max(1, retraced - overlap)
        logger.info('resuming (laser lead %d ticks)', lead)
        try:
            cnc.resume(lead)
        except OSError as e:
            logger.error('resume refused (%s); canceling', e)
            return False
        return True

    def _wait_kernel_idle(self, timeout_s: float = 10.0) -> bool:
        """After a stop or a backtrack: True once the kernel reports idle
        (the controlled decel has played out), False on timeout/fault."""
        deadline = monotonic() + timeout_s
        while monotonic() < deadline:
            state = cnc.state
            if state is MachineState.IDLE:
                return True
            if state is not MachineState.RUNNING:
                return False
            sleep(.01)
        return False

    def _run_loop(self, park: bool = False, lid_gated: bool = True,
                  pausable: bool = False, progress: '_JobProgress' = None) -> bool:
        """Play the loaded program. Returns True if the run was aborted
        (stopped before the program's end), False if it ran to completion.

        Reactions during the run - the factory's, both modes alike:
          - lid or interlock loop opens: controlled stop, job canceled
            (a print then parks; the park itself ignores the lid);
          - service cancel: the same stop;
          - cooling verdict pulled: latch relocked, the same stop;
          - button press (prints only): pause - stop, backtrack
            cloud_pause_backtrack_ticks with the laser off, hold; the
            next press resumes - forward, with the laser re-enabled
            cloud_pause_backtrack_ticks minus cloud_resume_lead_ticks
            short of the pause point, so the beam returns over ground
            the job already cut. A pause in a program's first moments
            retraces as far as the ring holds and leads by the same
            amount less. Lid/interlock/cancel while paused cancel the
            job from where it stands.
          - a live feed that stops making progress while the ring has
            room for it: the same stop and retrace, held until the feed
            moves again, and canceled rather than left standing if it
            does not. A dry ring is an underrun and a scrapped job; this
            is the same job with a hidden seam.
        The switch thread wakes this loop on every edge, so a reaction
        lands within milliseconds; the level read each pass is the
        backstop for an edge the thread missed.

        ``progress``, when a caller supplies one, reports the run to the
        service: once as it starts, at every pause, resume and hold, once
        more when it ends, and on its own interval in between.
        """
        _ring_room_reset()
        logger.info('starting run')
        logger.info('current state: %s' % cnc.state)
        set_button_color(ButtonColor.WHITE)
        self._button_edges = 0
        self._enclosure_edge = False
        self._run_wake.clear()
        cnc.run()
        # Wait for state transition
        wait_time = 20
        while cnc.state is not MachineState.RUNNING and wait_time != 0:
            wait_time = wait_time - 1
            sleep(.1)
        logger.info('current state: %s' % cnc.state)
        backtrack = int(_conf_float('cloud_pause_backtrack_ticks', 2000))
        lead = int(_conf_float('cloud_resume_lead_ticks', 1950))
        # The factory's two constants say one thing: come back on this many
        # ticks before the point the pause stopped at. Keeping the overlap
        # rather than the lead is what lets a short retrace stay correct - a
        # lead longer than the ground retraced would put the beam back on
        # past the pause point, leaving the cut unburned there.
        overlap = max(0, backtrack - lead)
        retraced = 0
        aborted = False
        paused = False
        # Feed watchdog state: the last progress seen from the feeder and
        # when, whether the job is being held for it, and how many times it
        # has had to be.
        feed_mark = self._feeder.written if self._feeder is not None else 0
        feed_at = monotonic()
        feed_held = False
        feed_deadline = 0.0
        feed_holds = 0
        # The cooling verdict: whether the job is held on it, how long it
        # may be, and whether this loop locked the latch on its account.
        verdict_held = False
        verdict_latched = False
        hold_deadline = 0.0
        hold_max_s = self._hold_max_s()
        if progress is not None:
            progress.send(force=True)
        while True:
            state = cnc.state
            if progress is not None:
                progress.send()
            armed = cooling_svc.armed
            v = cooling_svc.verdict() if armed else None
            v_fire_ok = bool(v and v.get('fire_ok'))
            v_hold = armed and (v is None or bool(v.get('hold', True)))
            v_resume = bool(v and v.get('resume_ok') and not v.get('hold', True))
            if state is MachineState.UNDERRUN:
                # A live-fed ring went dry mid-run. The stop was instant, so
                # steps were skipped at speed: the position is not to be
                # trusted, and the job did not finish. Acknowledge it (which
                # returns the device to idle) and report the job canceled.
                logger.error('pulse buffer ran dry mid-run after %s bytes; '
                             'position is no longer trusted',
                             cnc.position.bytes.processed)
                cnc.stop()
                self._running_action_cancelled = True
                aborted = True
                break
            if (state is not MachineState.RUNNING and not paused and not feed_held
                    and not verdict_held):
                break                       # program ended, or the kernel faulted
            switches = self._sw_thread.all_switches()
            enclosure = self._enclosure_open(switches)
            if enclosure is None and self._enclosure_edge:
                enclosure = 'lid opened'    # an edge the level read already missed
            self._enclosure_edge = False
            # A locally-aborted run must not report ':completed' to the
            # service: marking the action canceled routes the finish
            # through the ':cancelled' event.
            if (not park and self._feeder is not None
                    and self._feeder.error is not None):
                logger.error('pulse feed failed mid-run (%s); stopping motion',
                             self._feeder.error)
                self._running_action_cancelled = True
                aborted = True
            elif self._running_action_cancelled and not park:
                logger.warning('action canceled mid-run; stopping motion')
                aborted = True
            elif enclosure is not None and lid_gated and not park:
                logger.warning('%s mid-run; stopping motion', enclosure)
                self._running_action_cancelled = True
                aborted = True
            if aborted:
                if not paused and not feed_held and not verdict_held:
                    cnc.stop()
                    self._wait_kernel_idle()
                break

            # The cooling verdict (the cooling-engine contract, the same
            # as the GRBL controller's). fire_ok gates the beam and acts
            # at once: the latch. hold pauses the job the way the button
            # does (stop, retrace, hold) and resume_ok picks it back up
            # with the latch back; a hold that outlasts its bound cancels
            # the job. A verdict gone missing is the engine gone with the
            # laser hot: fire blocked and held, the check heater off, and
            # the run airflow written once by this process, as nobody
            # else will now.
            if armed and not v_fire_ok and not verdict_latched:
                verdict_latched = True
                cnc.laser_latch(1)
                if v is None:
                    logger.error('cooling verdict lost mid-run; fire blocked, holding')
                    WaterPump.heater_off()
                    cooling_svc.fallback_airflow()
                else:
                    logger.error('cooling verdict pulled fire mid-run: %s (%s)',
                                 v.get('verdict'), v.get('reason') or 'no reason given')
            if verdict_held:
                if v_resume and v_fire_ok:
                    if verdict_latched:
                        cnc.laser_latch(0)
                        verdict_latched = False
                    logger.info('cooling verdict clean; resuming')
                    if not self._resume_retraced(retraced, overlap):
                        self._running_action_cancelled = True
                        aborted = True
                        break
                    verdict_held = False
                    if pausable:
                        send_wss_event(self._q_msg_tx, self.running_action_id,
                                       'print:resumed')
                    if progress is not None:
                        progress.send(force=True)
                    sleep(.05)
                    continue
                if monotonic() > hold_deadline:
                    logger.error('held on the cooling verdict for %.0f s (%s); '
                                 'canceling the job', hold_max_s,
                                 v.get('verdict') if v else 'no verdict')
                    self._running_action_cancelled = True
                    aborted = True
                    break
            elif v_hold and not paused and not feed_held:
                logger.warning('cooling verdict asks for a hold (%s); pausing',
                               v.get('verdict') if v else 'no verdict')
                cnc.stop()
                if not self._wait_kernel_idle():
                    break               # fault: the state read above ends the loop
                pos = cnc.position
                if pos.bytes.processed >= pos.bytes.total:
                    break               # the decel ended the program: done
                ok, retraced = self._retrace(backtrack)
                if not ok:
                    break
                verdict_held = True
                hold_deadline = monotonic() + hold_max_s
                if pausable:
                    send_wss_event(self._q_msg_tx, self.running_action_id,
                                   'print:paused')
                if progress is not None:
                    progress.send(force=True)
                logger.info('held at %s, waiting for the cooling verdict', cnc.position)
                continue

            # The feed watchdog. Only a live-fed job can starve: one that fit
            # the ring is enqueued whole and has nothing left to wait for.
            if self._feeder is not None and not self._feeder.finished:
                now = monotonic()
                moved = self._feeder.written != feed_mark
                if moved:
                    feed_mark, feed_at = self._feeder.written, now
                if feed_held:
                    if moved and v_hold:
                        # The feed is back but the engine holds: the same
                        # hold, now the verdict's, resumed on its terms.
                        logger.info('the pulse feed moved again, but the cooling '
                                    'verdict holds (%s); staying held',
                                    v.get('verdict') if v else 'no verdict')
                        feed_held = False
                        verdict_held = True
                        hold_deadline = monotonic() + hold_max_s
                        continue
                    if moved:
                        logger.info('the pulse feed moved again after %d bytes; '
                                    'resuming the job', self._feeder.written)
                        if verdict_latched and v_fire_ok:
                            cnc.laser_latch(0)
                            verdict_latched = False
                        if not self._resume_retraced(retraced, overlap):
                            self._running_action_cancelled = True
                            aborted = True
                            break
                        feed_held = False
                        if pausable:
                            send_wss_event(self._q_msg_tx, self.running_action_id,
                                           'print:resumed')
                        if progress is not None:
                            progress.send(force=True)
                        # Give the kernel a moment to leave idle before the
                        # next pass reads the state.
                        sleep(.05)
                        continue
                    if now > feed_deadline:
                        logger.error('the pulse feed did not move in %.0f s of '
                                     'waiting; canceling the job', FEED_RECOVER_S)
                        self._running_action_cancelled = True
                        aborted = True
                        break
                elif not paused and now - feed_at > FEED_STALL_S and _ring_has_room():
                    feed_holds += 1
                    logger.error('the pulse feed has not moved in %.0f s with room '
                                 'in the ring (%d bytes fed); stopping the job '
                                 'before the ring runs dry', now - feed_at, feed_mark)
                    if feed_holds > FEED_MAX_HOLDS:
                        logger.error('the feed has stalled %d times this job; '
                                     'canceling rather than cutting it in pieces',
                                     feed_holds)
                        self._running_action_cancelled = True
                        aborted = True
                        cnc.stop()
                        self._wait_kernel_idle()
                        break
                    cnc.stop()
                    if not self._wait_kernel_idle():
                        break               # fault: the state read above ends the loop
                    pos = cnc.position
                    if pos.bytes.processed >= pos.bytes.total:
                        break               # the decel ended the program: done
                    ok, retraced = self._retrace(backtrack)
                    if not ok:
                        break
                    feed_held = True
                    feed_deadline = monotonic() + FEED_RECOVER_S
                    if pausable:
                        send_wss_event(self._q_msg_tx, self.running_action_id,
                                       'print:paused')
                    if progress is not None:
                        progress.send(force=True)
                    logger.info('held at %s, waiting for the feed', cnc.position)
                    continue

            # A press while the job is held for the feed is not lost: it is
            # left to be read once the job is moving again, where pausing is
            # a thing the machine can actually do.
            if pausable and self._button_edges and verdict_held:
                self._button_edges = 0
                logger.info('button pressed while held on the cooling verdict; '
                            'staying held')
            if pausable and self._button_edges and not feed_held:
                self._button_edges = 0
                if paused and v_hold:
                    logger.info('button pressed while paused, but the cooling '
                                'verdict holds (%s); staying held',
                                v.get('verdict') if v else 'no verdict')
                    paused = False
                    verdict_held = True
                    hold_deadline = monotonic() + hold_max_s
                    continue
                if paused:
                    logger.info('button pressed while paused')
                    if verdict_latched and v_fire_ok:
                        cnc.laser_latch(0)
                        verdict_latched = False
                    if not self._resume_retraced(retraced, overlap):
                        self._running_action_cancelled = True
                        aborted = True
                        break
                    paused = False
                    send_wss_event(self._q_msg_tx, self.running_action_id,
                                   'print:resumed')
                    if progress is not None:
                        progress.send(force=True)
                    # Give the kernel a moment to leave idle before the
                    # next pass reads the state.
                    sleep(.05)
                    continue
                logger.info('button pressed mid-run; pausing')
                cnc.stop()
                if not self._wait_kernel_idle():
                    break                   # fault: the state read above ends the loop
                pos = cnc.position
                if pos.bytes.processed >= pos.bytes.total:
                    break                   # the decel ended the program: done
                ok, retraced = self._retrace(backtrack)
                if not ok:
                    break
                paused = True
                send_wss_event(self._q_msg_tx, self.running_action_id,
                               'print:paused')
                if progress is not None:
                    progress.send(force=True)
                logger.info('paused at %s', cnc.position)
                continue
            if (paused or feed_held or verdict_held) and state not in (MachineState.IDLE,
                                                                       MachineState.RUNNING):
                break                       # the kernel faulted while held
            self._run_wake.wait(.1)
            self._run_wake.clear()
        end_state = cnc.state
        logger.info('current state: %s' % end_state)
        if not aborted and end_state is not MachineState.IDLE:
            # The kernel left the run on its own (a fault, a disable): the
            # program did not end, and the position is not to be trusted.
            # Never ':completed' - the service would dead-reckon from it.
            logger.error('run ended in kernel state %s; the job did not finish',
                         end_state.name.lower())
            self._running_action_cancelled = True
            aborted = True
        elif not aborted and not park:
            # Idle short of the program's end: something other than this
            # loop stopped the run (the cooling engine's own stop on a
            # fail tier, a supervisor). The job did not finish.
            try:
                pos = cnc.position
                short = pos.bytes.processed < pos.bytes.total
            except (OSError, ValueError):
                short = False
            if short:
                logger.error('run stopped short: %d of %d bytes played; the job '
                             'did not finish', pos.bytes.processed, pos.bytes.total)
                self._running_action_cancelled = True
                aborted = True
        set_button_color(ButtonColor.OFF)
        if progress is not None:
            # Where the job actually ended, whether that is the end of the
            # program or wherever it was stopped.
            progress.send(force=True)
        logger.info('finished run')
        return aborted

    def _safe_to_move(self, lid_gated: bool = True) -> bool:
        switches = self._sw_thread.all_switches()
        reason = self._enclosure_open(switches)
        if reason is not None and lid_gated:
            logger.info('%s, unsafe to move', reason)
            return False
        if cnc.state is not MachineState.IDLE:
            logger.info('machine is not idle, state: %s' % cnc.state.value)
            return False
        temp = temp_sensor.water_2.C
        # A backstop behind the cooling engine's own start gate (the
        # verdict is checked again before arming); a config without the
        # key leaves the gate to the engine.
        limit = get_cfg('THERMAL.MAX_START_TEMP')
        if limit is not None and temp > float(limit):
            logger.info('machine temp is too high, temp: %s' % temp)
            return False
        if temp <= -100:
            # A dead or disconnected coolant sensor reads the -273.15
            # sentinel, which must not pass the gate as "cold enough".
            logger.info('coolant sensor reads invalid (%s); unsafe to move' % temp)
            return False
        return True

    def _shutdown(self) -> None:
        logger.info('shutting down')
        # Safe posture in EVERY mode: under the broker this process's
        # exit is not a final close of the pulse device, so neither the
        # kernel dead-man nor the close-relock fires on it. Stop motion
        # and lock the latch explicitly, then hand the cooling engine a
        # final disarmed/idle report so it stands down through cooldown.
        # A running action is told to stop first and waited for: left to
        # itself it would read the stopped kernel as its program's end
        # and park after this stop, and the park would race the exit.
        # Every step runs whatever the ones before it did (the stop is
        # refused in a kernel fault).
        self._shutting_down = True
        self._running_action_cancelled = True
        self._run_wake.set()
        self._safe_steps('shutdown', (
            ('stop motion', cnc.stop),
            ('lock the laser latch', lambda: cnc.laser_latch(1)),
        ))
        thread = getattr(self, '_action_thread', None)
        if thread is not None and thread.is_alive():
            logger.info('waiting for the running action to stop')
            thread.join(10.0)
            if thread.is_alive():
                logger.error('the running action did not stop')
        self._safe_steps('shutdown', (
            ('lock the laser latch', lambda: cnc.laser_latch(1)),
            ('disarm the cooling engine', lambda: cooling_svc.set_armed(False)),
            ('report idle to the cooling engine', lambda: cooling_svc.set_mode('idle')),
            ('clear the job limits', cooling_svc.clear_limits),
        ))
        cooling_svc.stop = True
        self._sw_thread.stop = True
        logger.info('joining switch thread')
        self._sw_thread.join()
        # Rail policy belongs to the forgectrl broker when it owns the
        # device: disabling on handback would drop the rail out from
        # under the next writer. Standalone stands the rail down.
        if _inherited_pulse_dev() is None:
            cnc.disable()
        logger.info('shut down complete')

    def _switch_event(self, event: SwitchEvent) -> None:
        # Runs on the switch thread: report the edge to the service, flag
        # it for the button wait / run loop, and wake them. Nothing here
        # touches the cnc - the run loop owns those writes.
        logger.debug('received switch event %s' % str(event))
        if event.code == InputSwitch.SW_BUTTON:
            if event.val:
                logger.info('button pushed')
                send_wss_event(self._q_msg_tx, None, 'button:pressed')
                self._button_pressed = True
                self._button_edges += 1
                self._run_wake.set()
            else:
                logger.info('button released')
                send_wss_event(self._q_msg_tx, None, 'button:released')
        elif event.code == InputSwitch.SW_DOORS:
            if event.val:
                logger.info('lid closed')
                send_wss_event(self._q_msg_tx, None, 'lid:closed')
            else:
                logger.info('lid opened')
                send_wss_event(self._q_msg_tx, None, 'lid:opened')
                self._enclosure_edge = True
                self._run_wake.set()
        elif event.code == InputSwitch.SW_INTERLOCK:
            # Active = the remote-interlock loop OPENED. Not reported to
            # the service (cloud mode, docs.forgefirm.org); gates the job like the lid.
            if event.val:
                logger.info('interlock loop opened')
                self._enclosure_edge = True
                self._run_wake.set()
            else:
                logger.info('interlock loop closed')

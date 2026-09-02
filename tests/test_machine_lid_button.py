"""
(C) Copyright 2026
Scott Wiederhold, s.e.wiederhold@gmail.com
https://community.openglow.org

SPDX-License-Identifier:    MIT

Host tests for the cloud-mode job supervision in gfhardware.machine: how
a running or waiting job reacts to the lid, the interlock loop, the
button, and a service cancel, and what it reports about itself while it
runs. The real Machine code runs against a fake kernel wrapper (records
every cnc write, plays a simple state machine) and a fake switch monitor
(settable switch word, edge delivery on the switch thread), with the wire
events captured.

Run:  PYTHONPATH=.:../Glowforge-Utilities python3 -m unittest tests.test_machine_lid_button
"""
import json
import importlib.util
import os
import queue
import sys
import tempfile
import threading
import time
import types
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(os.path.dirname(ROOT), 'Glowforge-Utilities'))
os.environ['GFHOME_CONF'] = os.path.join(tempfile.gettempdir(), 'no-such-forgefirm.conf')

# The package __init__ imports the machine, which imports the hardware
# modules (one of them a compiled evdev extension): register a bare
# package first so gfhardware._common loads without it, then the fakes,
# then the machine.
_pkg = types.ModuleType('gfhardware')
_pkg.__path__ = [os.path.join(ROOT, 'gfhardware')]
sys.modules['gfhardware'] = _pkg

try:
    import fcntl                                                   # noqa: F401
except ImportError:
    # The job holds the pulse device flock'd for the kernel dead man's
    # switch. The lock is not what these tests exercise, so a stub keeps
    # them runnable on a host without it.
    _fcntl = types.ModuleType('fcntl')
    _fcntl.LOCK_EX = 2
    _fcntl.flock = lambda *a, **k: None
    sys.modules['fcntl'] = _fcntl

from gfhardware._common import (InputSwitch, MachineState, ButtonColor,   # noqa: E402
                                SwitchEvent, Position, AxisPosition, PulsPosition,
                                HeadInfo)


# ---------------------------------------------------------------- fakes

class FakeCNC:
    """Kernel wrapper stand-in. state plays: run() -> RUNNING for
    `run_reads` state reads, then IDLE (the program's end); stop() ->
    IDLE at once; resume(-n) -> RUNNING for `backtrack_reads` reads then
    IDLE; resume(+n) -> RUNNING again for the remainder."""

    def __init__(self):
        self.writes = []
        self._state = MachineState.IDLE
        self._reads_left = 0
        self.run_reads = 10 ** 9
        self.backtrack_reads = 3
        self._backward = False
        self.processed = 100
        self.total = 1000
        self.latch = 1
        # Ring room for the feed watchdog: plenty, unless a test says so.
        self.free = 10 ** 7
        # Steps of played, still-resident program a backward run can walk.
        # Plenty, unless a test says otherwise.
        self.backtrack_budget = 10 ** 6
        self.max_backtrack_error = None
        self.resume_error = None
        self.clear_error = None
        self.xy_steps = (10, 10)
        self.streaming_writes = []
        # After this many state reads a live-fed run reports a dry ring.
        self.underrun_after = None
        # After this many state reads the kernel faults out of the run.
        self.fault_after = None
        # The kernel refuses the stop (EPERM in a fault).
        self.stop_error = None

    # -- attributes the Machine constructor maps pulse-header keys onto
    def set_step_freq(self, v): self.writes.append(('step_freq', v))
    def set_x_decay(self, v): pass
    def set_x_current(self, v): pass
    def set_x_mode(self, v): pass
    def set_y_decay(self, v): pass
    def set_y_current(self, v): pass
    def set_y_mode(self, v): pass

    # -- job path
    def run(self):
        self.writes.append(('run', 1))
        self._state = MachineState.RUNNING
        self._backward = False
        self._reads_left = self.run_reads

    def stop(self):
        if self.stop_error is not None:
            raise self.stop_error
        self.writes.append(('stop', 1))
        self._state = MachineState.IDLE

    def resume(self, steps):
        if self.resume_error is not None:
            raise self.resume_error
        self.writes.append(('resume', int(steps)))
        self._state = MachineState.RUNNING
        self._backward = steps < 0
        self._reads_left = self.backtrack_reads if steps < 0 else self.run_reads

    def laser_latch(self, v):
        self.writes.append(('laser_latch', int(v)))
        self.latch = int(v)

    def clear_all(self): self.writes.append(('clear_all', 1))

    def clear_pulse_and_byte(self):
        if self.clear_error is not None:
            raise self.clear_error
        self.writes.append(('clear_pulse', 1))

    def set_streaming(self, val):
        self.streaming_writes.append(int(val))

    def set_pulse_dev(self, dev): pass
    def enable(self): pass
    def disable(self): pass
    def reset(self): pass

    @property
    def state(self):
        if self._state is MachineState.RUNNING:
            if self.underrun_after is not None:
                self.underrun_after -= 1
                if self.underrun_after <= 0:
                    self._state = MachineState.UNDERRUN
                    return self._state
            if self.fault_after is not None:
                self.fault_after -= 1
                if self.fault_after <= 0:
                    self._state = MachineState.FAULT
                    return self._state
            self._reads_left -= 1
            if self._reads_left <= 0:
                self._state = MachineState.IDLE
                # A backward run ends where it was asked to stop; only a
                # forward one can have played the program out.
                if not self._backward:
                    self.processed = self.total
        return self._state

    @property
    def max_backtrack(self):
        if self.max_backtrack_error is not None:
            raise self.max_backtrack_error
        return self.backtrack_budget

    @property
    def position(self):
        x = AxisPosition(self.xy_steps[0], self.xy_steps[0] / 53.333, 0.0)
        y = AxisPosition(self.xy_steps[1], self.xy_steps[1] / 53.333, 0.0)
        z = AxisPosition(0, 0.0, 0.0)
        return Position(x, y, z, PulsPosition(self.total, self.processed))


class FakeFeeder:
    """Feeder stand-in. ``written`` stands still until a test says the feed
    is moving, which is the whole difference between a wedged feeder and a
    working one."""

    def __init__(self, written=0, finished=False, moving=False):
        self._written = written
        self.moving = moving
        self.finished = finished
        self.error = None
        self.stopped = False

    @property
    def written(self):
        if self.moving:
            self._written += 1
        return self._written

    def stop(self, timeout=5.0):
        self.stopped = True


class FakeSwitches:
    """Switch monitor stand-in: a settable switch word plus edge delivery
    into the machine's handler on a helper thread (as the real monitor
    thread would)."""

    def __init__(self):
        # Pending edge timers from the previous test must never land in
        # this one's run loop.
        for t in getattr(self, 'timers', []):
            t.cancel()
        for t in getattr(self, 'timers', []):
            t.join(1.0)
        self.timers = []
        self.word = {s: False for s in InputSwitch}
        self.word[InputSwitch.SW_DOOR1] = True
        self.word[InputSwitch.SW_DOOR2] = True
        self.word[InputSwitch.SW_DOORS] = True
        self.handler = None
        self.started = False

    def start(self): self.started = True
    def join(self): pass
    stop = False

    def all_switches(self):
        return dict(self.word)

    def _edge(self, code, val):
        self.word[code] = val
        if self.handler:
            self.handler(SwitchEvent(0, 0, 5, code, val))

    def _later(self, delay, fn):
        t = threading.Timer(delay, fn)
        self.timers.append(t)
        t.start()

    def press(self, delay=0.0):
        self._later(delay, lambda: (self._edge(InputSwitch.SW_BUTTON, True),
                                    self._edge(InputSwitch.SW_BUTTON, False)))

    def open_lid(self, delay=0.0):
        self._later(delay, lambda: (self._edge(InputSwitch.SW_DOORS, False),
                                    self._edge(InputSwitch.SW_DOOR1, False)))

    def close_lid(self, delay=0.0):
        self._later(delay, lambda: (self._edge(InputSwitch.SW_DOOR1, True),
                                    self._edge(InputSwitch.SW_DOORS, True)))

    def open_interlock(self, delay=0.0):
        self._later(delay, lambda: self._edge(InputSwitch.SW_INTERLOCK, True))


class FakeCooling:
    """The engine's verdict as the fake sees it: fire_ok, and a hold that
    follows it unless a test says otherwise (the engine never blocks fire
    without asking for a hold); resume_ok is the hold's complement. stale
    stands for a missing or outdated verdict file."""

    def __init__(self):
        self.armed = False
        self.mode = 'idle'
        self._fire_ok = True
        self._hold = None
        self.stale = False
        self.name = 'OK'
        self.fallbacks = 0

    def set_armed(self, a): self.armed = bool(a)
    def set_mode(self, m): self.mode = m
    def fire_ok(self): return not self.stale and self._fire_ok

    def verdict(self):
        if self.stale:
            return None
        hold = (not self._fire_ok) if self._hold is None else self._hold
        return {'fire_ok': self._fire_ok, 'hold': hold, 'resume_ok': not hold,
                'verdict': self.name, 'reason': ''}

    def hold(self):
        v = self.verdict()
        return v is None or v['hold']

    def resume_ok(self):
        v = self.verdict()
        return bool(v and v['resume_ok'])

    def fallback_airflow(self): self.fallbacks += 1
    def clear_profile(self): pass
    def set_limits(self, limits): self.limits = dict(limits)
    def clear_limits(self): self.limits = {}
    def start(self): pass
    def profile_air_assist(self, v): pass
    def profile_exhaust(self, v): pass
    def profile_intake(self, v): pass


def _install_fakes():
    """Register stand-ins for the hardware modules before gfhardware.machine
    is imported (it star-imports them at module load)."""
    cnc_mod = types.ModuleType('gfhardware.cnc')
    cnc_mod.cnc = FakeCNC()
    cnc_mod.__all__ = ['cnc']

    sw_mod = types.ModuleType('gfhardware.switches')
    fake_sw = FakeSwitches()

    class SwitchMonitor:
        def __new__(cls, dev, handler):
            fake_sw.handler = handler
            return fake_sw
    sw_mod.SwitchMonitor = SwitchMonitor
    sw_mod.__all__ = ['SwitchMonitor']

    leds_mod = types.ModuleType('gfhardware.leds')
    leds_mod.button_colors = []
    leds_mod.set_button_color = lambda c: leds_mod.button_colors.append(c)
    leds_mod.head_all_led_off = lambda: None
    leds_mod.set_lid_led = lambda v: None
    leds_mod.set_head_led_from_pulse = lambda v: None
    leds_mod.__all__ = ['set_button_color', 'head_all_led_off', 'set_lid_led',
                        'set_head_led_from_pulse']

    cooling_mod = types.ModuleType('gfhardware.cooling')

    class _Temp:
        C = 20.0

    class _Sensors:
        water_2 = _Temp()
        all = {}
    cooling_mod.temp_sensor = _Sensors()

    class WaterPump:
        @staticmethod
        def heater_off(): pass
    cooling_mod.WaterPump = WaterPump
    cooling_mod.__all__ = ['temp_sensor', 'WaterPump']

    coolsvc_mod = types.ModuleType('gfhardware.coolsvc')
    coolsvc_mod.cooling_svc = FakeCooling()
    # The pure limit derivation and its tag sets come from the real module.
    _spec = importlib.util.spec_from_file_location(
        '_real_coolsvc', os.path.join(ROOT, 'gfhardware', 'coolsvc.py'))
    _real = importlib.util.module_from_spec(_spec)
    _spec.loader.exec_module(_real)
    coolsvc_mod.limits_from_header = _real.limits_from_header
    coolsvc_mod.LIMIT_TAGS = _real.LIMIT_TAGS
    coolsvc_mod.INERT_LIMIT_TAGS = _real.INERT_LIMIT_TAGS

    z_mod = types.ModuleType('gfhardware.z_axis')

    class ZAxis:
        homed = 0

        @staticmethod
        def home(): ZAxis.homed += 1

        @staticmethod
        def step(d): pass

        @staticmethod
        def set_mode_from_puls(v): pass

        @staticmethod
        def reset(): pass
    z_mod.ZAxis = ZAxis

    cam_mod = types.ModuleType('gfhardware.cam')
    cam_mod.GFCAM_HEAD = 0
    cam_mod.GFCAM_LID = 1
    cam_mod.captures = []

    def _capture(cam_sel=1, **kw):
        cam_mod.captures.append((cam_sel, kw))
        return b''
    cam_mod.capture = _capture

    id_mod = types.ModuleType('gfhardware.id')
    id_mod.serial = lambda: 'XXX-000'
    id_mod.hostname = lambda: 'host'
    id_mod.password = lambda: 'pw'

    for name, mod in (('gfhardware.cnc', cnc_mod), ('gfhardware.switches', sw_mod),
                      ('gfhardware.leds', leds_mod), ('gfhardware.cooling', cooling_mod),
                      ('gfhardware.coolsvc', coolsvc_mod), ('gfhardware.z_axis', z_mod),
                      ('gfhardware.cam', cam_mod), ('gfhardware.id', id_mod)):
        sys.modules[name] = mod
    return cnc_mod.cnc, fake_sw, coolsvc_mod.cooling_svc, leds_mod, cam_mod


CNC, SW, COOL, LEDS, CAM = _install_fakes()

import gfhardware.machine as machine_mod          # noqa: E402
from gfhardware.machine import Machine             # noqa: E402
from gfutilities.configuration import set_cfg     # noqa: E402

set_cfg('THERMAL.MAX_START_TEMP', 50)
# Point the reads the constructor makes at fakes.
machine_mod.read_file = lambda attr, binary=False: 'hw_id=0x4c\nserial=1\nversion=0x2\nr5=0\nr6=0'
EVENTS = []
machine_mod.send_wss_event = lambda q, action_id, event: EVENTS.append(event)
machine_mod.generate_linear_puls = lambda x, y, dev: None
UPLOADS = []
machine_mod.img_upload = lambda session, img, msg: UPLOADS.append(msg['id'])


def job_events():
    """The action's own wire events (the unsolicited button:*/lid:* edge
    reports are expected alongside and not part of these checks)."""
    return [e for e in EVENTS if e.startswith('print:')]


def make_machine():
    m = Machine()
    m.running_action_id = 42
    m._running_action_cancelled = False
    return m


class RunLoopTests(unittest.TestCase):

    def setUp(self):
        CNC.__init__()
        SW.__init__()
        COOL.__init__()
        del EVENTS[:]
        del LEDS.button_colors[:]
        self.m = make_machine()
        SW.handler = self.m._switch_event

    # -- lid / interlock mid-run --------------------------------------

    def test_lid_open_mid_run_stops_and_cancels(self):
        SW.open_lid(delay=0.05)
        t0 = time.monotonic()
        aborted = self.m._run_loop()
        dt = time.monotonic() - t0
        self.assertTrue(aborted)
        self.assertTrue(self.m._running_action_cancelled)
        self.assertIn(('stop', 1), CNC.writes)
        # Edge-driven: the stop lands well inside one 100 ms poll tick.
        self.assertLess(dt, 0.15, 'stop was not edge-driven (%.3f s)' % dt)

    def test_interlock_open_mid_run_stops_and_cancels(self):
        SW.open_interlock(delay=0.05)
        aborted = self.m._run_loop()
        self.assertTrue(aborted)
        self.assertTrue(self.m._running_action_cancelled)
        self.assertIn(('stop', 1), CNC.writes)

    def test_lid_open_before_run_stops_on_first_pass(self):
        SW.word[InputSwitch.SW_DOORS] = False
        aborted = self.m._run_loop()
        self.assertTrue(aborted)
        self.assertIn(('stop', 1), CNC.writes)

    def test_transient_lid_open_still_cancels(self):
        # Open and closed again before the loop's level read: the edge
        # is what counts (the hardware button latch set on it).
        SW.open_lid(delay=0.02)
        SW.close_lid(delay=0.03)
        aborted = self.m._run_loop()
        self.assertTrue(aborted)
        self.assertTrue(self.m._running_action_cancelled)

    # -- the park and the hunt ignore the lid ---------------------------

    def test_park_ignores_lid(self):
        CNC.run_reads = 5
        SW.open_lid(delay=0.02)
        aborted = self.m._run_loop(park=True)
        self.assertFalse(aborted)
        self.assertNotIn(('stop', 1), CNC.writes)

    def test_park_ignores_service_cancel(self):
        CNC.run_reads = 5
        self.m._running_action_cancelled = True
        aborted = self.m._run_loop(park=True)
        self.assertFalse(aborted)
        self.assertNotIn(('stop', 1), CNC.writes)

    def test_hunt_ignores_lid(self):
        CNC.run_reads = 5
        SW.word[InputSwitch.SW_DOORS] = False
        aborted = self.m._run_loop(lid_gated=False)
        self.assertFalse(aborted)
        self.assertNotIn(('stop', 1), CNC.writes)

    def test_service_cancel_mid_run_stops(self):
        threading.Timer(0.05, lambda: setattr(self.m, '_running_action_cancelled', True)).start()
        aborted = self.m._run_loop()
        self.assertTrue(aborted)
        self.assertIn(('stop', 1), CNC.writes)

    # -- a live feed that falls behind ---------------------------------

    def test_underrun_mid_run_stops_and_cancels(self):
        # A job longer than the ring is fed while it plays. If the ring ever
        # goes dry the kernel stops dead, so the job did not finish and the
        # position is not to be trusted: it has to end ':cancelled', never
        # ':completed'.
        CNC.underrun_after = 3
        aborted = self.m._run_loop()
        self.assertTrue(aborted)
        self.assertTrue(self.m._running_action_cancelled)
        # The stop is also the acknowledgement the kernel requires before it
        # will accept another run.
        self.assertIn(('stop', 1), CNC.writes)

    def test_underrun_while_paused_is_not_missed(self):
        CNC.underrun_after = 3
        aborted = self.m._run_loop(pausable=True)
        self.assertTrue(aborted)
        self.assertTrue(self.m._running_action_cancelled)

    def test_feed_failure_mid_run_stops_the_job(self):
        # The feeder cannot get the rest of the job into the ring: what is
        # already in there would keep playing, so the run has to be stopped.
        class DeadFeeder:
            error = OSError(5, 'I/O error')
            finished = False
            written = 0

            def stop(self, timeout=5.0):
                pass

        self.m._feeder = DeadFeeder()
        try:
            aborted = self.m._run_loop()
        finally:
            self.m._feeder = None
        self.assertTrue(aborted)
        self.assertTrue(self.m._running_action_cancelled)
        self.assertIn(('stop', 1), CNC.writes)

    def test_cooling_verdict_pulled_relocks_and_holds(self):
        # fire_ok false, hold true (OVERTEMP, COLD, WARMUP, FLAME, BUMP):
        # the latch at once, then the same pause as the button's.
        threading.Timer(0.05, lambda: setattr(COOL, '_fire_ok', False)).start()
        threading.Timer(0.6, lambda: setattr(self.m, '_running_action_cancelled', True)).start()
        COOL.armed = True
        aborted = self.m._run_loop(pausable=True)
        self.assertTrue(aborted)
        w = CNC.writes
        self.assertIn(('laser_latch', 1), w)
        self.assertIn(('stop', 1), w)
        self.assertIn(('resume', -2000), w)
        self.assertLess(w.index(('laser_latch', 1)), w.index(('stop', 1)))
        self.assertNotIn(('laser_latch', 0), w)
        self.assertIn('print:paused', job_events())

    # -- button pause / resume (prints) --------------------------------

    def test_button_pauses_and_resumes_a_print(self):
        SW.press(delay=0.05)                       # pause
        SW.press(delay=0.40)                       # resume
        threading.Timer(0.6, lambda: setattr(CNC, '_reads_left', 1)).start()   # end the program
        aborted = self.m._run_loop(pausable=True)
        self.assertFalse(aborted)
        w = CNC.writes
        self.assertIn(('stop', 1), w)
        self.assertIn(('resume', -2000), w)
        self.assertIn(('resume', 1950), w)
        self.assertLess(w.index(('stop', 1)), w.index(('resume', -2000)))
        self.assertLess(w.index(('resume', -2000)), w.index(('resume', 1950)))
        self.assertEqual(job_events(), ['print:paused', 'print:resumed'])
        self.assertFalse(self.m._running_action_cancelled)
        # The latch was never touched by the pause.
        self.assertNotIn(('laser_latch', 1), w)

    def test_button_does_not_pause_a_motion(self):
        CNC.run_reads = 5
        SW.press(delay=0.02)
        aborted = self.m._run_loop(pausable=False)
        self.assertFalse(aborted)
        self.assertNotIn(('stop', 1), CNC.writes)
        self.assertEqual(job_events(), [])

    def test_lid_open_while_paused_cancels_without_second_stop(self):
        SW.press(delay=0.05)                       # pause
        SW.open_lid(delay=0.40)
        aborted = self.m._run_loop(pausable=True)
        self.assertTrue(aborted)
        self.assertTrue(self.m._running_action_cancelled)
        self.assertEqual(CNC.writes.count(('stop', 1)), 1)   # only the pause's stop
        self.assertEqual(job_events(), ['print:paused'])

    def test_service_cancel_while_paused(self):
        SW.press(delay=0.05)
        threading.Timer(0.4, lambda: setattr(self.m, '_running_action_cancelled', True)).start()
        aborted = self.m._run_loop(pausable=True)
        self.assertTrue(aborted)
        self.assertEqual(job_events(), ['print:paused'])

    def test_pause_at_program_end_finishes(self):
        # The decel lands on the program's last byte: no backtrack, done.
        SW.press(delay=0.05)

        def end_it():
            CNC.processed = CNC.total
        threading.Timer(0.049, end_it).start()
        aborted = self.m._run_loop(pausable=True)
        self.assertFalse(aborted)
        self.assertNotIn(('resume', -2000), CNC.writes)
        self.assertEqual(job_events(), [])

    def test_backtrack_refused_pauses_in_place(self):
        # The kernel refuses the backtrack: the job holds where the decel
        # stopped, still paused, still resumable - and the resume asks for
        # the shortest lead there is, because nothing was retraced. A lead
        # over ground the job has not cut yet would leave that length
        # unburned, and a lead of zero is the kernel's "no laser at all".
        SW.press(delay=0.05)
        SW.press(delay=0.40)
        threading.Timer(0.6, lambda: setattr(CNC, '_reads_left', 1)).start()
        orig = CNC.resume

        def resume(steps):
            if steps < 0:
                raise OSError(1, 'EPERM')
            orig(steps)
        CNC.resume = resume
        try:
            aborted = self.m._run_loop(pausable=True)
        finally:
            del CNC.resume
        self.assertFalse(aborted)
        self.assertEqual(job_events(), ['print:paused', 'print:resumed'])
        self.assertIn(('resume', 1), CNC.writes)
        self.assertNotIn(('resume', 1950), CNC.writes)
        self.assertNotIn(('resume', 0), CNC.writes)

    def test_short_history_shortens_the_retrace_and_the_lead(self):
        # A pause early in a program, or on a ring a live feed has just
        # topped up, has less than the factory distance to walk back over.
        # The retrace shortens to what is there and the lead follows it,
        # keeping the 50-tick overlap the factory constants describe.
        CNC.backtrack_budget = 500
        SW.press(delay=0.05)
        SW.press(delay=0.40)
        threading.Timer(0.6, lambda: setattr(CNC, '_reads_left', 1)).start()
        aborted = self.m._run_loop(pausable=True)
        self.assertFalse(aborted)
        self.assertIn(('resume', -500), CNC.writes)
        self.assertIn(('resume', 450), CNC.writes)
        self.assertEqual(job_events(), ['print:paused', 'print:resumed'])

    def test_no_history_holds_and_resumes_lit(self):
        # Nothing to walk back over: no backward run is asked for at all,
        # and the resume puts the beam back on where it stopped.
        CNC.backtrack_budget = 0
        SW.press(delay=0.05)
        SW.press(delay=0.40)
        threading.Timer(0.6, lambda: setattr(CNC, '_reads_left', 1)).start()
        aborted = self.m._run_loop(pausable=True)
        self.assertFalse(aborted)
        self.assertFalse([w for w in CNC.writes if w[0] == 'resume' and w[1] < 0])
        self.assertIn(('resume', 1), CNC.writes)
        self.assertNotIn(('resume', 0), CNC.writes)
        self.assertEqual(job_events(), ['print:paused', 'print:resumed'])

    def test_unreadable_budget_asks_for_the_configured_distance(self):
        # No readback (an older module): ask for the factory distance and
        # let the kernel be the one to refuse it.
        CNC.max_backtrack_error = OSError(2, 'ENOENT')
        SW.press(delay=0.05)
        SW.press(delay=0.40)
        threading.Timer(0.6, lambda: setattr(CNC, '_reads_left', 1)).start()
        aborted = self.m._run_loop(pausable=True)
        self.assertFalse(aborted)
        self.assertIn(('resume', -2000), CNC.writes)
        self.assertIn(('resume', 1950), CNC.writes)


class ButtonWaitTests(unittest.TestCase):

    def setUp(self):
        CNC.__init__()
        SW.__init__()
        COOL.__init__()
        del EVENTS[:]
        self.m = make_machine()
        SW.handler = self.m._switch_event
        COOL.armed = True

    def test_press_ends_the_wait(self):
        SW.press(delay=0.05)
        self.m._button_wait({'id': 42})
        self.assertFalse(self.m._running_action_cancelled)
        self.assertNotIn(('laser_latch', 1), CNC.writes)

    def test_lid_open_cancels_the_wait(self):
        SW.open_lid(delay=0.05)
        self.m._button_wait({'id': 42})
        self.assertTrue(self.m._running_action_cancelled)
        self.assertIn(('laser_latch', 1), CNC.writes)
        self.assertFalse(COOL.armed)

    def test_interlock_open_cancels_the_wait(self):
        SW.open_interlock(delay=0.05)
        self.m._button_wait({'id': 42})
        self.assertTrue(self.m._running_action_cancelled)
        self.assertIn(('laser_latch', 1), CNC.writes)

    def test_press_with_lid_open_does_not_arm(self):
        SW.word[InputSwitch.SW_DOORS] = False
        SW.press(delay=0.02)
        self.m._button_wait({'id': 42})
        self.assertTrue(self.m._running_action_cancelled)
        self.assertIn(('laser_latch', 1), CNC.writes)


class StartGateTests(unittest.TestCase):

    def setUp(self):
        CNC.__init__()
        SW.__init__()
        COOL.__init__()
        del EVENTS[:]
        self.m = make_machine()
        SW.handler = self.m._switch_event

    def test_lid_open_at_start_cancels_a_gated_job(self):
        SW.word[InputSwitch.SW_DOORS] = False
        self.m._motion({'id': 42, 'action_type': 'motion', 'motion_url': 'x'})
        self.assertTrue(self.m._running_action_cancelled)
        self.assertNotIn(('run', 1), CNC.writes)

    def test_interlock_open_at_start_cancels_a_gated_job(self):
        SW.word[InputSwitch.SW_INTERLOCK] = True
        self.m._motion({'id': 42, 'action_type': 'print', 'motion_url': 'x'})
        self.assertTrue(self.m._running_action_cancelled)

    def test_hunt_start_gate_ignores_the_lid(self):
        SW.word[InputSwitch.SW_DOORS] = False
        self.assertTrue(self.m._safe_to_move(lid_gated=False))
        self.assertFalse(self.m._safe_to_move(lid_gated=True))


if __name__ == '__main__':
    unittest.main()


class ReturnHomeTests(unittest.TestCase):

    def setUp(self):
        CNC.__init__()
        SW.__init__()
        COOL.__init__()
        del EVENTS[:]
        self.m = make_machine()
        SW.handler = self.m._switch_event
        self.parks = []
        machine_mod.generate_linear_puls = lambda x, y, dev: self.parks.append((x, y))

    def tearDown(self):
        machine_mod.generate_linear_puls = lambda x, y, dev: None

    def test_park_clears_the_ring_before_it_runs(self):
        # The job did not play out (a print aborted mid-run, or one cancelled
        # at the button wait): what it left in the ring must not play ahead
        # of the park.
        CNC.xy_steps = (500, 300)
        CNC.run_reads = 3
        self.m._return_home(None)
        w = CNC.writes
        self.assertIn(('clear_pulse', 1), w)
        self.assertIn(('run', 1), w)
        self.assertLess(w.index(('clear_pulse', 1)), w.index(('run', 1)))
        self.assertEqual(self.parks, [(-500, -300)])
        self.assertEqual(job_events(), ['print:return_to_home:succeeded'])

    def test_park_at_the_start_runs_nothing(self):
        # Cancelled before it moved (the button wait): the ring holds the
        # whole print; nothing may run, the head is already at the start.
        CNC.xy_steps = (0, 0)
        self.m._return_home(None)
        self.assertIn(('clear_pulse', 1), CNC.writes)
        self.assertNotIn(('run', 1), CNC.writes)
        self.assertEqual(self.parks, [])
        self.assertEqual(job_events(), ['print:return_to_home:succeeded'])

    def test_refused_clear_parks_nothing(self):
        # The kernel refuses the clear (still running): never park on top of
        # an uncleared ring, and never claim success.
        CNC.xy_steps = (500, 300)
        CNC.clear_error = OSError(1, 'EPERM')
        self.m._return_home(None)
        self.assertNotIn(('run', 1), CNC.writes)
        self.assertEqual(self.parks, [])
        self.assertEqual(job_events(), [])


class FeedWatchdogTests(unittest.TestCase):
    """A live feed that wedges must not be left to play the ring dry: the
    job is stopped and retraced while there is still history to retrace
    over, and picked back up if the feed catches up."""

    def setUp(self):
        CNC.__init__()
        SW.__init__()
        COOL.__init__()
        del EVENTS[:]
        del LEDS.button_colors[:]
        self.m = make_machine()
        SW.handler = self.m._switch_event
        self._stall, self._recover = machine_mod.FEED_STALL_S, machine_mod.FEED_RECOVER_S
        machine_mod.FEED_STALL_S = 0.2

    def tearDown(self):
        machine_mod.FEED_STALL_S = self._stall
        machine_mod.FEED_RECOVER_S = self._recover
        self.m._feeder = None

    def test_stalled_feed_holds_the_job_and_resumes_when_it_moves(self):
        machine_mod.FEED_RECOVER_S = 5.0
        feeder = FakeFeeder(written=1000)
        self.m._feeder = feeder
        SW._later(0.6, lambda: setattr(feeder, 'moving', True))
        SW._later(1.2, lambda: setattr(CNC, '_reads_left', 1))

        aborted = self.m._run_loop(pausable=True)

        self.assertFalse(aborted)
        self.assertFalse(self.m._running_action_cancelled)
        w = CNC.writes
        self.assertIn(('stop', 1), w)               # stopped before the ring ran dry
        self.assertIn(('resume', -2000), w)         # retraced like a pause
        self.assertIn(('resume', 1950), w)          # and led back on over cut ground
        self.assertEqual(job_events(), ['print:paused', 'print:resumed'])

    def test_feed_that_never_moves_cancels_the_job(self):
        machine_mod.FEED_RECOVER_S = 0.5
        self.m._feeder = FakeFeeder(written=1000)

        aborted = self.m._run_loop(pausable=True)

        self.assertTrue(aborted)
        self.assertTrue(self.m._running_action_cancelled)
        self.assertIn(('stop', 1), CNC.writes)
        # Held, then given up on: the job never claims to have finished.
        self.assertEqual(job_events(), ['print:paused'])

    def test_a_full_ring_is_not_a_stalled_feed(self):
        # The feeder has written nothing for the whole run because there is
        # no room to write into. That is a healthy feed with a full window,
        # and stopping the job for it would be the watchdog inventing a
        # fault.
        CNC.free = 0
        CNC.run_reads = 12                          # the program ends on its own
        self.m._feeder = FakeFeeder(written=1000)

        aborted = self.m._run_loop(pausable=True)

        self.assertFalse(aborted)
        self.assertNotIn(('stop', 1), CNC.writes)
        self.assertEqual(job_events(), [])

    def test_a_job_that_fits_the_ring_is_never_watched(self):
        # Nothing left to feed: end-of-data is the end of the job.
        CNC.run_reads = 12
        self.m._feeder = FakeFeeder(written=1000, finished=True)

        aborted = self.m._run_loop(pausable=True)

        self.assertFalse(aborted)
        self.assertNotIn(('stop', 1), CNC.writes)

    def test_a_press_while_held_for_the_feed_is_not_lost(self):
        # Pausing a job that is already stopped is not a thing the machine
        # can do, so the press waits for the resume and pauses then.
        machine_mod.FEED_RECOVER_S = 5.0
        feeder = FakeFeeder(written=1000)
        self.m._feeder = feeder
        SW.press(delay=0.6)                         # while the job is held
        SW._later(0.9, lambda: setattr(feeder, 'moving', True))
        SW.press(delay=1.8)                         # resume from that pause
        SW._later(2.4, lambda: setattr(CNC, '_reads_left', 1))

        aborted = self.m._run_loop(pausable=True)

        self.assertFalse(aborted)
        self.assertEqual(job_events(),
                         ['print:paused', 'print:resumed',    # the watchdog's hold
                          'print:paused', 'print:resumed'])   # the operator's press


class JobLifecycleTests(unittest.TestCase):
    """A print warms up before its first fire and rests after its last, and
    a header key nothing acts on is recorded rather than ignored."""

    def setUp(self):
        CNC.__init__()
        SW.__init__()
        COOL.__init__()
        del EVENTS[:]
        self.m = make_machine()
        self.slept = []
        self._sleep = machine_mod.sleep
        machine_mod.sleep = lambda s: self.slept.append(s)

    def tearDown(self):
        machine_mod.sleep = self._sleep
        set_cfg('MOTION.WARM_UP_DELAY', None)
        set_cfg('MOTION.COOL_DOWN_DELAY', None)

    def test_the_factory_periods_are_the_defaults(self):
        # Nothing configured: the machine still does what the factory does,
        #.
        self.assertEqual(self.m._dwell('warm_up'), machine_mod.WARM_UP_DEFAULT_S)
        self.assertEqual(self.m._dwell('cool_down'), machine_mod.COOL_DOWN_DEFAULT_S)
        self.assertEqual(self.slept, [machine_mod.WARM_UP_DEFAULT_S,
                                      machine_mod.COOL_DOWN_DEFAULT_S])

    def test_a_configured_period_wins(self):
        set_cfg('MOTION.WARM_UP_DELAY', '5')
        self.assertEqual(self.m._dwell('warm_up'), 5.0)
        self.assertEqual(self.slept, [5.0])

    def test_zero_still_skips_the_period(self):
        # A machine whose config carries the zeros the old sample shipped
        # skips the hold, and the operator keeps that choice.
        set_cfg('MOTION.WARM_UP_DELAY', 0)
        set_cfg('MOTION.COOL_DOWN_DELAY', 0)
        self.assertEqual(self.m._dwell('warm_up'), 0.0)
        self.assertEqual(self.m._dwell('cool_down'), 0.0)
        self.assertEqual(self.slept, [])

    def test_a_nonsense_period_falls_back_to_the_factory_one(self):
        set_cfg('MOTION.WARM_UP_DELAY', 'soon')
        self.assertEqual(self.m._dwell('warm_up'), machine_mod.WARM_UP_DEFAULT_S)

    def test_an_unhandled_header_key_is_named_once(self):
        header = {'STfr': 10000, 'AArd': 200, 'MCsn': 0, 'PDfm': 0,
                  'ZZzz': 4242}
        with self.assertLogs(machine_mod.logger, level='DEBUG') as caught:
            gaps = self.m._log_header_gaps(header)
        self.assertEqual(gaps, ['ZZzz'])
        named = [ln for ln in caught.output if 'ZZzz=4242' in ln]
        self.assertEqual(len(named), 1, caught.output)

    def test_applied_and_checked_keys_are_not_reported_as_gaps(self):
        # The motion keys have appliers, the serial and format are checked
        # before the ring is touched, and the lifecycle keys get a line of
        # their own: none of them is an unrecorded decision.
        header = {'STfr': 10000, 'AArd': 200, 'EFrd': 300, 'IFrd': 100,
                  'MCsn': 0, 'PDfm': 0, 'CFrh': 1, 'CCwp': 5000,
                  'CCrp': 10000, 'CCup': 1}
        self.assertEqual(self.m._log_header_gaps(header), [])

    def test_declared_ignored_keys_are_counted_and_not_undecided(self):
        # The warm-up fan profile, a board ceiling, an IR threshold and the
        # HV cap are decisions with a recorded reason; ZZzz is not.
        header = {'STfr': 10000, 'AAwd': 1023, 'EFwd': 65535, 'BTrx': 9440,
                  'IRwx': 275, 'HIrx': 1023, 'ZZzz': 1}
        with self.assertLogs(machine_mod.logger, level='DEBUG') as caught:
            gaps = self.m._log_header_gaps(header)
        self.assertEqual(gaps, ['ZZzz'])
        line = [ln for ln in caught.output if 'have no applier here' in ln]
        self.assertEqual(len(line), 1, caught.output)
        self.assertIn('6 of 7 header keys have no applier here (5 declared ignored, 1 undecided)', line[0])
        declared = [ln for ln in caught.output if 'declared ignored: ' in ln]
        self.assertEqual(len(declared), 1, caught.output)
        for key in ('AAwd=1023', 'EFwd=65535', 'BTrx=9440', 'IRwx=275', 'HIrx=1023'):
            self.assertIn(key, declared[0])
        self.assertNotIn('ZZzz', declared[0])

    def test_every_declared_key_has_a_reason_and_no_applier(self):
        applied = {k for k, s in machine_mod.MACHINE_SETTINGS.items()
                   if s.idle or s.run or s.cool_down}
        for key, why in machine_mod.DECLARED_IGNORED.items():
            self.assertEqual(len(key), 4, key)
            self.assertTrue(why, key)
            self.assertNotIn(key, applied, key)
            self.assertNotIn(key, machine_mod.LIMIT_TAGS, key)
            self.assertNotIn(key, machine_mod.INERT_LIMIT_TAGS, key)

    def test_a_header_with_only_declared_keys_has_no_undecided_line(self):
        header = {'STfr': 10000, 'TRuc': 0, 'PTmn': 0, 'PTmx': 1023}
        with self.assertLogs(machine_mod.logger, level='DEBUG') as caught:
            gaps = self.m._log_header_gaps(header)
        self.assertEqual(gaps, [])
        self.assertFalse([ln for ln in caught.output if 'keys with no applier: ' in ln], caught.output)
        self.assertTrue([ln for ln in caught.output if '(3 declared ignored, 0 undecided)' in ln], caught.output)

    def test_the_lifecycle_keys_are_logged_even_when_absent(self):
        with self.assertLogs(machine_mod.logger, level='INFO') as caught:
            self.m._log_header_gaps({'STfr': 10000})
        line = [ln for ln in caught.output if 'job lifecycle keys' in ln]
        self.assertEqual(len(line), 1, caught.output)
        for key in machine_mod.LIFECYCLE_KEYS:
            self.assertIn('%s=-' % key, line[0])


class LidLampTests(unittest.TestCase):
    """A lid capture lights the bed unless the action asks it not to."""

    def setUp(self):
        CNC.__init__()
        SW.__init__()
        COOL.__init__()
        del CAM.captures[:]
        del UPLOADS[:]
        self.m = make_machine()

    def lamp(self):
        self.assertEqual(len(CAM.captures), 1, CAM.captures)
        cam_sel, kw = CAM.captures[0]
        self.assertEqual(cam_sel, machine_mod.cam.GFCAM_LID)
        return kw.get('illumination')

    def test_a_capture_with_no_settings_lights_the_lamp(self):
        self.m._lid_image({'id': 1})
        self.assertEqual(self.lamp(), 132)

    def test_the_service_can_ask_for_the_flash(self):
        self.m._lid_image({'id': 2}, {'LCfl': 1})
        self.assertEqual(self.lamp(), 132)

    def test_the_service_can_ask_for_the_bed_as_it_is_lit(self):
        self.m._lid_image({'id': 3}, {'LCfl': 0})
        self.assertEqual(self.lamp(), 0)

    def test_settings_that_say_nothing_about_the_flash_leave_it_on(self):
        self.m._lid_image({'id': 4}, {'HCil': 3})
        self.assertEqual(self.lamp(), 132)

    def test_the_image_still_goes_out(self):
        self.m._lid_image({'id': 5}, {'LCfl': 0})
        self.assertEqual(UPLOADS, [5])


class ProgressTests(unittest.TestCase):
    """What the app's progress bar is told while a print runs.

    The frames go into a real queue and come back off it as the wire sees
    them, so the shape is checked rather than assumed. The one thing that
    must hold whatever the machine does: the bar divides by the job, and
    the job's length does not move even when the kernel's byte counter
    does.
    """

    def setUp(self):
        CNC.__init__()
        SW.__init__()
        COOL.__init__()
        del EVENTS[:]
        del LEDS.button_colors[:]
        self.m = make_machine()
        SW.handler = self.m._switch_event
        self.q = queue.Queue()

    def frames(self):
        out = []
        while not self.q.empty():
            out.append(json.loads(self.q.get_nowait()))
        return out

    def progress(self, total, interval=0.02, action_id=42):
        return machine_mod._JobProgress(self.q, action_id, 'print:progress',
                                        total, interval=interval)

    # -- the frame itself ----------------------------------------------

    def test_the_frame_is_the_one_the_factory_sends(self):
        CNC.run()                                  # a job under way
        CNC.processed, CNC.total = 994, 33291208
        CNC.xy_steps = (7, 9)
        self.progress(33291208).send(force=True)
        frame = self.frames()[0]
        self.assertEqual(frame['type'], 'progress')
        self.assertEqual(frame['version'], 1)
        self.assertEqual(frame['action_id'], 42)
        self.assertEqual(frame['progress'], 'print:progress')
        self.assertEqual(frame['current'], 994)
        self.assertEqual(frame['units'], 'steps')
        self.assertEqual(frame['total'], 33291208)
        # The progress frame is also the periodic settings report: these
        # are the job's own values, and the sensor tags stay out of it.
        self.assertEqual(frame['settings']['values'],
                         {'CAid': 42, 'CCbp': 994, 'CCst': 1, 'CCxp': 7,
                          'CCyp': 9})

    def test_a_job_of_unknown_length_reports_position_without_a_total(self):
        # Better an honest position than a bar divided by a guess.
        self.progress(None).send(force=True)
        frame = self.frames()[0]
        self.assertNotIn('total', frame)
        self.assertIn('current', frame)

    def test_reporting_can_be_turned_off(self):
        self.progress(1000, interval=0).send(force=True)
        self.assertEqual(self.frames(), [])

    # -- what the bar divides by ----------------------------------------

    def test_the_bar_divides_by_the_job_not_the_kernel_counter(self):
        # A job longer than the ring is fed while it plays, so the kernel's
        # byte total climbs all job long. Dividing by it would leave the bar
        # near complete from the first frame to the last, which is the trap
        # the factory's own progress falls into. The job's length is what
        # the report divides by, and it does not move.
        job = 4000
        CNC.processed, CNC.total = 0, 1000
        CNC.run_reads = 10 ** 9

        def feed():
            for _ in range(20):
                CNC.total += 1000              # the ring is topped up
                CNC.processed += 200           # and played out
                time.sleep(0.01)
            CNC._reads_left = 1                # the program ends

        threading.Timer(0.05, feed).start()
        self.m._run_loop(pausable=True, progress=self.progress(job))

        frames = self.frames()
        self.assertGreater(len(frames), 2, 'no progress was reported')
        self.assertTrue(all(f['total'] == job for f in frames),
                        [f['total'] for f in frames])
        currents = [f['current'] for f in frames]
        self.assertEqual(currents, sorted(currents), currents)
        self.assertTrue(all(c <= job for c in currents), currents)
        # The last frame is the end of the job, not 12% of a moving total.
        self.assertEqual(currents[-1], job)

    def test_a_job_that_overruns_its_declared_length_still_ends_at_100(self):
        CNC.processed, CNC.total = 5000, 5000
        CNC.run_reads = 3
        with self.assertLogs(machine_mod.logger, level='WARNING') as caught:
            prog = self.progress(4000)
            prog.send(force=True)
            prog.send(force=True)
        frames = self.frames()
        self.assertEqual([f['current'] for f in frames], [4000, 4000])
        # Said once, not once per frame.
        over = [ln for ln in caught.output if 'declared' in ln]
        self.assertEqual(len(over), 1, caught.output)
        # The raw byte position is still reported as itself.
        self.assertEqual(frames[0]['settings']['values']['CCbp'], 5000)

    # -- when it reports -------------------------------------------------

    def test_a_run_reports_at_its_start_and_at_its_end(self):
        CNC.processed, CNC.total = 0, 1000
        CNC.run_reads = 3
        self.m._run_loop(progress=self.progress(1000, interval=3600))
        frames = self.frames()
        # A long interval: only the phase changes reported.
        self.assertEqual(len(frames), 2, frames)
        self.assertEqual(frames[0]['current'], 0)
        self.assertEqual(frames[-1]['current'], 1000)

    def test_a_pause_and_a_resume_each_report(self):
        CNC.processed, CNC.total = 500, 1000
        SW.press(delay=0.05)                       # pause
        SW.press(delay=0.40)                       # resume
        threading.Timer(0.6, lambda: setattr(CNC, '_reads_left', 1)).start()
        self.m._run_loop(pausable=True, progress=self.progress(1000, interval=3600))
        self.assertEqual(job_events(), ['print:paused', 'print:resumed'])
        # Start, pause, resume, end: four frames, and no more than the
        # phases asked for.
        self.assertEqual(len(self.frames()), 4)

    def test_a_job_held_for_a_stalled_feed_reports_the_hold(self):
        stall, recover = machine_mod.FEED_STALL_S, machine_mod.FEED_RECOVER_S
        machine_mod.FEED_STALL_S, machine_mod.FEED_RECOVER_S = 0.2, 5.0
        try:
            feeder = FakeFeeder(written=1000)
            self.m._feeder = feeder
            SW._later(0.6, lambda: setattr(feeder, 'moving', True))
            SW._later(1.2, lambda: setattr(CNC, '_reads_left', 1))
            self.m._run_loop(pausable=True,
                             progress=self.progress(1000, interval=3600))
        finally:
            machine_mod.FEED_STALL_S, machine_mod.FEED_RECOVER_S = stall, recover
            self.m._feeder = None
        self.assertEqual(job_events(), ['print:paused', 'print:resumed'])
        self.assertEqual(len(self.frames()), 4)

    def test_a_run_with_nothing_to_report_to_says_nothing(self):
        # A motion or a hunt is over before a first frame would land, and
        # the factory reports neither.
        CNC.run_reads = 3
        self.m._run_loop()
        self.assertEqual(self.frames(), [])


class FeederExitTests(unittest.TestCase):
    """A job longer than the ring keeps its feeder alive, parked on a full
    ring with the rest of the print in hand. Every way out of the job has
    to stop that feeder before the park clears the ring: a feeder left
    alive would refill the ring behind the clear, and the park (which
    ignores the lid and the cancel flag by design) would play the print's
    path. The whole job body runs here, against a feeder stand-in that
    records its stop among the kernel writes so the order can be checked."""

    class Source:
        body_size = 0
        program_size = 5000

        def read(self, n):
            return b''

    class JobFeeder(FakeFeeder):
        """Records its stop among the cnc writes, as the order against the
        park's clear is the point."""

        def __init__(self, finished):
            FakeFeeder.__init__(self, written=100, finished=finished)
            self.stats = {}
            self.job_total = 5000
            self.live = False

        def start(self):
            pass

        def wait_primed(self, timeout=120.0):
            return True

        def declare_live_feed(self):
            self.live = True

        def settle(self, timeout=60.0):
            return True

        def stop(self, timeout=5.0):
            FakeFeeder.stop(self, timeout)
            CNC.writes.append(('feeder_stop', 1))

    def setUp(self):
        CNC.__init__()
        SW.__init__()
        COOL.__init__()
        del EVENTS[:]
        self.m = make_machine()
        SW.handler = self.m._switch_event
        self.feeder = None
        self.long_job = True
        self._fetch = machine_mod.fetch_motion
        self._feeder_cls = machine_mod.PulseFeeder
        self._sleep = machine_mod.sleep
        self._conf = machine_mod._conf_float
        machine_mod.fetch_motion = lambda s, url, warn_bytes=0, reject_bytes=0: (
            {'header_data': {'STfr': 10000}}, self.Source())
        machine_mod.PulseFeeder = self._make_feeder
        machine_mod.sleep = lambda s: None
        self.parks = []
        machine_mod.generate_linear_puls = lambda x, y, dev: self.parks.append((x, y))

    def tearDown(self):
        machine_mod.fetch_motion = self._fetch
        machine_mod.PulseFeeder = self._feeder_cls
        machine_mod.sleep = self._sleep
        machine_mod._conf_float = self._conf
        machine_mod.generate_linear_puls = lambda x, y, dev: None

    def _make_feeder(self, source, dev):
        self.feeder = self.JobFeeder(finished=not self.long_job)
        return self.feeder

    @staticmethod
    def _print():
        return {'id': 42, 'action_type': 'print', 'motion_url': 'x'}

    def _assert_feeder_stopped_before_the_park(self):
        w = CNC.writes
        self.assertTrue(self.feeder.stopped, 'the feeder was left alive')
        self.assertIsNone(self.m._feeder)
        self.assertIn(('feeder_stop', 1), w)
        if ('clear_pulse', 1) in w:
            self.assertLess(w.index(('feeder_stop', 1)), w.index(('clear_pulse', 1)),
                            'the park cleared the ring with the feeder alive')

    def test_a_cancel_at_the_button_wait_stops_the_feeder_before_the_park(self):
        # A two-hour print, ring full, the operator opens the lid while the
        # button is waited for. The park must find no feeder.
        CNC.xy_steps = (0, 0)
        SW.open_lid(delay=0.05)
        self.m._motion_locked(self._print(), None)
        self.assertTrue(self.m._running_action_cancelled)
        self._assert_feeder_stopped_before_the_park()
        self.assertNotIn(('run', 1), CNC.writes)
        self.assertNotIn('print:running', job_events())
        self.assertEqual(self.parks, [])

    def test_a_cancel_at_the_button_wait_off_the_start_parks_after_the_feeder(self):
        # The same cancel with the head off the start (a hunt moved it): the
        # park runs, and only after the feeder is gone and the ring cleared.
        CNC.xy_steps = (500, 300)
        CNC.run_reads = 3
        SW.open_lid(delay=0.05)
        self.m._motion_locked(self._print(), None)
        self._assert_feeder_stopped_before_the_park()
        w = CNC.writes
        self.assertIn(('run', 1), w)
        self.assertLess(w.index(('feeder_stop', 1)), w.index(('run', 1)))
        self.assertEqual(self.parks, [(-500, -300)])

    def test_a_blocked_verdict_stops_the_feeder(self):
        COOL.stale = True
        CNC.xy_steps = (0, 0)
        self.m._motion_locked(self._print(), None)
        self.assertTrue(self.m._running_action_cancelled)
        self._assert_feeder_stopped_before_the_park()
        self.assertNotIn(('laser_latch', 0), CNC.writes)
        self.assertNotIn(('run', 1), CNC.writes)

    def test_a_timed_out_button_wait_stops_the_feeder(self):
        machine_mod._conf_float = lambda key, default: (
            1.0 if key == 'laser_button_timeout_s' else self._conf(key, default))
        CNC.xy_steps = (0, 0)
        self.m._motion_locked(self._print(), None)
        self.assertTrue(self.m._running_action_cancelled)
        self._assert_feeder_stopped_before_the_park()
        self.assertNotIn(('run', 1), CNC.writes)

    def test_a_crash_mid_job_stops_the_feeder(self):
        # An exception between the button and the run (here the warm-up
        # dwell) leaves through the same finally: the feeder is gone before
        # the action thread's cleanup ever sees it.
        def boom(s):
            raise RuntimeError('dwell failed')
        machine_mod.sleep = boom
        SW.press(delay=0.05)
        with self.assertRaises(RuntimeError):
            self.m._motion_locked(self._print(), None)
        self.assertTrue(self.feeder.stopped)
        self.assertIsNone(self.m._feeder)

    def test_the_park_refuses_a_live_feeder(self):
        # Whatever leaves a feeder alive, the park stops it and does not
        # run. No success is reported; the service re-hunts.
        CNC.xy_steps = (500, 300)
        feeder = self.JobFeeder(finished=False)
        self.m._feeder = feeder
        self.m._return_home(None)
        self.assertTrue(feeder.stopped)
        self.assertIsNone(self.m._feeder)
        self.assertNotIn(('clear_pulse', 1), CNC.writes)
        self.assertNotIn(('run', 1), CNC.writes)
        self.assertEqual(self.parks, [])
        self.assertEqual(job_events(), [])

    def test_a_long_print_that_runs_to_the_end_parks_after_its_feed(self):
        # The ordinary path through the same code: button pressed, the run
        # plays out, the feed is declared live and stopped before the park.
        CNC.xy_steps = (500, 300)
        CNC.run_reads = 3
        SW.press(delay=0.05)
        self.m._motion_locked(self._print(), None)
        self.assertFalse(self.m._running_action_cancelled)
        self.assertTrue(self.feeder.live)
        self._assert_feeder_stopped_before_the_park()
        w = CNC.writes
        self.assertEqual(w.count(('run', 1)), 2)          # the job, then the park
        self.assertIn('print:running', job_events())
        self.assertIn('print:return_to_home:succeeded', job_events())
        self.assertEqual(self.parks, [(-500, -300)])

    def test_a_print_that_fits_the_ring_is_unchanged(self):
        self.long_job = False
        CNC.xy_steps = (500, 300)
        CNC.run_reads = 3
        SW.press(delay=0.05)
        self.m._motion_locked(self._print(), None)
        self.assertFalse(self.feeder.live)
        self._assert_feeder_stopped_before_the_park()
        self.assertIn('print:return_to_home:succeeded', job_events())


class KernelFaultTests(unittest.TestCase):
    """The kernel leaving a run on its own (a fault, a disable) is not the
    program's end: the job is reported canceled, never completed, and the
    safing that follows runs to the end even where the kernel refuses the
    stop it starts with."""

    def setUp(self):
        CNC.__init__()
        SW.__init__()
        COOL.__init__()
        del EVENTS[:]
        self.m = make_machine()
        SW.handler = self.m._switch_event
        self.parks = []
        machine_mod.generate_linear_puls = lambda x, y, dev: self.parks.append((x, y))

    def tearDown(self):
        machine_mod.generate_linear_puls = lambda x, y, dev: None

    def test_a_fault_mid_run_is_an_abort(self):
        CNC.fault_after = 3
        aborted = self.m._run_loop()
        self.assertTrue(aborted)
        self.assertTrue(self.m._running_action_cancelled)

    def test_a_fault_while_paused_is_an_abort(self):
        SW.press(delay=0.05)                        # pause
        threading.Timer(0.3, lambda: setattr(CNC, '_state', MachineState.FAULT)).start()
        aborted = self.m._run_loop(pausable=True)
        self.assertTrue(aborted)
        self.assertTrue(self.m._running_action_cancelled)

    def test_a_faulted_park_reports_no_success(self):
        CNC.xy_steps = (500, 300)
        CNC.fault_after = 2
        self.m._return_home(None)
        self.assertNotIn('print:return_to_home:succeeded', job_events())

    def test_a_program_that_ends_is_not_an_abort(self):
        CNC.run_reads = 3
        self.assertFalse(self.m._run_loop())
        self.assertFalse(self.m._running_action_cancelled)

    def test_cleanup_runs_to_the_end_when_the_stop_is_refused(self):
        CNC.stop_error = OSError(1, 'EPERM')
        COOL.armed = True
        COOL.mode = 'run'
        self.m._feeder = FakeFeeder()
        self.m._action_cleanup()
        self.assertIn(('laser_latch', 1), CNC.writes)
        self.assertFalse(COOL.armed)
        self.assertEqual(COOL.mode, 'idle')
        self.assertIsNone(self.m._feeder)
        self.assertEqual(CNC.streaming_writes[-1], 0)

    def test_shutdown_runs_to_the_end_when_the_stop_is_refused(self):
        CNC.stop_error = OSError(1, 'EPERM')
        COOL.armed = True
        COOL.mode = 'run'
        self.m._shutdown()
        self.assertIn(('laser_latch', 1), CNC.writes)
        self.assertFalse(COOL.armed)
        self.assertEqual(COOL.mode, 'idle')
        self.assertTrue(COOL.stop)

    def test_shutdown_cancels_and_waits_for_the_running_action(self):
        # A run in progress on the action thread when the process goes
        # down: the thread is told to stop and is gone before the shutdown
        # returns, and it parked nothing.
        CNC.xy_steps = (500, 300)
        done = []

        def action():
            self.m._run_loop()
            self.m._return_home(None)
            done.append(True)
        t = threading.Thread(target=action, daemon=True)
        self.m._action_thread = t
        t.start()
        time.sleep(0.15)
        self.assertTrue(t.is_alive())
        self.m._shutdown()
        self.assertFalse(t.is_alive(), 'the action outlived the shutdown')
        self.assertTrue(done)
        self.assertTrue(self.m._running_action_cancelled)
        self.assertEqual(self.parks, [])
        self.assertEqual(CNC.writes.count(('run', 1)), 1)          # the job only
        self.assertNotIn('print:return_to_home:succeeded', job_events())
        self.assertIn(('stop', 1), CNC.writes)
        self.assertIn(('laser_latch', 1), CNC.writes)


class VerdictHoldTests(unittest.TestCase):
    """The cooling-engine contract in cloud mode: fire_ok is the latch,
    hold is a pause, resume_ok the resume, a hold with no end a cancel, and
    a verdict gone missing the emergency posture."""

    def setUp(self):
        CNC.__init__()
        SW.__init__()
        COOL.__init__()
        del EVENTS[:]
        self.m = make_machine()
        SW.handler = self.m._switch_event
        COOL.armed = True
        self._conf = machine_mod._conf_float
        machine_mod._conf_float = lambda key, default: (
            60.0 if key == 'cloud_hold_max_s' else self._conf(key, default))
        self.heater_offs = 0
        self._heater_off = machine_mod.WaterPump.heater_off
        machine_mod.WaterPump.heater_off = staticmethod(lambda: setattr(self, 'heater_offs', self.heater_offs + 1))

    def tearDown(self):
        machine_mod._conf_float = self._conf
        machine_mod.WaterPump.heater_off = self._heater_off

    def _release(self):
        COOL._fire_ok = True
        COOL._hold = False

    def test_a_hold_pauses_and_the_resume_brings_the_laser_back(self):
        threading.Timer(0.05, lambda: setattr(COOL, '_fire_ok', False)).start()
        threading.Timer(0.40, self._release).start()
        threading.Timer(0.70, lambda: setattr(CNC, '_reads_left', 1)).start()
        aborted = self.m._run_loop(pausable=True)
        self.assertFalse(aborted)
        self.assertFalse(self.m._running_action_cancelled)
        w = CNC.writes
        self.assertIn(('laser_latch', 1), w)
        self.assertIn(('resume', -2000), w)
        self.assertIn(('laser_latch', 0), w)
        self.assertIn(('resume', 1950), w)
        self.assertLess(w.index(('laser_latch', 0)), w.index(('resume', 1950)))
        self.assertEqual([e for e in job_events() if e in ('print:paused', 'print:resumed')],
                         ['print:paused', 'print:resumed'])

    def test_a_hold_with_fire_permitted_pauses_without_the_latch(self):
        # SUSPECT: hold asked, fire still permitted. The pause, not the latch.
        threading.Timer(0.05, lambda: setattr(COOL, '_hold', True)).start()
        threading.Timer(0.40, self._release).start()
        threading.Timer(0.70, lambda: setattr(CNC, '_reads_left', 1)).start()
        aborted = self.m._run_loop(pausable=True)
        self.assertFalse(aborted)
        w = CNC.writes
        self.assertIn(('resume', -2000), w)
        self.assertIn(('resume', 1950), w)
        self.assertNotIn(('laser_latch', 1), w)

    def test_a_hold_past_its_bound_cancels(self):
        machine_mod._conf_float = lambda key, default: (
            0.3 if key == 'cloud_hold_max_s' else self._conf(key, default))
        # the bound is clamped to 60 s and up: patch the reader the loop uses
        self.m._hold_max_s = lambda: 0.3
        threading.Timer(0.05, lambda: setattr(COOL, '_fire_ok', False)).start()
        aborted = self.m._run_loop(pausable=True)
        self.assertTrue(aborted)
        self.assertTrue(self.m._running_action_cancelled)
        self.assertNotIn(('resume', 1950), CNC.writes)

    def test_the_lid_ends_a_verdict_hold(self):
        threading.Timer(0.05, lambda: setattr(COOL, '_fire_ok', False)).start()
        SW.open_lid(delay=0.4)
        aborted = self.m._run_loop(pausable=True)
        self.assertTrue(aborted)
        self.assertTrue(self.m._running_action_cancelled)
        self.assertEqual(CNC.writes.count(('stop', 1)), 1)     # the hold's stop, no second

    def test_a_lost_verdict_holds_and_falls_back(self):
        threading.Timer(0.05, lambda: setattr(COOL, 'stale', True)).start()
        threading.Timer(0.40, lambda: setattr(COOL, 'stale', False)).start()
        threading.Timer(0.70, lambda: setattr(CNC, '_reads_left', 1)).start()
        aborted = self.m._run_loop(pausable=True)
        self.assertFalse(aborted)
        self.assertEqual(self.heater_offs, 1)
        self.assertEqual(COOL.fallbacks, 1)
        w = CNC.writes
        self.assertIn(('laser_latch', 1), w)
        self.assertIn(('resume', -2000), w)
        self.assertIn(('laser_latch', 0), w)
        self.assertIn(('resume', 1950), w)

    def test_the_button_does_not_resume_under_a_hold(self):
        SW.press(delay=0.05)                         # pause
        threading.Timer(0.20, lambda: setattr(COOL, '_hold', True)).start()
        SW.press(delay=0.40)                         # a resume the engine forbids
        threading.Timer(0.70, self._release).start()
        threading.Timer(1.00, lambda: setattr(CNC, '_reads_left', 1)).start()
        aborted = self.m._run_loop(pausable=True)
        self.assertFalse(aborted)
        w = CNC.writes
        self.assertEqual(w.count(('resume', 1950)), 1)
        self.assertEqual(w.count(('resume', -2000)), 1)

    def test_an_engine_stop_short_of_the_program_is_not_a_finish(self):
        # A fail tier (FIRE, CRASH) stops the kernel itself: idle, short of
        # the program. Never ':completed'.
        CNC.run_reads = 3
        CNC.processed = 100
        real_state = type(CNC).state
        def state(self_):
            s = real_state.fget(self_)
            if s is MachineState.IDLE:
                self_.processed = 100
            return s
        type(CNC).state = property(state)
        try:
            aborted = self.m._run_loop()
        finally:
            type(CNC).state = real_state
        self.assertTrue(aborted)
        self.assertTrue(self.m._running_action_cancelled)

    def test_a_motion_is_not_held_on_the_verdict(self):
        COOL.armed = False
        COOL._fire_ok = False
        CNC.run_reads = 3
        self.assertFalse(self.m._run_loop())
        self.assertNotIn(('laser_latch', 1), CNC.writes)


class VerdictWaitTests(unittest.TestCase):
    """Before the run: a print armed under a hold (WARMUP, COLD) waits for
    the engine to release it; only an absent engine refuses to arm."""

    def setUp(self):
        CNC.__init__()
        SW.__init__()
        COOL.__init__()
        del EVENTS[:]
        self.m = make_machine()
        SW.handler = self.m._switch_event
        self._conf = machine_mod._conf_float

    def tearDown(self):
        machine_mod._conf_float = self._conf

    def test_a_hold_is_waited_out(self):
        COOL._fire_ok = False
        COOL.name = 'WARMUP'
        threading.Timer(0.3, lambda: (setattr(COOL, '_fire_ok', True), setattr(COOL, 'name', 'OK'))).start()
        t0 = time.monotonic()
        self.m._verdict_wait()
        self.assertGreater(time.monotonic() - t0, 0.25)
        self.assertFalse(self.m._running_action_cancelled)
        self.assertNotIn(('laser_latch', 1), CNC.writes)

    def test_a_clean_verdict_does_not_wait(self):
        t0 = time.monotonic()
        self.m._verdict_wait()
        self.assertLess(time.monotonic() - t0, 0.1)

    def test_the_lid_ends_the_wait(self):
        COOL._fire_ok = False
        COOL.armed = True
        SW.open_lid(delay=0.05)
        self.m._verdict_wait()
        self.assertTrue(self.m._running_action_cancelled)
        self.assertIn(('laser_latch', 1), CNC.writes)
        self.assertFalse(COOL.armed)

    def test_a_lost_verdict_ends_the_wait(self):
        COOL._fire_ok = False
        COOL.armed = True
        threading.Timer(0.1, lambda: setattr(COOL, 'stale', True)).start()
        self.m._verdict_wait()
        self.assertTrue(self.m._running_action_cancelled)
        self.assertIn(('laser_latch', 1), CNC.writes)

    def test_the_bound_ends_the_wait(self):
        self.m._hold_max_s = lambda: 0.2
        COOL._fire_ok = False
        COOL.armed = True
        self.m._verdict_wait()
        self.assertTrue(self.m._running_action_cancelled)
        self.assertIn(('laser_latch', 1), CNC.writes)

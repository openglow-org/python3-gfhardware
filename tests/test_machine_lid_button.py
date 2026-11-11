"""
Copyright 2026 514 LLC d/b/a OpenGlow
Written by Scott Wiederhold
https://community.openglow.org

SPDX-License-Identifier:    MIT

Host tests for the cloud-mode job supervision in gfhardware.machine: how
a running or waiting job reacts to the lid, the interlock loop, the
button, and a service cancel, and what it reports about itself while it
runs. The real Machine code runs against a fake kernel wrapper (records
every cnc write, plays a simple state machine) and a fake switch monitor
(settable switch word, edge delivery on the switch thread), with the wire
events captured. Where a case measures a bound or would otherwise wait
out a hold, it runs on machine time (MachineClock): nothing sleeps, and a
job that never ends fails at the clock's horizon instead of hanging.

Run:  PYTHONPATH=.:../Glowforge-Utilities python3 -m unittest tests.test_machine_lid_button
"""
import heapq
import itertools
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
    IDLE; resume(+n) -> RUNNING again for the remainder.

    It also models the hardware's own button latch (hw_latch: 1 set, fire
    blocked; 0 cleared), as measured on the bench reference: a lock sets
    it, an open lid sets it, and only a press taken with the lock released
    and the lid closed clears it (FakeSwitches delivers the press and the
    lid). starts records, for every run and every forward resume, the
    hardware latch and the lock at that moment: a start that can fire is
    one with both at 0."""

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
        self.hw_latch = 1
        self.starts = []
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
    def set_motor_lock(self, v): self.writes.append(('motor_lock', str(v)))
    def set_x_decay(self, v): pass
    def set_x_current(self, v): pass
    def set_x_mode(self, v): pass
    def set_y_decay(self, v): pass
    def set_y_current(self, v): pass
    def set_y_mode(self, v): pass

    # -- job path
    def run(self):
        self.writes.append(('run', 1))
        self.starts.append(('run', self.hw_latch, self.latch))
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
        if steps > 0:
            self.starts.append(('resume', self.hw_latch, self.latch))
        self._state = MachineState.RUNNING
        self._backward = steps < 0
        self._reads_left = self.backtrack_reads if steps < 0 else self.run_reads

    def laser_latch(self, v):
        self.writes.append(('laser_latch', int(v)))
        self.latch = int(v)
        if self.latch:
            self.hw_latch = 1

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
        # The hardware's own button latch (FakeCNC): an open lid sets it; a
        # press clears it only with the lock released and the lid closed.
        if code == InputSwitch.SW_DOORS and not val:
            CNC.hw_latch = 1
        if (code == InputSwitch.SW_BUTTON and val and CNC.latch == 0
                and self.word[InputSwitch.SW_DOORS]):
            CNC.hw_latch = 0
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
    stands for a missing or outdated verdict file, and absent names the
    keys a document leaves out.

    The published armed flag is the engine's acknowledgment that it has
    taken the reported armed window and applied the run airflow, so it
    follows the reported state. Clearing ack models an engine that never
    takes the job: the verdict then stays the one computed for the idle
    session before the arm, which is what a caller must never fire on.

    ack_wait is the engine's own field for a fault that waits for its
    Acknowledge, whatever the verdict's name; None leaves the key out of
    the document. arms records every armed state reported, in order."""

    def __init__(self):
        self.armed = False
        self.arms = []
        self.mode = 'idle'
        self._fire_ok = True
        self._hold = None
        self.stale = False
        self.absent = ()
        self.ack = True
        self.ack_wait = None
        self.name = 'OK'
        self.reason = ''
        self.fallbacks = 0

    def set_armed(self, a):
        self.armed = bool(a)
        self.arms.append(self.armed)

    def set_mode(self, m): self.mode = m

    def verdict(self):
        if self.stale:
            return None
        hold = (not self._fire_ok) if self._hold is None else self._hold
        v = {'fire_ok': self._fire_ok, 'hold': hold, 'resume_ok': not hold,
             'armed': self.armed and self.ack,
             'verdict': self.name, 'reason': self.reason}
        if self.ack_wait is not None:
            v['ack_wait'] = self.ack_wait
        for key in self.absent:
            del v[key]
        return v

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
    leds_mod.set_button_breathe = lambda: leds_mod.button_colors.append('breathe')
    leds_mod.pattern = None             # what button_pattern() reads; None: unreadable
    leds_mod.button_pattern = lambda: leds_mod.pattern
    leds_mod.head_all_led_off = lambda: None
    leds_mod.set_lid_led = lambda v: None
    leds_mod.set_head_led_from_pulse = lambda v: None
    leds_mod.__all__ = ['set_button_color', 'set_button_breathe', 'button_pattern', 'head_all_led_off',
                        'set_lid_led',
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

        # The lens driver's writes land in the kernel wrapper's list, so a
        # test can order them against the lock and the run.
        @staticmethod
        def enable(): cnc_mod.cnc.writes.append(('z_enable', 0))

        @staticmethod
        def set_current(cur): cnc_mod.cnc.writes.append(('z_current', cur.value))
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


_MONOTONIC, _SLEEP = machine_mod.monotonic, machine_mod.sleep


class MachineClock:
    """Machine time for one case. The machine module's monotonic() and
    sleep() and the machine's run-loop wake read it, and what the case
    schedules (a press, a verdict change, a sample) runs on the machine's
    own thread when its time comes round, inside the wait or the sleep that
    spans it, as an edge from the switch thread would. Nothing really
    sleeps, so a case costs only its passes, and a bound is read to the
    pass. A job that would go on for good (a hold never lifted, a run that
    never ends) fails at the horizon instead of hanging."""

    START = 1000.0      # well past zero, as a real monotonic clock reads

    def __init__(self, case, machine, horizon=30.0):
        self.now = self.START
        self._horizon = self.START + horizon
        self._due = []
        self._order = itertools.count()
        case.addCleanup(setattr, machine_mod, 'monotonic', _MONOTONIC)
        case.addCleanup(setattr, machine_mod, 'sleep', _SLEEP)
        machine_mod.monotonic = lambda: self.now
        machine_mod.sleep = lambda s: self.run_to(self.now + s)
        machine._run_wake = _MachineWake(self)

    @property
    def elapsed(self):
        return self.now - self.START

    def at(self, t, fn):
        """Run fn at t seconds of machine time."""
        heapq.heappush(self._due, (self.START + t, next(self._order), fn))

    def press(self, t):
        self.at(t, lambda: (SW._edge(InputSwitch.SW_BUTTON, True),
                            SW._edge(InputSwitch.SW_BUTTON, False)))

    def run_to(self, until, wake=None):
        """Advance to until, running what falls due; a wake set on the way
        ends the wait there."""
        while self._due and self._due[0][0] <= until:
            at, _, fn = heapq.heappop(self._due)
            self.now = max(self.now, at)
            fn()
            if wake is not None and wake.flag:
                return True
        self.now = max(self.now, until)
        if self.now > self._horizon:
            raise AssertionError('the job was still going after %.0f s of machine time'
                                 % self.elapsed)
        return wake is not None and wake.flag


class _MachineWake:
    """The run loop's wake (a threading.Event) on machine time."""

    def __init__(self, clock):
        self._clock = clock
        self.flag = False

    def set(self): self.flag = True
    def clear(self): self.flag = False
    def is_set(self): return self.flag

    def wait(self, timeout):
        if self.flag:
            return True
        return self._clock.run_to(self._clock.now + timeout, self)


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

    def test_park_ignores_a_service_cancel_and_the_lid(self):
        for name, arrange in (('service cancel', lambda: setattr(self.m, '_running_action_cancelled', True)),
                              ('lid', lambda: SW.open_lid(delay=0.02))):
            with self.subTest(name):
                self.setUp()
                CNC.run_reads = 5
                arrange()
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
        # The job is paused (stopped, retraced, held idle) when the kernel
        # reports a dry ring: the hold must not hide it. The job ends
        # canceled, the underrun is acknowledged with a stop of its own,
        # and the held job never resumes.
        SW.press(delay=0.05)                       # pause
        threading.Timer(0.4, lambda: setattr(CNC, '_state', MachineState.UNDERRUN)).start()
        aborted = self.m._run_loop(pausable=True)
        self.assertTrue(aborted)
        self.assertTrue(self.m._running_action_cancelled)
        self.assertEqual(job_events(), ['print:paused'])
        self.assertEqual(CNC.writes.count(('stop', 1)), 2)   # the pause, then the acknowledgement
        self.assertFalse([w for w in CNC.writes if w[0] == 'resume' and w[1] > 0])

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

    def test_the_retrace_walks_what_the_ring_holds_and_leads_by_it(self):
        # With the factory distances (2000 back, a 1950 lead: a 50-tick
        # overlap), each row is what the ring offers and the writes the
        # retrace and the resume make. A refused backtrack holds where the
        # decel stopped and resumes with the shortest lead, because nothing
        # was retraced: a lead over ground not yet cut would leave it
        # unburned, and a lead of zero is the kernel's "no laser at all". A
        # short history shortens the retrace and the lead with it. No
        # history asks for no backward run. No readback asks for the
        # configured distance and lets the kernel refuse it.
        def refuse_backward():
            orig = CNC.resume

            def resume(steps):
                if steps < 0:
                    raise OSError(1, 'EPERM')
                orig(steps)
            CNC.resume = resume
        rows = (
            ('backtrack refused', refuse_backward, [('resume', 1)]),
            ('short history', lambda: setattr(CNC, 'backtrack_budget', 500),
             [('resume', -500), ('resume', 450)]),
            ('no history', lambda: setattr(CNC, 'backtrack_budget', 0), [('resume', 1)]),
            ('no readback', lambda: setattr(CNC, 'max_backtrack_error', OSError(2, 'ENOENT')),
             [('resume', -2000), ('resume', 1950)]),
        )
        try:
            for name, arrange, writes in rows:
                with self.subTest(name):
                    CNC.__init__()
                    vars(CNC).pop('resume', None)
                    arrange()
                    ok, retraced = self.m._retrace(2000)
                    self.assertTrue(ok)
                    self.assertTrue(self.m._resume_retraced(retraced, 50))
                    self.assertEqual(CNC.writes, writes)
        finally:
            vars(CNC).pop('resume', None)


class ButtonWaitTests(unittest.TestCase):

    def setUp(self):
        CNC.__init__()
        SW.__init__()
        COOL.__init__()
        del EVENTS[:]
        self.m = make_machine()
        SW.handler = self.m._switch_event
        COOL.armed = True

    def test_the_wait_puts_its_breathing_back_once_something_wrote_over_it(self):
        del LEDS.button_colors[:]
        LEDS.pattern = 'red'            # forgectrl's flash: left alone
        SW.press(delay=1.3)
        t = threading.Timer(0.6, lambda: setattr(LEDS, 'pattern', 'off'))   # the flash ended dark
        t.start()
        self.addCleanup(t.cancel)
        self.addCleanup(setattr, LEDS, 'pattern', None)
        self.m._button_wait({'id': 42})
        self.assertEqual(LEDS.button_colors.count('breathe'), 2, LEDS.button_colors)

    def test_press_ends_the_wait(self):
        del LEDS.button_colors[:]
        SW.press(delay=0.05)
        self.m._button_wait({'id': 42})
        self.assertFalse(self.m._running_action_cancelled)
        self.assertNotIn(('laser_latch', 1), CNC.writes)
        # it breathes while it waits, and stops asking once the press is taken
        self.assertEqual(LEDS.button_colors[0], 'breathe')
        self.assertNotEqual(LEDS.button_colors[-1], 'breathe')

    def test_lid_open_cancels_the_wait(self):
        SW.open_lid(delay=0.05)
        self.m._button_wait({'id': 42})
        self.assertTrue(self.m._running_action_cancelled)
        self.assertIn(('laser_latch', 1), CNC.writes)
        self.assertFalse(COOL.armed)

    def test_a_service_cancel_ends_the_wait_with_the_latch_locked(self):
        # The caller has unlocked the latch for the press; the service
        # cancels at 0.1, and the wait ends at its next pass, not at its
        # timeout, locked and disarmed.
        clock = MachineClock(self, self.m)
        CNC.latch = 0
        clock.at(0.1, lambda: setattr(self.m, '_running_action_cancelled', True))
        self.m._button_wait({'id': 42})
        self.assertLess(clock.elapsed, 0.25)
        self.assertEqual(CNC.latch, 1)
        self.assertFalse(COOL.armed)

    def test_interlock_open_cancels_the_wait(self):
        # The loop's level ends the wait at once, not the wait's own bound.
        self._short_timeout()
        SW.open_interlock(delay=0.05)
        t0 = time.monotonic()
        self.m._button_wait({'id': 42})
        dt = time.monotonic() - t0
        self.assertTrue(self.m._running_action_cancelled)
        self.assertIn(('laser_latch', 1), CNC.writes)
        self.assertLess(dt, 0.5, 'the wait ran to its bound (%.3f s)' % dt)

    def test_press_with_lid_open_does_not_arm(self):
        SW.word[InputSwitch.SW_DOORS] = False
        SW.press(delay=0.02)
        self.m._button_wait({'id': 42})
        self.assertTrue(self.m._running_action_cancelled)
        self.assertIn(('laser_latch', 1), CNC.writes)

    def _short_timeout(self):
        conf = machine_mod._conf_float
        self.addCleanup(setattr, machine_mod, '_conf_float', conf)
        machine_mod._conf_float = lambda key, default: (
            1.0 if key == 'laser_button_timeout_s' else conf(key, default))

    def test_a_button_held_before_the_wait_never_arms(self):
        # No edge arrives (the switch thread's path); the level reads
        # pressed from before the wait began and is never seen released,
        # so the wait runs out and relocks rather than arming on it.
        self._short_timeout()
        SW.word[InputSwitch.SW_BUTTON] = True
        self.m._button_wait({'id': 42})
        self.assertTrue(self.m._running_action_cancelled)
        self.assertIn(('laser_latch', 1), CNC.writes)
        self.assertFalse(COOL.armed)

    def test_a_level_after_a_release_still_arms(self):
        # The backstop for a gone switch thread: held, released, pressed
        # again, all as levels with no edge, and the wait ends on the press.
        self._short_timeout()
        SW.word[InputSwitch.SW_BUTTON] = True
        SW._later(0.2, lambda: SW.word.__setitem__(InputSwitch.SW_BUTTON, False))
        SW._later(0.4, lambda: SW.word.__setitem__(InputSwitch.SW_BUTTON, True))
        self.m._button_wait({'id': 42})
        self.assertFalse(self.m._running_action_cancelled)
        self.assertNotIn(('laser_latch', 1), CNC.writes)


class StartGateTests(unittest.TestCase):

    def setUp(self):
        CNC.__init__()
        SW.__init__()
        COOL.__init__()
        del EVENTS[:]
        self.m = make_machine()
        SW.handler = self.m._switch_event

    def test_an_open_enclosure_at_start_cancels_a_gated_job(self):
        # The lid (inactive = open) and the remote-interlock loop (active =
        # open) refuse the start the same way: canceled, nothing started.
        for code, val in ((InputSwitch.SW_DOORS, False), (InputSwitch.SW_INTERLOCK, True)):
            with self.subTest(switch=code.name):
                SW.__init__()
                self.m = make_machine()
                SW.word[code] = val
                self.m._motion({'id': 42, 'action_type': 'motion', 'motion_url': 'x'})
                self.assertTrue(self.m._running_action_cancelled)
                self.assertNotIn(('run', 1), CNC.writes)
                self.assertNotIn(('motor_lock', '0'), CNC.writes)

    def test_an_acknowledged_hunt_moves_nothing(self):
        # A GRBL homing session answers the service's hunt as done: no lens
        # home, no hunt file, no run. The lens reference is the runner's own,
        # after the session.
        homed = machine_mod.ZAxis.homed
        set_cfg('HOMING.HUNT_ACK_ONLY', True)
        try:
            self.m._hunt({'id': 42, 'action_type': 'hunt', 'motion_url': 'x'})
        finally:
            set_cfg('HOMING.HUNT_ACK_ONLY', False)
        self.assertEqual(machine_mod.ZAxis.homed, homed)
        self.assertNotIn(('run', 1), CNC.writes)
        self.assertNotIn(('motor_lock', '0'), CNC.writes)

    def test_hunt_start_gate_ignores_the_lid(self):
        SW.word[InputSwitch.SW_DOORS] = False
        self.assertTrue(self.m._safe_to_move(lid_gated=False))
        self.assertFalse(self.m._safe_to_move(lid_gated=True))

    def _coolant(self, c):
        self.addCleanup(setattr, machine_mod.temp_sensor.water_2, 'C', machine_mod.temp_sensor.water_2.C)
        machine_mod.temp_sensor.water_2.C = c

    def test_a_loop_over_the_start_limit_does_not_move(self):
        # THERMAL.MAX_START_TEMP is 50 here; at the limit it still starts.
        self._coolant(50.0)
        self.assertTrue(self.m._safe_to_move())
        self._coolant(50.5)
        self.assertFalse(self.m._safe_to_move())

    def test_a_dead_coolant_sensor_does_not_move(self):
        # A dead or disconnected sensor reads the -273.15 sentinel, which
        # must not pass as cold enough.
        self._coolant(-273.15)
        self.assertFalse(self.m._safe_to_move())


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
        frames = queue.Queue()

        aborted = self.m._run_loop(pausable=True, progress=machine_mod._JobProgress(
            frames, 42, 'print:progress', 1000, interval=3600))

        self.assertFalse(aborted)
        self.assertFalse(self.m._running_action_cancelled)
        w = CNC.writes
        self.assertIn(('stop', 1), w)               # stopped before the ring ran dry
        self.assertIn(('resume', -2000), w)         # retraced like a pause
        self.assertIn(('resume', 1950), w)          # and led back on over cut ground
        self.assertEqual(job_events(), ['print:paused', 'print:resumed'])
        self.assertEqual(frames.qsize(), 4)         # start, hold, resume, end: as a pause reports

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
        # Taken in slices, so the lid and a cancel are seen between them.
        self.assertAlmostEqual(sum(self.slept),
                               machine_mod.WARM_UP_DEFAULT_S + machine_mod.COOL_DOWN_DEFAULT_S)
        self.assertTrue(all(s <= machine_mod.DWELL_SLICE_S + 1e-9 for s in self.slept))

    def test_a_configured_period_wins_and_nonsense_falls_back(self):
        # A configured period wins; the zero the old sample shipped still
        # skips the hold, as the operator chose; nonsense falls back to the
        # factory period.
        for value, want in (('5', 5.0), (0, 0.0), ('soon', machine_mod.WARM_UP_DEFAULT_S)):
            with self.subTest(value=value):
                del self.slept[:]
                set_cfg('MOTION.WARM_UP_DELAY', value)
                self.assertEqual(self.m._dwell('warm_up'), want)
                self.assertAlmostEqual(sum(self.slept), want)

    def test_an_unhandled_header_key_is_a_gap(self):
        # ZZzz has no applier and no recorded reason: a gap. A key with a
        # recorded reason (DECLARED_IGNORED) has no applier either, and is
        # not one.
        declared = sorted(machine_mod.DECLARED_IGNORED)[0]
        header = {'STfr': 10000, 'AArd': 200, 'MCsn': 0, 'PDfm': 0,
                  declared: 1, 'ZZzz': 4242}
        self.assertEqual(self.m._log_header_gaps(header), ['ZZzz'])

    def test_applied_and_checked_keys_are_not_reported_as_gaps(self):
        # The motion keys have appliers, the serial and format are checked
        # before the ring is touched, and the lifecycle keys get a line of
        # their own: none of them is an unrecorded decision.
        header = {'STfr': 10000, 'AArd': 200, 'EFrd': 300, 'IFrd': 100,
                  'MCsn': 0, 'PDfm': 0, 'CFrh': 1, 'CCwp': 5000,
                  'CCrp': 10000, 'CCup': 1}
        self.assertEqual(self.m._log_header_gaps(header), [])

    def test_every_declared_key_has_a_reason_and_no_applier(self):
        applied = {k for k, s in machine_mod.MACHINE_SETTINGS.items()
                   if s.idle or s.run or s.cool_down}
        for key, why in machine_mod.DECLARED_IGNORED.items():
            self.assertEqual(len(key), 4, key)
            self.assertTrue(why, key)
            self.assertNotIn(key, applied, key)
            self.assertNotIn(key, machine_mod.LIMIT_TAGS, key)
            self.assertNotIn(key, machine_mod.INERT_LIMIT_TAGS, key)


class LidLampTests(unittest.TestCase):
    """The action's settings reach the lid capture (BaseMachine.lamp_level's
    own cases are Glowforge-Utilities')."""

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

    def test_lcfl_0_captures_the_bed_as_it_is_lit_and_the_image_still_goes_out(self):
        self.m._lid_image({'id': 3}, {'LCfl': 0})
        self.assertEqual(self.lamp(), 0)
        self.assertEqual(UPLOADS, [3])


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
        self.assertEqual(frame['action_id'], 42)
        self.assertEqual(frame['progress'], 'print:progress')
        self.assertEqual(frame['current'], 994)
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
        with self.assertLogs(machine_mod.logger, level='WARNING'):
            prog = self.progress(4000)
            prog.send(force=True)
            prog.send(force=True)
        frames = self.frames()
        self.assertEqual([f['current'] for f in frames], [4000, 4000])
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
            self.completes = False      # the feed finishes once declared live
            self.declare_error = None   # the declaration raises this

        def start(self):
            pass

        def wait_primed(self, timeout=120.0):
            return True

        def declare_live_feed(self):
            if self.declare_error is not None:
                raise self.declare_error
            self.live = True
            if self.completes:
                self.finished = True

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
        self.feed_completes = False
        self.declare_error = None
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
        self.feeder.completes = self.feed_completes
        self.feeder.declare_error = self.declare_error
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

    def test_the_lens_is_unlocked_and_driven_for_the_motion_and_rested_after(self):
        # The service's Z steps must reach the lens: the lock the idle config
        # sets (8) is lifted before the ring is fed and put back at idle, and
        # the head's Z driver is enabled and set to its drive current (0)
        # before the feed and rested at its hold current (1) at idle. At the
        # hold current the lens rises two steps into the service's ramp and
        # stalls, so the focus is never made.
        CNC.xy_steps = (0, 0)
        SW.open_lid(delay=0.05)
        self.m._motion_locked(self._print(), None)
        w = CNC.writes
        for first, then in ((('motor_lock', '0'), ('motor_lock', '8')),
                            (('z_current', 0), ('z_current', 1)),
                            (('z_enable', 0), ('motor_lock', '8'))):
            self.assertIn(first, w)
            self.assertIn(then, w)
            self.assertLess(w.index(first), w.index(then))
        # The driver is at its drive current before any feed and any run.
        self.assertLess(w.index(('z_current', 0)), w.index(('feeder_stop', 1)))
        self.assertEqual(w.count(('z_current', 0)), 1)

    def test_a_fault_waiting_for_its_acknowledge_refuses_the_print_before_the_button(self):
        CNC.xy_steps = (0, 0)
        COOL._fire_ok = False
        COOL.name = 'ACK'
        seen = []
        self.m._button_wait = lambda msg: seen.append('asked')
        self.m._motion_locked(self._print(), None)
        self.assertEqual(seen, [], "no press is asked for")
        self.assertTrue(self.m._running_action_cancelled)
        self.assertNotIn(True, COOL.arms, 'no armed session opened')
        self.assertNotIn(('laser_latch', 0), CNC.writes)

    def test_an_acknowledge_waiting_under_another_verdict_refuses_the_print_before_the_button(self):
        # A critical fault ended the last job and the loop is still hot: the
        # verdict's name is OVERTEMP, and only ack_wait says an Acknowledge
        # is pending. The print is refused before the armed session opens,
        # not after the hold has been waited out.
        CNC.xy_steps = (0, 0)
        COOL._fire_ok = False
        COOL.name = 'OVERTEMP'
        COOL.ack_wait = True
        seen = []

        def press(msg):
            seen.append('asked')
            self.m._running_action_cancelled = True     # a press asked for ends the test here
        self.m._button_wait = press
        self.m._motion_locked(self._print(), None)
        self.assertEqual(seen, [], "no press is asked for")
        self.assertTrue(self.m._running_action_cancelled)
        self.assertNotIn(True, COOL.arms, 'no armed session opened')
        self.assertNotIn(('laser_latch', 0), CNC.writes)

    def test_a_hold_with_no_acknowledge_waiting_still_asks_for_the_press(self):
        # The control: the same hot loop with nothing to acknowledge is
        # waited out, and then the press is asked for.
        CNC.xy_steps = (0, 0)
        COOL._fire_ok = False
        COOL.name = 'OVERTEMP'
        COOL.ack_wait = False
        timer = threading.Timer(0.3, lambda: (setattr(COOL, '_fire_ok', True), setattr(COOL, 'name', 'OK')))
        timer.start()
        self.addCleanup(timer.cancel)
        seen = []

        def press(msg):
            seen.append('asked')
            self.m._running_action_cancelled = True     # the test ends here
        self.m._button_wait = press
        self.m._motion_locked(self._print(), None)
        self.assertEqual(seen, ['asked'])

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

    # -- the latch: locked until the verdict has passed, unlocked before
    #    the press is asked for, so the press is taken with it unlocked ----
    def _lamp_into_the_writes(self):
        """The breathing lamp recorded among the cnc writes, so it can be
        ordered against the unlock."""
        self.addCleanup(setattr, machine_mod, 'set_button_breathe', machine_mod.set_button_breathe)
        self.addCleanup(setattr, machine_mod, 'set_button_color', machine_mod.set_button_color)
        machine_mod.set_button_breathe = lambda: CNC.writes.append(('lamp', 'breathe'))
        machine_mod.set_button_color = lambda c: CNC.writes.append(
            ('lamp', 'off' if c is ButtonColor.OFF else 'solid'))

    @staticmethod
    def _breathing_while_locked():
        """The writes after which the button breathes with the latch locked
        (with _lamp_into_the_writes; the latch starts locked, the lamp dark)."""
        latch, lamp, bad = 1, 'off', []
        for i, w in enumerate(CNC.writes):
            if w[0] == 'laser_latch':
                latch = w[1]
            elif w[0] == 'lamp':
                lamp = w[1]
            if latch == 1 and lamp == 'breathe':
                bad.append((i, w))
        return bad

    def _go_around(self, close, clear):
        """A print whose verdict stops permitting fire at 0.2 s into the
        button wait and permits it again at 0.6 s: a press at 0.35 s (under
        the lock) and the press that counts at 1.0 s."""
        self.feed_completes = True
        CNC.xy_steps = (0, 0)
        CNC.run_reads = 3
        self._lamp_into_the_writes()
        held = []
        timers = [threading.Timer(0.2, close),
                  threading.Timer(0.45, lambda: held.append(
                      (CNC.latch, self.m._running_action_cancelled, COOL.armed,
                       [w for w in CNC.writes if w[0] == 'lamp'][-1]))),
                  threading.Timer(0.6, clear),
                  threading.Timer(0.9, lambda: held.append(('run', 1) in CNC.writes))]
        for t in timers:
            t.start()
            self.addCleanup(t.cancel)
        SW.press(delay=0.35)
        SW.press(delay=1.0)
        self.m._motion_locked(self._print(), None)
        self.assertEqual(held[:1], [(1, False, True, ('lamp', 'off'))],
                         'while the verdict is closed: locked, not canceled, still armed, the button dark')
        self.assertEqual(held[1:], [False], 'the press taken under the lock started nothing')
        self.assertFalse(self.m._running_action_cancelled)
        self.assertEqual(CNC.starts, [('run', 0, 0)],
                         'the run starts on the second press, with the hardware latch cleared')
        self.assertEqual(self._breathing_while_locked(), [])
        self.assertEqual(CNC.writes.count(('lamp', 'breathe')), 2, 'asked for the press twice')

    def test_a_verdict_that_closes_during_the_button_wait_is_waited_out_and_the_press_asked_again(self):
        self._go_around(lambda: (setattr(COOL, '_fire_ok', False), setattr(COOL, 'name', 'COLD')),
                        lambda: (setattr(COOL, '_fire_ok', True), setattr(COOL, 'name', 'OK')))

    def test_a_verdict_missing_for_a_moment_during_the_button_wait_is_waited_out_the_same_way(self):
        self._go_around(lambda: setattr(COOL, 'stale', True), lambda: setattr(COOL, 'stale', False))

    def test_a_verdict_lost_for_longer_than_its_grace_cancels(self):
        self.addCleanup(setattr, machine_mod, 'VERDICT_STALL_GRACE_S', machine_mod.VERDICT_STALL_GRACE_S)
        machine_mod.VERDICT_STALL_GRACE_S = 0.5
        # The arm's own bound sits well past the grace (2 s + 3 s).
        self.m._hold_max_s = lambda: 3.0
        machine_mod._conf_float = lambda key, default: (
            2.0 if key == 'laser_button_timeout_s' else self._conf(key, default))
        CNC.xy_steps = (0, 0)
        timer = threading.Timer(0.2, lambda: setattr(COOL, 'stale', True))
        timer.start()
        self.addCleanup(timer.cancel)
        t0 = time.monotonic()
        with self.assertLogs(machine_mod.logger, 'WARNING') as logs:
            self.m._motion_locked(self._print(), None)
        self.assertLess(time.monotonic() - t0, 2.0)
        self.assertTrue(any('cooling hold cooling verdict lost' in line for line in logs.output))
        self.assertTrue(self.m._running_action_cancelled)
        self.assertNotIn(('run', 1), CNC.writes)
        self.assertEqual(CNC.latch, 1)
        self.assertFalse(COOL.armed)

    def test_an_acknowledge_that_comes_due_during_the_button_wait_cancels(self):
        # The arm's bound is 3 s here (1 s for the button, 2 s for a hold),
        # so a wait that sat the Acknowledge out would end in seconds, not
        # in the default 35 minutes.
        self.m._button_timeout_s = lambda: 1.0
        self.m._hold_max_s = lambda: 2.0
        CNC.xy_steps = (0, 0)
        timer = threading.Timer(0.2, lambda: (setattr(COOL, '_fire_ok', False), setattr(COOL, 'name', 'ACK')))
        timer.start()
        self.addCleanup(timer.cancel)
        t0 = time.monotonic()
        self.m._motion_locked(self._print(), None)
        self.assertLess(time.monotonic() - t0, 1.5)
        self.assertTrue(self.m._running_action_cancelled)
        self.assertEqual(CNC.latch, 1)
        self.assertFalse(COOL.armed)

    def test_the_arm_is_bounded_however_often_the_verdict_comes_and_goes(self):
        # The button waits 1 s and a hold 0.6 s here, so the arm ends 1.6 s
        # after the press is first asked for. The verdict flips every 0.2 s
        # for 5 s: no single wait ever reaches its own bound.
        self.m._hold_max_s = lambda: 0.6
        machine_mod._conf_float = lambda key, default: (
            1.0 if key == 'laser_button_timeout_s' else self._conf(key, default))
        CNC.xy_steps = (0, 0)
        stop = threading.Event()
        self.addCleanup(stop.set)

        def flap():
            for _ in range(25):
                if stop.wait(0.2):
                    return
                COOL._fire_ok = not COOL._fire_ok
            COOL._fire_ok = True
        flapper = threading.Thread(target=flap, daemon=True)
        flapper.start()
        t0 = time.monotonic()
        self.m._motion_locked(self._print(), None)
        took = time.monotonic() - t0
        stop.set()
        flapper.join(1.0)
        self.assertLess(took, 2.6, 'the arm outlived its overall bound: %.1f s' % took)
        self.assertGreater(took, 1.0, 'the first closed verdict ended the arm')
        self.assertTrue(self.m._running_action_cancelled)
        self.assertNotIn(('run', 1), CNC.writes)
        self.assertEqual(CNC.latch, 1)
        self.assertFalse(COOL.armed)

    def test_each_wait_ends_at_the_arms_bound(self):
        # The button waits 1 s and a hold 0.65 s, so the arm ends 1.65 s
        # after the press is first asked for, at 0, whichever wait it is in.
        # Each row closes the verdict during a button wait so that the wait
        # that follows would run past that bound on its own clock:
        #  - the button wait: closed at 0.8, open again at 1.2; the second
        #    button wait starts at 1.3, and its own timeout is 2.3;
        #  - the verdict wait: closed at 0.3, open at 0.5, closed again at
        #    1.2 and left closed; that verdict wait's own bound is 1.85.
        rows = (('the button wait', ((0.8, False), (1.2, True))),
                ('the verdict wait', ((0.3, False), (0.5, True), (1.2, False))))
        for name, fire_ok in rows:
            with self.subTest(name):
                self.tearDown()
                self.setUp()
                self.m._button_timeout_s = lambda: 1.0
                self.m._hold_max_s = lambda: 0.65
                clock = MachineClock(self, self.m)
                for at, ok in fire_ok:
                    clock.at(at, lambda ok=ok: setattr(COOL, '_fire_ok', ok))
                COOL.set_armed(True)
                self.m._arm_for_the_press(self._print())
                self.assertLess(clock.elapsed, 1.65 + 0.3, 'the arm outlived its bound')
                self.assertTrue(self.m._running_action_cancelled)
                self.assertEqual(CNC.latch, 1)
                self.assertFalse(COOL.armed)

    def test_the_press_is_taken_with_the_latch_unlocked_and_the_run_starts_on_it(self):
        self.feed_completes = True
        CNC.xy_steps = (0, 0)
        CNC.run_reads = 3
        self._lamp_into_the_writes()
        seen = []
        SW._later(0.05, lambda: (seen.append(CNC.latch),
                                 SW._edge(InputSwitch.SW_BUTTON, True),
                                 SW._edge(InputSwitch.SW_BUTTON, False)))
        self.m._motion_locked(self._print(), None)
        self.assertFalse(self.m._running_action_cancelled)
        self.assertEqual(CNC.starts, [('run', 0, 0)],
                         'the run starts with the hardware latch cleared and the lock released')
        self.assertEqual(seen, [0], 'the latch was locked at the press')
        w = CNC.writes
        self.assertIn(('run', 1), w)
        self.assertLess(w.index(('laser_latch', 0)), w.index(('lamp', 'breathe')),
                        'the button breathes only once a press can count')
        self.assertEqual(w.count(('laser_latch', 0)), 1)
        self.assertGreater(w.index(('laser_latch', 1), w.index(('run', 1))),
                           w.index(('run', 1)), 'the job end relocks')
        self.assertLess(EVENTS.index('button:pressed'), EVENTS.index('print:warmup:starting'),
                        'the warm-up is announced after the press')

    def test_a_press_before_the_unlock_does_not_count(self):
        # The session opens under a hold: the latch stays locked, the button
        # dark, and a press then is not the job's press. The hold clears at
        # 0.4, the latch unlocks, the button breathes, and the press at 0.8
        # starts the run.
        self.feed_completes = True
        CNC.xy_steps = (0, 0)
        CNC.run_reads = 3
        self._lamp_into_the_writes()
        COOL._fire_ok = False
        COOL.name = 'WARMUP'
        at_early_press = []
        SW._later(0.15, lambda: (at_early_press.append((CNC.latch, ('lamp', 'breathe') in CNC.writes)),
                                 SW._edge(InputSwitch.SW_BUTTON, True),
                                 SW._edge(InputSwitch.SW_BUTTON, False)))
        timer = threading.Timer(0.4, lambda: (setattr(COOL, '_fire_ok', True), setattr(COOL, 'name', 'OK')))
        timer.start()
        self.addCleanup(timer.cancel)
        before_second = []
        SW._later(0.7, lambda: before_second.append(('run', 1) in CNC.writes))
        SW.press(delay=0.8)
        self.m._motion_locked(self._print(), None)
        self.assertEqual(at_early_press, [(1, False)], 'locked and dark at the early press')
        self.assertEqual(before_second, [False], 'the early press started nothing')
        self.assertFalse(self.m._running_action_cancelled)
        self.assertEqual(CNC.starts, [('run', 0, 0)])

    def test_a_verdict_that_closes_or_goes_missing_during_the_warm_up_relocks_and_cancels(self):
        # A 3 s warm-up on machine time, the press at 0.05; at 0.35 the
        # verdict refuses fire, or goes missing. The job ends within the
        # warm-up's slice.
        set_cfg('MOTION.WARM_UP_DELAY', '3')
        set_cfg('MOTION.COOL_DOWN_DELAY', '0')
        self.addCleanup(set_cfg, 'MOTION.WARM_UP_DELAY', None)
        self.addCleanup(set_cfg, 'MOTION.COOL_DOWN_DELAY', None)
        rows = (('refuses fire', lambda: (setattr(COOL, '_fire_ok', False), setattr(COOL, 'name', 'SENSOR'))),
                ('goes missing', lambda: setattr(COOL, 'stale', True)))
        for name, close in rows:
            with self.subTest(name):
                self.tearDown()
                self.setUp()
                clock = MachineClock(self, self.m)
                CNC.xy_steps = (0, 0)
                clock.press(0.05)
                clock.at(0.35, close)
                self.m._motion_locked(self._print(), None)
                self.assertTrue(self.m._running_action_cancelled)
                self.assertLess(clock.elapsed, 0.35 + machine_mod.DWELL_SLICE_S + 0.05)
                self.assertNotIn(('run', 1), CNC.writes)
                self.assertEqual(CNC.latch, 1)
                self.assertFalse(COOL.armed)

    def test_a_fire_blocked_verdict_before_the_run_leaves_the_latch_locked_and_cancels(self):
        # The engine refuses fire for the whole bounded wait (a fail tier
        # left standing, a hold that never clears): the latch was never
        # unlocked, so there is nothing to relock, and the job never runs.
        self.m._hold_max_s = lambda: 0.3
        COOL._fire_ok = False
        COOL.name = 'CRASH'
        CNC.xy_steps = (0, 0)
        SW.press(delay=0.05)
        self.m._motion_locked(self._print(), None)
        self.assertTrue(self.m._running_action_cancelled)
        self.assertNotIn(('laser_latch', 0), CNC.writes)
        self.assertNotIn(('run', 1), CNC.writes)
        self.assertFalse(COOL.armed)
        self._assert_feeder_stopped_before_the_park()

    # -- the warm-up is supervised ----------------------------------------
    def test_a_lid_opened_during_the_warm_up_cancels_at_once(self):
        machine_mod.sleep = self._sleep                  # a real warm-up
        set_cfg('MOTION.WARM_UP_DELAY', '3')
        set_cfg('MOTION.COOL_DOWN_DELAY', '0')           # the cancel's tail is not the point
        try:
            CNC.xy_steps = (0, 0)
            SW.press(delay=0.05)
            SW.open_lid(delay=0.35)                      # inside the warm-up
            t0 = time.monotonic()
            self.m._motion_locked(self._print(), None)
            took = time.monotonic() - t0
        finally:
            set_cfg('MOTION.WARM_UP_DELAY', None)
            set_cfg('MOTION.COOL_DOWN_DELAY', None)
        self.assertTrue(self.m._running_action_cancelled)
        self.assertLess(took, 0.35 + 0.3, 'the warm-up sat out the lid: %.2f s' % took)
        self.assertEqual(CNC.latch, 1, 'the latch, unlocked for the press, is locked again')
        self.assertNotIn(('run', 1), CNC.writes)
        self.assertFalse(COOL.armed)

    # -- a live feed that did not land, or did not finish -----------------
    def test_a_live_feed_that_cannot_be_declared_cancels_before_the_run(self):
        self.declare_error = OSError('streaming=1 did not land')
        CNC.xy_steps = (0, 0)
        SW.press(delay=0.05)
        self.m._motion_locked(self._print(), None)
        self.assertTrue(self.m._running_action_cancelled)
        self.assertNotIn(('run', 1), CNC.writes)
        self.assertEqual(CNC.latch, 1, 'the latch, unlocked for the press, is locked again')
        self.assertFalse(COOL.armed)
        self._assert_feeder_stopped_before_the_park()

    def test_a_live_fed_run_that_ends_with_bytes_unfed_is_canceled(self):
        # The kernel went idle with the feeder still holding part of the
        # job: whatever played, the print did not finish, and the service
        # is never told it completed.
        CNC.xy_steps = (0, 0)
        CNC.run_reads = 3
        SW.press(delay=0.05)
        self.m._motion_locked(self._print(), None)
        self.assertTrue(self.feeder.live)
        self.assertFalse(self.feeder.finished)
        self.assertTrue(self.m._running_action_cancelled)
        self.assertIn(('run', 1), CNC.writes)
        self._assert_feeder_stopped_before_the_park()

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
        # plays out, the feed is declared live, finishes, and is stopped
        # before the park.
        self.feed_completes = True
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
        # the same posture as a finished action: the lens locked out of the
        # pulse path at its hold current
        self.assertIn(('motor_lock', '8'), CNC.writes)
        self.assertIn(('z_current', 1), CNC.writes)

    def test_cleanup_puts_the_button_out_last(self):
        # An exception in the button wait or the run loop skips the lamp
        # write that ends them: the cleanup puts out a button left breathing
        # or white, after the stop, the latch and the disarm.
        self.addCleanup(setattr, LEDS, 'pattern', None)
        self.addCleanup(setattr, machine_mod, 'set_button_color', machine_mod.set_button_color)
        seen = []
        machine_mod.set_button_color = lambda c: seen.append((c, CNC.latch, COOL.armed, COOL.mode))
        for left in ('breathe', 'white', None):
            del seen[:]
            CNC.latch = 0
            COOL.armed = True
            COOL.mode = 'run'
            LEDS.pattern = left
            self.m._action_cleanup()
            self.assertEqual(seen, [(ButtonColor.OFF, 1, False, 'idle')], left)
        # forgectrl's red flash, and a light that is not this client's, stay.
        for left in ('red', 'other', 'off'):
            del seen[:]
            LEDS.pattern = left
            self.m._action_cleanup()
            self.assertEqual(seen, [], left)

    def test_a_button_that_cannot_be_written_skips_nothing_in_the_cleanup(self):
        self.addCleanup(setattr, LEDS, 'pattern', None)
        self.addCleanup(setattr, machine_mod, 'set_button_color', machine_mod.set_button_color)

        def refuse(color):
            raise OSError(5, 'EIO')
        machine_mod.set_button_color = refuse
        LEDS.pattern = 'breathe'
        COOL.armed = True
        COOL.mode = 'run'
        self.m._feeder = FakeFeeder()
        self.m._action_cleanup()
        self.assertIn(('laser_latch', 1), CNC.writes)
        self.assertFalse(COOL.armed)
        self.assertEqual(COOL.mode, 'idle')
        self.assertIsNone(self.m._feeder)

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

    def _run(self):
        """A print's run, started as _feed_and_run starts it: the latch
        unlocked, and the press taken with it unlocked."""
        CNC.latch = 0
        CNC.hw_latch = 0
        self.m._press_live = True
        return self.m._run_loop(pausable=True, lit=True)

    def test_a_hold_pauses_and_the_resume_waits_for_a_press_with_the_latch_unlocked(self):
        # The lock that answers fire_ok=false sets the hardware's own button
        # latch: once the verdict is clean the latch is unlocked, and the job
        # resumes on a press taken from then on, not before.
        clock = MachineClock(self, self.m)
        before_press = []
        clock.at(0.05, lambda: setattr(COOL, '_fire_ok', False))
        clock.press(0.20)                       # under the lock: it does not count
        clock.at(0.40, self._release)
        clock.at(0.65, lambda: before_press.append((('resume', 1950) in CNC.writes, CNC.latch)))
        clock.press(0.80)
        clock.at(1.10, lambda: setattr(CNC, '_reads_left', 1))
        aborted = self._run()
        self.assertFalse(aborted)
        self.assertFalse(self.m._running_action_cancelled)
        w = CNC.writes
        self.assertIn(('laser_latch', 1), w)
        self.assertIn(('stop', 1), w)
        self.assertLess(w.index(('laser_latch', 1)), w.index(('stop', 1)))    # the latch at once
        self.assertIn(('resume', -2000), w)
        self.assertEqual(before_press, [(False, 0)],
                         'unlocked and waiting, not resumed, until the press')
        self.assertIn(('resume', 1950), w)
        self.assertLess(w.index(('laser_latch', 0)), w.index(('resume', 1950)))
        self.assertEqual(CNC.starts[-1], ('resume', 0, 0),
                         'the resume starts with the hardware latch cleared and the lock released')
        self.assertEqual([e for e in job_events() if e in ('print:paused', 'print:resumed')],
                         ['print:paused', 'print:resumed'])

    def test_the_wait_for_the_press_after_a_relock_has_no_clock_of_its_own(self):
        # Verdict clean and the latch unlocked: the print is where a print
        # paused on the button is, and waits as that one does. Neither the
        # button's timeout (1 s here) nor the hold's bound (0.6 s here) ends
        # it: 1.5 s after the unlock it still waits, breathing, unlocked,
        # not canceled, and the press resumes it.
        machine_mod._conf_float = lambda key, default: (
            1.0 if key == 'laser_button_timeout_s' else self._conf(key, default))
        self.m._hold_max_s = lambda: 0.6
        clock = MachineClock(self, self.m)
        del LEDS.button_colors[:]
        waiting = []
        clock.at(0.05, lambda: setattr(COOL, '_fire_ok', False))
        clock.at(0.30, self._release)
        clock.at(1.80, lambda: waiting.append(
            (self.m._running_action_cancelled, CNC.latch, CNC.hw_latch, LEDS.button_colors[-1:],
             ('resume', 1950) in CNC.writes)))
        clock.press(2.10)
        clock.at(2.40, lambda: setattr(CNC, '_reads_left', 1))
        aborted = self._run()
        self.assertEqual(waiting, [(False, 0, 1, ['breathe'], False)],
                         'past both clocks: not canceled, unlocked, breathing, not resumed')
        self.assertFalse(aborted)
        self.assertFalse(self.m._running_action_cancelled)
        self.assertEqual([s for s in CNC.starts if s[0] == 'resume'], [('resume', 0, 0)],
                         'the press resumes it, with the hardware latch cleared')

    def test_the_lid_ends_the_wait_for_the_press_after_a_relock(self):
        threading.Timer(0.05, lambda: setattr(COOL, '_fire_ok', False)).start()
        threading.Timer(0.30, self._release).start()
        SW.open_lid(delay=0.6)
        aborted = self._run()
        self.assertTrue(aborted)
        self.assertTrue(self.m._running_action_cancelled)
        self.assertNotIn(('resume', 1950), CNC.writes)
        self.assertEqual(CNC.latch, 1)

    def test_a_verdict_that_closes_again_before_the_press_locks_again(self):
        # Clean at 0.3 (unlocked, waiting), closed again at 0.5: locked
        # again and the button dark, so the press at 0.7 is taken under the
        # lock and leaves the board's latch set. Clean at 0.8, the press
        # that counts at 1.0.
        clock = MachineClock(self, self.m)
        del LEDS.button_colors[:]
        seen = []
        clock.at(0.05, lambda: setattr(COOL, '_fire_ok', False))
        clock.at(0.30, self._release)
        clock.at(0.50, lambda: (setattr(COOL, '_hold', None), setattr(COOL, '_fire_ok', False)))
        clock.at(0.65, lambda: seen.append((CNC.latch, LEDS.button_colors[-1:])))
        clock.press(0.70)
        clock.at(0.75, lambda: seen.append(CNC.hw_latch))
        clock.at(0.80, self._release)
        clock.press(1.00)
        clock.at(1.30, lambda: setattr(CNC, '_reads_left', 1))
        aborted = self._run()
        self.assertEqual(seen, [(1, [ButtonColor.OFF]), 1],
                         'locked and dark once the verdict closed again; the press then left the latch set')
        self.assertFalse(aborted)
        self.assertEqual([s for s in CNC.starts if s[0] == 'resume'], [('resume', 0, 0)])

    def test_a_verdict_that_closes_again_before_the_press_restarts_the_holds_bound(self):
        # The hold's bound is 0.6 s here. Held from 0.05, clean at 0.3 and
        # waiting for the press, closed again at 0.5 and left closed: the
        # bound runs again from 0.5, so the job is still held at 0.85 and
        # canceled before 1.4.
        self.m._hold_max_s = lambda: 0.6
        clock = MachineClock(self, self.m)
        held = []
        clock.at(0.05, lambda: setattr(COOL, '_fire_ok', False))
        clock.at(0.30, self._release)
        clock.at(0.50, lambda: (setattr(COOL, '_hold', None), setattr(COOL, '_fire_ok', False)))
        clock.at(0.85, lambda: held.append(self.m._running_action_cancelled))
        self.assertTrue(self._run())
        self.assertEqual(held, [False], 'canceled on the bound of the first hold')
        self.assertTrue(self.m._running_action_cancelled)
        self.assertLess(clock.elapsed, 1.4)

    def test_a_print_paused_on_the_button_under_a_relock_resumes_on_a_press_after_the_unlock(self):
        # Paused at 0.05; the verdict refuses fire with no hold asked from
        # 0.2 (the latch locks under the pause); a press at 0.35 is taken
        # under the lock and resumes nothing; at 0.5 fire is permitted but
        # a hold is asked, and the lock stays; clean at 0.7 (unlocked); the
        # press at 0.9 resumes. That press was taken with the latch unlocked,
        # so the hold with fire permitted from 1.1 to 1.3 resumes without
        # another; a job left waiting for one is canceled at 3.0.
        clock = MachineClock(self, self.m)
        locked = []
        clock.press(0.05)
        clock.at(0.20, lambda: (setattr(COOL, '_hold', False), setattr(COOL, '_fire_ok', False)))
        clock.press(0.35)
        clock.at(0.50, lambda: (setattr(COOL, '_hold', True), setattr(COOL, '_fire_ok', True)))
        clock.at(0.60, lambda: locked.append(CNC.latch))
        clock.at(0.70, self._release)
        clock.press(0.90)
        clock.at(1.10, lambda: setattr(COOL, '_hold', True))
        clock.at(1.30, self._release)
        clock.at(1.60, lambda: setattr(CNC, '_reads_left', 1))
        clock.at(3.00, lambda: setattr(self.m, '_running_action_cancelled', True))
        aborted = self._run()
        self.assertEqual(locked, [1], 'the lock released while a hold was still asked')
        self.assertFalse(aborted)
        self.assertEqual([s for s in CNC.starts if s[0] == 'resume'], [('resume', 0, 0)] * 2)

    def test_a_feed_hold_whose_latch_was_locked_under_it_resumes_on_a_press(self):
        self.m._feeder = FakeFeeder(written=100)
        fs, fr = machine_mod.FEED_STALL_S, machine_mod.FEED_RECOVER_S
        machine_mod.FEED_STALL_S, machine_mod.FEED_RECOVER_S = 0.1, 5.0
        self.addCleanup(setattr, machine_mod, 'FEED_STALL_S', fs)
        self.addCleanup(setattr, machine_mod, 'FEED_RECOVER_S', fr)
        clock = MachineClock(self, self.m)
        # Held for the feed from about 0.1; fire refused with no hold asked
        # from 0.3 to 0.5 (locked, then permitted); the feed moves at 0.6.
        clock.at(0.30, lambda: (setattr(COOL, '_hold', False), setattr(COOL, '_fire_ok', False)))
        clock.at(0.50, self._release)
        clock.at(0.60, lambda: setattr(self.m._feeder, 'moving', True))
        clock.press(1.00)
        clock.at(1.30, lambda: (setattr(self.m._feeder, 'finished', True), setattr(CNC, '_reads_left', 1)))
        aborted = self._run()
        self.assertFalse(aborted)
        self.assertEqual([s for s in CNC.starts if s[0] == 'resume'], [('resume', 0, 0)])

    def test_a_hold_with_fire_permitted_pauses_without_the_latch(self):
        # A hold asked with fire still permitted, as the verdict file may
        # say: the pause, not the latch.
        clock = MachineClock(self, self.m)
        clock.at(0.05, lambda: setattr(COOL, '_hold', True))
        clock.at(0.40, self._release)
        clock.at(0.70, lambda: setattr(CNC, '_reads_left', 1))
        aborted = self._run()
        self.assertFalse(aborted)
        w = CNC.writes
        self.assertIn(('resume', -2000), w)
        self.assertIn(('resume', 1950), w)
        # Never locked, so the first press still stands: no new press is
        # asked for, and the resume starts with the hardware latch cleared.
        self.assertNotIn(('laser_latch', 1), w[:w.index(('resume', 1950))])
        self.assertEqual(CNC.starts[-1], ('resume', 0, 0))

    # -- the button in the run: solid white while the job moves, breathing
    #    while a press would resume it, dark while one would not ----------
    def test_the_button_is_white_while_the_job_moves_and_breathes_while_a_press_would_resume_it(self):
        # Paused on the button at 0.1 with the latch unlocked, resumed by the
        # press at 0.5; the button read at 0.05, 0.35 and 0.75.
        del LEDS.button_colors[:]
        lamp = []
        for at in (0.05, 0.35, 0.75):
            SW._later(at, lambda: lamp.append(LEDS.button_colors[-1:]))
        SW.press(delay=0.10)
        SW.press(delay=0.50)
        SW._later(0.90, lambda: setattr(CNC, '_reads_left', 1))
        self.assertFalse(self._run())
        self.assertEqual(lamp, [[ButtonColor.WHITE], ['breathe'], [ButtonColor.WHITE]],
                         'moving, paused on the button, moving again')

    def test_the_button_is_dark_while_a_press_would_not_resume_the_job(self):
        # Each row has the job stopped by 0.3 s, reads the button at 0.45 and
        # cancels at 0.55. The two pauses breathe until 0.2, when the verdict
        # asks for a hold, or refuses fire with no hold asked, under them.
        fs, fr = machine_mod.FEED_STALL_S, machine_mod.FEED_RECOVER_S
        self.addCleanup(setattr, machine_mod, 'FEED_STALL_S', fs)
        self.addCleanup(setattr, machine_mod, 'FEED_RECOVER_S', fr)
        machine_mod.FEED_STALL_S, machine_mod.FEED_RECOVER_S = 0.1, 5.0

        def paused_under(hold, fire_ok):
            SW.press(delay=0.05)
            SW._later(0.20, lambda: (setattr(COOL, '_hold', hold), setattr(COOL, '_fire_ok', fire_ok)))
        rows = (
            ('a cooling hold', lambda: SW._later(0.05, lambda: setattr(COOL, '_fire_ok', False))),
            ('a feed hold', lambda: setattr(self.m, '_feeder', FakeFeeder(written=100))),
            ('a pause under a cooling hold', lambda: paused_under(True, True)),
            ('a pause under a set lock', lambda: paused_under(False, False)),
        )
        for name, arrange in rows:
            with self.subTest(name):
                self.tearDown()
                self.setUp()
                del LEDS.button_colors[:]
                lamp = []
                arrange()
                SW._later(0.45, lambda: lamp.append(LEDS.button_colors[-1:]))
                SW._later(0.55, lambda: setattr(self.m, '_running_action_cancelled', True))
                self.assertTrue(self._run())
                self.assertEqual(lamp, [[ButtonColor.OFF]])

    def test_the_run_puts_its_lamp_back_once_something_wrote_over_it(self):
        # forgectrl's red flash is on the button from the start, and leaves
        # it dark at 0.6: the run writes nothing over the red, and puts its
        # solid white back at its next look (once a second). The program
        # ends at 1.5.
        self.addCleanup(setattr, LEDS, 'pattern', None)
        clock = MachineClock(self, self.m)
        LEDS.pattern = 'red'
        del LEDS.button_colors[:]
        under_red = []
        clock.at(0.60, lambda: (under_red.append(LEDS.button_colors.count(ButtonColor.WHITE)),
                                setattr(LEDS, 'pattern', 'off')))
        clock.at(1.50, lambda: setattr(CNC, '_reads_left', 1))
        self.assertFalse(self._run())
        self.assertEqual(under_red, [1], 'the run start alone, nothing over the red')
        self.assertEqual(LEDS.button_colors.count(ButtonColor.WHITE), 2, 'put back once')

    def _hold_events(self):
        return [e for e in EVENTS if e.startswith('fault:') or e in ('print:paused', 'print:resumed')]

    def _hold_and_clear(self, name, reason=''):
        clock = MachineClock(self, self.m)

        def hold():
            COOL.name = name
            COOL.reason = reason
            COOL._fire_ok = False
        clock.at(0.05, hold)
        clock.at(0.40, self._release)
        clock.press(0.60)                       # the press the relock costs
        clock.at(0.90, lambda: setattr(CNC, '_reads_left', 1))
        self._run()

    def test_the_app_hears_why_a_print_is_held(self):
        for name, reason, fault in (
                ('OVERTEMP', '', 'coolant_recoverable_alert'),
                ('BUMP', '', 'head_accel_x_alert'),
                ('CRASH', '', 'head_accel_x_alert'),
                ('FLAME', '', 'lid_ir_1st_quartile_alert'),
                ('FIRE', '', 'lid_ir_1st_quartile_alert'),
                ('CRITICAL', '', 'interior_temp_alert'),
                ('AIRFLOW', 'AIRFLOW: exhaust 11640 under', 'exhaust_fan_alert'),
                ('AIRFLOW', 'AIRFLOW: intake_2 0 under', 'intake_2_fan_alert'),
                ('AIRFLOW', 'AIRFLOW: intake_1 0 under', 'intake_1_fan_alert'),
                ('AIRFLOW', 'AIRFLOW: air_assist 0 under', 'air_assist_alert')):
            with self.subTest(name=name, fault=fault):
                self.tearDown()
                self.setUp()
                self._hold_and_clear(name, reason)
                self.assertEqual(self._hold_events()[:4],
                                 ['fault:%s:set' % fault, 'print:paused',
                                  'fault:%s:reset' % fault, 'print:resumed'])

    def test_a_hold_with_no_message_of_its_own_is_a_plain_pause(self):
        for name, reason in (('SENSOR', ''), ('COLD', ''), ('EXT', ''),
                             ('AIRFLOW', 'AIRFLOW: an unnamed fan under')):
            with self.subTest(name=name, reason=reason):
                self.tearDown()
                self.setUp()
                self._hold_and_clear(name, reason)
                self.assertEqual(self._hold_events()[:2], ['print:paused', 'print:resumed'])

    def test_the_fault_the_app_hears_follows_the_verdict_that_holds(self):
        # Held on OVERTEMP from 0.05, the name CRITICAL from 0.2 (fire still
        # refused), COLD from 0.35 (no fault of its own), clean at 0.5, the
        # press at 0.7: each change resets the old fault before the new one
        # is set.
        clock = MachineClock(self, self.m)
        clock.at(0.05, lambda: (setattr(COOL, 'name', 'OVERTEMP'), setattr(COOL, '_fire_ok', False)))
        clock.at(0.20, lambda: setattr(COOL, 'name', 'CRITICAL'))
        clock.at(0.35, lambda: setattr(COOL, 'name', 'COLD'))
        clock.at(0.50, self._release)
        clock.press(0.70)
        clock.at(1.00, lambda: setattr(CNC, '_reads_left', 1))
        self.assertFalse(self._run())
        self.assertEqual(self._hold_events(),
                         ['fault:coolant_recoverable_alert:set', 'print:paused',
                          'fault:coolant_recoverable_alert:reset', 'fault:interior_temp_alert:set',
                          'fault:interior_temp_alert:reset', 'print:resumed'])

    def test_an_action_that_ends_in_an_exception_leaves_no_fault_set(self):
        # The action fails while a fault is up: motion() resets it on the
        # way out.
        self.m._q_msg_tx = queue.Queue()        # the base class's own events

        def held_then_failed(msg):
            self.m._fault_report({'verdict': 'OVERTEMP'})
            raise RuntimeError('the job failed while held')
        self.m._motion = held_then_failed
        with self.assertRaises(RuntimeError):
            self.m.motion({'id': 42, 'action_type': 'print', 'motion_url': 'x'})
        self.assertEqual(self._hold_events(), ['fault:coolant_recoverable_alert:set',
                                               'fault:coolant_recoverable_alert:reset'])
        self.assertIsNone(self.m._fault_up)

    def test_no_fault_stays_set_at_the_service(self):
        # Canceled while held: the record at the service must not keep it.
        def hold():
            COOL.name = 'OVERTEMP'
            COOL._fire_ok = False
        threading.Timer(0.05, hold).start()
        threading.Timer(0.40, lambda: setattr(self.m, '_running_action_cancelled', True)).start()
        self._run()
        events = self._hold_events()
        self.assertIn('fault:coolant_recoverable_alert:set', events)
        self.assertIn('fault:coolant_recoverable_alert:reset', events)
        self.assertIsNone(self.m._fault_up)

    def test_every_start_resets_every_hold_fault_in_the_map(self):
        # A client stopped mid-hold (the fail tiers restart it) could not
        # reset its own fault: the next start resets every fault in the map,
        # and sets none.
        self.m.start(None, queue.Queue())
        faults = set(machine_mod.HOLD_FAULTS.values()) | {f for _, f in machine_mod.AIRFLOW_FAULTS}
        self.assertEqual({e for e in EVENTS if e.startswith('fault:')},
                         {'fault:%s:reset' % f for f in faults})

    def test_a_hold_past_its_bound_cancels(self):
        # the bound is clamped to 60 s and up: patch the reader the loop uses
        self.m._hold_max_s = lambda: 0.3
        threading.Timer(0.05, lambda: setattr(COOL, '_fire_ok', False)).start()
        aborted = self._run()
        self.assertTrue(aborted)
        self.assertTrue(self.m._running_action_cancelled)
        self.assertNotIn(('resume', 1950), CNC.writes)
        self.assertNotIn(('laser_latch', 0), CNC.writes)     # a hold that ends in a cancel never unlocks

    def test_the_lid_ends_a_verdict_hold(self):
        threading.Timer(0.05, lambda: setattr(COOL, '_fire_ok', False)).start()
        SW.open_lid(delay=0.4)
        aborted = self._run()
        self.assertTrue(aborted)
        self.assertTrue(self.m._running_action_cancelled)
        self.assertEqual(CNC.writes.count(('stop', 1)), 1)     # the hold's stop, no second

    def test_a_lost_verdict_holds_and_falls_back(self):
        clock = MachineClock(self, self.m)
        clock.at(0.05, lambda: setattr(COOL, 'stale', True))
        clock.at(0.40, lambda: setattr(COOL, 'stale', False))
        clock.press(0.60)                       # the press the relock costs
        clock.at(0.90, lambda: setattr(CNC, '_reads_left', 1))
        aborted = self._run()
        self.assertFalse(aborted)
        self.assertEqual(self.heater_offs, 1)
        self.assertEqual(COOL.fallbacks, 1)
        w = CNC.writes
        self.assertIn(('laser_latch', 1), w)
        self.assertIn(('resume', -2000), w)
        self.assertIn(('laser_latch', 0), w)
        self.assertIn(('resume', 1950), w)
        self.assertEqual(CNC.starts[-1], ('resume', 0, 0))

    def test_a_verdict_without_fire_ok_locks_the_latch(self):
        # An absent fire_ok reads as false: the latch locks, though
        # nothing in the document asks for a hold. Mid-run:
        threading.Timer(0.05, lambda: setattr(COOL, 'absent', ('fire_ok',))).start()
        threading.Timer(0.40, lambda: setattr(CNC, '_reads_left', 1)).start()
        self._run()
        self.assertIn(('laser_latch', 1), CNC.writes)
        self.assertNotIn(('laser_latch', 0), CNC.writes)

    def test_a_run_does_not_start_on_a_verdict_that_refuses_fire_or_without_the_press(self):
        # The last look before the run: no start under a verdict that
        # refuses fire or is missing, and none without a press taken with
        # the latch unlocked. Each way the latch ends locked and the job
        # canceled. On machine time, a run that did start ends at the
        # clock's horizon.
        for case in ('verdict', 'missing', 'press'):
            with self.subTest(case=case):
                self.tearDown()
                self.setUp()
                MachineClock(self, self.m)
                CNC.latch = 0
                CNC.hw_latch = 0
                self.m._press_live = case != 'press'
                if case == 'verdict':
                    COOL.absent = ('fire_ok',)
                elif case == 'missing':
                    COOL.stale = True
                aborted = self.m._run_loop(pausable=True, lit=True)
                self.assertTrue(aborted)
                self.assertTrue(self.m._running_action_cancelled)
                self.assertNotIn(('run', 1), CNC.writes)
                self.assertEqual(CNC.latch, 1)
                self.assertFalse(COOL.armed)

    @staticmethod
    def _end_the_program():
        # Every forward run from here ends on its next state read.
        CNC.run_reads = 1
        CNC._reads_left = 1

    def test_a_hold_lifts_only_on_hold_false_and_resume_ok(self):
        # An absent hold reads as a hold, and an absent resume_ok as no
        # resume. With either key absent, the engine's release lifts
        # nothing: the job pauses, stays held, and the bound cancels it.
        resume = CNC.resume

        def retrace_then_release(steps):
            resume(steps)
            if steps < 0:
                COOL._hold = False                  # the engine lifts its hold
        try:
            for key in ('hold', 'resume_ok'):
                with self.subTest(absent=key):
                    CNC.__init__()
                    CNC.resume = retrace_then_release
                    COOL.__init__()
                    COOL.armed = True
                    COOL.absent = (key,)
                    COOL._hold = True               # the engine holds the job
                    self.m = make_machine()
                    SW.handler = self.m._switch_event
                    self.m._hold_max_s = lambda: 0.4
                    # A job that never holds, or that resumes, ends here
                    # instead of playing on for good.
                    end = threading.Timer(2.0, self._end_the_program)
                    end.start()
                    try:
                        aborted = self._run()
                    finally:
                        end.cancel()
                    self.assertTrue(aborted)
                    self.assertTrue(self.m._running_action_cancelled)
                    self.assertIn(('resume', -2000), CNC.writes)       # held and retraced
                    self.assertNotIn(('resume', 1950), CNC.writes)     # never resumed
        finally:
            vars(CNC).pop('resume', None)

    def test_the_button_does_not_resume_under_a_hold(self):
        clock = MachineClock(self, self.m)
        clock.press(0.05)                            # pause
        clock.at(0.20, lambda: setattr(COOL, '_hold', True))
        clock.press(0.40)                            # a resume the engine forbids
        clock.at(0.70, self._release)
        clock.at(1.00, lambda: setattr(CNC, '_reads_left', 1))
        aborted = self._run()
        self.assertFalse(aborted)
        w = CNC.writes
        self.assertEqual(w.count(('resume', 1950)), 1)
        self.assertEqual(w.count(('resume', -2000)), 1)
        # The hold never locked the latch, so the press that paused the job
        # was the last one needed: the resume starts with it cleared.
        self.assertEqual(CNC.starts[-1], ('resume', 0, 0))

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
        COOL.armed = True
        COOL._fire_ok = False
        COOL.name = 'WARMUP'
        threading.Timer(0.3, lambda: (setattr(COOL, '_fire_ok', True), setattr(COOL, 'name', 'OK'))).start()
        t0 = time.monotonic()
        self.m._verdict_wait()
        self.assertGreater(time.monotonic() - t0, 0.25)
        self.assertFalse(self.m._running_action_cancelled)
        # The latch is nobody's business during the hold: locked from
        # before the button, unlocked only once the run starts.
        self.assertNotIn(('laser_latch', 0), CNC.writes)
        self.assertNotIn(('laser_latch', 1), CNC.writes)
        # A hold with no fault of its own tells the app nothing.
        self.assertEqual([e for e in EVENTS if e.startswith(('print:', 'fault:'))], [])

    def test_a_clean_verdict_does_not_wait(self):
        COOL.armed = True
        t0 = time.monotonic()
        self.m._verdict_wait()
        self.assertLess(time.monotonic() - t0, 0.1)

    def test_an_unacknowledged_arm_does_not_start_the_run(self):
        """The verdict must answer this armed session, not the idle one
        before it. Everything here reads clean - fire_ok true, no hold -
        and the only thing missing is the engine's acknowledgment that it
        took the job and put the fans on the run profile. Starting on
        that verdict is what puts the beam on the work with idle
        airflow, so the wait must hold and then refuse."""
        self.m._hold_max_s = lambda: 0.3
        COOL.armed = True
        COOL.ack = False
        t0 = time.monotonic()
        self.m._verdict_wait()
        self.assertGreater(time.monotonic() - t0, 0.25)
        self.assertTrue(self.m._running_action_cancelled)
        self.assertIn(('laser_latch', 1), CNC.writes)
        self.assertFalse(COOL.armed)

    def test_the_wait_ends_when_the_engine_takes_the_job(self):
        """The acknowledgment arriving late is the ordinary case: the
        engine sees the armed report on its next tick. The run starts
        then, and not before."""
        COOL.armed = True
        COOL.ack = False
        threading.Timer(0.3, lambda: setattr(COOL, 'ack', True)).start()
        t0 = time.monotonic()
        self.m._verdict_wait()
        self.assertGreater(time.monotonic() - t0, 0.25)
        self.assertFalse(self.m._running_action_cancelled)
        self.assertNotIn(('laser_latch', 1), CNC.writes)

    def test_the_lid_ends_the_wait(self):
        COOL._fire_ok = False
        COOL.armed = True
        SW.open_lid(delay=0.05)
        self.m._verdict_wait()
        self.assertTrue(self.m._running_action_cancelled)
        self.assertIn(('laser_latch', 1), CNC.writes)
        self.assertFalse(COOL.armed)

    def test_a_lost_verdict_ends_the_wait(self):
        # The first wait gives a missing verdict no grace: gone at 0.1, it
        # ends the wait at the next pass.
        clock = MachineClock(self, self.m)
        COOL._fire_ok = False
        COOL.armed = True
        clock.at(0.1, lambda: setattr(COOL, 'stale', True))
        self.m._verdict_wait()
        self.assertTrue(self.m._running_action_cancelled)
        self.assertIn(('laser_latch', 1), CNC.writes)
        self.assertLess(clock.elapsed, 0.1 + 0.6)

    def test_the_bound_ends_the_wait(self):
        self.m._hold_max_s = lambda: 0.2
        COOL._fire_ok = False
        COOL.armed = True
        self.m._verdict_wait()
        self.assertTrue(self.m._running_action_cancelled)
        self.assertIn(('laser_latch', 1), CNC.writes)

    def test_a_verdict_without_hold_fire_ok_or_armed_does_not_start_the_run(self):
        # An absent hold reads as a hold, and an absent fire_ok or armed
        # as false. The rest of the document reads clean, and the wait
        # still runs out its bound and cancels the job with the latch
        # locked.
        for key in ('hold', 'fire_ok', 'armed'):
            with self.subTest(absent=key):
                self.setUp()
                self.m._hold_max_s = lambda: 0.2
                COOL.armed = True
                COOL.absent = (key,)
                self.m._verdict_wait()
                self.assertTrue(self.m._running_action_cancelled)
                self.assertIn(('laser_latch', 1), CNC.writes)
                self.assertFalse(COOL.armed)

    def test_a_print_canceled_in_the_wait_is_never_told_resumed(self):
        COOL.armed = True
        COOL._fire_ok = False
        COOL.name = 'OVERTEMP'
        self.m._hold_max_s = lambda: 0.3
        self.m._verdict_wait()
        self.assertTrue(self.m._running_action_cancelled)
        held = [e for e in EVENTS if e.startswith('fault:') or e in ('print:paused', 'print:resumed')]
        self.assertEqual(held, ['fault:coolant_recoverable_alert:set', 'print:paused',
                                'fault:coolant_recoverable_alert:reset'])

    def test_an_unacknowledged_fault_ends_the_wait_at_once(self):
        self.m._hold_max_s = lambda: 1.0       # a wait that sat it out ends in a second
        COOL._fire_ok = False
        COOL.armed = True
        COOL.name = 'ACK'
        t0 = time.monotonic()
        self.m._verdict_wait()
        self.assertLess(time.monotonic() - t0, 0.2)
        self.assertTrue(self.m._running_action_cancelled)
        self.assertIn(('laser_latch', 1), CNC.writes)
        self.assertFalse(COOL.armed)

    def test_an_acknowledge_waiting_under_another_verdict_ends_the_wait_at_once(self):
        # The same under a name that is not ACK: the hold is not waited out.
        self.m._hold_max_s = lambda: 2.0
        COOL._fire_ok = False
        COOL.armed = True
        COOL.name = 'OVERTEMP'
        COOL.ack_wait = True
        t0 = time.monotonic()
        self.m._verdict_wait()
        self.assertLess(time.monotonic() - t0, 0.2)
        self.assertTrue(self.m._running_action_cancelled)
        self.assertIn(('laser_latch', 1), CNC.writes)
        self.assertFalse(COOL.armed)

    def test_a_hot_loop_after_the_press_tells_the_app(self):
        COOL.armed = True
        COOL._fire_ok = False
        COOL.name = 'OVERTEMP'
        timer = threading.Timer(0.3, lambda: (setattr(COOL, '_fire_ok', True), setattr(COOL, 'name', 'OK')))
        timer.start()
        self.addCleanup(timer.cancel)
        self.m._verdict_wait()
        self.assertFalse(self.m._running_action_cancelled)
        self.assertEqual([e for e in EVENTS if e.startswith('fault:') or e in ('print:paused', 'print:resumed')],
                         ['fault:coolant_recoverable_alert:set', 'print:paused',
                          'fault:coolant_recoverable_alert:reset', 'print:resumed'])


if __name__ == '__main__':
    unittest.main()

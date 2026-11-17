"""
Copyright 2026 514 LLC d/b/a OpenGlow
Written by Scott Wiederhold
https://community.openglow.org

SPDX-License-Identifier:    MIT

Host tests for the way the one-shot web-service homing runner ends. Every
exit - a homed run with its lens moves, a failed lens move, an exit raised
inside one, SIGTERM from the controller that spawned it - has to stop the
machine (laser latched, device released) and then shut the service socket:
the runner shares the pulse device with the controller, and a runner left
running past its controller is a second writer on the ring. SIGTERM stops
the machine from the handler, ends every phase within its poll, and stops
a lens move at its next step; the session's deadline is the controller's
own.

Run:  PYTHONPATH=.:forgefirm-app:../Glowforge-Utilities python3 -m unittest tests.test_gfhome_exit
"""
import contextlib
import importlib.util
import os
import signal
import sys
import threading
import time
import types
import unittest
from unittest import mock

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'forgefirm-app'))
sys.path.insert(0, os.path.join(os.path.dirname(ROOT), 'Glowforge-Utilities'))

import gfhome                                                     # noqa: E402


ORDER = []          # the shutdown's steps, in the order they ran


class FakeWS:
    def __init__(self):
        self.shut = 0

    def shutdown(self):
        self.shut += 1
        ORDER.append('socket')


class FakeMachine:
    def __init__(self):
        self.stops = 0
        self.stopped_now = 0
        self.started = False
        self.running_action_id = None
        self._sw_thread = self
        self.lid_closed = True
        del ORDER[:]

    def start(self, session, q_tx):
        self.started = True

    def stop(self):
        self.stops += 1
        ORDER.append('stop')

    def stop_now(self):
        self.stopped_now += 1

    def all_switches(self):
        return {InputSwitch.SW_DOORS: self.lid_closed}


class InputSwitch:
    SW_DOORS = 'doors'


class LensStopped(RuntimeError):
    pass


def lens_modules(home):
    """gfhardware's lens axis, standing in for one call: home() does what
    the test says (a raise included)."""
    pkg = sys.modules.get('gfhardware') or types.ModuleType('gfhardware')
    common = types.ModuleType('gfhardware._common')
    common.InputSwitch = InputSwitch
    z = types.ModuleType('gfhardware.z_axis')

    class ZAxis:
        @staticmethod
        def home(stop=None):
            home()
    z.ZAxis = ZAxis
    z.LensStopped = LensStopped
    return mock.patch.dict(sys.modules, {'gfhardware': pkg, 'gfhardware._common': common,
                                         'gfhardware.z_axis': z})


def real_lens_modules():
    """gfhardware's real lens axis (z_axis.py over the real _common.py) under
    a bare package, which would otherwise import the whole machine: the
    patch to enter around a call, and the lens module, whose sysfs reads
    and writes and whose step delay the case takes over."""
    mods = {'gfhardware': types.ModuleType('gfhardware')}
    for name in ('_common', 'z_axis'):
        spec = importlib.util.spec_from_file_location(
            'gfhardware.' + name, os.path.join(ROOT, 'gfhardware', name + '.py'))
        mods['gfhardware.' + name] = importlib.util.module_from_spec(spec)
    patch = mock.patch.dict(sys.modules, mods)
    with patch:
        for name in ('_common', 'z_axis'):
            mod = mods['gfhardware.' + name]
            mod.__spec__.loader.exec_module(mod)
    return patch, mods['gfhardware.z_axis']


class FinishTests(unittest.TestCase):
    def setUp(self):
        self.ws = FakeWS()
        self.machine = FakeMachine()
        self.parks = []
        self._park = gfhome.ffmachine.park_lens
        gfhome.ffmachine.park_lens = lambda half_steps, stop=None: self.parks.append(half_steps)

    def tearDown(self):
        gfhome.ffmachine.park_lens = self._park

    def test_a_homed_run_moves_the_lens_then_shuts_down(self):
        moved = []
        with lens_modules(lambda: moved.append('home')):
            rc = gfhome.finish(self.machine, self.ws, 0)
        self.assertEqual(rc, 0)
        self.assertEqual(moved, ['home'])
        self.assertEqual(self.parks, [0])
        # The machine stops before the socket's close, which can wait on
        # the service for seconds.
        self.assertEqual(ORDER, ['stop', 'socket'])

    def real_lens(self, stop_after=None):
        """The real lens axis over a model lens: the hall sensor reads the
        edge from 3 steps up, a z_step write moves the lens one step ('1'
        up), and SIGTERM lands in the delay of step `stop_after`. A whole
        reference is 13 steps: 3 up to the edge, then five passes of one
        step off it and one back."""
        patch, z = real_lens_modules()
        self.writes, at = [], [0]

        def write(attr, val, binary=False):
            attr = attr.rsplit('/', 1)[-1]
            self.writes.append((attr, val))
            if attr == 'z_step':
                at[0] += 1 if val == '1' else -1
        z.write_file = write
        z.read_file = lambda attr, binary=False: '0' if at[0] >= 3 else '1'

        def step_delay(seconds):
            if self.steps() == stop_after:
                gfhome._stop.set()
        z.sleep = step_delay
        self.addCleanup(gfhome._stop.clear)
        return patch

    def steps(self):
        return sum(1 for attr, _ in self.writes if attr == 'z_step')

    def test_a_sigterm_during_the_reference_stops_the_lens_and_skips_the_park(self):
        # SIGTERM after the sixth step, in the second pass: the next step
        # never comes, the driver rests at its hold current in half-step
        # mode, and the park is not called.
        with self.real_lens(stop_after=6):
            rc = gfhome.finish(self.machine, self.ws, 0)
        self.assertEqual(rc, 2)
        self.assertEqual(self.steps(), 6)
        self.assertEqual(self.writes[-2:], [('z_current', '1'), ('z_mode', '1')])
        self.assertEqual(self.parks, [])
        self.assertEqual(ORDER, ['stop', 'socket'])

    def test_a_sigterm_during_the_park_stops_it_at_its_next_step(self):
        # The real park of 5 half-steps after the whole reference: SIGTERM
        # after its second step ends it there, the driver rested.
        gfhome.ffmachine.park_lens = self._park
        with self.real_lens(stop_after=15), \
                mock.patch.dict(os.environ, {'GFHOME_PARK_HALF_STEPS': '5'}):
            rc = gfhome.finish(self.machine, self.ws, 0)
        self.assertEqual(rc, 2)
        self.assertEqual(self.steps(), 15)
        self.assertEqual(self.writes[-2:], [('z_current', '1'), ('z_mode', '1')])
        self.assertEqual(ORDER, ['stop', 'socket'])

    def test_a_sigterm_before_the_lens_moves_moves_it_not_at_all(self):
        gfhome._stop.set()
        with self.real_lens():
            rc = gfhome.finish(self.machine, self.ws, 0)
        self.assertEqual(rc, 2)
        self.assertEqual(self.steps(), 0)
        self.assertEqual(self.writes[-2:], [('z_current', '1'), ('z_mode', '1')])
        self.assertEqual(self.parks, [])

    def test_an_exit_raised_inside_the_lens_move_still_stops_the_machine(self):
        # A SystemExit landing inside the lens move is not an Exception; the
        # stop that locks the laser and releases the device runs all the
        # same.
        def leave():
            raise SystemExit(2)
        with lens_modules(leave):
            rc = gfhome.finish(self.machine, self.ws, 0)
        self.assertEqual(rc, 2)
        self.assertEqual((self.ws.shut, self.machine.stops), (1, 1))

    def test_a_failed_park_still_stops_the_machine(self):
        def broken(half_steps, stop=None):
            raise RuntimeError('lens stall')
        gfhome.ffmachine.park_lens = broken
        with lens_modules(lambda: None):
            rc = gfhome.finish(self.machine, self.ws, 0)
        self.assertEqual(rc, 2)
        self.assertEqual((self.ws.shut, self.machine.stops), (1, 1))

    def test_a_run_that_did_not_home_moves_no_lens(self):
        moved = []
        with lens_modules(lambda: moved.append('home')):
            rc = gfhome.finish(self.machine, self.ws, 2)
        self.assertEqual(rc, 2)
        self.assertEqual(moved, [])
        self.assertEqual((self.ws.shut, self.machine.stops), (1, 1))

    def test_a_machine_that_will_not_stop_is_still_shut_down_around(self):
        self.machine.stop = lambda: (_ for _ in ()).throw(SystemExit(3))
        with lens_modules(lambda: None):
            rc = gfhome.finish(self.machine, self.ws, 2)
        self.assertEqual(rc, 2)
        self.assertEqual(self.ws.shut, 1)


class SigtermTests(unittest.TestCase):
    """The handler stops the machine itself and sets a flag that every
    phase of the session reads; the session leaves through the same
    shutdown as every other exit."""

    def setUp(self):
        self.ws = FakeWS()
        self.machine = FakeMachine()
        self.release = threading.Event()
        self._session = gfhome.get_session
        self._auth = gfhome.authenticate_machine
        self._connect = gfhome.ws_connect
        gfhome.get_session = lambda: object()
        gfhome.authenticate_machine = lambda s: True
        gfhome.ws_connect = lambda q_rx, q_tx, session=None: self.ws
        gfhome._stop.clear()

    def tearDown(self):
        self.release.set()
        gfhome.get_session = self._session
        gfhome.authenticate_machine = self._auth
        gfhome.ws_connect = self._connect
        gfhome._stop.clear()
        gfhome._machine = None

    def args(self, deadline_s=60):
        return types.SimpleNamespace(timeout=60, start_timeout=60, quiet=10,
                                     deadline=time.monotonic() + deadline_s)

    def test_the_handler_stops_the_machine_at_once(self):
        gfhome._machine = self.machine
        gfhome._on_sigterm(15, None)
        self.assertTrue(gfhome._stop.is_set())
        self.assertEqual(self.machine.stopped_now, 1)

    def test_a_sigterm_during_the_sign_in_ends_the_session_within_a_second(self):
        def sign_in(session):
            gfhome._stop.set()
            self.release.wait(30)               # a sign-in that hangs
            return True
        gfhome.authenticate_machine = sign_in
        t0 = time.monotonic()
        rc = gfhome.home(self.machine, self.args())
        self.assertEqual(rc, 2)
        self.assertLess(time.monotonic() - t0, 1.0)
        self.assertFalse(self.machine.started)

    def test_a_sign_in_that_hangs_ends_at_the_deadline(self):
        gfhome.authenticate_machine = lambda session: self.release.wait(30)
        t0 = time.monotonic()
        rc = gfhome.home(self.machine, self.args(deadline_s=0.3))
        self.assertEqual(rc, 2)
        self.assertLess(time.monotonic() - t0, 1.5)
        self.assertFalse(self.machine.started)

    def test_a_sigterm_inside_the_handler_returns_at_once(self):
        # A second SIGTERM lands while the handler's _stop.set() holds its
        # lock: the nested handler returns at once, and the first one stops
        # the machine.
        class SecondSignal(threading.Event):
            sent = False

            def set(self):
                with self._cond:
                    if not SecondSignal.sent:
                        SecondSignal.sent = True
                        signal.raise_signal(signal.SIGTERM)
                    self._flag = True
                    self._cond.notify_all()
        self.addCleanup(setattr, gfhome, '_stop', gfhome._stop)
        gfhome._stop = SecondSignal()
        gfhome._machine = self.machine
        previous = signal.signal(signal.SIGTERM, gfhome._on_sigterm)
        self.addCleanup(signal.signal, signal.SIGTERM, previous)

        class Blocked(BaseException):
            """Not an Exception, so the handler's own guard cannot swallow it."""

        def blocked(*_):
            raise Blocked('the nested handler blocked on the lock _stop.set() holds')
        alarm = signal.signal(signal.SIGALRM, blocked)
        self.addCleanup(signal.signal, signal.SIGALRM, alarm)
        signal.setitimer(signal.ITIMER_REAL, 2.0)
        try:
            signal.raise_signal(signal.SIGTERM)
        finally:
            signal.setitimer(signal.ITIMER_REAL, 0)
        self.assertTrue(SecondSignal.sent)
        self.assertTrue(gfhome._stop.is_set())
        self.assertEqual(self.machine.stopped_now, 1)

    def test_main_runs_a_homing_session_the_handler_can_stop(self):
        machine, seen, handlers = FakeMachine(), {}, {}

        def home(m, args):
            seen.update(machine=m, session=getattr(m, 'homing_session', False),
                        handler=gfhome._machine)
            return 2
        quiet = lambda *a, **k: None                    # noqa: E731
        with contextlib.ExitStack() as stack:
            for name in ('setup_logging', 'apply_identity_overrides', 'apply_user_agent',
                         'acknowledge_hunts'):
                stack.enter_context(mock.patch.object(gfhome.ffmachine, name, quiet))
            stack.enter_context(mock.patch.object(gfhome.ffmachine, 'build_machine', lambda: machine))
            stack.enter_context(mock.patch.object(gfhome, 'load_config', lambda path: True))
            stack.enter_context(mock.patch.object(gfhome, 'home', home))
            stack.enter_context(mock.patch.object(gfhome.signal, 'signal', handlers.__setitem__))
            stack.enter_context(mock.patch.object(sys, 'argv', ['gfhome']))
            rc = gfhome.main()
        self.assertEqual(rc, 2)
        self.assertIs(handlers.get(signal.SIGTERM), gfhome._on_sigterm)
        self.assertIs(seen['machine'], machine)
        self.assertTrue(seen['session'])
        self.assertIs(seen['handler'], machine)

    def test_a_sigterm_in_the_session_ends_it_through_the_shutdown(self):
        def start(session, q_tx):
            self.machine.started = True
            gfhome._stop.set()
        self.machine.start = start
        t0 = time.monotonic()
        with lens_modules(lambda: None):
            rc = gfhome.home(self.machine, self.args(deadline_s=5))
        self.assertEqual(rc, 2)
        self.assertLess(time.monotonic() - t0, 1.0)
        self.assertEqual(ORDER, ['stop', 'socket'])


class DeadlineTests(unittest.TestCase):
    """The session's end comes from the controller, on the clock it shares."""

    def test_the_controllers_deadline_stands_as_handed_over(self):
        # A budget of a few seconds is not floored: the controller's SIGTERM
        # comes at its own deadline, which the runner must not outlast.
        with mock.patch.dict(os.environ, {'GFHOME_DEADLINE': '1234.5'}):
            self.assertEqual(gfhome._deadline(240), 1234.5)

    def test_a_deadline_that_is_not_a_finite_number_counts_from_now(self):
        for value in ('nan', 'inf'):
            with mock.patch.dict(os.environ, {'GFHOME_DEADLINE': value}):
                self.assertAlmostEqual(gfhome._deadline(240), time.monotonic() + 240, delta=1.0)

    def test_run_by_hand_the_budget_counts_from_now(self):
        with mock.patch.dict(os.environ, {}, clear=False):
            os.environ.pop('GFHOME_DEADLINE', None)
            t = time.monotonic()
            self.assertAlmostEqual(gfhome._deadline(240), t + 240, delta=1.0)


if __name__ == '__main__':
    unittest.main()

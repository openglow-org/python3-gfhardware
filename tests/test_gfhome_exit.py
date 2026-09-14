"""
Copyright 2026 514 LLC d/b/a OpenGlow
Written by Scott Wiederhold
https://community.openglow.org

SPDX-License-Identifier:    MIT

Host tests for the way the one-shot web-service homing runner ends. Every
exit - a homed run with its lens moves, a failed lens move, an exit raised
inside one, SIGTERM from the controller that spawned it - has to shut the
service socket and stop the machine (laser latched, device released): the
runner shares the pulse device with the controller, and a runner left
running past its controller is a second writer on the ring.

Run:  PYTHONPATH=.:forgefirm-app:../Glowforge-Utilities python3 -m unittest tests.test_gfhome_exit
"""
import os
import sys
import types
import unittest
from unittest import mock

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'forgefirm-app'))
sys.path.insert(0, os.path.join(os.path.dirname(ROOT), 'Glowforge-Utilities'))

import gfhome                                                     # noqa: E402


class FakeWS:
    def __init__(self):
        self.shut = 0

    def shutdown(self):
        self.shut += 1


class FakeMachine:
    def __init__(self):
        self.stops = 0
        self.started = False
        self.running_action_id = None
        self._sw_thread = self

    def start(self, session, q_tx):
        self.started = True

    def stop(self):
        self.stops += 1

    def all_switches(self):
        return {InputSwitch.SW_DOORS: True}


class InputSwitch:
    SW_DOORS = 'doors'


def lens_modules(home):
    """gfhardware's lens axis, standing in for one call: home() does what
    the test says (a raise included)."""
    pkg = sys.modules.get('gfhardware') or types.ModuleType('gfhardware')
    common = types.ModuleType('gfhardware._common')
    common.InputSwitch = InputSwitch
    z = types.ModuleType('gfhardware.z_axis')

    class ZAxis:
        @staticmethod
        def home():
            home()
    z.ZAxis = ZAxis
    return mock.patch.dict(sys.modules, {'gfhardware': pkg, 'gfhardware._common': common,
                                         'gfhardware.z_axis': z})


class FinishTests(unittest.TestCase):
    def setUp(self):
        self.ws = FakeWS()
        self.machine = FakeMachine()
        self.parks = []
        self._park = gfhome.ffmachine.park_lens
        gfhome.ffmachine.park_lens = lambda half_steps: self.parks.append(half_steps)

    def tearDown(self):
        gfhome.ffmachine.park_lens = self._park

    def test_a_homed_run_moves_the_lens_then_shuts_down(self):
        moved = []
        with lens_modules(lambda: moved.append('home')):
            rc = gfhome.finish(self.machine, self.ws, 0)
        self.assertEqual(rc, 0)
        self.assertEqual(moved, ['home'])
        self.assertEqual(self.parks, [0])
        self.assertEqual((self.ws.shut, self.machine.stops), (1, 1))

    def test_an_exit_raised_inside_the_lens_move_still_stops_the_machine(self):
        # SIGTERM used to be sys.exit(2) from the handler: a SystemExit
        # landing inside the lens move is not an Exception, and the stop
        # that locks the laser and releases the device was skipped.
        def leave():
            raise SystemExit(2)
        with lens_modules(leave):
            rc = gfhome.finish(self.machine, self.ws, 0)
        self.assertEqual(rc, 2)
        self.assertEqual((self.ws.shut, self.machine.stops), (1, 1))

    def test_a_failed_park_still_stops_the_machine(self):
        def broken(half_steps):
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
    """The handler sets a flag; the session loop sees it within its poll
    and leaves through the same shutdown as every other exit."""

    def setUp(self):
        self.ws = FakeWS()
        self.machine = FakeMachine()
        self._session = gfhome.get_session
        self._auth = gfhome.authenticate_machine
        self._connect = gfhome.ws_connect
        gfhome.get_session = lambda: object()
        gfhome.authenticate_machine = lambda s: True
        gfhome.ws_connect = lambda q_rx, q_tx: self.ws
        gfhome._stop.clear()

    def tearDown(self):
        gfhome.get_session = self._session
        gfhome.authenticate_machine = self._auth
        gfhome.ws_connect = self._connect
        gfhome._stop.clear()

    def test_a_sigterm_ends_the_session_through_the_shutdown(self):
        args = types.SimpleNamespace(timeout=60, start_timeout=60, quiet=10)
        gfhome._stop.set()
        with lens_modules(lambda: None):
            rc = gfhome.home(self.machine, args)
        self.assertEqual(rc, 2)
        self.assertTrue(self.machine.started)
        self.assertEqual((self.ws.shut, self.machine.stops), (1, 1))


if __name__ == '__main__':
    unittest.main()

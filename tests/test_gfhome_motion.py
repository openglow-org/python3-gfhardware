"""
Copyright 2026 514 LLC d/b/a OpenGlow
Written by Scott Wiederhold
https://community.openglow.org

SPDX-License-Identifier:    MIT

Host tests for what the web-service homing runner counts as homed. The
service ends its sequence silently, so a motion that did not run whole
(cancelled, stopped short, crashed before its end) is followed by the same
quiet as a finished one; the head is short of the home, and the runner
must fail the session at that motion's end instead of declaring it homed.

Run:  PYTHONPATH=.:forgefirm-app:../Glowforge-Utilities python3 -m unittest tests.test_gfhome_motion
"""
import json
import os
import sys
import types
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from test_gfhome_exit import FakeMachine, FakeWS, gfhome, lens_modules    # noqa: E402

STATS = {'XEND': -13096, 'YEND': -7399}


class Witness:
    """The head accelerometer, having seen the gantry move."""
    motion_windows = 3

    def __init__(self):
        self.stop = False

    def start(self):
        pass


class SessionTests(unittest.TestCase):
    """One hunt, then one motion that ends the way the test says; the
    service then goes quiet."""

    def setUp(self):
        self.ws = FakeWS()
        self.machine = FakeMachine()
        self.machine._running_action_cancelled = False
        self.machine._motion_stats = {}
        self.ends = None
        saved = {k: getattr(gfhome, k) for k in ('get_session', 'authenticate_machine', 'ws_connect',
                                                 'dispatch_action', '_AccelWatch')}
        self.addCleanup(lambda: [setattr(gfhome, k, v) for k, v in saved.items()])
        park = gfhome.ffmachine.park_lens
        self.addCleanup(setattr, gfhome.ffmachine, 'park_lens', park)
        gfhome.ffmachine.park_lens = lambda half_steps: None
        gfhome.get_session = lambda: object()
        gfhome.authenticate_machine = lambda s: True
        gfhome._AccelWatch = Witness

        def connect(q_rx, q_tx):
            for action in ('hunt', 'motion'):
                q_rx.put(json.dumps({'action_type': action, 'status': 'ready'}))
            return self.ws
        gfhome.ws_connect = connect

        def dispatch(machine, msg, allow_print):
            self.assertFalse(allow_print)
            if msg['action_type'] == 'motion':
                self.ends(machine)
            return msg['action_type']
        gfhome.dispatch_action = dispatch
        gfhome._stop.clear()

    def home(self):
        args = types.SimpleNamespace(timeout=30, start_timeout=30, quiet=0)
        with lens_modules(lambda: None):
            with self.assertLogs('openglow', level='INFO') as logs:
                rc = gfhome.home(self.machine, args)
        return rc, [r.getMessage() for r in logs.records]

    def test_a_motion_that_ran_whole_homes(self):
        # The negative control: the same session, the motion finished.
        def whole(machine):
            machine._motion_stats = {'stats': dict(STATS)}
        self.ends = whole
        rc, said = self.home()
        self.assertEqual(rc, 0)
        self.assertTrue(any(m.startswith('homing complete') for m in said), said)

    def test_a_cancelled_motion_is_not_homed(self):
        # The run stopped short (the cooling engine's stop, a fault): the
        # client records the counters it reached and ends ':cancelled'.
        def short(machine):
            machine._motion_stats = {'stats': {'XEND': 4390, 'YEND': 2682}}
            machine._running_action_cancelled = True
        self.ends = short
        rc, said = self.home()
        self.assertEqual(rc, 2)
        self.assertTrue(any('did not run whole (cancelled)' in m for m in said), said)
        self.assertFalse(any(m.startswith('homing complete') for m in said), said)
        self.assertEqual((self.ws.shut, self.machine.stops), (1, 1))

    def test_a_motion_with_no_end_on_record_is_not_homed(self):
        # Crashed before its end: the action thread reports ':failed', the
        # cancel flag is untouched, and no step totals were recorded.
        self.ends = lambda machine: None
        rc, said = self.home()
        self.assertEqual(rc, 2)
        self.assertTrue(any('did not run whole (no end on record)' in m for m in said), said)
        self.assertFalse(any(m.startswith('homing complete') for m in said), said)


if __name__ == '__main__':
    unittest.main()

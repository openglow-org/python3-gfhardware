"""ffmachine.park_lens: the lens park after the runner's hall reference.

The controller hands the runner the whole half-steps from the hall edge
to the park height (GFHOME_PARK_HALF_STEPS, clamped there to the window
every head reaches without touching a stop). The park takes them at the
run current in half-step mode, up or down, and rests the motor at the
hold current; a count of zero moves nothing. The lens axis is a fake that
records every call, installed only for the duration of each call.

Copyright 2026 514 LLC d/b/a OpenGlow
Written by Scott Wiederhold
SPDX-License-Identifier: MIT
"""
import enum
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

import ffmachine                                                  # noqa: E402


class ZCur(enum.Enum):
    HIGH = 0
    LOW = 1


class Microstep(enum.Enum):
    FULL = 1
    HALF = 2


class Dir(enum.Enum):
    Neg = 0
    Pos = 1


CALLS = []


class FakeZAxis:
    @staticmethod
    def configure(enabled=None, current=ZCur.LOW, mode=Microstep.HALF):
        CALLS.append(('configure', enabled, current, mode))

    @staticmethod
    def step(direction=Dir.Pos, step_delay=0.18):
        CALLS.append(('step', direction))


def fake_lens_modules():
    """The lens axis and its enums, standing in for gfhardware's for one
    call; the real package, when it is loaded, stays as it is."""
    pkg = sys.modules.get('gfhardware') or types.ModuleType('gfhardware')
    common = types.ModuleType('gfhardware._common')
    common.ZCur, common.Microstep, common.Dir = ZCur, Microstep, Dir
    z = types.ModuleType('gfhardware.z_axis')
    z.ZAxis = FakeZAxis
    return mock.patch.dict(sys.modules, {'gfhardware': pkg, 'gfhardware._common': common,
                                         'gfhardware.z_axis': z})


class ParkTests(unittest.TestCase):
    def setUp(self):
        del CALLS[:]

    def park(self, half_steps):
        with fake_lens_modules():
            ffmachine.park_lens(half_steps)

    def test_a_park_up_takes_the_steps_at_the_run_current_and_rests(self):
        self.park(5)
        self.assertEqual(CALLS[0], ('configure', True, ZCur.HIGH, Microstep.HALF))
        self.assertEqual(CALLS[1:6], [('step', Dir.Pos)] * 5)
        self.assertEqual(CALLS[6], ('configure', None, ZCur.LOW, Microstep.HALF))
        self.assertEqual(len(CALLS), 7)

    def test_a_park_down_steps_toward_the_bed(self):
        self.park(-3)
        self.assertEqual(CALLS[1:4], [('step', Dir.Neg)] * 3)
        self.assertEqual(len(CALLS), 5)

    def test_a_zero_park_moves_nothing_and_touches_nothing(self):
        self.park(0)
        self.assertEqual(CALLS, [])


if __name__ == '__main__':
    unittest.main()

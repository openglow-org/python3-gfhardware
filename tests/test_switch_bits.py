"""
Copyright 2026 514 LLC d/b/a OpenGlow
Written by Scott Wiederhold
https://community.openglow.org

SPDX-License-Identifier:    MIT

Host test for the cloud client's switch bits. InputSwitch's values are the
EV_SW codes the switch reader looks up, and they are the device tree's map
(the forgectrl switch table on the docs site): the button bit 2, the series
lid chain bit 3, the HV_ENABLE readback bit 4, the remote-interlock loop
bit 5. Machine._enclosure_open() reads them with their sense: the lid chain
active is the lid closed, the interlock bit active is the loop open, and
the HV_ENABLE readback opens nothing. A renumbered or inverted bit fails
here. This is the agreement of the bit numbers, not a physical read.

Run:  PYTHONPATH=.:../Glowforge-Utilities python3 -m unittest tests.test_switch_bits
"""
import os
import sys
import tempfile
import types
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(os.path.dirname(ROOT), 'Glowforge-Utilities'))
os.environ['GFHOME_CONF'] = os.path.join(tempfile.gettempdir(), 'no-such-forgefirm.conf')

# gfhardware/__init__ pulls in the whole hardware stack, so register a bare
# package and stub the two compiled extensions the machine's imports load.
_pkg = types.ModuleType('gfhardware')
_pkg.__path__ = [os.path.join(ROOT, 'gfhardware')]
sys.modules['gfhardware'] = _pkg

_cam_ext = types.ModuleType('gfhardware._cam')
_cam_ext.grab = lambda *a, **kw: b''
sys.modules['gfhardware._cam'] = _cam_ext

_evdev = types.ModuleType('gfhardware.input.evdev')
_input = types.ModuleType('gfhardware.input')
_input.__path__ = [os.path.join(ROOT, 'gfhardware', 'input')]
_input.evdev = _evdev
sys.modules['gfhardware.input'] = _input
sys.modules['gfhardware.input.evdev'] = _evdev

from gfhardware._common import InputSwitch                     # noqa: E402
from gfhardware.machine import Machine                         # noqa: E402


def _switches(doors, interlock, hv_enable=False):
    states = {sw: False for sw in InputSwitch}
    states[InputSwitch.SW_DOORS] = doors
    states[InputSwitch.SW_INTERLOCK] = interlock
    states[InputSwitch.SW_HV_ENABLE] = hv_enable
    return states


class SwitchBitsTest(unittest.TestCase):

    def test_the_bits_are_the_device_trees(self):
        self.assertEqual((InputSwitch.SW_BUTTON, InputSwitch.SW_DOORS, InputSwitch.SW_HV_ENABLE,
                          InputSwitch.SW_INTERLOCK), (2, 3, 4, 5))

    def test_the_enclosure_reads_them_with_their_sense(self):
        self.assertEqual(Machine._enclosure_open(_switches(doors=False, interlock=False)), 'lid opened')
        self.assertEqual(Machine._enclosure_open(_switches(doors=True, interlock=True)), 'interlock opened')
        self.assertIsNone(Machine._enclosure_open(_switches(doors=True, interlock=False)))
        self.assertIsNone(Machine._enclosure_open(_switches(doors=True, interlock=False, hv_enable=True)),
                          'the HV_ENABLE readback is not an enclosure open')


if __name__ == '__main__':
    unittest.main()

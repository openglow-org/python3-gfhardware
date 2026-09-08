"""
Copyright 2026 514 LLC d/b/a OpenGlow
Written by Scott Wiederhold
https://community.openglow.org

SPDX-License-Identifier:    MIT

Host tests for the camera privacy gate in gfhardware: neither camera
captures unless the lid is closed.

This is the second enforcement point. forgectrl gates the path everything
normally takes, but the cloud client falls back to a direct V4L2 grab when
the daemon is unreachable, and the capture utility never goes through it at
all -- so gfhardware.cam.capture() has to refuse on its own, and refuse
before it configures the pipeline or touches the lamp.

Two things are proven here:
  * capture() refuses with the lid open, and nothing is touched when it does
  * switches.lid_closed() fails CLOSED - an unreadable lid reads as open

Run:  PYTHONPATH=. python3 -m unittest tests.test_cam_lid_gate
"""
import os
import sys
import tempfile
import types
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

# gfhardware/__init__ pulls in the whole hardware stack (compiled
# extensions included), so register a bare package and stub the two
# extensions these modules import at load time.
_pkg = types.ModuleType('gfhardware')
_pkg.__path__ = [os.path.join(ROOT, 'gfhardware')]
sys.modules['gfhardware'] = _pkg

_cam_ext = types.ModuleType('gfhardware._cam')
_cam_ext.grab = lambda *a, **kw: b'\xff\xd8not-a-real-jpeg\xff\xd9'
sys.modules['gfhardware._cam'] = _cam_ext

_evdev = types.ModuleType('gfhardware.input.evdev')
_input = types.ModuleType('gfhardware.input')
_input.__path__ = [os.path.join(ROOT, 'gfhardware', 'input')]
_input.evdev = _evdev
sys.modules['gfhardware.input'] = _input
sys.modules['gfhardware.input.evdev'] = _evdev

from gfhardware._common import InputSwitch                     # noqa: E402
from gfhardware import switches as switches_mod                # noqa: E402
from gfhardware import cam as cam_mod                          # noqa: E402


class CaptureGateTest(unittest.TestCase):
    """capture() must refuse with the lid open, and leave the machine alone."""

    def setUp(self):
        self.touched = []
        self.lid = True

        # Every side effect capture() can have, recorded rather than done.
        self._orig = {}
        for name, rec in (
            ('_media_ctl', lambda *a: self.touched.append(('media-ctl',) + a) or ''),
            ('_v4l2_ctl', lambda *a: self.touched.append(('v4l2-ctl',) + a)),
            ('write_attr', lambda *a: self.touched.append(('write_attr',) + a)),
            ('read_file', lambda *a: self.touched.append(('read_file',) + a) or '0'),
            ('grab', lambda *a: self.touched.append(('grab',) + a) or b'\xff\xd8\xff\xd9'),
        ):
            self._orig[name] = getattr(cam_mod, name)
            setattr(cam_mod, name, rec)
        self._orig['lid_closed'] = cam_mod.lid_closed
        cam_mod.lid_closed = lambda *a, **kw: self.lid

    def tearDown(self):
        for name, fn in self._orig.items():
            setattr(cam_mod, name, fn)

    def test_refuses_with_lid_open(self):
        self.lid = False
        for cam_sel in (cam_mod.GFCAM_LID, cam_mod.GFCAM_HEAD):
            self.touched.clear()
            with self.assertRaises(cam_mod.LidOpen):
                cam_mod.capture(cam_sel)
            self.assertEqual(self.touched, [],
                             'a refused capture touched the machine: %r' % self.touched)

    def test_lid_open_refusal_is_not_a_generic_error(self):
        """The cloud client tells a refusal apart from a hardware failure."""
        self.lid = False
        with self.assertRaises(PermissionError):
            cam_mod.capture(cam_mod.GFCAM_LID)

    def test_captures_with_lid_closed(self):
        self.lid = True
        # The pipeline configuration resolves entity names from media-ctl
        # output; stub the two lookups capture() makes of it.
        def media_ctl(*args):
            self.touched.append(('media-ctl',) + args)
            if args and args[0] == '-p':
                return ('entity 1: ov5648 0-0036 (1 pad, 1 link)\n'
                        'entity 2: ov5648 3-0036 (1 pad, 1 link)\n')
            return '/dev/video0'
        cam_mod._media_ctl = media_ctl

        out = cam_mod.capture(cam_mod.GFCAM_LID)
        self.assertTrue(out.startswith(b'\xff\xd8'))
        kinds = [t[0] for t in self.touched]
        self.assertIn('grab', kinds, 'a permitted capture never reached the grabber')
        self.assertIn('v4l2-ctl', kinds, 'the sensor was never configured')

    def test_lid_is_read_before_anything_else(self):
        """Ordering matters: a refusal must not leave the lamp raised."""
        order = []
        cam_mod.lid_closed = lambda *a, **kw: (order.append('lid'), False)[1]
        cam_mod.write_attr = lambda *a: order.append('lamp')
        with self.assertRaises(cam_mod.LidOpen):
            cam_mod.capture(cam_mod.GFCAM_LID)
        self.assertEqual(order, ['lid'])


class LidClosedFailsClosedTest(unittest.TestCase):
    """switches.lid_closed() reports closed only on a positive read."""

    def setUp(self):
        self.tmp = tempfile.NamedTemporaryFile(delete=False)
        self.tmp.write(b'not an evdev node')
        self.tmp.close()

    def tearDown(self):
        os.unlink(self.tmp.name)

    def _with_bits(self, fn):
        _evdev.ioctl_EVIOCG_bits = fn
        return switches_mod.lid_closed(self.tmp.name)

    def test_closed_when_doors_bit_set(self):
        self.assertTrue(self._with_bits(
            lambda fd, evtype: [InputSwitch.SW_DOORS.value]))

    def test_open_when_doors_bit_clear(self):
        # Both individual door switches set but not the series combination:
        # SW_DOORS is the signal the safety chain uses and the only one the
        # gate trusts.
        self.assertFalse(self._with_bits(
            lambda fd, evtype: [InputSwitch.SW_DOOR1.value,
                                InputSwitch.SW_DOOR2.value]))

    def test_open_when_read_fails(self):
        def boom(fd, evtype):
            raise OSError(5, 'EIO')
        self.assertFalse(self._with_bits(boom))

    def test_open_when_device_missing(self):
        _evdev.ioctl_EVIOCG_bits = lambda fd, evtype: [InputSwitch.SW_DOORS.value]
        self.assertFalse(switches_mod.lid_closed(self.tmp.name + '-missing'))


if __name__ == '__main__':
    unittest.main()

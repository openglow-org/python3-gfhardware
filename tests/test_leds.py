"""
Copyright 2026 514 LLC d/b/a OpenGlow
Written by Scott Wiederhold
https://community.openglow.org

SPDX-License-Identifier:    MIT

Host tests for the button LED as the cloud client reads it: the real
gfhardware.leds against a temporary directory of the three channels'
attribute files. button_pattern() is what keeps the client from writing
over forgectrl's red flash and from rewriting a lamp that already shows
what it set, so each word it returns is a rule:

  - dark is 'off'
  - breathing white, as set_button_breathe writes it, is 'breathe', and
    still 'breathe' at its trough, where the targets have swung to 0
  - solid white, as set_button_color writes it, is 'white', and still
    'white' once the LED trigger has settled its 255 and reads it back 254
  - forgectrl's red flash (red pulsing, green and blue dark) is 'red', and
    a red with another channel lit under it is not
  - a button that cannot be read is None

A color set over a breathe stops the pulse, so it reads as that color.

Run:  python3 -m unittest tests.test_leds
"""
import os
import shutil
import sys
import tempfile
import types
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

# The package __init__ imports the machine and its hardware modules:
# register a bare package so gfhardware.leds loads on its own.
_pkg = types.ModuleType('gfhardware')
_pkg.__path__ = [os.path.join(ROOT, 'gfhardware')]
sys.modules['gfhardware'] = _pkg

from gfhardware._common import ButtonColor      # noqa: E402
from gfhardware import leds                     # noqa: E402


class ButtonPatternTests(unittest.TestCase):

    def setUp(self):
        self.root = tempfile.mkdtemp(prefix='leds-test-')
        self.addCleanup(shutil.rmtree, self.root, True)
        for led in range(1, 4):
            os.mkdir(os.path.join(self.root, 'button_led_%d' % led))
        self.addCleanup(setattr, leds, 'BUTTON_LED', leds.BUTTON_LED)
        leds.BUTTON_LED = os.path.join(self.root, 'button_led_%s') + os.sep
        leds.set_button_color(ButtonColor.OFF)

    def put(self, led, attr, val):
        with open(os.path.join(self.root, 'button_led_%d' % led, attr), 'w') as f:
            f.write('%d\n' % val)

    def lamp(self, targets, pulses):
        for led in range(1, 4):
            self.put(led, 'target', targets[led - 1])
            self.put(led, 'pulse_on', pulses[led - 1])
            self.put(led, 'pulse_off', pulses[led - 1])

    def test_dark_is_off(self):
        self.assertEqual(leds.button_pattern(), 'off')

    def test_breathing_white_is_breathe(self):
        leds.set_button_breathe()
        self.assertEqual(leds.button_pattern(), 'breathe')
        # At the trough every target has swung to 0: still a breathe, not
        # forgectrl's red flash.
        self.lamp((0, 0, 0), (1800, 1800, 1800))
        self.assertEqual(leds.button_pattern(), 'breathe')

    def test_a_color_set_over_a_breathe_stops_the_pulse(self):
        # The LED trigger keeps a pulse going under a new target: the dark
        # that ends the button wait has to stop the breathing.
        leds.set_button_breathe()
        leds.set_button_color(ButtonColor.OFF)
        self.assertEqual(leds.button_pattern(), 'off')

    def test_solid_white_is_white_as_written_and_as_it_reads_back_settled(self):
        leds.set_button_color(ButtonColor.WHITE)
        self.assertEqual(leds.button_pattern(), 'white')
        # The trigger takes the value a fade settled at as the target: a
        # written 255 reads back 254 from then on.
        self.lamp((254, 254, 254), (0, 0, 0))
        self.assertEqual(leds.button_pattern(), 'white')
        self.lamp((253, 254, 254), (0, 0, 0))
        self.assertEqual(leds.button_pattern(), 'other')

    def test_the_red_flash_is_red(self):
        # As forgectrl's led.c writes it: red at 255 pulsing 1 s on and 1 s
        # off, green and blue at 0 with no pulse.
        self.lamp((255, 0, 0), (1000, 0, 0))
        self.assertEqual(leds.button_pattern(), 'red')
        # Between pulses the pulsing channel's target swings to 0.
        self.lamp((0, 0, 0), (1000, 0, 0))
        self.assertEqual(leds.button_pattern(), 'red')

    def test_a_red_with_another_channel_lit_is_not_the_red_flash(self):
        self.lamp((255, 255, 255), (1000, 0, 0))
        self.assertEqual(leds.button_pattern(), 'other')
        self.lamp((255, 110, 0), (350, 350, 0))     # forgectrl's blinking amber
        self.assertEqual(leds.button_pattern(), 'other')

    def test_a_button_that_cannot_be_read_is_none(self):
        os.unlink(os.path.join(self.root, 'button_led_2', 'target'))
        self.assertIsNone(leds.button_pattern())


if __name__ == '__main__':
    unittest.main()

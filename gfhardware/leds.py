"""
Copyright 2020 514 LLC d/b/a OpenGlow
Written by Scott Wiederhold
https://community.openglow.org
SPDX-License-Identifier:    MIT
"""
import logging

from gfhardware._common import *

logger = logging.getLogger(LOGGER_NAME)

# One button LED channel's attribute directory (1 red, 2 green, 3 blue).
BUTTON_LED = '/sys/class/leds/button_led_%s/'


def head_all_led_off() -> None:
    set_head_mhs_laser(0)
    set_head_uv_led(0)
    set_head_white_led(0)


def set_button_color(color: ButtonColor) -> None:
    """A solid color, or dark. The LED trigger keeps a pulse going under a new
    target, so any pulse stops first."""
    for led in range(1, 4):
        write_attr(BUTTON_LED % led + 'pulse_on', 0)
        write_attr(BUTTON_LED % led + 'pulse_off', 0)
        write_attr(BUTTON_LED % led + 'speed', 64)
        write_attr(BUTTON_LED % led + 'target', color.value[led - 1])


def _led_int(path: str):
    try:
        with open(path) as f:
            return int(f.read().strip())
    except (OSError, ValueError):
        return None


def button_pattern():
    """What the button shows: 'red' (forgectrl's flash: red pulsing, green
    and blue dark), 'breathe', 'white', 'off', 'other', or None when it
    cannot be read.

    Solid white is nothing pulsing and every target at 254 or 255: the LED
    trigger ends a fade once it is within a sixteenth of the target and
    takes the value it reached as the target, so a 255 that has settled
    reads back 254."""
    p, t = [], []
    for led in range(1, 4):
        p.append(_led_int(BUTTON_LED % led + 'pulse_on'))
        t.append(_led_int(BUTTON_LED % led + 'target'))
    if None in p or None in t:
        return None
    if p[0] > 0 and p[1] <= 0 and p[2] <= 0 and t[1] <= 0 and t[2] <= 0:
        return 'red'
    if all(v > 0 for v in p):
        return 'breathe'
    if all(v == 0 for v in p) and all(v >= 254 for v in t):
        return 'white'
    if all(v == 0 for v in p) and all(v == 0 for v in t):
        return 'off'
    return 'other'


def set_button_breathe() -> None:
    """Breathing white: a press is what the job waits for."""
    for led in range(1, 4):
        write_attr(BUTTON_LED % led + 'pulse_on', 0)
        write_attr(BUTTON_LED % led + 'pulse_off', 0)
        write_attr(BUTTON_LED % led + 'speed', 12)
        write_attr(BUTTON_LED % led + 'target', 200)
        write_attr(BUTTON_LED % led + 'pulse_on', 1800)
        write_attr(BUTTON_LED % led + 'pulse_off', 1800)


def set_head_led_from_pulse(value: int) -> None:
    if value == 1072693248:
        set_head_mhs_laser(1023)


def set_head_mhs_laser(val: int) -> None:
    write_attr(SYSFS_GF_BASE + 'head/measure_laser', val)


def set_head_uv_led(val: int) -> None:
    write_attr(SYSFS_GF_BASE + 'head/uv_led', val)


def set_head_white_led(val: int) -> None:
    write_attr(SYSFS_GF_BASE + 'head/white_led', val)


def set_lid_led(level: int) -> None:
    if level < 0 or level > 255:
        raise ValueError('lid_led must be between 0 and 255, level: %s', level)
    write_attr('/sys/class/leds/lid_led/target', level)


__all__ = ['button_pattern', 'head_all_led_off', 'set_button_breathe', 'set_button_color', 'set_head_led_from_pulse', 'set_head_mhs_laser', 'set_head_uv_led',
           'set_head_white_led', 'set_lid_led']

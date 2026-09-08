"""
Copyright 2020 514 LLC d/b/a OpenGlow
Written by Scott Wiederhold
https://community.openglow.org
SPDX-License-Identifier:    MIT
"""
import logging

from gfhardware._common import *

logger = logging.getLogger(LOGGER_NAME)


def head_all_led_off() -> None:
    set_head_mhs_laser(0)
    set_head_uv_led(0)
    set_head_white_led(0)


def set_button_color(color: ButtonColor) -> None:
    for led in range(1, 4):
        write_attr('/sys/class/leds/button_led_%s/target' % led, color.value[led - 1])


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


__all__ = ['head_all_led_off', 'set_button_color', 'set_head_led_from_pulse', 'set_head_mhs_laser', 'set_head_uv_led',
           'set_head_white_led', 'set_lid_led']

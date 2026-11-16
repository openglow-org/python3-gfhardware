"""
Copyright 2020 514 LLC d/b/a OpenGlow
Written by Scott Wiederhold
https://community.openglow.org
SPDX-License-Identifier:    MIT
"""
import logging
import os
from math import exp, log

from gfhardware._common import *

logger = logging.getLogger(LOGGER_NAME)

# The coolant converter's 10-bit rail. The attribute is a 16-bit register,
# so a count can also arrive past it.
COOLANT_RAIL = 1023


def _coolant_degc(count: int) -> float:
    # Factory beta-equation conversion: 10k B3380 NTC in a 10k divider
    # behind a 1.3x gain stage, 10-bit ADC. Authoritative statement with
    # derivation and reference points:
    # https://docs.forgefirm.org/technical/machine/sensors/
    # For a count from 1 to 1022.
    adc_f = 1024 * 1.3
    r = 10000 / (adc_f / count - 1)
    rinf = 10000 * exp(-3380 / 298.15)
    return 3380 / log(r / rinf) - 273.15


def coolant_no_reading(count: int):
    """Why a coolant count is taken as no reading, with what it converts to,
    for the log; None for a count that is a reading.

    By the circuit, a shorted sensor reads 0, or a few counts over it that
    convert to hundreds of degrees (3 counts is about 371 C): a value over
    100 C, 30 C past the critical line's highest setting, is taken as a
    short. An open sensor pulls its input to the rail, 1023, which converts
    to about -3.5 C; a count of 1023 or more is no reading, the counts just
    under the rail convert under 0 C, and a value under 0 C is no reading
    either: a loop that cold is not fit to run. This client drives no fans,
    so the upper bound takes no cooling away; the cooling engine judges its
    own readings.
    """
    if count <= 0:
        return 'shorted: count %d' % count
    if count >= COOLANT_RAIL:
        return 'open, or no frame: count %d, at the rail or past it' % count
    c = _coolant_degc(count)
    if c < 0:
        return 'open, or the loop under 0 C: count %d is %.1f C' % (count, c)
    if c > 100:
        return 'shorted: count %d is %.1f C, over 100 C' % (count, c)
    return None


class _TempSensor(object):
    def __init__(self, sensor_def: dict):
        # A path, or a callable that finds the path at each read.
        self._sensor_path = sensor_def.get('sensor_path') or None
        self._temp_calc = sensor_def.get('temp_calc') or None

    @property
    def temp(self) -> Temperature:
        path = self._sensor_path() if callable(self._sensor_path) else self._sensor_path
        raw_t = int(read_file(path))
        if self._temp_calc is None:
            return Temperature(raw_t, -999.9, -999.9)
        c = round(self._temp_calc(raw_t), 1)
        return Temperature(raw_t, c, round(((c * (9 / 5)) + 32), 1))

    @staticmethod
    def calc_lm75(in_value: int) -> float:
        return in_value/1000

    @staticmethod
    def calc_coolant(in_value: int) -> float:
        # A count taken as no reading (coolant_no_reading) returns the
        # -273.15 sentinel.
        if coolant_no_reading(in_value) is not None:
            return -273.15
        return _coolant_degc(in_value)

    @staticmethod
    def calc_power(in_value: int) -> float:
        # Best-guess linear fit, unverified (documented on the sensors page)
        return (in_value * 0.08715) - 21


HWMON_BASE = '/sys/class/hwmon'


def _lm75_hwmon_path() -> str:
    """Resolve the LM75 chassis sensor's hwmon node by name, at each read.

    hwmon numbering depends on probe order: on the 6.12 kernel hwmon0 is the
    built-in imx_thermal CPU-die zone, while the LM75 (module, binds later)
    lands elsewhere - a hardcoded hwmon0 would silently report CPU temperature
    as chassis temperature. Resolved per read, so a sensor that binds after
    this module loaded is found; until then the path names no file and the
    read fails.
    """
    base = HWMON_BASE
    try:
        for node in sorted(os.listdir(base)):
            try:
                with open('%s/%s/name' % (base, node)) as f:
                    # the driver names the node after the bound chip variant
                    # ('lm75b' on the factory board)
                    if f.read().strip().startswith('lm75'):
                        return '%s/%s/temp1_input' % (base, node)
            except OSError:
                continue
    except OSError:
        pass
    logger.warning('no lm75 hwmon node found; chassis temp unavailable')
    return '%s/hwmon-lm75-not-found/temp1_input' % base


class _Temp(object):
    def __init__(self):
        self._chassis = _TempSensor({
            'sensor_path': _lm75_hwmon_path,
            'temp_calc': _TempSensor.calc_lm75
        })

        self._water_1 = _TempSensor({
            'sensor_path': SYSFS_GF_BASE + 'pic/water_temp_1',
            'temp_calc': _TempSensor.calc_coolant
        })

        self._water_2 = _TempSensor({
            'sensor_path': SYSFS_GF_BASE + 'pic/water_temp_2',
            'temp_calc': _TempSensor.calc_coolant
        })

        self._power = _TempSensor({
            'sensor_path': SYSFS_GF_BASE + 'pic/pwr_temp',
            'temp_calc': _TempSensor.calc_power
        })

        self._tec = _TempSensor({
            'sensor_path': SYSFS_GF_BASE + 'pic/tec_temp',
            'temp_calc': None  # Don't yet know
        })

    @property
    def chassis(self) -> Temperature:
        return self._chassis.temp

    @property
    def water_1(self) -> Temperature:
        return self._water_1.temp

    @property
    def water_2(self) -> Temperature:
        return self._water_2.temp

    @property
    def power(self) -> Temperature:
        return self._power.temp

    @property
    def tec(self) -> Temperature:
        return self._tec.temp

    @property
    def all(self) -> dict:
        """Every sensor's reading, for the log. A sensor that cannot be read
        is None: a reading taken only to be logged never ends a job. The
        gates read their sensor directly and fail on a bad read."""
        temps = {}
        for sensor in TEMP_SENSORS:
            try:
                temps[sensor] = getattr(self, sensor)
            except (OSError, ValueError):
                temps[sensor] = None
        return temps


class WaterPump(object):
    """The one thermal write this library keeps: the cooling engine in
    forgectrl owns the pump, the fans, the TEC and the heater, and the
    only thing the cloud client does to them is switch the heater off on
    its way down."""
    @staticmethod
    def heater_off() -> None:
        write_file(SYSFS_GF_BASE + 'thermal/heater_pwm', '0')


temp_sensor = _Temp()

__all__ = ['temp_sensor', 'WaterPump', 'coolant_no_reading']

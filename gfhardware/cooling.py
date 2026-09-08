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


class _TempSensor(object):
    def __init__(self, sensor_def: dict):
        self._sensor_path = sensor_def.get('sensor_path') or None
        self._temp_calc = sensor_def.get('temp_calc') or None

    @property
    def temp(self) -> Temperature:
        raw_t = int(read_file(self._sensor_path))
        if self._temp_calc is None:
            return Temperature(raw_t, -999.9, -999.9)
        c = round(self._temp_calc(raw_t), 1)
        return Temperature(raw_t, c, round(((c * (9 / 5)) + 32), 1))

    @staticmethod
    def calc_lm75(in_value: int) -> float:
        return in_value/1000

    @staticmethod
    def calc_coolant(in_value: int) -> float:
        # Factory beta-equation conversion: 10k B3380 NTC in a 10k divider
        # behind a 1.3x gain stage, 10-bit ADC. Authoritative statement with
        # derivation and reference points:
        # https://docs.forgefirm.org/technical/machine/sensors/
        adc_f = 1024 * 1.3
        if in_value <= 0 or in_value >= adc_f:
            return -273.15      # open / shorted sensor
        r = 10000 / (adc_f / in_value - 1)
        rinf = 10000 * exp(-3380 / 298.15)
        return 3380 / log(r / rinf) - 273.15

    @staticmethod
    def calc_power(in_value: int) -> float:
        # Best-guess linear fit, unverified (documented on the sensors page)
        return (in_value * 0.08715) - 21


def _lm75_hwmon_path() -> str:
    """Resolve the LM75 chassis sensor's hwmon node by name.

    hwmon numbering depends on probe order: on the 6.12 kernel hwmon0 is the
    built-in imx_thermal CPU-die zone, while the LM75 (module, binds later)
    lands elsewhere - a hardcoded hwmon0 would silently report CPU temperature
    as chassis temperature.
    """
    base = '/sys/class/hwmon'
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
            'sensor_path': _lm75_hwmon_path(),
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
        temps = {}
        for sensor in TEMP_SENSORS:
            temps[sensor] = getattr(self, sensor)
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

__all__ = ['temp_sensor', 'WaterPump']

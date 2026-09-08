"""
Copyright 2020 514 LLC d/b/a OpenGlow
Written by Scott Wiederhold
https://community.openglow.org
SPDX-License-Identifier:    MIT
"""
import logging
from time import sleep
from typing import Union
from gfhardware._common import *

logger = logging.getLogger(LOGGER_NAME)


class ZAxis(object):
    # Full Z travel is a few dozen full steps; several times that bounds a
    # homing sweep so a failed hall sensor cannot drive the lens carriage
    # against its stop indefinitely.
    MAX_HOME_STEPS = 200

    @staticmethod
    def configure(enabled: bool = None, current: ZCur = ZCur.LOW, mode: Microstep = Microstep.HALF) -> None:
        logger.info('configuring z_enable: %s, current: %s, mode: %s' % (enabled, current, mode))
        ZAxis.set_current(current)
        ZAxis.set_mode(mode)
        if enabled is True:
            ZAxis.enable()
        elif enabled is False:
            ZAxis.disable()

    @staticmethod
    def disable() -> None:
        logger.debug('setting z_enable: 1')
        write_file(SYSFS_GF_BASE + 'head/z_enable', '1')

    @staticmethod
    def enable() -> None:
        logger.debug('setting z_enable: 0')
        write_file(SYSFS_GF_BASE + 'head/z_enable', '0')

    @staticmethod
    def home() -> list:
        logger.info('starting z homing cycle')
        concurrence = []
        z_pos = 0

        ZAxis.configure(enabled=True, current=ZCur.HIGH, mode=Microstep.FULL)
        ZAxis._step_until(False)
        ZAxis._step_until(True)

        while len(concurrence) < 5:
            logger.info('pass: %s', len(concurrence) + 1)
            z_pos = z_pos + (ZAxis._step_until(False) * 2)
            z_pos = z_pos + (ZAxis._step_until(True) * 2)
            concurrence.append(z_pos)
        ZAxis.configure(current=ZCur.LOW, mode=Microstep.HALF)
        logger.info('homing cycle complete: %s' % concurrence)
        return concurrence

    @staticmethod
    def _home_sense() -> bool:
        home = True if int(read_file(SYSFS_GF_BASE + 'head/hall_sensor')) == 0 else False
        logger.debug('read z_home as: %s' % home)
        return home

    @staticmethod
    def reset() -> None:
        logger.info('resetting z')
        ZAxis.configure(enabled=False, current=ZCur.LOW, mode=Microstep.HALF)

    @staticmethod
    def set_current(cur: Union[ZCur, str, int]) -> None:
        if isinstance(cur, ZCur):
            cur = str(cur.value)
        elif isinstance(cur, int):
            cur = str(cur)
        logger.debug('setting z_current to: %s' % cur)
        write_file(SYSFS_GF_BASE + 'head/z_current', cur)

    @staticmethod
    def set_mode(mode: Union[Microstep, str, int]) -> None:
        if isinstance(mode, Microstep):
            mode = mode.value
        if isinstance(mode, str):
            mode = int(mode)
        if mode < 1 or mode > 2:
            raise ValueError('Z mode can only be 1 or 2')
        mode = '0' if mode == 1 else '1'
        logger.debug('setting z_mode to: %s' % mode)
        write_file(SYSFS_GF_BASE + 'head/z_mode', mode)

    @staticmethod
    def set_mode_from_puls(mode: int) -> None:
        mode = 1 if mode == 0 else 2
        ZAxis.set_mode(mode)

    @staticmethod
    def step(direction: Dir = Dir.Pos, step_delay: float = .180) -> None:
        write_file(SYSFS_GF_BASE + 'cnc/z_step', str(direction.value))
        sleep(step_delay)

    @staticmethod
    def _step_until(at_home: bool) -> int:
        logger.info('starting z_step until at_home: %s' % at_home)
        step_count = 0
        step = 1 if at_home else -1
        step_dir = Dir.Pos if at_home else Dir.Neg
        while ZAxis._home_sense() is not at_home:
            if abs(step_count) >= ZAxis.MAX_HOME_STEPS:
                raise RuntimeError(
                    'z home sense never reached %s after %d steps '
                    '(failed hall sensor?)' % (at_home, abs(step_count)))
            logger.debug('sending z_step: %s' % step_dir)
            ZAxis.step(step_dir)
            step_count = step_count + step
        logger.info('took %s z_steps' % step_count)
        return step_count


__all__ = ['ZAxis']

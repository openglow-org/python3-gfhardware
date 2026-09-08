"""
Copyright 2020 514 LLC d/b/a OpenGlow
Written by Scott Wiederhold
https://community.openglow.org
SPDX-License-Identifier:    MIT
"""
import logging
import os
from typing import Union

from gfhardware._common import *

logger = logging.getLogger(LOGGER_NAME)


class _CNC(object):
    # Externally-held exclusive /dev/glowforge file object (the job-scoped
    # deadman fd, see machine.py). The device is exclusive-open in the kernel,
    # so while a job holds it, every seek/write must route through that one
    # fd instead of opening the device again (which would fail -EBUSY).
    pulse_dev = None

    @staticmethod
    def set_pulse_dev(dev):
        """Register (or clear, with None) the job-held pulse-device file."""
        _CNC.pulse_dev = dev

    @staticmethod
    def clear_all():
        _CNC._dev_seek(0)

    @staticmethod
    def clear_pulse_and_byte():
        _CNC._dev_seek(1)

    @staticmethod
    def clear_position():
        _CNC._dev_seek(2)

    @staticmethod
    def _command(cmd: str):
        write_file(SYSFS_GF_BASE + 'cnc/' + cmd, '1')

    @staticmethod
    def _dev_seek(count):
        if _CNC.pulse_dev is not None:
            os.lseek(_CNC.pulse_dev.fileno(), count, os.SEEK_SET)
            return
        # The forgectrl broker holds the device exclusive-open for its
        # lifetime; seek through the inherited fd - a transient
        # self-open would fail -EBUSY against it.
        fd = os.getenv('GF_PULSE_FD')
        if fd is not None:
            os.lseek(int(fd), count, os.SEEK_SET)
        else:
            with open(PULS_DEVICE, 'w') as f:
                os.lseek(f.fileno(), count, os.SEEK_SET)

    @staticmethod
    def disable():
        _CNC._command('disable')

    @staticmethod
    def enable():
        _CNC._command('enable')

    @staticmethod
    def halt():
        """Immediate stop, no deceleration (emergency use only - steps may
        be skipped at speed, so the position can no longer be trusted)."""
        _CNC._command('halt')

    @property
    def faults(self) -> str:
        return read_file(SYSFS_GF_BASE + 'cnc/faults')

    @property
    def ignored_faults(self) -> str:
        return read_file(SYSFS_GF_BASE + 'cnc/ignored_faults')

    @staticmethod
    def laser_latch(val):
        logger.info(val)
        write_attr(SYSFS_GF_BASE + 'cnc/laser_latch', val)

    @property
    def motor_lock(self) -> str:
        return read_file(SYSFS_GF_BASE + 'cnc/motor_lock')

    @property
    def position(self) -> Position:
        raw = read_file(SYSFS_GF_BASE + 'cnc/position', True)
        # Full 4-byte little-endian words; 3-byte slices would drop each MSB
        # and wrap the counters at 16 MiB.
        return Position(
            _CNC.position_calc(int.from_bytes(raw[0:4], byteorder='little', signed=True), self.x_mode, XY_STEP_PER_MM),
            _CNC.position_calc(int.from_bytes(raw[4:8], byteorder='little', signed=True), self.y_mode, XY_STEP_PER_MM),
            _CNC.position_calc(int.from_bytes(raw[8:12], byteorder='little', signed=True), 2, Z_STEP_PER_MM),
            PulsPosition(
                int.from_bytes(raw[16:20], byteorder='little', signed=False),
                int.from_bytes(raw[12:16], byteorder='little', signed=False)
            )
        )

    @staticmethod
    def position_calc(steps: int, mode: int, mm_per_step: float) -> AxisPosition:
        mm = (steps / mode) * mm_per_step
        return AxisPosition(steps, mm, mm / 25.4)

    @staticmethod
    def reset():
        # Rail policy belongs to the forgectrl broker when it owns the
        # device (GF_PULSE_FD): a disable here, with the enable that
        # follows moments later in machine setup, is a fast off/on
        # bounce on the marginal 40 V rail. Standalone keeps the full
        # known-state sequence.
        if os.getenv('GF_PULSE_FD') is None:
            _CNC.disable()
        _CNC.set_ignored_faults(0)
        _CNC.clear_all()
        _CNC.set_step_freq(10000)
        _CNC.set_x_mode(Microstep.M_8)
        _CNC.set_y_mode(Microstep.M_8)
        _CNC.set_x_decay(1)
        _CNC.set_y_decay(1)
        _CNC.set_x_current(33)
        _CNC.set_y_current(33)

    @staticmethod
    def resume(steps: int):
        """Kernel resume/backtrack (UAPI 'resume'): negative = laser off,
        run that many ticks backward and stop (preload model only);
        positive = run forward, re-enable the laser after that many
        ticks; 0 = run forward without re-enabling the laser."""
        write_attr(SYSFS_GF_BASE + 'cnc/resume', int(steps))

    @staticmethod
    def run():
        _CNC._command('run')

    @property
    def sdma_context(self) -> SDMA:
        line = read_file(SYSFS_GF_BASE + 'cnc/sdma_context').splitlines()
        return SDMA(**{
            'pc': line[0][3:7],
            'rpc': line[0][12:16],
            'spc': line[0][21:25],
            'epc': line[0][30:34],
            't': line[1][9:10],
            'sf': line[1][14:15],
            'df': line[1][19:20],
            'lm': line[1][24:25],
            'r0': line[2][4:12],
            'r1': line[2][17:25],
            'r2': line[2][30:38],
            'r3': line[2][43:51],
            'r4': line[2][56:64],
            'r5': line[2][69:77],
            'r6': line[3][4:12],
            'r7': line[3][17:25],
            'mda': line[3][30:38],
            'msa': line[3][43:51],
            'ms': line[3][56:64],
            'md': line[3][69:77],
            'pda': line[4][4:12],
            'psa': line[4][17:25],
            'ps': line[4][30:38],
            'pd': line[4][43:51],
            'ca': line[4][56:64],
            'cs': line[4][69:77],
            'dda': line[5][4:12],
            'dsa': line[5][17:25],
            'ds': line[5][30:38],
            'dd': line[5][43:51],
            'sc0': line[5][56:64],
            'sc1': line[5][69:77],
            'sc2': line[6][4:12],
            'sc3': line[6][17:25],
            'sc4': line[6][30:38],
            'sc5': line[6][43:51],
            'sc6': line[6][56:64],
            'sc7': line[6][69:77],
        })

    @staticmethod
    def set_ignored_faults(val: Union[str, int]):
        logger.info(val)
        write_attr(SYSFS_GF_BASE + 'cnc/ignored_faults', val)

    @staticmethod
    def set_motor_lock(val):
        logger.info(val)
        write_attr(SYSFS_GF_BASE + 'cnc/motor_lock', val)

    @staticmethod
    def set_step_freq(val: Union[str, int]):
        logger.info(val)
        write_attr(SYSFS_GF_BASE + 'cnc/step_freq', val)

    @staticmethod
    def set_x_current(val: Union[str, int]):
        logger.info(val)
        write_attr(SYSFS_GF_BASE + 'pic/x_step_current', val)

    @staticmethod
    def set_x_decay(val: Union[str, int]):
        logger.info(val)
        write_attr(SYSFS_GF_BASE + 'cnc/x_decay', val)

    @staticmethod
    def set_x_mode(val: Union[Microstep, int, str]):
        logger.info(val)
        if isinstance(val, Microstep):
            val = val.value
        write_attr(SYSFS_GF_BASE + 'cnc/x_mode', val)

    @staticmethod
    def set_y_current(val: Union[str, int]):
        logger.info(val)
        write_attr(SYSFS_GF_BASE + 'pic/y_step_current', val)

    @staticmethod
    def set_y_decay(val: Union[str, int]):
        logger.info(val)
        write_attr(SYSFS_GF_BASE + 'cnc/y_decay', val)

    @staticmethod
    def set_y_mode(val: Union[Microstep, int, str]):
        logger.info(val)
        if isinstance(val, Microstep):
            val = val.value
        write_attr(SYSFS_GF_BASE + 'cnc/y_mode', val)

    @property
    def state(self) -> MachineState:
        # The in-run safety poll lives on this property: an unknown or
        # unreadable state degrades to FAULT (not running, not safe to
        # start) instead of raising out of the poll and abandoning a
        # running job.
        try:
            state = read_file(SYSFS_GF_BASE + 'cnc/state')
        except OSError:
            return MachineState.FAULT
        if state == 'disabled':
            return MachineState.DISABLED
        elif state == 'idle':
            return MachineState.IDLE
        elif state == 'running':
            return MachineState.RUNNING
        elif state == 'fault':
            return MachineState.FAULT
        elif state == 'underrun':
            return MachineState.UNDERRUN
        else:
            logger.error('invalid cnc state: %r' % (state,))
            return MachineState.FAULT

    @property
    def free(self) -> int:
        """Bytes the pulse ring will accept right now, less its backtrack gap.

        Reading it costs two SDMA transactions, so it is for reporting and
        pre-flight, never for pacing: a feeder paces on -ENOMEM from the
        write instead.
        """
        return int(read_file(SYSFS_GF_BASE + 'cnc/free'))

    @property
    def max_backtrack(self) -> int:
        """Steps a backward run may be asked for right now.

        What the ring still holds of what it has already played, less the
        deceleration tail. A pause sizes both its backtrack and the resume's
        laser-on lead from this: the kernel refuses a longer request rather
        than quietly running a shorter one. Costs SDMA transactions, so read
        it once per pause, not in a loop.
        """
        return int(read_file(SYSFS_GF_BASE + 'cnc/max_backtrack'))

    @property
    def step_freq(self) -> int:
        return int(read_file(SYSFS_GF_BASE + 'cnc/step_freq'))

    @property
    def streaming(self) -> bool:
        """Whether end-of-data mid-run counts as an underrun."""
        return read_file(SYSFS_GF_BASE + 'cnc/streaming').strip() == '1'

    @staticmethod
    def set_streaming(val: Union[bool, int, str]):
        """Declare a live feed (1) or a fully-enqueued program (0).

        A live-fed run sets this before ``run`` so a ring that goes dry lands
        in ``underrun`` instead of looking like a clean finish, and clears it
        after the job's last bytes are enqueued so the real end-of-data counts
        as completion.
        """
        logger.info(val)
        write_attr(SYSFS_GF_BASE + 'cnc/streaming', int(val))

    @property
    def underruns(self) -> int:
        """Streaming underruns since the module was loaded."""
        return int(read_file(SYSFS_GF_BASE + 'cnc/underruns'))

    @staticmethod
    def stop():
        _CNC._command('stop')

    @property
    def x_current(self) -> int:
        return int(read_file(SYSFS_GF_BASE + 'pic/x_step_current'))

    @property
    def y_current(self) -> int:
        return int(read_file(SYSFS_GF_BASE + 'pic/y_step_current'))

    @property
    def x_decay(self) -> int:
        return int(read_file(SYSFS_GF_BASE + 'cnc/x_decay'))

    @property
    def x_mode(self) -> int:
        return int(read_file(SYSFS_GF_BASE + 'cnc/x_mode'))

    @property
    def y_decay(self) -> int:
        return int(read_file(SYSFS_GF_BASE + 'cnc/y_decay'))

    @property
    def y_mode(self) -> int:
        return int(read_file(SYSFS_GF_BASE + 'cnc/y_mode'))


cnc = _CNC()

__all__ = ['cnc']

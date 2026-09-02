"""
(C) Copyright 2020
Scott Wiederhold, s.e.wiederhold@gmail.com
https://community.openglow.org
SPDX-License-Identifier:    MIT

Based on python-evdev
Copyright (c) 2012-2016 Georgi Valkov. All rights reserved.

"""
import logging
from time import sleep
import os
import select
from threading import Thread
from typing import Callable

from gfhardware._common import *
if os.getenv('REMOTE_DEBUG'):
    evdev = load_installed_extension('input.evdev')
else:
    from gfhardware.input import evdev

logger = logging.getLogger(LOGGER_NAME)


class InputDevice(object):
    """
    A linux input device from which input events can be read.
    """
    __slots__ = ('path', 'fd')

    def __init__(self, dev):
        """
        Arguments
        ---------
        dev : str|bytes|PathLike
          Path to input device
        """
        self.path = dev if not hasattr(dev, '__fspath__') else dev.__fspath__()
        fd = os.open(dev, os.O_RDONLY | os.O_NONBLOCK)
        self.fd = fd

    def __del__(self):
        if hasattr(self, 'fd') and self.fd is not None:
            try:
                self.close()
            except OSError:
                pass

    def close(self):
        if self.fd > -1:
            try:
                os.close(self.fd)
            finally:
                self.fd = -1

    def read_loop(self) -> SwitchEvent:
        """
        Enter an endless :func:`select.select()` loop that yields input events.

        No EVIOCGRAB: other services (forgectrl's status endpoint) read this
        device concurrently, and exclusivity of button *meaning* comes from
        controller-mode selection (see the forgectrl contract,
        https://docs.forgefirm.org/technical/forgefirm/forgectrl/).
        """
        while True:
            r, w, x = select.select([self.fd], [], [], .1)
            if len(r) == 0:
                yield None
            else:
                for event in self.read():
                    yield event

    def read(self) -> SwitchEvent:
        """
        Read multiple input events from device. Return a generator object that
        yields :class:`InputEvent <evdev.events.InputEvent>` instances. Raises
        `BlockingIOError` if there are no available events at the moment.
        """
        # events -> [(sec, usec, type, code, val), ...]
        events = evdev.device_read_many(self.fd)

        for event in events:
            sec, usec, e_type, code, val = tuple(event)
            try:
                if e_type == 5:
                    code = InputSwitch(code)
                    val = True if int(val) == 1 else False
                elif e_type == 0:
                    code = SynCode(code)
            except ValueError:
                # A code this map does not know is not an event of ours;
                # it must not end the reader that carries the lid and the
                # button.
                logger.debug('ignoring unknown switch event %s/%s' % (e_type, code))
                continue
            yield SwitchEvent(sec, usec, e_type, code, val)

    def switch_states(self) -> dict:
        """
        Return current switch states.
        i.e. {<InputSwitch.DOOR1: 0>: True, <InputSwitch.DOOR2: 1>: False, ...}
        """
        active_switches = evdev.ioctl_EVIOCG_bits(self.fd, EventCode.EV_SW.value)
        switch_states = {}
        for switch in InputSwitch:
            switch_states[switch] = True if switch.value in active_switches else False
        return switch_states


def lid_closed(device: str = SWITCH_DEVICE) -> bool:
    """Is the lid closed right now?

    A one-shot read for callers that need the answer without running a
    SwitchMonitor thread (the camera privacy gate in gfhardware.cam). Reads
    SW_DOORS -- the series combination of both lid switches, the same signal
    the hardware safety chain uses -- so it cannot report closed while either
    switch says otherwise.

    Fails CLOSED: any error reading the device returns False. A caller that
    gates on this keeps the cameras dark rather than capturing on a bad read.
    """
    dev = None
    try:
        dev = InputDevice(device)
        return bool(dev.switch_states()[InputSwitch.SW_DOORS])
    except Exception:  # noqa: BLE001 - deliberately total: unreadable = not closed
        logger.warning('lid state unreadable; treating the lid as open')
        return False
    finally:
        if dev is not None:
            try:
                dev.close()
            except OSError:
                pass


class SwitchMonitor(Thread):
    """
    Switch Event Process Queue
    Responds to incoming WSS events for motion actions..
    """
    def __init__(self, input_dev: str, event_handler: Callable[[SwitchEvent], None]):
        """
        Initialize Switch Event Thread
        :param input_dev: Input Device object
        :type input_dev: str
        :param event_handler: Function to pass event object to
        :type event_handler: object
        """
        self._input_dev = InputDevice(input_dev)
        self.stop = False
        self._event_handler = event_handler
        # daemon: the blocking evdev read_loop must never keep the process
        # alive after the service loop has died - a wedged interpreter that
        # cannot exit is invisible to the supervisor's respawn.
        Thread.__init__(self, daemon=True)

    def run(self) -> None:
        logger.debug('THREAD START')
        # The button press reaches the run loop only through this thread,
        # so it does not end on an exception: the fault is logged and the
        # read loop is entered again.
        while not self.stop:
            try:
                for event in self._input_dev.read_loop():
                    if event is not None:
                        if event.type == 5:
                            self._event_handler(event)
                        elif event.type == 0 and event.code == SynCode.SYN_DROPPED:
                            logger.error('switch monitor dropped events: %s' % str(event))
                    if self.stop:
                        break
            except Exception:
                logger.exception('switch monitor failed; restarting the read loop')
                sleep(0.5)
        logger.debug('THREAD EXIT')

    def switch_state(self, switch: InputSwitch) -> bool:
        return self.all_switches().get(switch, None)

    def all_switches(self) -> dict:
        return self._input_dev.switch_states()


__all__ = ['SwitchMonitor', 'lid_closed']

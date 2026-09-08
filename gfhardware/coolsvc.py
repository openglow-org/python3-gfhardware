"""
Copyright 2020 514 LLC d/b/a OpenGlow
Written by Scott Wiederhold
https://community.openglow.org
SPDX-License-Identifier:    MIT
"""
import json
import logging
import os
import time
from threading import Thread, Lock
from urllib import parse, request

from gfhardware._common import LOGGER_NAME, SYSFS_GF_BASE, write_file

logger = logging.getLogger(LOGGER_NAME)

VERDICT_FILE = '/run/forgefirm/cooling.state'
# Reader staleness window per the contract: a verdict older than this
# is no verdict (fire blocked). The engine publishes at 1 Hz.
VERDICT_MAX_AGE_S = 2.0
REPORT_PERIOD_S = 1.0
# localhost: anything slower means forgectrl is wedged - drop the
# report, the level-triggered refresh retries in a second.
# The engine stands the fire gate down after 5 s without a report. The
# reporter has its own thread, so the timeout can take most of one
# report period: a slow answer from the daemon (a snapshot's encode on
# the single core) costs one report, not five.
REPORT_TIMEOUT_S = 1.0
# The run airflow a controller writes itself, once, when the verdict goes
# stale while the laser is armed: the engine is gone with the laser hot,
# and these are writes nobody else will make now. The same duties the
# GRBL controller falls back to.
FALLBACK_AIRFLOW = (('head/air_assist_pwm', '1023'),
                    ('thermal/exhaust_pwm', '65535'),
                    ('thermal/intake_pwm', '43278'))

# The job's operating envelope, as far as this machine carries it to the
# engine: the pulse-header tags that bound a gate the engine has or is
# getting, and the /cool/state parameter each becomes. The engine treats
# each as a limit that can only tighten its own configured one; a looser
# value is logged and ignored there (contract:
# https://docs.forgefirm.org/technical/forgefirm/cooling-engine/).
#
# Temperatures arrive in millidegrees. The tach windows are maximum
# periods in the kernel's own units (the factory compares them against
# the same sysfs attributes), so a maximum period is a minimum speed:
# exhaust and intake report nanoseconds at 2 pulses per revolution, the
# air assist microseconds at 8.
LIMIT_TAGS = {
    'CMrx': 'coolant_max_c',
    'CMrn': 'coolant_min_c',
    'EFrx': 'exhaust_min_rpm',
    'IFrx': 'intake_min_rpm',
    'AArx': 'air_assist_min_rpm',
}
# The tach minimum periods are maximum speeds, which nothing gates on;
# they are read so a header carrying them is not reported as a gap.
INERT_LIMIT_TAGS = ('AArn', 'EFrn', 'IFrn')
# What the service writes into a field it has nothing to say about: zero,
# the ADC rail, the signed extremes (the latter as the unsigned values a
# header parse yields) and the unsigned rail.
SENTINELS = frozenset((0, 1023, 0x7fffffff, 0x80000000, 0xffffffff))


def limits_from_header(header: dict) -> dict:
    """The per-job limits a pulse header carries, as /cool/state
    parameters. A tag that is absent, a sentinel, or converts to a
    number no real machine could mean yields nothing; the engine's local
    limit stands for it."""
    out = {}
    for tag, name in LIMIT_TAGS.items():
        val = header.get(tag)
        if not isinstance(val, int) or isinstance(val, bool) or val in SENTINELS or val < 0:
            continue
        if tag in ('CMrx', 'CMrn'):
            degc = val / 1000.0
            if not 0.0 < degc < 100.0:
                continue
            out[name] = round(degc, 2)
            continue
        if tag == 'AArx':
            rpm = 60e6 / (val * 8)       # microseconds, 8 pulses per rev
        else:
            rpm = 60e9 / (val * 2)       # nanoseconds, 2 pulses per rev
        if not 1.0 <= rpm <= 100000.0:
            continue
        out[name] = int(round(rpm))
    return out


class CoolingService(Thread):
    """Client of the forgectrl cooling engine (contract:
    https://docs.forgefirm.org/technical/forgefirm/cooling-engine/).

    The engine owns the thermal hardware - fan/pump/TEC/heater profiles,
    coolant flow verification, over-temp policy - for every controller
    mode. This client reports job state (mode, armed, and the per-job
    run fan duties from the pulse header) level-triggered at ~1 Hz, and
    reads the engine's published verdict on the fire path, treating a
    missing or stale verdict file as fire-blocked.
    """

    def __init__(self):
        self.stop = False
        self.armed = False
        self._lock = Lock()
        self._mode = 'idle'
        self._profile = {}
        self._limits = {}
        port = os.getenv('FORGECTRL_PORT', '80')
        self._url = 'http://127.0.0.1:%s/cool/state' % port
        Thread.__init__(self, daemon=True)

    # ------------------------------------------------------- job state

    def set_mode(self, mode: str) -> None:
        with self._lock:
            self._mode = mode
        self.report()

    def set_armed(self, armed: bool) -> None:
        self.armed = bool(armed)
        self.report()

    # Per-job run fan profile, fed from the pulse-header keys by the
    # MACHINE_SETTINGS map (AArd/EFrd/IFrd). The engine falls back to
    # its configured/factory duties for anything unset or out of range.
    def profile_air_assist(self, val) -> None:
        self._set_duty('air_assist', val)

    def profile_exhaust(self, val) -> None:
        self._set_duty('exhaust', val)

    def profile_intake(self, val) -> None:
        self._set_duty('intake', val)

    def _set_duty(self, key: str, val) -> None:
        try:
            with self._lock:
                self._profile[key] = int(val)
        except (TypeError, ValueError):
            logger.warning('bad %s duty in pulse header: %r', key, val)

    def clear_profile(self) -> None:
        with self._lock:
            self._profile = {}

    # Per-job limits from the pulse header (limits_from_header), riding
    # every report while the job is loaded so a lost report self-heals
    # and the engine's effective limits follow the job, not a moment.
    def set_limits(self, limits: dict) -> None:
        with self._lock:
            self._limits = dict(limits or {})

    def clear_limits(self) -> None:
        with self._lock:
            self._limits = {}

    def report(self) -> None:
        with self._lock:
            params = {'mode': self._mode, 'armed': int(self.armed)}
            params.update(self._profile)
            params.update(self._limits)
        try:
            request.urlopen(
                request.Request('%s?%s' % (self._url, parse.urlencode(params)),
                                method='POST'),
                timeout=REPORT_TIMEOUT_S).close()
        except Exception as e:
            # Level-triggered: the next report self-heals. Nothing a
            # half-restarted engine can throw (http.client exceptions
            # included) may kill the reporter thread - or abort a job when
            # report() is called from set_mode/set_armed on the action
            # thread.
            logger.debug('cooling report failed: %s', e)

    def run(self):
        while not self.stop:
            self.report()
            time.sleep(REPORT_PERIOD_S)
        # Parting report: the shutdown path set idle/disarmed before
        # stopping the reporter, so the engine hears the final state even
        # if those direct reports raced a restart.
        self.report()

    # --------------------------------------------------------- verdict

    def verdict(self) -> dict:
        """The engine's current verdict, or None when missing/stale
        (which readers must treat as fire_ok=False, hold=True)."""
        try:
            with open(VERDICT_FILE) as f:
                v = json.load(f)
            age = time.clock_gettime(time.CLOCK_MONOTONIC) - v['ts_mono']
            # Negative age = future-dated ts_mono, which would otherwise
            # read as permanently fresh (the C reader has the same guard).
            if age > VERDICT_MAX_AGE_S or age < 0:
                return None
            return v
        except (OSError, ValueError, KeyError, TypeError):
            return None

    def fire_ok(self) -> bool:
        v = self.verdict()
        return bool(v and v.get('fire_ok'))

    def hold(self) -> bool:
        """The engine asks for a hold. A missing or stale verdict holds."""
        v = self.verdict()
        return v is None or bool(v.get('hold', True))

    def resume_ok(self) -> bool:
        """The engine offers the resume from its hold. Never on a missing
        or stale verdict, and never while it still asks for the hold."""
        v = self.verdict()
        return bool(v and v.get('resume_ok') and not v.get('hold', True))

    @staticmethod
    def fallback_airflow() -> None:
        """The run airflow, written directly: for a verdict gone stale
        while the laser is armed (the engine is gone with the laser hot)."""
        for attr, duty in FALLBACK_AIRFLOW:
            try:
                write_file(SYSFS_GF_BASE + attr, duty)
            except OSError as e:
                logger.error('fallback airflow: cannot write %s: %s', attr, e)


cooling_svc = CoolingService()

__all__ = ['cooling_svc', 'CoolingService', 'limits_from_header', 'LIMIT_TAGS',
           'INERT_LIMIT_TAGS']

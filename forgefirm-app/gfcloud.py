#!/usr/bin/python3
"""
gfcloud - full Glowforge web-service controller for ForgeFIRM.

Runs the machine under the Glowforge web service (the factory cloud
experience): the phone/web app drives homing, framing, and printing.
Started by the gfcloud init service when controller_mode = cloud in
/data/forgefirm.conf, which keeps grblHAL down so this daemon owns
/dev/glowforge exclusively.

Reconnects (fresh single-use ws_token) and 401 re-auth are handled in
gfutilities. On SIGTERM the service loop stops and the machine is shut
down safe (laser latched, steppers disabled, deadman released).

Offline: with --offline, or while the marker file OFFLINE_MARKER exists
at start, the machine runs under gfutilities' OfflineService instead: no
account, no network, a local socket (OFFLINE_SOCKET) that takes the
service's action messages and hands back the machine's events. The
acceptance tests use it to run the machine's print behavior without a
job from the app. The marker lives under /run, so a reboot never comes
up offline by accident.

(C) Copyright 2026
Scott Wiederhold, s.e.wiederhold@gmail.com
SPDX-License-Identifier: MIT
"""
import argparse
import logging
import os
import shutil
import signal
import sys
import time
from pathlib import Path

from gfutilities.configuration import parse, get_cfg
from gfutilities import GFUIService

import ffmachine

CONF = '/data/etc/gfhome.conf'
CONF_SAMPLE = '/etc/gfhome.conf.sample'
OFFLINE_MARKER = '/run/gfcloud-offline'
OFFLINE_SOCKET = '/run/gfcloud-offline.sock'

logger = logging.getLogger('openglow')


def load_config(path: str) -> bool:
    if path == CONF and not Path(CONF).is_file() and Path(CONF_SAMPLE).is_file():
        Path(CONF).parent.mkdir(parents=True, exist_ok=True)
        # The config may carry credentials; keep the sample's owner-only
        # mode (copyfile does not preserve permissions).
        shutil.copyfile(CONF_SAMPLE, CONF)
        os.chmod(CONF, 0o600)
    if not Path(path).is_file():
        logger.error('config file %s not found', path)
        return False
    parse(path)
    if not get_cfg('SERVICE.SERVER_URL'):
        logger.error('config %s has no SERVICE section', path)
        return False
    ffmachine.setup_captures('gfcloud')
    return True


def main() -> int:
    ap = argparse.ArgumentParser(description='ForgeFIRM full Glowforge cloud controller')
    ap.add_argument('-c', '--config', default=CONF, help='config file (default %s)' % CONF)
    ap.add_argument('--offline', action='store_true',
                    help='no web service: take actions on %s (also when %s exists)'
                         % (OFFLINE_SOCKET, OFFLINE_MARKER))
    args = ap.parse_args()
    offline = args.offline or Path(OFFLINE_MARKER).exists()

    # Logging first: syslog under the gfcloud program name, level from
    # /data/forgefirm.conf (log_gfcloud_disk / _remote).
    ffmachine.setup_logging('gfcloud')
    if not load_config(args.config):
        return 1

    ffmachine.apply_identity_overrides()

    # Machine() reads the OCOTP identity and head info; it fails cleanly if
    # grblHAL still holds /dev/glowforge (controller_mode must be cloud).
    try:
        machine = ffmachine.build_machine()
    except Exception:
        logger.exception('machine init failed (is grblHAL still running? '
                         'controller_mode must be cloud)')
        return 1

    if offline:
        from gfutilities.service.offline import OfflineService
        logger.info('offline: actions from %s, no web service', OFFLINE_SOCKET)
        service = OfflineService(machine, path=OFFLINE_SOCKET)
    else:
        service = GFUIService(machine)

    def _shutdown(*_):
        logger.info('shutdown requested')
        service.request_stop()

    signal.signal(signal.SIGTERM, _shutdown)
    signal.signal(signal.SIGINT, _shutdown)

    # Run for the life of the daemon. connect() can fail if the network or
    # service is briefly unavailable (e.g. at boot); retry until stopped.
    # Once connected, run() stays up across WS drops (gfutilities reconnects
    # with a fresh token) and returns only when a stop is requested, having
    # shut the machine down safe.
    while not service.stop:
        if service.connect():
            service.run()
            break
        logger.error('connect failed; retrying in 10s')
        for _ in range(100):
            if service.stop:
                break
            time.sleep(0.1)

    logger.info('gfcloud exit')
    return 0


if __name__ == '__main__':
    sys.exit(main())

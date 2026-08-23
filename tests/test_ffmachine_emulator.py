"""
The emulator built for the board: gfutilities' Emulator in this machine's
identity, answering the service with a full settings report so the
connect-time hunt is sent (an empty report asks the service to skip
homing, which the protocol test must not do).

Host-side: the fuse identity is stubbed, gfutilities comes from the
sibling checkout, nothing touches hardware.

(C) Copyright 2026
Scott Wiederhold, s.e.wiederhold@gmail.com
SPDX-License-Identifier: MIT
"""
import json
import os
import sys
import tempfile
import types
import unittest
from queue import Queue

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'forgefirm-app'))
sys.path.insert(0, os.path.join(os.path.dirname(ROOT), 'Glowforge-Utilities'))

# The package __init__ imports the hardware modules; register a bare
# package with a fake fuse identity instead.
_pkg = types.ModuleType('gfhardware')
_pkg.__path__ = [os.path.join(ROOT, 'gfhardware')]
sys.modules['gfhardware'] = _pkg
_id = types.ModuleType('gfhardware.id')
_id.serial = lambda: 123456
_id.hostname = lambda: 'BBB-CCC'
_id.password = lambda: 'f' * 64
sys.modules['gfhardware.id'] = _id

import ffmachine                                                  # noqa: E402
from gfutilities.configuration import get_cfg, set_cfg            # noqa: E402
from gfutilities.device import settings as settings_mod           # noqa: E402


class EmulatorBuild(unittest.TestCase):
    def setUp(self):
        self.work = tempfile.mkdtemp()
        # build_emulator imports the identity at call time: other suites
        # install their own stub, so pin this one per test.
        sys.modules['gfhardware.id'] = _id
        sys.modules['gfhardware'].id = _id
        for key in ('MACHINE.SERIAL', 'MACHINE.HOSTNAME', 'MACHINE.PASSWORD',
                    'SETTINGS.SET', 'EMULATOR.ACTIVE', 'EMULATOR.BYPASS_HOMING'):
            set_cfg(key, None)

    def test_identity_is_the_machines(self):
        ffmachine.build_emulator(self.work, self.work)
        self.assertEqual(get_cfg('MACHINE.SERIAL'), 123456)
        self.assertEqual(get_cfg('MACHINE.HOSTNAME'), 'BBB-CCC')
        self.assertEqual(get_cfg('MACHINE.PASSWORD'), 'f' * 64)
        self.assertTrue(get_cfg('EMULATOR.ACTIVE'))
        self.assertEqual(get_cfg('EMULATOR.IMAGE_SRC_DIR'), self.work)

    def test_a_config_override_wins_over_the_fuses(self):
        set_cfg('MACHINE.SERIAL', 999)
        ffmachine.build_emulator(self.work, self.work)
        self.assertEqual(get_cfg('MACHINE.SERIAL'), 999)

    def test_settings_report_carries_the_values_so_the_service_hunts(self):
        ffmachine.build_emulator(self.work, self.work)
        q = Queue()
        settings_mod.send_report(q, {'id': 77})
        frame = json.loads(q.get_nowait())
        self.assertEqual(frame['event'], 'settings:completed')
        self.assertEqual(frame['action_id'], 77)
        values = frame['settings'].get('values')
        self.assertTrue(values, 'an empty settings report asks the service to skip its hunt: %s'
                        % frame['settings'])
        self.assertEqual(values['SAid'], 77)
        self.assertIn('MCsn', values)


if __name__ == '__main__':
    unittest.main()

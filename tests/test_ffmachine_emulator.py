"""
The emulator built for the board: gfutilities' Emulator in this machine's
identity, answering the service with a full settings report so the
connect-time hunt is sent (an empty report asks the service to skip
homing, which the protocol test must not do). And the opposite lever for
the hardware machine: inhibit_connect_hunt makes the first report the
reconnect form, so the service keeps its head position.

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


def real_coolsvc():
    """The real cooling client as a fresh module instance: other suites
    install a fake under the same name, and the emulator's reporter must
    never touch that one."""
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        'gfhardware.coolsvc', os.path.join(ROOT, 'gfhardware', 'coolsvc.py'))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class EmulatorBuild(unittest.TestCase):
    def setUp(self):
        self.work = tempfile.mkdtemp()
        # build_emulator imports the identity and the cooling client at
        # call time: other suites install their own stubs, so pin these
        # per test.
        sys.modules['gfhardware.id'] = _id
        sys.modules['gfhardware'].id = _id
        self.coolsvc = real_coolsvc()
        self.saved_coolsvc = sys.modules.get('gfhardware.coolsvc')
        sys.modules['gfhardware.coolsvc'] = self.coolsvc
        for key in ('MACHINE.SERIAL', 'MACHINE.HOSTNAME', 'MACHINE.PASSWORD',
                    'SETTINGS.SET', 'EMULATOR.ACTIVE', 'EMULATOR.BYPASS_HOMING'):
            set_cfg(key, None)

    def tearDown(self):
        self.coolsvc.cooling_svc.stop = True
        if self.saved_coolsvc is not None:
            sys.modules['gfhardware.coolsvc'] = self.saved_coolsvc
        else:
            sys.modules.pop('gfhardware.coolsvc', None)

    def test_identity_is_the_machines(self):
        ffmachine.build_emulator(self.work, self.work)
        self.assertEqual(get_cfg('MACHINE.SERIAL'), 123456)
        self.assertEqual(get_cfg('MACHINE.HOSTNAME'), 'BBB-CCC')
        self.assertEqual(get_cfg('MACHINE.PASSWORD'), 'f' * 64)
        self.assertTrue(get_cfg('EMULATOR.ACTIVE'))
        self.assertEqual(get_cfg('EMULATOR.IMAGE_SRC_DIR'), self.work)

    def test_the_emulator_reports_idle_to_the_cooling_engine(self):
        svc = self.coolsvc.cooling_svc
        ffmachine.build_emulator(self.work, self.work)
        self.assertTrue(svc.is_alive())                          # the 1 Hz reporter is up
        self.assertEqual(svc._mode, 'idle')
        self.assertFalse(svc.armed)
        ffmachine.build_emulator(self.work, self.work)           # a second build does not restart it

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


class NoHunt(unittest.TestCase):
    def setUp(self):
        for key in ('SETTINGS.SET', 'EMULATOR.ACTIVE', 'EMULATOR.BYPASS_HOMING'):
            set_cfg(key, None)

    def _report(self):
        q = Queue()
        settings_mod.send_report(q, {'id': 5})
        return json.loads(q.get_nowait())['settings']

    def test_the_first_report_carries_the_values_by_default(self):
        self.assertTrue(self._report().get('values'))

    def test_inhibit_connect_hunt_makes_the_first_report_the_reconnect_form(self):
        ffmachine.inhibit_connect_hunt()
        self.assertEqual(self._report(), {})
        # and every report after it, as on a reconnect
        self.assertEqual(self._report(), {})


class OneStartMarkers(unittest.TestCase):
    def test_a_marker_is_read_and_taken_down_by_the_start_that_found_it(self):
        import gfcloud
        d = tempfile.mkdtemp()
        present, absent = os.path.join(d, 'nohunt'), os.path.join(d, 'offline')
        open(present, 'w').close()
        found = gfcloud.take_markers((present, absent))
        self.assertEqual(found, {present: True, absent: False})
        self.assertFalse(os.path.exists(present))          # this start only
        self.assertEqual(gfcloud.take_markers((present, absent)), {present: False, absent: False})

    def test_the_markers_are_read_before_the_slow_imports(self):
        src = open(os.path.join(ROOT, 'forgefirm-app', 'gfcloud.py')).read()
        self.assertLess(src.index('MARKERS = take_markers()'), src.index('from gfutilities'))


if __name__ == '__main__':
    unittest.main()

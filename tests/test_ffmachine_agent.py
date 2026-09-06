"""ffmachine.apply_user_agent: the User-Agent the Glowforge service sees.

ForgeFIRM presents itself as ForgeFIRM/<version>, the version from the
image stamp: a release stamp "v0.0.1" gives ForgeFIRM/0.0.1, the dev
image's "<timestamp> (dev)" stamp is kept as it is, and an unreadable or
empty stamp gives ForgeFIRM/unknown. A user_agent set in the app config
wins; an empty one does not.

(C) Copyright 2026
Scott Wiederhold, s.e.wiederhold@gmail.com
SPDX-License-Identifier: MIT
"""
import os
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'forgefirm-app'))
sys.path.insert(0, os.path.join(os.path.dirname(ROOT), 'Glowforge-Utilities'))

import ffmachine                                                  # noqa: E402
from gfutilities.configuration import get_cfg, set_cfg           # noqa: E402


class UserAgentTest(unittest.TestCase):
    def setUp(self):
        set_cfg('SERVICE.USER_AGENT', None)
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)

    def stamp(self, text):
        path = os.path.join(self.tmp.name, 'forgefirm-version')
        with open(path, 'w') as f:
            f.write(text)
        return path

    def test_release_stamp_drops_the_v(self):
        self.assertEqual(ffmachine.forgefirm_version(self.stamp('v0.0.1\n')), '0.0.1')

    def test_dev_stamp_is_kept(self):
        self.assertEqual(ffmachine.forgefirm_version(self.stamp('20260906123456 (dev)\n')),
                         '20260906123456 (dev)')

    def test_missing_stamp(self):
        path = os.path.join(self.tmp.name, 'absent')
        self.assertEqual(ffmachine.forgefirm_version(path), 'unknown')

    def test_empty_stamp(self):
        self.assertEqual(ffmachine.forgefirm_version(self.stamp('\n')), 'unknown')

    def test_apply_sets_the_forgefirm_agent(self):
        ffmachine.apply_user_agent(self.stamp('v0.0.1\n'))
        self.assertEqual(get_cfg('SERVICE.USER_AGENT'), 'ForgeFIRM/0.0.1')

    def test_config_user_agent_wins(self):
        set_cfg('SERVICE.USER_AGENT', 'Custom/1')
        ffmachine.apply_user_agent(self.stamp('v0.0.1\n'))
        self.assertEqual(get_cfg('SERVICE.USER_AGENT'), 'Custom/1')

    def test_empty_config_user_agent_falls_to_forgefirm(self):
        set_cfg('SERVICE.USER_AGENT', '')
        ffmachine.apply_user_agent(self.stamp('v0.0.1\n'))
        self.assertEqual(get_cfg('SERVICE.USER_AGENT'), 'ForgeFIRM/0.0.1')

    def test_session_carries_the_forgefirm_agent(self):
        from gfutilities.service import websocket
        ffmachine.apply_user_agent(self.stamp('v0.0.1\n'))
        self.assertEqual(websocket.get_session().headers['user-agent'], 'ForgeFIRM/0.0.1')


if __name__ == '__main__':
    unittest.main()

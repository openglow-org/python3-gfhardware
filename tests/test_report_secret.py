# Copyright 2026 514 LLC d/b/a OpenGlow
# Written by Scott Wiederhold
# https://community.openglow.org
# SPDX-License-Identifier:    MIT
"""The cooling report channel is the running controller's alone: the
supervisor hands the controller a secret at its spawn, and every report
carries it. What is read once leaves the environment, so nothing the
client starts inherits it; a value that is not 32 hex digits is never put
in a header.
"""
import os
import sys
import types
import unittest
from unittest import mock

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(os.path.dirname(ROOT), 'Glowforge-Utilities'))

_pkg = types.ModuleType('gfhardware')
_pkg.__path__ = [os.path.join(ROOT, 'gfhardware')]
sys.modules.setdefault('gfhardware', _pkg)

from gfhardware import coolsvc  # noqa: E402
from gfhardware.coolsvc import CoolingService  # noqa: E402

SECRET = '0123456789abcdef0123456789abcdef'


class ReportSecretTests(unittest.TestCase):
    def setUp(self):
        self.sent = []

        def fake_urlopen(req, timeout=None):
            self.sent.append(req)
            return mock.MagicMock()
        self.patcher = mock.patch.object(coolsvc.request, 'urlopen', fake_urlopen)
        self.patcher.start()
        self.addCleanup(self.patcher.stop)
        self.addCleanup(os.environ.pop, 'GF_REPORT_SECRET', None)

    def headers(self):
        return [{k.lower(): v for k, v in r.header_items()} for r in self.sent]

    def test_every_report_carries_the_secret(self):
        os.environ['GF_REPORT_SECRET'] = SECRET
        svc = CoolingService()
        svc.set_mode('run')
        svc.set_armed(True)
        svc.report()
        self.assertEqual(len(self.sent), 3)
        for h in self.headers():
            self.assertEqual(h.get('x-forgefirm-report'), SECRET)

    def test_the_secret_leaves_the_environment_once_read(self):
        os.environ['GF_REPORT_SECRET'] = SECRET
        CoolingService()
        self.assertNotIn('GF_REPORT_SECRET', os.environ)

    def test_no_secret_no_header(self):
        os.environ.pop('GF_REPORT_SECRET', None)
        svc = CoolingService()
        svc.report()
        self.assertNotIn('x-forgefirm-report', self.headers()[0])

    def test_a_value_that_is_not_32_hex_digits_is_never_a_header(self):
        for bad in ('short', SECRET[:-1] + 'G', SECRET + '0', SECRET[:-1] + '\n', SECRET.upper(),
                    SECRET[:16] + '\r\nX-Evil: 1'):
            self.sent.clear()
            os.environ['GF_REPORT_SECRET'] = bad
            svc = CoolingService()
            svc.report()
            h = self.headers()[0]
            self.assertNotIn('x-forgefirm-report', h, repr(bad))
            self.assertNotIn('x-evil', h, repr(bad))
            self.assertNotIn('GF_REPORT_SECRET', os.environ)


if __name__ == '__main__':
    unittest.main()

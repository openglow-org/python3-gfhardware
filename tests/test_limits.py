"""The per-job limits the cloud client hands the cooling engine: what a
pulse header's envelope tags become as /cool/state parameters, what a
sentinel or an absurd value becomes (nothing), and that the limits ride
every report while a job is loaded and leave with it.
"""
import os
import sys
import types
import unittest
from unittest import mock
from urllib import parse

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(os.path.dirname(ROOT), 'Glowforge-Utilities'))

_pkg = types.ModuleType('gfhardware')
_pkg.__path__ = [os.path.join(ROOT, 'gfhardware')]
sys.modules['gfhardware'] = _pkg

from gfhardware import coolsvc  # noqa: E402
from gfhardware.coolsvc import limits_from_header, CoolingService  # noqa: E402

# A cut job's header, as captured: real fan duties, a real air-assist tach
# maximum, the coolant window in millidegrees, empty exhaust/intake windows.
CUT = {'AArd': 1023, 'AArn': 0, 'AArx': 64500, 'EFrd': 65535, 'EFrn': 0, 'EFrx': 0,
       'IFrd': 43278, 'IFrn': 0, 'IFrx': 0, 'CMrn': 5000, 'CMrx': 33000, 'PTmn': 0, 'PTmx': 1023}


class LimitsFromHeaderTests(unittest.TestCase):
    def test_a_cut_header_yields_the_coolant_window_and_the_air_assist_floor(self):
        self.assertEqual(limits_from_header(CUT),
                         {'coolant_max_c': 33.0, 'coolant_min_c': 5.0, 'air_assist_min_rpm': 116})
        self.assertEqual(limits_from_header({'CMrx': 28500}), {'coolant_max_c': 28.5})   # millidegrees

    def test_a_tach_maximum_period_is_a_minimum_speed_in_the_kernels_units(self):
        # exhaust/intake: nanoseconds at 2 pulses per rev; air assist:
        # microseconds at 8 pulses per rev.
        self.assertEqual(limits_from_header({'EFrx': 30000000}), {'exhaust_min_rpm': 1000})
        self.assertEqual(limits_from_header({'IFrx': 60000000}), {'intake_min_rpm': 500})
        self.assertEqual(limits_from_header({'AArx': 7500}), {'air_assist_min_rpm': 1000})

    def test_sentinels_yield_nothing(self):
        for v in (0, 1023, 0x7fffffff, 0x80000000, 0xffffffff):
            for tag in ('CMrx', 'CMrn', 'EFrx', 'IFrx', 'AArx'):
                self.assertEqual(limits_from_header({tag: v}), {}, '%s=%s' % (tag, v))

    def test_absent_negative_and_non_integer_values_yield_nothing(self):
        self.assertEqual(limits_from_header({}), {})
        self.assertEqual(limits_from_header({'CMrx': -5}), {})
        self.assertEqual(limits_from_header({'CMrx': '33000'}), {})
        self.assertEqual(limits_from_header({'CMrx': 33000.0}), {})
        self.assertEqual(limits_from_header({'CMrx': True}), {})

    def test_values_no_machine_could_mean_yield_nothing(self):
        self.assertEqual(limits_from_header({'CMrx': 100000}), {})       # 100 C
        self.assertEqual(limits_from_header({'EFrx': 200000}), {})       # 150000 rpm
        self.assertEqual(limits_from_header({'AArx': 2000000000}), {})   # under 1 rpm

    def test_the_minimum_period_tags_are_read_but_yield_nothing(self):
        self.assertEqual(limits_from_header({'AArn': 5000, 'EFrn': 5000, 'IFrn': 5000}), {})


class ReportTests(unittest.TestCase):
    def setUp(self):
        self.svc = CoolingService()
        self.urls = []

        def fake_urlopen(req, timeout=None):
            self.urls.append(req.full_url)
            return mock.Mock(close=lambda: None)
        self.patcher = mock.patch.object(coolsvc.request, 'urlopen', fake_urlopen)
        self.patcher.start()

    def tearDown(self):
        self.patcher.stop()

    def params(self):
        return dict(parse.parse_qsl(self.urls[-1].split('?', 1)[1]))

    def test_limits_ride_every_report_with_the_profile(self):
        self.svc.profile_exhaust(65535)
        self.svc.set_limits(limits_from_header(CUT))
        self.svc.report()
        self.svc.report()
        self.assertEqual(len(self.urls), 2)
        p = self.params()
        self.assertEqual(p['mode'], 'idle')
        self.assertEqual(p['exhaust'], '65535')
        self.assertEqual(p['coolant_max_c'], '33.0')
        self.assertEqual(p['coolant_min_c'], '5.0')
        self.assertEqual(p['air_assist_min_rpm'], '116')
        self.assertNotIn('exhaust_min_rpm', p)

    def test_cleared_limits_leave_the_report(self):
        self.svc.set_limits(limits_from_header({'AArd': 204}))      # a job with no limits
        self.svc.report()
        self.assertEqual(set(self.params()), {'mode', 'armed'})
        self.svc.set_limits({'coolant_max_c': 30.0})
        self.svc.report()
        self.assertIn('coolant_max_c', self.params())
        self.svc.clear_limits()
        self.svc.report()
        self.assertNotIn('coolant_max_c', self.params())


if __name__ == '__main__':
    unittest.main()

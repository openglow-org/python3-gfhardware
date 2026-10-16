"""The per-job limits the cloud client hands the cooling engine: what a
pulse header's envelope tags become as /cool/state parameters, what a
sentinel or an absurd value becomes (nothing), and that the limits ride
every report while a job is loaded and leave with it. Also the engine's
verdict file as the client reads it: the 2 s window on the monotonic
clock, the fail-safe value of an absent key, and a missing or torn
document.
"""
import math
import os
import sys
import tempfile
import time
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


# The document as the engine publishes it: the keys in its order, the
# booleans, ts_mono to the millisecond, the trailing newline.
ENGINE_FORM = ('{"seq":%d,"ts_mono":%.3f,"fire_ok":%s,"verdict":"%s","hold":%s,'
               '"resume_ok":%s,"armed":%s,"reason":"%s","down_c":%.2f,"up_c":%.2f}\n')


def _b(v):
    return 'true' if v else 'false'


@unittest.skipUnless(hasattr(time, 'clock_gettime'), 'the verdict clock is CLOCK_MONOTONIC')
class VerdictFileTests(unittest.TestCase):
    """The verdict file as the engine writes it and the client reads it."""

    def setUp(self):
        self.dir = tempfile.mkdtemp()
        self.path = os.path.join(self.dir, 'cooling.state')
        self.patcher = mock.patch.object(coolsvc, 'VERDICT_FILE', self.path)
        self.patcher.start()
        self.svc = CoolingService()

    def tearDown(self):
        self.patcher.stop()
        for f in os.listdir(self.dir):
            os.remove(os.path.join(self.dir, f))
        os.rmdir(self.dir)

    @staticmethod
    def stamp(age_s=0.0):
        # The clock floored to the millisecond, as the engine stamps it, so
        # a %.3f stamp never leads the clock it is read against.
        return math.floor(time.clock_gettime(time.CLOCK_MONOTONIC) * 1000) / 1000 - age_s

    def publish(self, fire_ok=True, verdict='OK', hold=False, armed=True, reason='',
                age_s=0.0, seq=1):
        # resume_ok is the hold's complement, as the engine derives it.
        self.write(ENGINE_FORM % (seq, self.stamp(age_s), _b(fire_ok), verdict, _b(hold),
                                  _b(not hold), _b(armed), reason, 21.3, 21.1))

    def write(self, body):
        # Atomic replace, as the engine publishes.
        tmp = self.path + '.tmp'
        with open(tmp, 'w') as f:
            f.write(body)
        os.replace(tmp, self.path)

    def test_a_clean_verdict_permits(self):
        self.publish()
        v = self.svc.verdict()
        self.assertEqual((v['fire_ok'], v['hold'], v['resume_ok'], v['armed'], v['verdict']),
                         (True, False, True, True, 'OK'))
        self.assertTrue(self.svc.fire_ok())
        self.assertFalse(self.svc.hold())
        self.assertTrue(self.svc.resume_ok())

    def test_a_blocking_verdict_holds(self):
        self.publish(fire_ok=False, verdict='WARMUP', hold=True,
                     reason='coolant 18.0 C under the 21.0 C start gate')
        self.assertFalse(self.svc.fire_ok())
        self.assertTrue(self.svc.hold())
        self.assertFalse(self.svc.resume_ok())
        self.assertEqual(self.svc.verdict()['verdict'], 'WARMUP')

    def test_the_window_is_two_seconds_on_the_monotonic_clock(self):
        self.publish(age_s=1.9)
        self.assertIsNotNone(self.svc.verdict())
        # A clean document past the window is no verdict: fire blocked, hold.
        self.publish(age_s=2.1)
        self.assertIsNone(self.svc.verdict())
        self.assertFalse(self.svc.fire_ok())
        self.assertTrue(self.svc.hold())
        self.assertFalse(self.svc.resume_ok())

    def test_a_stamp_ahead_of_the_clock_is_no_verdict(self):
        # Future-dated, it would otherwise read as fresh for good.
        for ahead_s in (0.25, 5.0):
            with self.subTest(ahead_s=ahead_s):
                self.publish(age_s=-ahead_s)
                self.assertIsNone(self.svc.verdict())
                self.assertTrue(self.svc.hold())

    def test_a_missing_torn_or_wrong_document_holds(self):
        self.assertIsNone(self.svc.verdict())           # no file
        self.assertTrue(self.svc.hold())
        now = self.stamp()
        for body in ('{"seq":1,"ts_mono":%.3f,"fire_ok":true' % now,     # torn: no closing brace
                     '[]\n',                                              # not an object
                     '{"fire_ok":true}\n',                                # no ts_mono
                     '{"ts_mono":"soon","fire_ok":true}\n'):              # ts_mono not a number
            with self.subTest(body=body):
                self.write(body)
                self.assertIsNone(self.svc.verdict())
                self.assertTrue(self.svc.hold())

    def test_an_absent_key_takes_the_fail_safe_value(self):
        self.write('{"ts_mono":%.3f}\n' % self.stamp())
        self.assertFalse(self.svc.fire_ok())
        self.assertTrue(self.svc.hold())
        self.assertFalse(self.svc.resume_ok())
        # Never a resume while the engine still asks for the hold.
        self.write('{"ts_mono":%.3f,"fire_ok":true,"hold":true,"resume_ok":true}\n' % self.stamp())
        self.assertFalse(self.svc.resume_ok())
        # A resume takes both keys: an absent hold still holds, and an
        # absent resume_ok offers none.
        self.write('{"ts_mono":%.3f,"fire_ok":true,"resume_ok":true}\n' % self.stamp())
        self.assertFalse(self.svc.resume_ok())
        self.write('{"ts_mono":%.3f,"fire_ok":true,"hold":false}\n' % self.stamp())
        self.assertFalse(self.svc.resume_ok())


if __name__ == '__main__':
    unittest.main()

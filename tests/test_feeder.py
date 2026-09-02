"""
(C) Copyright 2026
Scott Wiederhold, s.e.wiederhold@gmail.com
https://community.openglow.org

SPDX-License-Identifier:    MIT

Host tests for the pulse feeder: a print can be several times longer than
the kernel ring, so the ring is a window onto the job rather than the place
the job lives. These drive the feeder against a fake ring that refuses with
-ENOMEM when full and drains on demand, which is the only behavior of the
device the feeder depends on.

Run:  PYTHONPATH=.:../Glowforge-Utilities python3 -m unittest tests.test_feeder
"""
import errno
import os
import sys
import time
import types
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(os.path.dirname(ROOT), 'Glowforge-Utilities'))

# The package __init__ imports the machine and its hardware modules; register
# a bare package and a fake kernel wrapper first, then import the feeder.
_pkg = types.ModuleType('gfhardware')
_pkg.__path__ = [os.path.join(ROOT, 'gfhardware')]
sys.modules['gfhardware'] = _pkg


class FakeCNC:
    def __init__(self):
        self.streaming_writes = []

    def set_streaming(self, val):
        self.streaming_writes.append(int(val))

    @property
    def streaming(self):
        return bool(self.streaming_writes and self.streaming_writes[-1])


_cnc_mod = types.ModuleType('gfhardware.cnc')
_cnc_mod.cnc = FakeCNC()
sys.modules['gfhardware.cnc'] = _cnc_mod

from gfutilities.puls import decode_all_steps                    # noqa: E402
from gfutilities.puls.source import PulseSource                  # noqa: E402
import gfhardware.feeder as feeder_mod                           # noqa: E402
from gfhardware.feeder import PulseFeeder                        # noqa: E402

CNC = _cnc_mod.cnc


def _puls(payload: bytes) -> bytes:
    fields = (b'STfr' + (10000).to_bytes(4, 'little')
              + b'MCsn' + (0).to_bytes(4, 'little')
              + b'PDfm' + (0).to_bytes(4, 'little'))
    return b'\x80GF1' + (8 + len(fields)).to_bytes(4, 'little') + fields + payload


class FakeRing:
    """The pulse device: accepts writes until the ring is full, then refuses
    with -ENOMEM until something drains it. Keeps every byte it accepted, in
    order, so the job can be checked against what was sent."""

    def __init__(self, capacity):
        self.capacity = capacity
        self.accepted = bytearray()
        self.in_ring = 0
        self.fail_with = None

    def write(self, chunk):
        if self.fail_with is not None:
            raise self.fail_with
        if self.in_ring + len(chunk) > self.capacity:
            raise OSError(errno.ENOMEM, 'Cannot allocate memory')
        self.in_ring += len(chunk)
        self.accepted += chunk

    def drain(self, count):
        self.in_ring = max(0, self.in_ring - count)


class _Declaring:
    """A job source that says how long it is, right or wrong."""

    def __init__(self, data: bytes, program_size):
        self._data = data
        self._at = 0
        self.program_size = program_size

    def read(self, count: int) -> bytes:
        out = self._data[self._at:self._at + count]
        self._at += len(out)
        return out


def _wait(pred, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if pred():
            return True
        time.sleep(0.005)
    return False


class FeederTest(unittest.TestCase):
    def setUp(self):
        CNC.streaming_writes = []

    def _feeder(self, payload, capacity, chunk=1024):
        source = PulseSource(_puls(payload))
        ring = FakeRing(capacity)
        return PulseFeeder(source, ring, chunk=chunk, retry_s=0.01), ring

    # -- the ordinary job ------------------------------------------------
    def test_job_that_fits_is_enqueued_whole_and_is_not_a_live_feed(self):
        payload = bytes(range(256)) * 20                 # 5120 bytes
        feeder, ring = self._feeder(payload, capacity=1 << 20)
        feeder.start()
        self.assertTrue(feeder.wait_primed(timeout=5))
        self.assertTrue(_wait(lambda: feeder.finished))
        feeder.stop()
        self.assertEqual(bytes(ring.accepted), payload)
        self.assertEqual(feeder.written, len(payload))
        # Never declared live: end-of-data means the job finished, which is
        # exactly the behavior a job that fits has always had.
        self.assertFalse(feeder.streaming)
        self.assertEqual(CNC.streaming_writes, [])

    # -- the job that outruns the ring -----------------------------------
    def test_job_longer_than_the_ring_is_fed_as_it_plays(self):
        payload = bytes(range(256)) * 400                # 102400 bytes
        capacity = 8192                                  # ring holds 8 KB
        feeder, ring = self._feeder(payload, capacity, chunk=1024)
        feeder.start()

        # Priming stops at a full ring, with the job far from finished.
        self.assertTrue(feeder.wait_primed(timeout=5))
        self.assertFalse(feeder.finished)
        self.assertEqual(feeder.written, capacity)

        feeder.declare_live_feed()
        self.assertEqual(CNC.streaming_writes, [1])

        # Play it: every time the ring drains, the feeder tops it up.
        deadline = time.monotonic() + 20
        while not feeder.finished and time.monotonic() < deadline:
            ring.drain(4096)
            time.sleep(0.005)
        self.assertTrue(feeder.finished, 'feeder never finished the job')

        # The whole job arrived, in order, and the device was told the feed
        # is over only after the last bytes were in.
        self.assertEqual(bytes(ring.accepted), payload)
        self.assertEqual(feeder.written, len(payload))
        self.assertEqual(CNC.streaming_writes, [1, 0])
        feeder.stop()

    def test_ring_full_is_not_an_error(self):
        payload = bytes(200000)
        feeder, ring = self._feeder(payload, capacity=4096)
        feeder.start()
        self.assertTrue(feeder.wait_primed(timeout=5))
        time.sleep(0.1)                                  # sit against a full ring
        self.assertIsNone(feeder.error)
        self.assertEqual(feeder.written, 4096)
        feeder.stop()

    # -- how long the job is ---------------------------------------------
    def test_the_job_total_is_the_job_before_the_feed_finishes(self):
        # What a progress report divides by, and the reason it can be
        # reported from the first frame: the length is known while most of
        # the job is still waiting to be fed.
        payload = bytes(range(256)) * 400                # 102400 bytes
        feeder, ring = self._feeder(payload, capacity=8192, chunk=1024)
        feeder.start()
        self.assertTrue(feeder.wait_primed(timeout=5))
        self.assertFalse(feeder.finished)
        self.assertEqual(feeder.job_total, len(payload))
        feeder.stop()

    def test_a_finished_feed_reports_what_it_actually_delivered(self):
        payload = bytes(range(256)) * 20
        feeder, ring = self._feeder(payload, capacity=1 << 20)
        feeder.start()
        self.assertTrue(_wait(lambda: feeder.finished))
        self.assertEqual(feeder.job_total, len(payload))
        self.assertEqual(feeder.job_total, feeder.written)
        feeder.stop()

    def test_a_job_that_will_not_say_how_long_it_is_reports_no_total(self):
        feeder = PulseFeeder(_Declaring(bytes(4096), None), FakeRing(1 << 20),
                             chunk=1024, retry_s=0.01)
        self.assertIsNone(feeder.job_total)

    def test_a_job_that_does_not_end_where_it_said_it_would_is_named(self):
        # The declared length is what every progress frame divided by all
        # job long, so a job that delivers something else is worth a line.
        feeder = PulseFeeder(_Declaring(bytes(4096), 9999), FakeRing(1 << 20),
                             chunk=1024, retry_s=0.01)
        with self.assertLogs(feeder_mod.logger, level='WARNING') as caught:
            feeder.start()
            self.assertTrue(_wait(lambda: feeder.finished))
        feeder.stop()
        self.assertTrue([ln for ln in caught.output
                         if 'declared 9999' in ln and '4096' in ln], caught.output)

    # -- accounting ------------------------------------------------------
    def test_step_totals_match_decoding_the_whole_job(self):
        payload = bytes(range(256)) * 40
        feeder, ring = self._feeder(payload, capacity=1 << 20)
        feeder.start()
        self.assertTrue(_wait(lambda: feeder.finished))
        self.assertTrue(_wait(lambda: feeder.stats is not None
                              and feeder.stats['XTOT'] == decode_all_steps(payload)['XTOT']))
        feeder.stop()
        self.assertEqual(feeder.stats, decode_all_steps(payload))

    # -- failures --------------------------------------------------------
    def test_write_error_stops_the_feed_and_is_reported(self):
        payload = bytes(4096)
        feeder, ring = self._feeder(payload, capacity=1 << 20)
        ring.fail_with = OSError(errno.EIO, 'I/O error')
        feeder.start()
        self.assertFalse(feeder.wait_primed(timeout=5))
        self.assertIsInstance(feeder.error, OSError)
        self.assertEqual(feeder.error.errno, errno.EIO)
        self.assertFalse(feeder.finished)
        feeder.stop()

    def test_abandoned_live_feed_leaves_the_device_out_of_live_mode(self):
        # A cancelled job must not leave the next one being read as a live
        # feed, where its ordinary end-of-data would look like a starved ring.
        payload = bytes(200000)
        feeder, ring = self._feeder(payload, capacity=4096)
        feeder.start()
        self.assertTrue(feeder.wait_primed(timeout=5))
        feeder.declare_live_feed()
        self.assertEqual(CNC.streaming_writes, [1])
        feeder.stop()
        self.assertEqual(CNC.streaming_writes, [1, 0])
        self.assertFalse(feeder.streaming)


if __name__ == '__main__':
    unittest.main()

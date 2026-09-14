"""
Copyright 2026 514 LLC d/b/a OpenGlow
Written by Scott Wiederhold
https://community.openglow.org

SPDX-License-Identifier:    MIT

Host tests for the one check the feeder makes on a job's bytes on their
way to the ring: a stream must carry a laser power byte before its first
FIRE byte. The kernel resets the duty to about full on every run, so a
stream that fires first would put its first pulses out at full power. The
check runs chunk by chunk as the bytes are read, so the offending byte
never reaches the ring: before the run the job is refused; past the primed
window the feed stops and the run ends on a starved ring.

Run:  PYTHONPATH=.:../Glowforge-Utilities python3 -m unittest tests.test_pulse_body
"""
import os
import sys
import time
import types
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(os.path.dirname(ROOT), 'Glowforge-Utilities'))

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

from gfutilities.puls.source import PulseSource                  # noqa: E402
from gfhardware.feeder import PulseFeeder, PulseBodyError        # noqa: E402

# The pulse byte: bit 7 set is a power level, otherwise a step byte whose
# bit 4 is FIRE.
POWER = b'\x80'
FIRE = b'\x10'
STEP = b'\x01'

# A real capture, where one is at hand (the tree's reverse-engineering
# resources are not part of the repository).
CAPTURE = os.path.join(os.path.dirname(ROOT), '_RESOURCES', '2026-06-17_085021.puls')


def _puls(payload: bytes) -> bytes:
    fields = (b'STfr' + (10000).to_bytes(4, 'little')
              + b'MCsn' + (0).to_bytes(4, 'little')
              + b'PDfm' + (0).to_bytes(4, 'little'))
    return b'\x80GF1' + (8 + len(fields)).to_bytes(4, 'little') + fields + payload


class FakeRing:
    def __init__(self, capacity):
        self.capacity = capacity
        self.accepted = bytearray()
        self.in_ring = 0

    def write(self, chunk):
        if self.in_ring + len(chunk) > self.capacity:
            raise OSError(12, 'Cannot allocate memory')
        self.in_ring += len(chunk)
        self.accepted += chunk

    def drain(self, count):
        self.in_ring = max(0, self.in_ring - count)


def _wait(pred, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if pred():
            return True
        time.sleep(0.005)
    return False


class PulseBodyTests(unittest.TestCase):
    def _feed(self, payload, capacity=1 << 20, chunk=1024):
        ring = FakeRing(capacity)
        feeder = PulseFeeder(PulseSource(_puls(payload)), ring, chunk=chunk, retry_s=0.01)
        feeder.start()
        return feeder, ring

    def test_fire_before_any_power_byte_is_refused_before_the_ring(self):
        payload = STEP * 100 + FIRE + STEP * 50 + POWER + STEP * 20
        feeder, ring = self._feed(payload)
        self.assertFalse(feeder.wait_primed(timeout=5))
        self.assertIsInstance(feeder.error, PulseBodyError)
        self.assertIn('byte 100', str(feeder.error))
        self.assertEqual(len(ring.accepted), 0, 'the refused chunk reached the ring')
        self.assertFalse(feeder.finished)
        feeder.stop()

    def test_a_power_byte_first_is_accepted(self):
        payload = STEP * 10 + POWER + STEP * 100 + FIRE * 50 + STEP * 20
        feeder, ring = self._feed(payload)
        self.assertTrue(feeder.wait_primed(timeout=5))
        self.assertTrue(_wait(lambda: feeder.finished))
        self.assertIsNone(feeder.error)
        self.assertEqual(bytes(ring.accepted), payload)
        feeder.stop()

    def test_the_power_byte_counts_across_chunks(self):
        # The power byte in one chunk licenses the fire in a later one.
        payload = POWER + STEP * 3000 + FIRE * 10 + STEP * 500
        feeder, ring = self._feed(payload, chunk=1024)
        self.assertTrue(feeder.wait_primed(timeout=5))
        self.assertTrue(_wait(lambda: feeder.finished))
        self.assertIsNone(feeder.error)
        self.assertEqual(bytes(ring.accepted), payload)
        feeder.stop()

    def test_a_late_fire_past_the_primed_window_stops_the_feed_short_of_it(self):
        # The ring holds the first window; the offending byte is beyond it.
        # The feed stops before that chunk, so it never reaches the ring.
        payload = STEP * 6000 + FIRE + STEP * 100
        feeder, ring = self._feed(payload, capacity=4096, chunk=1024)
        self.assertTrue(feeder.wait_primed(timeout=5))
        self.assertIsNone(feeder.error)
        ring.drain(4096)
        self.assertTrue(_wait(lambda: feeder.error is not None))
        self.assertIsInstance(feeder.error, PulseBodyError)
        self.assertIn('byte 6000', str(feeder.error))
        self.assertNotIn(FIRE, bytes(ring.accepted))
        self.assertFalse(feeder.finished)
        feeder.stop()

    def test_a_dark_job_passes(self):
        payload = STEP * 5000
        feeder, ring = self._feed(payload)
        self.assertTrue(feeder.wait_primed(timeout=5))
        self.assertTrue(_wait(lambda: feeder.finished))
        self.assertIsNone(feeder.error)
        feeder.stop()

    @unittest.skipUnless(os.path.exists(CAPTURE), 'no capture at hand')
    def test_a_real_capture_passes(self):
        with open(CAPTURE, 'rb') as f:
            body = f.read()
        ring = FakeRing(256 << 20)
        feeder = PulseFeeder(PulseSource(body), ring, chunk=256 * 1024, retry_s=0.01)
        feeder.start()
        self.assertTrue(feeder.wait_primed(timeout=30))
        self.assertTrue(_wait(lambda: feeder.finished, timeout=60))
        self.assertIsNone(feeder.error)
        self.assertGreater(feeder.written, 0)
        feeder.stop()


if __name__ == '__main__':
    unittest.main()

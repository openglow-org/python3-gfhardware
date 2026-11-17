"""
Copyright 2026 514 LLC d/b/a OpenGlow
Written by Scott Wiederhold
https://community.openglow.org

SPDX-License-Identifier:    MIT
"""
import errno
import logging
import re
import threading
from array import array
from time import monotonic

from gfutilities.puls import decode_all_steps

from gfhardware._common import LOGGER_NAME
from gfhardware.cnc import cnc

logger = logging.getLogger(LOGGER_NAME)

# Bytes offered to the ring per write. Large enough that a long job is not
# millions of syscalls, small enough that a refusal costs little.
CHUNK = 256 * 1024

# How long to wait before offering a refused chunk again. The ring drains at
# the step frequency (10 kB/s for a print), so a full ring stays full for
# minutes: there is nothing to gain from asking more often.
RETRY_S = 0.5

# How many enqueued bytes may sit undecoded before the accounting is forced
# to catch up in line with the feed. Set above the largest ring so filling one
# never stalls on it: the backlog is bounded by the ring in any case, since a
# full ring is exactly when the feeder has time to decode.
PENDING_MAX = 48 * 1024 * 1024

_BACKOFF = (errno.ENOMEM, errno.EBUSY, errno.EAGAIN)

# The pulse byte: bit 7 set is a laser power level; bit 7 clear is a step
# byte, and its bit 4 is FIRE. The kernel resets the duty to about full on
# every run, so a stream must carry a power byte before its first FIRE or
# the first pulses fire at full power. The feeder refuses such a stream
# before the FIRE byte reaches the ring.
_POWER_BYTE = re.compile(rb'[\x80-\xff]')
_FIRE_BYTE = re.compile(rb'[\x10-\x1f\x30-\x3f\x50-\x5f\x70-\x7f]')

# The power bytes of everything written, counted per POWER_BLOCK payload
# bytes. A power byte plays in the same tick as the step byte after it, so a
# stretch of the job lasts as many ticks as it holds step bytes, not bytes:
# what a pause needs to size its resume's laser-off lead in ticks.
POWER_BLOCK = 256
_POWER_FLAGS = bytes(1 if b & 0x80 else 0 for b in range(256))


class PulseBodyError(Exception):
    """The job's bytes are not fit for the ring."""


class PulseFeeder:
    """Keeps the kernel pulse ring fed from a job held in memory.

    A print can be several times longer than the ring: three hours of cutting
    is over 100 MB of steps against 32 MiB of buffer. So the ring is not where
    the job lives, it is a window onto it. Fill the window, start the run, and
    top it up as it drains, which is what the machine's own firmware does.

    ``-ENOMEM`` from the write is the only pacing signal needed: it means the
    window is full, and the answer is to wait and offer the same bytes again.
    The ring holds tens of minutes, so the feeder is never the urgent party.

    Step accounting is deferred to the moments when the ring is full and the
    feeder has nothing else to do, so it can never stand between the machine
    and the bytes it is waiting for.
    """

    def __init__(self, source, dev, chunk: int = CHUNK, retry_s: float = RETRY_S,
                 fire_allowed: bool = True):
        """``fire_allowed`` False is a job that never fires (a motion, a
        hunt): any FIRE byte in it is refused before the ring, as a FIRE
        byte before any power byte is in a job that may fire."""
        self._fire_allowed = fire_allowed
        self._source = source
        self._dev = dev
        self._chunk = chunk
        self._retry_s = retry_s
        self._thread = None
        self._stop = threading.Event()
        self._primed = threading.Event()
        self._done = threading.Event()
        self._streaming = False
        self._written = 0
        self._error = None
        self._stats = None
        self._pending = []
        self._pending_bytes = 0
        self._books = threading.Lock()
        self._powered = False           # a power byte has been seen
        self._scanned = 0               # payload bytes checked so far
        self._power_counts = array('H')  # power bytes per POWER_BLOCK written
        self._indexed = 0               # payload bytes counted into it

    # -- state -----------------------------------------------------------
    @property
    def streaming(self) -> bool:
        """True once the job has proved longer than the ring."""
        return self._streaming

    @property
    def written(self) -> int:
        """Payload bytes accepted by the device so far."""
        return self._written

    @property
    def chunk(self) -> int:
        """Bytes offered to the ring per write: ``written`` moves by this
        much at a time, so a live feed shows no progress while one chunk's
        worth of ring drains."""
        return self._chunk

    @property
    def indexed(self) -> int:
        """Payload bytes whose power bytes have been counted: everything the
        ring has taken."""
        return self._indexed

    def lead_bytes(self, start: int, ticks: int):
        """The fewest payload bytes from offset ``start`` whose step bytes
        cover ``ticks`` ticks: what a laser-off lead from there must count to
        last that long. Power bytes are known per POWER_BLOCK only, so they
        are taken to sit at the window's start in every block it touches:
        the answer never falls short. None when the window runs past what has
        been written."""
        counts, indexed = self._power_counts, self._indexed
        if start < 0:
            return None
        block, off = divmod(start, POWER_BLOCK)
        need, length = ticks, 0
        while need > 0:
            span = min(POWER_BLOCK, indexed - block * POWER_BLOCK) - off
            if span <= 0:
                return None
            power = counts[block]
            steps = max(0, span - power)
            if steps >= need:
                return length + min(span, power + need)
            length += span
            need -= steps
            block += 1
            off = 0
        return length

    def _index_power(self, chunk: bytes) -> None:
        """Count the power bytes of a chunk the ring took, per POWER_BLOCK of
        the job's payload."""
        flags = chunk.translate(_POWER_FLAGS)
        pos = 0
        part = self._indexed % POWER_BLOCK
        if part:
            pos = min(POWER_BLOCK - part, len(flags))
            self._power_counts[-1] += flags.count(1, 0, pos)
        for at in range(pos, len(flags), POWER_BLOCK):
            self._power_counts.append(flags.count(1, at, at + POWER_BLOCK))
        self._indexed += len(chunk)

    @property
    def error(self) -> Exception:
        """The write error that stopped the feed, if one did."""
        return self._error

    @property
    def stats(self) -> dict:
        """Step and laser totals for the bytes fed so far."""
        return self._stats

    @property
    def finished(self) -> bool:
        """True once every byte of the job has been enqueued."""
        return self._done.is_set()

    @property
    def job_total(self) -> int:
        """How long the whole job is, for anything that reports progress.

        Once the feed is done, what went in is the authority. Before that it
        is what the job declared itself to be, which is known before the
        first byte plays and does not move while it does. None when the job
        will not say, and a caller reports position without a total rather
        than dividing by a guess.
        """
        if self._done.is_set():
            return self._written
        return getattr(self._source, 'program_size', None)

    # -- lifecycle -------------------------------------------------------
    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name='pulse-feeder',
                                        daemon=True)
        self._thread.start()

    def wait_primed(self, timeout: float = 120.0) -> bool:
        """Block until the ring is full or the whole job is in it.

        False on timeout or on a write error, either of which means the job
        must not start.
        """
        remaining = timeout
        while not self._primed.wait(timeout=0.05):
            if self._done.is_set() or self._error is not None:
                break
            remaining -= 0.05
            if remaining <= 0:
                logger.error('feeder did not fill the ring within %.0fs', timeout)
                return False
        return self._error is None

    def declare_live_feed(self) -> None:
        """Tell the device this run is live-fed, before the run starts.

        Only for a job that did not fit: one that fits is enqueued in full and
        keeps the ordinary meaning of end-of-data.
        """
        self._set_streaming(True)

    def stop(self, timeout: float = 5.0) -> None:
        """Stop feeding and leave the device out of live-feed mode."""
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout)
            if self._thread.is_alive():
                logger.error('pulse feeder did not stop')
            self._thread = None
        if self._streaming:
            # An abandoned job must not leave the next one being read as a
            # live feed, where its ordinary end-of-data would be an underrun.
            try:
                self._set_streaming(False)
            except OSError as e:
                logger.error('%s', e)

    # -- the feed --------------------------------------------------------
    def _set_streaming(self, on: bool) -> None:
        """Put the device in or out of live-feed mode, and prove it.

        The write is read back: a declaration that did not land would let
        the device read the first drained window as the job's end. Raises
        OSError when the write fails or the readback disagrees; the caller
        decides what that means for the job (before a run: fatal).
        """
        try:
            cnc.set_streaming(on)
        except OSError as e:
            raise OSError('could not set streaming=%d: %s' % (int(on), e)) from e
        if bool(cnc.streaming) != bool(on):
            raise OSError('streaming=%d did not land: the device reads %d'
                          % (int(on), int(bool(cnc.streaming))))
        self._streaming = on

    def _fit_for_the_ring(self, chunk: bytes) -> bool:
        """Refuse a stream whose first FIRE byte precedes any power byte,
        and any FIRE byte in a job that may not fire.

        Checked on the way to the ring, so the offending byte never reaches
        it: a refusal before the run cancels the job; one past the primed
        window stops the feed, and the run ends on a starved ring rather
        than a full-power pulse.
        """
        if not self._fire_allowed:
            fire = _FIRE_BYTE.search(chunk)
            if fire is not None:
                self._error = PulseBodyError(
                    'laser fire at payload byte %d in a job that never fires'
                    % (self._scanned + fire.start()))
                logger.error('refusing the job: %s', self._error)
                return False
            self._scanned += len(chunk)
            return True
        if self._powered:
            return True
        fire = _FIRE_BYTE.search(chunk)
        power = _POWER_BYTE.search(chunk)
        if fire is not None and (power is None or fire.start() < power.start()):
            offset = self._scanned + fire.start()
            self._error = PulseBodyError(
                'laser fire at payload byte %d before any power byte: the first '
                'pulses would fire at full power' % offset)
            logger.error('refusing the job: %s', self._error)
            return False
        if power is not None:
            self._powered = True
        self._scanned += len(chunk)
        return True

    def _account(self, limit: int = 1) -> None:
        """Decode up to ``limit`` already-enqueued chunks."""
        for _ in range(limit):
            with self._books:
                if not self._pending:
                    return
                chunk = self._pending.pop(0)
                self._pending_bytes -= len(chunk)
                self._stats = decode_all_steps(chunk, self._stats)

    def settle(self, timeout: float = 60.0) -> bool:
        """Finish the step accounting for everything already enqueued.

        The totals are what the end of a job is checked against, so they have
        to cover the whole job before they are read. Called on the job's own
        thread once the feed is complete, where a moment's decoding costs
        nothing.
        """
        deadline = monotonic() + timeout
        while self._pending:
            if monotonic() > deadline:
                logger.warning('step accounting incomplete: %d bytes undecoded',
                               self._pending_bytes)
                return False
            self._account(8)
        return True

    def _write(self, chunk: bytes) -> bool:
        """Offer one chunk; True once written. A full ring simply waits."""
        while not self._stop.is_set():
            try:
                self._dev.write(chunk)
            except OSError as e:
                if e.errno not in _BACKOFF:
                    self._error = e
                    logger.error('pulse write failed after %d bytes: %s',
                                 self._written, e)
                    return False
                # The ring is full, or a pause is backtracking through it.
                # Either way these bytes are still ours to offer again, and
                # the wait is the right moment to catch the accounting up.
                if not self._primed.is_set():
                    logger.info('ring full after %d bytes; the job is longer '
                                'than the ring and will be fed as it plays',
                                self._written)
                    self._primed.set()
                self._account()
                self._stop.wait(self._retry_s)
                continue
            self._index_power(chunk)
            self._written += len(chunk)
            with self._books:
                self._pending.append(chunk)
                self._pending_bytes += len(chunk)
            while self._pending_bytes > PENDING_MAX:
                self._account()
            return True
        return False

    def _run(self) -> None:
        try:
            while not self._stop.is_set():
                chunk = self._source.read(self._chunk)
                if not chunk:
                    break
                if not self._fit_for_the_ring(chunk):
                    return
                if not self._write(chunk):
                    return
            if self._stop.is_set():
                return
            if self._streaming:
                # Every byte is enqueued: the next end-of-data is the end of
                # the job, not a starved ring. A device that will not leave
                # live-feed mode would read the job's own end as one, so
                # the failure is the feed's.
                try:
                    self._set_streaming(False)
                except OSError as e:
                    self._error = e
                    logger.error('%s', e)
                    return
            logger.info('feeder finished: %d bytes enqueued', self._written)
            declared = getattr(self._source, 'program_size', None)
            if declared is not None and declared != self._written:
                # Progress was reported against the declared length all job
                # long, so a job that does not end where it said it would is
                # worth a line: it is the one thing that can put the bar out.
                logger.warning('job declared %d bytes and delivered %d',
                               declared, self._written)
            self._done.set()
            self._primed.set()
            # Whatever accounting is left can finish while the machine plays
            # what it already has.
            while self._pending and not self._stop.is_set():
                self._account()
        except Exception as e:                              # pragma: no cover
            self._error = e
            logger.exception('pulse feeder failed')
        finally:
            self._primed.set()

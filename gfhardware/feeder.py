"""
Copyright 2026 514 LLC d/b/a OpenGlow
Written by Scott Wiederhold
https://community.openglow.org

SPDX-License-Identifier:    MIT
"""
import errno
import logging
import threading
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

    def __init__(self, source, dev, chunk: int = CHUNK, retry_s: float = RETRY_S):
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
            self._set_streaming(False)

    # -- the feed --------------------------------------------------------
    def _set_streaming(self, on: bool) -> None:
        try:
            cnc.set_streaming(on)
            self._streaming = on
        except OSError as e:
            logger.error('could not set streaming=%d: %s', int(on), e)

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
                if not self._write(chunk):
                    return
            if self._stop.is_set():
                return
            if self._streaming:
                # Every byte is enqueued: the next end-of-data is the end of
                # the job, not a starved ring.
                self._set_streaming(False)
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

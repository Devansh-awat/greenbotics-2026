"""Logging setup, shared by both challenges and the drivers.

One logger tree, `robot.*`, shared by the challenge code and the drivers
(`robot.motor` is set up inside src/motors/motor.py). Everything goes through a
QueueHandler so that no `log.*()` call on the control loop can ever block on disk
I/O -- a QueueListener thread does the formatting and writing.

To add your own logging, pick the logger whose area you are in and call it:

    from src.logs.setup import log, vlog
    log.info("turn %d done, heading %.1f", turn_counter, heading)

Use `%s`-style lazy args, not f-strings: the formatting then only happens if the
record is actually emitted. In a tight loop wrap it in a Throttle so you get one
line every N seconds instead of one per frame.
"""

import logging
import logging.handlers
import queue
import sys
import time
# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------
# Everything goes through the `logging` module. The control loop must never block
# on disk I/O, so handlers live behind a QueueHandler/QueueListener: log calls on
# the hot path only append to an in-memory queue, and a background thread does the
# formatting and writing.
#
# Loggers are named per subsystem so a run log can be filtered:
#   robot.control  robot.vision  robot.camera  robot.imu
#   robot.sensors  robot.video   robot.park    robot.perf

log = logging.getLogger("robot.control")
vlog = logging.getLogger("robot.vision")
clog = logging.getLogger("robot.camera")
ilog = logging.getLogger("robot.imu")
slog = logging.getLogger("robot.sensors")
wlog = logging.getLogger("robot.video")
plog = logging.getLogger("robot.park")
perflog = logging.getLogger("robot.perf")

_log_listener = None


def setup_logging(log_path, console_level=logging.INFO, file_level=logging.DEBUG):
    """Install the queue-backed logging pipeline. Returns the QueueListener.

    Console gets INFO and up (readable during a run); the file gets DEBUG and up
    (full per-frame telemetry for post-run diagnosis).
    """
    global _log_listener

    root = logging.getLogger("robot")
    root.setLevel(logging.DEBUG)
    root.propagate = False
    for h in list(root.handlers):
        root.removeHandler(h)

    file_fmt = logging.Formatter(
        "%(asctime)s.%(msecs)03d %(levelname)-5s %(name)-13s %(message)s",
        datefmt="%H:%M:%S",
    )
    console_fmt = logging.Formatter(
        "%(asctime)s.%(msecs)03d %(levelname)-5s %(name)-13s %(message)s",
        datefmt="%H:%M:%S",
    )

    fh = logging.FileHandler(log_path, mode="w")
    fh.setLevel(file_level)
    fh.setFormatter(file_fmt)

    ch = logging.StreamHandler(sys.__stderr__)
    ch.setLevel(console_level)
    ch.setFormatter(console_fmt)

    log_q = queue.Queue(-1)
    qh = logging.handlers.QueueHandler(log_q)
    qh.setLevel(logging.DEBUG)
    root.addHandler(qh)

    _log_listener = logging.handlers.QueueListener(
        log_q, fh, ch, respect_handler_level=True
    )
    _log_listener.start()
    return _log_listener


def shutdown_logging():
    global _log_listener
    if _log_listener is not None:
        _log_listener.stop()
        _log_listener = None


class Throttle:
    """Rate-limiter for logging inside tight control loops.

    The parking routines spin at 100 Hz; logging every iteration floods the file
    and costs real time. `if throttle:` is True at most once per `period` seconds.
    """

    __slots__ = ("period", "_next")

    def __init__(self, period=0.25):
        self.period = period
        self._next = 0.0

    def __bool__(self):
        now = time.monotonic()
        if now >= self._next:
            self._next = now + self.period
            return True
        return False

def _reset_child_logging(name):
    """Drop the logging state inherited across fork and log straight to stderr.

    The parent's logging pipeline is a QueueHandler feeding a QueueListener THREAD.
    Threads do not survive fork -- the child inherits the handler and the queue, but
    not the thread draining it, and potentially inherits the queue's mutex in a
    locked state if the fork raced the listener. Rather than rely on children never
    logging, cut the inheritance here and give them a plain stderr handler so their
    exceptions are still visible.
    """
    root = logging.getLogger("robot")
    for h in list(root.handlers):
        root.removeHandler(h)
    h = logging.StreamHandler(sys.__stderr__)
    h.setFormatter(logging.Formatter(
        f"%(asctime)s.%(msecs)03d %(levelname)-5s {name} %(message)s", datefmt="%H:%M:%S"))
    root.addHandler(h)
    root.setLevel(logging.INFO)
    return logging.getLogger(f"robot.{name}")


def _fmt(v, nd=1):
    """Format a possibly-None sensor reading for logging."""
    return "None" if v is None else f"{v:.{nd}f}"

"""Annotated-run recording, in its own process.

Kept off the control loop entirely: see the comments below for why avc1 costs what
it does and why frames are decimated before they reach the encoder.
"""

import multiprocessing as mp
import time
from multiprocessing import shared_memory

import cv2
import numpy as np

from src.obstacle_challenge.logsetup import wlog, _reset_child_logging
from src.obstacle_challenge.tuning import (
    CAMERA_FPS, FRAME_HEIGHT, FRAME_WIDTH, VIDEO_EVERY_N, VIDEO_FPS,
    VIDEO_QUEUE_SLOTS,
)

# ---------------------------------------------------------------------------
# Video encoding, in its own process
# ---------------------------------------------------------------------------
# avc1 is software x264 on a Pi 5 (there is no hardware H.264 encoder) and costs
# ~9.7 ms/frame -- more than half a core. In a thread it fought the control loop;
# in a process it gets its own core and the GIL is irrelevant.
#
# Frames reach it through a shared-memory ring buffer: the writer claims a free
# slot, memcpys into it (0.05 ms) and passes the slot *index* over a Queue. If no
# slot is free the frame is dropped -- recording must never stall the robot.


def _encoder_worker(path, fourcc_name, fps, size, shm_name, slot_bytes,
                    n_slots, work_q, free_q):
    _reset_child_logging("w-encode")
    cv2.setNumThreads(1)
    shm = shared_memory.SharedMemory(name=shm_name)
    slots = np.ndarray((n_slots, size[1], size[0], 3), np.uint8, buffer=shm.buf)
    out = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*fourcc_name), fps, size)
    written = 0
    try:
        while True:
            idx = work_q.get()
            if idx is None:
                break
            try:
                out.write(slots[idx])
                written += 1
            finally:
                free_q.put(idx)
    finally:
        out.release()
        shm.close()


class VideoEncoderProcess:
    """Drop-on-full ring buffer feeding an encoder process."""

    def __init__(self, path, fourcc_name, fps=None, size=(FRAME_WIDTH, FRAME_HEIGHT),
                 n_slots=None, every_n=None):
        # Resolved here, NOT as default arguments: a default arg is evaluated once at
        # import, so `every_n=VIDEO_EVERY_N` would freeze the value and make changing
        # the constant at runtime silently do nothing.
        self.path = path
        self.fourcc_name = fourcc_name
        self.size = size
        self.n_slots = VIDEO_QUEUE_SLOTS if n_slots is None else n_slots
        self.every_n = max(1, int(VIDEO_EVERY_N if every_n is None else every_n))
        # Metadata playback rate (default VIDEO_FPS = 10.0 for slow-motion playback)
        fps = VIDEO_FPS if fps is None else fps
        self.fps = fps
        n_slots = self.n_slots
        self._seen = 0
        self.dropped = 0
        self.submitted = 0
        self.decimated = 0
        ctx = mp.get_context('fork')
        slot_bytes = size[0] * size[1] * 3
        self.shm = shared_memory.SharedMemory(create=True, size=n_slots * slot_bytes)
        self.slots = np.ndarray((n_slots, size[1], size[0], 3), np.uint8,
                                buffer=self.shm.buf)
        self.work_q = ctx.Queue()
        self.free_q = ctx.Queue()
        for i in range(n_slots):
            self.free_q.put(i)
        self.proc = ctx.Process(
            target=_encoder_worker, name="video-encoder", daemon=True,
            args=(path, fourcc_name, fps, size, self.shm.name, slot_bytes,
                  n_slots, self.work_q, self.free_q))

    def start(self):
        self.proc.start()
        wlog.info("Encoder process started (pid %d, %s -> %s, %d slots, "
                  "every %d%s frame, %.1f fps metadata)",
                  self.proc.pid, self.fourcc_name, self.path, self.n_slots,
                  self.every_n, {1: "st", 2: "nd", 3: "rd"}.get(self.every_n, "th"),
                  self.fps)

    def write(self, frame):
        """Non-blocking. Records every Nth frame; drops if the encoder is behind.

        Decimation lives here rather than at the call sites so every writer -- main
        loop, initial maneuver, both parking routines -- gets it automatically.
        """
        self._seen += 1
        if self._seen % self.every_n:
            self.decimated += 1
            return False
        self.submitted += 1
        try:
            idx = self.free_q.get_nowait()
        except Exception:
            self.dropped += 1
            return False
        np.copyto(self.slots[idx], frame)
        self.work_q.put(idx)
        return True

    def stop(self, timeout=10.0):
        try:
            self.work_q.put(None)
            self.proc.join(timeout=timeout)
            if self.proc.is_alive():
                wlog.warning("Encoder did not finish in %.0fs; terminating", timeout)
                self.proc.terminate()
        except Exception:
            wlog.exception("Error stopping encoder")
        finally:
            try:
                self.shm.close()
                self.shm.unlink()
            except Exception:
                pass
        wlog.info("Encoder stopped. %d frames offered, %d skipped by decimation, "
                  "%d submitted, %d dropped by backpressure (%.1f%%)",
                  self._seen, self.decimated, self.submitted, self.dropped,
                  100.0 * self.dropped / max(1, self.submitted))

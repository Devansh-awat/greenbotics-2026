"""Fork-join of the two independent halves of frame processing.

The arena mask (~3.3 ms) and the colour masks (~2.5 ms) do not depend on each
other, so they run in two worker processes and are joined before contouring.
This is a FORK-JOIN, not a pipeline: the frame the robot acts on is the frame it
just captured, never the previous one. A pipeline would raise latency, which is
the opposite of what we want.

The one thing both halves need -- crop + colour-convert + bilateral-filter the
working slice -- is computed ONCE, in this (main) process, via
pipeline.prepare_colour_slice(), before the fork. Neither worker touches the raw
BGR frame anymore. The black threshold is owned by the arena worker (it needs it
for the wall scan anyway) and shared with the colour worker's output only at
assembly time in VisionPool.process() -- the colour worker never re-derives it.
"""

import logging
import multiprocessing as mp
import sys
import time
from multiprocessing import shared_memory

import cv2
import numpy as np

from src.logs.setup import vlog, _reset_child_logging
from src.vision import pipeline

# ---------------------------------------------------------------------------
# VisionPool -- fork-join of the two independent halves of the pipeline
# ---------------------------------------------------------------------------


def _arena_worker(hsv_name, arena_name, black_name, sky_name, flag_name, go, done, stop):
    """Worker process: builds the arena mask AND the shared black threshold."""
    wl = _reset_child_logging("w-arena")
    cv2.setNumThreads(pipeline.VISION_WORKER_THREADS)
    h_shm = shared_memory.SharedMemory(name=hsv_name)
    a_shm = shared_memory.SharedMemory(name=arena_name)
    b_shm = shared_memory.SharedMemory(name=black_name)
    s_shm = shared_memory.SharedMemory(name=sky_name)
    fl_shm = shared_memory.SharedMemory(name=flag_name)
    hsv = np.ndarray((pipeline.SLICE_HEIGHT, pipeline.FRAME_WIDTH, 3), np.uint8, buffer=h_shm.buf)
    arena_out = np.ndarray((2, pipeline.FRAME_HEIGHT, pipeline.FRAME_WIDTH), np.uint8, buffer=a_shm.buf)
    black_out = np.ndarray((pipeline.SLICE_HEIGHT, pipeline.FRAME_WIDTH), np.uint8, buffer=b_shm.buf)
    sky_out = np.ndarray((pipeline.FRAME_WIDTH,), np.int32, buffer=s_shm.buf)
    flags = np.ndarray((2,), np.int32, buffer=fl_shm.buf)
    try:
        while True:
            go.wait()
            go.clear()
            if stop.is_set():
                break
            try:
                arena_mask, floor_mask, sky, mask_black = pipeline.build_arena_mask_from_prepared(hsv)
                black_out[:] = mask_black
                if arena_mask is None:
                    flags[0] = 0
                else:
                    arena_out[0] = arena_mask
                    arena_out[1] = floor_mask
                    sky_out[:] = sky
                    flags[0] = 1
            except Exception:
                flags[0] = 0
                wl.exception("arena worker failed on a frame")
            finally:
                done.set()
    finally:
        for s in (h_shm, a_shm, b_shm, s_shm, fl_shm):
            s.close()


def _colour_worker(hsv_name, mask_name, go, done, stop):
    """Worker process: thresholds the five non-black colour masks."""
    wl = _reset_child_logging("w-colour")
    cv2.setNumThreads(pipeline.VISION_WORKER_THREADS)
    h_shm = shared_memory.SharedMemory(name=hsv_name)
    m_shm = shared_memory.SharedMemory(name=mask_name)
    hsv = np.ndarray((pipeline.SLICE_HEIGHT, pipeline.FRAME_WIDTH, 3), np.uint8, buffer=h_shm.buf)
    masks_out = np.ndarray((len(pipeline.COLOUR_MASK_NAMES), pipeline.SLICE_HEIGHT, pipeline.FRAME_WIDTH), np.uint8, buffer=m_shm.buf)
    try:
        while True:
            go.wait()
            go.clear()
            if stop.is_set():
                break
            try:
                pipeline.compute_colour_masks_only(hsv, out=masks_out)
            except Exception:
                masks_out[:] = 0
                wl.exception("colour worker failed on a frame")
            finally:
                done.set()
    finally:
        h_shm.close()
        m_shm.close()


class VisionPool:
    """Two worker processes that split one frame's vision work and rejoin.

    MUST be started before any hardware is initialised and before any thread is
    created. It uses the 'fork' start method, and forking a process that already
    owns the camera, GPIO handles or live threads copies that state into the child
    where it is invalid -- a classic source of hangs and double-frees. Starting the
    pool first makes the children clean.
    """

    def __init__(self):
        self.ok = False
        self.failed = False
        self._procs = []

    def start(self):
        ctx = mp.get_context('fork')
        self.hsv_shm = shared_memory.SharedMemory(
            create=True, size=pipeline.SLICE_HEIGHT * pipeline.FRAME_WIDTH * 3)
        self.arena_shm = shared_memory.SharedMemory(
            create=True, size=2 * pipeline.FRAME_HEIGHT * pipeline.FRAME_WIDTH)
        self.black_shm = shared_memory.SharedMemory(
            create=True, size=pipeline.SLICE_HEIGHT * pipeline.FRAME_WIDTH)
        self.sky_shm = shared_memory.SharedMemory(create=True, size=pipeline.FRAME_WIDTH * 4)
        self.flag_shm = shared_memory.SharedMemory(create=True, size=2 * 4)
        self.mask_shm = shared_memory.SharedMemory(
            create=True, size=len(pipeline.COLOUR_MASK_NAMES) * pipeline.SLICE_HEIGHT * pipeline.FRAME_WIDTH)

        self.hsv = np.ndarray((pipeline.SLICE_HEIGHT, pipeline.FRAME_WIDTH, 3), np.uint8,
                              buffer=self.hsv_shm.buf)
        self.arena = np.ndarray((2, pipeline.FRAME_HEIGHT, pipeline.FRAME_WIDTH), np.uint8,
                                buffer=self.arena_shm.buf)
        self.black = np.ndarray((pipeline.SLICE_HEIGHT, pipeline.FRAME_WIDTH), np.uint8,
                                buffer=self.black_shm.buf)
        self.sky = np.ndarray((pipeline.FRAME_WIDTH,), np.int32, buffer=self.sky_shm.buf)
        self.flags = np.ndarray((2,), np.int32, buffer=self.flag_shm.buf)
        self.masks = np.ndarray((len(pipeline.COLOUR_MASK_NAMES), pipeline.SLICE_HEIGHT, pipeline.FRAME_WIDTH), np.uint8,
                                buffer=self.mask_shm.buf)

        self.go_a, self.done_a = ctx.Event(), ctx.Event()
        self.go_c, self.done_c = ctx.Event(), ctx.Event()
        self.stop_ev = ctx.Event()

        pa = ctx.Process(
            target=_arena_worker, name="vision-arena", daemon=True,
            args=(self.hsv_shm.name, self.arena_shm.name, self.black_shm.name,
                  self.sky_shm.name, self.flag_shm.name, self.go_a, self.done_a, self.stop_ev))
        pc = ctx.Process(
            target=_colour_worker, name="vision-colour", daemon=True,
            args=(self.hsv_shm.name, self.mask_shm.name,
                  self.go_c, self.done_c, self.stop_ev))
        pa.start()
        pc.start()
        self._procs = [pa, pc]
        self.ok = True
        vlog.info("VisionPool started (pids %d, %d)", pa.pid, pc.pid)

    def process(self, frame):
        """Fork-join one frame. Returns (masks, arena_mask, floor_mask, sky, seeded).

        Returns None if a worker missed its deadline; the caller then falls back to
        inline processing and the pool stays disabled for the rest of the run.
        """
        cs_slice = pipeline.prepare_colour_slice(frame)   # shared prep, done once, serially
        np.copyto(self.hsv, cs_slice)
        self.flags[0] = 0
        self.done_a.clear()
        self.done_c.clear()
        self.go_a.set()
        self.go_c.set()

        if not self.done_a.wait(pipeline.VISION_POOL_TIMEOUT) or \
           not self.done_c.wait(pipeline.VISION_POOL_TIMEOUT):
            vlog.error("VisionPool worker missed the %.0f ms deadline -- "
                       "falling back to inline processing for the rest of the run",
                       pipeline.VISION_POOL_TIMEOUT * 1000)
            self.failed = True
            self.ok = False
            return None

        masks = {name: self.masks[i] for i, name in enumerate(pipeline.COLOUR_MASK_NAMES)}
        masks['black'] = self.black
        seeded = bool(self.flags[0])
        if seeded:
            return masks, self.arena[0], self.arena[1], self.sky, True
        return masks, None, None, None, False

    def stop(self):
        if not self._procs:
            return
        # Cleared FIRST, and before the shared memory below is unlinked: anything still
        # holding a reference to this pool must fall back to inline processing rather
        # than reach into buffers that are about to be freed. process_video_frame()
        # gates on `.ok`, and a stale pool there is a segfault, not an exception.
        self.ok = False
        self.stop_ev.set()
        self.go_a.set()
        self.go_c.set()
        for p in self._procs:
            p.join(timeout=1.0)
            if p.is_alive():
                vlog.warning("VisionPool worker %s did not exit; terminating", p.name)
                p.terminate()
        self._procs = []
        for s in (self.hsv_shm, self.arena_shm, self.black_shm, self.sky_shm,
                  self.flag_shm, self.mask_shm):
            try:
                s.close()
                s.unlink()
            except Exception:
                pass
        vlog.info("VisionPool stopped.")

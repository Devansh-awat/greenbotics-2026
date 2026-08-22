"""Run recording, in its own process.

Kept off the control loop entirely: see the comments below for why avc1 costs what
it does and why frames are decimated before they reach the encoder.

A run records ONE video, never two. Either the control loop annotates each frame and
this encoder stores the result (the default), or `-v` stores untouched camera frames
and src/tools/annotate_run.py rebuilds the annotated video afterwards. Recording both
at once meant two x264 instances -- ~16 encoder threads against 2 vision workers on 4
cores -- which cost 9 fps of control loop even after every other cost was removed.
"""

import json
import multiprocessing as mp
import time
from multiprocessing import shared_memory

import cv2
import numpy as np

from src.logs.setup import wlog, _reset_child_logging
from src.obstacle_challenge.tuning import (
    CAMERA_FPS, FRAME_HEIGHT, FRAME_WIDTH, VIDEO_EVERY_N, VIDEO_FPS,
    VIDEO_QUEUE_SLOTS,
)
from src.vision.pipeline import detections_to_record

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
#
# Nothing here pre-processes the frame. `-v` used to bilateral-filter the raw
# recording so it would "match the detector", but it filtered BGR while the pipeline
# filters HSV (prepare_colour_slice converts first, HSV_BEFORE_BLUR=True) -- a
# different operation on different pixels, so it never matched. It also cost ~8 ms
# per frame on the control loop, which is what dropped `-v` runs to 40 fps. Record
# untouched frames; the colour tuners apply the real filter at playback time.


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
        self.tags = []
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

    def write(self, frame, tag=None, prepare=None):
        """Non-blocking. Records every Nth frame; drops if the encoder is behind.

        `prepare`, if given, is called as prepare(frame) to produce the frame actually
        stored, and ONLY for frames that survive decimation. Annotation costs 1.88 ms
        -- the caller used to pay that on every frame and then hand half of them
        straight to the decimation counter below. Pass the work in rather than doing it
        first, and the loop stops paying for frames it is about to discard.

        Decimation lives here rather than at the call sites so every writer -- main
        loop, initial maneuver, both parking routines -- gets it automatically.

        `tag` (normally the control-loop frame counter) is recorded for every frame that
        actually reaches the encoder, and dumped next to the video by stop(). Without it a
        video frame number cannot be mapped back to a control frame: decimation is
        predictable but the drop-on-full path below is not, so the offset between the two
        drifts by an unknowable amount over a run. Two encoders drop independently, so
        this is also the only way to line the raw and annotated videos up with each other.
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
        if prepare is not None:
            frame = prepare(frame)
        np.copyto(self.slots[idx], frame)
        self.work_q.put(idx)
        # Appended for EVERY accepted frame, tagged or not. The index is positional --
        # line i describes video frame i -- so skipping untagged writes (the maneuver
        # scans, which have no control-frame number) would shift every later line by one
        # and silently corrupt the mapping.
        self.tags.append(tag)
        return True

    def _write_index(self):
        """video frame number -> control frame number, one per line."""
        if not self.tags:
            return
        try:
            with open(self.path + ".frames.txt", "w") as fh:
                fh.write("# video_frame control_frame  ('-' = written outside the "
                         "main loop, e.g. an initial-maneuver scan)\n")
                for i, t in enumerate(self.tags):
                    fh.write(f"{i} {'-' if t is None else t}\n")
        except Exception:
            wlog.exception("Could not write frame index for %s", self.path)

    def stop(self, timeout=10.0):
        self._write_index()
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


# ---------------------------------------------------------------------------
# Overlay sidecar
# ---------------------------------------------------------------------------
# In raw-recording mode the video holds untouched frames and the annotated version is
# rebuilt later, so everything the overlay draws has to survive the trip. Two kinds of
# thing do not survive on their own:
#
#   - control state -- steering angle, turn count, RPM, the visual target -- which
#     exists only inside the loop and is in no frame at any fidelity;
#   - the detections, which could be re-derived from the pixels but would not come back
#     the same, because H.264 is lossy and inRange() is a hard threshold. See
#     detections_to_record() in src/vision/pipeline.py for the measurements.
#
# Both are recorded here, so the rebuild is exact rather than approximate.
#
# One JSON line per RECORDED frame, keyed by video frame number rather than positional,
# so frames the maneuvers write (which carry no control state) simply have no line
# instead of shifting everything after them.


class OverlayLog:
    """Per-frame control state for the offline annotator. One JSON object per line.

    append()/append_passthrough() only buffer records in memory -- the json.dumps()
    + file write (real, synchronous disk I/O) used to happen right there on the
    control loop, gating how soon the loop got back to fetch the next frame. It's
    all done once in close(), after the run's main loop has already exited.
    """

    def __init__(self, video_path):
        self.path = video_path + ".overlay.jsonl"
        self.written = 0
        self._records = []
        self._open = True

    def append(self, video_frame, driving_direction, debug_info,
               visual_target_x=None, visual_target_line=None, detections=None):
        if not self._open:
            return
        rec = {"v": video_frame, "dir": driving_direction, "debug": list(debug_info)}
        # Omitted when absent rather than written as null: most frames have no visual
        # target, and this file gets one line per recorded frame for a whole run.
        if visual_target_x is not None:
            rec["tx"] = int(visual_target_x)
        if visual_target_line is not None:
            rec["tl"] = visual_target_line
        if detections is not None:
            rec["det"] = detections_to_record(detections)
        self._records.append(rec)

    def append_passthrough(self, video_frame):
        """Mark a frame as already annotated at record time.

        The parking routines draw their own telemetry and never run the detection
        pipeline, so there is nothing for the rebuild to replay. They store the drawn
        frame instead, and this tells annotate_run.py to pass it through rather than
        drawing detections the robot never computed over the top of it.
        """
        if not self._open:
            return
        self._records.append({"v": video_frame, "pre": True})

    def close(self):
        if not self._open:
            return
        self._open = False
        try:
            with open(self.path, "w") as fh:
                for rec in self._records:
                    fh.write(json.dumps(rec, separators=(",", ":")) + "\n")
                    self.written += 1
        except Exception:
            wlog.exception("Could not write overlay log %s; annotation data will be "
                           "missing from the rebuilt video", self.path)
            return
        wlog.info("Overlay log written: %s (%d frames)", self.path, self.written)

    @staticmethod
    def load(video_path):
        """video frame number -> overlay record. Used by the offline annotator."""
        out = {}
        with open(video_path + ".overlay.jsonl") as fh:
            for line in fh:
                line = line.strip()
                if line:
                    rec = json.loads(line)
                    out[rec["v"]] = rec
        return out

"""Rebuild a run's annotated video from the raw footage recorded by `main.py -v`.

`-v` stores untouched camera frames and skips annotation entirely, so the control loop
runs one x264 encoder instead of two (see src/obstacle_challenge/video.py). This script
puts the annotation back, offline, where it costs nothing that matters.

Nothing is re-derived for control-loop frames. Both the detections and the control
state (steering angle, turn count, RPM, the visual target) are read back from
raw.mp4.overlay.jsonl, which the loop wrote one line per recorded frame, so the rebuilt
video shows exactly what the robot saw and did.

That is why the detections are logged rather than recomputed from raw.mp4: H.264 is
lossy and inRange() is a hard threshold, so replaying the pipeline over the recording
reproduces detections on only ~24% of frames. See detections_to_record() in
src/vision/pipeline.py for the measurements and why lossless recording is not the fix.

Frames written outside the control loop are handled per routine:

  - Parking draws its own telemetry and never runs the detection pipeline, so it stores
    the drawn frame and marks it; those pass through here untouched.
  - The initial-maneuver scan does run the pipeline but logs nothing, so its handful of
    frames are recomputed from the pixels and are approximate.

The run summary reports how many frames fell into each case.

Usage:
    python3 -m src.obstacle_challenge.annotate_run                     # latest run
    python3 -m src.obstacle_challenge.annotate_run obstacle/<ts>       # a run folder
    python3 -m src.obstacle_challenge.annotate_run path/to/raw.mp4
    python3 -m src.obstacle_challenge.annotate_run -o out.mp4 <target>
"""

import argparse
import glob
import os
import sys
import threading
import time

import cv2

from src.logs.setup import log
from src.obstacle_challenge.tuning import VIDEO_FOURCC, VIDEO_FPS
from src.obstacle_challenge.video import OverlayLog
from src.vision import pipeline as vision
from src.vision.pipeline import (
    annotate_video_frame, process_video_frame, record_to_detections,
)


def find_latest_raw(base_dir="obstacle"):
    matches = glob.glob(os.path.join(base_dir, "*", "raw.mp4"))
    return max(matches, key=os.path.getmtime) if matches else None


def resolve_target(target):
    """Accept a run folder, a raw.mp4 path, or nothing (latest run)."""
    if target is None:
        path = find_latest_raw()
        if path is None:
            sys.exit("No raw.mp4 found under obstacle/. Record one with `main.py -v`.")
        return path
    if os.path.isdir(target):
        path = os.path.join(target, "raw.mp4")
        if not os.path.exists(path):
            sys.exit(f"No raw.mp4 in {target}")
        return path
    if not os.path.exists(target):
        sys.exit(f"No such file: {target}")
    return target


def annotate_run(raw_path, out_path=None, fps=None, progress_every=100, pause_check=None):
    """Rebuild obstacle.mp4 from raw.mp4 + the overlay sidecar.

    `pause_check`, if given, is called once per frame before it is processed. Pass
    `threading.Event().wait` (or similar) to make the rebuild pausable -- the caller
    clears the event to freeze this function between frames with zero CPU use (it's
    parked in a blocking wait, not polling), and sets it again to resume. Used by
    BackgroundAnnotator below to get out of the way of parking's own camera work.
    """
    # Force inline vision. main.py calls this from its cleanup handler, AFTER
    # vision_pool.stop() has unlinked the workers' shared memory -- but the module
    # global still points at that dead pool, and process_video_frame() would happily
    # use it and segfault. Batch work wants inline processing anyway: the fork-join
    # exists to cut per-frame latency on a moving robot, which is meaningless here.
    vision.vision_pool = None

    if out_path is None:
        # Next to the raw footage, under the name the live path would have used.
        out_path = os.path.join(os.path.dirname(raw_path), "obstacle.mp4")

    try:
        overlay = OverlayLog.load(raw_path)
    except FileNotFoundError:
        overlay = {}
        print(f"warning: no {raw_path}.overlay.jsonl -- annotating detections only, "
              f"without the debug overlay", file=sys.stderr)

    # Raises rather than sys.exit()s: main.py calls this from its cleanup handler, and a
    # SystemExit there is a BaseException that would sail past `except Exception` and
    # abort the shutdown over a video that failed to rebuild.
    cap = cv2.VideoCapture(raw_path)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open {raw_path}")
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    # Match the source's playback rate by default, so the rebuilt video plays back at
    # the same speed as the raw one (VIDEO_FPS is deliberately slow-motion).
    if fps is None:
        fps = cap.get(cv2.CAP_PROP_FPS) or VIDEO_FPS

    # Written to a scratch file and renamed only on success. An mp4 is finalised by
    # release(), so a process that dies mid-rebuild leaves a file that exists, looks
    # like a video and will not play. Better to leave no obstacle.mp4 at all than a
    # broken one -- absent is unambiguous, 48 bytes of header is not.
    #
    # The suffix goes BEFORE the extension (obstacle.part.mp4, not obstacle.mp4.part):
    # VideoWriter picks the container from the filename extension, so writing to
    # ".part" silently produced an AVI, which the rename then disguised as an mp4.
    _root, _ext = os.path.splitext(out_path)
    tmp_path = f"{_root}.part{_ext}"
    out = cv2.VideoWriter(tmp_path, cv2.VideoWriter_fourcc(*VIDEO_FOURCC),
                          fps, (width, height))
    if not out.isOpened():
        cap.release()
        raise RuntimeError(f"Could not open {tmp_path} for writing")

    print(f"{raw_path} -> {out_path}")
    print(f"  {total} frames, {width}x{height} @ {fps:.1f} fps, "
          f"{len(overlay)} overlay records")

    idx = 0
    replayed = 0
    passthrough = 0
    t0 = time.monotonic()
    try:
        while True:
            if pause_check is not None:
                pause_check()
            ok, frame = cap.read()
            if not ok:
                break
            rec = overlay.get(idx, {})
            if rec.get("pre"):
                # Parking: drawn at record time with telemetry this script has no way
                # to reconstruct. Annotating it again would paint detections the robot
                # never computed over the top.
                out.write(frame)
                passthrough += 1
            else:
                if "det" in rec:
                    detections = record_to_detections(rec["det"])
                    replayed += 1
                else:
                    # No logged detections: an initial-maneuver scan frame, or a run
                    # recorded before detection logging existed. Re-derive from the
                    # pixels, which is close but not exact -- see detections_to_record().
                    detections = process_video_frame(frame)
                out.write(annotate_video_frame(
                    frame, detections, rec.get("dir"),
                    debug_info=rec.get("debug", []),
                    visual_target_x=rec.get("tx"),
                    visual_target_line=rec.get("tl")))
            idx += 1
            if progress_every and idx % progress_every == 0:
                el = time.monotonic() - t0
                print(f"  {idx}/{total} ({idx / el:.1f} fps)", end="\r", flush=True)
    except BaseException:
        cap.release()
        out.release()
        # Includes KeyboardInterrupt, hence BaseException: a Ctrl-C halfway through
        # should not leave an unplayable obstacle.mp4 behind either.
        try:
            os.unlink(tmp_path)
        except OSError:
            pass
        raise
    else:
        cap.release()
        out.release()
        os.replace(tmp_path, out_path)

    el = time.monotonic() - t0
    print(f"  {idx} frames in {el:.1f}s ({idx / max(el, 1e-9):.1f} fps){' ' * 20}")
    print(f"  {replayed} exact (logged detections), "
          f"{passthrough} already annotated (parking, passed through), "
          f"{idx - replayed - passthrough} re-derived from pixels (approximate)")
    return out_path


class BackgroundAnnotator:
    """Runs annotate_run() on a background thread, pausable and joinable.

    Started as soon as the driving portion of a run ends (turn 13 / parking begins)
    so the ~9s rebuild overlaps with parking's idle CPU time instead of blocking
    exit after the robot stops. A thread, not a process: main.py already forks its
    child processes (vision pool, video encoder) before any hardware or thread
    exists, specifically to avoid forking a live picamera2/gpiozero/thread state --
    by the time parking starts, all of that is up, so a new process here would be
    exactly the hazard that comment warns about. A thread has no such restriction,
    and OpenCV's per-frame work releases the GIL, so it costs real cores without
    needing process-level isolation.

    Pausing is cooperative, not preemptive: pause() just clears an Event, and the
    annotation loop blocks on it between frames via `pause_check`. A blocked
    thread burns 0% CPU (it's not polling), so this is a real pause, not a
    slowdown -- exactly what parking's Step 4 magenta-line tracking needs while
    it owns the camera's latency budget.

    `finalize`, if given, is called on the background thread before annotate_run()
    starts -- NOT by the caller of start(). It exists for video_encoder.stop(): an
    mp4 only becomes readable once the encoder's child process has actually joined,
    which can take anywhere from tens of ms to over a second depending on its queue
    backlog. Calling that from the control thread right before parking's first move
    (motor.move(14, ...) in parking()) turned into a variable-length, unbraked pause
    with the robot still coasting on momentum from the drive -- i.e. exactly the
    "pauses then overshoots forward" bug this replaced. Doing it here instead keeps
    the control thread's stop_rpm_control() -> parking() transition instant; whatever
    the flush costs is paid entirely on this thread, which the caller never waits on
    except via join() (see below).
    """

    def __init__(self, raw_path, out_path, finalize=None):
        self.raw_path = raw_path
        self.out_path = out_path
        self.error = None
        self._finalize = finalize
        self._resume_event = threading.Event()
        self._resume_event.set()   # not paused
        self._done_event = threading.Event()
        self._thread = threading.Thread(
            target=self._run, name="bg-annotate", daemon=True)

    def start(self):
        self._thread.start()

    def _run(self):
        try:
            if self._finalize is not None:
                self._finalize()
            annotate_run(self.raw_path, self.out_path,
                         pause_check=self._resume_event.wait)
        except Exception as exc:
            self.error = exc
            log.exception("Background annotation failed for %s", self.raw_path)
        finally:
            self._done_event.set()

    def pause(self):
        self._resume_event.clear()

    def resume(self):
        self._resume_event.set()

    def is_done(self):
        return self._done_event.is_set()

    def join(self, timeout=None):
        """Block until the rebuild finishes. Returns immediately if already done."""
        already_done = self.is_done()
        # A caller might invoke join() while paused (e.g. run aborted mid-Step-4) --
        # resume first or this would hang forever waiting on a frozen thread.
        self.resume()
        finished = self._done_event.wait(timeout)
        if not already_done:
            log.info("Background annotation %s.",
                     "finished" if finished else "did not finish before timeout")
        return finished


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument(
        "target", nargs="?", default=None,
        help="Run folder or raw.mp4 path. Defaults to the newest raw.mp4 under obstacle/.")
    parser.add_argument(
        "-o", "--output", default=None,
        help="Output path. Defaults to obstacle.mp4 beside the raw footage.")
    parser.add_argument(
        "--fps", type=float, default=None,
        help="Playback fps of the output. Defaults to the raw video's own rate.")
    args = parser.parse_args()

    cv2.setNumThreads(0)   # 0 = all cores; nothing is competing for them now
    annotate_run(resolve_target(args.target), args.output, args.fps)


if __name__ == "__main__":
    main()

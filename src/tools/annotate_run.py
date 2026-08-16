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
import time

import cv2

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


def annotate_run(raw_path, out_path=None, fps=None, progress_every=100):
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

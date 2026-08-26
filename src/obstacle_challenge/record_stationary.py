"""Record raw.mp4 into obstacle/<timestamp>/ like a normal -v run, but never drive.

Use this to film calibration/test footage by moving the robot by hand: same folder
layout and encoder as main.py -v, minus motors/servo/vision entirely.
"""

import argparse
import os
import sys
import time
from datetime import datetime

import cv2

from src.logs.setup import setup_logging, shutdown_logging, wlog as log
from src.threads.hw_threads import CameraThread
from src.obstacle_challenge.tuning import VIDEO_FOURCC
from src.obstacle_challenge.video import VideoEncoderProcess
from src.sensors import camera


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=None,
                         help="Stop after this many seconds (default: run until Ctrl-C).")
    parser.add_argument("--every-n", type=int, default=None, metavar="N",
                         help="Record every Nth frame (default: VIDEO_EVERY_N).")
    args = parser.parse_args()

    run_timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    run_folder = os.path.join("obstacle", run_timestamp)
    os.makedirs(run_folder, exist_ok=True)
    raw_video_path = os.path.join(run_folder, "raw.mp4")
    log_path = os.path.join(run_folder, "obstacle_output.txt")

    setup_logging(log_path)
    log.info("=== Stationary raw recording | run %s ===", run_timestamp)

    cv2.setNumThreads(4)
    video_encoder = VideoEncoderProcess(raw_video_path, VIDEO_FOURCC, every_n=args.every_n)
    video_encoder.start()
    log.info("Recording raw frames -> %s", raw_video_path)

    if not camera.initialize():
        log.error("Camera failed to initialize (already in use by another process?).")
        video_encoder.stop()
        shutdown_logging()
        sys.exit(1)

    camera_thread = CameraThread(camera)
    camera_thread.start()

    frame_counter = 0
    start = time.monotonic()
    try:
        while args.seconds is None or (time.monotonic() - start) < args.seconds:
            frame, frame_counter, _ = camera_thread.get_next_frame(frame_counter)
            if frame is None:
                continue
            video_encoder.write(frame, tag=frame_counter)
    except KeyboardInterrupt:
        log.info("Stopped by Ctrl-C.")
    finally:
        camera_thread.stop_event.set()
        camera_thread.join(timeout=5)
        camera.cleanup()
        video_encoder.stop()
        shutdown_logging()
        print(f"Saved: {raw_video_path}")


if __name__ == "__main__":
    main()

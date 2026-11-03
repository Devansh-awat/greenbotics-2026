"""Record raw.mp4 into obstacle/<timestamp>/ like a normal -v run, but never drive.

Use this to film calibration/test footage by moving the robot by hand: same folder
layout and encoder as main.py -v, minus motors/servo/vision entirely.

The button toggles recording and the LED is on while recording, so the robot can be
carried to the next spot without filming it. It starts PAUSED (LED off); press once
to start. --no-button records from the start, as before.

Every camera frame's libcamera metadata (AWB colour gains, analogue/digital gain,
exposure, colour temperature, CCM, ...) goes to raw.mp4.camera.jsonl, paused frames
included, so you can see how auto-white-balance and auto-gain move while the robot
is carried around too. Each line has "frame" (camera frame counter, the same number
raw.mp4.frames.txt maps video frames to) and "video_frame" (index into raw.mp4, or
null if the frame was not recorded -- paused, decimated or dropped).
"""

import argparse
import json
import os
import sys
import threading
import time
from datetime import datetime

import cv2
from gpiozero import Button, LED

from src.logs.setup import setup_logging, shutdown_logging, wlog as log
from src.obstacle_challenge import config
from src.obstacle_challenge.tuning import VIDEO_FOURCC
from src.obstacle_challenge.video import VideoEncoderProcess
from src.sensors import camera

# Metadata keys worth keeping. The full dict also has per-frame timing/focus
# noise that would triple the file for nothing.
META_KEYS = (
    "ColourGains", "AnalogueGain", "DigitalGain", "ExposureTime", "ColourTemperature",
    "Lux", "ColourCorrectionMatrix", "SensorBlackLevels", "AeLocked", "FrameDuration",
    "SensorTimestamp",
)


def _jsonable(v):
    if isinstance(v, (list, tuple)):
        return [_jsonable(x) for x in v]
    if isinstance(v, float):
        return round(v, 5)
    return v


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--seconds", type=float, default=None,
                         help="Stop after this many seconds (default: run until Ctrl-C).")
    parser.add_argument("--every-n", type=int, default=None, metavar="N",
                         help="Record every Nth frame (default: VIDEO_EVERY_N).")
    parser.add_argument("--no-button", action="store_true",
                         help="Record from the start; the button is not used.")
    args = parser.parse_args()

    run_timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    run_folder = os.path.join("obstacle", run_timestamp)
    os.makedirs(run_folder, exist_ok=True)
    raw_video_path = os.path.join(run_folder, "raw.mp4")
    log_path = os.path.join(run_folder, "obstacle_output.txt")
    meta_path = raw_video_path + ".camera.jsonl"

    setup_logging(log_path)
    log.info("=== Stationary raw recording | run %s ===", run_timestamp)

    cv2.setNumThreads(4)
    video_encoder = VideoEncoderProcess(raw_video_path, VIDEO_FOURCC, every_n=args.every_n)
    video_encoder.start()
    log.info("Recording raw frames -> %s, camera metadata -> %s", raw_video_path, meta_path)

    if not camera.initialize():
        log.error("Camera failed to initialize (already in use by another process?).")
        video_encoder.stop()
        shutdown_logging()
        sys.exit(1)

    led = LED(config.LED_PIN)
    recording = threading.Event()
    button = None
    if args.no_button:
        recording.set()
        led.on()
    else:
        button = Button(config.BUTTON_PIN, bounce_time=0.05)

        def toggle():
            if recording.is_set():
                recording.clear()
                led.off()
            else:
                recording.set()
                led.on()
            # Logged from the loop (with the video frame index) -- see below.

        button.when_pressed = toggle
        log.info("Paused. Press the button to start/stop recording (LED on = recording).")

    meta_file = open(meta_path, "w")
    frame_counter = 0
    was_recording = False
    start = time.monotonic()
    try:
        while args.seconds is None or (time.monotonic() - start) < args.seconds:
            # Captured inline rather than via CameraThread: the request API is the only
            # way to get the metadata of the exact frame, and this loop does nothing
            # else (the encoder is its own process), so it keeps up with 56fps.
            frame, meta = camera.capture_frame_with_metadata()
            if frame is None:
                continue
            frame_counter += 1

            rec = recording.is_set()
            if rec != was_recording:
                log.info("%s recording at camera frame %d (video frame %d)",
                         "STARTED" if rec else "PAUSED", frame_counter,
                         len(video_encoder.tags))
                was_recording = rec

            video_frame = None
            if rec and video_encoder.write(frame, tag=frame_counter):
                video_frame = len(video_encoder.tags) - 1

            row = {"frame": frame_counter, "video_frame": video_frame, "rec": rec}
            row.update({k: _jsonable(meta[k]) for k in META_KEYS if k in meta})
            meta_file.write(json.dumps(row) + "\n")
    except KeyboardInterrupt:
        log.info("Stopped by Ctrl-C.")
    finally:
        meta_file.close()
        led.off()
        led.close()
        if button is not None:
            button.close()
        camera.cleanup()
        video_encoder.stop()
        shutdown_logging()
        print(f"Saved: {raw_video_path}\n       {meta_path}")


if __name__ == "__main__":
    main()

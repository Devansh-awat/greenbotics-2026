"""
Open Challenge -- main control program.

Shares its whole vision pipeline and multiprocessing model with the obstacle
challenge (see CLAUDE.md and src/vision/pipeline.py / src/vision/pool.py):
the same `process_video_frame()` -- arena mask + colour thresholding
fork-joined across two worker processes via VisionPool -- the same
CameraThread/ImuThread (src/threads/hw_threads.py), the same
VideoEncoderProcess for recording, and the same `robot.*` logging pipeline
(src/logs/setup.py). Only the driving logic below is open-challenge specific
(no pillars, no parking).

process_video_frame() also detects pillars and the parking (magenta) walls,
which don't exist on the open track -- `detected_blocks` and
`detected_magenta` simply come back empty every frame, which this loop never
reads.

Run it from the repo root:  python3 -m src.open_challenge.main
"""

import argparse
import math
import os
import threading
import time
from collections import deque
from datetime import datetime

import cv2
import numpy as np
from gpiozero import Button, LED

from src.motors import motor, servo
from src.sensors import bno086, camera

from src.open_challenge import config
from src.open_challenge.config import *
from src.obstacle_challenge.control import get_angular_difference
from src.obstacle_challenge.tuning import CAMERA_STALE_TIMEOUT_S
from src.threads.hw_threads import CameraThread, ImuThread, PerfMonitor
from src.logs.setup import Throttle, log, setup_logging, shutdown_logging
from src.obstacle_challenge.video import VideoEncoderProcess
from src.tools.power_mode import ensure_performance
from src.vision import pipeline as vision
from src.vision.pipeline import annotate_video_frame, process_video_frame
from src.vision.pool import VisionPool


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Open Challenge main control program.")
    parser.add_argument(
        "-b", "--button",
        action="store_true",
        help="Wait for hardware button press before starting the run."
    )
    args = parser.parse_args()

    run_timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    base_folder = "open"
    run_folder = os.path.join(base_folder, run_timestamp)
    os.makedirs(run_folder, exist_ok=True)
    video_path = os.path.join(run_folder, 'open.mp4')
    log_path = os.path.join(run_folder, 'open_output.txt')

    setup_logging(log_path)
    log.info("=== Open Challenge | run %s ===", run_timestamp)
    log.info("Logging to %s", log_path)

    # If the between-runs idle-power trim is still active (powersave governor),
    # undo it now -- before any latency-sensitive work starts.
    ensure_performance(log)

    # ---- Configure vision pipeline for Open Challenge ----
    vision.configure(config)

    # ---- Child processes FIRST -------------------------------------------
    # Same ordering constraint as the obstacle challenge: fork before any
    # hardware handle or thread exists (see src/obstacle_challenge/main.py).
    vision_pool = None
    if USE_VISION_POOL:
        vision_pool = VisionPool()
        vision_pool.start()
        vision.vision_pool = vision_pool
        cv2.setNumThreads(2)
    else:
        cv2.setNumThreads(4)
        log.info("VisionPool disabled; processing inline.")

    video_encoder = VideoEncoderProcess(video_path, VIDEO_FOURCC)
    video_encoder.start()
    log.info("Recording to %s (%s)", video_path, VIDEO_FOURCC)

    # ---- Hardware ---------------------------------------------------------
    camera.initialize()
    motor.initialize()
    servo.initialize()
    button = Button(config.BUTTON_PIN)
    led = LED(config.LED_PIN)

    camera_thread = CameraThread(camera)
    camera_thread.start()

    imu_initialized_event = threading.Event()
    imu_thread = ImuThread(bno086, imu_initialized_event)
    imu_thread.start()
    log.info("Waiting for IMU to initialize...")
    imu_initialized_event.wait()
    log.info("IMU is ready. Proceeding with main logic.")

    led.on()
    if args.button:
        log.info("Waiting for button press...")
        button.wait_for_press()
        log.info("Button pressed! Starting run.")
    led.off()
    time.sleep(0.5)

    INITIAL_HEADING = None
    while INITIAL_HEADING is None:
        log.debug("Waiting for first valid heading reading...")
        heading = imu_thread.get_heading()
        if heading is not None:
            INITIAL_HEADING = heading
        time.sleep(0.05)
    log.info("Initial heading locked: %.1f", INITIAL_HEADING)

    orange_detection_history = deque([False] * ORANGE_DETECTION_HISTORY_LENGTH, maxlen=ORANGE_DETECTION_HISTORY_LENGTH)
    cooldown_frames = 0
    turn_counter = 0

    prev_angle = 0
    prev_wall_error = 0.0
    past_frame_counter = 0
    # Set from the first floor line we see: orange first -> clockwise, blue first
    # -> counter-clockwise. v5 decides this from the inner wall areas instead, but
    # that only works from the parking position, which the open challenge doesn't
    # start in. Until a line has been seen it stays None and the corner turn falls
    # back to comparing the two sides.
    driving_direction = None
    final_run_initiated = False
    final_run_start_time = None
    final_run_start_pos = None

    cpr = motor.encoder.counts_per_rev if (motor.encoder is not None) else (48 * 4.4 * 38 / 13)
    wheel_diameter_mm = motor.encoder.wheel_diameter_mm if (motor.encoder is not None) else 62.4
    circ_cm = (math.pi * wheel_diameter_mm) / 10.0

    perf = PerfMonitor()
    frame_debug_throttle = Throttle(1.0)

    try:
        run_start_time = time.monotonic()
        motor.start_rpm_control(TARGET_RPM, direction="forward")
        camera_stalled = False

        while True:
            angle = 0
            debug = []

            # Blocking wait -- no busy-spin, same as the obstacle loop.
            frame, frame_counter, capture_ts, fresh = camera_thread.get_next_frame(past_frame_counter)
            if frame is None or not fresh:
                # Stale frame -- see CameraThread.get_next_frame(). Cut power rather
                # than steer on pixels we already used.
                if not camera_stalled:
                    camera_stalled = True
                    log.critical("CAMERA STALL: no new frame for >%.0f ms after f=%d "
                                 "-- stopping motor.",
                                 CAMERA_STALE_TIMEOUT_S * 1000.0, past_frame_counter)
                    motor.stop_rpm_control()
                    motor.brake()
                continue
            if camera_stalled:
                camera_stalled = False
                log.warning("CAMERA STALL over -- restarting motor control.")
                motor.start_rpm_control(TARGET_RPM, direction="forward")
            skipped = max(0, frame_counter - past_frame_counter - 1)
            past_frame_counter = frame_counter

            t_proc = time.perf_counter()
            detections = process_video_frame(frame)
            proc_ms = (time.perf_counter() - t_proc) * 1000.0

            detected_orange_object = detections['detected_orange']
            detected_blue_object = detections['detected_blue']

            # --- Driving direction (first line wins) ---
            if driving_direction is None:
                if detected_orange_object:
                    driving_direction = 'clockwise'
                    log.info("Orange line seen first -> driving_direction = clockwise")
                elif detected_blue_object:
                    driving_direction = 'counter-clockwise'
                    log.info("Blue line seen first -> driving_direction = counter-clockwise")

            # --- Turn Counting Logic ---
            orange_detected_this_frame = bool(detected_orange_object)
            orange_detection_history.append(orange_detected_this_frame)

            if cooldown_frames > 0:
                cooldown_frames -= 1
            elif not orange_detection_history[-ORANGE_DETECTION_HISTORY_LENGTH] and all(list(orange_detection_history)[1:]):
                turn_counter += 1
                cooldown_frames = ORANGE_COOLDOWN_FRAMES
                log.info("turn_counter ----------------> %d", turn_counter)

            # --- Steering Logic ---
            detected_walls = detections['detected_walls']
            left_pixel_size = sum(obj['area'] for obj in detected_walls if obj['type'] == 'wall_left')
            right_pixel_size = sum(obj['area'] for obj in detected_walls if obj['type'] == 'wall_right')
            wall_inner_left_size = sum(obj['area'] for obj in detected_walls if obj['type'] == 'wall_inner_left')
            wall_inner_right_size = sum(obj['area'] for obj in detected_walls if obj['type'] == 'wall_inner_right')

            # If one side has inner wall and other does not, inflate that inner wall size
            if wall_inner_left_size > 0 and wall_inner_right_size == 0:
                wall_inner_left_size = wall_inner_left_size * 2 + 1000
            elif wall_inner_right_size > 0 and wall_inner_left_size == 0:
                wall_inner_right_size = wall_inner_right_size * 2 + 1000

            # Combine outer and inner ROIs per side before checking if one side is lost
            total_left = left_pixel_size + wall_inner_left_size
            total_right = right_pixel_size + wall_inner_right_size

            # One side gone: push the side we CAN see hard, so the robot swings away
            # from the wall it is about to clip rather than drifting on a small error.
            if total_left < 700 and total_right > 100:
                total_right = total_right * 2 + 25000
            elif total_right < 700 and total_left > 100:
                total_left = total_left * 2 + 25000

            # PD on the left/right area imbalance
            wall_error = total_left - total_right
            wall_derivative = wall_error - prev_wall_error
            angle = (wall_error * WALL_KP) + (wall_derivative * WALL_KD)
            prev_wall_error = wall_error

            # Corner: a black band right in front, or the line ROI filled with wall.
            # ADDS to the PD angle rather than overriding it, so the wall error still
            # shapes how tight the turn is.
            if driving_direction == 'counter-clockwise':
                corner_turn = -CORNER_TURN_ANGLE
            elif driving_direction == 'clockwise':
                corner_turn = CORNER_TURN_ANGLE
            else:
                # Direction not established yet: turn away from the bigger wall.
                corner_turn = (-CORNER_TURN_ANGLE if total_left < total_right
                               else CORNER_TURN_ANGLE)

            close_black_area = sum(obj['area'] for obj in detections.get('detected_close_black', []))
            if (close_black_area > CLOSE_BLACK_AREA_THRESHOLD
                    or detections.get('line_roi_wall_pct', 0) > LINE_ROI_WALL_PCT_THRESHOLD):
                angle += corner_turn
            # Both walls out of frame with a floor line under us: mid-corner, keep
            # turning. Stacks with the trigger above, as in v5.
            if total_left == 0 and total_right == 0 and (detected_orange_object or detected_blue_object):
                angle += corner_turn

            # --- ACTUATE FIRST ---
            # Steer immediately after deciding; annotation and recording come after.
            angle = np.clip(angle, -40, 40)
            if angle != prev_angle:
                servo.set_angle(angle)
            prev_angle = angle

            current_rpm = motor.get_measured_rpm()
            latency_ms = (time.monotonic() - capture_ts) * 1000.0
            # ---- everything below is off the critical path ----------------

            perf.add(latency_ms, proc_ms, skipped)
            perf.maybe_report(extra=f" | {current_rpm:5.1f} rpm (target {TARGET_RPM:.0f})")

            debug.extend([f"L:{int(left_pixel_size)}", f"R:{int(right_pixel_size)}"])
            debug.extend([f"IL:{int(wall_inner_left_size)}", f"IR:{int(wall_inner_right_size)}"])
            debug.append(f"Err:{int(wall_error)}")
            debug.append(f"Wall%:{int(detections.get('line_roi_wall_pct', 0))}")
            debug.append(f"Angle:{int(angle)}")
            debug.append(f"RPM:{int(round(current_rpm))}")
            debug.append(f"Turns:{turn_counter}")
            debug.append(f"Dir:{driving_direction or '?'}")

            if frame_debug_throttle:
                log.debug("f=%d angle=%+6.1f turns=%d rpm=%5.1f lat=%5.2fms vis=%4.2fms walls=%d skip=%d",
                          frame_counter, float(angle), turn_counter, current_rpm, latency_ms, proc_ms,
                          len(detected_walls), skipped)

            annotated_frame = annotate_video_frame(
                frame, detections, driving_direction, debug_info=debug)
            video_encoder.write(annotated_frame)

            # --- Exit Conditions ---
            if turn_counter == 12 and not final_run_initiated and get_angular_difference(imu_thread.get_heading(), INITIAL_HEADING) < 30:
                final_run_initiated = True
                final_run_start_time = time.monotonic()
                final_run_start_pos = motor.encoder_position()
                log.info("12 turns reached. Driving final %.1f cm (encoder-based) before stop...", FINAL_STOP_DISTANCE_CM)
                servo.set_angle(0.0)
                prev_angle = 0.0

            if final_run_initiated:
                curr_pos = motor.encoder_position()
                dist_traveled_cm = 0.0
                encoder_done = False
                if curr_pos is not None and final_run_start_pos is not None:
                    delta_ticks = abs(curr_pos - final_run_start_pos)
                    dist_traveled_cm = (delta_ticks / cpr) * circ_cm
                    encoder_done = (dist_traveled_cm >= FINAL_STOP_DISTANCE_CM)

                # Fallback timeout if encoder is unavailable
                timeout_done = (time.monotonic() - final_run_start_time) >= 1.5

                if encoder_done or timeout_done:
                    log.info("Final distance complete (%.1f cm / %s). Stopping.",
                             dist_traveled_cm, "encoder" if encoder_done else "timeout fallback")
                    motor.stop_rpm_control()
                    break

            if turn_counter >= 13:
                motor.stop_rpm_control()
                break

    except Exception:
        log.exception("ERROR during execution")

    finally:
        run_end_time = time.monotonic()
        total_run_time = run_end_time - run_start_time
        log.info("Run completed in: %.2f seconds.", total_run_time)

        motor.stop_rpm_control()
        motor.brake()
        log.info("Signaling threads to stop...")
        camera_thread.stop()
        imu_thread.stop()

        log.info("Waiting for threads to complete...")
        camera_thread.join(timeout=5)
        imu_thread.join(timeout=5)
        log.info("All threads have completed.")

        # Child processes last: the encoder still has queued frames to flush.
        try:
            video_encoder.stop()
        except Exception:
            log.exception("Error stopping encoder")
        if vision_pool is not None:
            try:
                vision_pool.stop()
            except Exception:
                log.exception("Error stopping vision pool")

        camera.cleanup()
        servo.set_angle(0)
        servo.cleanup()
        motor.cleanup()
        cv2.destroyAllWindows()
        log.info("Run complete. Log saved to %s", log_path)
        shutdown_logging()

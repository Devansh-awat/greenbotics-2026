"""
Obstacle Challenge -- main control program (v5).

Runs on the robot's Raspberry Pi 5. See CLAUDE.md for the hardware map. This file
holds the run setup and the per-frame control loop, and nothing else; everything it
calls lives in a sibling module -- vision, threads and logging are shared with the
open challenge, so they live outside this package:

    src/logs/setup.py      the `robot.*` logger tree, Throttle, non-blocking log queue
    tuning.py               every constant -- colour ranges, ROIs, gains, perf switches
    src/vision/pipeline.py  arena mask, colour masks, process_video_frame, annotation
    src/vision/pool.py      the two vision worker processes and their shared memory
    video.py                the annotated-run recorder, in its own process
    src/threads/hw_threads.py  CameraThread / ImuThread / SensorThread / PerfMonitor
    control.py              heading maths and the gyro-stabilised drive primitives
    maneuvers.py            the scripted sequences: initial maneuver, parking, parking2

Performance model (measured 2026-08-01 on the real robot, 640x360):

    camera frame interval ............ 17.7 ms   (56.0 fps, HARD sensor limit)
    process_video_frame .............. ~6.2 ms   (arena mask 3.3, colour+contours 2.5)
    annotate_video_frame ............. ~0.5 ms
    avc1 (software x264) encode ...... ~9.7 ms   (Pi 5 has NO hardware H.264 encoder)

The camera is the ceiling: the IMX708 full-FoV mode reports FrameDuration=17849us no
matter what FrameRate or FrameDurationLimits we ask for. The only faster sensor mode
(1536x864 @120fps) crops to the centre 66%, which puts the side wall-following ROIs
outside the frame. So 56 fps is the target, not something to beat.

Vision is therefore NOT the bottleneck -- it is 6.2 ms of a 17.7 ms budget. The old
loop dropped to 40-50 fps because of CPU contention, from three things this version
fixes:

  1. The loop busy-spun on `get_frame()` + `continue`, burning a whole core polling
     and starving the camera thread. Now it blocks on a condition variable.
  2. avc1 encoding ran in a *thread*, so it fought the control loop for the GIL and a
     core. Now it runs in its own process (see video.VideoEncoderProcess). avc1 is
     kept -- mp4v is cheaper but produces bigger files and won't open in QuickTime.
  3. Annotation, video write, and a `sleep(1/60 - elapsed)` all happened BEFORE
     `servo.set_angle()`, adding up to ~25 ms between seeing a frame and steering on
     it. Now the loop actuates first and records afterwards.

Multiprocessing of the vision itself (VisionPool) is a smaller, real win: the arena
mask and the colour thresholding are independent until they meet at a bitwise_and, so
they fork-join across two processes. Measured 5.19 ms -> 4.17 ms (1.25x). The split is
fork-join, NOT a pipeline -- a pipeline would raise latency, which is the opposite of
what we want. Frames move through /dev/shm (0.05 ms memcpy); a Queue/pickle path would
cost more than the work saved.

Run it from the repo root:  python3 -m src.obstacle_challenge.main
"""

import argparse
import math
import os
import sys
import time
from collections import deque
from datetime import datetime

import cv2
import numpy as np
import threading
from gpiozero import Button, LED
import subprocess

from src.motors import motor, servo
from src.sensors import bno086, camera, distance

from src.obstacle_challenge import config, control, maneuvers
from src.threads.hw_threads import (
    CameraThread, ImuThread, PerfMonitor, SensorThread,
)
from src.logs.setup import Throttle, log, setup_logging, shutdown_logging
from src.obstacle_challenge.tuning import *
from src.obstacle_challenge.video import OverlayLog, VideoEncoderProcess
from src.tools.annotate_run import BackgroundAnnotator, annotate_run
from src.tools.power_mode import ensure_performance
from src.vision import pipeline as vision
from src.vision.pipeline import annotate_video_frame, process_video_frame
from src.vision.pool import VisionPool


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Obstacle Challenge main control program.")
    parser.add_argument(
        "-b", "--button",
        action="store_true",
        help="Wait for hardware button press before starting the run."
    )
    parser.add_argument(
        "-v", "--raw-video",
        action="store_true",
        help="Record raw.mp4 -- untouched camera frames -- INSTEAD OF the annotated "
             "obstacle.mp4, and log what was detected alongside it. obstacle.mp4 is "
             "rebuilt automatically in the background as soon as turn 13 completes and "
             "parking begins (see --no-annotate), so you still get both videos and the "
             "program exits the instant parking finishes. Cheaper on the control loop "
             "than annotating live, and raw.mp4 is what the colour tuners want."
    )
    parser.add_argument(
        "--no-annotate",
        action="store_true",
        help="With -v, do NOT rebuild obstacle.mp4 at all. Normally the rebuild starts "
             "in the background the moment parking begins and overlaps with parking's "
             "idle CPU time (pausing during parking's own camera-tracking step), so it "
             "already costs the run nothing; use this only if you don't want "
             "obstacle.mp4 this run and will rebuild later."
    )
    parser.add_argument(
        "--every-n", type=int, default=None, metavar="N",
        help=f"Record every Nth frame (default {VIDEO_EVERY_N}). N=1 records all 56 fps "
             "but measurably starves the vision workers -- loop work p50 goes 8.6 -> "
             "12.2 ms and p95 to 20.2 ms, past the 17.9 ms frame budget -- so expect to "
             "drop below 56 fps. Raise it to shrink the recording instead."
    )
    args = parser.parse_args()

    run_timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    base_folder = "obstacle"
    run_folder = os.path.join(base_folder, run_timestamp)
    os.makedirs(run_folder, exist_ok=True)
    video_path = os.path.join(run_folder, 'obstacle.mp4')
    raw_video_path = os.path.join(run_folder, 'raw.mp4')
    log_path = os.path.join(run_folder, 'obstacle_output.txt')

    setup_logging(log_path)
    log.info("=== Obstacle Challenge v5 | run %s ===", run_timestamp)
    log.info("Logging to %s", log_path)
    if args.raw_video:
        log.info("Raw video recording (-v): storing untouched frames -> %s. "
                 "Rebuild the annotated video with "
                 "`python3 -m src.obstacle_challenge.annotate_run %s`",
                 raw_video_path, run_folder)

    # If the between-runs idle-power trim is still active (powersave governor),
    # undo it now -- before any latency-sensitive work starts.
    ensure_performance(log)

    # ---- Child processes FIRST -------------------------------------------
    # These are forked, and fork copies the parent's whole address space. Forking
    # after picamera2/lgpio/gpiozero have opened device handles, or after threads
    # exist, hands the child invalid state -- hangs and double-frees. So the pool
    # and the encoder are created before any hardware is touched and before any
    # thread is started.
    vision_pool = None
    if USE_VISION_POOL:
        vision_pool = VisionPool()
        vision_pool.start()
        # process_video_frame() reads this; setting it is what switches the
        # pipeline from inline to pooled.
        vision.vision_pool = vision_pool
        # Workers own a core each; leave the main process fewer OpenCV threads so
        # they don't oversubscribe the 4 cores.
        cv2.setNumThreads(2)
    else:
        cv2.setNumThreads(4)
        log.info("VisionPool disabled; processing inline.")

    # Exactly ONE encoder either way. -v swaps what it stores (untouched frames rather
    # than annotated ones) instead of adding a second process; see video.py for why a
    # second x264 instance is not affordable on 4 cores.
    record_raw = args.raw_video
    video_encoder = VideoEncoderProcess(
        raw_video_path if record_raw else video_path, VIDEO_FOURCC,
        every_n=args.every_n)
    video_encoder.start()
    log.info("Recording %s to %s (%s)", "raw" if record_raw else "annotated",
             video_encoder.path, VIDEO_FOURCC)
    # Control state the rebuilt annotation needs and cannot recover from pixels.
    overlay_log = OverlayLog(raw_video_path) if record_raw else None

    # A second, always-idle encoder for parking's own footage. Started here (before
    # any hardware/thread exists) for the same fork-safety reason as video_encoder
    # above -- parking begins mid-run, long after threads and picamera2 are up, so
    # this cannot be created lazily at that point. It sits at 0 frames/0% CPU for the
    # entire drive; parking.py only starts writing to it once turn 13 is reached, so
    # the driving video never has to share space with parking's telemetry overlay --
    # see _record_parking() in maneuvers.py.
    parking_video_path = os.path.join(run_folder, 'parking.mp4')
    parking_video_encoder = VideoEncoderProcess(
        parking_video_path, VIDEO_FOURCC, every_n=args.every_n)
    parking_video_encoder.start()
    log.info("Recording parking footage to %s (%s)", parking_video_path, VIDEO_FOURCC)

    # ---- Hardware ---------------------------------------------------------
    if not camera.initialize():
        # Most common cause: something else (a colour tuner, another run) still has
        # the camera open -- picamera2 fails init but capture_frame() would then just
        # return None forever, so the run would drive fully blind instead of erroring.
        # Bail out now, before any motor/servo/thread is touched, and clean up the
        # three processes already started above.
        log.error("Camera failed to initialize (already in use by another process?). "
                  "Aborting run before touching motors/threads.")
        try:
            video_encoder.stop()
        except Exception:
            log.exception("Error stopping encoder")
        try:
            parking_video_encoder.stop()
        except Exception:
            log.exception("Error stopping parking encoder")
        if overlay_log is not None:
            overlay_log.close()
        if vision_pool is not None:
            try:
                vision_pool.stop()
            except Exception:
                log.exception("Error stopping vision pool")
        shutdown_logging()
        sys.exit(1)
    motor.initialize()
    servo.initialize()
    button = Button(config.BUTTON_PIN)
    led = LED(config.LED_PIN)

    orange_detection_history = deque([False] * ORANGE_DETECTION_HISTORY_LENGTH, maxlen=ORANGE_DETECTION_HISTORY_LENGTH)
    cooldown_frames = 0
    orange_detection_history.append(False)
    turn_counter = 0
    angle = 0
    prevangle = 0
    prev_rpm = 0
    prev_wall_error = 0.0

    camera_thread = CameraThread(camera)
    camera_thread.start()

    sensors_initialized_event = threading.Event()
    sensor_thread = SensorThread(distance, sensors_initialized_event)
    sensor_thread.start()

    imu_initialized_event = threading.Event()
    imu_thread = ImuThread(bno086, imu_initialized_event)
    imu_thread.start()

    log.info("Waiting for distance sensors and IMU to initialize...")
    sensors_initialized_event.wait()
    log.info("Sensors are ready.")
    imu_initialized_event.wait()
    log.info("IMU is ready. Locking dynamic calibration for run...")
    imu_thread.lock_calibration()
    log.info("Proceeding with main logic.")
    led.on()
    if args.button:
        log.info("Waiting for button press...")
        button.wait_for_press()
        log.info("Button pressed! Starting run.")
        led.off()
        time.sleep(1)
    led.off()
    subprocess.run(["pinctrl", "FAN_PWM", "op", "dl"], check=False)
    driving_direction = 'clockwise'
    past_frame_counter = 0
    max_direction_frames = 60
    consecutive_required = 3
    candidate_direction = None
    consecutive_count = 0

    log.info("Detecting start driving direction from camera frames...")
    for _ in range(max_direction_frames):
        frame, past_frame_counter, _ = camera_thread.get_next_frame(past_frame_counter)
        if frame is None:
            continue

        detections = process_video_frame(frame)
        wall_inner_left_size = sum(obj['area'] for obj in detections.get('detected_walls', []) if obj['type'] == 'wall_inner_left')
        wall_inner_right_size = sum(obj['area'] for obj in detections.get('detected_walls', []) if obj['type'] == 'wall_inner_right')

        if wall_inner_left_size == 0 and wall_inner_right_size == 0:
            current_dir_label = "unknown"
            consecutive_count = 0
            log.debug("Direction detect: 0 px on both sides (left=0, right=0), waiting for more frames...")
        elif wall_inner_left_size > wall_inner_right_size:
            current_dir_label = "clockwise"
            if candidate_direction == current_dir_label:
                consecutive_count += 1
            else:
                candidate_direction = current_dir_label
                consecutive_count = 1
        elif wall_inner_right_size > wall_inner_left_size:
            current_dir_label = "counter-clockwise"
            if candidate_direction == current_dir_label:
                consecutive_count += 1
            else:
                candidate_direction = current_dir_label
                consecutive_count = 1
        else:
            current_dir_label = "equal"
            consecutive_count = 0

        debug_items = [
            f"StartDir:{current_dir_label}",
            f"L:{int(wall_inner_left_size)}",
            f"R:{int(wall_inner_right_size)}",
        ]

        # Record direction detection frames into the video
        if record_raw:
            if video_encoder.write(frame, tag=past_frame_counter):
                if overlay_log is not None:
                    overlay_log.append(
                        len(video_encoder.tags) - 1,
                        current_dir_label,
                        debug_items,
                        visual_target_x=None,
                        visual_target_line=None,
                        detections=detections,
                    )
        else:
            video_encoder.write(
                frame, tag=past_frame_counter,
                prepare=lambda f, det=detections, d_dir=current_dir_label, dbg=debug_items: annotate_video_frame(
                    f, det, d_dir,
                    debug_info=dbg,
                    visual_target_x=None,
                    visual_target_line=None,
                ),
            )

        log.info("Direction detect frame - left: %.0f, right: %.0f -> candidate: %s (%d/%d)",
                 wall_inner_left_size, wall_inner_right_size,
                 current_dir_label, consecutive_count, consecutive_required)

        if candidate_direction is not None and consecutive_count >= consecutive_required:
            driving_direction = candidate_direction
            break

    if candidate_direction is not None and consecutive_count < consecutive_required:
        driving_direction = candidate_direction
        log.warning("Direction detection reached max frames without full consensus, using last candidate: %s", driving_direction)
    elif candidate_direction is None:
        log.warning("Direction detection found 0 px on both sides across %d frames, defaulting to clockwise",
                    max_direction_frames)
        driving_direction = 'clockwise'

    log.info("Decided driving direction: %s", driving_direction)

    INITIAL_HEADING = None
    while INITIAL_HEADING is None:
        heading = imu_thread.get_heading()
        if heading is not None:
            INITIAL_HEADING = heading
            break
        log.debug("Waiting for first valid heading reading...")
        time.sleep(0.05)
    log.info("Initial heading locked: %.1f°", INITIAL_HEADING)

    # Hand the scripted sequences the live objects and run-level values they read.
    # Everything they need now exists; nothing below may rebind these.
    control.bind(imu_thread=imu_thread)
    maneuvers.bind(
        camera_thread=camera_thread,
        imu_thread=imu_thread,
        sensor_thread=sensor_thread,
        video_encoder=video_encoder,
        parking_video_encoder=parking_video_encoder,
        # Tells the maneuvers to record untouched frames too -- several of them draw
        # straight onto the frame they write, which would poison raw.mp4.
        RECORD_RAW=record_raw,
        overlay_log=overlay_log,
        INITIAL_HEADING=INITIAL_HEADING,
        driving_direction=driving_direction,
    )

    perf = PerfMonitor()

    # Set once turn 13 starts the background rebuild (see the turn_counter >= 13
    # branch below). Declared here, before the run can fail, so the `finally` block
    # can tell "rebuild already started in the background" apart from "never reached
    # turn 13" -- an aborted run (exception, or the stop button before turn 13) needs
    # the old synchronous rebuild as a fallback, since nothing else will have done it.
    bg_annotator = None

    try:
        run_start_time = time.monotonic()
        past_frame_counter = 0
        frame_counter = 0
        maneuvers.perform_initial_maneuver()
        log.info("Initial maneuver done; entering main control loop.")

        motor.start_rpm_control(INITIAL_RPM, "forward")
        # Same detail is already in the overlay JSON (one record per recorded frame,
        # replayable by annotate_run.py), so this is just a coarse human-readable
        # heartbeat -- throttled hard to keep the log file grep-able.
        frame_debug_throttle = Throttle(1.0)
        prev_rpm = INITIAL_RPM
        last_block_target_rpm = INITIAL_RPM
        frames_since_block_seen = BLOCK_TARGET_GRACE_FRAMES
        loop_frames = 0
        tracking_green_block = False
        green_post_pass_frames = 0
        tracking_red_block = False
        red_post_pass_frames = 0
        prev_target_error = 0.0

        while True:
            target_rpm = INITIAL_RPM
            angle = 0
            active_block_y = None
            debug = []
            visual_target_x = None
            visual_target_line = None

            # Blocking wait -- no busy-spin. `capture_ts` is when the camera thread
            # actually received this frame, which is what capture->servo latency is
            # measured against.
            frame, frame_counter, capture_ts = camera_thread.get_next_frame(past_frame_counter)
            if frame is None:
                continue
            skipped = max(0, frame_counter - past_frame_counter - 1)
            past_frame_counter = frame_counter
            loop_frames += 1

            sensor_readings = sensor_thread.get_readings()

            t_proc = time.perf_counter()
            detections = process_video_frame(frame)
            proc_ms = (time.perf_counter() - t_proc) * 1000.0

            if detections.get('detected_magenta'):
                if driving_direction == 'clockwise':
                    detections['detected_magenta'].sort(key=lambda x: x['centroid'][0])
                else:
                    detections['detected_magenta'].sort(key=lambda x: x['centroid'][0], reverse=True)
            detected_blocks = [
                b for b in detections['detected_blocks']
                if not (
                    (
                        b.get('color') == 'green'
                        and b.get('centroid', (0, 0))[0] < 200
                        and b.get('contour') is not None
                        and len(b['contour']) > 0
                        and b['contour'].reshape(-1, 2)[:, 1].min() <= full_frame_roi[1] + 2
                    )
                    or (
                        b.get('color') == 'red'
                        and b.get('centroid', (0, 0))[0] > 440
                        and b.get('contour') is not None
                        and len(b['contour']) > 0
                        and b['contour'].reshape(-1, 2)[:, 1].min() <= full_frame_roi[1] + 2
                    )
                )
            ]
            detected_walls = detections['detected_walls']
            detected_orange_object = detections['detected_orange']
            detected_blue_object = detections['detected_blue']
            close_black_area = sum(obj['area'] for obj in detections.get('detected_close_black', []))

            blue_detected_this_frame = bool(detected_blue_object)
            orange_detected_this_frame = bool(detected_orange_object)
            orange_detection_history.append(orange_detected_this_frame)

            if cooldown_frames > 0:
                cooldown_frames -= 1
            elif not orange_detection_history[-ORANGE_DETECTION_HISTORY_LENGTH] and all(list(orange_detection_history)[1:]):
                turn_counter += 1
                cooldown_frames = ORANGE_COOLDOWN_FRAMES
                log.info("turn_counter ----------------> %d", turn_counter)

            if detected_blocks:
                prev_wall_error = 0.0
                is_close_block = False
                for block in detected_blocks:
                    if block['type'] == 'close_block':
                        is_close_block = True
                        if block['color'] == 'magenta' and (time.monotonic()-run_start_time) > 5:
                            if driving_direction == 'clockwise':
                                angle = -25
                            else:
                                angle = 30
                        elif block['color'] == 'red':
                            angle = -25
                        elif block['color'] == 'green':
                            angle = 30
                        # elif block['color'] == 'black':
                        #     # Temporarily disabled reverse on close black
                        #     if driving_direction == 'clockwise':
                        #         angle = 30
                        #     else:
                        #         angle = -25
                        else:
                            is_close_block = False
                            break
                        log.warning("CLOSE BLOCK (%s) -- reverse-and-swerve, angle=%d",
                                    block['color'], angle)
                        motor.stop_rpm_control()
                        servo.set_angle(angle)
                        # BUG: direct duty + stopwatch. This is the reflex that backs
                        # off a pillar filling the frame, and it is the WORST place in
                        # the program to be open-loop: 60% duty for 0.5 s covers a very
                        # different distance on a fresh battery than on a flat one, and
                        # it is running because we are about to hit something. Convert
                        # to encoder distance, e.g.
                        #     control.drive_distance_with_gyro(heading, 12, rpm=80,
                        #                                      direction='reverse')
                        # then the same 12 cm comes back every time. Needs a heading to
                        # hold -- capture imu_thread.get_heading() before the swerve.
                        motor.reverse(60)
                        time.sleep(0.5)
                        motor.forward(60)
                        servo.set_angle(-angle)
                        time.sleep(0.3)
                        motor.start_rpm_control(INITIAL_RPM, "forward")
                        prev_rpm = INITIAL_RPM
                        target_rpm = INITIAL_RPM
                        last_block_target_rpm = INITIAL_RPM
                        frames_since_block_seen = BLOCK_TARGET_GRACE_FRAMES
                        if block['color'] == 'green' or (block['color'] == 'black' and driving_direction == 'clockwise'):
                            green_post_pass_frames = POST_GREEN_CLIP_FRAMES
                        elif block['color'] == 'red' or (block['color'] == 'black' and driving_direction == 'counter-clockwise'):
                            red_post_pass_frames = POST_RED_CLIP_FRAMES
                        tracking_green_block = False
                        tracking_red_block = False
                        break

                if not is_close_block:
                    candidate_blocks = [b for b in detected_blocks if b['type'] == 'block']
                    block = None
                    if candidate_blocks:
                        b0 = candidate_blocks[0]
                        if len(candidate_blocks) > 1:
                            b1 = candidate_blocks[1]
                            # Slanted threshold for red-to-red block handoff:
                            if RED_HANDOFF_X_CENTER != RED_HANDOFF_X_EDGE:
                                red_req_y = RED_HANDOFF_Y_EDGE + (RED_HANDOFF_Y_CENTER - RED_HANDOFF_Y_EDGE) * (
                                    (b0['centroid'][0] - RED_HANDOFF_X_EDGE) / (RED_HANDOFF_X_CENTER - RED_HANDOFF_X_EDGE)
                                )
                            else:
                                red_req_y = RED_HANDOFF_Y_CENTER

                            if (
                                b0['color'] == 'red'
                                and b1['color'] == 'red'
                                and b0['centroid'][0] < RED_HANDOFF_X_CENTER
                                and b0['centroid'][1] >= red_req_y
                            ):
                                block = b1
                            elif b0['color'] == 'green' and b1['color'] == 'green' and b0['centroid'][0] > 500 and b0['centroid'][1] >= GREEN_HANDOFF_MIN_Y:
                                block = b1
                            else:
                                block = b0
                        else:
                            block = b0

                    if block is not None:
                        block_color = block['color']
                        block_x, block_y = block['centroid']
                        active_block_y = block_y
                        frames_since_block_seen = 0

                        if block_color == 'red':
                            if tracking_green_block:
                                tracking_green_block = False
                                green_post_pass_frames = POST_GREEN_CLIP_FRAMES
                            tracking_red_block = True
                            red_post_pass_frames = 0
                            # TUNING PARAMETERS
                            RED_OTHER_X = 297
                            RED_OTHER_Y = 0
                            RED_ORIGIN_X = 0
                            RED_ORIGIN_Y = FRAME_HEIGHT
                            RED_IDEAL_ANGLE = math.degrees(math.atan2(RED_OTHER_X - RED_ORIGIN_X, RED_ORIGIN_Y - RED_OTHER_Y))

                            wall_inner_left_size = sum(obj['area'] for obj in detected_walls if obj['type'] == 'wall_inner_left')
                            wall_inner_right_size = sum(obj['area'] for obj in detected_walls if obj['type'] == 'wall_inner_right')
                            target = 300 if block_y > 170 and 200 < block_x < 440 else 150
                            candidate_magentas = [
                                m for m in detections.get('detected_magenta', [])
                                if (m['centroid'][0] - block_x) >= 30 and abs(m['target_y'] - block_y) < 45
                            ]
                            if candidate_magentas and driving_direction == 'counter-clockwise':
                                chosen_magenta = min(candidate_magentas, key=lambda m: m['centroid'][0] - block_x)
                                magenta_left_x = chosen_magenta['contour'][:, 0, 0].min()
                                red_right_x = block['contour'][:, 0, 0].max()
                                target_x_line = int(0.55 * magenta_left_x + 0.45 * red_right_x)
                                visual_target_x = target_x_line
                                target_error = target_x_line - FRAME_MIDPOINT_X
                                target_derivative = target_error - prev_target_error
                                angle = (target_error * TARGET_LINE_KP) + (target_derivative * TARGET_LINE_KD)
                                prev_target_error = target_error
                            elif driving_direction == 'clockwise' and wall_inner_right_size > 0:
                                pts = block['contour'].reshape(-1, 2)
                                bottom_pts = pts[pts[:, 1] >= pts[:, 1].max() - 2]
                                corner_x = int(bottom_pts[:, 0].max()) if len(bottom_pts) > 0 else int(pts[:, 0].max())
                                corner_y = int(pts[:, 1].max())

                                found_black_x = None
                                pure_black_mask = detections.get('pure_black_mask')
                                if pure_black_mask is not None:
                                    y_min_bound = inner_right_roi_y
                                    y_max_bound = inner_right_roi_y + inner_right_roi_h - 1
                                    y_start = max(y_min_bound, corner_y - 10)
                                    y_end = min(y_max_bound, corner_y + 10)
                                    scan_start_x = max(corner_x + 1, inner_right_roi_x)
                                    scan_end_x = inner_right_roi_x + inner_right_roi_w - 1
                                    ys_min = max(1, y_start - GLOBAL_Y_OFFSET)
                                    ys_max = min(pure_black_mask.shape[0] - 2, y_end - GLOBAL_Y_OFFSET)
                                    xs_min = max(1, scan_start_x)
                                    xs_max = min(pure_black_mask.shape[1] - 2, scan_end_x)
                                    if ys_min <= ys_max and xs_min <= xs_max:
                                        crop = pure_black_mask[ys_min - 1 : ys_max + 2, xs_min - 1 : xs_max + 2]
                                        eroded = cv2.erode(crop, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
                                        valid_eroded = eroded[1:-1, 1:-1]
                                        col_has_black = np.any(valid_eroded > 0, axis=0)
                                        cols = np.where(col_has_black)[0]
                                        if len(cols) > 0:
                                            found_black_x = int(xs_min + cols[0])

                                if found_black_x is not None:
                                    target_x_line = int((corner_x + found_black_x) / 2)
                                    visual_target_x = target_x_line
                                    target_error = target_x_line - FRAME_MIDPOINT_X
                                    target_derivative = target_error - prev_target_error
                                    angle = (target_error * TARGET_LINE_KP) + (target_derivative * TARGET_LINE_KD)
                                    prev_target_error = target_error
                                else:
                                    prev_target_error = 0.0
                                    visual_target_line = ((RED_ORIGIN_X, RED_ORIGIN_Y), (block_x, block_y), RED_IDEAL_ANGLE, (RED_OTHER_X, RED_OTHER_Y))
                                    current_angle = math.degrees(math.atan2(block_x - RED_ORIGIN_X, RED_ORIGIN_Y - block_y))
                                    angle = (current_angle - RED_IDEAL_ANGLE) * 1.5
                            else:
                                prev_target_error = 0.0
                                visual_target_line = ((RED_ORIGIN_X, RED_ORIGIN_Y), (block_x, block_y), RED_IDEAL_ANGLE, (RED_OTHER_X, RED_OTHER_Y))
                                current_angle = math.degrees(math.atan2(block_x - RED_ORIGIN_X, RED_ORIGIN_Y - block_y))
                                angle = (current_angle - RED_IDEAL_ANGLE) * 1.5
                            if wall_inner_left_size > 3000: angle = np.clip(angle, 15, 45)
                            elif wall_inner_right_size > 3000: angle = np.clip(angle, -45, -15)
                            else: angle = np.clip(angle, -45, 35)

                            debug.append(f"Blk:R({int(block_x)},{int(block_y)})")
                            if visual_target_x is not None:
                                debug.append(f"MidX:{int(visual_target_x)}")
                            debug.append(f"Tgt:{int(target)}")
                            debug.extend([f"IL:{int(wall_inner_left_size)}", f"IR:{int(wall_inner_right_size)}"])

                        elif block_color == 'green':
                            if tracking_red_block:
                                tracking_red_block = False
                                red_post_pass_frames = POST_RED_CLIP_FRAMES
                            tracking_green_block = True
                            green_post_pass_frames = 0
                            # TUNING PARAMETERS
                            GREEN_OTHER_X = 343
                            GREEN_OTHER_Y = 0
                            GREEN_ORIGIN_X = FRAME_WIDTH
                            GREEN_ORIGIN_Y = FRAME_HEIGHT
                            GREEN_IDEAL_ANGLE = math.degrees(math.atan2(GREEN_OTHER_X - GREEN_ORIGIN_X, GREEN_ORIGIN_Y - GREEN_OTHER_Y))

                            wall_inner_left_size = sum(obj['area'] for obj in detected_walls if obj['type'] == 'wall_inner_left')
                            wall_inner_right_size = sum(obj['area'] for obj in detected_walls if obj['type'] == 'wall_inner_right')
                            target = 300 if block_y > 170 and 200 < block_x < 440 else 150
                            candidate_magentas = [
                                m for m in detections.get('detected_magenta', [])
                                if (block_x - m['centroid'][0]) >= 30 and abs(m['target_y'] - block_y) < 45
                            ]
                            if candidate_magentas and driving_direction == 'clockwise':
                                chosen_magenta = min(candidate_magentas, key=lambda m: block_x - m['centroid'][0])
                                magenta_right_x = chosen_magenta['contour'][:, 0, 0].max()
                                green_left_x = block['contour'][:, 0, 0].min()
                                target_x_line = int(0.55 * magenta_right_x + 0.45 * green_left_x)
                                visual_target_x = target_x_line
                                target_error = target_x_line - FRAME_MIDPOINT_X
                                target_derivative = target_error - prev_target_error
                                angle = (target_error * TARGET_LINE_KP) + (target_derivative * TARGET_LINE_KD)
                                prev_target_error = target_error
                            elif driving_direction == 'counter-clockwise' and wall_inner_left_size > 0:
                                pts = block['contour'].reshape(-1, 2)
                                bottom_pts = pts[pts[:, 1] >= pts[:, 1].max() - 2]
                                corner_x = int(bottom_pts[:, 0].min()) if len(bottom_pts) > 0 else int(pts[:, 0].min())
                                corner_y = int(pts[:, 1].max())

                                found_black_x = None
                                pure_black_mask = detections.get('pure_black_mask')
                                if pure_black_mask is not None:
                                    y_min_bound = inner_left_roi_y
                                    y_max_bound = inner_left_roi_y + inner_left_roi_h - 1
                                    y_start = max(y_min_bound, corner_y - 10)
                                    y_end = min(y_max_bound, corner_y + 10)
                                    scan_start_x = min(corner_x - 1, inner_left_roi_x + inner_left_roi_w - 1)
                                    scan_end_x = inner_left_roi_x
                                    ys_min = max(1, y_start - GLOBAL_Y_OFFSET)
                                    ys_max = min(pure_black_mask.shape[0] - 2, y_end - GLOBAL_Y_OFFSET)
                                    xs_min = max(1, scan_end_x)
                                    xs_max = min(pure_black_mask.shape[1] - 2, scan_start_x)
                                    if ys_min <= ys_max and xs_min <= xs_max:
                                        crop = pure_black_mask[ys_min - 1 : ys_max + 2, xs_min - 1 : xs_max + 2]
                                        eroded = cv2.erode(crop, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
                                        valid_eroded = eroded[1:-1, 1:-1]
                                        col_has_black = np.any(valid_eroded > 0, axis=0)
                                        cols = np.where(col_has_black)[0]
                                        if len(cols) > 0:
                                            found_black_x = int(xs_min + cols[-1])

                                if found_black_x is not None:
                                    target_x_line = int((corner_x + found_black_x) / 2)
                                    visual_target_x = target_x_line
                                    target_error = target_x_line - FRAME_MIDPOINT_X
                                    target_derivative = target_error - prev_target_error
                                    angle = (target_error * TARGET_LINE_KP) + (target_derivative * TARGET_LINE_KD)
                                    prev_target_error = target_error
                                else:
                                    prev_target_error = 0.0
                                    visual_target_line = ((GREEN_ORIGIN_X, GREEN_ORIGIN_Y), (block_x, block_y), GREEN_IDEAL_ANGLE, (GREEN_OTHER_X, GREEN_OTHER_Y))
                                    current_angle = math.degrees(math.atan2(block_x - GREEN_ORIGIN_X, GREEN_ORIGIN_Y - block_y))
                                    angle = (current_angle - GREEN_IDEAL_ANGLE) * 1.5
                            else:
                                prev_target_error = 0.0
                                visual_target_line = ((GREEN_ORIGIN_X, GREEN_ORIGIN_Y), (block_x, block_y), GREEN_IDEAL_ANGLE, (GREEN_OTHER_X, GREEN_OTHER_Y))
                                current_angle = math.degrees(math.atan2(block_x - GREEN_ORIGIN_X, GREEN_ORIGIN_Y - block_y))
                                angle = (current_angle - GREEN_IDEAL_ANGLE) * 1.5
                            if wall_inner_left_size > 3000: angle = np.clip(angle, 15, 45)
                            elif wall_inner_right_size > 3000: angle = np.clip(angle, -45, -15)
                            else: angle = np.clip(angle, -35, 45)

                            debug.append(f"Blk:G({int(block_x)},{int(block_y)})")
                            if visual_target_x is not None:
                                debug.append(f"MidX:{int(visual_target_x)}")
                            debug.append(f"Tgt:{int(target)}")
                            debug.extend([f"IL:{int(wall_inner_left_size)}", f"IR:{int(wall_inner_right_size)}"])
                    else:
                        if tracking_green_block:
                            tracking_green_block = False
                            green_post_pass_frames = POST_GREEN_CLIP_FRAMES
                        if tracking_red_block:
                            tracking_red_block = False
                            red_post_pass_frames = POST_RED_CLIP_FRAMES
                        if frames_since_block_seen < BLOCK_TARGET_GRACE_FRAMES:
                            frames_since_block_seen += 1
            else:
                if tracking_green_block:
                    tracking_green_block = False
                    green_post_pass_frames = POST_GREEN_CLIP_FRAMES
                if tracking_red_block:
                    tracking_red_block = False
                    red_post_pass_frames = POST_RED_CLIP_FRAMES
                if frames_since_block_seen < BLOCK_TARGET_GRACE_FRAMES:
                    frames_since_block_seen += 1
                prev_target_error = 0.0
                left_pixel_size,right_pixel_size,wall_inner_left_size,wall_inner_right_size,target=0,0,0,0,0
                left_pixel_size = sum(obj['area'] for obj in detected_walls if obj['type'] == 'wall_left')
                right_pixel_size = sum(obj['area'] for obj in detected_walls if obj['type'] == 'wall_right')
                wall_inner_left_size = sum(obj['area'] for obj in detected_walls if obj['type'] == 'wall_inner_left')
                wall_inner_right_size = sum(obj['area'] for obj in detected_walls if obj['type'] == 'wall_inner_right')

                total_left = left_pixel_size + wall_inner_left_size
                total_right = right_pixel_size + wall_inner_right_size

                if total_left < 700 and total_right > 100:
                    total_right = total_right * 2 + 25000
                elif total_right < 700 and total_left > 100:
                    total_left = total_left * 2 + 25000

                debug.extend([f"L:{int(left_pixel_size)}", f"R:{int(right_pixel_size)}"])
                debug.extend([f"IL:{int(wall_inner_left_size)}", f"IR:{int(wall_inner_right_size)}"])
                wall_error = total_left - total_right
                wall_derivative = wall_error - prev_wall_error
                debug.append(f"Err:{int(wall_error)}")
                debug.append(f"Wall%:{int(detections.get('line_roi_wall_pct', 0))}")
                angle = (wall_error * WALL_KP) + (wall_derivative * WALL_KD) + 1
                prev_wall_error = wall_error
                if close_black_area > 3000 or detections.get('line_roi_wall_pct', 0) > 50:
                    if driving_direction == 'clockwise':
                        angle += 35
                    else:
                        angle += -35
                if total_left == 0 and total_right == 0 and (detected_orange_object or detected_blue_object):
                    if driving_direction == 'clockwise':
                        angle += 35
                    else:
                        angle += -35
                if total_left == 0 and total_right == 0:
                    if driving_direction == 'clockwise':
                        angle += 20
                    else:
                        angle += -20
                elif left_pixel_size==0 and driving_direction=='counter-clockwise':
                    angle += -20
                elif right_pixel_size==0 and driving_direction=='clockwise':
                    angle += 20

            # ---- ACTUATE FIRST ------------------------------------------
            # Steering happens here, immediately after the decision. Annotation,
            # recording and telemetry all come afterwards. The old loop did the
            # opposite -- annotate, write, then sleep(1/60 - elapsed), THEN steer --
            # which put up to ~25 ms between seeing a frame and reacting to it.
            if green_post_pass_frames > 0:
                if close_black_area > 3000 or detections.get('line_roi_wall_pct', 0) > 50:
                    angle = np.clip(angle, -40, 40)
                    debug.append(f"GClipOvr:{green_post_pass_frames}")
                else:
                    angle = np.clip(angle, -POST_GREEN_MAX_ANGLE, POST_GREEN_MAX_ANGLE)
                    debug.append(f"GClip:{green_post_pass_frames}")
                green_post_pass_frames -= 1
            elif red_post_pass_frames > 0:
                if close_black_area > 3000 or detections.get('line_roi_wall_pct', 0) > 50:
                    angle = np.clip(angle, -40, 40)
                    debug.append(f"RClipOvr:{red_post_pass_frames}")
                else:
                    angle = np.clip(angle, -POST_RED_MAX_ANGLE, POST_RED_MAX_ANGLE)
                    debug.append(f"RClip:{red_post_pass_frames}")
                red_post_pass_frames -= 1
            else:
                angle = np.clip(angle, -40, 40)
            if angle != prevangle:
                servo.set_angle(angle)
            prevangle = angle

            # Dynamic speed calculation based on linear block height and steering angle
            if USE_VARIABLE_SPEED:
                a = min(1.0, abs(angle) / 40.0)
                f_steering = 1.0 - (0.5 * a)

                if active_block_y is not None:
                    roi_y_min = full_frame_roi[1]
                    roi_y_max = full_frame_roi[1] + full_frame_roi[3]
                    # Linear scale from top of ROI (y_min) down to 20px before ROI bottom.
                    # Lower 20px of ROI stays at lowest RPM (MIN_RPM).
                    scale_y_max = max(roi_y_min + 1, roi_y_max - 20)
                    y_clamped = np.clip(active_block_y, roi_y_min, scale_y_max)
                    h = 1.0 - (y_clamped - roi_y_min) / (scale_y_max - roi_y_min)
                    f_height = h
                    target_rpm = MIN_RPM + (MAX_RPM - MIN_RPM) * f_height * f_steering
                    last_block_target_rpm = target_rpm
                elif frames_since_block_seen < BLOCK_TARGET_GRACE_FRAMES:
                    target_rpm = last_block_target_rpm * f_steering
                else:
                    target_rpm = MIN_RPM + (MAX_RPM - MIN_RPM) * f_steering
            else:
                target_rpm = MIN_RPM

            # Apply rate limiter (+10 RPM accel, -200 RPM decel per frame)
            rpm_diff = target_rpm - prev_rpm
            rpm_diff = np.clip(rpm_diff, -MAX_DECEL_PER_FRAME, MAX_ACCEL_PER_FRAME)
            commanded_rpm = prev_rpm + rpm_diff

            if commanded_rpm != prev_rpm:
                motor.set_rpm_target(commanded_rpm, "forward")
            prev_rpm = commanded_rpm

            latency_ms = (time.monotonic() - capture_ts) * 1000.0
            # ---- everything below is off the critical path ----------------

            perf.add(latency_ms, proc_ms, skipped)
            perf.maybe_report()

            debug.append(f"Angle:{int(angle)}")
            debug.append(f"Turns:{turn_counter}")
            debug.append(f"RPM:{int(commanded_rpm)}")
            debug.append(f"Lat:{int(latency_ms)}ms")
            debug.append(f"F:{frame_counter}")
            debug.append(f"Dir:{driving_direction or '?'}")

            blocks_detail = ",".join(f"{b['color']}:{int(b['area'])}@y={b['centroid'][1]}" for b in detected_blocks) if detected_blocks else "-"

            if frame_debug_throttle:
                log.debug("f=%d angle=%+6.1f rpm=%4.0f turns=%d lat=%5.2fms vis=%4.2fms "
                          "blocks=%d[%s] walls=%d skip=%d",
                          frame_counter, float(angle), commanded_rpm, turn_counter,
                          latency_ms, proc_ms, len(detected_blocks), blocks_detail, len(detected_walls),
                          skipped)

            if record_raw:
                # The frame goes in untouched -- no annotation, no filtering. Both are
                # reapplied offline by annotate_run.py, which is the whole point: the
                # loop pays for a memcpy instead of a draw pass.
                if video_encoder.write(frame, tag=frame_counter):
                    # tags gets one entry per accepted frame, so its last index IS the
                    # video frame number this overlay record belongs to. append() only
                    # buffers in memory now -- the json.dumps()+file write is real
                    # synchronous disk I/O that used to run right here on the control
                    # loop, gating how soon the loop got back to the next frame. It's
                    # deferred to OverlayLog.close() (run once, after the loop exits).
                    overlay_log.append(
                        len(video_encoder.tags) - 1, driving_direction, debug,
                        visual_target_x, visual_target_line, detections)
            else:
                # Annotation is handed to write() rather than done first, so the 1.88 ms
                # is only spent on frames that actually get recorded.
                video_encoder.write(
                    frame, tag=frame_counter,
                    prepare=lambda f: annotate_video_frame(
                        f, detections, driving_direction, debug_info=debug,
                        visual_target_x=visual_target_x,
                        visual_target_line=visual_target_line))

            angle = 0

            if button.is_pressed:
                log.info("Stop button pressed -- ending run.")
                motor.stop_rpm_control()
                motor.brake()
                break
            if turn_counter >= 13:
                motor.stop_rpm_control()
                parking_start_time = time.monotonic()
                time_before_parking = parking_start_time - run_start_time
                log.info("--- Time before parking (13 turns complete): %.2fs ---", time_before_parking)

                if record_raw and not args.no_annotate:
                    # Finalising the driving video (video_encoder.stop(), which joins
                    # the encoder's child process) and closing overlay_log both have to
                    # happen before annotate_run() can open raw.mp4 -- but NOT on this
                    # thread: video_encoder.stop() blocks until the encoder's queue is
                    # flushed, which is a few hundred ms of unpredictable length, and
                    # doing that here -- right before parking's first move -- turned
                    # into a variable, unbraked coast (robot pauses, then overshoots
                    # forward by however long the flush took). finalize= runs both
                    # calls on the background thread instead, so this thread hands off
                    # to maneuvers.parking()/parking2() immediately. parking() itself
                    # never touches video_encoder any more -- _record_parking() writes
                    # to parking_video_encoder instead -- so deferring the stop here
                    # does not race parking's own recording.
                    def _finalize_driving_video():
                        video_encoder.stop()
                        overlay_log.close()
                    bg_annotator = BackgroundAnnotator(
                        raw_video_path, video_path, finalize=_finalize_driving_video)
                    bg_annotator.start()
                    # Only rebindable exception to "nothing below may rebind these"
                    # above: bg_annotator cannot exist until the driving video it
                    # rebuilds is finalised, which only happens here. Lets parking's
                    # Step 4 magenta-line tracking pause/resume it around the one
                    # part of parking that is itself latency-sensitive on the camera.
                    maneuvers.bind(bg_annotator=bg_annotator)
                    log.info("Re-annotation started in the background during parking.")

                if driving_direction == 'clockwise':
                    maneuvers.parking()
                else:
                    # offset_heading = (INITIAL_HEADING - 5) % 360
                    # log.info("Offsetting initial heading for CCW parking2: %.1f° -> %.1f°", INITIAL_HEADING, offset_heading)
                    # maneuvers.bind(INITIAL_HEADING=offset_heading)
                    maneuvers.parking2()
                run_end_time = time.monotonic()
                log.info("--- Time spent in parking: %.2fs ---", run_end_time - parking_start_time)
                log.info("--- Total run time: %.2fs ---", run_end_time - run_start_time)
                motor.brake()
                servo.set_angle(0)
                break

    except Exception:
        log.exception("ERROR during execution")

    finally:
        log.info("Stopping motor RPM control and cleaning up motor...")
        try:
            motor.stop_rpm_control()
            motor.cleanup()
        except Exception:
            log.exception("Error cleaning up motor")

        try:
            servo.set_angle(0)
            servo.cleanup()
        except Exception:
            log.exception("Error cleaning up servo")

        log.info("Signaling threads to stop...")
        camera_thread.stop()
        sensor_thread.stop()
        imu_thread.stop()

        log.info("Waiting for threads to complete...")
        camera_thread.join(timeout=5)
        sensor_thread.join(timeout=5)
        imu_thread.join(timeout=5)
        log.info("All threads have completed.")

        # Child processes last: the encoders still have queued frames to flush.
        # video_encoder.stop() is a no-op if the turn_counter >= 13 branch already
        # stopped it -- VideoEncoderProcess.stop() tolerates being called twice.
        try:
            video_encoder.stop()
        except Exception:
            log.exception("Error stopping encoder")
        try:
            parking_video_encoder.stop()
        except Exception:
            log.exception("Error stopping parking encoder")
        if overlay_log is not None:
            overlay_log.close()
        if vision_pool is not None:
            try:
                vision_pool.stop()
            except Exception:
                log.exception("Error stopping vision pool")

        camera.cleanup()
        cv2.destroyAllWindows()
        subprocess.run(["pinctrl", "FAN_PWM", "a0"], check=False)

        if record_raw and not args.no_annotate:
            if bg_annotator is not None:
                t_join = time.monotonic()
                bg_annotator.join()
                join_s = time.monotonic() - t_join
                if bg_annotator.error is None:
                    log.info("Background annotation joined (%.2fs wait); %s ready.",
                             join_s, video_path)
                else:
                    log.warning("Background annotation failed (%.2fs wait); rerun "
                                "`python3 -m src.obstacle_challenge.annotate_run %s` "
                                "to retry.", join_s, run_folder)
            else:
                # turn 13 was never reached -- stop button pressed early, or an
                # exception before then -- so nothing has rebuilt obstacle.mp4 yet.
                # Deliberately the last thing done here: the vision workers and
                # hardware threads are all gone, so this has the machine to itself.
                try:
                    cv2.setNumThreads(0)   # 0 = all cores; nothing competing for them
                    log.info("Rebuilding annotated video from %s...", raw_video_path)
                    annotate_run(raw_video_path, video_path)
                    log.info("Annotated video written to %s", video_path)
                except Exception:
                    log.exception("Could not rebuild the annotated video. The run "
                                  "itself is fine -- rerun `python3 -m "
                                  "src.obstacle_challenge.annotate_run %s` to retry",
                                  run_folder)

        log.info("Run complete. Log saved to %s", log_path)
        shutdown_logging()

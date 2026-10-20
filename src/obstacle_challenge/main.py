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
import os
import sys
import time
from datetime import datetime

import cv2
import threading
import _thread
from gpiozero import Button, LED
import subprocess

from src.motors import motor, servo
from src.sensors import bno086, camera, distance

from src.obstacle_challenge import config, control, maneuvers, rules
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


def detect_driving_direction(camera_thread, video_encoder, overlay_log, record_raw):
    """Clockwise if more inner wall is seen on the left than on the right, over
    up to 60 frames (3 agreeing frames in a row decide). Records the frames."""
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

    return driving_direction


def actuate(cmd, prev_angle):
    """Carry out a rules.Command: the swerve reflex if asked for, then servo, then
    motor. Returns the angle now on the servo."""
    if cmd.swerve is not None:
        log.warning("CLOSE BLOCK (%s) -- reverse-and-swerve, angle=%d", cmd.rule, cmd.swerve)
        motor.stop_rpm_control()
        servo.set_angle(cmd.swerve)
        # BUG: direct duty + stopwatch. This is the reflex that backs off a pillar
        # filling the frame, and it is the WORST place in the program to be open-loop:
        # 60% duty for 0.5 s covers a very different distance on a fresh battery than
        # on a flat one, and it is running because we are about to hit something.
        # Convert to encoder distance, e.g.
        #     control.drive_distance_with_gyro(heading, 12, rpm=80, direction='reverse')
        # then the same 12 cm comes back every time. Needs a heading to hold --
        # capture imu_thread.get_heading() before the swerve.
        motor.reverse(CLOSE_SWERVE_REVERSE_DUTY)
        time.sleep(CLOSE_SWERVE_REVERSE_S)
        servo.set_angle(-cmd.swerve)
        motor.forward(CLOSE_SWERVE_FORWARD_DUTY)
        time.sleep(CLOSE_SWERVE_FORWARD_S)
        motor.start_rpm_control(INITIAL_RPM, "forward")
    if cmd.angle != prev_angle:
        servo.set_angle(cmd.angle)
    if cmd.rpm_changed:
        motor.set_rpm_target(cmd.rpm, "forward")
    return cmd.angle


def drive_phase(camera_thread, video_encoder, overlay_log, record_raw, perf,
                driving_direction, run_start_time, mem, stop_when):
    """The per-frame loop: see -> decide (rules.py) -> act -> record. Runs until
    stop_when(world, mem) returns something truthy, checked at the end of every
    frame, and returns that value. `mem` (rules.Memory) carries over between
    phases."""
    motor.start_rpm_control(INITIAL_RPM, "forward")
    mem.prev_rpm = INITIAL_RPM
    # Same detail is already in the overlay JSON (one record per recorded frame,
    # replayable by annotate_run.py), so this is just a coarse human-readable
    # heartbeat -- throttled hard to keep the log file grep-able.
    frame_debug_throttle = Throttle(1.0)
    prev_angle = 0
    past_frame_counter = 0

    while True:
        # Blocking wait -- no busy-spin. `capture_ts` is when the camera thread
        # actually received this frame, which is what capture->servo latency is
        # measured against.
        frame, frame_counter, capture_ts = camera_thread.get_next_frame(past_frame_counter)
        if frame is None:
            continue
        skipped = max(0, frame_counter - past_frame_counter - 1)
        past_frame_counter = frame_counter

        # ---- SEE ----------------------------------------------------------
        t_proc = time.perf_counter()
        detections = process_video_frame(frame)
        proc_ms = (time.perf_counter() - t_proc) * 1000.0
        world = rules.build_world(detections, driving_direction,
                                  time.monotonic() - run_start_time)
        if rules.count_turns(world, mem):
            log.info("turn_counter ----------------> %d", mem.turns)

        # ---- DECIDE + ACT FIRST -------------------------------------------
        # Steering happens immediately after the decision. Annotation, recording and
        # telemetry all come afterwards: the old loop did them first, which put up
        # to ~25 ms between seeing a frame and reacting to it.
        cmd = rules.decide(world, mem)
        prev_angle = actuate(cmd, prev_angle)

        latency_ms = (time.monotonic() - capture_ts) * 1000.0
        # ---- everything below is off the critical path --------------------

        perf.add(latency_ms, proc_ms, skipped)
        perf.maybe_report()

        debug = cmd.debug + [
            f"Angle:{int(cmd.angle)}",
            f"Turns:{mem.turns}",
            f"RPM:{int(cmd.rpm)}",
            f"Lat:{int(latency_ms)}ms",
            f"F:{frame_counter}",
            f"Dir:{driving_direction or '?'}",
            f"Rule:{cmd.rule}",
        ]

        if frame_debug_throttle:
            blocks_detail = ",".join(f"{b['color']}:{int(b['area'])}@y={b['centroid'][1]}"
                                     for b in world.blocks) if world.blocks else "-"
            log.debug("f=%d rule=%s angle=%+6.1f rpm=%4.0f turns=%d lat=%5.2fms vis=%4.2fms "
                      "blocks=%d[%s] walls=%d skip=%d",
                      frame_counter, cmd.rule, float(cmd.angle), cmd.rpm, mem.turns,
                      latency_ms, proc_ms, len(world.blocks), blocks_detail,
                      len(detections['detected_walls']), skipped)

        if record_raw:
            # The frame goes in untouched -- no annotation, no filtering. Both are
            # reapplied offline by annotate_run.py, which is the whole point: the
            # loop pays for a memcpy instead of a draw pass.
            if video_encoder.write(frame, tag=frame_counter):
                # tags gets one entry per accepted frame, so its last index IS the
                # video frame number this overlay record belongs to. append() only
                # buffers in memory; the file is written by OverlayLog.close().
                overlay_log.append(
                    len(video_encoder.tags) - 1, driving_direction, debug,
                    cmd.target_x, cmd.target_line, detections)
        else:
            # Annotation is handed to write() rather than done first, so its cost
            # is only spent on frames that actually get recorded.
            video_encoder.write(
                frame, tag=frame_counter,
                prepare=lambda f: annotate_video_frame(
                    f, detections, driving_direction, debug_info=debug,
                    visual_target_x=cmd.target_x,
                    visual_target_line=cmd.target_line))

        stop = stop_when(world, mem)
        if stop:
            return stop


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
                 "`python3 -m src.tools.annotate_run %s`",
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
    driving_direction = detect_driving_direction(camera_thread, video_encoder, overlay_log, record_raw)

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

    # Set once the finish phase starts the background rebuild (see the finish
    # phase below). Declared here, before the run can fail, so the `finally` block
    # can tell "rebuild already started in the background" apart from "never reached
    # turn 13" -- an aborted run (exception, or the stop button before turn 13) needs
    # the old synchronous rebuild as a fallback, since nothing else will have done it.
    bg_annotator = None

    def _safety_watchdog():
        """E-stop on sustained tilt or a stop-button press, even mid-maneuver.

        The maneuvers block the main thread, so this runs on its own thread and
        brakes the motor itself, then interrupts the main thread to unwind into
        the cleanup in the finally block below."""
        tilt_samples = 0
        while True:
            time.sleep(0.02)
            reason = None
            if button.is_pressed:
                reason = "Stop button pressed"
            else:
                tilt = imu_thread.get_tilt()
                tilt_samples = tilt_samples + 1 if (
                    tilt is not None and tilt > config.TILT_THRESHOLD_DEGREES) else 0
                if tilt_samples >= 5:
                    reason = "Tilt %.1f deg exceeds %.1f deg" % (
                        tilt, config.TILT_THRESHOLD_DEGREES)
            if reason:
                log.error("E-STOP: %s -- stopping run.", reason)
                try:
                    motor.stop_rpm_control()
                    motor.brake()
                except Exception:
                    log.exception("Error braking in E-stop")
                _thread.interrupt_main()
                return

    while button.is_pressed:  # don't treat the start press as a stop press
        time.sleep(0.02)
    threading.Thread(target=_safety_watchdog, name="safety", daemon=True).start()

    try:
        run_start_time = time.monotonic()

        # ---- The run, as phases: start -> drive -> finish -> done ----------
        # Each phase runs once and names the next. To change the shape of the run
        # (skip the start maneuver, stop instead of parking, add a phase), edit
        # DO_INITIAL_MANEUVER / TURNS_TO_FINISH / FINISH_MODE in tuning.py or this
        # block.
        # The rules' memory (turn count, tracking, ...): carries over between phases.
        mem = rules.Memory()
        phase = 'start'
        while phase != 'done':
            log.info("=== Phase: %s ===", phase)

            if phase == 'start':
                if DO_INITIAL_MANEUVER:
                    maneuvers.perform_initial_maneuver()
                    log.info("Initial maneuver done; entering main control loop.")
                phase = 'drive'

            elif phase == 'drive':
                drive_phase(camera_thread, video_encoder, overlay_log, record_raw, perf,
                            driving_direction, run_start_time, mem,
                            stop_when=lambda world, mem: mem.turns >= TURNS_TO_FINISH)
                phase = 'finish'

            elif phase == 'finish':
                motor.stop_rpm_control()
                parking_start_time = time.monotonic()
                log.info("--- Time before finish (%d turns complete): %.2fs ---",
                         TURNS_TO_FINISH, parking_start_time - run_start_time)

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

                if FINISH_MODE == 'park':
                    if driving_direction == 'clockwise':
                        maneuvers.parking()
                    else:
                        maneuvers.parking2()
                else:
                    log.info("FINISH_MODE=%r: stopping without parking.", FINISH_MODE)
                run_end_time = time.monotonic()
                log.info("--- Time spent in finish: %.2fs ---", run_end_time - parking_start_time)
                log.info("--- Total run time: %.2fs ---", run_end_time - run_start_time)
                motor.brake()
                servo.set_angle(0)
                phase = 'done'

            else:
                raise ValueError(f"unknown phase {phase!r}")

    except KeyboardInterrupt:
        log.warning("Run aborted by safety watchdog (or Ctrl-C).")
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
        # video_encoder.stop() is a no-op if the finish phase already
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
                                "`python3 -m src.tools.annotate_run %s` "
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
                                  "src.tools.annotate_run %s` to retry",
                                  run_folder)

        log.info("Run complete. Log saved to %s", log_path)
        shutdown_logging()

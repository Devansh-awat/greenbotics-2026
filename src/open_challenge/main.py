"""
Open Challenge -- main control program.

Shares its whole vision pipeline and multiprocessing model with the obstacle
challenge (see CLAUDE.md and src/vision/pipeline.py / src/vision/pool.py):
the same `process_video_frame()` -- arena mask + colour thresholding
fork-joined across two worker processes via VisionPool -- the same
CameraThread/ImuThread (src/threads/hw_threads.py), the same
VideoEncoderProcess for recording, and the same `robot.*` logging pipeline
(src/logs/setup.py). The per-frame decisions are in src/open_challenge/rules.py
(same World / Memory / Command / RULES shape as the obstacle challenge); this file
holds the setup, actuate(), drive_phase() and the run's phases (drive -> final).

process_video_frame() looks for pillars only inside full_frame_roi /
close_block_roi, which config.py sets to (0, 0, 0, 0) for the open track, so
`detected_blocks` comes back empty. Give them a real size to see pillars here.

Run it from the repo root:  python3 -m src.open_challenge.main
"""

import argparse
import math
import os
import threading
import time
from datetime import datetime

import cv2
from gpiozero import Button, LED

from src.motors import motor, servo
from src.sensors import bno086, camera

from src.open_challenge import config, rules
from src.open_challenge.config import *
from src.obstacle_challenge import control
from src.obstacle_challenge.control import get_angular_difference
from src.threads.hw_threads import CameraThread, ImuThread, PerfMonitor
from src.logs.setup import Throttle, ilog, log, setup_logging, shutdown_logging
from src.obstacle_challenge.video import VideoEncoderProcess
from src.tools.power_mode import ensure_performance
from src.vision import pipeline as vision
from src.vision.pipeline import annotate_video_frame, process_video_frame
from src.vision.pool import VisionPool


def actuate(cmd, prev_angle):
    """Carry out a rules.Command: new rpm target if it changed, then the servo.
    Returns the angle now on the servo."""
    if cmd.rpm_changed:
        motor.set_rpm_target(cmd.rpm, direction="forward")
        log.info("rpm target -> %d (%s)", cmd.rpm,
                 "corner" if cmd.rpm == CORNER_RPM else
                 "straight" if cmd.rpm == STRAIGHT_RPM else cmd.rule)
    if cmd.angle != prev_angle:
        servo.set_angle(cmd.angle)
    return cmd.angle


class DriveContext:
    """The live objects every drive phase needs, created once in __main__."""
    def __init__(self, camera_thread, imu_thread, video_encoder, perf, run_start_time):
        self.camera_thread = camera_thread
        self.imu_thread = imu_thread
        self.video_encoder = video_encoder
        self.perf = perf
        self.run_start_time = run_start_time
        self.past_frame_counter = 0
        self.prev_angle = 0
        self.frame_debug_throttle = Throttle(1.0)
        self.vibration_throttle = Throttle(getattr(config, "VIBRATION_LOG_PERIOD", 0.5))
        self.latest_vibration_deg_s = 0.0


def drive_phase(ctx, mem, stop_when):
    """The per-frame loop: see -> decide (rules.py) -> act -> record. Runs until
    stop_when(world, mem) returns something truthy, checked at the end of every
    frame, and returns that value."""
    while True:
        # Blocking wait -- no busy-spin, same as the obstacle loop.
        frame, frame_counter, capture_ts = ctx.camera_thread.get_next_frame(ctx.past_frame_counter)
        if frame is None:
            continue
        skipped = max(0, frame_counter - ctx.past_frame_counter - 1)
        ctx.past_frame_counter = frame_counter

        # ---- SEE ----------------------------------------------------------
        t_proc = time.perf_counter()
        detections = process_video_frame(frame)
        proc_ms = (time.perf_counter() - t_proc) * 1000.0
        world = rules.build_world(detections, frame, mem, time.monotonic() - ctx.run_start_time)
        if rules.learn_direction(world, mem):
            log.info("%s line seen first -> driving_direction = %s",
                     "Orange" if world.orange else "Blue", mem.direction)
        if rules.count_turns(world, mem):
            log.info("turn_counter ----------------> %d", mem.turns)

        # ---- DECIDE + ACT FIRST -------------------------------------------
        # Annotation and recording come after.
        cmd = rules.decide(world, mem)
        ctx.prev_angle = actuate(cmd, ctx.prev_angle)

        current_rpm = motor.get_measured_rpm()
        latency_ms = (time.monotonic() - capture_ts) * 1000.0
        # ---- everything below is off the critical path --------------------

        ctx.perf.add(latency_ms, proc_ms, skipped)
        ctx.perf.maybe_report(extra=f" | {current_rpm:5.1f} rpm (target {mem.target_rpm:.0f})")

        debug = cmd.debug + [
            f"Angle:{int(cmd.angle)}",
            f"RPM:{int(round(current_rpm))}/{int(mem.target_rpm)}",
            f"WallRow:{world.wall_row}",
            f"Turns:{mem.turns}",
            f"Dir:{mem.direction or '?'}",
            f"Vib:{ctx.latest_vibration_deg_s:.1f}",
            f"Rule:{cmd.rule}",
        ]

        if ctx.vibration_throttle:
            vib = ctx.imu_thread.get_vibration(getattr(config, "VIBRATION_LOG_PERIOD", 0.5))
            if vib:
                ctx.latest_vibration_deg_s = vib["total_deg_s"]
                ilog.info(
                    "Gyro vibration: %5.2f deg/s | pitch/roll wobble: %5.2f deg/s (x: %4.1f, y: %4.1f) | yaw jitter: %5.2f deg/s | peak: %5.2f deg/s (%d samples)",
                    vib["total_deg_s"],
                    vib["pitch_roll_deg_s"],
                    vib["axes_deg_s"][0],
                    vib["axes_deg_s"][1],
                    vib["yaw_jitter_deg_s"],
                    vib["peak_deg_s"],
                    vib["samples"],
                )

        if ctx.frame_debug_throttle:
            log.debug("f=%d rule=%s angle=%+6.1f turns=%d rpm=%5.1f lat=%5.2fms vis=%4.2fms walls=%d skip=%d",
                      frame_counter, cmd.rule, float(cmd.angle), mem.turns, current_rpm, latency_ms,
                      proc_ms, len(detections['detected_walls']), skipped)

        ctx.video_encoder.write(
            frame, tag=frame_counter,
            prepare=lambda f: annotate_video_frame(
                f, detections, mem.direction, debug_info=debug))

        stop = stop_when(world, mem)
        if stop:
            return stop


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
    # The gyro drive moves in control.py (drive_distance_with_gyro, ...) read the
    # IMU through this; bound here so a phase can use them.
    control.bind(imu_thread=imu_thread)

    # The rules' memory: turn count, direction (learned from the first floor line --
    # orange first -> clockwise, blue first -> counter-clockwise), wall error, speed
    # schedule. It carries over from one phase to the next.
    mem = rules.Memory()

    perf = PerfMonitor()
    run_start_time = None

    try:
        run_start_time = time.monotonic()
        motor.start_rpm_control(STRAIGHT_RPM, direction="forward",
                                ff_reseed=RPM_FF_RESEED, brake_err=RPM_BRAKE_ERR)
        ctx = DriveContext(camera_thread, imu_thread, video_encoder, perf, run_start_time)

        # ---- The run, as phases: drive -> final -> done ----------------------
        # Each phase runs once and names the next. To change the shape of the run,
        # edit FINAL_AFTER_TURNS / MAX_TURNS in config.py, or add a phase here.
        phase = 'drive'
        while phase != 'done':
            log.info("=== Phase: %s ===", phase)

            if phase == 'drive':
                # Laps, until the final straight can start (or the safety stop).
                def drive_done(world, mem):
                    if (mem.turns == FINAL_AFTER_TURNS and get_angular_difference(
                            imu_thread.get_heading(), INITIAL_HEADING) < FINAL_HEADING_TOLERANCE):
                        return 'final'
                    if mem.turns >= MAX_TURNS:
                        return 'stop'
                    return None
                phase = drive_phase(ctx, mem, drive_done)

            elif phase == 'final':
                # Back in the start section: drive FINAL_STOP_DISTANCE_CM more, then stop.
                final_start_time = time.monotonic()
                final_start_pos = motor.encoder_position()
                cpr = motor.encoder.counts_per_rev if (motor.encoder is not None) else (48 * 4.4 * 38 / 13)
                wheel_diameter_mm = motor.encoder.wheel_diameter_mm if (motor.encoder is not None) else 62.4
                circ_cm = (math.pi * wheel_diameter_mm) / 10.0
                coast = STRAIGHT_RPM > FINAL_COAST_ABOVE_RPM
                mem.hold_speed = True   # the speed schedule stops changing the rpm
                if coast:
                    # High speed: coast motor immediately, brake when distance or time exceeded
                    motor.stop_rpm_control(brake=False)
                    log.info("%d turns reached & heading aligned. Final run initiated (STRAIGHT_RPM > %d): coasting motor (brake if dist >= %.1f cm or time >= %.2f s)...",
                             FINAL_AFTER_TURNS, FINAL_COAST_ABOVE_RPM, FINAL_STOP_DISTANCE_CM, FINAL_STOP_DELAY)
                else:
                    # Lower speed: continue driving at own speed, stop after distance reached
                    log.info("%d turns reached. Driving final %.1f cm (encoder-based at %.0f RPM) before stop...",
                             FINAL_AFTER_TURNS, FINAL_STOP_DISTANCE_CM, mem.target_rpm)

                def final_done(world, mem):
                    curr_pos = motor.encoder_position()
                    dist_cm = 0.0
                    if curr_pos is not None and final_start_pos is not None:
                        dist_cm = (abs(curr_pos - final_start_pos) / cpr) * circ_cm
                    elapsed = time.monotonic() - final_start_time
                    if dist_cm >= FINAL_STOP_DISTANCE_CM or elapsed >= FINAL_STOP_DELAY:
                        log.info("Final run done (%s: %.1f cm / %.2fs).",
                                 "distance" if dist_cm >= FINAL_STOP_DISTANCE_CM else "time",
                                 dist_cm, elapsed)
                        return 'brake' if coast else 'stop_rpm'
                    if mem.turns >= MAX_TURNS:
                        return 'stop'
                    return None
                ending = drive_phase(ctx, mem, final_done)
                if ending == 'brake':
                    motor.brake()
                    phase = 'done'
                elif ending == 'stop_rpm':
                    motor.stop_rpm_control()
                    phase = 'done'
                else:
                    phase = ending

            elif phase == 'stop':
                motor.stop_rpm_control()
                motor.brake()
                phase = 'done'

            else:
                raise ValueError(f"unknown phase {phase!r}")

    except Exception:
        log.exception("ERROR during execution")

    finally:
        run_end_time = time.monotonic()
        if run_start_time is not None:
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

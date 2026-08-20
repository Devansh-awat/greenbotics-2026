"""The scripted, open-loop-ish sequences: initial maneuver and the two parkings.

These run before and after the main control loop and drive the motor directly, so
they are kept out of main_v5.py -- the only thing left there is the per-frame loop.

They read run-level state (which thread objects to poll, the locked initial
heading, the chosen driving direction) that does not exist until main_v5 has
started its threads. Rather than thread a context object through several hundred
lines of sequencing code, main_v5 calls bind() once at startup and these become
module globals. See bind() below for the full list.

Two tiers of "how far did we go" problems live in here, both marked BUG:

  1. Direct duty (`motor.forward(60)`) -- open loop on speed AND on distance. Fix
     with control.drive_distance_with_gyro(), which closes both.
  2. Closed-loop rpm but ended by a stopwatch (`start_rpm_control(...)` then
     `while time.monotonic() - t0 < N`) -- most of parking()/parking2(). Much
     better, since rpm is held, but the distance still drifts with load. These
     want the encoder too; they are lower priority than tier 1.
"""

import time

import numpy as np

from src.motors import motor, servo
from src.obstacle_challenge.control import (
    drive_distance_with_gyro, drive_straight_with_gyro, get_angular_difference,
    steer_with_gyro,
)
from src.logs.setup import Throttle, _fmt, log, plog
from src.obstacle_challenge.tuning import *
from src.vision.pipeline import annotate_video_frame, process_video_frame

# --- Injected by main_v5.bind_runtime() -------------------------------------
# None of these can be imported: they are live objects created after the hardware
# is up (the threads), or values decided at run time (the heading and direction).
camera_thread = None      # CameraThread
imu_thread = None         # ImuThread
sensor_thread = None      # SensorThread
video_encoder = None      # VideoEncoderProcess -- the driving video (raw or annotated)
parking_video_encoder = None  # VideoEncoderProcess -- parking.mp4, always separate
overlay_log = None        # OverlayLog, or None when not recording raw
RECORD_RAW = False        # True when the encoder is storing untouched frames (-v)
INITIAL_HEADING = None    # float, the heading locked in at the start of the run
driving_direction = None  # 'clockwise' | 'counter-clockwise'
bg_annotator = None       # BackgroundAnnotator rebuilding obstacle.mp4, or None


def bind(**kwargs):
    """Inject the run-level objects and values listed above."""
    globals().update(kwargs)


# --- Recording ---------------------------------------------------------------
# Under -v the encoder holds untouched camera frames for annotate_run.py to rebuild
# from, so in general nothing drawn on may reach it -- hence separate raw/annotated
# helpers rather than one, since these routines draw straight onto the frame they
# write (in place, via cv2.rectangle/putText) and the raw write has to happen first.
#
# Parking is the exception and uses _record_parking(): see its docstring.

def _record_raw(frame):
    """Write an untouched frame. No-op unless recording raw."""
    if RECORD_RAW and video_encoder is not None:
        video_encoder.write(frame)


def _record_annotated(frame):
    """Write a drawn-on frame. No-op when recording raw."""
    if not RECORD_RAW and video_encoder is not None:
        video_encoder.write(frame)


def _record_parking(frame):
    """Write a parking frame to the standalone parking.mp4 -- never the driving video.

    The parking loops draw their own telemetry (servo angle, magenta pixel count,
    "Armed to Stop", the tracked line) and never call process_video_frame, so there is
    nothing here for annotate_run.py to replay -- stitching these into raw.mp4 used to
    need passthrough bookkeeping (append_passthrough) just so the rebuild knew to leave
    them alone. Keeping them in a separate video removes that bookkeeping entirely and
    lets the driving video (raw.mp4/obstacle.mp4) be finalised the moment turn 13
    completes, before parking starts -- see main.py's turn_counter >= 13 branch, which
    is what lets the background re-annotation start immediately instead of waiting for
    parking to also finish writing to it.

    parking_video_encoder sits idle (0 frames, 0% CPU) for the entire drive and only
    starts writing here, so it never competes with anything on the way to turn 13.
    """
    if parking_video_encoder is None:
        return
    parking_video_encoder.write(frame)


def perform_initial_maneuver():
    log.info("--- Executing Full Initial Maneuver ---")

    MANEUVER_SPEED = 55
    SERVO_TURN_ANGLE = 60.0
    if driving_direction == 'clockwise':
        SCAN_TRIGGER_ANGLE_DEG = 25.0
    else:
        SCAN_TRIGGER_ANGLE_DEG = -25
    TOTAL_TURN_ANGLE_DEG = 85.0
    HEADING_LOCK_TOLERANCE = 5.0

    DRIVE_FORWARD_DURATION = 1.4
    DRIVE_FORWARD_SHORT_DURATION = 0
    REVERSE_DURATION = 0.4

    if driving_direction == "clockwise":
        initial_turn_servo = SERVO_TURN_ANGLE
        final_turn_servo = -SERVO_TURN_ANGLE
        direction_modifier = 1
    else:
        initial_turn_servo = -SERVO_TURN_ANGLE
        final_turn_servo = SERVO_TURN_ANGLE
        direction_modifier = -1

    def calculate_target_heading(base, offset):
        return (base + offset + 360) % 360

    scan_heading = calculate_target_heading(INITIAL_HEADING, SCAN_TRIGGER_ANGLE_DEG * direction_modifier)
    ninety_degree_heading = calculate_target_heading(INITIAL_HEADING, TOTAL_TURN_ANGLE_DEG * direction_modifier)
    log.info("Direction: %s | initial heading %.1f° | scan at %.1f° | full turn target %.1f°",
             driving_direction.upper(), INITIAL_HEADING, scan_heading, ninety_degree_heading)

    servo.set_angle_unlimited(initial_turn_servo)
    time.sleep(0.02)
    # BUG: direct duty. The loop below exits on HEADING, so distance isn't the
    # criterion here and this one is only half-broken -- but the turn radius still
    # varies with duty, so how far the robot travels while turning 25 deg changes
    # with battery state. Convert to motor.start_rpm_control(rpm, "forward") so the
    # turn is entered at a repeatable ground speed.
    motor.forward(MANEUVER_SPEED)

    log.info("Starting initial turn...")
    turn_throttle = Throttle(0.2)
    while get_angular_difference((INITIAL_HEADING+SCAN_TRIGGER_ANGLE_DEG)%360, imu_thread.get_heading()) > 10:
        if turn_throttle:
            log.debug("initial turn: heading=%s diff=%.1f", _fmt(imu_thread.get_heading()),
                      get_angular_difference((INITIAL_HEADING+SCAN_TRIGGER_ANGLE_DEG)%360, imu_thread.get_heading()))
        time.sleep(0.01)
    motor.brake()

    log.info("Scan angle reached. Pausing to scan for objects...")
    detected_block_color = None
    scan_start_time = time.monotonic()
    while time.monotonic() - scan_start_time < 1.0:
        frame, frame_counter = camera_thread.get_frame()
        if frame is None: continue

        _record_raw(frame)
        detections = process_video_frame(frame)
        if not RECORD_RAW:
            _record_annotated(
                annotate_video_frame(frame, detections, driving_direction))
        main_blocks = [b for b in detections.get('detected_blocks', []) if b['type'] == 'block']

        if main_blocks:
            if main_blocks[0]['area'] > 1000 and main_blocks[0]['centroid'][0] < 540:
                detected_block_color = main_blocks[0]['color']
                log.info("Block found during scan: %s (area %.0f, x=%d)",
                         detected_block_color.upper(), main_blocks[0]['area'],
                         main_blocks[0]['centroid'][0])
                break

    if detected_block_color is None:
        log.info("Scan complete. No block was detected.")

    log.info("90-degree turn complete. Performing action based on scan...")
    time.sleep(0.5)

    action_taken = "None"
    drive_target_heading = ninety_degree_heading

    if driving_direction == 'counter-clockwise':
        if detected_block_color == 'green':
            # BUG: direct duty. Heading-terminated, so convert to
            # motor.start_rpm_control(50, "forward") for a repeatable turn radius.
            motor.forward(50)
            while get_angular_difference((INITIAL_HEADING - 110) % 360, imu_thread.get_heading()) > 5:
                servo.set_angle(steer_with_gyro(imu_thread.get_heading(),(INITIAL_HEADING - 110) % 360))
            motor.brake()
            drive_distance_with_gyro((INITIAL_HEADING - 110) % 360,23,200)
            motor.reverse(50)
            while get_angular_difference(INITIAL_HEADING-15, imu_thread.get_heading()) > 5:
                servo.set_angle(-steer_with_gyro(imu_thread.get_heading(), INITIAL_HEADING-15))
                time.sleep(0.01)
            motor.brake()
            drive_distance_with_gyro(INITIAL_HEADING-15,10,120,'reverse')
            servo.set_angle(0)
            log.info("Initial maneuver: green block reverse turn to 0° completed.")
            return
        elif detected_block_color == 'red':
            # motor.start_rpm_control(70, 'forward')
            # while get_angular_difference(ninety_degree_heading, imu_thread.get_heading()) > 10:
            #     servo.set_angle(steer_with_gyro(imu_thread.get_heading(),ninety_degree_heading))
            # action_taken = "REVERSE_BEFORE_TURN for 18cm"
            # drive_distance_with_gyro(drive_target_heading, 18, rpm=60, direction='reverse')
            drive_straight_with_gyro((INITIAL_HEADING-40)%360, 0.3, 70, 'forward')
            log.info("Initial maneuver: no block, short forward and return.")
            return
        else:
            # BUG: timed + direct duty -> drive_distance_with_gyro(..., <cm>, rpm=80).
            drive_straight_with_gyro((INITIAL_HEADING-55)%360, 0.2, 70, 'forward')
            log.info("Initial maneuver: no block, short forward and return.")
            return

    else:
        if detected_block_color == 'green':
            drive_straight_with_gyro((INITIAL_HEADING+30)%360, 0.5, 70, 'forward',2)
            log.info("Initial maneuver: no block, short forward and return.")
            return
        elif detected_block_color == 'red':
            motor.start_rpm_control(200, 'forward')
            while get_angular_difference(ninety_degree_heading, imu_thread.get_heading()) > 5:
                servo.set_angle(steer_with_gyro(imu_thread.get_heading(), ninety_degree_heading))
                time.sleep(0.01)
            motor.stop_rpm_control()
            motor.brake()
            action_taken = "DRIVE_FORWARD_RED_CW for 10cm"
            drive_distance_with_gyro(drive_target_heading, 17, rpm=200)
            motor.start_rpm_control(200, 'forward')
            while get_angular_difference(imu_thread.get_heading(), INITIAL_HEADING) > 5:
                servo.set_angle(steer_with_gyro(imu_thread.get_heading(),INITIAL_HEADING,2))
            motor.stop_rpm_control()
            servo.set_angle(0)
            return
        else:
            # BUG: timed + direct duty -> drive_distance_with_gyro(..., <cm>, rpm=80).
            drive_straight_with_gyro((INITIAL_HEADING+55)%360, 0.3, 70, 'forward')
            log.info("Initial maneuver: no block, short forward and return.")
            return

    motor.brake()
    log.info("Action complete: %s", action_taken)
    time.sleep(0.5)

    log.info("Performing final turn to return to %.1f°...", INITIAL_HEADING)
    # BUG: direct duty. Heading-terminated, so convert to
    # motor.start_rpm_control(50, "forward").
    motor.forward(50)

    while get_angular_difference(imu_thread.get_heading(), INITIAL_HEADING) > 15:
        servo.set_angle(steer_with_gyro(imu_thread.get_heading(),INITIAL_HEADING,2))
    motor.brake()
    servo.set_angle(0)
    # BUG: direct duty + stopwatch (0.3 s at duty 45). This straightens the robot up
    # before the main loop takes over, so an inconsistent distance here offsets the
    # entire first lap. Convert to
    #     control.drive_distance_with_gyro(INITIAL_HEADING, <cm>, rpm=45, direction='reverse')
    # Note the steering here is INVERTED (-steer_with_gyro) because the robot is
    # reversing; drive_distance_with_gyro does not do that for you -- pass the
    # heading you want to end up on and check the sign on the bench first.
    motor.reverse(45)
    start_time = time.monotonic()
    while time.monotonic() - start_time < 0.4:
        servo.set_angle(-steer_with_gyro(imu_thread.get_heading(),INITIAL_HEADING,1))
    time.sleep(0.5)
    motor.brake()
    log.info("--- Initial Maneuver Complete. Transitioning to straight driving. ---")

def parking():
    fn_start = time.monotonic()
    plog.info("--- Parking (clockwise) sequence started ---")
    drive_distance_with_gyro(INITIAL_HEADING,14,200)
    motor.brake()
    servo.set_angle_unlimited(-60)
    motor.stop_rpm_control()
    motor.start_rpm_control(80, "reverse")
    parking_start = time.monotonic()
    throttle = Throttle(0.1)
    while True:
        sensor_readings = sensor_thread.get_readings()
        heading = imu_thread.get_heading()
        if throttle:
            plog.debug("reverse: back=%s heading=%s", _fmt(sensor_readings['distance_back']), _fmt(heading))
        target_turn_heading = (INITIAL_HEADING + 90) % 360
        has_turned = get_angular_difference(target_turn_heading, heading) < 15
        if sensor_readings['distance_back'] is not None:
            if has_turned and sensor_readings['distance_back'] < 130:
                break
        if time.monotonic() - parking_start > 6.0:
            plog.warning("Parking reverse timeout reached!")
            break
        servo.set_angle_unlimited(-steer_with_gyro(heading,(INITIAL_HEADING+90)%360, kp=1, min_servo_angle=-60, max_servo_angle=60))
        time.sleep(0.01)

    motor.stop_rpm_control()
    time.sleep(0.3)
    motor.start_rpm_control(60, "forward")
    while get_angular_difference((INITIAL_HEADING+180)%360, imu_thread.get_heading()) > 5:
        heading = imu_thread.get_heading()
        servo.set_angle(steer_with_gyro(heading,(INITIAL_HEADING+180)%360, kp=1,min_servo_angle=-40, max_servo_angle=40))
        time.sleep(0.01)
    motor.stop_rpm_control()
    servo.set_angle(0)
    first_magenta_line_passed = False
    on_first_line = False

    MAGENTA_HIGH_THRESHOLD = 500
    MAGENTA_LOW_THRESHOLD = 200
    ROI_Y_START = 140
    ROI_X_START = 600
    TARGET_Y_OFFSET_FROM_BOTTOM = 180

    motor.stop_rpm_control()
    motor.start_rpm_control(100, "forward")
    past_frame_counter = 0
    plog.info("Tracking line to magenta stop...")
    # Step 4: this is the one part of parking that reads the camera in real time, so
    # it owns the frame-to-servo latency budget the way the main control loop did.
    # Freeze the background rebuild for the duration -- see BackgroundAnnotator.
    if bg_annotator is not None:
        bg_annotator.pause()
    while True:
        frame, frame_counter, _ = camera_thread.get_next_frame(past_frame_counter)
        if frame is None:
            plog.error("Failed to get frame, breaking loop.")
            break
        past_frame_counter = frame_counter

        frame_height, frame_width, _ = frame.shape
        target_y_global = frame_height - TARGET_Y_OFFSET_FROM_BOTTOM
        target_y_in_roi = target_y_global - ROI_Y_START

        roi = frame[ROI_Y_START:, ROI_X_START:]
        mask = cv2.inRange(cv2.cvtColor(roi, cv2.COLOR_BGR2HSV), HSV_RANGES['LOWER_BLACK'], np.array([180, 255, 40]))
        roi[mask == 255] = (255, 255, 255)
        y_coords = np.argmax(mask, axis=0)
        valid_y_coords = y_coords[mask[y_coords, np.arange(roi.shape[1])] > 0]

        steering_value = 0.0
        if valid_y_coords.size > 0:
            average_y = np.mean(valid_y_coords)
            error = target_y_in_roi - average_y
            steering_value = 0.8 * error
            cv2.line(frame, (ROI_X_START, ROI_Y_START + int(average_y)), (frame_width, ROI_Y_START + int(average_y)), (0, 0, 255), 2)

        servo.set_angle(steering_value)

        cv2.line(frame, (ROI_X_START, target_y_global), (frame_width, target_y_global), (0, 255, 0), 2)

        roi_stop = frame[330:360, 426:640]
        hsv_stop = cv2.cvtColor(roi_stop, cv2.COLOR_BGR2HSV)
        mask_magenta = cv2.inRange(hsv_stop, HSV_RANGES['LOWER_MAGENTA'], HSV_RANGES['UPPER_MAGENTA'])
        magenta_pixel_count = cv2.countNonZero(mask_magenta)

        cv2.rectangle(frame, (ROI_X_START, ROI_Y_START), (frame_width, frame_height), (0, 255, 0), 2)
        cv2.putText(frame, f"Servo angle: {steering_value:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.rectangle(frame, (426, 330), (640, 360), (255, 0, 255), 2)
        cv2.putText(frame, f"Magenta Pixels: {magenta_pixel_count}", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

        state_text = f"Armed to Stop: {first_magenta_line_passed}"
        cv2.putText(frame, state_text, (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

        _record_parking(frame)

        if first_magenta_line_passed:
            if magenta_pixel_count > MAGENTA_HIGH_THRESHOLD:
                plog.info("Second magenta line detected (%d px). Stopping.", magenta_pixel_count)
                break
        else:
            if on_first_line:
                if magenta_pixel_count < MAGENTA_LOW_THRESHOLD:
                    plog.info("First magenta line fully crossed. Armed to stop on the next one.")
                    first_magenta_line_passed = True
            else:
                if magenta_pixel_count > MAGENTA_HIGH_THRESHOLD:
                    plog.info("Detected what seems to be the first magenta line (%d px).", magenta_pixel_count)
                    on_first_line = True
    if bg_annotator is not None:
        bg_annotator.resume()
    servo.set_angle(0)
    time.sleep(0.47)  # INCREASING MAKES ROBOT STOP MORE FORWARD
    motor.stop_rpm_control()
    time.sleep(0.3)
    motor.start_rpm_control(60, "reverse")
    servo.set_angle_unlimited(55)
    while get_angular_difference((INITIAL_HEADING+100)%360, imu_thread.get_heading()) > 10:
        time.sleep(0.01)
    motor.stop_rpm_control()
    plog.debug("first reverse turn done: %s", sensor_thread.get_readings())
    motor.start_rpm_control(40, "reverse")
    servo.set_angle(0)
    parking_start = time.monotonic()
    throttle = Throttle(0.1)
    while True:
        dist = sensor_thread.get_readings()['distance_back']
        if throttle:
            plog.debug("reverse for parking: back=%s", _fmt(dist))
        if dist is not None and dist < 160:
            break
        if time.monotonic() - parking_start > 6.0:
            plog.warning("Parking second reverse timeout reached!")
            break
        time.sleep(0.01)
    motor.stop_rpm_control()
    time.sleep(0.3)
    motor.start_rpm_control(40, "forward")
    time.sleep(0.1)
    parking_start = time.monotonic()
    throttle = Throttle(0.1)
    while True:
        sensor_readings = sensor_thread.get_readings()
        dist = sensor_readings['distance_back']
        heading = imu_thread.get_heading()
        if throttle:
            plog.debug("forward for parking: back=%s heading=%s", _fmt(dist), _fmt(heading))
        if dist is not None and dist > 190:
            break
        if time.monotonic() - parking_start > 5.0:
            plog.warning("Parking forward drive timeout reached!")
            break
        servo.set_angle(steer_with_gyro(heading, (INITIAL_HEADING + 95) % 360, kp=1.0))
        time.sleep(0.01)
    motor.stop_rpm_control()
    time.sleep(0.3)
    servo.set_angle_unlimited(-65)
    motor.start_rpm_control(40, "reverse")
    manuver_start_time = time.monotonic()
    throttle = Throttle(0.1)
    while True:
        dist = sensor_thread.get_readings()['distance_back']
        if throttle:
            plog.debug("reverse 2 for parking: back=%s", _fmt(dist))
        if dist is not None:
            if dist <= 100:
                break
        if get_angular_difference((INITIAL_HEADING+170)%360, imu_thread.get_heading()) < 2:
            break
        if time.monotonic() - manuver_start_time > 6:
            break
        time.sleep(0.01)
    motor.stop_rpm_control()
    time.sleep(0.3)
    motor.start_rpm_control(40, "forward")
    throttle = Throttle(0.1)
    while True:
        dist_center = sensor_thread.get_readings()['distance_center']
        if throttle:
            plog.debug("forward alignment: center=%s", _fmt(dist_center))
        if dist_center is not None and dist_center < 135:
            break
        if get_angular_difference(imu_thread.get_heading(), (INITIAL_HEADING+180)%360) < 2:
            break
        servo.set_angle(steer_with_gyro(imu_thread.get_heading(),(INITIAL_HEADING+183)%360, kp=1.5))
        time.sleep(0.01)
    motor.stop_rpm_control()
    time.sleep(0.3)

    motor.start_rpm_control(40, "reverse")
    throttle = Throttle(0.1)
    while True:
        dist = sensor_thread.get_readings()['distance_back']
        if throttle:
            plog.debug("final reverse: back=%s", _fmt(dist))
        servo.set_angle(-steer_with_gyro(imu_thread.get_heading(),(INITIAL_HEADING+180)%360, kp=1.5))
        if dist is not None:
            if dist <= 95:
                break
        if get_angular_difference((INITIAL_HEADING+183)%360, imu_thread.get_heading()) < 2:
            break
        time.sleep(0.01)
    motor.stop_rpm_control()
    plog.info("--- parking() completed in %.2fs ---", time.monotonic() - fn_start)

def parking2():
    fn_start = time.monotonic()
    plog.info("--- Parking2 (counter-clockwise) sequence started ---")
    plog.debug("start center distance: %s", _fmt(sensor_thread.get_readings()['distance_center']))
    motor.stop_rpm_control()

    t_step = time.monotonic()
    plog.info("[Parking2 1/10] Forward approach (200 RPM)...")
    motor.start_rpm_control(200, "forward")
    throttle = Throttle(0.1)
    while True:
        sensor_readings = sensor_thread.get_readings()
        distance_center = sensor_readings.get('distance_center')
        heading = imu_thread.get_heading()
        target_heading = (INITIAL_HEADING + 5) % 360
        is_aligned = get_angular_difference(target_heading, heading) < 8
        if throttle:
            plog.debug("forward: center=%s heading=%s aligned=%s", _fmt(distance_center), _fmt(heading), is_aligned)
        if is_aligned and distance_center is not None and distance_center <= 220:
            plog.info("Distance is %.1f with heading aligned (%.1f vs %.1f). Exiting loop.", distance_center, heading, target_heading)
            break
        servo.set_angle(steer_with_gyro(heading, target_heading, kp=1))
        time.sleep(0.01)
    motor.stop_rpm_control()
    plog.info("[Parking2 1/10] Completed in %.2fs", time.monotonic() - t_step)
    time.sleep(0.3)

    t_step = time.monotonic()
    plog.info("[Parking2 2/10] Reverse 90 turn (200 RPM)...")
    servo.set_angle_unlimited(60)
    motor.start_rpm_control(120, "reverse")
    parking_start = time.monotonic()
    throttle = Throttle(0.1)
    while True:
        sensor_readings = sensor_thread.get_readings()
        heading = imu_thread.get_heading()
        if throttle:
            plog.debug("reverse: back=%s heading=%s", _fmt(sensor_readings['distance_back']), _fmt(heading))
        target_turn_heading = (INITIAL_HEADING - 80) % 360
        has_turned = get_angular_difference(target_turn_heading, heading) < 15
        if sensor_readings['distance_back'] is not None:
            if has_turned and sensor_readings['distance_back'] < 155:
                break
        if time.monotonic() - parking_start > 6.0:
            plog.warning("Parking2 reverse timeout reached!")
            break
        servo.set_angle_unlimited(-steer_with_gyro(heading, (INITIAL_HEADING-80)%360, kp=1, min_servo_angle=-60, max_servo_angle=60))
        time.sleep(0.01)

    motor.stop_rpm_control()
    plog.info("[Parking2 2/10] Completed in %.2fs", time.monotonic() - t_step)
    time.sleep(0.3)

    t_step = time.monotonic()
    plog.info("[Parking2 3/10] Realigning heading (150 RPM)...")
    motor.start_rpm_control(150, "forward")
    while get_angular_difference((INITIAL_HEADING-180)%360, imu_thread.get_heading()) > 7:
        heading = imu_thread.get_heading()
        servo.set_angle(steer_with_gyro(heading, (INITIAL_HEADING-180)%360, kp=1, min_servo_angle=-40, max_servo_angle=40))
        time.sleep(0.01)
    motor.stop_rpm_control()
    plog.info("[Parking2 3/10] Completed in %.2fs", time.monotonic() - t_step)
    servo.set_angle(0)

    ROI_Y_START = 140
    ROI_X_END = 40
    TARGET_Y_OFFSET_FROM_BOTTOM = 180

    MAGENTA_HIGH_THRESHOLD = 500
    MAGENTA_LOW_THRESHOLD = 200

    first_magenta_line_passed = False
    on_first_line = False

    t_step = time.monotonic()
    plog.info("[Parking2 4/10] Tracking line to magenta stop (200 RPM)...")
    motor.start_rpm_control(200, "forward")
    past_frame_counter = 0
    # Step 4: the one part of parking2 that reads the camera in real time. Freeze the
    # background rebuild for the duration -- see BackgroundAnnotator.
    if bg_annotator is not None:
        bg_annotator.pause()
    while True:
        frame, frame_counter, _ = camera_thread.get_next_frame(past_frame_counter)
        if frame is None:
            plog.error("Failed to get frame, breaking loop.")
            break
        past_frame_counter = frame_counter

        frame_height, frame_width, _ = frame.shape
        target_y_global = frame_height - TARGET_Y_OFFSET_FROM_BOTTOM
        target_y_in_roi = target_y_global - ROI_Y_START

        roi_black_line = frame[ROI_Y_START:, :ROI_X_END]
        mask_black = cv2.inRange(cv2.cvtColor(roi_black_line, cv2.COLOR_BGR2HSV), HSV_RANGES['LOWER_BLACK'], np.array([180, 255, 40]))
        y_coords = np.argmax(mask_black, axis=0)
        valid_y_coords = y_coords[mask_black[y_coords, np.arange(roi_black_line.shape[1])] > 0]

        steering_value = 0.0
        if valid_y_coords.size > 0:
            average_y = np.mean(valid_y_coords)
            error = average_y - target_y_in_roi
            steering_value = 0.8 * error
            cv2.line(frame, (0, ROI_Y_START + int(average_y)), (ROI_X_END, ROI_Y_START + int(average_y)), (0, 0, 255), 2)

        servo.set_angle(steering_value)

        cv2.line(frame, (0, target_y_global), (ROI_X_END, target_y_global), (0, 255, 0), 2)
        cv2.rectangle(frame, (0, ROI_Y_START), (ROI_X_END, frame_height), (0, 255, 0), 2)
        cv2.putText(frame, f"Servo angle: {steering_value:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

        roi_stop = frame[310:340, 0:214]
        hsv_stop = cv2.cvtColor(roi_stop, cv2.COLOR_BGR2HSV)
        mask_magenta = cv2.inRange(hsv_stop, HSV_RANGES['LOWER_MAGENTA'], HSV_RANGES['UPPER_MAGENTA'])
        magenta_pixel_count = cv2.countNonZero(mask_magenta)

        cv2.rectangle(frame, (0, 330), (214, 360), (255, 0, 255), 2)
        cv2.putText(frame, f"Magenta Pixels: {magenta_pixel_count}", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

        state_text = f"Armed to Stop: {first_magenta_line_passed}"
        cv2.putText(frame, state_text, (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

        if first_magenta_line_passed:
            if magenta_pixel_count > MAGENTA_HIGH_THRESHOLD:
                plog.info("Second magenta line detected (%d px). Stopping.", magenta_pixel_count)
                break
        else:
            if on_first_line:
                if magenta_pixel_count < MAGENTA_LOW_THRESHOLD:
                    plog.info("First magenta line fully crossed. Armed to stop on the next one.")
                    first_magenta_line_passed = True
            else:
                if magenta_pixel_count > MAGENTA_HIGH_THRESHOLD:
                    plog.info("Detected what seems to be the first magenta line (%d px).", magenta_pixel_count)
                    on_first_line = True

        _record_parking(frame)

    if bg_annotator is not None:
        bg_annotator.resume()
    servo.set_angle(1)
    time.sleep(0.22)
    motor.stop_rpm_control()
    plog.info("[Parking2 4/10] Completed in %.2fs", time.monotonic() - t_step)
    time.sleep(0.3)

    t_step = time.monotonic()
    plog.info("[Parking2 5/10] Reverse turn to 100 deg (70 RPM)...")
    motor.start_rpm_control(70, "reverse")
    servo.set_angle_unlimited(-55)
    while get_angular_difference((INITIAL_HEADING-100)%360, imu_thread.get_heading()) > 10:
        time.sleep(0.01)
    motor.stop_rpm_control()
    plog.info("[Parking2 5/10] Completed in %.2fs", time.monotonic() - t_step)

    t_step = time.monotonic()
    plog.info("[Parking2 6/10] Reverse to back distance < 160 (120 RPM)...")
    motor.start_rpm_control(120, "reverse")
    servo.set_angle(0)
    parking_start = time.monotonic()
    throttle = Throttle(0.1)
    while True:
        dist = sensor_thread.get_readings()['distance_back']
        if throttle:
            plog.debug("reverse for parking: back=%s", _fmt(dist))
        if dist is not None and dist < 160:
            break
        if time.monotonic() - parking_start > 6.0:
            plog.warning("Parking2 second reverse timeout reached!")
            break
        time.sleep(0.01)

    motor.stop_rpm_control()
    plog.info("[Parking2 6/10] Completed in %.2fs", time.monotonic() - t_step)
    time.sleep(0.3)

    t_step = time.monotonic()
    plog.info("[Parking2 7/10] Forward drive back distance > 220 (120 RPM)...")
    motor.start_rpm_control(120, "forward")
    time.sleep(0.1)
    parking_start = time.monotonic()
    throttle = Throttle(0.1)
    while True:
        sensor_readings = sensor_thread.get_readings()
        dist = sensor_readings['distance_back']
        heading = imu_thread.get_heading()
        if throttle:
            plog.debug("forward for parking: back=%s heading=%s", _fmt(dist), _fmt(heading))
        if dist is not None and dist > 160:
            break
        if time.monotonic() - parking_start > 5.0:
            plog.warning("Parking2 forward drive timeout reached!")
            break
        servo.set_angle(steer_with_gyro(heading, (INITIAL_HEADING - 85) % 360, kp=1.0))
        time.sleep(0.01)

    motor.stop_rpm_control()
    plog.info("[Parking2 7/10] Completed in %.2fs", time.monotonic() - t_step)
    time.sleep(0.3)

    t_step = time.monotonic()
    plog.info("[Parking2 8/10] Reverse 2 to back distance <= 120 (100 RPM)...")
    motor.start_rpm_control(40, "reverse")
    servo.set_angle_unlimited(65)
    manuver_start_time = time.monotonic()
    throttle = Throttle(0.1)
    while True:
        dist = sensor_thread.get_readings()['distance_back']
        if throttle:
            plog.debug("reverse 2 for parking: back=%s", _fmt(dist))
        if dist is not None:
            if dist <= 120:
                break
        if get_angular_difference((INITIAL_HEADING-180)%360, imu_thread.get_heading()) < 2:
            break
        if time.monotonic() - manuver_start_time > 6:
            break
        time.sleep(0.01)

    motor.stop_rpm_control()
    plog.info("[Parking2 8/10] Completed in %.2fs", time.monotonic() - t_step)
    time.sleep(0.3)

    t_step = time.monotonic()
    plog.info("[Parking2 9/10] Forward center alignment < 135 (60 RPM)...")
    motor.start_rpm_control(40, "forward")
    throttle = Throttle(0.05)
    while True:
        dist_center = sensor_thread.get_readings()['distance_center']
        if throttle:
            plog.debug("forward alignment: center=%s heading=%s", _fmt(dist_center), _fmt(imu_thread.get_heading()))
        if dist_center is not None and dist_center < 100:
            break
        if get_angular_difference(imu_thread.get_heading(), (INITIAL_HEADING-180)%360) < 2:
            break
        servo.set_angle(steer_with_gyro(imu_thread.get_heading(), (INITIAL_HEADING-180)%360, kp=1.5))
        time.sleep(0.01)

    motor.stop_rpm_control()
    plog.info("[Parking2 9/10] heading at exit: %s", _fmt(imu_thread.get_heading()))
    plog.info("[Parking2 9/10] Completed in %.2fs", time.monotonic() - t_step)
    time.sleep(0.3)

    t_step = time.monotonic()
    plog.info("[Parking2 10/10] Final reverse docking (60 RPM)...")
    motor.start_rpm_control(40, "reverse")
    throttle = Throttle(0.05)
    while True:
        dist = sensor_thread.get_readings()['distance_back']
        if throttle:
            plog.debug("final reverse: back=%s heading=%s", _fmt(dist), _fmt(imu_thread.get_heading()))
        servo.set_angle(-steer_with_gyro(imu_thread.get_heading(), (INITIAL_HEADING-180)%360, kp=1.5))
        if dist is not None:
            if dist <= 120:
                break
        if get_angular_difference((INITIAL_HEADING-180)%360, imu_thread.get_heading()) < 2:
            break
        time.sleep(0.01)
    motor.stop_rpm_control()
    plog.info("[Parking2 10/10] heading at exit: %s", _fmt(imu_thread.get_heading()))
    plog.info("[Parking2 10/10] Completed in %.2fs", time.monotonic() - t_step)
    plog.info("--- parking2() completed in %.2fs ---", time.monotonic() - fn_start)

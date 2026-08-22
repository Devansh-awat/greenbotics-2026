from collections import deque
import time
import threading
import queue
import cv2
import numpy as np
from gpiozero import Button, LED
import os
import sys
import traceback
from datetime import datetime

# Import all settings from the config file
import src.open_challenge.config as config

# Import hardware control modules
from src.sensors import bno086, camera, distance
from src.motors import motor, servo

# Color Definitions from obstacle_challenge tuning
from src.obstacle_challenge.tuning import (
    LOWER_ORANGE,
    UPPER_ORANGE,
    LOWER_BLUE,
    UPPER_BLUE,
)

# Line ROI Parameters from main_v2
LINE_ROI_X, LINE_ROI_Y, LINE_ROI_W, LINE_ROI_H = 280, 175, 80, 40

# Wall ROI Parameters from main_v2 PARKING (Left: x=0..40, Right: x=600..640, y=140..360)
LEFT_WALL_ROI_X, LEFT_WALL_ROI_Y, LEFT_WALL_ROI_W, LEFT_WALL_ROI_H = 0, 140, 40, 220
RIGHT_WALL_ROI_X, RIGHT_WALL_ROI_Y, RIGHT_WALL_ROI_W, RIGHT_WALL_ROI_H = 600, 140, 40, 220

# Black Wall HSV Upper Limit from main_v2 parking
UPPER_BLACK_WALL = np.array([180, 255, 40])

# Turn Counting Parameters
ORANGE_COOLDOWN_FRAMES = 50
ORANGE_DETECTION_HISTORY_LENGTH = 4

# Line & Wall Follow Parameters
SET_Y = 220
KP_WALL = 0.18
KD_WALL = 0.12
KP_GYRO = 1.2
LINE_MIN_AREA = 20


def get_angular_difference_signed(angle1, angle2):
    """Returns signed difference (angle1 - angle2) clamped to [-180, 180]."""
    if angle1 is None or angle2 is None:
        return 0.0
    diff = angle1 - angle2
    while diff <= -180:
        diff += 360
    while diff > 180:
        diff -= 360
    return diff


class CameraThread(threading.Thread):
    def __init__(self, camera_instance):
        super().__init__()
        self.camera = camera_instance
        self.latest_frame = None
        self.cond = threading.Condition()
        self.stop_event = threading.Event()
        self.daemon = True
        self.frame_counter = 0

    def run(self):
        while not self.stop_event.is_set():
            frame = self.camera.capture_frame()
            with self.cond:
                self.frame_counter += 1
                self.latest_frame = frame
                self.cond.notify_all()

    def get_frame(self):
        with self.cond:
            if self.latest_frame is not None:
                return self.latest_frame.copy(), self.frame_counter
            return None, self.frame_counter

    def get_next_frame(self, last_counter, timeout=1.0):
        with self.cond:
            self.cond.wait_for(
                lambda: self.frame_counter != last_counter or self.stop_event.is_set(),
                timeout=timeout,
            )
            if self.latest_frame is not None:
                return self.latest_frame.copy(), self.frame_counter
            return None, self.frame_counter

    def stop(self):
        with self.cond:
            self.stop_event.set()
            self.cond.notify_all()


class ImuThread(threading.Thread):
    def __init__(self, bno, init_event):
        super().__init__()
        self.bno = bno
        self.initialization_complete = init_event
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.daemon = True
        self.heading = None

    def run(self):
        try:
            self.bno.initialize()
            print("ImuThread: IMU initialized.")
            self.initialization_complete.set()
            while not self.stop_event.is_set():
                heading = self.bno.get_heading()
                if heading is not None:
                    with self.lock:
                        self.heading = heading
                time.sleep(0.01)
        except Exception as e:
            print(f"ImuThread: ERROR during initialization/operation: {e}")
            traceback.print_exc()
            self.initialization_complete.set()
        finally:
            self.bno.cleanup()
            print("ImuThread: IMU cleanup complete.")

    def get_heading(self):
        with self.lock:
            return self.heading

    def stop(self):
        self.stop_event.set()


class VideoWriterThread(threading.Thread):
    def __init__(self, path, fourcc, fps, frame_size):
        super().__init__()
        self.out = cv2.VideoWriter(path, fourcc, fps, frame_size)
        self.queue = queue.Queue()
        self.stop_event = threading.Event()
        self.daemon = True

    def run(self):
        while not self.stop_event.is_set() or not self.queue.empty():
            try:
                frame = self.queue.get(timeout=0.1)
                self.out.write(frame)
                self.queue.task_done()
            except queue.Empty:
                continue
            except:
                print("VideoWriterThread: ERROR writing frame")
                traceback.print_exc()
                continue
        self.out.release()

    def write(self, frame):
        if not self.stop_event.is_set():
            self.queue.put(frame)

    def stop(self):
        self.stop_event.set()


def process_video_frame(frame, nav_state='SEARCH_LINE'):
    processed_data = {
        'detected_orange': False,
        'detected_blue': False,
        'orange_centroid': None,
        'blue_centroid': None,
        'lowest_y_left': None,
        'lowest_y_right': None,
    }

    blurred = cv2.GaussianBlur(frame, (3, 3), 0)
    hsv_frame = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # 1. Orange Line Detection in Line ROI (Always active for turn counting & start line detection)
    line_crop = hsv_frame[LINE_ROI_Y:LINE_ROI_Y+LINE_ROI_H, LINE_ROI_X:LINE_ROI_X+LINE_ROI_W]
    mask_orange = cv2.inRange(line_crop, LOWER_ORANGE, UPPER_ORANGE)

    contours_orange, _ = cv2.findContours(mask_orange, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours_orange:
        biggest_orange = max(contours_orange, key=cv2.contourArea)
        if cv2.contourArea(biggest_orange) > LINE_MIN_AREA:
            processed_data['detected_orange'] = True
            M = cv2.moments(biggest_orange)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"]) + LINE_ROI_X
                cy = int(M["m01"] / M["m00"]) + LINE_ROI_Y
                processed_data['orange_centroid'] = (cx, cy)

    # Blue Line Detection (Active during initial line search)
    if nav_state == 'SEARCH_LINE':
        mask_blue = cv2.inRange(line_crop, LOWER_BLUE, UPPER_BLUE)
        contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours_blue:
            biggest_blue = max(contours_blue, key=cv2.contourArea)
            if cv2.contourArea(biggest_blue) > LINE_MIN_AREA:
                processed_data['detected_blue'] = True
                M = cv2.moments(biggest_blue)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"]) + LINE_ROI_X
                    cy = int(M["m01"] / M["m00"]) + LINE_ROI_Y
                    processed_data['blue_centroid'] = (cx, cy)

    kernel = np.ones((3, 3), np.uint8)

    # 2. Black wall detection (ONLY process the chosen side's wall ROI)
    if nav_state == 'FOLLOW_LEFT' or nav_state == 'SEARCH_LINE':
        left_crop = hsv_frame[LEFT_WALL_ROI_Y:LEFT_WALL_ROI_Y+LEFT_WALL_ROI_H, LEFT_WALL_ROI_X:LEFT_WALL_ROI_X+LEFT_WALL_ROI_W]
        mask_black_left = cv2.inRange(left_crop, config.LOWER_BLACK, UPPER_BLACK_WALL)
        mask_black_left = cv2.morphologyEx(mask_black_left, cv2.MORPH_OPEN, kernel)

        has_black_left = np.any(mask_black_left > 0, axis=0)
        if np.any(has_black_left):
            flipped = mask_black_left[::-1, :]
            first_bottom_idx = np.argmax(flipped > 0, axis=0)
            lowest_y_roi = (mask_black_left.shape[0] - 1) - first_bottom_idx
            valid_lowest_y_global = lowest_y_roi[has_black_left] + LEFT_WALL_ROI_Y
            processed_data['lowest_y_left'] = float(np.mean(valid_lowest_y_global))

    if nav_state == 'FOLLOW_RIGHT' or nav_state == 'SEARCH_LINE':
        right_crop = hsv_frame[RIGHT_WALL_ROI_Y:RIGHT_WALL_ROI_Y+RIGHT_WALL_ROI_H, RIGHT_WALL_ROI_X:RIGHT_WALL_ROI_X+RIGHT_WALL_ROI_W]
        mask_black_right = cv2.inRange(right_crop, config.LOWER_BLACK, UPPER_BLACK_WALL)
        mask_black_right = cv2.morphologyEx(mask_black_right, cv2.MORPH_OPEN, kernel)

        has_black_right = np.any(mask_black_right > 0, axis=0)
        if np.any(has_black_right):
            flipped = mask_black_right[::-1, :]
            first_bottom_idx = np.argmax(flipped > 0, axis=0)
            lowest_y_roi = (mask_black_right.shape[0] - 1) - first_bottom_idx
            valid_lowest_y_global = lowest_y_roi[has_black_right] + RIGHT_WALL_ROI_Y
            processed_data['lowest_y_right'] = float(np.mean(valid_lowest_y_global))

    return processed_data


def annotate_video_frame(frame, detections, state, angle, turn_counter=0, debug_info="", fps=0.0):
    annotated_frame = frame.copy()
    h, w, _ = frame.shape

    # Always draw Line ROI box
    cv2.rectangle(annotated_frame, (LINE_ROI_X, LINE_ROI_Y), (LINE_ROI_X + LINE_ROI_W, LINE_ROI_Y + LINE_ROI_H), (255, 255, 0), 2)

    # Draw active Wall ROI boxes
    if state == 'SEARCH_LINE' or state == 'FOLLOW_LEFT':
        cv2.rectangle(annotated_frame, (LEFT_WALL_ROI_X, LEFT_WALL_ROI_Y), (LEFT_WALL_ROI_X + LEFT_WALL_ROI_W, LEFT_WALL_ROI_Y + LEFT_WALL_ROI_H), (0, 165, 255), 2)
    if state == 'SEARCH_LINE' or state == 'FOLLOW_RIGHT':
        cv2.rectangle(annotated_frame, (RIGHT_WALL_ROI_X, RIGHT_WALL_ROI_Y), (RIGHT_WALL_ROI_X + RIGHT_WALL_ROI_W, RIGHT_WALL_ROI_H + RIGHT_WALL_ROI_Y), (0, 165, 255), 2)

    # Draw SET_Y target line
    cv2.line(annotated_frame, (0, SET_Y), (w, SET_Y), (255, 0, 255), 2)
    cv2.putText(annotated_frame, f"SET_Y={SET_Y}", (10, SET_Y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)

    # Highlight line centroids
    if detections['orange_centroid']:
        cv2.circle(annotated_frame, detections['orange_centroid'], 8, (0, 165, 255), -1)
        cv2.putText(annotated_frame, "ORANGE LINE", detections['orange_centroid'], cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 2)

    if detections['blue_centroid']:
        cv2.circle(annotated_frame, detections['blue_centroid'], 8, (255, 0, 0), -1)
        cv2.putText(annotated_frame, "BLUE LINE", detections['blue_centroid'], cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

    # Highlight wall points (bottom edge of black pixels)
    if detections['lowest_y_left'] is not None:
        y_l = int(detections['lowest_y_left'])
        cv2.line(annotated_frame, (LEFT_WALL_ROI_X, y_l), (LEFT_WALL_ROI_X + LEFT_WALL_ROI_W, y_l), (0, 255, 0), 2)
        cv2.circle(annotated_frame, (LEFT_WALL_ROI_X + LEFT_WALL_ROI_W // 2, y_l), 6, (0, 255, 0), -1)
        cv2.putText(annotated_frame, f"Left Y={y_l}", (10, y_l - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    if detections['lowest_y_right'] is not None:
        y_r = int(detections['lowest_y_right'])
        cv2.line(annotated_frame, (RIGHT_WALL_ROI_X, y_r), (RIGHT_WALL_ROI_X + RIGHT_WALL_ROI_W, y_r), (0, 255, 0), 2)
        cv2.circle(annotated_frame, (RIGHT_WALL_ROI_X + RIGHT_WALL_ROI_W // 2, y_r), 6, (0, 255, 0), -1)
        cv2.putText(annotated_frame, f"Right Y={y_r}", (RIGHT_WALL_ROI_X, y_r - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    # Display status text including frame-by-frame FPS
    cv2.putText(annotated_frame, f"STATE: {state} | TURNS: {turn_counter}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    cv2.putText(annotated_frame, f"ANGLE: {angle:.1f} | FPS: {fps:.1f}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    if debug_info:
        cv2.putText(annotated_frame, str(debug_info), (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    return annotated_frame


if __name__ == "__main__":
    camera.initialize()
    motor.initialize()
    servo.initialize()
    led = LED(config.LED_PIN)

    run_timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    base_folder = "open"
    run_folder = os.path.join(base_folder, run_timestamp)
    os.makedirs(run_folder, exist_ok=True)

    video_path = os.path.join(run_folder, 'open.mp4')
    log_path = os.path.join(run_folder, 'open_output.txt')

    log_file = open(log_path, 'w')
    sys.stdout = log_file
    sys.stderr = log_file

    fourcc = cv2.VideoWriter_fourcc(*'avc1')
    video_writer_thread = VideoWriterThread(video_path, fourcc, 30, (config.FRAME_WIDTH, config.FRAME_HEIGHT))
    video_writer_thread.start()

    prev_angle = 0
    past_frame_counter = 0
    prev_y_err = None

    # Frame-by-frame FPS tracking
    last_loop_time = time.monotonic()
    current_fps = 0.0

    orange_detection_history = deque([False] * ORANGE_DETECTION_HISTORY_LENGTH, maxlen=ORANGE_DETECTION_HISTORY_LENGTH)
    cooldown_frames = 0
    turn_counter = 0
    final_run_initiated = False
    final_run_start_time = None

    # Start camera and IMU threads
    imu_initialized_event = threading.Event()
    camera_thread = CameraThread(camera)
    camera_thread.start()
    imu_thread = ImuThread(bno086, imu_initialized_event)
    imu_thread.start()

    print("MainThread: Waiting for IMU to initialize...")
    imu_initialized_event.wait()
    print("MainThread: IMU is ready. Proceeding with main logic.")
    led.on()
    time.sleep(0.5)
    led.off()

    INITIAL_HEADING = None
    while INITIAL_HEADING is None:
        print("MainThread: Waiting for first valid heading reading...")
        heading = imu_thread.get_heading()
        if heading is not None:
            INITIAL_HEADING = heading
        time.sleep(0.05)
    print(f"MainThread: Initial heading locked: {INITIAL_HEADING}")

    # Navigation state: 'SEARCH_LINE', 'FOLLOW_LEFT', 'FOLLOW_RIGHT'
    nav_state = 'SEARCH_LINE'

    try:
        run_start_time = time.monotonic()
        motor.forward(70)

        while True:
            frame, frame_counter = camera_thread.get_frame()
            if frame is None or frame_counter == past_frame_counter:
                continue
            past_frame_counter = frame_counter

            # --- Calculate Frame-by-Frame FPS ---
            now = time.monotonic()
            dt = now - last_loop_time
            last_loop_time = now
            if dt > 0:
                instant_fps = 1.0 / dt
                current_fps = 0.8 * current_fps + 0.2 * instant_fps if current_fps > 0 else instant_fps

            detections = process_video_frame(frame, nav_state)
            current_heading = imu_thread.get_heading()

            # --- Turn Counting Logic ---
            orange_detected_this_frame = bool(detections['detected_orange'])
            orange_detection_history.append(orange_detected_this_frame)

            if cooldown_frames > 0:
                cooldown_frames -= 1
            elif not orange_detection_history[-ORANGE_DETECTION_HISTORY_LENGTH] and all(list(orange_detection_history)[1:]):
                turn_counter += 1
                prev_y_err = None  # Reset derivative to 0 on each turn count
                cooldown_frames = ORANGE_COOLDOWN_FRAMES
                print("turn_counter ---------------->", turn_counter)

            # State transition logic
            if nav_state == 'SEARCH_LINE':
                if detections['detected_orange']:
                    nav_state = 'FOLLOW_RIGHT'
                    print("Detected ORANGE line -> Switched to FOLLOW_RIGHT wall mode")
                elif detections['detected_blue']:
                    nav_state = 'FOLLOW_LEFT'
                    print("Detected BLUE line -> Switched to FOLLOW_LEFT wall mode")

            # Control calculation based on state
            angle = 0.0
            debug_info = ""

            if nav_state == 'SEARCH_LINE':
                # Drive straight using IMU gyro heading
                heading_err = get_angular_difference_signed(current_heading, INITIAL_HEADING)
                angle = -KP_GYRO * heading_err
                debug_info = f"Gyro Error: {heading_err:.1f}"

            elif nav_state == 'FOLLOW_LEFT':
                # Follow left side wall with PD control
                y_val = detections['lowest_y_left']
                if y_val is None:
                    angle = -35.0  # Turn left 35 degrees if no black pixel seen on left wall
                    prev_y_err = None
                    debug_info = "Left Y: None -> Turn Left -35°"
                    print(f"[FOLLOW_LEFT] Wall Y: None | Steering Angle: {angle:.1f}°")
                else:
                    y_err = y_val - SET_Y
                    raw_deriv = 0.0 if prev_y_err is None else (y_err - prev_y_err)
                    deriv = float(np.clip(raw_deriv, -15.0, 15.0))
                    prev_y_err = y_err
                    angle = KP_WALL * y_err + KD_WALL * deriv
                    debug_info = f"Left Y: {y_val:.1f}, Err: {y_err:.1f}, Deriv: {deriv:.1f}"
                    print(f"[FOLLOW_LEFT] Wall Y: {y_val:.2f} | Err: {y_err:.2f} | Deriv: {deriv:.2f} | Steering Angle: {angle:.1f}°")

            elif nav_state == 'FOLLOW_RIGHT':
                # Follow right side wall with PD control
                y_val = detections['lowest_y_right']
                if y_val is None:
                    angle = 35.0  # Turn right 35 degrees if no black pixel seen on right wall
                    prev_y_err = None
                    debug_info = "Right Y: None -> Turn Right 35°"
                    print(f"[FOLLOW_RIGHT] Wall Y: None | Steering Angle: {angle:.1f}°")
                else:
                    y_err = y_val - SET_Y
                    raw_deriv = 0.0 if prev_y_err is None else (y_err - prev_y_err)
                    deriv = float(np.clip(raw_deriv, -15.0, 15.0))
                    prev_y_err = y_err
                    angle = -(KP_WALL * y_err + KD_WALL * deriv)
                    debug_info = f"Right Y: {y_val:.1f}, Err: {y_err:.1f}, Deriv: {deriv:.1f}"
                    print(f"[FOLLOW_RIGHT] Wall Y: {y_val:.2f} | Err: {y_err:.2f} | Deriv: {deriv:.2f} | Steering Angle: {angle:.1f}°")

            # Steering smoothing and limiting
            angle = np.clip(angle, prev_angle - 10, prev_angle + 10)
            angle = np.clip(angle, -40, 40)
            servo.set_angle(angle)
            prev_angle = angle

            # Annotate and record frame
            annotated_frame = annotate_video_frame(frame, detections, nav_state, angle, turn_counter=turn_counter, debug_info=debug_info, fps=current_fps)
            try:
                video_writer_thread.write(annotated_frame)
            except Exception as e:
                print(e)

            # --- Exit Conditions ---
            if turn_counter == 12 and not final_run_initiated and abs(get_angular_difference_signed(current_heading, INITIAL_HEADING)) < 30:
                print("12 turns reached. Stopping in 0.6 seconds")
                final_run_initiated = True
                final_run_start_time = time.monotonic()

            if final_run_initiated and (time.monotonic() - final_run_start_time) >= 0.3:
                print("0.3 second complete. Stopping.")
                break
            if turn_counter >= 13:
                motor.brake()
                break

    finally:
        run_end_time = time.monotonic()
        run_time = run_end_time - run_start_time
        print(f'Run completed in: {run_time:.2f} seconds.')
        # Cleanup
        motor.brake()
        print("MainThread: Signaling threads to stop...")
        camera_thread.stop()
        imu_thread.stop()
        video_writer_thread.stop()

        print("MainThread: Waiting for threads to complete...")
        camera_thread.join()
        imu_thread.join()
        video_writer_thread.join()
        print("MainThread: All threads have completed.")

        camera.cleanup()
        servo.set_angle(0)
        servo.cleanup()
        motor.cleanup()
        cv2.destroyAllWindows()
        print("Program finished.")
        if 'log_file' in locals() and not log_file.closed:
            print(f"Log file saved to {log_path}")
            log_file.close()

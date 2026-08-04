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

ORANGE_COOLDOWN_FRAMES = 50
ORANGE_DETECTION_HISTORY_LENGTH = 4


def get_angular_difference(angle1, angle2):
    if angle1 is None or angle2 is None:
        return 360
    diff = angle1 - angle2
    while diff <= -180:
        diff += 360
    while diff > 180:
        diff -= 360
    return abs(diff)


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


def process_video_frame(frame):
    """Port of obstacle_challenge/vision.py (v5), minus everything about pillars.

    Same working-slice structure, same masks and the same detection dicts the v5
    wall-following branch reads: the four wall ROI areas, the close-black band,
    the orange/blue line objects and `line_roi_wall_pct`.
    """
    processed_data = {
        'detected_walls': [],
        'detected_orange': [],
        'detected_blue': [],
        'detected_close_black': [],
        'line_roi_wall_pct': 0.0,
    }

    y0, y1 = config.GLOBAL_Y_OFFSET, config.GLOBAL_Y_END
    frame_slice = frame[y0:y1, :]
    if config.USE_BILATERAL:
        frame_slice = cv2.bilateralFilter(
            frame_slice, config.BILATERAL_D,
            config.BILATERAL_SIGMA_COLOR, config.BILATERAL_SIGMA_SPACE)
    else:
        frame_slice = cv2.GaussianBlur(frame_slice, (1, 7), 0)
    hsv_slice = cv2.cvtColor(frame_slice, cv2.COLOR_BGR2HSV)

    mask_black = cv2.inRange(hsv_slice, config.LOWER_BLACK, config.UPPER_BLACK)

    # Orange and blue are only ever read inside the line ROI, so threshold that
    # 80x40 patch alone rather than the whole slice.
    lx, ly, lw, lh = config.line_roi_x, config.line_roi_y, config.line_roi_w, config.line_roi_h
    ly_slice = ly - y0
    line_hsv = hsv_slice[ly_slice:ly_slice + lh, lx:lx + lw]
    mask_orange_line = cv2.inRange(line_hsv, config.LOWER_ORANGE, config.UPPER_ORANGE)
    mask_blue_line = cv2.inRange(line_hsv, config.LOWER_BLUE, config.UPPER_BLUE)

    # v5 subtracts the pillar/line colours from black before wall detection, so a
    # dark blue line can't be counted as wall. No pillars here, so blue is the
    # only subtraction that applies.
    global_blue_mask = np.zeros_like(mask_black)
    global_blue_mask[ly_slice:ly_slice + lh, lx:lx + lw] = mask_blue_line
    pure_black_mask = cv2.bitwise_and(mask_black, cv2.bitwise_not(global_blue_mask))

    roi_mask_walls_slice = config.roi_mask_walls[y0:y1, :]
    roi_mask_close_black_slice = config.roi_mask_close_black[y0:y1, :]
    final_mask_walls = cv2.bitwise_and(pure_black_mask, roi_mask_walls_slice)
    final_mask_close_black = cv2.bitwise_and(pure_black_mask, roi_mask_close_black_slice)

    def _line_object(mask, color):
        if cv2.countNonZero(mask) == 0:
            return None
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        biggest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(biggest_contour)
        if area <= config.ORANGE_MIN_AREA:
            return None
        M = cv2.moments(biggest_contour)
        if M["m00"] == 0:
            return None
        cx = int(M["m10"] / M["m00"]) + lx
        cy = int(M["m01"] / M["m00"]) + ly
        return {'color': color, 'area': area, 'centroid': (cx, cy),
                'contour': biggest_contour + [lx, ly]}

    orange_obj = _line_object(mask_orange_line, 'orange')
    if orange_obj:
        processed_data['detected_orange'].append(orange_obj)
    blue_obj = _line_object(mask_blue_line, 'blue')
    if blue_obj:
        processed_data['detected_blue'].append(blue_obj)

    # Detect "close black" walls for sharp turn avoidance
    if cv2.countNonZero(final_mask_close_black) > 0:
        contours, _ = cv2.findContours(final_mask_close_black, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > config.WALL_MIN_AREA:
                processed_data['detected_close_black'].append(
                    {'area': area, 'contour': contour + [0, y0]})

    # Detect side walls for steering
    wall_contours_by_roi = {job['type']: [] for job in config.WALL_JOBS}
    if cv2.countNonZero(final_mask_walls) > 0:
        contours, _ = cv2.findContours(final_mask_walls, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            if cv2.contourArea(c) > config.WALL_MIN_AREA:
                M = cv2.moments(c)
                if M["m00"] == 0: continue
                cx = int(M["m10"] / M["m00"])

                # Determine which ROI the contour belongs to
                job_type = 'unknown'
                for job in config.WALL_JOBS:
                    jx, _, jw, _ = job['roi']
                    if jx <= cx < jx + jw:
                        job_type = job['type']
                        break

                if job_type != 'unknown':
                    wall_contours_by_roi[job_type].append(c)

    # Find the biggest contour in each ROI and add it to the processed data
    for job_type, contour_list in wall_contours_by_roi.items():
        if contour_list:
            biggest_contour = max(contour_list, key=cv2.contourArea)
            area = cv2.contourArea(biggest_contour)
            processed_data['detected_walls'].append(
                {'type': job_type, 'area': area, 'contour': biggest_contour + [0, y0]})

    # How much of the line ROI is wall. Together with detected_close_black this is
    # what triggers the corner turn -- close_black sees a band across the front,
    # this sees the wall filling the patch we look for the floor line in.
    wall_line_crop = pure_black_mask[ly_slice:ly_slice + lh, lx:lx + lw]
    total_roi_pixels = lw * lh
    if total_roi_pixels > 0:
        processed_data['line_roi_wall_pct'] = (
            cv2.countNonZero(wall_line_crop) / total_roi_pixels) * 100.0

    return processed_data


def annotate_video_frame(frame, detections, debug_info=""):
    annotated_frame = frame.copy()
    light_blue = (255, 255, 0)

    # Define all ROIs for drawing
    all_rois = [
        config.left_side_job['roi'], config.right_side_job['roi'],
        config.inner_left_side_job['roi'], config.inner_right_side_job['roi'],
        (config.line_roi_x, config.line_roi_y, config.line_roi_w, config.line_roi_h),
        (config.close_x, config.close_y, config.close_w, config.close_h),
    ]
    # Draw ROIs on the frame
    for x, y, w, h in all_rois:
        cv2.rectangle(annotated_frame, (x, y), (x + w, y + h), light_blue, 2)

    # Draw detected walls
    for wall in detections['detected_walls']:
        cv2.drawContours(annotated_frame, [wall['contour']], -1, (255, 0, 0), 2)

    for black_obj in detections['detected_close_black']:
        cv2.drawContours(annotated_frame, [black_obj['contour']], -1, (0, 0, 0), 2)

    # Draw detected floor lines
    for orange_obj in detections['detected_orange']:
        cv2.drawContours(annotated_frame, [orange_obj['contour']], -1, (0, 165, 255), 2)
    for blue_obj in detections['detected_blue']:
        cv2.drawContours(annotated_frame, [blue_obj['contour']], -1, (255, 0, 0), 2)

    # Add debug text to the frame
    cv2.putText(annotated_frame, str(debug_info), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)

    return annotated_frame


if __name__ == "__main__":
    camera.initialize()
    motor.initialize()
    servo.initialize()
    # button = Button(config.BUTTON_PIN)
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

    orange_detection_history = deque([False] * ORANGE_DETECTION_HISTORY_LENGTH, maxlen=ORANGE_DETECTION_HISTORY_LENGTH)
    cooldown_frames = 0

    final_run_initiated = False
    final_run_start_time = None
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

    # Start camera and sensor threads
    imu_initialized_event = threading.Event()
    camera_thread = CameraThread(camera)
    camera_thread.start()
    imu_thread = ImuThread(bno086, imu_initialized_event)
    imu_thread.start()
    
    print("MainThread: Waiting for IMU to initialize...")
    imu_initialized_event.wait()
    print("MainThread: IMU is ready. Proceeding with main logic.")  
    led.on()
    # button.wait_for_press()
    led.off()
    time.sleep(0.5)
    INITIAL_HEADING = None
    while INITIAL_HEADING is None:
        print("MainThread: Waiting for first valid heading reading...")
        heading = imu_thread.get_heading()
        if heading is not None:
            INITIAL_HEADING = heading
        time.sleep(0.05)
    print(f"MainThread: Initial heading locked: {INITIAL_HEADING}")
    try:
        run_start_time = time.monotonic()
        motor.forward(70)

        while True:
            angle = 0
            debug = []
            # Block on the condition variable instead of busy-spinning on
            # get_frame() + continue -- the old spin burned a core polling and
            # starved the camera thread.
            frame, frame_counter = camera_thread.get_next_frame(past_frame_counter)
            if frame is None:
                continue
            past_frame_counter = frame_counter

            # Process frame to find walls and floor lines
            detections = process_video_frame(frame)

            detected_orange_object = detections['detected_orange']
            detected_blue_object = detections['detected_blue']

            # --- Driving direction (first line wins) ---
            if driving_direction is None:
                if detected_orange_object:
                    driving_direction = 'clockwise'
                    print("Orange line seen first -> driving_direction = clockwise")
                elif detected_blue_object:
                    driving_direction = 'counter-clockwise'
                    print("Blue line seen first -> driving_direction = counter-clockwise")

            # --- Turn Counting Logic ---
            orange_detected_this_frame = bool(detected_orange_object)
            orange_detection_history.append(orange_detected_this_frame)

            if cooldown_frames > 0:
                cooldown_frames -= 1
            elif not orange_detection_history[-ORANGE_DETECTION_HISTORY_LENGTH] and all(list(orange_detection_history)[1:]):
                turn_counter += 1
                cooldown_frames = ORANGE_COOLDOWN_FRAMES
                print("turn_counter ---------------->", turn_counter)

            # --- Steering Logic (ported from main_v5's wall-following branch) ---
            detected_walls = detections['detected_walls']
            left_pixel_size = sum(obj['area'] for obj in detected_walls if obj['type'] == 'wall_left')
            right_pixel_size = sum(obj['area'] for obj in detected_walls if obj['type'] == 'wall_right')
            wall_inner_left_size = sum(obj['area'] for obj in detected_walls if obj['type'] == 'wall_inner_left')
            wall_inner_right_size = sum(obj['area'] for obj in detected_walls if obj['type'] == 'wall_inner_right')

            # One side gone: push the side we CAN see hard, so the robot swings away
            # from the wall it is about to clip rather than drifting on a small error.
            if left_pixel_size < 700 and (right_pixel_size + wall_inner_right_size) > 100:
                right_pixel_size *= 2
                right_pixel_size += 25000
            elif right_pixel_size < 700 and (left_pixel_size + wall_inner_left_size) > 100:
                left_pixel_size *= 2
                left_pixel_size += 25000

            # PD on the left/right area imbalance, inner ROIs included. The +1 is a
            # small constant bias carried over from v5.
            wall_error = (left_pixel_size + wall_inner_left_size) - (right_pixel_size + wall_inner_right_size)
            wall_derivative = wall_error - prev_wall_error
            angle = (wall_error * config.WALL_KP) + (wall_derivative * config.WALL_KD) + 1
            prev_wall_error = wall_error

            # Corner: a black band right in front, or the line ROI filled with wall.
            # ADDS to the PD angle rather than overriding it, so the wall error still
            # shapes how tight the turn is.
            if driving_direction == 'counter-clockwise':
                corner_turn = -config.CORNER_TURN_ANGLE
            elif driving_direction == 'clockwise':
                corner_turn = config.CORNER_TURN_ANGLE
            else:
                # Direction not established yet: turn away from the bigger wall.
                corner_turn = (-config.CORNER_TURN_ANGLE if left_pixel_size < right_pixel_size
                               else config.CORNER_TURN_ANGLE)

            close_black_area = sum(obj['area'] for obj in detections['detected_close_black'])
            if (close_black_area > config.CLOSE_BLACK_AREA_THRESHOLD
                    or detections['line_roi_wall_pct'] > config.LINE_ROI_WALL_PCT_THRESHOLD):
                angle += corner_turn
            # Both walls out of frame with a floor line under us: mid-corner, keep
            # turning. Stacks with the trigger above, as in v5.
            if left_pixel_size == 0 and right_pixel_size == 0 and (detected_orange_object or detected_blue_object):
                angle += corner_turn

            # --- ACTUATE FIRST ---
            # Steer immediately after deciding; annotation and recording come after.
            # v5 also drops the +/-10 per-frame slew limit the old loop had -- that
            # limit is a large part of why corners were entered late.
            angle = np.clip(angle, -40, 40)
            if angle != prev_angle:
                servo.set_angle(angle)
            prev_angle = angle

            # --- Annotation and Video Recording ---
            debug.extend([f"L:{int(left_pixel_size)}", f"R:{int(right_pixel_size)}"])
            debug.extend([f"IL:{int(wall_inner_left_size)}", f"IR:{int(wall_inner_right_size)}"])
            debug.append(f"Err:{int(wall_error)}")
            debug.append(f"Wall%:{int(detections['line_roi_wall_pct'])}")
            debug.append(f"Angle:{int(angle)}")
            debug.append(f"Turns:{turn_counter}")
            debug.append(f"Dir:{driving_direction or '?'}")
            annotated_frame = annotate_video_frame(frame, detections, debug_info=str(debug))
            try:
                video_writer_thread.write(annotated_frame)
            except Exception as e:
                print(e)
            
            # --- Exit Conditions ---
            if turn_counter == 12 and not final_run_initiated and get_angular_difference(imu_thread.get_heading(), INITIAL_HEADING) < 30:
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
        # --- Cleanup ---
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

from collections import deque
import random
import time
import queue
from src.sensors import camera, bno055, distance
from src.motors import motor, servo
from gpiozero import Button
import numpy as np
import cv2
import threading
import os
import sys
import logging
from datetime import datetime
import math

logger = logging.getLogger("obstacle_challenge")
logger.addHandler(logging.NullHandler())


class Throttle:
    """Rate-limits noisy per-iteration debug logging (tight loops poll every ~10ms)."""

    def __init__(self, interval=0.2):
        self.interval = interval
        self.last = 0.0

    def ready(self):
        now = time.monotonic()
        if now - self.last >= self.interval:
            self.last = now
            return True
        return False


MOTOR_SPEED = 60

FRAME_WIDTH = 640
FRAME_HEIGHT = 360
FRAME_MIDPOINT_X = FRAME_WIDTH // 2

USE_LAB = False

# HSV/LAB Color Ranges
HSV_RANGES = {
    'LOWER_RED_1': np.array([0, 98, 60]), 'UPPER_RED_1': np.array([4, 230, 166]),
    'LOWER_RED_2': np.array([174, 98, 60]), 'UPPER_RED_2': np.array([180, 230, 166]),
    'LOWER_GREEN': np.array([42, 85, 39]), 'UPPER_GREEN': np.array([88, 190, 135]),
    'LOWER_BLACK': np.array([0, 0, 0]), 'UPPER_BLACK': np.array([180, 60, 80]),
    'LOWER_ORANGE': np.array([6, 50, 182]), 'UPPER_ORANGE': np.array([15, 255, 255]),
    'LOWER_BLUE': np.array([114, 50, 110]), 'UPPER_BLUE': np.array([123, 255, 255]),
    'LOWER_MAGENTA': np.array([158, 73, 64]), 'UPPER_MAGENTA': np.array([172, 255, 223])
}

LAB_RANGES = {
    'LOWER_RED_1': np.array([35, 138, 131]), 'UPPER_RED_1': np.array([110, 176, 156]),
    'LOWER_RED_2': np.array([35, 138, 131]), 'UPPER_RED_2': np.array([110, 176, 156]),
    'LOWER_GREEN': np.array([44, 84, 124]), 'UPPER_GREEN': np.array([112, 124, 164]),
    'LOWER_BLACK': np.array([0, 115, 115]), 'UPPER_BLACK': np.array([90, 134, 134]),
    'LOWER_ORANGE': np.array([130, 129, 133]), 'UPPER_ORANGE': np.array([189, 167, 173]),
    'LOWER_BLUE': np.array([45, 129, 72]), 'UPPER_BLUE': np.array([153, 163, 119]),
    'LOWER_MAGENTA': np.array([72, 147, 48]), 'UPPER_MAGENTA': np.array([159, 174, 130])
}

if USE_LAB:
    COLOR_RANGES = LAB_RANGES
else:
    COLOR_RANGES = HSV_RANGES

LOWER_RED_1 = COLOR_RANGES['LOWER_RED_1']
UPPER_RED_1 = COLOR_RANGES['UPPER_RED_1']
LOWER_RED_2 = COLOR_RANGES['LOWER_RED_2']
UPPER_RED_2 = COLOR_RANGES['UPPER_RED_2']
LOWER_GREEN = COLOR_RANGES['LOWER_GREEN']
UPPER_GREEN = COLOR_RANGES['UPPER_GREEN']
LOWER_BLACK = COLOR_RANGES['LOWER_BLACK']
UPPER_BLACK = COLOR_RANGES['UPPER_BLACK']
LOWER_ORANGE = COLOR_RANGES['LOWER_ORANGE']
UPPER_ORANGE = COLOR_RANGES['UPPER_ORANGE']
LOWER_BLUE = COLOR_RANGES['LOWER_BLUE']
UPPER_BLUE = COLOR_RANGES['UPPER_BLUE']

WALL_MIN_AREA = 300
BLOCK_MIN_AREA = 500
CLOSE_BLOCK_MIN_AREA = 15
ORANGE_COOLDOWN_FRAMES = 100
ORANGE_DETECTION_HISTORY_LENGTH = 4

# ROIs shifted down by 15px
left_roi_x, left_roi_y, left_roi_w, left_roi_h = 0, 127, 135, 163
right_roi_x, right_roi_y, right_roi_w, right_roi_h = 505, 127, 135, 163
inner_left_roi_x, inner_left_roi_y, inner_left_roi_w, inner_left_roi_h = 140, 172, 100, 113
inner_right_roi_x, inner_right_roi_y, inner_right_roi_w, inner_right_roi_h = 400, 172 , 100, 113
full_frame_roi = (0, 65, 640, 195)
close_block_roi = (250, 210, 140, 10)
line_roi_x, line_roi_y, line_roi_w, line_roi_h = 280, 180, 80, 40

left_side_job = {'roi': (left_roi_x, left_roi_y, left_roi_w, left_roi_h), 'type': 'wall_left'}
right_side_job = {'roi': (right_roi_x, right_roi_y, right_roi_w, right_roi_h), 'type': 'wall_right'}
inner_left_side_job = {'roi': (inner_left_roi_x, inner_left_roi_y, inner_left_roi_w, inner_left_roi_h), 'type': 'wall_inner_left'}
inner_right_side_job = {'roi': (inner_right_roi_x, inner_right_roi_y, inner_right_roi_w, inner_right_roi_h), 'type': 'wall_inner_right'}

WALL_JOBS = [left_side_job, right_side_job, inner_left_side_job, inner_right_side_job]

class ImuThread(threading.Thread):
    def __init__(self, bno, init_event):
        super().__init__()
        self.name = "ImuThread"
        self.bno = bno
        self.initialization_complete = init_event
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.daemon = True
        self.heading = None

    def run(self):
        try:
            ok = self.bno.initialize()
            if ok:
                logger.info("IMU initialized.")
            else:
                logger.error("IMU init FAILED, heading unavailable.")
            self.initialization_complete.set()
            if not ok:
                return
            while not self.stop_event.is_set():
                heading = self.bno.get_heading()
                with self.lock:
                    self.heading = heading
                # BNO055 fusion output updates at 100 Hz; polling faster just burns CPU/GIL
                time.sleep(0.005)
        except Exception:
            logger.exception("Error during initialization/operation")
            self.initialization_complete.set()
        finally:
            self.bno.cleanup()
            logger.info("IMU cleanup complete.")

    def get_heading(self):
        with self.lock:
            return self.heading

    def stop(self):
        self.stop_event.set()

class SensorThread(threading.Thread):
    def __init__(self, dist, init_event):
        super().__init__()
        self.name = "SensorThread"
        self.dist = dist
        self.initialization_complete = init_event
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.daemon = True
        self.distance_left = None
        self.distance_right = None
        self.distance_back = None
        self.distance_center = None

    def run(self):
        try:
            for attempt in range(3):
                try:
                    logger.info("Initializing distance sensors...")
                    self.dist.initialise()
                    logger.info("Both sensors initialized.")
                    break
                except Exception:
                    logger.exception(f"Error during initialization (attempt {attempt + 1}/3)")
                time.sleep(0.3)
            time.sleep(0.3)
            logger.info("Initialization complete flag set.")
            self.initialization_complete.set()

            consecutive_none = {ch: 0 for ch in [0, 3]}
            reinit_threshold = 15
            reinit_cooldown = 3.0
            last_reinit = {ch: 0.0 for ch in [0, 3]}
            last_diag = time.monotonic()
            while not self.stop_event.is_set():
                try:
                    readings = {}
                    for ch in list(consecutive_none.keys()):
                        val = self.dist.get_distance(ch)
                        readings[ch] = val
                        if val is None:
                            consecutive_none[ch] = consecutive_none.get(ch, 0) + 1
                        else:
                            consecutive_none[ch] = 0

                    with self.lock:
                        self.distance_left = None
                        self.distance_center = readings.get(0)
                        self.distance_right = None
                        self.distance_back = readings.get(3)

                    now_mono = time.monotonic()
                    for ch, count in list(consecutive_none.items()):
                        if (count >= reinit_threshold
                                and now_mono - last_reinit[ch] >= reinit_cooldown):
                            logger.warning(f"Channel {ch} returned None {count}x, reinitializing...")
                            ok = self.dist.reinit_sensor(ch)
                            (logger.info if ok else logger.warning)(f"Reinit channel {ch} -> {ok}")
                            last_reinit[ch] = now_mono
                            consecutive_none[ch] = 0

                    if time.monotonic() - last_diag >= 2.0:
                        try:
                            logger.debug(f"Distance diag: {self.dist.get_diag()}")
                        except Exception:
                            pass
                        last_diag = time.monotonic()

                    time.sleep(1 / 30)
                except Exception:
                    logger.exception("Error during sensor reading")
                    time.sleep(0.1)
        except Exception:
            logger.exception("Error during initialization/operation")
            self.initialization_complete.set()
        finally:
            logger.info("Cleaning up distance sensors...")
            self.dist.cleanup()
            logger.info("Distance sensor cleanup complete.")

    def get_readings(self):
        with self.lock:
            return {
                'distance_left': self.distance_left,
                'distance_center': self.distance_center,
                'distance_right': self.distance_right,
                'distance_back' : self.distance_back
            }

    def stop(self):
        self.stop_event.set()

class CameraThread(threading.Thread):
    def __init__(self, camera_instance):
        super().__init__()
        self.name = "CameraThread"
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
                return self.latest_frame, self.frame_counter
            return None

    def get_next_frame(self, last_counter, timeout=1.0):
        with self.cond:
            self.cond.wait_for(
                lambda: self.frame_counter != last_counter or self.stop_event.is_set(),
                timeout=timeout,
            )
            return self.latest_frame, self.frame_counter

    def stop(self):
        with self.cond:
            self.stop_event.set()
            self.cond.notify_all()

class VideoWriterThread(threading.Thread):
    def __init__(self, path, fourcc, fps, frame_size):
        super().__init__()
        self.name = "VideoWriterThread"
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
            except Exception:
                logger.exception("Error writing frame")
                continue
        self.out.release()

    def write(self, frame):
        if not self.stop_event.is_set():
            self.queue.put(frame)

    def stop(self):
        self.stop_event.set()

class AnnotateAndWriteThread(threading.Thread):
    def __init__(self, writer_thread):
        super().__init__()
        self.name = "AnnotateAndWriteThread"
        self.writer_thread = writer_thread
        self.queue = queue.Queue(maxsize=2)
        self.stop_event = threading.Event()
        self.daemon = True

    def run(self):
        while not self.stop_event.is_set() or not self.queue.empty():
            try:
                item = self.queue.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                frame, detections, driving_direction, debug_info, visual_target_x, visual_target_line = item
                annotated = annotate_video_frame(
                    frame, detections, driving_direction,
                    debug_info=debug_info,
                    visual_target_x=visual_target_x,
                    visual_target_line=visual_target_line,
                )
                self.writer_thread.write(annotated)
            except Exception:
                logger.exception("Error processing frame")

    def submit(self, frame, detections, driving_direction, debug_info="", visual_target_x=None, visual_target_line=None):
        if self.stop_event.is_set():
            return
        item = (frame, detections, driving_direction, debug_info, visual_target_x, visual_target_line)
        try:
            self.queue.put_nowait(item)
        except queue.Full:
            try:
                self.queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self.queue.put_nowait(item)
            except queue.Full:
                pass

    def stop(self):
        self.stop_event.set()

def process_video_frame(frame):
    processed_data = {
        'detected_blocks': [],
        'detected_walls': [],
        'detected_orange': [],
        'detected_blue': []
    }

    def converted_crop(x, y, w, h):
        crop = cv2.GaussianBlur(frame[y:y+h, x:x+w], (1, 7), 0)
        return cv2.cvtColor(crop, cv2.COLOR_BGR2Lab if USE_LAB else cv2.COLOR_BGR2HSV)

    # --- 1. Crop and Detect ---
    mx, my, mw, mh = full_frame_roi
    main_crop = converted_crop(mx, my, mw, mh)

    mask_red1_main = cv2.inRange(main_crop, LOWER_RED_1, UPPER_RED_1)
    if USE_LAB:
        mask_red_main = mask_red1_main
    else:
        mask_red2_main = cv2.inRange(main_crop, LOWER_RED_2, UPPER_RED_2)
        mask_red_main = cv2.bitwise_or(mask_red1_main, mask_red2_main)
    mask_green_main = cv2.inRange(main_crop, LOWER_GREEN, UPPER_GREEN)

    cx, cy, cw, ch = close_block_roi
    close_crop = converted_crop(cx, cy, cw, ch)

    mask_red1_close = cv2.inRange(close_crop, LOWER_RED_1, UPPER_RED_1)
    if USE_LAB:
        mask_red_close = mask_red1_close
    else:
        mask_red2_close = cv2.inRange(close_crop, LOWER_RED_2, UPPER_RED_2)
        mask_red_close = cv2.bitwise_or(mask_red1_close, mask_red2_close)

    mask_green_close = cv2.inRange(close_crop, LOWER_GREEN, UPPER_GREEN)

    lx, ly, lw, lh = line_roi_x, line_roi_y, line_roi_w, line_roi_h
    line_crop = converted_crop(lx, ly, lw, lh)
    mask_orange_line = cv2.inRange(line_crop, LOWER_ORANGE, UPPER_ORANGE)
    mask_blue_line = cv2.inRange(line_crop, LOWER_BLUE, UPPER_BLUE)

    # --- 2. Wall Detection (per wall ROI, blocks subtracted where their ROIs overlap) ---
    block_mask_regions = [
        (cv2.bitwise_or(mask_red_main, mask_green_main), (mx, my, mw, mh)),
        (cv2.bitwise_or(mask_red_close, mask_green_close), (cx, cy, cw, ch)),
    ]

    wall_masks = {}
    for job in WALL_JOBS:
        wx, wy, ww, wh = job['roi']
        wall_crop = converted_crop(wx, wy, ww, wh)
        mask_black = cv2.inRange(wall_crop, LOWER_BLACK, UPPER_BLACK)
        for block_mask, (bx, by, bw, bh) in block_mask_regions:
            ox1, oy1 = max(wx, bx), max(wy, by)
            ox2, oy2 = min(wx + ww, bx + bw), min(wy + wh, by + bh)
            if ox1 < ox2 and oy1 < oy2:
                wall_region = mask_black[oy1-wy:oy2-wy, ox1-wx:ox2-wx]
                block_region = block_mask[oy1-by:oy2-by, ox1-bx:ox2-bx]
                mask_black[oy1-wy:oy2-wy, ox1-wx:ox2-wx] = cv2.bitwise_and(
                    wall_region, cv2.bitwise_not(block_region))
        wall_masks[job['type']] = mask_black

    # --- 4. Contour Finding ---

    # Blocks (Red, Green - Main & Close)
    def process_block_contours(mask, offset_x, offset_y, b_type, b_color, min_area):
        blocks = []
        if cv2.countNonZero(mask) > 0:
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in contours:
                area = cv2.contourArea(c)
                if area > min_area:
                    M = cv2.moments(c)
                    if M["m00"] != 0:
                        c_x = int(M["m10"] / M["m00"]) + offset_x
                        c_y = int(M["m01"] / M["m00"]) + offset_y
                        c_global = c + [offset_x, offset_y]
                        blocks.append({'type': b_type, 'color': b_color, 'area': area, 'centroid': (c_x, c_y), 'contour': c_global})
        return blocks

    all_detected_blocks = []
    all_detected_blocks.extend(process_block_contours(mask_red_main, mx, my, 'block', 'red', BLOCK_MIN_AREA))
    all_detected_blocks.extend(process_block_contours(mask_green_main, mx, my, 'block', 'green', BLOCK_MIN_AREA))

    all_detected_blocks.extend(process_block_contours(mask_red_close, cx, cy, 'close_block', 'red', CLOSE_BLOCK_MIN_AREA))
    all_detected_blocks.extend(process_block_contours(mask_green_close, cx, cy, 'close_block', 'green', CLOSE_BLOCK_MIN_AREA))

    main_blocks = [b for b in all_detected_blocks if b['type'] == 'block']
    other_blocks = [b for b in all_detected_blocks if b['type'] != 'block']
    main_blocks.sort(key=lambda b: b['centroid'][1], reverse=True)
    processed_data['detected_blocks'] = main_blocks + other_blocks
    
    # Orange (Line)
    if cv2.countNonZero(mask_orange_line) > 0:
        contours, _ = cv2.findContours(mask_orange_line, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            biggest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(biggest_contour)
            if area > 20:
                M = cv2.moments(biggest_contour)
                if M["m00"] != 0:
                    cx_val = int(M["m10"] / M["m00"]) + lx
                    cy_val = int(M["m01"] / M["m00"]) + ly
                    biggest_contour_global = biggest_contour + [lx, ly]
                    processed_data['detected_orange'].append({'type': 'orange_block', 'color': 'orange', 'area': area, 'centroid': (cx_val, cy_val), 'contour': biggest_contour_global})

    # Blue (Line)
    if cv2.countNonZero(mask_blue_line) > 0:
        contours, _ = cv2.findContours(mask_blue_line, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            biggest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(biggest_contour)
            if area > 20:
                M = cv2.moments(biggest_contour)
                if M["m00"] != 0:
                    cx_val = int(M["m10"] / M["m00"]) + lx
                    cy_val = int(M["m01"] / M["m00"]) + ly
                    biggest_contour_global = biggest_contour + [lx, ly]
                    processed_data['detected_blue'].append({'type': 'blue_block', 'color': 'blue', 'area': area, 'centroid': (cx_val, cy_val), 'contour': biggest_contour_global})

    # Walls: biggest qualifying contour per wall ROI
    for job in WALL_JOBS:
        wx, wy, ww, wh = job['roi']
        mask_black = wall_masks[job['type']]
        if cv2.countNonZero(mask_black) == 0:
            continue
        contours, _ = cv2.findContours(mask_black, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        biggest_contour, biggest_area = None, 0
        for c in contours:
            area = cv2.contourArea(c)
            if area > WALL_MIN_AREA and area > biggest_area:
                biggest_contour, biggest_area = c, area
        if biggest_contour is not None:
            M = cv2.moments(biggest_contour)
            if M["m00"] != 0:
                c_x = int(M["m10"] / M["m00"]) + wx
                c_y = int(M["m01"] / M["m00"]) + wy
                biggest_contour_global = biggest_contour + [wx, wy]
                processed_data['detected_walls'].append({'type': job['type'], 'color': 'black', 'area': biggest_area, 'centroid': (c_x, c_y), 'contour': biggest_contour_global})

    return processed_data

def annotate_video_frame(frame, detections, driving_direction, debug_info="", visual_target_x=None, visual_target_line=None):
    annotated_frame = frame
    light_blue = (255, 255, 0)
    target_line_color = (255, 0, 255)

    all_rois = [
        (left_roi_x, left_roi_y, left_roi_w, left_roi_h),
        (right_roi_x, right_roi_y, right_roi_w, right_roi_h),
        (inner_left_roi_x, inner_left_roi_y, inner_left_roi_w, inner_left_roi_h),
        (inner_right_roi_x, inner_right_roi_y, inner_right_roi_w, inner_right_roi_h),
        full_frame_roi,
        close_block_roi,
        (line_roi_x, line_roi_y, line_roi_w, line_roi_h)
    ]
    for x, y, w, h in all_rois:
        cv2.rectangle(annotated_frame, (x, y), (x + w, y + h), light_blue, 2)

    for wall in detections['detected_walls']:
        cv2.drawContours(annotated_frame, [wall['contour']], -1, (0, 0, 0), 2)

    for block in detections['detected_blocks']:
        draw_color = (255, 255, 255)
        if block['color'] == 'red':
            draw_color = (0, 0, 255)
        elif block['color'] == 'green':
            draw_color = (0, 255, 0)
        cv2.drawContours(annotated_frame, [block['contour']], -1, draw_color, 2)

    for orange_obj in detections.get('detected_orange', []):
        cv2.drawContours(annotated_frame, [orange_obj['contour']], -1, (0, 165, 255), 2)

    for blue_obj in detections.get('detected_blue', []):
        cv2.drawContours(annotated_frame, [blue_obj['contour']], -1, (255, 0, 0), 2)

    if visual_target_x is not None:
        cv2.line(annotated_frame, (visual_target_x, 0), (visual_target_x, FRAME_HEIGHT), target_line_color, 2)

    if visual_target_line is not None:
        pt1, pt2, ideal_angle = visual_target_line[:3]
        cv2.line(annotated_frame, pt1, pt2, (0, 255, 255), 2)
        
        if len(visual_target_line) == 4:
            pt3 = visual_target_line[3]
            cv2.line(annotated_frame, pt1, pt3, (0, 255, 255), 3)
        else:
            origin_x, origin_y = pt1[0], pt1[1]
            target_len = 200
            target_pt_x = int(origin_x + target_len * math.sin(math.radians(ideal_angle)))
            target_pt_y = int(origin_y - target_len * math.cos(math.radians(ideal_angle)))
            cv2.line(annotated_frame, (origin_x, origin_y), (target_pt_x, target_pt_y), (0, 255, 255), 3)

    cv2.putText(annotated_frame, str(debug_info), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
    
    return annotated_frame

def get_angular_difference(angle1, angle2):
    if angle1 is None or angle2 is None:
        return 360
    diff = angle1 - angle2
    while diff <= -180:
        diff += 360
    while diff > 180:
        diff -= 360
    return abs(diff)

def steer_with_gyro(current_heading: float, 
                    target_heading: float, 
                    kp: float = 0.85, 
                    min_servo_angle: int = -45, 
                    max_servo_angle: int = 45) -> float:
    error = target_heading - current_heading
    if error > 180:
        error -= 360
    elif error < -180:
        error += 360
    steer_angle = kp * error
    clamped_steer_angle = np.clip(steer_angle, min_servo_angle, max_servo_angle)
    return clamped_steer_angle

def drive_straight_with_gyro(target_heading, duration, speed, direction='forward'):
    logger.info(f"Driving {direction} with gyro stabilization for {duration}s...")
    KP = 0.85
    start_time = time.monotonic()
    if direction == 'forward':
        motor.forward(speed)
    else:
        motor.reverse(speed)

    while time.monotonic() - start_time < duration:
        current_heading = imu_thread.get_heading()
        if current_heading is None:
            time.sleep(0.01)
            continue
        error = target_heading - current_heading
        while error <= -180: error += 360
        while error > 180: error -= 360
        steer_angle = KP * error
        steer_angle = np.clip(steer_angle, -45, 45)
        servo.set_angle(steer_angle)
        time.sleep(0.01)

    motor.brake()
    servo.set_angle(0)
    logger.info("Gyro-stabilized drive complete.")

def perform_initial_maneuver():
    logger.info("--- Executing Full Initial Maneuver ---")
    SERVO_TURN_ANGLE = 40.0
    if driving_direction == "clockwise":
        initial_turn_servo = SERVO_TURN_ANGLE
        target_forward_heading = (INITIAL_HEADING + 55) % 360
        drive_duration = 0.2
    else:
        initial_turn_servo = -SERVO_TURN_ANGLE
        target_forward_heading = (INITIAL_HEADING - 55) % 360
        drive_duration = 0.2

    logger.info(f"Direction: {driving_direction.upper()}")
    logger.info(f"Initial Heading: {INITIAL_HEADING:.1f}°")
    logger.info(f"Forward Drive Target Heading: {target_forward_heading:.1f}°")

    motor.forward(80)
    servo.set_angle_unlimited(initial_turn_servo)
    logger.info("Starting initial turn (skipping scan)...")

    while get_angular_difference(target_forward_heading, imu_thread.get_heading()) > 15:
        time.sleep(0.01)

    logger.info("Driving forward with gyro stabilization...")
    drive_straight_with_gyro(target_forward_heading, drive_duration, 70, 'forward')

    logger.info(f"Performing final turn to return to {INITIAL_HEADING:.1f}°...")
    motor.forward(50)

    while get_angular_difference(imu_thread.get_heading(), INITIAL_HEADING) > 15:
        servo.set_angle(steer_with_gyro(imu_thread.get_heading(), INITIAL_HEADING, kp=2.0, min_servo_angle=-30, max_servo_angle=30))
        time.sleep(0.01)
        
    motor.brake()
    servo.set_angle(0)

    logger.info("Reversing slightly to align heading...")
    motor.reverse(50)
    start_time = time.monotonic()
    while time.monotonic() - start_time < 0.4:
        servo.set_angle(-steer_with_gyro(imu_thread.get_heading(), INITIAL_HEADING, kp=1.0))
        time.sleep(0.01)
    time.sleep(0.5)
    motor.brake()
    servo.set_angle(0)
    logger.info("--- Initial Maneuver Complete. Transitioning to straight driving. ---")

def parking():
    logger.info("--- Parking (clockwise) sequence started ---")
    servo.set_angle_unlimited(-60)
    motor.start_rpm_control(80, "reverse")
    _dbg = Throttle(0.2)
    while True:
        sensor_readings = sensor_thread.get_readings()
        heading = imu_thread.get_heading()
        if _dbg.ready():
            logger.debug(f"Reverse to wall: distance_back={sensor_readings['distance_back']} heading={heading} measured_rpm={motor.get_measured_rpm():.1f}")
        if sensor_readings['distance_back'] is not None and sensor_readings['distance_back'] < 70:
            break
        servo.set_angle_unlimited(-steer_with_gyro(heading,(INITIAL_HEADING+90)%360, kp=1, min_servo_angle=-60, max_servo_angle=60))
        time.sleep(0.01)

    motor.start_rpm_control(80, "forward")
    while get_angular_difference((INITIAL_HEADING+180)%360, imu_thread.get_heading()) > 5:
        heading = imu_thread.get_heading()
        servo.set_angle(steer_with_gyro(heading,(INITIAL_HEADING+180)%360, kp=1,min_servo_angle=-40, max_servo_angle=40))
    motor.stop_rpm_control()
    servo.set_angle(0)
    first_magenta_line_passed = False
    on_first_line = False

    MAGENTA_HIGH_THRESHOLD = 500
    MAGENTA_LOW_THRESHOLD = 200
    ROI_Y_START = 60
    ROI_X_START = 600
    TARGET_Y_OFFSET_FROM_BOTTOM = 230 

    motor.start_rpm_control(120, "forward")
    past_frame_counter = 0
    while True:
        frame, frame_counter = camera_thread.get_next_frame(past_frame_counter)
        if frame is None:
            logger.warning("Failed to get frame, breaking loop.")
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
        
        roi_stop = frame[310:340, 426:640]
        hsv_stop = cv2.cvtColor(roi_stop, cv2.COLOR_BGR2HSV)
        mask_magenta = cv2.inRange(hsv_stop, HSV_RANGES['LOWER_MAGENTA'], HSV_RANGES['UPPER_MAGENTA'])
        magenta_pixel_count = cv2.countNonZero(mask_magenta)

        cv2.rectangle(frame, (ROI_X_START, ROI_Y_START), (frame_width, frame_height), (0, 255, 0), 2)
        cv2.putText(frame, f"Servo angle: {steering_value:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.rectangle(frame, (426, 330), (640, 360), (255, 0, 255), 2)
        cv2.putText(frame, f"Magenta Pixels: {magenta_pixel_count}", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        state_text = f"Armed to Stop: {first_magenta_line_passed}"
        cv2.putText(frame, state_text, (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

        try:
            video_writer_thread.write(frame)
        except Exception:
            logger.exception("Error writing frame to video writer")

        if first_magenta_line_passed:
            if magenta_pixel_count > MAGENTA_HIGH_THRESHOLD:
                logger.info("Second magenta line detected. Stopping.")
                break
        else:
            if on_first_line:
                if magenta_pixel_count < MAGENTA_LOW_THRESHOLD:
                    logger.info("First magenta line fully crossed. Now armed to stop on the next one.")
                    first_magenta_line_passed = True
            else:
                if magenta_pixel_count > MAGENTA_HIGH_THRESHOLD:
                    logger.info("Detected what seems to be the first magenta line.")
                    on_first_line = True
    servo.set_angle(3)
    time.sleep(0.85)
    motor.stop_rpm_control()
    motor.start_rpm_control(80, "reverse")
    servo.set_angle_unlimited(55)
    while get_angular_difference((INITIAL_HEADING+100)%360, imu_thread.get_heading()) > 10:
            pass
    motor.stop_rpm_control()
    logger.info(f"Parking first reverse turn done: {sensor_thread.get_readings()}")
    motor.start_rpm_control(80, "reverse")
    servo.set_angle(0)
    logger.info("Reversing to back wall...")
    _dbg = Throttle(0.2)
    while True:
        dist = sensor_thread.get_readings()['distance_back']
        if _dbg.ready():
            logger.debug(f"Reverse for parking back distance: {dist} measured_rpm={motor.get_measured_rpm():.1f}")
        if dist is not None and dist < 170:
            break
        time.sleep(0.01)
    logger.info(f"Parking forward, readings: {sensor_thread.get_readings()}")
    motor.start_rpm_control(80, "forward")
    _dbg = Throttle(0.2)
    while True:
        sensor_readings = sensor_thread.get_readings()
        dist = sensor_readings['distance_back']
        heading = imu_thread.get_heading()
        if _dbg.ready():
            logger.debug(f"Forward for parking back distance: {dist} heading={heading} measured_rpm={motor.get_measured_rpm():.1f}")
        if dist is not None and dist > 170:
            break
        servo.set_angle(steer_with_gyro(heading, (INITIAL_HEADING + 90) % 360, kp=1.0))
        time.sleep(0.01)
    motor.stop_rpm_control()
    motor.start_rpm_control(80, "reverse")
    servo.set_angle_unlimited(-65)
    manuver_start_time = time.monotonic()
    while True:
        dist = sensor_thread.get_readings()['distance_back']
        if dist is not None:
            if dist <= 80:
                break
        if get_angular_difference((INITIAL_HEADING+180)%360, imu_thread.get_heading()) < 2:
            break
        if time.monotonic() - manuver_start_time > 6:
            break
    motor.stop_rpm_control()
    motor.start_rpm_control(80, "forward")
    while True:
        if sensor_thread.get_readings()['distance_center'] is not None and sensor_thread.get_readings()['distance_center'] < 75:
            break
        if get_angular_difference(imu_thread.get_heading(), (INITIAL_HEADING+180)%360) < 2:
            break
        servo.set_angle(steer_with_gyro(imu_thread.get_heading(),(INITIAL_HEADING+180)%360, kp=1.5))
        time.sleep(0.01)
    motor.stop_rpm_control()
    motor.start_rpm_control(60, "reverse")
    while True:
        dist = sensor_thread.get_readings()['distance_back']
        servo.set_angle(-steer_with_gyro(imu_thread.get_heading(),(INITIAL_HEADING+180)%360, kp=1.5))
        if dist is not None:
            if dist <= 75:
                break
        if get_angular_difference((INITIAL_HEADING+180)%360, imu_thread.get_heading()) < 2:
            break
    motor.stop_rpm_control()
    logger.info("--- Parking (clockwise) sequence complete ---")

def parking2():
    logger.info("--- Parking (counter-clockwise) sequence started ---")
    motor.forward(60)
    logger.info(f"Forward, distance_center={sensor_thread.get_readings()['distance_center']}")
    _dbg = Throttle(0.2)
    while True:
        sensor_readings = sensor_thread.get_readings()
        distance_center = sensor_readings.get('distance_center')
        if distance_center is not None and distance_center <= 200:
            logger.info(f"Distance is {distance_center}. Exiting loop.")
            break
        if _dbg.ready():
            logger.debug(f"distance_center={distance_center} heading={imu_thread.get_heading()}")
        servo.set_angle(steer_with_gyro(imu_thread.get_heading(), (INITIAL_HEADING+5) % 360, kp=1))
        time.sleep(0.01)
    motor.reverse(60)
    _dbg = Throttle(0.2)
    while True:
        sensor_readings = sensor_thread.get_readings()
        heading = imu_thread.get_heading()
        if _dbg.ready():
            logger.debug(f"distance_back={sensor_readings['distance_back']} heading={heading}")
        if sensor_readings['distance_back'] is not None and sensor_readings['distance_back'] < 160:
            break
        servo.set_angle_unlimited(-steer_with_gyro(heading,(INITIAL_HEADING-90)%360, kp=2, min_servo_angle=-60, max_servo_angle=60))
        time.sleep(0.01)
    motor.forward(60) 
    while get_angular_difference((INITIAL_HEADING-180)%360, imu_thread.get_heading()) > 5:
            heading = imu_thread.get_heading()
            servo.set_angle(steer_with_gyro(heading,(INITIAL_HEADING-180)%360, kp=1,min_servo_angle=-40, max_servo_angle=40))   
    ROI_Y_START = 140
    ROI_X_END = 40
    TARGET_Y_OFFSET_FROM_BOTTOM = 185

    MAGENTA_HIGH_THRESHOLD = 500
    MAGENTA_LOW_THRESHOLD = 200

    first_magenta_line_passed = False
    on_first_line = False

    past_frame_counter = 0
    while True:
        frame, frame_counter = camera_thread.get_next_frame(past_frame_counter)
        if frame is None:
            logger.warning("Failed to get frame, breaking loop.")
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
                logger.info("Second magenta line detected. Stopping.")
                break
        else:
            if on_first_line:
                if magenta_pixel_count < MAGENTA_LOW_THRESHOLD:
                    logger.info("First magenta line fully crossed. Now armed to stop on the next one.")
                    first_magenta_line_passed = True
            else:
                if magenta_pixel_count > MAGENTA_HIGH_THRESHOLD:
                    logger.info("Detected what seems to be the first magenta line.")
                    on_first_line = True

        try:
            video_writer_thread.write(frame)
        except Exception:
            logger.exception("Error writing frame to video writer")

    servo.set_angle(1)
    time.sleep(1.0)
    motor.brake()
    motor.reverse(60)
    servo.set_angle_unlimited(-60)
    while get_angular_difference((INITIAL_HEADING-100)%360, imu_thread.get_heading()) > 10:
            pass
    motor.brake()
    logger.info(f"Parking first reverse turn done: {sensor_thread.get_readings()}")
    motor.reverse(60)
    servo.set_angle(0)
    logger.info("Reversing to back wall...")
    _dbg = Throttle(0.2)
    while True:
        dist = sensor_thread.get_readings()['distance_back']
        if _dbg.ready():
            logger.debug(f"Reverse for parking back distance: {dist}")
        if dist is not None and dist < 180:
            break
        time.sleep(0.01)
    logger.info(f"Parking forward, readings: {sensor_thread.get_readings()}")
    motor.brake()
    motor.reverse(60)
    servo.set_angle_unlimited(65)
    manuver_start_time = time.monotonic()
    while True:
        dist = sensor_thread.get_readings()['distance_back']
        if dist is not None:
            if dist <= 90:
                break
        if get_angular_difference((INITIAL_HEADING+180)%360, imu_thread.get_heading()) < 2:
            break
        if time.monotonic() - manuver_start_time > 6:
            break
    motor.brake()
    motor.forward(60)
    while True:
        if sensor_thread.get_readings()['distance_center'] is not None and sensor_thread.get_readings()['distance_center'] < 75:
            break
        if get_angular_difference(imu_thread.get_heading(), (INITIAL_HEADING+180)%360) < 2:
            break
        servo.set_angle(steer_with_gyro(imu_thread.get_heading(),(INITIAL_HEADING+180)%360, kp=1.5))
        time.sleep(0.01)
    motor.brake()
    motor.reverse(60)
    while True:
        dist = sensor_thread.get_readings()['distance_back']
        servo.set_angle(-steer_with_gyro(imu_thread.get_heading(),(INITIAL_HEADING+180)%360, kp=1.5))
        if dist is not None:
            if dist <= 55:
                break
        if get_angular_difference((INITIAL_HEADING+180)%360, imu_thread.get_heading()) < 2:
            break
    motor.brake()
    logger.info("--- Parking (counter-clockwise) sequence complete ---")

if __name__ == "__main__":
    run_timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    base_folder = "obstacle"
    run_folder = os.path.join(base_folder, run_timestamp)    
    os.makedirs(run_folder, exist_ok=True)
    video_path = os.path.join(run_folder, 'obstacle.mp4')
    log_path = os.path.join(run_folder, 'obstacle_output.txt')
    log_file = open(log_path, 'w')
    sys.stdout = log_file
    sys.stderr = log_file

    # All of main_v4's own logging goes through `logger` -> formatted with a
    # timestamp/level/thread name and written into the same per-run log file
    # (via the redirected sys.stdout above, so stray print()s from other
    # modules like motor.py still land in the same file, just unformatted).
    log_handler = logging.StreamHandler(sys.stdout)
    log_handler.setFormatter(logging.Formatter(
        "%(asctime)s.%(msecs)03d [%(levelname)-7s] %(threadName)-20s %(message)s",
        datefmt="%H:%M:%S",
    ))
    logger.addHandler(log_handler)
    logger.setLevel(logging.DEBUG)

    fourcc = cv2.VideoWriter_fourcc(*'avc1')

    video_writer_thread = VideoWriterThread(video_path, fourcc, 20, (640, 360))
    video_writer_thread.start()
    annotate_and_write_thread = AnnotateAndWriteThread(video_writer_thread)
    annotate_and_write_thread.start()

    if not camera.initialize():
        logger.error("FATAL: Camera initialization failed. Exiting.")
        sys.exit(1)
    motor.initialize()
    servo.initialize()
    button = Button(23)

    prevangle = 0
    prevspeed = 0

    camera_thread = CameraThread(camera)
    camera_thread.start()

    sensors_initialized_event = threading.Event()
    sensor_thread = SensorThread(distance, sensors_initialized_event)
    sensor_thread.start()
    logger.info("Waiting for sensors to initialize...")
    sensors_initialized_event.wait()
    logger.info("Sensors are ready.")

    imu_initialized_event = threading.Event()
    imu_thread = ImuThread(bno055, imu_initialized_event)
    imu_thread.start()
    logger.info("Waiting for IMU to initialize...")
    imu_initialized_event.wait()
    logger.info("IMU is ready. Proceeding with main logic.")

    time.sleep(1)

    # Hardcode driving direction to clockwise
    driving_direction = 'clockwise'
    logger.info(f"Hardcoded driving direction: {driving_direction.upper()}")

    INITIAL_HEADING = None
    logger.info("Waiting for first valid heading reading...")
    while INITIAL_HEADING is None:
        heading = imu_thread.get_heading()
        if heading is not None:
            INITIAL_HEADING = heading
        time.sleep(0.05)
    logger.info(f"Initial heading locked: {INITIAL_HEADING}")

    # Initialize variables for lap (turn) counting
    orange_detection_history = deque([False] * ORANGE_DETECTION_HISTORY_LENGTH, maxlen=ORANGE_DETECTION_HISTORY_LENGTH)
    cooldown_frames = 0
    turn_counter = 0

    try:
        run_start_time = time.monotonic()
        last_turn_time = 0
        past_frame_counter = 0
        frame_counter = 0
        perform_initial_maneuver()
        motor.forward(MOTOR_SPEED)
        frame_start_time = time.perf_counter()
        logger.info(f"Starting run at {datetime.now().strftime('%H:%M:%S.%f')[:-3]}")
        while True:
            if button.is_pressed:
                logger.info("Run stopped by button.")
                break

            speed = MOTOR_SPEED
            angle = 0
            debug = []
            visual_target_x = None
            visual_target_line = None
            frame, frame_counter = camera_thread.get_next_frame(past_frame_counter)
            if frame is None:
                continue
            past_frame_counter = frame_counter

            detections = process_video_frame(frame)
            detected_blocks = detections['detected_blocks']
            detected_walls = detections['detected_walls']
            detected_orange = detections['detected_orange']
            detected_blue = detections['detected_blue']

            # Lap counting logic using orange line crossing detection
            orange_detected_this_frame = bool(detected_orange)
            orange_detection_history.append(orange_detected_this_frame)

            current_time = time.monotonic()
            if cooldown_frames > 0:
                cooldown_frames -= 1
            elif last_turn_time == 0 or (current_time - last_turn_time >= 1.5):
                if not orange_detection_history[-ORANGE_DETECTION_HISTORY_LENGTH] and all(list(orange_detection_history)[1:]):
                    turn_counter += 1
                    time_since_last_turn = current_time - last_turn_time if last_turn_time > 0 else 0.0
                    time_since_start = current_time - run_start_time
                    cooldown_frames = ORANGE_COOLDOWN_FRAMES
                    logger.info(f"Turn {turn_counter} detected | Since last turn: {time_since_last_turn:.2f}s | Total elapsed: {time_since_start:.2f}s")
                    last_turn_time = current_time

            # Stop after 13 turns (3 complete laps + start line crossing)
            if turn_counter >= 13:
                logger.info("Completed 13 turns (3 laps). Stopping.")
                if driving_direction == 'clockwise':
                    parking()
                else:
                    parking2()
                break

            if detected_blocks:
                is_close_block = False
                for block in detected_blocks:
                    if block['type'] == 'close_block':
                        is_close_block = True
                        if block['color'] == 'red':
                            angle = -25
                        elif block['color'] == 'green':
                            angle = 30
                        else:
                            is_close_block = False
                            break
                        servo.set_angle(angle)
                        motor.reverse(60)
                        time.sleep(0.5)
                        motor.forward(60)
                        servo.set_angle(-angle)
                        time.sleep(0.3)
                        motor.forward(MOTOR_SPEED)
                        break
                
                if not is_close_block:
                    candidate_blocks = [b for b in detected_blocks if b['type'] == 'block']
                    block = None
                    if candidate_blocks:
                        if candidate_blocks[0]['centroid'][1] >= 220 and len(candidate_blocks) > 1:
                            block = candidate_blocks[1]
                        else:
                            block = candidate_blocks[0]
                    
                    if block is not None:
                        block_color = block['color']
                        block_x, block_y = block['centroid']
                        debug.append((block_x, block_y))
                        
                        if block_color == 'red':
                            RED_OTHER_X = 240
                            RED_OTHER_Y = 0
                            RED_ORIGIN_X = 20
                            RED_ORIGIN_Y = FRAME_HEIGHT
                            RED_IDEAL_ANGLE = math.degrees(math.atan2(RED_OTHER_X - RED_ORIGIN_X, RED_ORIGIN_Y - RED_OTHER_Y))
                            
                            visual_target_line = ((RED_ORIGIN_X, RED_ORIGIN_Y), (block_x, block_y), RED_IDEAL_ANGLE, (RED_OTHER_X, RED_OTHER_Y))
                            current_angle = math.degrees(math.atan2(block_x - RED_ORIGIN_X, RED_ORIGIN_Y - block_y))
                            angle = (current_angle - RED_IDEAL_ANGLE) * 1.5
                            
                            wall_inner_right_size = sum(obj['area'] for obj in detected_walls if obj['type'] == 'wall_inner_right')
                            if wall_inner_right_size > 3000:
                                angle = np.clip(angle, -45, -10)
                            else:
                                angle = np.clip(angle, -45, 35)
                        
                        elif block_color == 'green':
                            GREEN_OTHER_X = 400
                            GREEN_OTHER_Y = 0
                            GREEN_ORIGIN_X = 620
                            GREEN_ORIGIN_Y = FRAME_HEIGHT
                            GREEN_IDEAL_ANGLE = math.degrees(math.atan2(GREEN_OTHER_X - GREEN_ORIGIN_X, GREEN_ORIGIN_Y - GREEN_OTHER_Y))
     
                            visual_target_line = ((GREEN_ORIGIN_X, GREEN_ORIGIN_Y), (block_x, block_y), GREEN_IDEAL_ANGLE, (GREEN_OTHER_X, GREEN_OTHER_Y))
                            current_angle = math.degrees(math.atan2(block_x - GREEN_ORIGIN_X, GREEN_ORIGIN_Y - block_y))
                            angle = (current_angle - GREEN_IDEAL_ANGLE) * 1.5
                            
                            wall_inner_left_size = sum(obj['area'] for obj in detected_walls if obj['type'] == 'wall_inner_left')
                            if wall_inner_left_size > 3000:
                                angle = np.clip(angle, 15, 45)
                            else:
                                angle = np.clip(angle, -45, 45)
                    else:
                        left_pixel_size = sum(obj['area'] for obj in detected_walls if obj['type'] == 'wall_left')
                        right_pixel_size = sum(obj['area'] for obj in detected_walls if obj['type'] == 'wall_right')
                        wall_inner_left_size = sum(obj['area'] for obj in detected_walls if obj['type'] == 'wall_inner_left')
                        wall_inner_right_size = sum(obj['area'] for obj in detected_walls if obj['type'] == 'wall_inner_right')
                        
                        if (detected_orange or detected_blue) and (
                            (driving_direction == 'clockwise' and right_pixel_size == 0) or
                            (driving_direction == 'counter-clockwise' and left_pixel_size == 0)
                        ):
                            if driving_direction == 'clockwise':
                                angle = 35
                            else:
                                angle = -35
                        else:
                            if driving_direction == 'clockwise':
                                if right_pixel_size < 1000 and (left_pixel_size + wall_inner_left_size) > 100:
                                    left_pixel_size *= 4
                                    left_pixel_size += 12500
                            elif driving_direction == 'counter-clockwise':
                                if left_pixel_size < 1000 and (right_pixel_size + wall_inner_right_size) > 100:
                                    right_pixel_size *= 4
                                    right_pixel_size += 12500
                            # Proportional (P) controller for wall following
                            angle = ((left_pixel_size + wall_inner_left_size) - (right_pixel_size + wall_inner_right_size)) * 0.001
        
            else:
                left_pixel_size = sum(obj['area'] for obj in detected_walls if obj['type'] == 'wall_left')
                right_pixel_size = sum(obj['area'] for obj in detected_walls if obj['type'] == 'wall_right')
                wall_inner_left_size = sum(obj['area'] for obj in detected_walls if obj['type'] == 'wall_inner_left')
                wall_inner_right_size = sum(obj['area'] for obj in detected_walls if obj['type'] == 'wall_inner_right')
                
                if (detected_orange or detected_blue) and (
                    (driving_direction == 'clockwise' and right_pixel_size == 0) or
                    (driving_direction == 'counter-clockwise' and left_pixel_size == 0)
                ):
                    if driving_direction == 'clockwise':
                        angle = 35
                    else:
                        angle = -35
                else:
                    if driving_direction == 'clockwise':
                        if right_pixel_size < 1000 and (left_pixel_size + wall_inner_left_size) > 100:
                            left_pixel_size *= 4
                            left_pixel_size += 12500
                    elif driving_direction == 'counter-clockwise':
                        if left_pixel_size < 1000 and (right_pixel_size + wall_inner_right_size) > 100:
                            right_pixel_size *= 4
                            right_pixel_size += 12500
                    # Proportional (P) controller for wall following
                    angle = ((left_pixel_size + wall_inner_left_size) - (right_pixel_size + wall_inner_right_size)) * 0.001

            debug.append(round(angle))
            debug.append(turn_counter)

            elapsed = time.perf_counter() - frame_start_time
            if elapsed < 1/60:
                time.sleep(1/60 - elapsed)
            frame_end_time = time.perf_counter()
            fps = 1/(frame_end_time - frame_start_time)
            frame_start_time = time.perf_counter()
            debug.append(round(fps))
            debug.append(frame_counter)
            
            try:
                annotate_and_write_thread.submit(
                    frame, detections, driving_direction,
                    debug_info=str(debug),
                    visual_target_x=visual_target_x,
                    visual_target_line=visual_target_line,
                )
            except Exception:
                logger.exception("Error submitting frame to annotate/write thread")

            angle = np.clip(angle, prevangle-6, prevangle+6)
            angle = np.clip(angle,-30,30)
            if angle != prevangle:
                servo.set_angle(angle)
            prevangle = angle
            if speed != prevspeed:
                motor.forward(speed)
            prevspeed = speed
            angle = 0

    except KeyboardInterrupt:
        logger.info("Run stopped by KeyboardInterrupt.")
    except Exception:
        logger.exception("Error during execution")

    finally:
        try:
            motor.stop_rpm_control()
            motor.brake()
            time.sleep(0.5)
            motor.cleanup()
        except BaseException as e:
            logger.warning(f"Motor priority shutdown error (ignored): {e}")

        try:
            run_end_time = time.monotonic()
            total_time = run_end_time - run_start_time
            logger.info(f"{'=' * 60}")
            logger.info(f"Run completed | Total time: {total_time:.2f}s | Total turns: {turn_counter}")
            logger.info(f"{'=' * 60}")
        except BaseException:
            pass

        try:
            servo.set_angle(0)
        except BaseException:
            pass

        try:
            time.sleep(0.5)
        except BaseException:
            pass

        logger.info("Signaling threads to stop...")
        try:
            camera_thread.stop()
        except BaseException:
            pass
        try:
            sensor_thread.stop()
        except BaseException:
            pass
        try:
            imu_thread.stop()
        except BaseException:
            pass
        try:
            annotate_and_write_thread.stop()
        except BaseException:
            pass

        logger.info("Waiting for threads to complete...")
        try:
            camera_thread.join(timeout=1.0)
        except BaseException:
            pass
        try:
            sensor_thread.join(timeout=1.0)
        except BaseException:
            pass
        try:
            imu_thread.join(timeout=1.0)
        except BaseException:
            pass
        try:
            annotate_and_write_thread.join(timeout=1.0)
        except BaseException:
            pass
        try:
            video_writer_thread.stop()
        except BaseException:
            pass
        try:
            video_writer_thread.join(timeout=1.0)
        except BaseException:
            pass

        try:
            video_writer_thread.out.release()
            logger.info("VideoWriter released successfully.")
        except BaseException as e:
            logger.warning(f"Error releasing VideoWriter: {e}")

        try:
            camera.cleanup()
        except BaseException:
            pass

        try:
            servo.set_angle(0)
            servo.cleanup()
        except BaseException:
            pass

        try:
            cv2.destroyAllWindows()
        except BaseException:
            pass

        if 'log_file' in locals() and not log_file.closed:
            try:
                logger.info(f"Log file saved to {log_path}")
                log_file.close()
            except BaseException:
                pass

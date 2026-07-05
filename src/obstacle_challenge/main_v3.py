from collections import deque
import random
import time
import queue
from src.sensors import bno055, camera, distance, encoder
from src.motors import motor, servo
from src.motors.motor import move as encoder_move
import numpy as np
import cv2
import threading
from gpiozero import Button, LED
import os
import sys
import traceback
from datetime import datetime
import math

# MOTOR_SPEED = 88
# ORANGE_COOLDOWN_FRAMES = 45

MOTOR_SPEED = 60
ORANGE_COOLDOWN_FRAMES = 20

#MOTOR_SPEED = 92
#ORANGE_COOLDOWN_FRAMES = 45

ORANGE_DETECTION_HISTORY_LENGTH = 4

WALL_THRESHOLD = 200

FRAME_WIDTH = 640
FRAME_HEIGHT = 360
FRAME_MIDPOINT_X = FRAME_WIDTH // 2

USE_LAB = False

# HSV values recomputed from saved samples at the 10-90 percentile (tighter than
# 2-98, to drop the dark/desaturated tail that drifts toward neutral -- exposure
# is kept low at 3ms to avoid motion blur). S floor clamped >=50 and S/V ceilings
# opened to 255 (brightness varies); hue bounds kept from the data.
HSV_RANGES = {
    'LOWER_RED_1': np.array([0, 92, 43]), 'UPPER_RED_1': np.array([4, 230, 166]),
    'LOWER_RED_2': np.array([173, 92, 43]), 'UPPER_RED_2': np.array([180, 230, 166]),
    'LOWER_GREEN': np.array([42, 85, 39]), 'UPPER_GREEN': np.array([88, 190, 135]),
    'LOWER_BLACK': np.array([0, 0, 0]), 'UPPER_BLACK': np.array([180, 95, 70]),
    'LOWER_ORANGE': np.array([6, 50, 182]), 'UPPER_ORANGE': np.array([15, 255, 255]),
    'LOWER_BLUE': np.array([114, 50, 110]), 'UPPER_BLUE': np.array([123, 255, 255]),
    'LOWER_MAGENTA': np.array([158, 73, 64]), 'UPPER_MAGENTA': np.array([172, 255, 223])
}

# LAB values from the polygon tuner (OpenCV LAB: L,a,b = 0-255).
# Red is a single LAB region, so RED_1 and RED_2 are duplicated to satisfy the
# two-range code path below. MAGENTA and BLACK were not re-annotated (magenta
# samples were lost to a tuner bug, now fixed) -- kept at their previous values.
LAB_RANGES = {
    # Chroma bounds are kept on the colour's own side of the neutral point
    # (a=128, b=128) so achromatic black/white/grey are rejected:
    #   RED/ORANGE: a floor > 128   GREEN: a ceiling < 128   BLUE: b ceiling < 128
    # RED/GREEN re-tuned from a fresh capture. Red a/b floors raised to 138/131
    # (margins 10/+3 from neutral) so dark/neutral black pixels stop matching red;
    # still ~98% red coverage. Red is one LAB region -> duplicated into RED_1/RED_2.
    'LOWER_RED_1': np.array([35, 138, 131]), 'UPPER_RED_1': np.array([110, 176, 156]),
    'LOWER_RED_2': np.array([35, 138, 131]), 'UPPER_RED_2': np.array([110, 176, 156]),
    'LOWER_GREEN': np.array([44, 84, 124]), 'UPPER_GREEN': np.array([112, 124, 164]),
    'LOWER_BLACK': np.array([0, 115, 115]), 'UPPER_BLACK': np.array([90, 134, 134]),
    'LOWER_ORANGE': np.array([130, 129, 133]), 'UPPER_ORANGE': np.array([189, 167, 173]),
    'LOWER_MAGENTA': np.array([72, 147, 48]), 'UPPER_MAGENTA': np.array([159, 174, 130]),
    'LOWER_BLUE': np.array([45, 129, 72]), 'UPPER_BLUE': np.array([153, 163, 119])
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
LOWER_MAGENTA = COLOR_RANGES['LOWER_MAGENTA']
UPPER_MAGENTA = COLOR_RANGES['UPPER_MAGENTA']
LOWER_BLUE = COLOR_RANGES['LOWER_BLUE']
UPPER_BLUE = COLOR_RANGES['UPPER_BLUE']
target=0
detection_params = {'min_area': 300, 'return_rule': 'biggest_in_job', 'return_mask': True}
WALL_MIN_AREA = detection_params['min_area']
BLOCK_MIN_AREA = 500
MAGENTA_MIN_AREA = 500
CLOSE_BLOCK_MIN_AREA = 15

left_roi_x, left_roi_y, left_roi_w, left_roi_h = 0, 127, 135, 163
right_roi_x, right_roi_y, right_roi_w, right_roi_h = 505, 127, 135, 163
inner_left_roi_x, inner_left_roi_y, inner_left_roi_w, inner_left_roi_h = 140, 172, 100, 113
inner_right_roi_x, inner_right_roi_y, inner_right_roi_w, inner_right_roi_h = 400, 172 , 100, 113
full_frame_roi = (0, 65, 640, 195)
close_block_roi = (250, 210, 140, 10)
line_roi_x, line_roi_y, line_roi_w, line_roi_h = 280, 180, 80, 40
close_x, close_y, close_w, close_h = 140, 130, 360, 10

left_side_job = {'roi': (left_roi_x, left_roi_y, left_roi_w, left_roi_h), 'type': 'wall_left'}
right_side_job = {'roi': (right_roi_x, right_roi_y, right_roi_w, right_roi_h), 'type': 'wall_right'}
inner_left_side_job = {'roi': (inner_left_roi_x, inner_left_roi_y, inner_left_roi_w, inner_left_roi_h), 'type': 'wall_inner_left'}
inner_right_side_job = {'roi': (inner_right_roi_x, inner_right_roi_y, inner_right_roi_w, inner_right_roi_h), 'type': 'wall_inner_right'}

roi_mask_walls = np.zeros((FRAME_HEIGHT, FRAME_WIDTH), dtype="uint8")
for job in [left_side_job, right_side_job, inner_left_side_job, inner_right_side_job]:
    x, y, w, h = job['roi']
    cv2.rectangle(roi_mask_walls, (x, y), (x + w, y + h), 255, -1)

roi_mask_main_blocks = np.zeros((FRAME_HEIGHT, FRAME_WIDTH), dtype="uint8")
x, y, w, h = full_frame_roi
cv2.rectangle(roi_mask_main_blocks, (x, y), (x + w, y + h), 255, -1)

roi_mask_close_blocks = np.zeros((FRAME_HEIGHT, FRAME_WIDTH), dtype="uint8")
x, y, w, h = close_block_roi
cv2.rectangle(roi_mask_close_blocks, (x, y), (x + w, y + h), 255, -1)

roi_mask_line = np.zeros((FRAME_HEIGHT, FRAME_WIDTH), dtype="uint8")
cv2.rectangle(roi_mask_line, (line_roi_x, line_roi_y), (line_roi_x + line_roi_w, line_roi_h + line_roi_y), 255, -1)

roi_mask_magenta = np.zeros((FRAME_HEIGHT, FRAME_WIDTH), dtype="uint8")
x, y, w, h = full_frame_roi
cv2.rectangle(roi_mask_magenta, (x, y), (x + w, y + h), 255, -1)

roi_mask_close_black = np.zeros((FRAME_HEIGHT, FRAME_WIDTH), dtype="uint8")
cv2.rectangle(roi_mask_close_black, (close_x, close_y), (close_x + close_w, close_y + close_h), 255, -1)

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
            ok = self.bno.initialize()
            print("ImuThread: IMU initialized." if ok else "ImuThread: IMU init FAILED, heading unavailable.")
            self.initialization_complete.set()
            if not ok:
                return
            while not self.stop_event.is_set():
                heading = self.bno.get_heading()
                with self.lock:
                    self.heading = heading
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
                print(f"VideoWriterThread: ERROR writing frame")
                traceback.print_exc()
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
                print("AnnotateAndWriteThread: ERROR processing frame")
                traceback.print_exc()

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

class DatasetCaptureThread(threading.Thread):
    """Captures raw unannotated frames every ~1 second for YOLO training data."""
    def __init__(self, camera_thread_ref, save_dir):
        super().__init__()
        self.camera_thread = camera_thread_ref
        self.save_dir = save_dir
        self.stop_event = threading.Event()
        self.daemon = True
        self.capture_count = 0
        self.run_prefix = datetime.now().strftime("%Y%m%d_%H%M%S")

    def run(self):
        while not self.stop_event.is_set():
            # Wait ~1 second with a small random jitter (±0.2s) for variety
            wait_time = 1.0 + random.uniform(-0.2, 0.2)
            if self.stop_event.wait(timeout=wait_time):
                break  # stop was requested during the wait

            result = self.camera_thread.get_frame()
            if result is None:
                continue
            frame, _ = result
            if frame is None:
                continue

            filename = os.path.join(self.save_dir, f"{self.run_prefix}_{self.capture_count:06d}.jpg")
            try:
                cv2.imwrite(filename, frame)
                self.capture_count += 1
            except Exception as e:
                print(f"DatasetCaptureThread: ERROR saving frame: {e}")

        print(f"DatasetCaptureThread: Saved {self.capture_count} frames to {self.save_dir}")

    def stop(self):
        self.stop_event.set()

class SensorThread(threading.Thread):
    def __init__(self, dist, init_event):
        super().__init__()
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
                    print("SensorThread: Initializing distance sensors...")
                    self.dist.initialise()
                    print("SensorThread: Both sensors initialized.")
                    break
                except Exception as e:
                    print(f"SensorThread: ERROR during initialization: {e}")
                    traceback.print_exc()
                time.sleep(0.3)
            time.sleep(0.3)
            print("SensorThread: Initialization complete flag set.")
            self.initialization_complete.set()

            # VL53L1X wiring: mux channel 0 = front, mux channel 3 = back
            consecutive_none = {ch: 0 for ch in [0, 3]}
            reinit_threshold = 15  # ~0.5s of no data before reinit attempt
            reinit_cooldown = 3.0  # min seconds between reinit attempts per ch
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
                        self.distance_center = readings.get(0)   # front
                        self.distance_right = None
                        self.distance_back = readings.get(3)     # back

                    now_mono = time.monotonic()
                    for ch, count in list(consecutive_none.items()):
                        if (count >= reinit_threshold
                                and now_mono - last_reinit[ch] >= reinit_cooldown):
                            # Reinit on a cooldown only. While the motor is
                            # driving, the back sensor NAKs the bus (Errno 121)
                            # and reinit can't succeed anyway; hammering it just
                            # starves the IMU. This recovers the channel once the
                            # electrical noise clears, without disrupting steering.
                            print(f"SensorThread: channel {ch} returned None "
                                  f"{count}x, reinitializing...")
                            ok = self.dist.reinit_sensor(ch)
                            print(f"SensorThread: reinit channel {ch} -> {ok}")
                            last_reinit[ch] = now_mono
                            consecutive_none[ch] = 0

                    # Every ~2s, dump why each channel returned None so we can
                    # tell 'not_ready' (sensor not framing) from 'no_target'
                    # (nothing in range) from 'err'/'absent'.
                    if time.monotonic() - last_diag >= 2.0:
                        try:
                            print(f"DIAG distance reasons: {self.dist.get_diag()}")
                        except Exception:
                            pass
                        last_diag = time.monotonic()

                    time.sleep(1 / 30)  # ~30 Hz, matches the 33 ms timing budget
                except Exception as e:
                    print(f"SensorThread: ERROR during sensor reading: {e}")
                    traceback.print_exc()
                    time.sleep(0.1)
        
        except Exception as e:
            print(f"SensorThread: ERROR during initialization/operation: {e}")
            traceback.print_exc()
            # Still set the event so main thread doesn't hang forever
            self.initialization_complete.set()
        
        finally:
            print("SensorThread: Cleaning up distance sensors...")
            self.dist.cleanup()
            print("SensorThread: Distance sensor cleanup complete.")

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

def get_angular_difference(angle1, angle2):
    if angle1 is None or angle2 is None:
        return 360
    diff = angle1 - angle2
    while diff <= -180:
        diff += 360
    while diff > 180:
        diff -= 360
    return abs(diff)

def process_video_frame(frame):
    processed_data = {
        'detected_blocks': [],
        'detected_walls': [],
        'detected_orange': [],
        'detected_blue' : [],
        'detected_magenta': [],
        'detected_close_black': []
    }
    
    # frame = cv2.GaussianBlur(frame,(1,7),0) # Commented out in original? No, it was active.
    # Global Slicing Constants
    GLOBAL_Y_OFFSET = 50
    GLOBAL_Y_END = 255
    SLICE_HEIGHT = GLOBAL_Y_END - GLOBAL_Y_OFFSET

    # Slice the frame first
    frame_slice = frame[GLOBAL_Y_OFFSET:GLOBAL_Y_END, :]
    frame_slice = cv2.GaussianBlur(frame_slice, (1, 7), 0)
    
    if USE_LAB:
        hsv_slice = cv2.cvtColor(frame_slice, cv2.COLOR_BGR2Lab)
    else:
        hsv_slice = cv2.cvtColor(frame_slice, cv2.COLOR_BGR2HSV)

    # --- 1. Crop and Detect (Relative to Slice) ---

    # Main Blocks ROI (Red, Green, Magenta)
    # full_frame_roi = (0, 100, 640, 165) -> y=100 in global is y=0 in slice
    mx, my, mw, mh = full_frame_roi
    my_slice = my - GLOBAL_Y_OFFSET
    # Ensure we don't go out of bounds
    my_slice = max(0, my_slice)
    
    main_crop = hsv_slice[my_slice:my_slice+mh, mx:mx+mw]
    
    mask_red1_main = cv2.inRange(main_crop, LOWER_RED_1, UPPER_RED_1)
    if USE_LAB:
        # In LAB, Red is continuous, so we only need one mask
        mask_red_main = mask_red1_main
    else:
        # In HSV, Red wraps around 180->0, so we need two masks combined
        mask_red2_main = cv2.inRange(main_crop, LOWER_RED_2, UPPER_RED_2)
        mask_red_main = cv2.bitwise_or(mask_red1_main, mask_red2_main)
    mask_green_main = cv2.inRange(main_crop, LOWER_GREEN, UPPER_GREEN)
    mask_magenta_main = cv2.inRange(main_crop, LOWER_MAGENTA, UPPER_MAGENTA)

    # Line ROI (Orange, Blue)
    # line_roi_y = 200 -> y=200 in global is y=100 in slice
    lx, ly, lw, lh = line_roi_x, line_roi_y, line_roi_w, line_roi_h
    ly_slice = ly - GLOBAL_Y_OFFSET
    
    line_crop = hsv_slice[ly_slice:ly_slice+lh, lx:lx+lw]
    
    mask_orange_line = cv2.inRange(line_crop, LOWER_ORANGE, UPPER_ORANGE)
    mask_blue_line = cv2.inRange(line_crop, LOWER_BLUE, UPPER_BLUE)

    # Close Blocks ROI (Red, Green, Magenta)
    # close_block_roi = (250, 230, 140, 10) -> y=230 in global is y=130 in slice
    cx, cy, cw, ch = close_block_roi
    cy_slice = cy - GLOBAL_Y_OFFSET
    
    close_crop = hsv_slice[cy_slice:cy_slice+ch, cx:cx+cw]
    
    mask_red1_close = cv2.inRange(close_crop, LOWER_RED_1, UPPER_RED_1)
    
    if USE_LAB:
        mask_red_close = mask_red1_close
    else:
        mask_red2_close = cv2.inRange(close_crop, LOWER_RED_2, UPPER_RED_2)
        mask_red_close = cv2.bitwise_or(mask_red1_close, mask_red2_close)

    mask_green_close = cv2.inRange(close_crop, LOWER_GREEN, UPPER_GREEN)
    mask_magenta_close = cv2.inRange(close_crop, LOWER_MAGENTA, UPPER_MAGENTA)

    # Close Black ROI
    # close_y = 120 -> y=120 in global is y=20 in slice
    cbx, cby, cbw, cbh = close_x, close_y, close_w, close_h
    cby_slice = cby - GLOBAL_Y_OFFSET

    # --- 2. Reconstruct Global Masks (Slice Size) for Wall/Black Detection ---
    
    global_red_mask = np.zeros((SLICE_HEIGHT, FRAME_WIDTH), dtype="uint8")
    global_green_mask = np.zeros((SLICE_HEIGHT, FRAME_WIDTH), dtype="uint8")
    global_blue_mask = np.zeros((SLICE_HEIGHT, FRAME_WIDTH), dtype="uint8")
    global_magenta_mask = np.zeros((SLICE_HEIGHT, FRAME_WIDTH), dtype="uint8")

    # Paste Main Blocks
    global_red_mask[my_slice:my_slice+mh, mx:mx+mw] = cv2.bitwise_or(global_red_mask[my_slice:my_slice+mh, mx:mx+mw], mask_red_main)
    global_green_mask[my_slice:my_slice+mh, mx:mx+mw] = cv2.bitwise_or(global_green_mask[my_slice:my_slice+mh, mx:mx+mw], mask_green_main)
    global_magenta_mask[my_slice:my_slice+mh, mx:mx+mw] = cv2.bitwise_or(global_magenta_mask[my_slice:my_slice+mh, mx:mx+mw], mask_magenta_main)

    # Paste Close Blocks
    global_red_mask[cy_slice:cy_slice+ch, cx:cx+cw] = cv2.bitwise_or(global_red_mask[cy_slice:cy_slice+ch, cx:cx+cw], mask_red_close)
    global_green_mask[cy_slice:cy_slice+ch, cx:cx+cw] = cv2.bitwise_or(global_green_mask[cy_slice:cy_slice+ch, cx:cx+cw], mask_green_close)
    global_magenta_mask[cy_slice:cy_slice+ch, cx:cx+cw] = cv2.bitwise_or(global_magenta_mask[cy_slice:cy_slice+ch, cx:cx+cw], mask_magenta_close)

    # Paste Line Blocks (Blue)
    global_blue_mask[ly_slice:ly_slice+lh, lx:lx+lw] = cv2.bitwise_or(global_blue_mask[ly_slice:ly_slice+lh, lx:lx+lw], mask_blue_line)

    # --- 3. Wall and Black Detection ---
    
    mask_black = cv2.inRange(hsv_slice, LOWER_BLACK, UPPER_BLACK)
    
    mask_red_or_green = cv2.bitwise_or(global_red_mask, global_green_mask)
    mask_red_or_green_or_blue = cv2.bitwise_or(mask_red_or_green, global_blue_mask)
    
    pure_black_mask = cv2.bitwise_and(mask_black, cv2.bitwise_not(mask_red_or_green_or_blue))
    black_or_magenta_mask = cv2.bitwise_or(pure_black_mask, global_magenta_mask)

    # Sliced ROI Masks
    roi_mask_walls_slice = roi_mask_walls[GLOBAL_Y_OFFSET:GLOBAL_Y_END, :]
    roi_mask_close_black_slice = roi_mask_close_black[GLOBAL_Y_OFFSET:GLOBAL_Y_END, :]

    final_mask_walls = cv2.bitwise_and(pure_black_mask, roi_mask_walls_slice)
    final_mask_close_black = cv2.bitwise_and(black_or_magenta_mask, roi_mask_close_black_slice)

    # --- 4. Contour Finding (Fast-Fail) ---

    # Magenta (Main)
    # We already have mask_magenta_main from the crop.
    # But original code used final_mask_magenta = bitwise_and(mask_magenta, roi_mask_magenta)
    # roi_mask_magenta was full_frame_roi. So mask_magenta_main IS the correct mask.
    
    if cv2.countNonZero(mask_magenta_main) > 0:
        contours, _ = cv2.findContours(mask_magenta_main, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            biggest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(biggest_contour)

            if area > MAGENTA_MIN_AREA:
                M = cv2.moments(biggest_contour)
                if M["m00"] != 0:
                    # Correct coordinates
                    # cx is relative to crop, so add mx (which is relative to slice? No, mx is absolute X)
                    # cy is relative to crop, so add my_slice (relative to slice) + GLOBAL_Y_OFFSET (absolute)
                    # Actually, my is absolute Y. So my_slice + GLOBAL_Y_OFFSET = my.
                    # So we can just add mx and my.
                    cx = int(M["m10"] / M["m00"]) + mx
                    cy = int(M["m01"] / M["m00"]) + my

                    # Correct contour for drawing/display if needed
                    biggest_contour_global = biggest_contour + [mx, my]

                    leftmost_x = biggest_contour_global[:, 0, 0].min()
                    rightmost_x = biggest_contour_global[:, 0, 0].max()

                    dist_to_center_left = abs(leftmost_x - FRAME_MIDPOINT_X)
                    dist_to_center_right = abs(rightmost_x - FRAME_MIDPOINT_X)

                    if dist_to_center_left <= dist_to_center_right:
                        target_x = leftmost_x
                    else:
                        target_x = rightmost_x
                    
                    processed_data['detected_magenta'].append({
                        'type': 'magenta_block',
                        'area': area,
                        'centroid': (cx, cy),
                        'contour': biggest_contour_global,
                        'target_x': target_x,
                        'target_y': cy
                    })

    # Blocks (Red, Green, Magenta - Main & Close)
    
    # Helper function to process block contours
    def process_block_contours(mask, offset_x, offset_y, b_type, b_color, min_area):
        if cv2.countNonZero(mask) > 0:
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                biggest_contour = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(biggest_contour)
                if area > min_area:
                    M = cv2.moments(biggest_contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"]) + offset_x
                        cy = int(M["m01"] / M["m00"]) + offset_y
                        biggest_contour_global = biggest_contour + [offset_x, offset_y]
                        return {'type': b_type, 'color': b_color, 'area': area, 'centroid': (cx, cy), 'contour': biggest_contour_global}
        return None

    all_detected_blocks = []
    
    # Main Red
    res = process_block_contours(mask_red_main, mx, my, 'block', 'red', BLOCK_MIN_AREA)
    if res: all_detected_blocks.append(res)
    
    # Main Green
    res = process_block_contours(mask_green_main, mx, my, 'block', 'green', BLOCK_MIN_AREA)
    if res: all_detected_blocks.append(res)

    # Close Red
    res = process_block_contours(mask_red_close, cx, cy, 'close_block', 'red', CLOSE_BLOCK_MIN_AREA)
    if res: all_detected_blocks.append(res)

    # Close Green
    res = process_block_contours(mask_green_close, cx, cy, 'close_block', 'green', CLOSE_BLOCK_MIN_AREA)
    if res: all_detected_blocks.append(res)

    # Close Magenta
    res = process_block_contours(mask_magenta_close, cx, cy, 'close_block', 'magenta', CLOSE_BLOCK_MIN_AREA)
    if res: all_detected_blocks.append(res)

    main_blocks = [b for b in all_detected_blocks if b['type'] == 'block']
    other_blocks = [b for b in all_detected_blocks if b['type'] != 'block']
    if len(main_blocks) > 1:
        lowest_main_block = max(main_blocks, key=lambda b: b['centroid'][1])
        main_blocks = [lowest_main_block]
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
                    cx = int(M["m10"] / M["m00"]) + lx
                    cy = int(M["m01"] / M["m00"]) + ly
                    biggest_contour_global = biggest_contour + [lx, ly]
                    processed_data['detected_orange'].append({'type': 'orange_block', 'color': 'orange', 'area': area, 'centroid': (cx, cy), 'contour': biggest_contour_global})

    # Blue (Line)
    if cv2.countNonZero(mask_blue_line) > 0:
        contours, _ = cv2.findContours(mask_blue_line, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            biggest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(biggest_contour)
            if area > 20:
                M = cv2.moments(biggest_contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"]) + lx
                    cy = int(M["m01"] / M["m00"]) + ly
                    biggest_contour_global = biggest_contour + [lx, ly]
                    processed_data['detected_blue'].append({'type': 'blue_block', 'color': 'blue', 'area': area, 'centroid': (cx, cy), 'contour': biggest_contour_global})

    # Close Black
    if cv2.countNonZero(final_mask_close_black) > 0:
        contours, _ = cv2.findContours(final_mask_close_black, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > WALL_MIN_AREA: 
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"]) + GLOBAL_Y_OFFSET
                    contour_global = contour + [0, GLOBAL_Y_OFFSET]
                    processed_data['detected_close_black'].append({
                        'type': 'close_black', 
                        'color': 'black', 
                        'area': area, 
                        'centroid': (cx, cy), 
                        'contour': contour_global 
                    })

    # Walls
    wall_contours_by_roi = {job['type']: [] for job in [left_side_job, right_side_job, inner_left_side_job, inner_right_side_job]}
    if cv2.countNonZero(final_mask_walls) > 0:
        contours, _ = cv2.findContours(final_mask_walls, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            if cv2.contourArea(c) > WALL_MIN_AREA:
                M = cv2.moments(c)
                if M["m00"] == 0: continue
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"]) + GLOBAL_Y_OFFSET

                job_type = 'unknown'
                if left_side_job['roi'][0] <= cx < left_side_job['roi'][0] + left_side_job['roi'][2]: job_type = left_side_job['type']
                elif right_side_job['roi'][0] <= cx < right_side_job['roi'][0] + right_side_job['roi'][2]: job_type = right_side_job['type']
                elif inner_left_side_job['roi'][0] <= cx < inner_left_side_job['roi'][0] + inner_left_side_job['roi'][2]: job_type = inner_left_side_job['type']
                elif inner_right_side_job['roi'][0] <= cx < inner_right_side_job['roi'][0] + inner_right_side_job['roi'][2]: job_type = inner_right_side_job['type']

                if job_type != 'unknown':
                    wall_contours_by_roi[job_type].append(c)

    for job_type, contour_list in wall_contours_by_roi.items():
        if contour_list:
            biggest_contour = max(contour_list, key=cv2.contourArea)
            area = cv2.contourArea(biggest_contour)
            M = cv2.moments(biggest_contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"]) + GLOBAL_Y_OFFSET
                biggest_contour_global = biggest_contour + [0, GLOBAL_Y_OFFSET]
                processed_data['detected_walls'].append({'type': job_type, 'color': 'black', 'area': area, 'centroid': (cx, cy), 'contour': biggest_contour_global})
    
    return processed_data

def annotate_video_frame(frame, detections, driving_direction, debug_info="", visual_target_x=None, visual_target_line=None):
    annotated_frame = frame
    light_blue = (255, 255, 0)
    target_line_color = (255, 0, 255)
    midpoint_color = (0, 255, 255)
    magenta_target_color = (255, 255, 255)

    all_rois = [
        (left_roi_x, left_roi_y, left_roi_w, left_roi_h),
        (right_roi_x, right_roi_y, right_roi_w, right_roi_h),
        (inner_left_roi_x, inner_left_roi_y, inner_left_roi_w, inner_left_roi_h),
        (inner_right_roi_x, inner_right_roi_y, inner_right_roi_w, inner_right_roi_h),
        (line_roi_x, line_roi_y, line_roi_w, line_roi_h),
        (close_x, close_y, close_w, close_h),
        full_frame_roi,
        close_block_roi
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
        elif block['color'] == 'magenta':
            draw_color = (255, 0, 255)
        cv2.drawContours(annotated_frame, [block['contour']], -1, draw_color, 2)

    for orange_obj in detections['detected_orange']:
        cv2.drawContours(annotated_frame, [orange_obj['contour']], -1, (0, 165, 255), 2)

    for blue_obj in detections['detected_blue']:
        cv2.drawContours(annotated_frame, [blue_obj['contour']], -1, (255, 0, 0), 2)

    for black_obj in detections.get('detected_close_black', []):
        cv2.drawContours(annotated_frame, [black_obj['contour']], -1, (0, 0, 0), 2)

    for magenta_obj in detections['detected_magenta']:
        cv2.drawContours(annotated_frame, [magenta_obj['contour']], -1, (255, 0, 255), 2)
        target_x = magenta_obj['target_x']
        cy = magenta_obj['centroid'][1]
        cv2.circle(annotated_frame, (target_x, cy), 7, (255, 255, 255), -1)

    if visual_target_x is not None:
        cv2.line(annotated_frame, (visual_target_x, 0), (visual_target_x, FRAME_HEIGHT), target_line_color, 2)

    if visual_target_line is not None:
        # Draw the actual line pointing towards the block (Cyan)
        pt1, pt2, ideal_angle = visual_target_line[:3]
        cv2.line(annotated_frame, pt1, pt2, (0, 255, 255), 2)
        
        # Calculate and draw the "ideal" target line (Yellow)
        if len(visual_target_line) == 4:
            pt3 = visual_target_line[3]
            cv2.line(annotated_frame, pt1, pt3, (0, 255, 255), 3)
        else:
            origin_x, origin_y = pt1[0], pt1[1]
            # Calculate target point based on angle
            target_len = 200
            target_pt_x = int(origin_x + target_len * math.sin(math.radians(ideal_angle)))
            target_pt_y = int(origin_y - target_len * math.cos(math.radians(ideal_angle)))
            # Draw the target line
            cv2.line(annotated_frame, (origin_x, origin_y), (target_pt_x, target_pt_y), (0, 255, 255), 3)

    for magenta_obj in detections['detected_magenta']:
        target_x = magenta_obj['target_x']
        cy = magenta_obj['centroid'][1]
        cv2.circle(annotated_frame, (target_x, cy), 7, (255, 255, 255), -1)

    cv2.putText(annotated_frame, str(debug_info), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
    
    return annotated_frame

def drive_straight_with_gyro(target_heading, duration, speed, direction='forward'):
    print(f"Driving {direction} with gyro stabilization for {duration}s...")
    
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
    print("Gyro-stabilized drive complete.")

def steer_with_gyro(current_heading: float, 
                    target_heading: float, 
                    kp: float = 0.85, 
                    min_servo_angle: int = -45, 
                    max_servo_angle: int = 45) -> float:
    """
    Calculates the servo steering angle to maintain a target heading using a gyro.

    This function uses a Proportional (P) controller to correct the robot's
    trajectory. It correctly handles the "wrap-around" issue where headings
    jump between 359 and 0 degrees.

    Args:
        current_heading (float): The current direction of the robot in degrees (0-359).
        target_heading (float): The desired direction in degrees (0-359).
        kp (float): The proportional gain constant. This critical value determines
                    how aggressively the robot corrects its course.
                    - Higher value = more aggressive, sharper turns (can lead to oscillation).
                    - Lower value = softer, smoother turns (can be slow to correct).
        min_servo_angle (int): The minimum allowable angle for the servo (e.g., full left).
        max_servo_angle (int): The maximum allowable angle for the servo (e.g., full right).

    Returns:
        float: The calculated servo angle, clamped within the min/max limits.
    """
    # 1. Calculate the error between target and current heading.
    error = target_heading - current_heading

    # 2. Handle the wrap-around issue (e.g., turning from 350 degrees to 10 degrees).
    # The shortest path is not -340 degrees, but +20 degrees.
    if error > 180:
        error -= 360
    elif error < -180:
        error += 360

    # 3. Calculate the steering correction.
    # This is the "Proportional" part of the controller. The steering angle is
    # directly proportional to the error, scaled by the gain 'kp'.
    steer_angle = kp * error

    # 4. Clamp the steering angle to the servo's physical limits.
    # This prevents the code from trying to turn the servo beyond its capabilities.
    clamped_steer_angle = np.clip(steer_angle, min_servo_angle, max_servo_angle)

    return clamped_steer_angle

def perform_initial_maneuver():
    print("--- Executing Full Initial Maneuver ---")

    SERVO_TURN_ANGLE = 40.0
    
    if driving_direction == "clockwise":
        initial_turn_servo = SERVO_TURN_ANGLE
        direction_modifier = 1
        target_forward_heading = (INITIAL_HEADING + 55) % 360
        drive_duration = 0.2
    else:
        initial_turn_servo = -SERVO_TURN_ANGLE
        direction_modifier = -1
        target_forward_heading = (INITIAL_HEADING - 55) % 360
        drive_duration = 0.2

    print(f"Direction: {driving_direction.upper()}")
    print(f"Initial Heading: {INITIAL_HEADING:.1f}°")
    print(f"Forward Drive Target Heading: {target_forward_heading:.1f}°")

    # Start the initial turn (skip scan completely)
    motor.forward(80)
    servo.set_angle_unlimited(initial_turn_servo)
    print("Starting initial turn (skipping scan)...")
    
    while get_angular_difference(target_forward_heading, imu_thread.get_heading()) > 15:
        time.sleep(0.01)

    print("Driving forward with gyro stabilization...")
    drive_straight_with_gyro(target_forward_heading, drive_duration, 70, 'forward')

    print(f"Performing final turn to return to {INITIAL_HEADING:.1f}°...")
    motor.forward(50)

    while get_angular_difference(imu_thread.get_heading(), INITIAL_HEADING) > 15:
        servo.set_angle(steer_with_gyro(imu_thread.get_heading(), INITIAL_HEADING, kp=2.0, min_servo_angle=-30, max_servo_angle=30))
        time.sleep(0.01)
        
    motor.brake()
    servo.set_angle(0)
    
    print("Reversing slightly to align heading...")
    motor.reverse(50)
    start_time = time.monotonic()
    while time.monotonic() - start_time < 0.4:
        servo.set_angle(-steer_with_gyro(imu_thread.get_heading(), INITIAL_HEADING, kp=1.0))
        time.sleep(0.01)
    time.sleep(0.5)
    motor.brake()
    servo.set_angle(0)
    print("--- Initial Maneuver Complete. Transitioning to straight driving. ---")

def parking():
    # motor.forward(65)
    # print('forward')
    # print(sensor_thread.get_readings()['distance_center'])
    # heading = 15
    # if driving_direction == 'clockwise':
    #     heading = 2
    # while True:
    #     sensor_readings = sensor_thread.get_readings()
    #     distance_center = sensor_readings.get('distance_center')
    #     if distance_center is not None and distance_center <= 700:
    #         print(f"Distance is {distance_center}. Exiting loop.")
    #         break
    #     servo.set_angle(steer_with_gyro(sensor_readings['heading'], (INITIAL_HEADING + heading) % 360, kp=0.8))
    #     time.sleep(0.01)
    motor.start_rpm_control(80, "reverse")
    #return
    #print(sensor_thread.get_readings()['distance_back'], sensor_readings['heading'])
    while True:
        sensor_readings = sensor_thread.get_readings()
        heading = imu_thread.get_heading()
        print(sensor_readings['distance_back'], heading)
        if sensor_readings['distance_back'] is not None and sensor_readings['distance_back'] < 30:
            break
        servo.set_angle_unlimited(-steer_with_gyro(heading,(INITIAL_HEADING+90)%360, kp=1, min_servo_angle=-60, max_servo_angle=60))
        time.sleep(0.01)
    #return
    motor.start_rpm_control(80, "forward")
    while get_angular_difference((INITIAL_HEADING+180)%360, imu_thread.get_heading()) > 5:
        #print(INITIAL_HEADING, imu_thread.get_heading())
        heading = imu_thread.get_heading()
        servo.set_angle(steer_with_gyro(heading,(INITIAL_HEADING+180)%360, kp=1,min_servo_angle=-40, max_servo_angle=40))
    motor.stop_rpm_control()
    servo.set_angle(0)
    first_magenta_line_passed = False
    # This helper flag tracks if we are currently over the first line.
    on_first_line = False

    # Thresholds
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
            print("Failed to get frame, breaking loop.")
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
        except Exception as e:
            print(e)

        if first_magenta_line_passed:
            if magenta_pixel_count > MAGENTA_HIGH_THRESHOLD:
                print("Second magenta line detected. Stopping.")
                break
        else:
            if on_first_line:
                if magenta_pixel_count < MAGENTA_LOW_THRESHOLD:
                    print("First magenta line fully crossed. Now armed to stop on the next one.")
                    first_magenta_line_passed = True
            else:
                if magenta_pixel_count > MAGENTA_HIGH_THRESHOLD:
                    print("Detected what seems to be the first magenta line.")
                    on_first_line = True
    servo.set_angle(3)
    time.sleep(0.45)
    motor.stop_rpm_control()
    motor.start_rpm_control(80, "reverse")
    servo.set_angle_unlimited(55)
    while get_angular_difference((INITIAL_HEADING+100)%360, imu_thread.get_heading()) > 10:
            pass
    motor.stop_rpm_control()
    print('parking first reverse turn:',sensor_thread.get_readings())
    motor.start_rpm_control(80, "reverse")
    servo.set_angle(0)
    print('reverse')
    while True:
        dist = sensor_thread.get_readings()['distance_back']
        print('Reverse for parking back distance:', dist)
        if dist is not None and dist < 170:
            break
        time.sleep(0.01)
    print('parking forward:',sensor_thread.get_readings())
    motor.start_rpm_control(80, "forward")
    while True:
        sensor_readings = sensor_thread.get_readings()
        dist = sensor_readings['distance_back']
        heading = imu_thread.get_heading()
        print('Forward for parking back distance:', dist, 'heading:', heading)
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

def parkingb():
    motor.reverse(60)
    while True:
        sensor_readings = sensor_thread.get_readings()
        heading = imu_thread.get_heading()
        print(sensor_readings['distance_back'], heading)
        if sensor_readings['distance_back'] is not None and sensor_readings['distance_back'] < 160:
            break
        servo.set_angle_unlimited(-steer_with_gyro(heading,(INITIAL_HEADING+90)%360, kp=1, min_servo_angle=-60, max_servo_angle=60))
        time.sleep(0.01)
    motor.brake()

    motor.reverse(60)
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
            if dist <= 65:
                break
        if get_angular_difference((INITIAL_HEADING+180)%360, imu_thread.get_heading()) < 2:
            break
    motor.brake()

def parking2():
    global INITIAL_HEADING
    motor.forward(60)
    print('forward')
    print(sensor_thread.get_readings()['distance_center'])
    while True:
        sensor_readings = sensor_thread.get_readings()
        distance_center = sensor_readings.get('distance_center')
        if distance_center is not None and distance_center <= 200:
            print(f"Distance is {distance_center}. Exiting loop.")
            break
        print(distance_center, imu_thread.get_heading())
        servo.set_angle(steer_with_gyro(imu_thread.get_heading(), (INITIAL_HEADING+5) % 360, kp=1))
        time.sleep(0.01)
    motor.reverse(60)
    #return
    #print(sensor_thread.get_readings()['distance_back'], sensor_readings['heading'])
    while True:
        sensor_readings = sensor_thread.get_readings()
        heading = imu_thread.get_heading()
        print(sensor_readings['distance_back'], heading)
        if sensor_readings['distance_back'] is not None and sensor_readings['distance_back'] < 160:
            break
        servo.set_angle_unlimited(-steer_with_gyro(heading,(INITIAL_HEADING-90)%360, kp=2, min_servo_angle=-60, max_servo_angle=60))
        time.sleep(0.01)
    #return
    motor.forward(60) 
    while get_angular_difference((INITIAL_HEADING-180)%360, imu_thread.get_heading()) > 5:
            #print(INITIAL_HEADING, imu_thread.get_heading())
            heading = imu_thread.get_heading()
            servo.set_angle(steer_with_gyro(heading,(INITIAL_HEADING-180)%360, kp=1,min_servo_angle=-40, max_servo_angle=40))   
    ROI_Y_START = 140
    ROI_X_END = 40
    TARGET_Y_OFFSET_FROM_BOTTOM = 185

    MAGENTA_HIGH_THRESHOLD = 500
    MAGENTA_LOW_THRESHOLD = 200

    ROI_Y_START = 140
    ROI_X_END = 40
    TARGET_Y_OFFSET_FROM_BOTTOM = 185

    # State variables for magenta line detection
    first_magenta_line_passed = False
    on_first_line = False

    past_frame_counter = 0
    while True:
        frame, frame_counter = camera_thread.get_next_frame(past_frame_counter)
        if frame is None:
            print("Failed to get frame, breaking loop.")
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

        # Magenta line detection ROI on bottom left
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
                print("Second magenta line detected. Stopping.")
                break
        else:
            if on_first_line:
                if magenta_pixel_count < MAGENTA_LOW_THRESHOLD:
                    print("First magenta line fully crossed. Now armed to stop on the next one.")
                    first_magenta_line_passed = True
            else:
                if magenta_pixel_count > MAGENTA_HIGH_THRESHOLD:
                    print("Detected what seems to be the first magenta line.")
                    on_first_line = True
        
        try:
            video_writer_thread.write(frame)
        except Exception as e:
            print(e)

    servo.set_angle(1)
    time.sleep(1.0)
    motor.brake()
    motor.reverse(60)
    servo.set_angle_unlimited(-60)
    while get_angular_difference((INITIAL_HEADING-100)%360, imu_thread.get_heading()) > 10:
            pass
    motor.brake()
    print('parking first reverse turn:',sensor_thread.get_readings())
    motor.reverse(60)
    servo.set_angle(0)
    print('reverse')
    while True:
        dist = sensor_thread.get_readings()['distance_back']
        print('Reverse for parking back distance:', dist)
        if dist is not None and dist < 180:
            break
        time.sleep(0.01)
    print('parking forward:',sensor_thread.get_readings())
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
    
def parking2b():
    global INITIAL_HEADING
    motor.reverse(60)
    while True:
        sensor_readings = sensor_thread.get_readings()
        heading = imu_thread.get_heading()
        print(sensor_readings['distance_back'], heading)
        if sensor_readings['distance_back'] is not None and sensor_readings['distance_back'] < 180:
            break
        servo.set_angle_unlimited(-steer_with_gyro(heading,(INITIAL_HEADING-90)%360, kp=2, min_servo_angle=-60, max_servo_angle=60))
        time.sleep(0.01)
    motor.brake()

    motor.reverse(60)
    servo.set_angle_unlimited(65)
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
    fourcc = cv2.VideoWriter_fourcc(*'avc1')

    video_writer_thread = VideoWriterThread(video_path, fourcc, 20, (640, 360))
    video_writer_thread.start()
    annotate_and_write_thread = AnnotateAndWriteThread(video_writer_thread)
    annotate_and_write_thread.start()
    
    if not camera.initialize():
        print("MainThread: FATAL: Camera initialization failed. Exiting.")
        sys.exit(1)
    motor.initialize()
    servo.initialize()
    button = Button(23)
    led = LED(12)
    
    orange_detection_history = deque([False] * ORANGE_DETECTION_HISTORY_LENGTH,maxlen=ORANGE_DETECTION_HISTORY_LENGTH)
    cooldown_frames = 0
    orange_detection_history.append(False)
    turn_counter = 0
    angle = 0
    prevangle = 0
    prevspeed = 0

    # Close-block loss tracking: remember when a red/green block was last being
    # avoided and dropped out of the frame (passed under/beside the robot). The
    # inner-corner recovery latch below uses this to know the robot has just
    # come off a close block.
    BLOCK_CLOSE_Y = 160
    frames_since_block_lost = 999
    last_block_was_close = False

    # Inner-corner over-rotation recovery latch:
    # Driving clockwise, after avoiding a green pillar -> corner -> red pillar,
    # the robot can end up OVER-rotated (~135 instead of ~90) pointing straight
    # at the convex corner of the central island. In that exact pose:
    #   - none of the four wall ROIs see the outer boundary (it is out of frame),
    #   - the island corner sits dead-centre and shows up as a CENTERED, PARTIAL
    #     black band -- not wide enough to trip the close_black>3000 rule, with
    #     open floor on both sides.
    # The normal no-wall fallback reads "no walls" as an open corner and nudges
    # FURTHER in the travel direction (right, clockwise), driving deeper into the
    # corner. This latch detects that pose and actively COUNTER-steers out of it
    # (opposite the travel direction). Mirror of the block-recovery latch above.
    RECOVERY_STEER = 22             # counter-steer magnitude (deg, before slew/clip)
    RECOVERY_BLACK_AREA_MIN = 800   # centered black must be at least this (partial)
    RECOVERY_BLACK_AREA_MAX = 3000  # ...and below the existing full-width trip
    RECOVERY_CENTROID_TOL = 60      # |black centroid - midpoint| must be within this
    RECOVERY_WINDOW_FRAMES = 25     # frames after losing a close block we may fire (> BLOCK_RECOVERY_FRAMES)
    RECOVERY_WALL_REACQUIRE = 200   # a side wall area above this ends the recovery
    RECOVERY_MAX_FRAMES = 40        # hard cap so the latch can never get stuck
    in_inner_corner_recovery = False
    inner_corner_recovery_frames = 0

    camera_thread = CameraThread(camera)
    camera_thread.start()

    dataset_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), 'dataset')
    os.makedirs(dataset_dir, exist_ok=True)
    dataset_thread = DatasetCaptureThread(camera_thread, dataset_dir)
    dataset_thread.start()
        
    sensors_initialized_event = threading.Event()
    sensor_thread = SensorThread(distance, sensors_initialized_event)
    sensor_thread.start()
    print("MainThread: Waiting for sensors to initialize...")
    sensors_initialized_event.wait() 
    print("MainThread: Sensors are ready.")    

    imu_initialized_event = threading.Event()
    imu_thread = ImuThread(bno055, imu_initialized_event)
    imu_thread.start()
    print("MainThread: Waiting for IMU to initialize...")    
    imu_initialized_event.wait()
    print("MainThread: IMU is ready. Proceeding with main logic.")    
    
    time.sleep(1)
    led.on()
    # button.wait_for_press()
    led.off()
    time.sleep(0.5)
    driving_direction = 'clockwise'
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
        last_turn_time = run_start_time
        past_frame_counter = 0
        frame_counter = 0
        perform_initial_maneuver()
        motor.forward(MOTOR_SPEED)
        frame_start_time = time.perf_counter()
        print(f"[{run_start_time:.2f}] Starting run at {datetime.now().strftime('%H:%M:%S.%f')[:-3]}")
        while True:
            speed = MOTOR_SPEED
            angle=0
            debug = []
            visual_target_x = None
            visual_target_line = None
            block_avoided_this_frame = False
            frame, frame_counter = camera_thread.get_next_frame(past_frame_counter)
            if frame is None:
                continue
            past_frame_counter = frame_counter
            sensor_readings = sensor_thread.get_readings()

            detections = process_video_frame(frame)
            detected_blocks = detections['detected_blocks']
            detected_walls = detections['detected_walls']
            detected_orange_object = detections['detected_orange']
            detected_blue_object = detections['detected_blue']

            blue_detected_this_frame = bool(detected_blue_object)
            orange_detected_this_frame = bool(detected_orange_object)
            orange_detection_history.append(orange_detected_this_frame)

            if cooldown_frames > 0:
                cooldown_frames -= 1
            elif not orange_detection_history[-ORANGE_DETECTION_HISTORY_LENGTH] and all(list(orange_detection_history)[1:]):
                current_time = time.monotonic()
                turn_counter += 1
                time_since_last_turn = current_time - last_turn_time
                time_since_start = current_time - run_start_time
                cooldown_frames = ORANGE_COOLDOWN_FRAMES
                print(f"[{current_time:.2f}] Turn {turn_counter} detected | Since last turn: {time_since_last_turn:.2f}s | Total elapsed: {time_since_start:.2f}s")
                last_turn_time = current_time

            if detected_blocks:
                is_close_block = False
                for block in detected_blocks:
                    if block['type'] == 'close_block':
                        is_close_block = True
                        if block['color'] == 'magenta' and (time.monotonic()-run_start_time)>5:
                            if driving_direction == 'clockwise':
                                angle = -25
                            else:
                                angle = 30
                        elif block['color'] == 'red':
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
                    block = detected_blocks[0]
                    block_color = block['color']
                    block_x, block_y = block['centroid']
                    # Mark that we are actively steering around a block this
                    # frame, and remember if it was low/close (about to exit the
                    # bottom of the frame) so the recovery latch can engage once
                    # it disappears.
                    block_avoided_this_frame = True
                    last_block_was_close = block_y > BLOCK_CLOSE_Y
                    debug.append((block_x, block_y))
                    
                    if block_color == 'red':
                        # TUNING PARAMETERS
                        RED_OTHER_X = 240
                        RED_OTHER_Y = 0
                        RED_ORIGIN_X = 20
                        RED_ORIGIN_Y = FRAME_HEIGHT
                        RED_IDEAL_ANGLE = math.degrees(math.atan2(RED_OTHER_X - RED_ORIGIN_X, RED_ORIGIN_Y - RED_OTHER_Y))
                        
                        wall_inner_right_size = sum(obj['area'] for obj in detected_walls if obj['type'] == 'wall_inner_right')
                        target = 300 if block_y > 170 and 200 < block_x < 440 else 150
                        debug.append(target)
                        if detections['detected_magenta'] and driving_direction == 'counter-clockwise' and abs(detections['detected_magenta'][0]['target_y']-block_y)<70 and abs(detections['detected_magenta'][0]['centroid'][0]-block_x)>70:
                            target_x = detections['detected_magenta'][0]['target_x']
                            midpoint_x = (block_x + target_x) // 2
                            visual_target_x = midpoint_x
                            angle = (midpoint_x - FRAME_MIDPOINT_X) * 0.10
                        else:
                            visual_target_line = ((RED_ORIGIN_X, RED_ORIGIN_Y), (block_x, block_y), RED_IDEAL_ANGLE, (RED_OTHER_X, RED_OTHER_Y))
                            current_angle = math.degrees(math.atan2(block_x - RED_ORIGIN_X, RED_ORIGIN_Y - block_y))
                            angle = (current_angle - RED_IDEAL_ANGLE) * 1.5
                        if wall_inner_right_size > 3000: angle = np.clip(angle, -45, -10)
                        else: angle = np.clip(angle, -45, 35)
                    
                    elif block_color == 'green':
                        # TUNING PARAMETERS
                        GREEN_OTHER_X = 400
                        GREEN_OTHER_Y = 0
                        GREEN_ORIGIN_X = 620
                        GREEN_ORIGIN_Y = FRAME_HEIGHT
                        GREEN_IDEAL_ANGLE = math.degrees(math.atan2(GREEN_OTHER_X - GREEN_ORIGIN_X, GREEN_ORIGIN_Y - GREEN_OTHER_Y))

                        wall_inner_left_size = sum(obj['area'] for obj in detected_walls if obj['type'] == 'wall_inner_left')
                        target = 300 if block_y > 160 and 240 < block_x < 400 else 150
                        if detections['detected_magenta'] and driving_direction == 'clockwise' and abs(detections['detected_magenta'][0]['target_y']-block_y)<70:
                            target_x = detections['detected_magenta'][0]['target_x']
                            midpoint_x = (block_x + target_x) // 2
                            visual_target_x = midpoint_x
                            angle = (midpoint_x - FRAME_MIDPOINT_X) * 0.10
                        else:
                            visual_target_line = ((GREEN_ORIGIN_X, GREEN_ORIGIN_Y), (block_x, block_y), GREEN_IDEAL_ANGLE, (GREEN_OTHER_X, GREEN_OTHER_Y))
                            current_angle = math.degrees(math.atan2(block_x - GREEN_ORIGIN_X, GREEN_ORIGIN_Y - block_y))
                            angle = (current_angle - GREEN_IDEAL_ANGLE) * 1.5
                        if wall_inner_left_size > 3000: angle = np.clip(angle, 15, 45)
                        else: angle = np.clip(angle, -45, 45 )
            elif detections['detected_magenta']:
                if driving_direction == 'clockwise':
                    target = 320-200
                else:
                    target = 320+220
                angle = (detections['detected_magenta'][0]['centroid'][0] - target) * 0.15
        
            else:
                left_pixel_size,right_pixel_size,wall_inner_left_size,wall_inner_right_size,target=0,0,0,0,0
                left_pixel_size = sum(obj['area'] for obj in detected_walls if obj['type'] == 'wall_left')
                right_pixel_size = sum(obj['area'] for obj in detected_walls if obj['type'] == 'wall_right')
                wall_inner_left_size = sum(obj['area'] for obj in detected_walls if obj['type'] == 'wall_inner_left')
                wall_inner_right_size = sum(obj['area'] for obj in detected_walls if obj['type'] == 'wall_inner_right')
                # Capture the un-boosted outer-wall areas: the block below inflates
                # left/right_pixel_size for wall-following, but the inner-corner
                # recovery detector needs the genuine "is any outer wall visible?".
                raw_left_pixel_size = left_pixel_size
                raw_right_pixel_size = right_pixel_size
                if driving_direction == 'clockwise':
                    if right_pixel_size<1000 and (left_pixel_size + wall_inner_left_size)>100:
                        left_pixel_size *= 4
                        left_pixel_size += 12500
                elif driving_direction == 'counter-clockwise':
                    if left_pixel_size<1000 and (right_pixel_size + wall_inner_right_size)>100:
                        right_pixel_size *= 4
                        right_pixel_size += 12500

                debug.extend([left_pixel_size + wall_inner_left_size, right_pixel_size + wall_inner_right_size])
                angle = ((left_pixel_size + wall_inner_left_size) - (right_pixel_size + wall_inner_right_size)) * 0.001

                close_black_area = sum(obj['area'] for obj in detections.get('detected_close_black', []))
                if close_black_area > 3000:
                    if driving_direction == 'clockwise':
                        angle += 35
                    else:
                        angle += -35
                if left_pixel_size == 0 and right_pixel_size == 0 and(detected_orange_object or detected_blue_object):
                    if driving_direction == 'clockwise':
                        angle += 35
                    else:
                        angle += -35
                elif left_pixel_size == 0 and wall_inner_left_size == 0 and right_pixel_size == 0 and wall_inner_right_size == 0:
                    if driving_direction == 'clockwise':
                        angle += 17.5
                    else:
                        angle += -17.5
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
            except Exception as e:
                print(e)
            angle = np.clip(angle, prevangle-6, prevangle+6)
            angle = np.clip(angle,-30,30)
            if angle != prevangle:
                servo.set_angle(angle)
            prevangle = angle
            if speed != prevspeed:
                motor.forward(speed)
            prevspeed = speed
            angle = 0

            # Track frames since we last steered around a block, used by the
            # block-recovery latch in the wall-following branch above.
            if block_avoided_this_frame:
                frames_since_block_lost = 0
            else:
                frames_since_block_lost += 1
            
            if False:
                cv2.imshow("Robot Live Feed", annotated_frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            
            if button.is_pressed:
                motor.brake()
                run_end_time = time.monotonic()
                total_time = run_end_time - run_start_time
                print(f"\n[{run_end_time:.2f}] Run stopped by button | Total time: {total_time:.2f}s | Total turns: {turn_counter}")
                break
            if turn_counter >= 13:
                if driving_direction == 'clockwise':
                    parking()
                else:
                    parking2()
                run_end_time = time.monotonic()
                run_time = run_end_time-run_start_time
                print(run_time)
                motor.brake()
                break

    except Exception as e:
        print(f"MainThread: ERROR during execution: {e}")
        traceback.print_exc()

    finally:
        # Stop the background RPM controller FIRST: it runs in a daemon thread
        # that re-commands the motor every ~20ms, so until it's stopped the motor
        # won't actually halt, and if we close the GPIO chip while it's still
        # looping its next gpio_write hits a dead handle ('unknown handle').
        # --- PRIORITY: halt and fully release the motor immediately, before the
        # slow teardown below (bounded thread joins, camera cleanup). This
        # guarantees the robot stops even if the rest drags on for seconds.
        try:
            motor.stop_rpm_control()
            motor.brake()
            motor.cleanup()
        except Exception as e:
            print(f"Motor priority shutdown error (ignored): {e}")

        # --- Everything below is non-urgent and may take a couple seconds. ---
        try:
            run_end_time = time.monotonic()
            total_time = run_end_time - run_start_time
            print(f"\n{'='*60}")
            print(f"[{run_end_time:.2f}] Run completed | Total time: {total_time:.2f}s | Total turns: {turn_counter}")
            if turn_counter > 0:
                avg_time_per_turn = total_time / turn_counter
                print(f"Average time per turn: {avg_time_per_turn:.2f}s")
            print(f"{'='*60}\n")
        except:
            pass
        servo.set_angle(0)
        time.sleep(0.5)
        print(sensor_thread.get_readings())
        print("MainThread: Signaling threads to stop...")
        dataset_thread.stop()
        camera_thread.stop()
        sensor_thread.stop()
        imu_thread.stop()
        annotate_and_write_thread.stop()

        print("MainThread: Waiting for threads to complete...")
        # Bounded joins so a single Ctrl-C actually exits. The threads are
        # daemons, so anything still stuck (e.g. a sensor mid-reinit on a dead
        # bus) is torn down with the process instead of hanging the join.
        dataset_thread.join(timeout=2)
        camera_thread.join(timeout=2)
        sensor_thread.join(timeout=2)
        imu_thread.join(timeout=2)
        annotate_and_write_thread.join(timeout=2)
        video_writer_thread.stop()
        video_writer_thread.join(timeout=2)
        print("MainThread: All threads have completed.")

        camera.cleanup()
        servo.set_angle(0)
        servo.cleanup()
        cv2.destroyAllWindows()
        if 'log_file' in locals() and not log_file.closed:
            print(f"Log file saved to {log_path}") # This will print to your console
            log_file.close()

from collections import deque
import time
import queue
from src.sensors import bno086, camera, distance
from src.motors import motor, servo
import numpy as np
import cv2
import threading
from gpiozero import Button, LED
from src.obstacle_challenge import config
import os
import sys
import traceback
from datetime import datetime
import math

# MOTOR_SPEED = 88
# ORANGE_COOLDOWN_FRAMES = 45

MOTOR_SPEED = 65
ORANGE_COOLDOWN_FRAMES = 50

#MOTOR_SPEED = 92
#ORANGE_COOLDOWN_FRAMES = 45

ORANGE_DETECTION_HISTORY_LENGTH = 4

WALL_THRESHOLD = 200
WALL_KP = 0.0006
WALL_KD = 0.0003

GYRO_KP = 0.85
GYRO_KD = 0.1


FRAME_WIDTH = 640
FRAME_HEIGHT = 360
FRAME_MIDPOINT_X = FRAME_WIDTH // 2

USE_LAB = False

HSV_RANGES = {
    'LOWER_RED_1': np.array([0, 92, 43]), 'UPPER_RED_1': np.array([4, 230, 166]),
    'LOWER_RED_2': np.array([175, 100, 43]), 'UPPER_RED_2': np.array([180, 230, 140]),
    'LOWER_GREEN': np.array([42, 85, 38]), 'UPPER_GREEN': np.array([88, 190, 135]),
    'LOWER_BLACK': np.array([0, 0, 0]), 'UPPER_BLACK': np.array([180, 95, 70]),
    'LOWER_ORANGE': np.array([6, 50, 182]), 'UPPER_ORANGE': np.array([15, 255, 255]),
    'LOWER_BLUE': np.array([114, 50, 110]), 'UPPER_BLUE': np.array([123, 255, 255]),
    'LOWER_MAGENTA': np.array([158, 73, 64]), 'UPPER_MAGENTA': np.array([172, 255, 223])
}

LAB_RANGES = {
    'LOWER_RED_1': np.array([30, 159, 137]), 'UPPER_RED_1': np.array([158, 175, 169]),
    'LOWER_RED_2': np.array([20, 150, 150]), 'UPPER_RED_2': np.array([200, 255, 255]), # Duplicate for now
    'LOWER_GREEN': np.array([79, 80, 115]), 'UPPER_GREEN': np.array([134, 129, 146]),
    'LOWER_BLACK': np.array([0, 115, 115]), 'UPPER_BLACK': np.array([130, 134, 134]),
    'LOWER_ORANGE': np.array([97, 136, 138]), 'UPPER_ORANGE': np.array([177, 169, 172]),
    'LOWER_MAGENTA': np.array([72, 147, 48]), 'UPPER_MAGENTA': np.array([159, 174, 130]),
    'LOWER_BLUE': np.array([28, 136, 44]), 'UPPER_BLUE': np.array([100, 163, 104])
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
BLOCK_MIN_AREA = 350
MAGENTA_MIN_AREA = 300
CLOSE_BLOCK_MIN_AREA = 15

left_roi_x, left_roi_y, left_roi_w, left_roi_h = 0, 130, 135, 150
right_roi_x, right_roi_y, right_roi_w, right_roi_h = 505, 130, 135, 150
inner_left_roi_x, inner_left_roi_y, inner_left_roi_w, inner_left_roi_h = 140, 155, 100, 100
inner_right_roi_x, inner_right_roi_y, inner_right_roi_w, inner_right_roi_h = 400, 155 , 100, 100
line_roi_x, line_roi_y, line_roi_w, line_roi_h = 280, 190, 80, 40
close_x,close_y,close_w,close_h = 140,110,360,10
full_frame_roi = (0, 80, 640, 170)
close_block_roi = (280, 215, 80, 10)

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

# --- Arena mask -------------------------------------------------------------
# v5 only. The fixed ROIs above say *where in the frame* to look; they can't say
# *whether the thing we found is inside the arena*. full_frame_roi spans the whole
# width, so any red/green blob in rows 80-250 becomes a pillar -- including blobs in
# the far track section seen over the inner wall, and anything outside the arena.
#
# The arena mask fixes that. It is rebuilt every frame from the image itself: take
# the largest connected non-black blob that CONTAINS the patch of mat directly in
# front of the robot, then extend it upward through the black wall standing on it,
# stopping at the wall's top edge. The far section is a separate blob (the wall
# breaks connectivity) so it is structurally unreachable, not merely thresholded out.
#
# Unlike a brightness-threshold pipeline, pillars are automatically part of the floor
# blob here -- they aren't black, so they pass NOT(black) and stay continuous with the
# mat. No colour repainting stage is needed. Magenta counts as floor too, so the
# parking walls survive the gate.
USE_ARENA_MASK = True        # False -> byte-identical behaviour to main_v2
ARENA_Y_TOP = 50             # hard clamp: nothing above this row is ever arena
ARENA_Y_BOTTOM = 250         # below this the chassis is in frame (chassis: x180 y250 w280 h110)
ARENA_SEED_PT = (320, 230)   # patch of mat directly in front of the robot, frame coords
ARENA_TOP_MARGIN = 0         # TUNING KNOB: push the skyline DOWN N px to trim the wall band
MAX_WALL_RUN = 160           # run longer than this = column untrusted (see build_arena_mask).
                             # Measured on real frames: wall runs reach p99=90, max=155 px
                             # when the robot is close to a wall. 70 clipped 6% of columns.
MIN_WALL_THICK = 8           # a black run must be >= this tall to count as a wall
WALL_GAP_SEAL = 3            # seal bright breaks up to this tall inside a wall, so the
                             # upward scan isn't stopped early by glare/noise/artifacts.
                             # Do NOT raise much: sealing also lets the run climb past
                             # the wall into dark background. Measured over 40 frames --
                             # seal 3: 0.44 px/col jagged, 1/40 frames collapse to the
                             # full band; seal 9: 0.41 px/col but 7/40 collapse.
ARENA_CLOSE_KERNEL = np.ones((9, 9), np.uint8)
ARENA_SKY_SMOOTH = 31        # sliding-MEDIAN window over sky[x]; <=1 disables

USE_GROUND_CONTACT = True    # reject blocks that aren't standing on the mat
GROUND_PROBE_DY = 12         # probe strip this far below a block's bounding box
GROUND_CONTACT_MIN = 0.5     # >= this fraction of the strip must be floor

ARENA_BAND_H = ARENA_Y_BOTTOM - ARENA_Y_TOP
ARENA_SEED_LOCAL = (float(ARENA_SEED_PT[0]), float(ARENA_SEED_PT[1] - ARENA_Y_TOP))
_ARENA_ROWS = np.arange(ARENA_BAND_H, dtype=np.int32)[:, None]
_ARENA_WALL_KERNEL = np.ones((MIN_WALL_THICK, 1), np.uint8)   # vertical, for the wall scan
_ARENA_WALL_CLOSE_KERNEL = np.ones((WALL_GAP_SEAL, 1), np.uint8)
ARENA_PASSTHROUGH = np.full((FRAME_HEIGHT, FRAME_WIDTH), 255, dtype="uint8")


def _sliding_median(arr, window):
    """1-D sliding median over the skyline.

    This MUST be a median, not a minimum. A minimum filter can only ever expand the
    accepted region, and it propagates the single most-leaked column across its whole
    window -- one column where the floor blob escapes above the wall drags `window`
    neighbours to the top of the band with it, which collapses the whole mask into a
    rectangle. A median rejects isolated bad columns instead of spreading them.
    """
    if window <= 1:
        return arr
    pad = window // 2
    padded = np.pad(arr, pad, mode='edge')
    win = np.lib.stride_tricks.sliding_window_view(padded, window)
    return np.median(win, axis=-1).astype(np.int32)


def build_arena_mask(frame):
    """
    Returns (arena_mask, floor_mask, sky), or (None, None, None) if the seed point
    isn't on any floor blob (camera covered, nose into a wall) -- caller falls back
    to pass-through so a bad frame degrades to main_v2 behaviour instead of blanking
    every detection.

    arena_mask : full-frame uint8. Inside the band it is the solid skyline region.
                 ABOVE the band it is 0 -- ARENA_Y_TOP is a hard clamp, nothing up
                 there is ever arena. BELOW the band it is 255 (pass-through) so the
                 wall ROIs, which run to y=280, are never clipped.
    floor_mask : full-frame uint8, the drivable mat with interior holes PRESERVED.
                 Used only by the ground-contact test -- the solid arena fill would
                 make that test pass trivially.
    sky        : per-column top boundary, band-local. Annotation only.
    """
    band = frame[ARENA_Y_TOP:ARENA_Y_BOTTOM, :]
    hsv = cv2.cvtColor(band, cv2.COLOR_BGR2HSV)

    # 1. Floor candidate. Black walls are the only thing excluded -- pillars, floor
    #    lines and magenta parking walls all pass and stay part of the blob.
    mask_black = cv2.inRange(hsv, LOWER_BLACK, UPPER_BLACK)
    floor_cand = cv2.bitwise_not(mask_black)
    # Closing seals seams/glare/cable shadows in the mat. Careful raising this: a
    # kernel wide enough to bridge the arena wall merges the far section into our
    # blob and silently defeats the whole mechanism.
    floor_cand = cv2.morphologyEx(floor_cand, cv2.MORPH_CLOSE, ARENA_CLOSE_KERNEL)

    # 2. Keep only the blob reachable from the robot's nose, then take the largest.
    contours, _ = cv2.findContours(floor_cand, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    seeded = [c for c in contours if cv2.pointPolygonTest(c, ARENA_SEED_LOCAL, False) >= 0]
    if not seeded:
        return None, None, None
    cnt = max(seeded, key=cv2.contourArea)

    filled = np.zeros_like(floor_cand)
    cv2.drawContours(filled, [cnt], -1, 255, -1)
    floor = cv2.bitwise_and(floor_cand, filled)   # fill-then-AND: interior holes come back

    # 3. Skyline, by PER-COLUMN WALL SCAN.
    #
    #    Deriving the top boundary from the floor blob's upper edge does not work.
    #    The room beyond the arena is bright, so it passes NOT(black) too, and the
    #    wall is a thin barrier -- wherever it thins out or leaves frame, the blob
    #    escapes into the room and floor_top collapses to 0 for that column. Verified
    #    on real frames: the mask degenerated into a full-band rectangle.
    #
    #    Instead, scan each column upward from the bottom for the first SUSTAINED
    #    black run -- that is the near wall -- and take the top of that run. This is
    #    purely local per column, so no amount of sideways leakage can affect it, and
    #    anything above the near wall (far track section, room, shoes) is excluded by
    #    construction.
    #
    #    The vertical MORPH_OPEN keeps only black runs >= MIN_WALL_THICK px tall, so
    #    speckle, floor-line shadows and pillar edges can't be mistaken for a wall.
    #    Seal small BRIGHT breaks inside the wall first. The upward walk below needs
    #    a strictly contiguous black run, so a single bright pixel -- glare, a mat
    #    line reflecting off the wall, sensor noise, a JPEG artifact -- stops it dead.
    #    Different columns break at different heights, which made the top boundary
    #    visibly jagged while the bottom edge stayed smooth, and left wall_top a
    #    median of 12px BELOW the topmost black pixel in the same column.
    wall_black = cv2.morphologyEx(mask_black, cv2.MORPH_CLOSE, _ARENA_WALL_CLOSE_KERNEL)
    thick = cv2.morphologyEx(wall_black, cv2.MORPH_OPEN, _ARENA_WALL_KERNEL)
    thick_b = thick > 0
    has = thick_b.any(axis=0)

    # lowest row of a wall run in each column, then walk up to the top of that run
    lowest = (ARENA_BAND_H - 1) - np.argmax(thick_b[::-1], axis=0)
    chain = thick_b | (_ARENA_ROWS > lowest)     # force everything below `lowest` True
    run = np.minimum.accumulate(chain[::-1], axis=0)[::-1]
    wall_top = ARENA_BAND_H - run.sum(axis=0)

    # A run longer than MAX_WALL_RUN means the wall has probably merged into a dark
    # background above it, so we can't locate its top edge in this column.
    #
    # Do NOT clamp to `lowest - MAX_WALL_RUN`: that lands the boundary in the MIDDLE
    # of the wall, which is the worst of both worlds -- it neither includes the wall
    # nor excludes what's beyond it -- and it makes the skyline track the wall's
    # ragged BOTTOM edge, which is where the jagged mid-wall boundary came from.
    # Mark the column untrusted instead and let neighbours supply the value.
    trusted = has & ((lowest - wall_top) <= MAX_WALL_RUN)
    fill = int(np.median(wall_top[trusted])) if trusted.any() else 0
    sky = np.where(trusted, wall_top, fill).astype(np.int32)
    sky = sky + ARENA_TOP_MARGIN
    sky = _sliding_median(sky, ARENA_SKY_SMOOTH)
    sky = np.clip(sky, 0, ARENA_BAND_H)

    band_mask = ((_ARENA_ROWS >= sky).astype(np.uint8)) * 255

    arena_mask = np.zeros((FRAME_HEIGHT, FRAME_WIDTH), dtype="uint8")
    arena_mask[ARENA_Y_TOP:ARENA_Y_BOTTOM, :] = band_mask
    arena_mask[ARENA_Y_BOTTOM:, :] = 255   # below the band: pass-through, never clip

    # Bound the floor blob by the arena region too -- if it did leak into the room,
    # those pixels must not be able to validate a block in the ground-contact test.
    floor_mask = np.zeros((FRAME_HEIGHT, FRAME_WIDTH), dtype="uint8")
    floor_mask[ARENA_Y_TOP:ARENA_Y_BOTTOM, :] = cv2.bitwise_and(floor, band_mask)

    return arena_mask, floor_mask, sky

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
            return None

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
                print(f"VideoWriterThread: ERROR writing frame")
                traceback.print_exc()
                continue
        self.out.release()

    def write(self, frame):
        if not self.stop_event.is_set():
            self.queue.put(frame)

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
        # Channels are the sensors' XSHUT GPIOs: 17=front, 16=back (both wired),
        # 22=left, 27=right (not wired yet — get_distance() returns None for them
        # until they are, so polling them is harmless).
        self.channels = [
            distance.FRONT_CHANNEL,
            distance.BACK_CHANNEL,
            distance.LEFT_CHANNEL,
            distance.RIGHT_CHANNEL,
        ]
        self.history = {ch: deque(maxlen=5) for ch in self.channels}

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

            consecutive_none = {ch: 0 for ch in self.channels}
            reinit_threshold = 30
            while not self.stop_event.is_set():
                try:
                    readings = {}
                    for ch in list(consecutive_none.keys()):
                        val = self.dist.get_distance(ch)
                        if val is not None and val < 50.0:
                            val = None
                        with self.lock:
                            if val is not None:
                                self.history[ch].append(val)
                                consecutive_none[ch] = 0
                            else:
                                consecutive_none[ch] = consecutive_none.get(ch, 0) + 1
                            
                            if self.history[ch]:
                                sorted_vals = sorted(self.history[ch])
                                if len(sorted_vals) >= 3:
                                    readings[ch] = sum(sorted_vals[1:-1]) / (len(sorted_vals) - 2)
                                else:
                                    readings[ch] = sum(sorted_vals) / len(sorted_vals)
                            else:
                                readings[ch] = None

                    with self.lock:
                        self.distance_center = readings.get(distance.FRONT_CHANNEL)
                        self.distance_back = readings.get(distance.BACK_CHANNEL)
                        self.distance_left = readings.get(distance.LEFT_CHANNEL)
                        self.distance_right = readings.get(distance.RIGHT_CHANNEL)

                    for ch, count in list(consecutive_none.items()):
                        if count >= reinit_threshold:
                            #ok = distance.reinit_sensor(ch)
                            #print(f"DEBUG: Re-init returned: {ok}")
                            #consecutive_none[ch] = 0 if ok else count
                            pass

                    time.sleep(0.02) # account for timing budget 
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

    def get_history(self, ch):
        with self.lock:
            return list(self.history[ch])

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
    GLOBAL_Y_OFFSET = min(
        left_roi_y, right_roi_y, inner_left_roi_y, inner_right_roi_y,
        line_roi_y, close_y, full_frame_roi[1], close_block_roi[1]
    )
    GLOBAL_Y_END = max(
        left_roi_y + left_roi_h, right_roi_y + right_roi_h,
        inner_left_roi_y + inner_left_roi_h, inner_right_roi_y + inner_right_roi_h,
        line_roi_y + line_roi_h, close_y + close_h,
        full_frame_roi[1] + full_frame_roi[3], close_block_roi[1] + close_block_roi[3]
    )
    SLICE_HEIGHT = GLOBAL_Y_END - GLOBAL_Y_OFFSET

    # Slice the frame first
    frame_slice = frame[GLOBAL_Y_OFFSET:GLOBAL_Y_END, :]
    frame_slice = cv2.GaussianBlur(frame_slice, (1, 7), 0)
    
    if USE_LAB:
        hsv_slice = cv2.cvtColor(frame_slice, cv2.COLOR_BGR2Lab)
    else:
        hsv_slice = cv2.cvtColor(frame_slice, cv2.COLOR_BGR2HSV)

    # --- 0. Arena mask (v5) ---
    # Rebuilt every frame. Gates the colour masks BEFORE findContours, so nothing
    # outside the arena can ever become a contour -- and therefore can never win the
    # max(contours, key=contourArea) pick inside process_block_contours().
    floor_mask = None
    arena_sky = None
    arena_mask = ARENA_PASSTHROUGH
    if USE_ARENA_MASK:
        arena_mask, floor_mask, arena_sky = build_arena_mask(frame)
        if arena_mask is None:
            arena_mask = ARENA_PASSTHROUGH
            floor_mask = None
    processed_data['arena_mask'] = arena_mask
    processed_data['arena_sky'] = arena_sky
    processed_data['arena_seeded'] = floor_mask is not None

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

    # Gate to the arena. full_frame_roi spans rows 80-250, inside the arena band.
    arena_main = arena_mask[my:my + mh, mx:mx + mw]
    mask_red_main = cv2.bitwise_and(mask_red_main, arena_main)
    mask_green_main = cv2.bitwise_and(mask_green_main, arena_main)
    mask_magenta_main = cv2.bitwise_and(mask_magenta_main, arena_main)

    # Line ROI (Orange, Blue)
    # line_roi_y = 200 -> y=200 in global is y=100 in slice
    lx, ly, lw, lh = line_roi_x, line_roi_y, line_roi_w, line_roi_h
    ly_slice = ly - GLOBAL_Y_OFFSET
    
    line_crop = hsv_slice[ly_slice:ly_slice+lh, lx:lx+lw]
    
    mask_orange_line = cv2.inRange(line_crop, LOWER_ORANGE, UPPER_ORANGE)
    mask_blue_line = cv2.inRange(line_crop, LOWER_BLUE, UPPER_BLUE)

    # line_roi spans rows 190-230, inside the arena band.
    arena_line = arena_mask[ly:ly + lh, lx:lx + lw]
    mask_orange_line = cv2.bitwise_and(mask_orange_line, arena_line)
    mask_blue_line = cv2.bitwise_and(mask_blue_line, arena_line)

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

    # close_block_roi spans rows 215-225, inside the arena band.
    arena_close = arena_mask[cy:cy + ch, cx:cx + cw]
    mask_red_close = cv2.bitwise_and(mask_red_close, arena_close)
    mask_green_close = cv2.bitwise_and(mask_green_close, arena_close)
    mask_magenta_close = cv2.bitwise_and(mask_magenta_close, arena_close)

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
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > MAGENTA_MIN_AREA:
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"]) + mx
                    cy = int(M["m01"] / M["m00"]) + (GLOBAL_Y_OFFSET + my_slice)
                    contour_global = contour + [mx, GLOBAL_Y_OFFSET + my_slice]

                    leftmost_x = contour_global[:, 0, 0].min()
                    rightmost_x = contour_global[:, 0, 0].max()

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
                        'contour': contour_global,
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

                        # Ground-contact test. The arena mask is a SOLID fill that
                        # includes the wall band, so a red logo on the wall or a
                        # pillar in the far section poking into that band still
                        # passes it. This asks the orthogonal question: is the thing
                        # standing on the mat? Real pillars have floor beneath them;
                        # anything resting on a wall has black beneath it.
                        # Skipped for 'close_block' -- that 10px strip is an
                        # emergency reverse-and-swerve reflex where a false negative
                        # is worse than a false positive.
                        if USE_GROUND_CONTACT and floor_mask is not None and b_type == 'block':
                            rx, ry, rw, rh = cv2.boundingRect(biggest_contour_global)
                            probe_y = ry + rh + GROUND_PROBE_DY
                            if probe_y < ARENA_Y_BOTTOM:
                                strip = floor_mask[probe_y, rx:rx + rw]
                                if strip.size and (np.count_nonzero(strip) / strip.size) < GROUND_CONTACT_MIN:
                                    return None

                        return {'type': b_type, 'color': b_color, 'area': area, 'centroid': (cx, cy), 'contour': biggest_contour_global}
        return None

    all_detected_blocks = []
    
    # Main Red
    res = process_block_contours(mask_red_main, mx, GLOBAL_Y_OFFSET + my_slice, 'block', 'red', BLOCK_MIN_AREA)
    if res: all_detected_blocks.append(res)
    
    # Main Green
    res = process_block_contours(mask_green_main, mx, GLOBAL_Y_OFFSET + my_slice, 'block', 'green', BLOCK_MIN_AREA)
    if res: all_detected_blocks.append(res)

    # Close Red
    res = process_block_contours(mask_red_close, cx, GLOBAL_Y_OFFSET + cy_slice, 'close_block', 'red', CLOSE_BLOCK_MIN_AREA)
    if res: all_detected_blocks.append(res)

    # Close Green
    res = process_block_contours(mask_green_close, cx, GLOBAL_Y_OFFSET + cy_slice, 'close_block', 'green', CLOSE_BLOCK_MIN_AREA)
    if res: all_detected_blocks.append(res)

    # Close Magenta
    res = process_block_contours(mask_magenta_close, cx, GLOBAL_Y_OFFSET + cy_slice, 'close_block', 'magenta', CLOSE_BLOCK_MIN_AREA)
    if res: all_detected_blocks.append(res)

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
    
    # Wall detection in line ROI (detecting black wall to prevent crash)
    wall_line_crop = pure_black_mask[ly_slice:ly_slice+lh, lx:lx+lw]
    wall_pixels = cv2.countNonZero(wall_line_crop)
    total_roi_pixels = lw * lh
    line_roi_wall_pct = (wall_pixels / total_roi_pixels) * 100.0 if total_roi_pixels > 0 else 0.0

    processed_data['line_roi_wall_pct'] = line_roi_wall_pct
    processed_data['detected_line_roi_wall'] = []

    if cv2.countNonZero(wall_line_crop) > 0:
        contours, _ = cv2.findContours(wall_line_crop, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            contour_global = contour + [lx, ly]
            processed_data['detected_line_roi_wall'].append({
                'contour': contour_global
            })
    
    return processed_data

def annotate_video_frame(frame, detections, driving_direction, debug_info="", visual_target_x=None, visual_target_line=None):
    annotated_frame = frame.copy()
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

    # --- Arena mask overlay (v5) ---
    arena_mask = detections.get('arena_mask')
    if arena_mask is not None and USE_ARENA_MASK:
        # band limits + chassis rect, as static reference
        cv2.line(annotated_frame, (0, ARENA_Y_TOP), (FRAME_WIDTH, ARENA_Y_TOP), (90, 90, 90), 1)
        cv2.line(annotated_frame, (0, ARENA_Y_BOTTOM), (FRAME_WIDTH, ARENA_Y_BOTTOM), (90, 90, 90), 1)
        cv2.rectangle(annotated_frame, (180, 250), (460, 359), (90, 90, 90), 1)

        # accepted region outline
        band = arena_mask[ARENA_Y_TOP:ARENA_Y_BOTTOM, :]
        arena_cnts, _ = cv2.findContours(band, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in arena_cnts:
            cv2.drawContours(annotated_frame, [c + [0, ARENA_Y_TOP]], -1, (0, 255, 255), 1)

        # skyline: the per-column top boundary. Tune ARENA_TOP_MARGIN against this.
        arena_sky = detections.get('arena_sky')
        if arena_sky is not None:
            pts = np.stack([np.arange(FRAME_WIDTH, dtype=np.int32),
                            arena_sky.astype(np.int32) + ARENA_Y_TOP], axis=1)
            cv2.polylines(annotated_frame, [pts], False, (0, 200, 255), 1)

        # seed point -- green if it landed on a floor blob, red if we fell back to
        # pass-through. First thing to check when the mask misbehaves.
        seeded = detections.get('arena_seeded', False)
        cv2.circle(annotated_frame, ARENA_SEED_PT, 4, (0, 255, 0) if seeded else (0, 0, 255), -1)

    if detections.get('line_roi_wall_pct', 0) > 50:
        for black_obj in detections.get('detected_line_roi_wall', []):
            cv2.drawContours(annotated_frame, [black_obj['contour']], -1, (0, 0, 0), 2)

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

def drive_straight_with_gyro(target_heading, duration, speed, direction='forward', kp=GYRO_KP, kd=GYRO_KD):
    print(f"Driving {direction} with gyro stabilization for {duration}s...")
    
    start_time = time.monotonic()
    
    if direction == 'forward':
        motor.forward(speed)
    else:
        motor.reverse(speed)

    prev_error = 0.0

    while time.monotonic() - start_time < duration:
        current_heading = imu_thread.get_heading()
        if current_heading is None:
            time.sleep(0.01)
            continue

        error = target_heading - current_heading
        
        while error <= -180: error += 360
        while error > 180: error -= 360

        derivative = error - prev_error
        steer_angle = (kp * error) + (kd * derivative)
        prev_error = error
        
        steer_angle = np.clip(steer_angle, -45, 45)

        servo.set_angle(steer_angle)
        time.sleep(0.01)

    motor.brake()
    servo.set_angle(0)
    print("Gyro-stabilized drive complete.")

def steer_with_gyro(current_heading: float, 
                    target_heading: float, 
                    kp: float = GYRO_KP, 
                    kd: float = GYRO_KD,
                    prev_error: float = 0.0,
                    min_servo_angle: int = -45, 
                    max_servo_angle: int = 45) -> float:
    """
    Calculates the servo steering angle to maintain a target heading using a gyro PD controller.

    This function uses a Proportional-Derivative (PD) controller to correct the robot's
    trajectory. It correctly handles the "wrap-around" issue where headings
    jump between 359 and 0 degrees.

    Args:
        current_heading (float): The current direction of the robot in degrees (0-359).
        target_heading (float): The desired direction in degrees (0-359).
        kp (float): The proportional gain constant.
        kd (float): The derivative gain constant.
        prev_error (float): The previous heading error.
        min_servo_angle (int): The minimum allowable angle for the servo (e.g., full left).
        max_servo_angle (int): The maximum allowable angle for the servo (e.g., full right).

    Returns:
        float: The calculated servo angle, clamped within the min/max limits.
    """
    if current_heading is None or target_heading is None:
        return 0.0

    # 1. Calculate the error between target and current heading.
    error = target_heading - current_heading

    # 2. Handle the wrap-around issue (e.g., turning from 350 degrees to 10 degrees).
    # The shortest path is not -340 degrees, but +20 degrees.
    if error > 180:
        error -= 360
    elif error < -180:
        error += 360

    # 3. Calculate derivative and steering correction (PD controller).
    derivative = error - prev_error
    steer_angle = (kp * error) + (kd * derivative)

    # 4. Clamp the steering angle to the servo's physical limits.
    clamped_steer_angle = np.clip(steer_angle, min_servo_angle, max_servo_angle)

    return clamped_steer_angle

def perform_initial_maneuver():
    print("--- Executing Full Initial Maneuver ---")

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
    print(f"Direction: {driving_direction.upper()}")
    print(f"Initial Heading: {INITIAL_HEADING:.1f}°")
    print(f"Scan Will Trigger At: {scan_heading:.1f}°")
    print(f"Full Turn Target: {ninety_degree_heading:.1f}°")

    servo.set_angle_unlimited(initial_turn_servo)
    time.sleep(0.1)
    motor.forward(MANEUVER_SPEED)
    
    print("Starting initial turn...")
    print(imu_thread.get_heading(), get_angular_difference((INITIAL_HEADING+SCAN_TRIGGER_ANGLE_DEG)%360, imu_thread.get_heading()))
    while get_angular_difference((INITIAL_HEADING+SCAN_TRIGGER_ANGLE_DEG)%360, imu_thread.get_heading()) > 10:
        print(imu_thread.get_heading(), get_angular_difference((INITIAL_HEADING+SCAN_TRIGGER_ANGLE_DEG)%360, imu_thread.get_heading()))
        time.sleep(0.01)
        pass
    motor.brake()
    print(f"Scan angle reached. Pausing to scan for objects...")
    detected_block_color = None
    scan_start_time = time.monotonic()
    while time.monotonic() - scan_start_time < 1.0:
        frame, frame_counter = camera_thread.get_frame()
        if frame is None: continue

        detections = process_video_frame(frame)
        annotated_frame = annotate_video_frame(frame, detections, driving_direction)
        try:
            video_writer_thread.write(annotated_frame)
        except Exception as e:
            print(e)
        main_blocks = [b for b in detections.get('detected_blocks', []) if b['type'] == 'block']
        
        if main_blocks:
            if main_blocks[0]['area']>1000 and main_blocks[0]['centroid'][0]<540:
                detected_block_color = main_blocks[0]['color']
                print(f"Block Found! Color: {detected_block_color.upper()}. Ending scan.")
                break
    
    if detected_block_color is None:
        print("Scan complete. No block was detected.")

    print("90-degree turn complete. Performing action based on scan...")
    time.sleep(0.5)

    action_taken = "None"
    drive_target_heading = ninety_degree_heading

    if driving_direction == 'counter-clockwise':
        if detected_block_color == 'green':
            motor.forward(50)
            while get_angular_difference((INITIAL_HEADING - 100) % 360, imu_thread.get_heading()) > 5:
                servo.set_angle(steer_with_gyro(imu_thread.get_heading(),(INITIAL_HEADING - 100) % 360))
            action_taken = f"DRIVE_FORWARD_GREEN for {0.05}s"
            drive_straight_with_gyro((INITIAL_HEADING - 100) % 360, 0.05, 50, 'forward')
        elif detected_block_color == 'red':
            motor.forward(50)
            while get_angular_difference(ninety_degree_heading, imu_thread.get_heading()) > 5:
                servo.set_angle(steer_with_gyro(imu_thread.get_heading(),ninety_degree_heading))
            action_taken = f"REVERSE_BEFORE_TURN for {0.9}s"
            drive_straight_with_gyro(drive_target_heading, 0.9, 50, 'reverse')
        else:
            drive_straight_with_gyro((INITIAL_HEADING-55)%360, 0.2, 70, 'forward')
            return
            
    else:
        if detected_block_color == 'green':
            motor.forward(50)
            while get_angular_difference(ninety_degree_heading, imu_thread.get_heading()) > 5:
                servo.set_angle(steer_with_gyro(imu_thread.get_heading(),ninety_degree_heading))
            action_taken = f"REVERSE_FOR_GREEN_CW for {0.8}s"
            drive_straight_with_gyro(drive_target_heading, 0.8, 50, 'reverse')
        elif detected_block_color == 'red':
            motor.forward(50)
            while get_angular_difference(ninety_degree_heading, imu_thread.get_heading()) > 5:
                servo.set_angle(steer_with_gyro(imu_thread.get_heading(),ninety_degree_heading))
            action_taken = f"DRIVE_FORWARD_RED_CW for {0.3}s"
            drive_straight_with_gyro(drive_target_heading, 0.3, 50, 'forward')
        else:
            drive_straight_with_gyro((INITIAL_HEADING+55)%360, 0.5, 70, 'forward')
            return

    motor.brake()
    print(f"Action complete: {action_taken}")
    time.sleep(0.5)

    print(f"Performing final turn to return to {INITIAL_HEADING:.1f}°...")
    motor.forward(50)
    #servo.set_angle(final_turn_servo)

    while get_angular_difference(imu_thread.get_heading(), INITIAL_HEADING) > 15:
        servo.set_angle(steer_with_gyro(imu_thread.get_heading(),INITIAL_HEADING,2))
    motor.brake()
    servo.set_angle(0)
    motor.reverse(45)
    start_time = time.monotonic()
    while time.monotonic() - start_time < 0.3:
        servo.set_angle(-steer_with_gyro(imu_thread.get_heading(),INITIAL_HEADING,1))
    time.sleep(0.5)
    motor.brake()
    print("--- Initial Maneuver Complete. Transitioning to straight driving. ---")

def parking():
    fn_start = time.monotonic()
    print("--- Parking (clockwise) sequence started ---")
    servo.set_angle(3)
    motor.move(14, min_speed=45)
    motor.brake()
    servo.set_angle_unlimited(-60)
    motor.stop_rpm_control()
    motor.start_rpm_control(80, "reverse")
    parking_start = time.monotonic()
    while True:
        sensor_readings = sensor_thread.get_readings()
        heading = imu_thread.get_heading()
        print(f"Parking reverse distance_back: {sensor_readings['distance_back']} (history: {sensor_thread.get_history(distance.BACK_CHANNEL)}), heading: {heading}")
        target_turn_heading = (INITIAL_HEADING + 90) % 360
        has_turned = get_angular_difference(target_turn_heading, heading) < 15
        if sensor_readings['distance_back'] is not None:
            if has_turned and sensor_readings['distance_back'] < 130:
                break
        if time.monotonic() - parking_start > 6.0:
            print("WARNING: Parking reverse timeout reached!")
            break
        servo.set_angle_unlimited(-steer_with_gyro(heading,(INITIAL_HEADING+90)%360, kp=1, min_servo_angle=-60, max_servo_angle=60))
        time.sleep(0.01)

    motor.stop_rpm_control()
    time.sleep(0.3)
    motor.start_rpm_control(40, "forward")
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
    servo.set_angle(0)
    time.sleep(0.62) # INCREASING MAKES ROBOT STOP MORE FORWARD
    motor.stop_rpm_control()
    time.sleep(0.3)
    motor.start_rpm_control(40, "reverse")
    servo.set_angle_unlimited(55)
    while get_angular_difference((INITIAL_HEADING+100)%360, imu_thread.get_heading()) > 10:
        time.sleep(0.01)
    motor.stop_rpm_control()
    print('parking first reverse turn:',sensor_thread.get_readings())
    motor.start_rpm_control(40, "reverse")
    servo.set_angle(0)
    print('reverse')
    parking_start = time.monotonic()
    while True:
        dist = sensor_thread.get_readings()['distance_back']
        print('Reverse for parking back distance:', dist, f'(history: {sensor_thread.get_history(distance.BACK_CHANNEL)})')
        if dist is not None and dist < 160:
            break
        if time.monotonic() - parking_start > 6.0:
            print("WARNING: Parking second reverse timeout reached!")
            break
        time.sleep(0.01)
    print('parking forward:',sensor_thread.get_readings())
    motor.stop_rpm_control()
    time.sleep(0.3)
    motor.start_rpm_control(40, "forward")
    time.sleep(0.1)
    parking_start = time.monotonic()
    while True:
        sensor_readings = sensor_thread.get_readings()
        dist = sensor_readings['distance_back']
        heading = imu_thread.get_heading()
        print('Forward for parking back distance:', dist, f'(history: {sensor_thread.get_history(distance.BACK_CHANNEL)})', 'heading:', heading)
        if dist is not None and dist > 200:
            break
        if time.monotonic() - parking_start > 5.0:
            print("WARNING: Parking forward drive timeout reached!")
            break
        servo.set_angle(steer_with_gyro(heading, (INITIAL_HEADING + 95) % 360, kp=1.0))
        time.sleep(0.01)
    motor.stop_rpm_control()
    time.sleep(0.3)
    motor.start_rpm_control(40, "reverse")
    servo.set_angle_unlimited(-65)
    manuver_start_time = time.monotonic()
    while True:
        dist = sensor_thread.get_readings()['distance_back']
        print('Reverse 2 for parking back distance:', dist, f'(history: {sensor_thread.get_history(distance.BACK_CHANNEL)})')
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
    while True:
        dist_center = sensor_thread.get_readings()['distance_center']
        print('Forward alignment center distance:', dist_center, f'(history: {sensor_thread.get_history(distance.FRONT_CHANNEL)})')
        if dist_center is not None and dist_center < 135:
            break
        if get_angular_difference(imu_thread.get_heading(), (INITIAL_HEADING+180)%360) < 2:
            break
        servo.set_angle(steer_with_gyro(imu_thread.get_heading(),(INITIAL_HEADING+183)%360, kp=1.5))
        time.sleep(0.01)
    motor.stop_rpm_control()
    time.sleep(0.3)

    motor.start_rpm_control(40, "reverse")
    while True:
        dist = sensor_thread.get_readings()['distance_back']
        print('Final reverse back distance:', dist, f'(history: {sensor_thread.get_history(distance.BACK_CHANNEL)})')
        servo.set_angle(-steer_with_gyro(imu_thread.get_heading(),(INITIAL_HEADING+180)%360, kp=1.5))
        if dist is not None:
            if dist <= 95:
                break
        if get_angular_difference((INITIAL_HEADING+183)%360, imu_thread.get_heading()) < 2:
            break
        time.sleep(0.01)
    motor.stop_rpm_control()
    fn_end = time.monotonic()
    print(f"--- parking() completed in {fn_end - fn_start:.2f}s ---")
    
def parking2():
    fn_start = time.monotonic()
    print("--- Parking2 (counter-clockwise) sequence started ---")
    print(sensor_thread.get_readings()['distance_center'])
    motor.stop_rpm_control()
    
    t_step = time.monotonic()
    print("[Parking2 Step 1/10] Starting forward approach (200 RPM)...")
    motor.start_rpm_control(200, "forward")
    while True:
        sensor_readings = sensor_thread.get_readings()
        distance_center = sensor_readings.get('distance_center')
        print(f"Parking2 forward distance_center: {distance_center} (history: {sensor_thread.get_history(distance.FRONT_CHANNEL)}), heading: {imu_thread.get_heading()}")
        if distance_center is not None and distance_center <= 200:
            print(f"Distance is {distance_center}. Exiting loop.")
            break
        servo.set_angle(steer_with_gyro(imu_thread.get_heading(), (INITIAL_HEADING+5) % 360, kp=1))
        time.sleep(0.01)
    motor.stop_rpm_control()
    print(f"[Parking2 Step 1/10] Completed in {time.monotonic() - t_step:.2f}s")
    time.sleep(0.3)
    
    t_step = time.monotonic()
    print("[Parking2 Step 2/10] Starting reverse 90 turn (200 RPM)...")
    servo.set_angle_unlimited(60)
    motor.start_rpm_control(200, "reverse")
    parking_start = time.monotonic()
    while True:
        sensor_readings = sensor_thread.get_readings()
        heading = imu_thread.get_heading()
        print(f"Parking2 reverse distance_back: {sensor_readings['distance_back']} (history: {sensor_thread.get_history(distance.BACK_CHANNEL)}), heading: {heading}")
        target_turn_heading = (INITIAL_HEADING - 90) % 360
        has_turned = get_angular_difference(target_turn_heading, heading) < 15
        if sensor_readings['distance_back'] is not None:
            if has_turned and sensor_readings['distance_back'] < 155:
                break
        if time.monotonic() - parking_start > 6.0:
            print("WARNING: Parking2 reverse timeout reached!")
            break
        servo.set_angle_unlimited(-steer_with_gyro(heading, (INITIAL_HEADING-90)%360, kp=1, min_servo_angle=-60, max_servo_angle=60))
        time.sleep(0.01)
    
    motor.stop_rpm_control()
    print(f"[Parking2 Step 2/10] Completed in {time.monotonic() - t_step:.2f}s")
    time.sleep(0.3)
    
    t_step = time.monotonic()
    print("[Parking2 Step 3/10] Realigning heading (150 RPM)...")
    motor.start_rpm_control(150, "forward")
    while get_angular_difference((INITIAL_HEADING-180)%360, imu_thread.get_heading()) > 7:
        heading = imu_thread.get_heading()
        servo.set_angle(steer_with_gyro(heading, (INITIAL_HEADING-180)%360, kp=1, min_servo_angle=-40, max_servo_angle=40))
        time.sleep(0.01)
    motor.stop_rpm_control()
    print(f"[Parking2 Step 3/10] Completed in {time.monotonic() - t_step:.2f}s")
    servo.set_angle(0)
    
    ROI_Y_START = 140
    ROI_X_END = 40
    TARGET_Y_OFFSET_FROM_BOTTOM = 180

    MAGENTA_HIGH_THRESHOLD = 500
    MAGENTA_LOW_THRESHOLD = 200

    first_magenta_line_passed = False
    on_first_line = False

    t_step = time.monotonic()
    print("[Parking2 Step 4/10] Tracking line to magenta stop (200 RPM)...")
    motor.start_rpm_control(200, "forward")
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
    time.sleep(0.25)
    motor.stop_rpm_control()
    print(f"[Parking2 Step 4/10] Completed in {time.monotonic() - t_step:.2f}s")
    time.sleep(0.3)
    
    t_step = time.monotonic()
    print("[Parking2 Step 5/10] Reverse turn to 100 deg (70 RPM)...")
    motor.start_rpm_control(70, "reverse")
    servo.set_angle_unlimited(-55)
    while get_angular_difference((INITIAL_HEADING-100)%360, imu_thread.get_heading()) > 10:
        time.sleep(0.01)
    motor.stop_rpm_control()
    print(f"[Parking2 Step 5/10] Completed in {time.monotonic() - t_step:.2f}s")
    
    t_step = time.monotonic()
    print("[Parking2 Step 6/10] Reverse to back distance < 160 (120 RPM)...")
    print('parking first reverse turn:', sensor_thread.get_readings())
    motor.start_rpm_control(120, "reverse")
    servo.set_angle(0)
    print('reverse')
    parking_start = time.monotonic()
    while True:
        dist = sensor_thread.get_readings()['distance_back']
        print('Reverse for parking back distance:', dist, f'(history: {sensor_thread.get_history(distance.BACK_CHANNEL)})')
        if dist is not None and dist < 160:
            break
        if time.monotonic() - parking_start > 6.0:
            print("WARNING: Parking2 second reverse timeout reached!")
            break
        time.sleep(0.01)
        
    print('parking forward:', sensor_thread.get_readings())
    motor.stop_rpm_control()
    print(f"[Parking2 Step 6/10] Completed in {time.monotonic() - t_step:.2f}s")
    time.sleep(0.3)
    
    t_step = time.monotonic()
    print("[Parking2 Step 7/10] Forward drive back distance > 220 (120 RPM)...")
    motor.start_rpm_control(120, "forward")
    time.sleep(0.1)
    parking_start = time.monotonic()
    while True:
        sensor_readings = sensor_thread.get_readings()
        dist = sensor_readings['distance_back']
        heading = imu_thread.get_heading()
        print('Forward for parking back distance:', dist, f'(history: {sensor_thread.get_history(distance.BACK_CHANNEL)})', 'heading:', heading)
        if dist is not None and dist > 220:
            break
        if time.monotonic() - parking_start > 5.0:
            print("WARNING: Parking2 forward drive timeout reached!")
            break
        servo.set_angle(steer_with_gyro(heading, (INITIAL_HEADING - 85) % 360, kp=1.0))
        time.sleep(0.01)
        
    motor.stop_rpm_control()
    print(f"[Parking2 Step 7/10] Completed in {time.monotonic() - t_step:.2f}s")
    time.sleep(0.3)

    t_step = time.monotonic()
    print("[Parking2 Step 8/10] Reverse 2 to back distance <= 100 (100 RPM)...")
    motor.start_rpm_control(100, "reverse")
    servo.set_angle_unlimited(65)
    manuver_start_time = time.monotonic()
    while True:
        dist = sensor_thread.get_readings()['distance_back']
        print('Reverse 2 for parking back distance:', dist, f'(history: {sensor_thread.get_history(distance.BACK_CHANNEL)})')
        if dist is not None:
            if dist <= 120:
                break
        if get_angular_difference((INITIAL_HEADING-180)%360, imu_thread.get_heading()) < 2:
            break
        if time.monotonic() - manuver_start_time > 6:
            break
        time.sleep(0.01)
            
    motor.stop_rpm_control()
    print(f"[Parking2 Step 8/10] Completed in {time.monotonic() - t_step:.2f}s")
    time.sleep(0.3)

    t_step = time.monotonic()
    print("[Parking2 Step 9/10] Forward center alignment < 135 (60 RPM)...")
    motor.start_rpm_control(60, "forward")
    while True:
        dist_center = sensor_thread.get_readings()['distance_center']
        print('Forward alignment center distance:', dist_center, f'(history: {sensor_thread.get_history(distance.FRONT_CHANNEL)})')
        if dist_center is not None and dist_center < 135:
            break
        if get_angular_difference(imu_thread.get_heading(), (INITIAL_HEADING-180)%360) < 2:
            break
        servo.set_angle(steer_with_gyro(imu_thread.get_heading(), (INITIAL_HEADING-180)%360, kp=1.5))
        time.sleep(0.01)
        
    motor.stop_rpm_control()
    print(f"[Parking2 Step 9/10] Completed in {time.monotonic() - t_step:.2f}s")
    time.sleep(0.3)

    t_step = time.monotonic()
    print("[Parking2 Step 10/10] Final reverse docking (60 RPM)...")
    motor.start_rpm_control(60, "reverse")
    while True:
        dist = sensor_thread.get_readings()['distance_back']
        print('Final reverse back distance:', dist, f'(history: {sensor_thread.get_history(distance.BACK_CHANNEL)})')
        servo.set_angle(-steer_with_gyro(imu_thread.get_heading(), (INITIAL_HEADING-180)%360, kp=1.5))
        if dist is not None:
            if dist <= 105:
                break
        if get_angular_difference((INITIAL_HEADING-180)%360, imu_thread.get_heading()) < 2:
            break
        time.sleep(0.01)
    motor.stop_rpm_control()
    print(f"[Parking2 Step 10/10] Completed in {time.monotonic() - t_step:.2f}s")
    fn_end = time.monotonic()
    print(f"--- parking2() completed in {fn_end - fn_start:.2f}s ---")

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

    video_writer_thread = VideoWriterThread(video_path, fourcc, 10, (640, 360))
    video_writer_thread.start()
    
    camera.initialize()
    motor.initialize()
    servo.initialize()
    button = Button(config.BUTTON_PIN)
    led = LED(config.LED_PIN)
    
    orange_detection_history = deque([False] * ORANGE_DETECTION_HISTORY_LENGTH,maxlen=ORANGE_DETECTION_HISTORY_LENGTH)
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
    print("MainThread: Waiting for sensors to initialize...")
    sensors_initialized_event.wait() 
    print("MainThread: Sensors are ready.")    

    imu_initialized_event = threading.Event()
    imu_thread = ImuThread(bno086, imu_initialized_event)
    imu_thread.start()
    print("MainThread: Waiting for IMU to initialize...")    
    imu_initialized_event.wait()
    print("MainThread: IMU is ready. Proceeding with main logic.")    
    
    time.sleep(1)
    led.on()
    # button.wait_for_press()
    led.off()
    driving_direction = 'clockwise'
    past_frame_counter = 0
    for _ in range(10):
        frame, past_frame_counter = camera_thread.get_next_frame(past_frame_counter)
        if frame is not None:
            detections = process_video_frame(frame)
            wall_inner_left_size = sum(obj['area'] for obj in detections.get('detected_walls', []) if obj['type'] == 'wall_inner_left')
            wall_inner_right_size = sum(obj['area'] for obj in detections.get('detected_walls', []) if obj['type'] == 'wall_inner_right')
            print(f"Initial inner wall black areas - Left: {wall_inner_left_size}, Right: {wall_inner_right_size}")
            if wall_inner_left_size > wall_inner_right_size:
                driving_direction = 'clockwise'
            else:
                driving_direction = 'counter-clockwise'
            break
        time.sleep(0.05)
    print(f"Decided driving direction: {driving_direction}")
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
        past_frame_counter = 0
        frame_counter = 0
        # temporarily skip initial maneuver
        perform_initial_maneuver()
        print('out of parking')
        MIN_RPM = 300.0
        MAX_RPM = 1.10 * motor.MAX_WHEEL_RPM
        MAX_ACCEL_PER_FRAME = 10.0
        MAX_DECEL_PER_FRAME = 200.0
        INITIAL_RPM = 50.0

        motor.start_rpm_control(INITIAL_RPM, "forward")
        prev_rpm = INITIAL_RPM
        BLOCK_TARGET_GRACE_FRAMES = 5
        last_block_target_rpm = INITIAL_RPM
        frames_since_block_seen = BLOCK_TARGET_GRACE_FRAMES
        frame_start_time = time.perf_counter()
        print('before loop')
        while True:
            target_rpm = INITIAL_RPM
            angle = 0
            active_block_y = None
            debug = []
            visual_target_x = None
            visual_target_line = None
            frame, frame_counter = camera_thread.get_frame()
            if frame_counter == past_frame_counter:
                continue
            past_frame_counter = frame_counter
            if frame is None:
                continue
            sensor_readings = sensor_thread.get_readings()

            detections = process_video_frame(frame)
            if detections.get('detected_magenta'):
                if driving_direction == 'clockwise':
                    detections['detected_magenta'].sort(key=lambda x: x['centroid'][0])
                else:
                    detections['detected_magenta'].sort(key=lambda x: x['centroid'][0], reverse=True)
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
                turn_counter += 1
                cooldown_frames = ORANGE_COOLDOWN_FRAMES
                print("turn_counter ---------------->", turn_counter)

            if detected_blocks:
                prev_wall_error = 0.0
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
                        motor.stop_rpm_control()
                        servo.set_angle(angle)
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
                        active_block_y = block_y
                        frames_since_block_seen = 0

                        if block_color == 'red':
                            # TUNING PARAMETERS
                            RED_OTHER_X = 297
                            RED_OTHER_Y = 0
                            RED_ORIGIN_X = 0
                            RED_ORIGIN_Y = FRAME_HEIGHT
                            RED_IDEAL_ANGLE = math.degrees(math.atan2(RED_OTHER_X - RED_ORIGIN_X, RED_ORIGIN_Y - RED_OTHER_Y))
                            
                            wall_inner_left_size = sum(obj['area'] for obj in detected_walls if obj['type'] == 'wall_inner_left')
                            wall_inner_right_size = sum(obj['area'] for obj in detected_walls if obj['type'] == 'wall_inner_right')
                            target = 300 if block_y > 170 and 200 < block_x < 440 else 150
                            debug.append(target)
                            if detections['detected_magenta'] and driving_direction == 'counter-clockwise' and abs(detections['detected_magenta'][0]['target_y']-block_y)<70 and abs(detections['detected_magenta'][0]['centroid'][0]-block_x)>70:
                                target_x = detections['detected_magenta'][0]['target_x']
                                midpoint_x = (block_x + target_x) // 2
                                visual_target_x = midpoint_x
                                angle = ((midpoint_x - FRAME_MIDPOINT_X) * 0.20)
                            else:
                                visual_target_line = ((RED_ORIGIN_X, RED_ORIGIN_Y), (block_x, block_y), RED_IDEAL_ANGLE, (RED_OTHER_X, RED_OTHER_Y))
                                current_angle = math.degrees(math.atan2(block_x - RED_ORIGIN_X, RED_ORIGIN_Y - block_y))
                                angle = (current_angle - RED_IDEAL_ANGLE) * 1.5
                            if wall_inner_left_size > 3000: angle = np.clip(angle, 15, 45)
                            elif wall_inner_right_size > 3000: angle = np.clip(angle, -45, -10)
                            else: angle = np.clip(angle, -45, 35)
                        
                        elif block_color == 'green':
                            # TUNING PARAMETERS
                            GREEN_OTHER_X = 352
                            GREEN_OTHER_Y = 0
                            GREEN_ORIGIN_X = FRAME_WIDTH+20
                            GREEN_ORIGIN_Y = FRAME_HEIGHT
                            GREEN_IDEAL_ANGLE = math.degrees(math.atan2(GREEN_OTHER_X - GREEN_ORIGIN_X, GREEN_ORIGIN_Y - GREEN_OTHER_Y))

                            wall_inner_left_size = sum(obj['area'] for obj in detected_walls if obj['type'] == 'wall_inner_left')
                            wall_inner_right_size = sum(obj['area'] for obj in detected_walls if obj['type'] == 'wall_inner_right')
                            target = 300 if block_y > 160 and 240 < block_x < 400 else 150
                            if detections['detected_magenta'] and driving_direction == 'clockwise' and (block_x - detections['detected_magenta'][0]['centroid'][0]) >= 30:
                                target_x = detections['detected_magenta'][0]['target_x']
                                midpoint_x = (block_x + target_x) // 2
                                visual_target_x = midpoint_x
                                angle = ((midpoint_x - FRAME_MIDPOINT_X) * 0.35)
                            else:
                                visual_target_line = ((GREEN_ORIGIN_X, GREEN_ORIGIN_Y), (block_x, block_y), GREEN_IDEAL_ANGLE, (GREEN_OTHER_X, GREEN_OTHER_Y))
                                current_angle = math.degrees(math.atan2(block_x - GREEN_ORIGIN_X, GREEN_ORIGIN_Y - block_y))
                                angle = (current_angle - GREEN_IDEAL_ANGLE) * 1.5
                            if wall_inner_left_size > 3000: angle = np.clip(angle, 15, 45)
                            elif wall_inner_right_size > 3000: angle = np.clip(angle, -45, -10)
                            else: angle = np.clip(angle, -45, 45 )
            else:
                if frames_since_block_seen < BLOCK_TARGET_GRACE_FRAMES:
                    frames_since_block_seen += 1
                left_pixel_size,right_pixel_size,wall_inner_left_size,wall_inner_right_size,target=0,0,0,0,0
                left_pixel_size = sum(obj['area'] for obj in detected_walls if obj['type'] == 'wall_left')
                right_pixel_size = sum(obj['area'] for obj in detected_walls if obj['type'] == 'wall_right')
                wall_inner_left_size = sum(obj['area'] for obj in detected_walls if obj['type'] == 'wall_inner_left')
                wall_inner_right_size = sum(obj['area'] for obj in detected_walls if obj['type'] == 'wall_inner_right')
                if left_pixel_size<700 and (right_pixel_size + wall_inner_right_size)>100:
                    right_pixel_size *= 2
                    right_pixel_size += 25000
                elif right_pixel_size<700 and (left_pixel_size + wall_inner_left_size)>100:
                    left_pixel_size *= 2
                    left_pixel_size += 25000
                
                debug.extend([left_pixel_size, right_pixel_size])
                wall_error = (left_pixel_size + wall_inner_left_size) - (right_pixel_size + wall_inner_right_size)
                wall_derivative = wall_error - prev_wall_error
                angle = (wall_error * WALL_KP) + (wall_derivative * WALL_KD) + 1
                prev_wall_error = wall_error
                close_black_area = sum(obj['area'] for obj in detections.get('detected_close_black', []))
                if close_black_area > 3000 or detections.get('line_roi_wall_pct', 0) > 50:
                    if driving_direction == 'clockwise':
                        angle += 35
                    else:
                        angle += -35
                if left_pixel_size == 0 and right_pixel_size == 0 and(detected_orange_object or detected_blue_object):
                    if driving_direction == 'clockwise':
                        angle += 35
                    else:
                        angle += -35
            debug.append(round(angle))
            debug.append(turn_counter)
            debug.append(round(target_rpm))

            elapsed = time.perf_counter() - frame_start_time
            if elapsed < 1/60:
                time.sleep(1/60 - elapsed)
            frame_end_time = time.perf_counter()
            fps = 1/(frame_end_time - frame_start_time)
            frame_start_time = time.perf_counter()
            debug.append(round(fps))
            debug.append(frame_counter)
            annotated_frame = annotate_video_frame(frame, detections, driving_direction, debug_info=str(debug), visual_target_x=visual_target_x, visual_target_line=visual_target_line)
            
            try:
                video_writer_thread.write(annotated_frame)
            except Exception as e:
                print(e)
            angle = np.clip(angle, -40, 40)
            if angle != prevangle:
                servo.set_angle(angle)
            prevangle = angle

            # Dynamic speed calculation based on linear block height and steering angle
            a = min(1.0, abs(angle) / 40.0)
            f_steering = 1.0 - (0.5 * a)

            if active_block_y is not None:
                y_clamped = np.clip(active_block_y, 130, 150)
                h = 1.0 - (y_clamped - 130.0) / (150.0 - 130.0)
                f_height = h
                target_rpm = MIN_RPM + (MAX_RPM - MIN_RPM) * f_height * f_steering
                last_block_target_rpm = target_rpm
            elif frames_since_block_seen < BLOCK_TARGET_GRACE_FRAMES:
                target_rpm = last_block_target_rpm * f_steering
            else:
                target_rpm = MIN_RPM + (MAX_RPM - MIN_RPM) * f_steering

            # Apply rate limiter (+10 RPM accel, -50 RPM decel per frame)
            rpm_diff = target_rpm - prev_rpm
            rpm_diff = np.clip(rpm_diff, -MAX_DECEL_PER_FRAME, MAX_ACCEL_PER_FRAME)
            commanded_rpm = prev_rpm + rpm_diff

            if commanded_rpm != prev_rpm:
                motor.set_rpm_target(commanded_rpm, "forward")
            prev_rpm = commanded_rpm
            angle = 0
            
            if False:
                cv2.imshow("Robot Live Feed", annotated_frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            
            if button.is_pressed:
                motor.stop_rpm_control()
                motor.brake()
                break
            if turn_counter >= 13:
                motor.stop_rpm_control()
                parking_start_time = time.monotonic()
                time_before_parking = parking_start_time - run_start_time
                print(f"--- Time before parking (13 turns complete): {time_before_parking:.2f}s ---")
                if driving_direction == 'clockwise':
                    parking()
                else:
                    parking2()
                run_end_time = time.monotonic()
                time_in_parking = run_end_time - parking_start_time
                total_run_time = run_end_time - run_start_time
                print(f"--- Time spent in parking: {time_in_parking:.2f}s ---")
                print(f"--- Total run time: {total_run_time:.2f}s ---")
                motor.brake()
                break
    except Exception as e:
        print(f"MainThread: ERROR during execution: {e}")
        traceback.print_exc()        

    finally:
        print("MainThread: Stopping motor RPM control and cleaning up motor...")
        try:
            motor.stop_rpm_control()
            motor.cleanup()
        except Exception as e:
            print(f"MainThread: Error cleaning up motor: {e}")

        try:
            servo.set_angle(0)
            servo.cleanup()
        except Exception as e:
            print(f"MainThread: Error cleaning up servo: {e}")

        print("MainThread: Signaling threads to stop...")
        camera_thread.stop()
        sensor_thread.stop()
        imu_thread.stop()
        video_writer_thread.stop()

        print("MainThread: Waiting for threads to complete...")
        camera_thread.join()
        sensor_thread.join()
        imu_thread.join()
        video_writer_thread.join()
        print("MainThread: All threads have completed.")
                
        camera.cleanup()
        cv2.destroyAllWindows()
        if 'log_file' in locals() and not log_file.closed:
            print(f"Log file saved to {log_path}") # This will print to your console
            log_file.close()
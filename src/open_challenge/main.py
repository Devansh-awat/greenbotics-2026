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
    processed_data = {
        'detected_walls': [],
        'detected_orange': [],
        'detected_close_black': []
    }
    
    frame = cv2.GaussianBlur(frame, (1, 7), 0)
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask_black = cv2.inRange(hsv_frame, config.LOWER_BLACK, config.UPPER_BLACK)
    mask_orange = cv2.inRange(hsv_frame, config.LOWER_ORANGE, config.UPPER_ORANGE)

    # Apply ROI masks to get detections only in the desired areas
    final_mask_walls = cv2.bitwise_and(mask_black, config.roi_mask_walls)
    final_mask_orange = cv2.bitwise_and(mask_orange, config.roi_mask_orange)
    final_mask_close_black = cv2.bitwise_and(mask_black, config.roi_mask_close_black)

    # Detect orange line for turn counting
    contours, _ = cv2.findContours(final_mask_orange, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        biggest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(biggest_contour)
        if area > config.ORANGE_MIN_AREA:
            M = cv2.moments(biggest_contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                processed_data['detected_orange'].append({'area': area, 'centroid': (cx, cy)})

    # Detect "close black" walls for sharp turn avoidance
    contours, _ = cv2.findContours(final_mask_close_black, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > config.WALL_MIN_AREA:
            processed_data['detected_close_black'].append({'area': area})

    # Detect side walls for steering
    wall_contours_by_roi = {job['type']: [] for job in [config.left_side_job, config.right_side_job, config.inner_left_side_job, config.inner_right_side_job]}
    contours, _ = cv2.findContours(final_mask_walls, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for c in contours:
        if cv2.contourArea(c) > config.WALL_MIN_AREA:
            M = cv2.moments(c)
            if M["m00"] == 0: continue
            cx = int(M["m10"] / M["m00"])

            # Determine which ROI the contour belongs to
            job_type = 'unknown'
            if config.left_side_job['roi'][0] <= cx < config.left_side_job['roi'][0] + config.left_side_job['roi'][2]: job_type = config.left_side_job['type']
            elif config.right_side_job['roi'][0] <= cx < config.right_side_job['roi'][0] + config.right_side_job['roi'][2]: job_type = config.right_side_job['type']
            elif config.inner_left_side_job['roi'][0] <= cx < config.inner_left_side_job['roi'][0] + config.inner_left_side_job['roi'][2]: job_type = config.inner_left_side_job['type']
            elif config.inner_right_side_job['roi'][0] <= cx < config.inner_right_side_job['roi'][0] + config.inner_right_side_job['roi'][2]: job_type = config.inner_right_side_job['type']

            if job_type != 'unknown':
                wall_contours_by_roi[job_type].append(c)

    # Find the biggest contour in each ROI and add it to the processed data
    for job_type, contour_list in wall_contours_by_roi.items():
        if contour_list:
            biggest_contour = max(contour_list, key=cv2.contourArea)
            area = cv2.contourArea(biggest_contour)
            processed_data['detected_walls'].append({'type': job_type, 'area': area, 'contour': biggest_contour})
    
    return processed_data


def annotate_video_frame(frame, detections, debug_info=""):
    annotated_frame = frame.copy()
    light_blue = (255, 255, 0)

    # Define all ROIs for drawing
    all_rois = [
        config.left_side_job['roi'], config.right_side_job['roi'],
        config.inner_left_side_job['roi'], config.inner_right_side_job['roi'],
        (config.orange_roi_x, config.orange_roi_y, config.orange_roi_w, config.orange_roi_h),
        (config.close_x, config.close_y, config.close_w, config.close_h),
    ]
    # Draw ROIs on the frame
    for x, y, w, h in all_rois:
        cv2.rectangle(annotated_frame, (x, y), (x + w, y + h), light_blue, 2)

    # Draw detected walls
    for wall in detections['detected_walls']:
        cv2.drawContours(annotated_frame, [wall['contour']], -1, (255, 0, 0), 2)

    # Draw detected orange line
    if detections['detected_orange']:
        cv2.circle(annotated_frame, detections['detected_orange'][0]['centroid'], 5, (0, 165, 255), -1)

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
    past_frame_counter = 0

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
        motor.forward(config.MOTOR_SPEED)

        while True:
            angle = 0
            debug = []
            frame, frame_counter = camera_thread.get_frame()
            if frame is None or frame_counter == past_frame_counter:
                continue
            past_frame_counter = frame_counter
            
            # Process frame to find walls and orange lines
            detections = process_video_frame(frame)
            
            # --- Turn Counting Logic ---
            orange_detected_this_frame = bool(detections['detected_orange'])
            orange_detection_history.append(orange_detected_this_frame)
            
            if cooldown_frames > 0:
                cooldown_frames -= 1
            elif not orange_detection_history[-ORANGE_DETECTION_HISTORY_LENGTH] and all(list(orange_detection_history)[1:]):
                turn_counter += 1
                cooldown_frames = ORANGE_COOLDOWN_FRAMES
                print("turn_counter ---------------->", turn_counter)

            # --- Steering Logic ---
            left_pixel_size = sum(obj['area'] for obj in detections['detected_walls'] if 'left' in obj['type'])
            right_pixel_size = sum(obj['area'] for obj in detections['detected_walls'] if 'right' in obj['type'])

            if left_pixel_size < 100 and right_pixel_size > 100:
                right_pixel_size += 25000
            elif right_pixel_size < 100 and left_pixel_size > 100:
                left_pixel_size += 25000
            
            angle = ((left_pixel_size - right_pixel_size) * 0.0005)
            
            close_black_area = sum(obj['area'] for obj in detections.get('detected_close_black', []))
            if close_black_area > 3000:
                if left_pixel_size < right_pixel_size:
                    angle = -35  # Turn left
                else:
                    angle = 35   # Turn right
            
            angle = np.clip(angle, prev_angle - 10, prev_angle + 10)
            angle = np.clip(angle, -40, 40)
            servo.set_angle(angle)
            prev_angle = angle

            # --- Annotation and Video Recording ---
            debug.extend([f"L:{int(left_pixel_size)}", f"R:{int(right_pixel_size)}"])
            debug.append(f"Angle:{int(angle)}")
            debug.append(f"Turns:{turn_counter}")
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

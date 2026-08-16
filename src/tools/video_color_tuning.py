"""Single-window Video-based HSV Color Tuning Tool for Obstacle Challenge.

Allows tuning HSV color ranges interactively on recorded run videos (e.g., the
unfiltered raw.mp4 written by the `-v` flag). The bilateral filter is applied here at
playback time, after the colour conversion, matching prepare_colour_slice() in
src/vision/pipeline.py.

Features:
- Single-window layout containing all sliders (Frame, Color, H/S/V) and video panels
- Automatically loads the latest video from obstacle/<timestamp>/obstacle.mp4 or a custom path
- Play / Pause (SPACE) and frame scrubbing slider (0 to total_frames)
- Frame step forward/backward (d/a or ./, or arrow keys) & 10-frame jumps (f/b)
- Color selector for RED, GREEN, BLUE, MAGENTA, ORANGE, BLACK (keys 1-6, 'c', or Color slider)
- Interactive HSV trackbars with red-hue wraparound (H1 & H2) support
- Real-time mouse hover pixel inspection: exact HSV/BGR + 3x3 neighborhood circular mean
- Toggle ROI boxes overlay (key 'o')
- Save / Export formatted HSV_RANGES for src/obstacle_challenge/tuning.py (key 's')
"""

import argparse
import glob
import os
import sys
import time
from copy import deepcopy

import cv2
import numpy as np

from src.obstacle_challenge.tuning import (
    BILATERAL_D, BILATERAL_SIGMA_COLOR, BILATERAL_SIGMA_SPACE, FRAME_HEIGHT,
    FRAME_WIDTH, HSV_RANGES, USE_BILATERAL, close_block_roi, close_x, close_y,
    close_w, close_h, full_frame_roi, inner_left_roi_x, inner_left_roi_y,
    inner_left_roi_w, inner_left_roi_h, inner_right_roi_x, inner_right_roi_y,
    inner_right_roi_w, inner_right_roi_h, left_roi_x, left_roi_y, left_roi_w,
    left_roi_h, line_roi_x, line_roi_y, line_roi_w, line_roi_h, right_roi_x,
    right_roi_y, right_roi_w, right_roi_h,
)

COLORS = ['RED', 'GREEN', 'BLUE', 'MAGENTA', 'ORANGE', 'BLACK']
COLOR_SHORTCUTS = {ord(str(i + 1)): c for i, c in enumerate(COLORS)}

WINDOW_NAME = "Obstacle Challenge - HSV Video Tuner"


def find_latest_video(base_dir="obstacle", prefer_raw=True):
    """Find the most recent video in the obstacle/ folder, prioritizing raw footage (raw.mp4)."""
    if not os.path.exists(base_dir):
        return None

    if prefer_raw:
        # Search for raw.mp4 files inside subdirectories first
        raw_matches = glob.glob(os.path.join(base_dir, "*", "raw.mp4"))
        if raw_matches:
            raw_matches.sort(key=lambda p: os.path.getmtime(p), reverse=True)
            return raw_matches[0]

    # Search for all obstacle.mp4 files inside subdirectories
    matches = glob.glob(os.path.join(base_dir, "*", "obstacle.mp4"))
    if not matches:
        matches = glob.glob(os.path.join(base_dir, "*", "*.mp4"))
    if not matches:
        matches = glob.glob(os.path.join(base_dir, "*.mp4"))

    if not matches:
        return None

    # Sort by directory name / modification time descending
    matches.sort(key=lambda p: os.path.getmtime(p), reverse=True)
    return matches[0]


def get_default_color_ranges():
    """Build a working copy of HSV ranges from tuning.py."""
    ranges = {}
    for color in COLORS:
        if color == 'RED':
            ranges['RED'] = {
                'H1_min': int(HSV_RANGES.get('LOWER_RED_1', [0, 70, 43])[0]),
                'H1_max': int(HSV_RANGES.get('UPPER_RED_1', [4, 230, 166])[0]),
                'H2_min': int(HSV_RANGES.get('LOWER_RED_2', [176, 70, 43])[0]),
                'H2_max': int(HSV_RANGES.get('UPPER_RED_2', [180, 230, 140])[0]),
                'S_min': int(HSV_RANGES.get('LOWER_RED_1', [0, 70, 43])[1]),
                'S_max': int(HSV_RANGES.get('UPPER_RED_1', [4, 230, 166])[1]),
                'V_min': int(HSV_RANGES.get('LOWER_RED_1', [0, 70, 43])[2]),
                'V_max': int(HSV_RANGES.get('UPPER_RED_1', [4, 230, 166])[2]),
            }
        else:
            lower_key = f'LOWER_{color}'
            upper_key = f'UPPER_{color}'
            lo = HSV_RANGES.get(lower_key, np.array([0, 0, 0]))
            up = HSV_RANGES.get(upper_key, np.array([180, 255, 255]))
            ranges[color] = {
                'H_min': int(lo[0]),
                'H_max': int(up[0]),
                'S_min': int(lo[1]),
                'S_max': int(up[1]),
                'V_min': int(lo[2]),
                'V_max': int(up[2]),
            }
    return ranges


class VideoColorTuner:
    def __init__(self, video_path, apply_filter_override=False):
        self.video_path = video_path
        self.apply_filter_override = apply_filter_override
        self.cap = cv2.VideoCapture(video_path)
        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open video file: {video_path}")

        self.total_frames = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
        self.fps = self.cap.get(cv2.CAP_PROP_FPS) or 30.0
        self.frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or FRAME_WIDTH
        self.frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or FRAME_HEIGHT

        self.current_frame_idx = 0
        self.is_playing = False
        self.playback_speed = 1.0
        self.show_rois = True
        self.apply_bilateral = self.apply_filter_override

        self.active_color_idx = 0
        self.color_ranges = get_default_color_ranges()
        self.original_ranges = deepcopy(self.color_ranges)

        self.mouse_x = -1
        self.mouse_y = -1
        self.current_frame = None
        self.current_hsv = None

        self._trackbar_updating = False

    @property
    def active_color(self):
        return COLORS[self.active_color_idx]

    def _mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_MOUSEMOVE:
            self.mouse_x = x
            self.mouse_y = y

    def _on_frame_trackbar(self, val):
        if not self._trackbar_updating:
            self.seek_frame(val)

    def _on_color_trackbar(self, val):
        if not self._trackbar_updating:
            self.set_active_color_index(val)

    def _on_hsv_trackbar(self, val):
        if self._trackbar_updating:
            return
        c = self.active_color
        if c == 'RED':
            self.color_ranges['RED']['H1_min'] = cv2.getTrackbarPos('H1_min (H_min)', WINDOW_NAME)
            self.color_ranges['RED']['H1_max'] = cv2.getTrackbarPos('H1_max (H_max)', WINDOW_NAME)
            self.color_ranges['RED']['H2_min'] = cv2.getTrackbarPos('H2_min (Red only)', WINDOW_NAME)
            self.color_ranges['RED']['H2_max'] = cv2.getTrackbarPos('H2_max (Red only)', WINDOW_NAME)
            self.color_ranges['RED']['S_min'] = cv2.getTrackbarPos('S_min', WINDOW_NAME)
            self.color_ranges['RED']['S_max'] = cv2.getTrackbarPos('S_max', WINDOW_NAME)
            self.color_ranges['RED']['V_min'] = cv2.getTrackbarPos('V_min', WINDOW_NAME)
            self.color_ranges['RED']['V_max'] = cv2.getTrackbarPos('V_max', WINDOW_NAME)
        else:
            self.color_ranges[c]['H_min'] = cv2.getTrackbarPos('H1_min (H_min)', WINDOW_NAME)
            self.color_ranges[c]['H_max'] = cv2.getTrackbarPos('H1_max (H_max)', WINDOW_NAME)
            self.color_ranges[c]['S_min'] = cv2.getTrackbarPos('S_min', WINDOW_NAME)
            self.color_ranges[c]['S_max'] = cv2.getTrackbarPos('S_max', WINDOW_NAME)
            self.color_ranges[c]['V_min'] = cv2.getTrackbarPos('V_min', WINDOW_NAME)
            self.color_ranges[c]['V_max'] = cv2.getTrackbarPos('V_max', WINDOW_NAME)

    def set_active_color_index(self, idx):
        self.active_color_idx = idx % len(COLORS)
        self.update_trackbars_for_color()

    def update_trackbars_for_color(self):
        """Update slider positions on the window to reflect current color values."""
        self._trackbar_updating = True
        c = self.active_color
        rng = self.color_ranges[c]
        try:
            cv2.setTrackbarPos('Color (1-6)', WINDOW_NAME, self.active_color_idx)
            if c == 'RED':
                cv2.setTrackbarPos('H1_min (H_min)', WINDOW_NAME, rng['H1_min'])
                cv2.setTrackbarPos('H1_max (H_max)', WINDOW_NAME, rng['H1_max'])
                cv2.setTrackbarPos('H2_min (Red only)', WINDOW_NAME, rng['H2_min'])
                cv2.setTrackbarPos('H2_max (Red only)', WINDOW_NAME, rng['H2_max'])
                cv2.setTrackbarPos('S_min', WINDOW_NAME, rng['S_min'])
                cv2.setTrackbarPos('S_max', WINDOW_NAME, rng['S_max'])
                cv2.setTrackbarPos('V_min', WINDOW_NAME, rng['V_min'])
                cv2.setTrackbarPos('V_max', WINDOW_NAME, rng['V_max'])
            else:
                cv2.setTrackbarPos('H1_min (H_min)', WINDOW_NAME, rng['H_min'])
                cv2.setTrackbarPos('H1_max (H_max)', WINDOW_NAME, rng['H_max'])
                cv2.setTrackbarPos('H2_min (Red only)', WINDOW_NAME, 0)
                cv2.setTrackbarPos('H2_max (Red only)', WINDOW_NAME, 180)
                cv2.setTrackbarPos('S_min', WINDOW_NAME, rng['S_min'])
                cv2.setTrackbarPos('S_max', WINDOW_NAME, rng['S_max'])
                cv2.setTrackbarPos('V_min', WINDOW_NAME, rng['V_min'])
                cv2.setTrackbarPos('V_max', WINDOW_NAME, rng['V_max'])
        except Exception:
            pass
        self._trackbar_updating = False

    def seek_frame(self, frame_idx):
        frame_idx = max(0, min(self.total_frames - 1, frame_idx))
        self.cap.set(cv2.CAP_PROP_POS_FRAMES, frame_idx)
        ret, frame = self.cap.read()
        if ret and frame is not None:
            self.current_frame_idx = frame_idx
            self.current_frame = frame
            self._update_hsv()
            self._update_frame_trackbar()

    def _update_frame_trackbar(self):
        self._trackbar_updating = True
        try:
            cv2.setTrackbarPos('Frame', WINDOW_NAME, self.current_frame_idx)
        except Exception:
            pass
        self._trackbar_updating = False

    def read_next_frame(self):
        ret, frame = self.cap.read()
        if not ret or frame is None:
            # Loop back to beginning
            self.seek_frame(0)
            return
        self.current_frame_idx += 1
        self.current_frame = frame
        self._update_hsv()
        self._update_frame_trackbar()

    def _update_hsv(self):
        """Convert then filter, in that order -- the same order the pipeline uses.

        prepare_colour_slice() in src/vision/pipeline.py runs BGR->HSV first and
        bilateral-filters the HSV image (HSV_BEFORE_BLUR=True). Filtering the BGR
        frame and converting afterwards is a different operation on different
        pixels, so ranges tuned that way do not transfer to the running robot.
        """
        if self.current_frame is None:
            return
        hsv = cv2.cvtColor(self.current_frame, cv2.COLOR_BGR2HSV)
        if self.apply_bilateral:
            # Mirrors the pipeline's own USE_BILATERAL branch, GaussianBlur kernel
            # included, so flipping that constant does not silently make the tuner
            # disagree with the robot again.
            if USE_BILATERAL:
                hsv = cv2.bilateralFilter(
                    hsv, BILATERAL_D, BILATERAL_SIGMA_COLOR, BILATERAL_SIGMA_SPACE
                )
            else:
                hsv = cv2.GaussianBlur(hsv, (1, 7), 0)
        self.current_hsv = hsv

    def compute_mask(self):
        if self.current_hsv is None:
            return None
        c = self.active_color
        rng = self.color_ranges[c]
        if c == 'RED':
            lower1 = np.array([rng['H1_min'], rng['S_min'], rng['V_min']])
            upper1 = np.array([rng['H1_max'], rng['S_max'], rng['V_max']])
            lower2 = np.array([rng['H2_min'], rng['S_min'], rng['V_min']])
            upper2 = np.array([rng['H2_max'], rng['S_max'], rng['V_max']])
            m1 = cv2.inRange(self.current_hsv, lower1, upper1)
            m2 = cv2.inRange(self.current_hsv, lower2, upper2)
            mask = cv2.bitwise_or(m1, m2)
        else:
            lower = np.array([rng['H_min'], rng['S_min'], rng['V_min']])
            upper = np.array([rng['H_max'], rng['S_max'], rng['V_max']])
            mask = cv2.inRange(self.current_hsv, lower, upper)

        if c in ('RED', 'GREEN'):
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask

    def draw_rois(self, img):
        """Draw the main Obstacle Challenge ROIs for visual reference."""
        overlay = img.copy()
        # Wall ROIs (cyan)
        cv2.rectangle(overlay, (left_roi_x, left_roi_y), (left_roi_x + left_roi_w, left_roi_y + left_roi_h), (255, 255, 0), 1)
        cv2.rectangle(overlay, (right_roi_x, right_roi_y), (right_roi_x + right_roi_w, right_roi_y + right_roi_h), (255, 255, 0), 1)
        # Inner wall ROIs (yellow)
        cv2.rectangle(overlay, (inner_left_roi_x, inner_left_roi_y), (inner_left_roi_x + inner_left_roi_w, inner_left_roi_y + inner_left_roi_h), (0, 255, 255), 1)
        cv2.rectangle(overlay, (inner_right_roi_x, inner_right_roi_y), (inner_right_roi_x + inner_right_roi_w, inner_right_roi_y + inner_right_roi_h), (0, 255, 255), 1)
        # Main blocks ROI (green)
        fx, fy, fw, fh = full_frame_roi
        cv2.rectangle(overlay, (fx, fy), (fx + fw, fy + fh), (0, 255, 0), 1)
        # Line ROI (magenta)
        cv2.rectangle(overlay, (line_roi_x, line_roi_y), (line_roi_x + line_roi_w, line_roi_y + line_roi_h), (255, 0, 255), 1)
        # Close black ROI (red)
        cv2.rectangle(overlay, (close_x, close_y), (close_x + close_w, close_y + close_h), (0, 0, 255), 1)
        return overlay

    def export_ranges(self):
        """Print current HSV_RANGES dictionary formatted for src/obstacle_challenge/tuning.py."""
        print("\n" + "=" * 65)
        print("=== EXPORTED HSV_RANGES (for src/obstacle_challenge/tuning.py) ===")
        print("=" * 65)
        print("HSV_RANGES = {")
        r = self.color_ranges
        red = r['RED']
        print(f"    'LOWER_RED_1': np.array([{red['H1_min']}, {red['S_min']}, {red['V_min']}]), "
              f"'UPPER_RED_1': np.array([{red['H1_max']}, {red['S_max']}, {red['V_max']}]),")
        print(f"    'LOWER_RED_2': np.array([{red['H2_min']}, {red['S_min']}, {red['V_min']}]), "
              f"'UPPER_RED_2': np.array([{red['H2_max']}, {red['S_max']}, {red['V_max']}]),")

        for color in ['GREEN', 'BLACK', 'ORANGE', 'BLUE', 'MAGENTA']:
            c = r[color]
            print(f"    'LOWER_{color}': np.array([{c['H_min']}, {c['S_min']}, {c['V_min']}]), "
                  f"'UPPER_{color}': np.array([{c['H_max']}, {c['S_max']}, {c['V_max']}]),")
        print("}")
        print("=" * 65 + "\n")

    def run(self):
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(WINDOW_NAME, self._mouse_callback)

        # Guard against trackbar callbacks during initialization
        self._trackbar_updating = True

        # Initial values for first active color (RED)
        red_rng = self.color_ranges['RED']
        cv2.createTrackbar('Frame', WINDOW_NAME, 0, max(1, self.total_frames - 1), self._on_frame_trackbar)
        cv2.createTrackbar('Color (1-6)', WINDOW_NAME, 0, len(COLORS) - 1, self._on_color_trackbar)
        cv2.createTrackbar('H1_min (H_min)', WINDOW_NAME, red_rng['H1_min'], 180, self._on_hsv_trackbar)
        cv2.createTrackbar('H1_max (H_max)', WINDOW_NAME, red_rng['H1_max'], 180, self._on_hsv_trackbar)
        cv2.createTrackbar('H2_min (Red only)', WINDOW_NAME, red_rng['H2_min'], 180, self._on_hsv_trackbar)
        cv2.createTrackbar('H2_max (Red only)', WINDOW_NAME, red_rng['H2_max'], 180, self._on_hsv_trackbar)
        cv2.createTrackbar('S_min', WINDOW_NAME, red_rng['S_min'], 255, self._on_hsv_trackbar)
        cv2.createTrackbar('S_max', WINDOW_NAME, red_rng['S_max'], 255, self._on_hsv_trackbar)
        cv2.createTrackbar('V_min', WINDOW_NAME, red_rng['V_min'], 255, self._on_hsv_trackbar)
        cv2.createTrackbar('V_max', WINDOW_NAME, red_rng['V_max'], 255, self._on_hsv_trackbar)

        self.update_trackbars_for_color()
        self._trackbar_updating = False
        self.seek_frame(0)

        print(f"\nLoaded video: {self.video_path}")
        print(f"Total frames: {self.total_frames} | Dimensions: {self.frame_width}x{self.frame_height}")
        print("\nShortcuts:")
        print("  SPACE       : Play / Pause")
        print("  d / a / . , : Step 1 frame forward / backward")
        print("  f / b       : Jump 10 frames forward / backward")
        print("  1 - 6 / c   : Select color (1:RED, 2:GREEN, 3:BLUE, 4:MAGENTA, 5:ORANGE, 6:BLACK)")
        print("  o           : Toggle ROI boxes overlay")
        print("  r           : Reset active color to original defaults")
        print("  s           : Save & Print HSV_RANGES configuration block")
        print("  q / ESC     : Quit\n")

        last_play_time = time.time()

        try:
            while True:
                # Video playback step
                if self.is_playing:
                    now = time.time()
                    interval = 1.0 / (self.fps * self.playback_speed)
                    if now - last_play_time >= interval:
                        self.read_next_frame()
                        last_play_time = now

                if self.current_frame is None:
                    time.sleep(0.02)
                    continue

                # Prepare source display frame
                src_disp = self.current_frame.copy()
                if self.show_rois:
                    src_disp = self.draw_rois(src_disp)

                # Compute color mask and result
                mask = self.compute_mask()
                mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
                result_bgr = cv2.bitwise_and(self.current_frame, self.current_frame, mask=mask)

                # Stack 3 panels: [Source] | [Mask] | [Result]
                stacked = np.hstack((src_disp, mask_bgr, result_bgr))
                h, w = self.frame_height, self.frame_width

                # Mouse hover inspection
                if 0 <= self.mouse_y < h and 0 <= self.mouse_x < stacked.shape[1]:
                    px = self.mouse_x % w
                    py = self.mouse_y

                    # Exact values
                    h_val, s_val, v_val = self.current_hsv[py, px]
                    b_val, g_val, r_val = self.current_frame[py, px]

                    # 3x3 neighborhood circular mean
                    y1, y2 = max(0, py - 1), min(h, py + 2)
                    x1, x2 = max(0, px - 1), min(w, px + 2)
                    patch_hsv = self.current_hsv[y1:y2, x1:x2]

                    h_rad = patch_hsv[:, :, 0].astype(np.float32) * (2.0 * np.pi / 180.0)
                    sin_mean = np.mean(np.sin(h_rad))
                    cos_mean = np.mean(np.cos(h_rad))
                    avg_h = (np.arctan2(sin_mean, cos_mean) * (180.0 / (2.0 * np.pi))) % 180.0
                    avg_s = np.mean(patch_hsv[:, :, 1])
                    avg_v = np.mean(patch_hsv[:, :, 2])

                    score_val = int(b_val) - int(g_val)
                    info_text = (
                        f"Pos: ({px:3d},{py:3d}) | "
                        f"HSV: [{h_val:3d},{s_val:3d},{v_val:3d}] | "
                        f"3x3 Avg HSV: [{int(round(avg_h)):3d},{int(round(avg_s)):3d},{int(round(avg_v)):3d}] | "
                        f"BGR: [{b_val:3d},{g_val:3d},{r_val:3d}] | "
                        f"score: {score_val}"
                    )

                    # Top HUD
                    cv2.rectangle(stacked, (0, 0), (stacked.shape[1], 32), (18, 18, 18), -1)
                    cv2.putText(
                        stacked, info_text, (10, 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 255, 255), 1, cv2.LINE_AA
                    )

                    # Markers on all 3 panels
                    for panel in range(3):
                        cx = px + panel * w
                        cv2.rectangle(stacked, (cx - 1, py - 1), (cx + 1, py + 1), (0, 255, 255), 1)
                        cv2.drawMarker(stacked, (cx, py), (0, 255, 255), cv2.MARKER_CROSS, 12, 1)
                else:
                    status = "PLAYING" if self.is_playing else "PAUSED"
                    hud_text = (
                        f"[{status}] Frame: {self.current_frame_idx + 1}/{self.total_frames} | "
                        f"Tuning: {self.active_color} | "
                        f"ROIs: {'ON' if self.show_rois else 'OFF'} (o) | "
                        f"Filter: {'Bilateral' if self.apply_bilateral else 'None'} | "
                        f"Export: (s) | Quit: (q)"
                    )
                    cv2.rectangle(stacked, (0, 0), (stacked.shape[1], 32), (18, 18, 18), -1)
                    cv2.putText(
                        stacked, hud_text, (10, 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.54, (220, 220, 220), 1, cv2.LINE_AA
                    )

                # Panel labels
                cv2.putText(stacked, "Source (ROIs: 'o')", (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                cv2.putText(stacked, f"Mask ({self.active_color})", (w + 10, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                cv2.putText(stacked, "Result", (2 * w + 10, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

                cv2.imshow(WINDOW_NAME, stacked)

                # Handle keyboard input
                key = cv2.waitKey(15) & 0xFF
                if key in (ord('q'), 27):  # q or ESC
                    break
                elif key == ord(' '):
                    self.is_playing = not self.is_playing
                elif key in (ord('d'), ord('.'), 83):  # step forward
                    self.is_playing = False
                    self.read_next_frame()
                elif key in (ord('a'), ord(','), 81):  # step backward
                    self.is_playing = False
                    self.seek_frame(self.current_frame_idx - 1)
                elif key in (ord('f'), ord('l')):      # 10 frames forward
                    self.seek_frame(self.current_frame_idx + 10)
                elif key in (ord('b'), ord('j')):      # 10 frames backward
                    self.seek_frame(self.current_frame_idx - 10)
                elif key in COLOR_SHORTCUTS:
                    col_name = COLOR_SHORTCUTS[key]
                    self.set_active_color_index(COLORS.index(col_name))
                elif key == ord('c'):
                    self.set_active_color_index((self.active_color_idx + 1) % len(COLORS))
                elif key == ord('o'):
                    self.show_rois = not self.show_rois
                elif key == ord('r'):
                    # Reset active color to original defaults
                    self.color_ranges[self.active_color] = deepcopy(self.original_ranges[self.active_color])
                    self.update_trackbars_for_color()
                    print(f"Reset {self.active_color} to default values.")
                elif key == ord('s'):
                    self.export_ranges()

        except KeyboardInterrupt:
            pass
        finally:
            self.cap.release()
            cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description="Obstacle Challenge Video HSV Color Tuner.")
    parser.add_argument(
        "video",
        nargs="?",
        default=None,
        help="Path to video file (.mp4) or run folder. If not provided, automatically uses the latest raw.mp4 in obstacle/ directory."
    )
    # On by default: raw.mp4 is recorded untouched, so the filter has to be applied
    # here for the mask to match what the robot's pipeline actually thresholds.
    parser.add_argument(
        "-n", "--no-filter",
        dest="filter", action="store_false",
        help="Skip the bilateral filter during playback. The filter is applied by "
             "default because raw.mp4 is recorded unfiltered; use this to see the "
             "unsmoothed pixels."
    )
    args = parser.parse_args()

    video_path = args.video
    if not video_path:
        video_path = find_latest_video()
        if not video_path:
            print("ERROR: No video files found in obstacle/ directory.")
            print("Please specify a video path: python3 -m src.tools.video_color_tuning <path_to_video.mp4>")
            sys.exit(1)
        print(f"Auto-detected latest video: {video_path}")
    elif os.path.isdir(video_path):
        raw_in_dir = os.path.join(video_path, "raw.mp4")
        obs_in_dir = os.path.join(video_path, "obstacle.mp4")
        if os.path.exists(raw_in_dir):
            video_path = raw_in_dir
        elif os.path.exists(obs_in_dir):
            video_path = obs_in_dir
        else:
            mp4s = glob.glob(os.path.join(video_path, "*.mp4"))
            if mp4s:
                video_path = mp4s[0]
            else:
                print(f"ERROR: No .mp4 video found in directory: {video_path}")
                sys.exit(1)
        print(f"Using video from directory: {video_path}")
    elif not os.path.exists(video_path):
        print(f"ERROR: Video file not found: {video_path}")
        sys.exit(1)

    tuner = VideoColorTuner(video_path, apply_filter_override=args.filter)
    tuner.run()


if __name__ == '__main__':
    main()

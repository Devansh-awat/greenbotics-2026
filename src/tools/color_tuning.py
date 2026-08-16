"""Live Camera HSV Color Tuning Tool for Obstacle Challenge.

Allows tuning HSV color ranges interactively using the live camera feed with real-time feedback.

Features:
- Single-window layout with live preview, color mask, and masked result
- Color selector for RED, GREEN, BLUE, MAGENTA, ORANGE, BLACK (keys 1-6, 'c', or Color slider)
- Interactive HSV trackbars with red-hue wraparound (H1 & H2) support
- Real-time mouse hover pixel inspection: exact HSV/BGR + 9px circular mean HSV + B-G as score
- Save / Export formatted HSV_RANGES for src/obstacle_challenge/tuning.py (key 's')
- Reset active color to defaults (key 'r')
"""

import os
import sys
import time
from copy import deepcopy

import cv2
import numpy as np

from src.sensors import camera
from src.obstacle_challenge.tuning import (
    BILATERAL_D,
    BILATERAL_SIGMA_COLOR,
    BILATERAL_SIGMA_SPACE,
    HSV_RANGES,
    USE_BILATERAL,
)

COLORS = ['RED', 'GREEN', 'BLUE', 'MAGENTA', 'ORANGE', 'BLACK']
COLOR_SHORTCUTS = {ord(str(i + 1)): c for i, c in enumerate(COLORS)}
WINDOW_NAME = "Live HSV Color Tuner"


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


class LiveColorTuner:
    def __init__(self):
        self.active_color_idx = 0
        self.color_ranges = get_default_color_ranges()
        self.original_ranges = deepcopy(self.color_ranges)

        self.mouse_x = -1
        self.mouse_y = -1
        self._trackbar_updating = False

    @property
    def active_color(self):
        return COLORS[self.active_color_idx]

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_MOUSEMOVE:
            self.mouse_x = x
            self.mouse_y = y

    def on_color_trackbar(self, val):
        if not self._trackbar_updating:
            self.set_active_color_index(val)

    def on_hsv_trackbar(self, val):
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

    def compute_mask(self, hsv_frame):
        c = self.active_color
        rng = self.color_ranges[c]
        if c == 'RED':
            lower1 = np.array([rng['H1_min'], rng['S_min'], rng['V_min']])
            upper1 = np.array([rng['H1_max'], rng['S_max'], rng['V_max']])
            lower2 = np.array([rng['H2_min'], rng['S_min'], rng['V_min']])
            upper2 = np.array([rng['H2_max'], rng['S_max'], rng['V_max']])
            m1 = cv2.inRange(hsv_frame, lower1, upper1)
            m2 = cv2.inRange(hsv_frame, lower2, upper2)
            mask = cv2.bitwise_or(m1, m2)
        else:
            lower = np.array([rng['H_min'], rng['S_min'], rng['V_min']])
            upper = np.array([rng['H_max'], rng['S_max'], rng['V_max']])
            mask = cv2.inRange(hsv_frame, lower, upper)

        if c in ('RED', 'GREEN'):
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask

    def export_ranges(self):
        """Print current HSV_RANGES dictionary formatted for src/obstacle_challenge/tuning.py."""
        print("\n" + "=" * 65)
        print("=== EXPORTED HSV_RANGES (for src/obstacle_challenge/tuning.py) ===")
        print("=" * 65)
        print("HSV_RANGES = {")
        red = self.color_ranges['RED']
        print(f"    'LOWER_RED_1': np.array([{red['H1_min']}, {red['S_min']}, {red['V_min']}]), "
              f"'UPPER_RED_1': np.array([{red['H1_max']}, {red['S_max']}, {red['V_max']}]),")
        print(f"    'LOWER_RED_2': np.array([{red['H2_min']}, {red['S_min']}, {red['V_min']}]), "
              f"'UPPER_RED_2': np.array([{red['H2_max']}, {red['S_max']}, {red['V_max']}]),")

        for color in ['GREEN', 'BLACK', 'ORANGE', 'BLUE', 'MAGENTA']:
            c = self.color_ranges[color]
            print(f"    'LOWER_{color}': np.array([{c['H_min']}, {c['S_min']}, {c['V_min']}]), "
                  f"'UPPER_{color}': np.array([{c['H_max']}, {c['S_max']}, {c['V_max']}]),")
        print("}")
        print("=" * 65 + "\n")

    def run(self):
        print("Initializing camera...")
        if not camera.initialize():
            print("ERROR: Failed to initialize camera.")
            return

        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(WINDOW_NAME, self.mouse_callback)

        self._trackbar_updating = True
        red_rng = self.color_ranges['RED']
        cv2.createTrackbar('Color (1-6)', WINDOW_NAME, 0, len(COLORS) - 1, self.on_color_trackbar)
        cv2.createTrackbar('H1_min (H_min)', WINDOW_NAME, red_rng['H1_min'], 180, self.on_hsv_trackbar)
        cv2.createTrackbar('H1_max (H_max)', WINDOW_NAME, red_rng['H1_max'], 180, self.on_hsv_trackbar)
        cv2.createTrackbar('H2_min (Red only)', WINDOW_NAME, red_rng['H2_min'], 180, self.on_hsv_trackbar)
        cv2.createTrackbar('H2_max (Red only)', WINDOW_NAME, red_rng['H2_max'], 180, self.on_hsv_trackbar)
        cv2.createTrackbar('S_min', WINDOW_NAME, red_rng['S_min'], 255, self.on_hsv_trackbar)
        cv2.createTrackbar('S_max', WINDOW_NAME, red_rng['S_max'], 255, self.on_hsv_trackbar)
        cv2.createTrackbar('V_min', WINDOW_NAME, red_rng['V_min'], 255, self.on_hsv_trackbar)
        cv2.createTrackbar('V_max', WINDOW_NAME, red_rng['V_max'], 255, self.on_hsv_trackbar)

        self.update_trackbars_for_color()
        self._trackbar_updating = False

        print("\n" + "=" * 55)
        print("  Live HSV Color Tuning Tool")
        print("=" * 55)
        print("Controls:")
        print("  1 - 6 / c   : Select color (1:RED, 2:GREEN, 3:BLUE, 4:MAGENTA, 5:ORANGE, 6:BLACK)")
        print("  r           : Reset active color to default values")
        print("  s           : Save & Print HSV_RANGES configuration block")
        print("  q / ESC     : Quit")
        print("=" * 55 + "\n")

        try:
            while True:
                frame = camera.capture_frame()
                if frame is None:
                    time.sleep(0.01)
                    continue

                if USE_BILATERAL:
                    frame_filtered = cv2.bilateralFilter(
                        frame, BILATERAL_D, BILATERAL_SIGMA_COLOR, BILATERAL_SIGMA_SPACE
                    )
                else:
                    frame_filtered = cv2.GaussianBlur(frame, (5, 5), 0)

                hsv_frame = cv2.cvtColor(frame_filtered, cv2.COLOR_BGR2HSV)
                mask = self.compute_mask(hsv_frame)
                mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
                result_bgr = cv2.bitwise_and(frame_filtered, frame_filtered, mask=mask)

                stacked = np.hstack((frame_filtered, mask_bgr, result_bgr))
                h, w = frame_filtered.shape[:2]

                # Mouse hover pixel inspection
                if 0 <= self.mouse_y < h and 0 <= self.mouse_x < stacked.shape[1]:
                    px = self.mouse_x % w
                    py = self.mouse_y

                    h_val, s_val, v_val = hsv_frame[py, px]
                    b_val, g_val, r_val = frame_filtered[py, px]

                    # 3x3 circular mean on Hue
                    y1, y2 = max(0, py - 1), min(h, py + 2)
                    x1, x2 = max(0, px - 1), min(w, px + 2)
                    patch_hsv = hsv_frame[y1:y2, x1:x2]

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
                        f"9px Avg HSV: [{int(round(avg_h)):3d},{int(round(avg_s)):3d},{int(round(avg_v)):3d}] | "
                        f"BGR: [{b_val:3d},{g_val:3d},{r_val:3d}] | "
                        f"score: {score_val}"
                    )

                    cv2.rectangle(stacked, (0, 0), (stacked.shape[1], 32), (18, 18, 18), -1)
                    cv2.putText(
                        stacked, info_text, (10, 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.56, (0, 255, 255), 1, cv2.LINE_AA
                    )

                    for panel in range(3):
                        cx = px + panel * w
                        cv2.rectangle(stacked, (cx - 1, py - 1), (cx + 1, py + 1), (0, 255, 255), 1)
                        cv2.drawMarker(stacked, (cx, py), (0, 255, 255), cv2.MARKER_CROSS, 12, 1)
                else:
                    hud_text = (
                        f"Tuning: {self.active_color} | "
                        f"Filter: {'Bilateral' if USE_BILATERAL else 'Gaussian'} | "
                        f"Keys: 1-6 / c: Color | r: Reset | s: Export | q: Quit"
                    )
                    cv2.rectangle(stacked, (0, 0), (stacked.shape[1], 32), (18, 18, 18), -1)
                    cv2.putText(
                        stacked, hud_text, (10, 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.54, (220, 220, 220), 1, cv2.LINE_AA
                    )

                cv2.putText(stacked, "Camera Feed", (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                cv2.putText(stacked, f"Mask ({self.active_color})", (w + 10, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                cv2.putText(stacked, "Result", (2 * w + 10, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

                cv2.imshow(WINDOW_NAME, stacked)

                key = cv2.waitKey(1) & 0xFF
                if key in (ord('q'), 27):
                    break
                elif key in COLOR_SHORTCUTS:
                    col_name = COLOR_SHORTCUTS[key]
                    self.set_active_color_index(COLORS.index(col_name))
                elif key == ord('c'):
                    self.set_active_color_index((self.active_color_idx + 1) % len(COLORS))
                elif key == ord('r'):
                    self.color_ranges[self.active_color] = deepcopy(self.original_ranges[self.active_color])
                    self.update_trackbars_for_color()
                    print(f"Reset {self.active_color} to default values.")
                elif key == ord('s'):
                    self.export_ranges()

        except KeyboardInterrupt:
            pass
        finally:
            camera.cleanup()
            cv2.destroyAllWindows()


def main():
    tuner = LiveColorTuner()
    tuner.run()


if __name__ == '__main__':
    main()

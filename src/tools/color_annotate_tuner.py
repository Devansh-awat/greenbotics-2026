"""Polygon-annotation colour tuner (supports Video files & Live Camera).

Allows annotating obstacle colours by drawing polygons on video footage or live camera frames.
All pixels within accepted polygons are pooled per colour across multiple frames/angles.
A robust range is computed per channel using percentiles + margin, outputting ready-to-paste
HSV_RANGES and LAB_RANGES configuration blocks for src/obstacle_challenge/tuning.py.

Usage:
  # On a specific recorded run folder or video file (like video_color_tuning):
  python3 -m src.tools.color_annotate_tuner obstacle/2026-11-05_18-00-10
  python3 -m src.tools.color_annotate_tuner obstacle/2026-11-05_18-00-10/raw.mp4

  # Automatically pick latest video from obstacle/:
  python3 -m src.tools.color_annotate_tuner --latest

  # On live camera (default when no video argument is passed):
  python3 -m src.tools.color_annotate_tuner --live

Controls:
  SPACE         : Play / Pause (Video) or Freeze / Unfreeze (Live) (locked during selection)
  . / , (or d/p): Step 1 frame forward / backward (Video) (locked during selection)
  f / b         : Jump 10 frames forward / backward (Video) (locked during selection)
  1 - 6 / c     : Select active color (1:RED, 2:GREEN, 3:BLUE, 4:MAGENTA, 5:ORANGE, 6:BLACK)
  Left-Click    : Place polygon vertex (auto-pauses if playing)
  e             : Auto-expand polygon to edge of obstacle (LAB color distance)
  a / ENTER     : Accept polygon -> pool all pixels inside for active colour
  u             : Undo auto-expansion (if expanded) OR undo last vertex
  z             : Undo / revert last accepted polygon submission
  r             : Reset / clear current in-progress polygon
  x             : Clear all pooled samples for the active colour
  s             : Save current samples & print formatted HSV_RANGES / LAB_RANGES
  q / ESC       : Quit & print final formatted HSV_RANGES / LAB_RANGES
"""

import argparse
import glob
import os
import sys
import time

import cv2
import numpy as np

# Lazy / optional import of camera
try:
    from src.sensors import camera
except Exception:
    camera = None

# Optional import of pipeline filter parameters
try:
    from src.obstacle_challenge.tuning import (
        BILATERAL_D, BILATERAL_SIGMA_COLOR, BILATERAL_SIGMA_SPACE, USE_BILATERAL
    )
except Exception:
    USE_BILATERAL = True
    BILATERAL_D, BILATERAL_SIGMA_COLOR, BILATERAL_SIGMA_SPACE = 5, 50, 50

# --- Configuration ---------------------------------------------------------
COLORS = ['RED', 'GREEN', 'BLUE', 'MAGENTA', 'ORANGE', 'BLACK']

COLOR_BGR = {
    'RED': (0, 0, 255),
    'GREEN': (0, 255, 0),
    'BLUE': (255, 100, 0),
    'MAGENTA': (255, 0, 255),
    'ORANGE': (0, 165, 255),
    'BLACK': (100, 100, 100),
}

COLOR_SHORTCUTS = {
    ord('1'): 'RED',
    ord('2'): 'GREEN',
    ord('3'): 'BLUE',
    ord('4'): 'MAGENTA',
    ord('5'): 'ORANGE',
    ord('6'): 'BLACK',
}

P_LOW, P_HIGH = 2.0, 98.0          # percentile bounds (reject outlier pixels)
MARGIN_HUE = 3                     # extra slack on hue (0-179)
MARGIN_OTHER = 10                  # extra slack on S, V, L, a, b (0-255)
BLUR_KERNEL = (5, 5)              # fallback preprocessing kernel
SAMPLE_DIR = os.path.join(os.path.dirname(__file__), 'color_samples')

# Colours that should be kept disjoint from each other (similar hues).
DECONFLICT_PAIRS = [('RED', 'MAGENTA')]
# ---------------------------------------------------------------------------


def find_latest_video(base_dir="obstacle", prefer_raw=True):
    """Find the most recent video in the obstacle/ folder, prioritizing raw footage."""
    if not os.path.exists(base_dir):
        return None

    if prefer_raw:
        raw_matches = glob.glob(os.path.join(base_dir, "*", "raw.mp4"))
        if raw_matches:
            raw_matches.sort(key=lambda p: os.path.getmtime(p), reverse=True)
            return raw_matches[0]

    matches = glob.glob(os.path.join(base_dir, "*", "obstacle.mp4"))
    if not matches:
        matches = glob.glob(os.path.join(base_dir, "*", "*.mp4"))
    if not matches:
        matches = glob.glob(os.path.join(base_dir, "*.mp4"))

    if not matches:
        return None

    matches.sort(key=lambda p: os.path.getmtime(p), reverse=True)
    return matches[0]


def resolve_video_path(video_path):
    """Resolve a directory or file path into a playable video file."""
    if not video_path:
        return None
    if os.path.isdir(video_path):
        raw_in_dir = os.path.join(video_path, "raw.mp4")
        obs_in_dir = os.path.join(video_path, "obstacle.mp4")
        if os.path.exists(raw_in_dir):
            return raw_in_dir
        elif os.path.exists(obs_in_dir):
            return obs_in_dir
        mp4s = glob.glob(os.path.join(video_path, "*.mp4"))
        if mp4s:
            return mp4s[0]
        return None
    if os.path.exists(video_path):
        return video_path
    return None


def _percentile_range(channel, margin, hi):
    """Robust [low, high] for a single non-wrapping channel."""
    lo = np.percentile(channel, P_LOW) - margin
    up = np.percentile(channel, P_HIGH) + margin
    return max(0, int(round(lo))), min(int(hi), int(round(up)))


def _hue_range(hue):
    """Return one or two (lo, hi) hue ranges, handling 0/180 wraparound."""
    hue = hue.astype(int) % 180
    occupied = np.zeros(180, dtype=bool)
    counts = np.bincount(hue, minlength=180)
    thresh = max(1, int(0.001 * hue.size))   # ignore <0.1% stray bins
    occupied[counts >= thresh] = True

    occ = np.where(occupied)[0]
    if occ.size == 0:
        return [(0, 179)]

    # Find the largest circular gap between consecutive occupied degrees.
    deg = np.concatenate([occ, occ[:1] + 180])
    gaps = np.diff(deg)
    g = int(np.argmax(gaps))
    offset = int(occ[(g + 1) % occ.size])

    rot = (hue - offset) % 180
    lo_rot = np.percentile(rot, P_LOW) - MARGIN_HUE
    hi_rot = np.percentile(rot, P_HIGH) + MARGIN_HUE
    lo = int(round(lo_rot)) + offset
    hi = int(round(hi_rot)) + offset

    lo_m = lo % 180
    hi_m = hi % 180
    if hi - lo >= 179:
        return [(0, 179)]
    if lo_m <= hi_m:
        return [(lo_m, hi_m)]
    return [(0, hi_m), (lo_m, 179)]


def compute_hsv(bgr_pixels, color):
    """Return list of (lower, upper) np.int arrays in HSV for this colour."""
    hsv = cv2.cvtColor(bgr_pixels.reshape(-1, 1, 3), cv2.COLOR_BGR2HSV).reshape(-1, 3)
    s_lo, s_hi = _percentile_range(hsv[:, 1], MARGIN_OTHER, 255)
    v_lo, v_hi = _percentile_range(hsv[:, 2], MARGIN_OTHER, 255)
    out = []
    for (h_lo, h_hi) in _hue_range(hsv[:, 0]):
        out.append((np.array([h_lo, s_lo, v_lo]), np.array([h_hi, s_hi, v_hi])))
    return out, hsv


def compute_lab(bgr_pixels):
    """Return list of (lower, upper) np.int arrays in LAB for this colour."""
    lab = cv2.cvtColor(bgr_pixels.reshape(-1, 1, 3), cv2.COLOR_BGR2Lab).reshape(-1, 3)
    lo, hi = [], []
    for c in range(3):
        a, b = _percentile_range(lab[:, c], MARGIN_OTHER, 255)
        lo.append(a)
        hi.append(b)
    return [(np.array(lo), np.array(hi))], lab


def _deconflict_lab(ranges, conv, a_name, b_name):
    """Trim two LAB ranges so they don't overlap, on their best-separated channel."""
    A, B = conv[a_name], conv[b_name]
    seps = [abs(np.median(A[:, c]) - np.median(B[:, c])) for c in range(3)]
    c = int(np.argmax(seps))
    ma, mb = np.median(A[:, c]), np.median(B[:, c])
    bnd = int(round((ma + mb) / 2))
    lo_name, hi_name = (a_name, b_name) if ma < mb else (b_name, a_name)
    _, lo_upper = ranges[lo_name][0]
    if lo_upper[c] > bnd:
        lo_upper[c] = bnd
    hi_lower, _ = ranges[hi_name][0]
    if hi_lower[c] < bnd:
        hi_lower[c] = bnd
    return c


def _deconflict_hsv_hue(ranges, conv, a_name, b_name):
    """Trim HSV hue so RED/MAGENTA don't overlap (hue folded to remove wrap)."""
    def fold(h):
        h = h.astype(int)
        return np.where(h >= 90, h - 180, h)

    A = fold(conv[a_name][:, 0])
    B = fold(conv[b_name][:, 0])
    bnd_fold = (np.median(A) + np.median(B)) / 2.0
    bnd = int(round(bnd_fold)) % 180

    lower_is_a = np.median(A) < np.median(B)
    below, above = (a_name, b_name) if lower_is_a else (b_name, a_name)

    for lower, upper in ranges[above]:
        if lower[0] >= 90 and lower[0] < bnd:
            lower[0] = bnd
    for lower, upper in ranges[below]:
        if upper[0] > 90 and upper[0] > bnd:
            upper[0] = bnd
    return bnd


def _fmt(name, lower, upper):
    return (f"    'LOWER_{name}': np.array({list(map(int, lower))}), "
            f"'UPPER_{name}': np.array({list(map(int, upper))}),")


def compute_colour_split(red_bgr, mag_bgr, ref_frame=None):
    """Compute adaptive white-balanced threshold and band for COLOUR_SPLIT in tuning.py."""
    mat_b, mat_g = 170.0, 170.0
    if ref_frame is not None:
        hsv_ref = cv2.cvtColor(ref_frame, cv2.COLOR_BGR2HSV)
        mat_mask = cv2.inRange(hsv_ref, np.array([0, 0, 100]), np.array([180, 50, 255]))
        if np.count_nonzero(mat_mask) >= 40:
            mat_bgr = ref_frame[mat_mask > 0]
            mat_b = float(np.median(mat_bgr[:, 0]))
            mat_g = float(np.median(mat_bgr[:, 1]))

    # Mat-balanced score = 100 * (B / B_mat - G / G_mat)
    score_red = 100.0 * (red_bgr[:, 0].astype(float) / max(1.0, mat_b) - red_bgr[:, 1].astype(float) / max(1.0, mat_g))
    score_mag = 100.0 * (mag_bgr[:, 0].astype(float) / max(1.0, mat_b) - mag_bgr[:, 1].astype(float) / max(1.0, mat_g))

    p95_red = float(np.percentile(score_red, 95))
    p05_mag = float(np.percentile(score_mag, 5))
    threshold = round((p95_red + p05_mag) / 2.0, 1)

    hsv_red = cv2.cvtColor(red_bgr.reshape(-1, 1, 3), cv2.COLOR_BGR2HSV).reshape(-1, 3)
    hsv_mag = cv2.cvtColor(mag_bgr.reshape(-1, 1, 3), cv2.COLOR_BGR2HSV).reshape(-1, 3)

    v_red_med = float(np.median(hsv_red[:, 2]))
    v_mag_med = float(np.median(hsv_mag[:, 2]))
    v_ratio = round(min(0.9, max(0.45, v_red_med / max(1.0, v_mag_med))), 2)

    # Dynamic band bounds based on sampled pixels
    red_h1 = hsv_red[hsv_red[:, 0] <= 30]
    red_h1_max = int(np.percentile(red_h1[:, 0], 98) + 1) if len(red_h1) else 6
    red_h1_max = min(6, max(4, red_h1_max))   # cap at 6 to avoid orange floor line leak
    red_h1_s_max = min(160, int(np.percentile(red_h1[:, 1], 98) + 10)) if len(red_h1) else 160
    red_h1_v_max = min(130, int(np.percentile(red_h1[:, 2], 98) + 15)) if len(red_h1) else 120

    mag_h_min = int(np.percentile(hsv_mag[:, 0], 2) - 3) if len(hsv_mag) else 145
    band_h_min = min(145, max(135, mag_h_min))

    s_min = int(min(np.percentile(hsv_red[:, 1], 2), np.percentile(hsv_mag[:, 1], 2)) - 10)
    s_min = max(30, min(65, s_min))
    v_min = int(min(np.percentile(hsv_red[:, 2], 2), np.percentile(hsv_mag[:, 2], 2)) - 10)
    v_min = max(20, min(40, v_min))

    return {
        'threshold': threshold,
        'mat_bg': (mat_b, mat_g),
        'score_red_stats': (float(np.min(score_red)), float(np.median(score_red)), p95_red),
        'score_mag_stats': (p05_mag, float(np.median(score_mag)), float(np.max(score_mag))),
        'v_ratio': v_ratio,
        'band_h_min': band_h_min,
        's_min': s_min,
        'v_min': v_min,
        'red_h1_max': red_h1_max,
        'red_h1_s_max': red_h1_s_max,
        'red_h1_v_max': red_h1_v_max,
    }


def export_and_report(samples, ref_frame=None):
    """Compute and print HSV_RANGES, LAB_RANGES, and COLOUR_SPLIT configuration blocks."""
    if not samples:
        print("\n[!] No samples collected yet; nothing to compute.")
        return

    # Check if samples dictionary actually has pixels
    active_samples = {c: px for c, px in samples.items() if len(px) > 0}
    if not active_samples:
        print("\n[!] All sampled colours are empty; nothing to compute.")
        return

    hsv_ranges, lab_ranges = {}, {}
    hsv_conv, lab_conv = {}, {}
    for color, px in active_samples.items():
        h, hconv = compute_hsv(px, color)
        l, lconv = compute_lab(px)
        hsv_ranges[color] = h
        lab_ranges[color] = l
        hsv_conv[color] = hconv
        lab_conv[color] = lconv

    # Deconflict pairs
    for a, b in DECONFLICT_PAIRS:
        if a in active_samples and b in active_samples:
            cb = _deconflict_hsv_hue(hsv_ranges, hsv_conv, a, b)
            cl = _deconflict_lab(lab_ranges, lab_conv, a, b)
            print(f"\nDeconflicted {a}/{b}: HSV hue boundary={cb}, "
                  f"LAB split on channel {'Lab'[cl]}")

    print("\n" + "=" * 65)
    print("=== EXPORTED HSV_RANGES (for src/obstacle_challenge/tuning.py) ===")
    print("=" * 65)
    print("HSV_RANGES = {")
    for name, rngs in hsv_ranges.items():
        if len(rngs) == 2:
            for i, (lo, up) in enumerate(rngs, 1):
                print(_fmt(f"{name}_{i}", lo, up))
        else:
            print(_fmt(name, rngs[0][0], rngs[0][1]))
    print("}")

    print("\nLAB_RANGES = {")
    for name, rngs in lab_ranges.items():
        if len(rngs) == 2:
            for i, (lo, up) in enumerate(rngs, 1):
                print(_fmt(f"{name}_{i}", lo, up))
        else:
            print(_fmt(name, rngs[0][0], rngs[0][1]))
    print("}")
    print("=" * 65)

    # If both RED and MAGENTA were annotated, compute and output COLOUR_SPLIT!
    if 'RED' in active_samples and 'MAGENTA' in active_samples:
        cs = compute_colour_split(active_samples['RED'], active_samples['MAGENTA'], ref_frame)
        print("\n" + "=" * 65)
        print("=== EXPORTED COLOUR_SPLIT (for src/obstacle_challenge/tuning.py) ===")
        print("=" * 65)
        print("COLOUR_SPLIT = None if (USE_LAB or not USE_COLOUR_SPLIT) else dict(")
        print("    low='red', high='magenta',")
        print(f"    band=[(np.array([{cs['band_h_min']}, {cs['s_min']}, {cs['v_min']}]), np.array([180, 255, 255])),")
        print(f"          (np.array([0, {cs['s_min']}, {cs['v_min']}]), np.array([{cs['red_h1_max']}, {cs['red_h1_s_max']}, {cs['red_h1_v_max']}]))],")
        print("    mat=(np.array([0, 0, 100]), np.array([180, 50, 255])),   # the white mat, in HSV")
        print(f"    threshold={cs['threshold']},            # balanced score: below -> low ('red')")
        print("    mixed_frac=(0.15, 0.85),")
        print(f"    mixed_v_ratio={cs['v_ratio']},")
        print("    mixed_min_hw=0.5,")
        print("    mixed_min_px=30,")
        print(")")
        print("=" * 65)
        print(f"  [COLOUR_SPLIT Diagnostics]")
        print(f"    White Mat reference (B, G): ({cs['mat_bg'][0]:.1f}, {cs['mat_bg'][1]:.1f})")
        print(f"    RED score:     p5={cs['score_red_stats'][0]:.1f}, med={cs['score_red_stats'][1]:.1f}, p95={cs['score_red_stats'][2]:.1f}")
        print(f"    MAGENTA score: p5={cs['score_mag_stats'][0]:.1f}, med={cs['score_mag_stats'][1]:.1f}, p95={cs['score_mag_stats'][2]:.1f}")
        print(f"    Separation gap: [{cs['score_red_stats'][2]:.1f} .. {cs['score_mag_stats'][0]:.1f}] -> Recommended threshold: {cs['threshold']}")
    elif 'RED' in active_samples or 'MAGENTA' in active_samples:
        missing = 'MAGENTA' if 'RED' in active_samples else 'RED'
        print(f"\n[Tip] Annotate {missing} in this session to also output the COLOUR_SPLIT configuration block.")

    # Help note on V_max
    for color, rngs in hsv_ranges.items():
        for lo, up in rngs:
            if up[2] < 200 and color in ('GREEN', 'RED', 'BLUE', 'ORANGE', 'MAGENTA'):
                print(f"  [Tip] {color} V_max is {up[2]}. If lighting on track varies, "
                      f"consider relaxing V_max towards 255 to prevent dropouts.")
                break


class AnnotateTuner:
    def __init__(self, video_path=None, apply_filter=True, load_previous=False):
        self.video_path = video_path
        self.is_video = (video_path is not None)
        self.apply_filter = apply_filter
        self.window_name = "Polygon Colour Tuner"
        if self.is_video:
            self.window_name += f" - [{os.path.basename(video_path)}]"

        self.cap = None
        self.total_frames = 0
        self.current_frame_idx = 0
        self.is_playing = False
        self._trackbar_updating = False

        if self.is_video:
            self.cap = cv2.VideoCapture(video_path)
            if not self.cap.isOpened():
                raise RuntimeError(f"Could not open video file: {video_path}")
            self.total_frames = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
            self.fps = self.cap.get(cv2.CAP_PROP_FPS) or 30.0

        os.makedirs(SAMPLE_DIR, exist_ok=True)
        # By default, do NOT load previous npys: start 100% fresh for the current video/session
        self.samples = {}   # color -> Nx3 array
        if load_previous:
            for color in COLORS:
                npy_path = os.path.join(SAMPLE_DIR, f"{color}.npy")
                if os.path.exists(npy_path):
                    try:
                        arr = np.load(npy_path)
                        if arr.size and arr.ndim == 2 and arr.shape[1] == 3:
                            self.samples[color] = arr
                    except Exception:
                        pass
                        pass

        self.active_color_idx = 0
        self.pts = []          # in-progress polygon points [(x, y), ...]
        self.live_frozen = False
        self.current_raw_frame = None
        self.current_filtered_frame = None

        self.toast_msg = ""
        self.toast_color = (0, 255, 0)
        self.toast_until = 0.0
        self.history = []      # list of (color, px_count) for undoing accepted batches
        self.expanded_backup = None  # backup of points before auto-expansion (for 'u' revert)

    @property
    def active_color(self):
        return COLORS[self.active_color_idx]

    def show_toast(self, msg, color=(0, 255, 0), duration=2.5):
        self.toast_msg = msg
        self.toast_color = color
        self.toast_until = time.time() + duration

    def set_active_color(self, color_name):
        if color_name in COLORS:
            self.active_color_idx = COLORS.index(color_name)
            self.pts = []

    def next_color(self):
        self.active_color_idx = (self.active_color_idx + 1) % len(COLORS)
        self.pts = []

    def _mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            # If playing in video mode, clicking pauses immediately
            if self.is_video and self.is_playing:
                self.is_playing = False
            # If live and not frozen, clicking freezes immediately
            elif not self.is_video and not self.live_frozen:
                self.live_frozen = True

            self.expanded_backup = None
            self.pts.append((x, y))

    def _on_frame_trackbar(self, val):
        if self._trackbar_updating or not self.is_video:
            return
        if self.pts:
            # Active polygon selection in progress: block scrubbing and restore slider position
            self._trackbar_updating = True
            try:
                cv2.setTrackbarPos('Frame', self.window_name, self.current_frame_idx)
            except Exception:
                pass
            self._trackbar_updating = False
            self.show_toast("Finish selection first: 'a' to accept, 'r' to cancel", (0, 165, 255), 2.5)
            return
        self.seek_frame(val)

    def seek_frame(self, frame_idx):
        if not self.is_video:
            return
        frame_idx = max(0, min(frame_idx, self.total_frames - 1))
        self.current_frame_idx = frame_idx
        self.cap.set(cv2.CAP_PROP_POS_FRAMES, frame_idx)
        ret, frame = self.cap.read()
        if ret:
            self.current_raw_frame = frame
            self.current_filtered_frame = self._filter_frame(frame)

    def _filter_frame(self, frame):
        if not self.apply_filter or frame is None:
            return frame
        if USE_BILATERAL:
            return cv2.bilateralFilter(frame, BILATERAL_D, BILATERAL_SIGMA_COLOR, BILATERAL_SIGMA_SPACE)
        else:
            return cv2.GaussianBlur(frame, BLUR_KERNEL, 0)

    def accept_polygon(self):
        if len(self.pts) < 3:
            msg = "[!] Need at least 3 points to form a polygon."
            print(f"  {msg}")
            self.show_toast(msg, (0, 165, 255), 2.5)
            return

        target_frame = self.current_filtered_frame
        if target_frame is None:
            return

        mask = np.zeros(target_frame.shape[:2], dtype=np.uint8)
        cv2.fillPoly(mask, [np.array(self.pts, dtype=np.int32)], 255)
        px = target_frame[mask > 0]
        if px.size:
            color = self.active_color
            if color in self.samples and len(self.samples[color]):
                self.samples[color] = np.vstack([self.samples[color], px.reshape(-1, 3)])
            else:
                self.samples[color] = px.reshape(-1, 3)

            total_px = len(self.samples[color])
            np.save(os.path.join(SAMPLE_DIR, f"{color}.npy"), self.samples[color])
            self.history.append((color, len(px)))
            msg = f"+ {color}: added {len(px):,} px (total {total_px:,} px -> {color}.npy)"
            print(f"  {msg}")
            self.show_toast(msg, (0, 255, 0), 3.0)
        else:
            msg = "[!] No pixels enclosed inside polygon."
            print(f"  {msg}")
            self.show_toast(msg, (0, 165, 255), 2.5)

        self.pts = []
        self.expanded_backup = None

    def expand_polygon(self):
        """Auto-expand user polygon outward in LAB space until hitting color boundary (e.g. white mat)."""
        if len(self.pts) < 3:
            msg = "[!] Need at least 3 points to expand"
            print(f"  {msg}")
            self.show_toast(msg, (0, 165, 255), 2.0)
            return

        target_frame = self.current_filtered_frame
        if target_frame is None:
            return

        # Save current drawn points for 'u' revert
        self.expanded_backup = list(self.pts)

        h, w = target_frame.shape[:2]
        mask = np.zeros((h, w), dtype=np.uint8)
        cv2.fillPoly(mask, [np.array(self.pts, dtype=np.int32)], 255)

        lab = cv2.cvtColor(target_frame, cv2.COLOR_BGR2Lab)
        seed_px = lab[mask > 0]
        if len(seed_px) < 10:
            self.show_toast("[!] Seed region too small to expand", (0, 165, 255), 2.0)
            return

        med = np.median(seed_px, axis=0)
        dists_in = np.linalg.norm(seed_px.astype(float) - med, axis=1)
        p90 = np.percentile(dists_in, 90)
        # Adaptive tolerance based on seed variance (bounded between 18.0 and 30.0)
        tolerance = min(30.0, max(18.0, p90 * 1.45 + 4.0))

        color_dist = np.linalg.norm(lab.astype(float) - med, axis=2)
        valid_color = (color_dist <= tolerance).astype(np.uint8) * 255

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        curr_mask = mask.copy()
        for _ in range(25):
            dilated = cv2.dilate(curr_mask, kernel, iterations=1)
            new_mask = cv2.bitwise_and(dilated, valid_color)
            if np.array_equal(curr_mask, new_mask):
                break
            curr_mask = new_mask

        cnts, _ = cv2.findContours(curr_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            self.show_toast("[!] Expansion failed to find contour", (0, 165, 255), 2.0)
            return

        largest = max(cnts, key=cv2.contourArea)
        approx = cv2.approxPolyDP(largest, 2.0, True)
        if len(approx) < 3:
            self.show_toast("[!] Expanded contour invalid", (0, 165, 255), 2.0)
            return

        self.pts = [(int(p[0][0]), int(p[0][1])) for p in approx]
        msg = f"[Expand] Expanded boundary (press 'u' to revert, 'a' to accept)"
        print(f"  {msg}")
        self.show_toast(msg, (0, 255, 200), 3.0)

    def undo_vertex_or_expansion(self):
        """Undo auto-expansion if active; otherwise pop the last polygon vertex."""
        if self.expanded_backup is not None:
            self.pts = list(self.expanded_backup)
            self.expanded_backup = None
            msg = "[Undo] Reverted auto-expansion back to drawn polygon"
            print(f"  {msg}")
            self.show_toast(msg, (0, 165, 255), 2.0)
        elif self.pts:
            self.pts.pop()

    def undo_last_accept(self):
        """Undo / revert the most recently accepted polygon submission."""
        if not self.history:
            msg = "Undo: No submissions to revert"
            print(f"  [!] {msg}")
            self.show_toast(msg, (150, 150, 150), 2.0)
            return

        color, count = self.history.pop()
        if color in self.samples and len(self.samples[color]) >= count:
            self.samples[color] = self.samples[color][:-count]
            npy_path = os.path.join(SAMPLE_DIR, f"{color}.npy")
            if len(self.samples[color]) > 0:
                np.save(npy_path, self.samples[color])
            elif os.path.exists(npy_path):
                try:
                    os.remove(npy_path)
                except Exception:
                    pass

            rem = len(self.samples[color])
            msg = f"[Undo] Reverted accept: -{count:,} px from {color} (remaining: {rem:,} px)"
            print(f"  {msg}")
            self.show_toast(msg, (0, 165, 255), 3.0)
        else:
            self.show_toast(f"Cannot revert {color}: samples mismatch", (0, 0, 255), 2.5)

    def clear_active_color_samples(self):
        color = self.active_color
        if color in self.samples and len(self.samples[color]):
            count = len(self.samples[color])
            self.samples[color] = np.empty((0, 3), dtype=np.uint8)
            # Remove from history any entries for this color
            self.history = [h for h in self.history if h[0] != color]
            npy_path = os.path.join(SAMPLE_DIR, f"{color}.npy")
            if os.path.exists(npy_path):
                try:
                    os.remove(npy_path)
                except Exception:
                    pass
            msg = f"[x] Cleared {count:,} samples for {color}."
            print(f"  {msg}")
            self.show_toast(msg, (0, 100, 255), 2.5)
        else:
            msg = f"[x] No samples to clear for {color}."
            print(f"  {msg}")
            self.show_toast(msg, (150, 150, 150), 2.0)
        self.pts = []
        self.expanded_backup = None

    def draw_ui(self):
        if self.current_raw_frame is None:
            return None

        disp = self.current_raw_frame.copy()
        h, w = disp.shape[:2]

        # Top overlay bar (height 44)
        header = disp[:44, :].copy()
        cv2.rectangle(disp, (0, 0), (w, 44), (20, 20, 20), -1)
        cv2.addWeighted(disp[:44, :], 0.75, header, 0.25, 0, disp[:44, :])

        # Bottom overlay bar (height 22)
        footer = disp[h - 22:, :].copy()
        cv2.rectangle(disp, (0, h - 22), (w, h), (15, 15, 15), -1)
        cv2.addWeighted(disp[h - 22:, :], 0.85, footer, 0.15, 0, disp[h - 22:, :])

        # Status text line 1
        act_col = self.active_color
        act_bgr = COLOR_BGR.get(act_col, (0, 255, 0))
        pooled_n = len(self.samples.get(act_col, []))

        # Color badge
        cv2.rectangle(disp, (8, 6), (22, 20), act_bgr, -1)
        cv2.rectangle(disp, (8, 6), (22, 20), (255, 255, 255), 1)

        if self.is_video:
            status_tag = f"Frame {self.current_frame_idx + 1}/{self.total_frames} " \
                         f"[{'PLAY' if self.is_playing else 'PAUSE'}]"
        else:
            status_tag = f"[{'FROZEN' if self.live_frozen else 'LIVE'}]"

        line1 = f"  {act_col} ({pooled_n:,} px)  |  {status_tag}"
        cv2.putText(disp, line1, (26, 17), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)

        # Line 2: All colors quick summary
        summary_parts = []
        for i, c in enumerate(COLORS, 1):
            n = len(self.samples.get(c, []))
            prefix = ">" if c == act_col else ""
            summary_parts.append(f"{prefix}[{i}]{c[:3]}:{n}")
        line2 = "  ".join(summary_parts)
        cv2.putText(disp, line2, (10, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (190, 190, 190), 1, cv2.LINE_AA)

        # Footer shortcut keys
        if self.is_video:
            footer_text = "SPACE:Play/Pause | ./,:±1f | f/b:±10f | 1-6/c:Col | Click:Pt | e:Expand | a/ENTER:Accept | u:Undo | z:UndoSub | r:Clear | s:Save | q:Quit"
        else:
            footer_text = "SPACE:Freeze | 1-6/c:Col | Click:Pt | e:Expand | a/ENTER:Accept | u:Undo | z:UndoSub | r:Clear | x:ResetCol | s:Save | q:Quit"
        cv2.putText(disp, footer_text, (8, h - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.32, (200, 200, 200), 1, cv2.LINE_AA)

        # Draw polygon vertices and edges
        pts = self.pts
        if pts:
            # Filled translucent overlay when polygon >= 3 pts
            if len(pts) >= 3:
                poly_overlay = disp.copy()
                cv2.fillPoly(poly_overlay, [np.array(pts, dtype=np.int32)], act_bgr)
                cv2.addWeighted(poly_overlay, 0.35, disp, 0.65, 0, disp)

            # Polylines and vertex dots
            cv2.polylines(disp, [np.array(pts, dtype=np.int32)], len(pts) >= 3, act_bgr, 2, cv2.LINE_AA)
            for p in pts:
                cv2.circle(disp, p, 4, (0, 255, 255), -1, cv2.LINE_AA)
                cv2.circle(disp, p, 4, (0, 0, 0), 1, cv2.LINE_AA)

        # On-screen transient notification banner
        if time.time() < self.toast_until and self.toast_msg:
            (tw, th), _ = cv2.getTextSize(self.toast_msg, cv2.FONT_HERSHEY_SIMPLEX, 0.44, 1)
            tx = (w - tw) // 2
            ty = 68
            cv2.rectangle(disp, (tx - 8, ty - th - 5), (tx + tw + 8, ty + 5), (15, 15, 15), -1)
            cv2.rectangle(disp, (tx - 8, ty - th - 5), (tx + tw + 8, ty + 5), self.toast_color, 1)
            cv2.putText(disp, self.toast_msg, (tx, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.44, self.toast_color, 1, cv2.LINE_AA)

        return disp

    def run(self):
        cv2.namedWindow(self.window_name)
        cv2.setMouseCallback(self.window_name, self._mouse_callback)

        if self.is_video:
            cv2.createTrackbar('Frame', self.window_name, 0, max(1, self.total_frames - 1), self._on_frame_trackbar)
            self.seek_frame(0)
            print(f"\nLoaded video: {self.video_path}")
            print(f"Total frames: {self.total_frames}")
        else:
            print("\nLive camera stream active.")

        print("\nShortcuts:")
        if self.is_video:
            print("  SPACE         : Play / Pause (locked during selection)")
            print("  . / , (or d/p): Step 1 frame forward / backward (locked during selection)")
            print("  f / b         : Jump 10 frames forward / backward (locked during selection)")
        else:
            print("  SPACE / f     : Freeze / Unfreeze live frame")
        print("  1 - 6 / c     : Select colour (1:RED, 2:GREEN, 3:BLUE, 4:MAGENTA, 5:ORANGE, 6:BLACK)")
        print("  Left-Click    : Add polygon point (pauses video)")
        print("  e             : Auto-expand polygon to edge of obstacle (LAB color distance)")
        print("  a / ENTER     : Accept polygon & pool pixels for active colour")
        print("  u             : Undo auto-expansion (if expanded) OR undo last vertex")
        print("  z             : Undo / revert last accepted polygon submission")
        print("  r             : Reset / clear current in-progress polygon")
        print("  x             : Clear all pooled pixels for current colour")
        print("  s             : Save & Print HSV_RANGES block now")
        print("  q / ESC       : Quit & Print exported configuration blocks\n")

        running = True
        while running:
            # Fetch frame
            if self.is_video:
                if self.is_playing:
                    ret, frame = self.cap.read()
                    if not ret:
                        # loop video to frame 0
                        self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                        self.current_frame_idx = 0
                        ret, frame = self.cap.read()
                    else:
                        self.current_frame_idx += 1

                    if ret:
                        self.current_raw_frame = frame
                        self.current_filtered_frame = self._filter_frame(frame)
                        self._trackbar_updating = True
                        cv2.setTrackbarPos('Frame', self.window_name, self.current_frame_idx)
                        self._trackbar_updating = False
            else:
                if not self.live_frozen or self.current_raw_frame is None:
                    raw = camera.capture_frame() if camera else None
                    if raw is not None:
                        self.current_raw_frame = raw
                        self.current_filtered_frame = self._filter_frame(raw)
                    else:
                        time.sleep(0.03)
                        continue

            disp = self.draw_ui()
            if disp is not None:
                cv2.imshow(self.window_name, disp)

            # Frame rate delay
            wait_time = int(1000 / self.fps) if (self.is_video and self.is_playing) else 25
            key = cv2.waitKey(wait_time) & 0xFF

            if key in (27, ord('q')):
                running = False
                break
            elif key == ord(' '):
                if self.is_video:
                    if self.pts:
                        self.show_toast("Active selection! Press 'a' to accept or 'r' to cancel", (0, 165, 255), 2.5)
                    else:
                        self.is_playing = not self.is_playing
                else:
                    self.live_frozen = not self.live_frozen
            elif key in (ord('a'), 10, 13):
                # 'a' or ENTER: Exclusively accepts the polygon
                self.accept_polygon()
            elif key in (ord('.'), ord('d'), ord('n'), ord('>'), 83, 65363) and self.is_video:
                if self.pts:
                    self.show_toast("Active selection! Press 'a' to accept or 'r' to cancel", (0, 165, 255), 2.5)
                else:
                    self.is_playing = False
                    self.seek_frame(self.current_frame_idx + 1)
                    self._trackbar_updating = True
                    cv2.setTrackbarPos('Frame', self.window_name, self.current_frame_idx)
                    self._trackbar_updating = False
            elif key in (ord(','), ord('p'), ord('<'), ord('['), 81, 65361) and self.is_video:
                if self.pts:
                    self.show_toast("Active selection! Press 'a' to accept or 'r' to cancel", (0, 165, 255), 2.5)
                else:
                    self.is_playing = False
                    self.seek_frame(self.current_frame_idx - 1)
                    self._trackbar_updating = True
                    cv2.setTrackbarPos('Frame', self.window_name, self.current_frame_idx)
                    self._trackbar_updating = False
            elif key in (ord('f'), ord(']')):
                if self.is_video:
                    if self.pts:
                        self.show_toast("Active selection! Press 'a' to accept or 'r' to cancel", (0, 165, 255), 2.5)
                    else:
                        self.is_playing = False
                        self.seek_frame(self.current_frame_idx + 10)
                        self._trackbar_updating = True
                        cv2.setTrackbarPos('Frame', self.window_name, self.current_frame_idx)
                        self._trackbar_updating = False
                else:
                    self.live_frozen = not self.live_frozen
            elif key == ord('b') and self.is_video:
                if self.pts:
                    self.show_toast("Active selection! Press 'a' to accept or 'r' to cancel", (0, 165, 255), 2.5)
                else:
                    self.is_playing = False
                    self.seek_frame(self.current_frame_idx - 10)
                    self._trackbar_updating = True
                    cv2.setTrackbarPos('Frame', self.window_name, self.current_frame_idx)
                    self._trackbar_updating = False
            elif key in COLOR_SHORTCUTS:
                if self.pts:
                    self.show_toast("Active selection! Press 'a' to accept or 'r' to cancel", (0, 165, 255), 2.5)
                else:
                    self.set_active_color(COLOR_SHORTCUTS[key])
            elif key == ord('c'):
                if self.pts:
                    self.show_toast("Active selection! Press 'a' to accept or 'r' to cancel", (0, 165, 255), 2.5)
                else:
                    self.next_color()
            elif key == ord('e'):
                self.expand_polygon()
            elif key == ord('z'):
                self.undo_last_accept()
            elif key == ord('u'):
                self.undo_vertex_or_expansion()
            elif key == ord('r'):
                if self.pts:
                    self.pts = []
                    self.expanded_backup = None
                    self.show_toast("Selection cancelled", (150, 150, 150), 1.5)
            elif key == ord('x'):
                self.clear_active_color_samples()
            elif key == ord('s'):
                export_and_report(self.samples, ref_frame=self.current_raw_frame)

        # Cleanup
        if self.cap:
            self.cap.release()
        if not self.is_video and camera:
            camera.cleanup()
        cv2.destroyAllWindows()

        # Final export
        export_and_report(self.samples, ref_frame=self.current_raw_frame)


def main():
    parser = argparse.ArgumentParser(
        description="Polygon-annotation colour tuner for Obstacle Challenge (supports video & live camera)."
    )
    parser.add_argument(
        "video",
        nargs="?",
        default=None,
        help="Path to video file (.mp4) or obstacle run directory (e.g. obstacle/2026-11-05_18-00-10). "
             "If omitted and not --live, auto-detects latest video if available, or starts live camera."
    )
    parser.add_argument(
        "--live",
        action="store_true",
        help="Force live camera stream."
    )
    parser.add_argument(
        "-l", "--latest",
        action="store_true",
        help="Automatically use the latest video from obstacle/ folder."
    )
    parser.add_argument(
        "-n", "--no-filter",
        dest="filter",
        action="store_false",
        help="Skip preprocessing filter during playback/annotation."
    )
    parser.add_argument(
        "--resume", "--load-previous",
        dest="load_previous",
        action="store_true",
        help="Load previously saved .npy samples from color_samples/. Default is False (starts 100%% clean)."
    )
    args = parser.parse_args()

    video_target = None
    if args.live:
        video_target = None
    elif args.video:
        resolved = resolve_video_path(args.video)
        if not resolved:
            print(f"ERROR: Video or directory not found: {args.video}")
            sys.exit(1)
        video_target = resolved
    elif args.latest:
        latest = find_latest_video()
        if not latest:
            print("ERROR: No video files found in obstacle/ folder.")
            sys.exit(1)
        video_target = latest
    else:
        # Default behavior when no arguments are provided:
        # If camera initializes successfully, use live camera.
        # Otherwise, fall back to the latest recorded video.
        if camera and camera.initialize():
            print("Using Live Camera.")
            video_target = None
        else:
            latest = find_latest_video()
            if latest:
                print(f"Camera unavailable or not specified. Auto-loading latest video: {latest}")
                video_target = latest
            else:
                print("ERROR: No camera available and no video files found.")
                sys.exit(1)

    tuner = AnnotateTuner(
        video_path=video_target,
        apply_filter=args.filter,
        load_previous=args.load_previous
    )
    tuner.run()


if __name__ == '__main__':
    main()

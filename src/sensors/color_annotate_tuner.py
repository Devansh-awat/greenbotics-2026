"""Polygon-annotation colour tuner.

Workflow
--------
For each colour in COLORS the live camera view is shown. You:
  1. Press SPACE to freeze a frame.
  2. Left-click >=3 points to draw a polygon around the coloured block.
  3. Press 'a' to accept the polygon  -> every pixel inside it is pooled.
  4. Press SPACE / 'n' to go back to live and capture another angle.
  5. Repeat from several angles / distances, then press 'c' for the next colour.

At the end the script pools ALL annotated pixels per colour (across every angle),
computes a robust range per channel using percentiles + a margin, and prints
ready-to-paste HSV_RANGES / LAB_RANGES lines for src/obstacle_challenge config.

It also:
  * auto-handles the HSV red hue wraparound (emits RED_1 / RED_2),
  * guarantees the RED and MAGENTA ranges do not overlap (they are similar),
  * saves the raw pooled pixels to .npy so ranges can be recomputed later
    with different margins WITHOUT re-annotating.

Keys
----
  live:    SPACE/f freeze frame | c next colour | q quit
  frozen:  left-click add point | a accept polygon | u undo point
           r reset polygon | SPACE/n back to live | c next colour | q quit

Note: frames from camera.capture_frame() are treated as BGR, matching the
existing src/sensors/color_tuning.py convention.
"""

import cv2
import numpy as np
import os
import time

from src.sensors import camera

# --- Configuration ---------------------------------------------------------
# Order matters only for display. Skip any colour by pressing 'c' with no
# samples collected for it.
COLORS = ['RED', 'GREEN', 'BLUE', 'ORANGE', 'MAGENTA', 'BLACK']

P_LOW, P_HIGH = 2.0, 98.0          # percentile bounds (reject outlier pixels)
MARGIN_HUE = 3                     # extra slack on hue (0-179)
MARGIN_OTHER = 10                  # extra slack on S, V, L, a, b (0-255)
BLUR_KERNEL = (5, 5)              # must match production preprocessing
SAMPLE_DIR = os.path.join(os.path.dirname(__file__), 'color_samples')

HSV_MAX = np.array([179, 255, 255])
LAB_MAX = np.array([255, 255, 255])

# Colours that should be kept disjoint from each other (similar hues).
DECONFLICT_PAIRS = [('RED', 'MAGENTA')]
# ---------------------------------------------------------------------------


def _clip(vals, hi):
    return np.clip(np.round(vals), 0, hi).astype(int)


def _percentile_range(channel, margin, hi):
    """Robust [low, high] for a single non-wrapping channel."""
    lo = np.percentile(channel, P_LOW) - margin
    up = np.percentile(channel, P_HIGH) + margin
    return max(0, int(round(lo))), min(int(hi), int(round(up)))


def _hue_range(hue):
    """Return one or two (lo, hi) hue ranges, handling the 0/180 wraparound.

    The hue circle is cut at its largest empty gap so the occupied arc becomes
    contiguous; percentiles are taken on the rotated arc and mapped back.
    """
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
    # Arc starts just after the gap that ends at occ rolled by (g+1).
    offset = int(occ[(g + 1) % occ.size])

    rot = (hue - offset) % 180
    lo_rot = np.percentile(rot, P_LOW) - MARGIN_HUE
    hi_rot = np.percentile(rot, P_HIGH) + MARGIN_HUE
    lo = int(round(lo_rot)) + offset
    hi = int(round(hi_rot)) + offset

    lo_m = lo % 180
    hi_m = hi % 180
    if hi - lo >= 179:                       # spans everything
        return [(0, 179)]
    if lo_m <= hi_m:
        return [(lo_m, hi_m)]
    # Wraps across the seam -> two ranges.
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
    # lower-median cluster keeps upper[c] <= bnd; higher keeps lower[c] >= bnd.
    lo_lower, lo_upper = ranges[lo_name][0]
    if lo_upper[c] > bnd:
        lo_upper[c] = bnd
    hi_lower, hi_upper = ranges[hi_name][0]
    if hi_lower[c] < bnd:
        hi_lower[c] = bnd
    return c


def _deconflict_hsv_hue(ranges, conv, a_name, b_name):
    """Trim HSV hue so RED/MAGENTA don't overlap (hue folded to remove wrap)."""
    def fold(h):
        h = h.astype(int)
        return np.where(h >= 90, h - 180, h)        # -90..89, red ~ 0

    A = fold(conv[a_name][:, 0])
    B = fold(conv[b_name][:, 0])
    bnd_fold = (np.median(A) + np.median(B)) / 2.0
    bnd = int(round(bnd_fold)) % 180                # back to 0..179

    lower_is_a = np.median(A) < np.median(B)
    below, above = (a_name, b_name) if lower_is_a else (b_name, a_name)

    # 'above' (closer to hue 0/180 from the high side) -> its high-hue range lower bound >= bnd
    for lower, upper in ranges[above]:
        if lower[0] >= 90 and lower[0] < bnd:       # the wrapped (~180) range
            lower[0] = bnd
    # 'below' (e.g. magenta ~160) -> upper hue <= bnd
    for lower, upper in ranges[below]:
        if upper[0] > 90 and upper[0] > bnd:
            upper[0] = bnd
    return bnd


# ---------------------------------------------------------------------------
# Annotation UI
# ---------------------------------------------------------------------------
_state = {'pts': []}


def _on_mouse(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN and param['frozen']:
        _state['pts'].append((x, y))


def annotate_color(color, window):
    """Collect pooled BGR pixels for one colour.

    Returns (Nx3 BGR array, quit_flag). The pooled pixels collected so far are
    ALWAYS returned -- including when the user presses 'q' to quit -- so a colour
    in progress is never silently discarded.
    """
    pooled = []
    quit_flag = False
    param = {'frozen': False}
    cv2.setMouseCallback(window, _on_mouse, param)
    frozen_frame = None

    while True:
        if not param['frozen']:
            raw = camera.capture_frame()
            if raw is None:
                time.sleep(0.05)
                continue
            frame = cv2.GaussianBlur(raw, BLUR_KERNEL, 0)
            disp = frame.copy()
            n = sum(len(p) for p in pooled)
            cv2.putText(disp, f"{color}: {n} px pooled  [SPACE freeze | c next | q quit]",
                        (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        else:
            disp = frozen_frame.copy()
            pts = _state['pts']
            for p in pts:
                cv2.circle(disp, p, 3, (0, 255, 0), -1)
            if len(pts) >= 2:
                cv2.polylines(disp, [np.array(pts)], len(pts) >= 3, (0, 255, 0), 1)
            cv2.putText(disp, f"{color}: click >=3 pts  [a accept | u undo | r reset | n live | q quit]",
                        (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        cv2.imshow(window, disp)
        key = cv2.waitKey(20) & 0xFF

        if key in (ord(' '), ord('f')):
            if not param['frozen']:
                frozen_frame = frame.copy()
                _state['pts'] = []
                param['frozen'] = True
            else:
                param['frozen'] = False        # back to live for a new angle
        elif key == ord('u') and param['frozen']:
            if _state['pts']:
                _state['pts'].pop()
        elif key == ord('r') and param['frozen']:
            _state['pts'] = []
        elif key == ord('a') and param['frozen']:
            pts = _state['pts']
            if len(pts) >= 3:
                mask = np.zeros(frozen_frame.shape[:2], dtype=np.uint8)
                cv2.fillPoly(mask, [np.array(pts, dtype=np.int32)], 255)
                px = frozen_frame[mask > 0]
                if px.size:
                    pooled.append(px.reshape(-1, 3))
                    print(f"  + {color}: added {len(px)} px (total "
                          f"{sum(len(p) for p in pooled)})")
                _state['pts'] = []
                param['frozen'] = False
            else:
                print("  ! need at least 3 points")
        elif key == ord('c'):       # done with this colour, keep samples
            break
        elif key == ord('q'):       # quit, but still keep this colour's samples
            quit_flag = True
            break

    cv2.setMouseCallback(window, lambda *a: None)
    if pooled:
        return np.vstack(pooled), quit_flag
    return np.empty((0, 3), dtype=np.uint8), quit_flag


def _fmt(name, lower, upper):
    return (f"    'LOWER_{name}': np.array({list(map(int, lower))}), "
            f"'UPPER_{name}': np.array({list(map(int, upper))}),")


def report(hsv_ranges, lab_ranges):
    print("\n" + "=" * 60)
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
        print(_fmt(name, rngs[0][0], rngs[0][1]))
    print("}")
    print("=" * 60)


def main():
    if not camera.initialize():
        print("Failed to initialize camera.")
        return
    os.makedirs(SAMPLE_DIR, exist_ok=True)

    window = "Polygon Colour Tuner"
    cv2.namedWindow(window)

    samples = {}        # color -> Nx3 BGR pixels
    for color in COLORS:
        print(f"\n=== Annotating {color} ===")
        px, quit_flag = annotate_color(color, window)
        if len(px):
            samples[color] = px
            np.save(os.path.join(SAMPLE_DIR, f"{color}.npy"), px)
            print(f"  saved {len(px)} px -> {color}.npy")
        else:
            print(f"  (no samples for {color}, skipped)")
        if quit_flag:
            print("  (quit requested -- samples above were kept)")
            break

    camera.cleanup()
    cv2.destroyAllWindows()

    if not samples:
        print("No samples collected; nothing to compute.")
        return

    hsv_ranges, lab_ranges = {}, {}
    hsv_conv, lab_conv = {}, {}
    for color, px in samples.items():
        h, hconv = compute_hsv(px, color)
        l, lconv = compute_lab(px)
        hsv_ranges[color] = h
        lab_ranges[color] = l
        hsv_conv[color] = hconv
        lab_conv[color] = lconv

    # Keep similar colours disjoint.
    for a, b in DECONFLICT_PAIRS:
        if a in samples and b in samples:
            cb = _deconflict_hsv_hue(hsv_ranges, hsv_conv, a, b)
            cl = _deconflict_lab(lab_ranges, lab_conv, a, b)
            print(f"\nDeconflicted {a}/{b}: HSV hue boundary={cb}, "
                  f"LAB split on channel {'Lab'[cl]}")

    report(hsv_ranges, lab_ranges)
    print(f"\nRaw pixels saved in {SAMPLE_DIR}/ (recompute later without re-annotating).")


if __name__ == '__main__':
    main()

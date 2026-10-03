"""Find the base of a side wall: the line where the black wall meets the white mat.

Why the base and not the top: the wall's TOP edge is only visible when whatever is
behind the wall is brighter than the wall. A dark venue floor (or dark clothes, a
grey hall) merges with the wall and the top edge disappears. The BASE is always a
white-mat -> black-wall transition, because both the mat and the wall are fixed by
the rules -- whatever is beyond the wall never enters the measurement.

Method, per column, walking UP from the bottom of the ROI:
  1. the column must start on mat (bright bottom rows), else it's the robot body;
  2. the base is the first row that is darker than WALL_DARK_RATIO x that column's
     own mat brightness and stays dark for WALL_DARK_RUN rows (relative, so it does
     not care about venue lighting, and thin blue/orange lines don't qualify);
  3. if magenta is hit first, or sits just above the dark run (the shadow under a
     parking wall), the column is occluded and skipped.
Only base points inside a narrow band of rows near the robot are used, and the
result is their x position at a reference row. The base is NOT fitted with a line
and extrapolated: near the robot it is often visible only as a short stretch in a
frame corner, where lens distortion bends it, and a line through that stretch
points far too steeply up the frame (seen on 2026-10-02_17-03-07). The wall's top
edge is not used either: dark furniture behind a wall against a bright room
produces convincing "top edges" nowhere near the wall.

The measurement is local, like the original edge-ROI follower: closer to the wall,
or yawed toward it, moves the base toward the centre of the frame.
"""

import cv2
import numpy as np

WALL_DARK_RATIO = 0.45   # dark = V below this fraction of the column's mat V
WALL_DARK_RUN = 6        # rows the dark run must last to count as wall
MAT_MIN_V = 80           # a column whose bottom is darker than this isn't on mat
MAT_REF_ROWS = 25        # bottom rows used to measure the column's mat brightness
SHADOW_CHECK_ROWS = 20   # magenta this close above the dark run = parking-wall shadow
COLUMN_STEP = 2          # sample every Nth column (halves the cost, plenty of points)
MIN_BAND_POINTS = 4      # fewer base points than this inside the band -> not seen
# Base slope |dy/dx| near the robot, used only to shift points by <= half the band
# height to the reference row. Measured 0.85-1.3 on parking footage; 1.0 => the
# worst-case shift error is ~4 px at the band edges.
BAND_SLOPE = 1.0

LOWER_MAGENTA = np.array([145, 73, 40])
UPPER_MAGENTA = np.array([175, 255, 255])


def wall_base_points(frame, x0, x1, y_top, y_bot):
    """Return (xs, ys) of the wall base in full-frame coords, one per valid column."""
    roi = frame[y_top:y_bot, x0:x1:COLUMN_STEP]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    v = hsv[..., 2]
    h, w = v.shape

    mat = np.percentile(v[h - MAT_REF_ROWS:], 70, axis=0)        # (w,)
    dark = v < (mat * WALL_DARK_RATIO)[None, :]
    # sustained[r] = dark[r] and the WALL_DARK_RUN-1 rows above it are dark too.
    sustained = dark.copy()
    for k in range(1, WALL_DARK_RUN):
        sustained[k:] &= dark[:-k]
    sustained[:WALL_DARK_RUN - 1] = False
    magenta = cv2.inRange(hsv, LOWER_MAGENTA, UPPER_MAGENTA) > 0

    # Lowest row (closest to the robot) of each kind, per column; -1 if none.
    flip = slice(None, None, -1)
    has_dark = sustained.any(axis=0)
    base = np.where(has_dark, h - 1 - np.argmax(sustained[flip], axis=0), -1)
    has_mag = magenta.any(axis=0)
    first_mag = np.where(has_mag, h - 1 - np.argmax(magenta[flip], axis=0), -1)

    # Magenta hit before the wall, or magenta right above a dark run (its shadow).
    occluded = has_mag & (first_mag >= base - SHADOW_CHECK_ROWS)
    ok = has_dark & (mat >= MAT_MIN_V) & ~occluded

    xs = (x0 + np.arange(w) * COLUMN_STEP)[ok]
    ys = (base[ok] + y_top)
    return xs, ys


def wall_base_x(frame, side, x0, x1, ref_y, band, y_top=60, y_bot=330):
    """x of the near `side` ('right'|'left') wall's base at row `ref_y`, or None.

    Uses only base points with |y - ref_y| <= band (median of them, each shifted to
    ref_y along BAND_SLOPE). Returns (x, n_points, xs, ys) -- the points for drawing.
    """
    xs, ys = wall_base_points(frame, x0, x1, y_top, y_bot)
    k = np.abs(ys - ref_y) <= band
    xs, ys = xs[k], ys[k]
    if len(xs) < MIN_BAND_POINTS:
        return None, len(xs), xs, ys
    slope = BAND_SLOPE if side == 'right' else -BAND_SLOPE
    x = float(np.median(xs + (ref_y - ys) / slope))
    return x, len(xs), xs, ys

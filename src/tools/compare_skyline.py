"""Compare four arena-boundary ("skyline") methods on recorded raw footage.

Writes one annotated mp4 per input with all four boundaries drawn over the raw
frame, plus a stats table so the choice isn't purely eyeballed.

  A  wall-top     (yellow)   production BEFORE D -- top edge of the near wall's run.
                             Needs black-vs-background contrast at the TOP of the wall,
                             which a grey surround does not provide.
  B  mat-edge     (green)    bottom edge of the same run = the wall/mat junction.
                             Only ever needs black-vs-white contrast.
  C  floor-blob   (magenta)  seeded connected blob of POSITIVE floor colours
                             (white + red + green + magenta + orange + blue), per-column
                             top edge. Outside the arena is a separate blob that does
                             not touch the nose seed -- unless one pixel bridges them.
  D  floor-contig (orange)   C, but each column walks UP from its own lowest floor
                             pixel and stops at the first break, so a hairline bridge
                             elsewhere in the frame cannot drag the boundary up.

All four are band-local (row 0 == ARENA_Y_TOP) and all three get the same
unmeasured-column fill and the same sliding median, so the comparison is of the
measurement, not of the post-processing.

Usage:
    python3 -m src.tools.compare_skyline                  # 5 most recent raw.mp4
    python3 -m src.tools.compare_skyline obstacle/<ts>/raw.mp4 ...
    python3 -m src.tools.compare_skyline -n 3 --max-frames 400
"""

import argparse
import glob
import os
import sys

import cv2
import numpy as np

from src.vision import pipeline as P

# Method C's white threshold. Not in tuning.py -- no white mask existed before this.
# Measured off the mat around the nose seed on three runs: S 4-15, V 167-190, against
# UPPER_BLACK's V ceiling of 70. Re-measure for a new venue with video_color_tuning.py.
WHITE_S_MAX = 60
WHITE_V_MIN = 110

# Rows at the top of the band probed to decide "is this column mat all the way up".
TOP_PROBE_ROWS = 6

# The wall-scan knobs, as they were in pipeline.py before method D shipped. Kept here
# so A and B stay runnable as baselines now that production no longer provides them.
MIN_WALL_THICK = 8
WALL_GAP_SEAL = 3
MAX_WALL_RUN = 160
_WALL_KERNEL = np.ones((MIN_WALL_THICK, 1), np.uint8)
_WALL_CLOSE_KERNEL = np.ones((WALL_GAP_SEAL, 1), np.uint8)

COL_A = (0, 255, 255)      # yellow
COL_B = (0, 255, 0)        # green
COL_C = (255, 0, 255)      # magenta
COL_D = (255, 160, 0)      # cyan-blue


def _fill_and_smooth(measured, ok, force_open=None):
    """Shared post-processing: interpolate unmeasured columns from their neighbours,
    then sliding-median. Returns (sky, info) or (None, info).

    Interpolation, not a global median: the mat edge is the projection of a straight
    physical line, so a column with no measurement inherits the line its neighbours
    describe. np.interp edge-clamps past the last measured column, which extends that
    line to the frame border instead of inventing a value there.

    `force_open` columns are pinned to 0 and kept OUT of the interpolation. Not every
    unmeasured column is unknown: on this camera pose the far wall frequently sits
    ABOVE ARENA_Y_TOP, so the column holds mat all the way to the top of the band and
    the honest answer is "all of it is arena". Interpolating those from a neighbour
    that DID see a wall would gate out real floor, which is the exact failure the
    old global-median fill was removed for.
    """
    cols = np.arange(P.FRAME_WIDTH)
    if force_open is not None:
        ok = ok & ~force_open
    info = {'interp': 1.0 - (ok.sum() + (0 if force_open is None else force_open.sum())) / cols.size}
    if not ok.any():
        if force_open is not None and force_open.all():
            return np.zeros(P.FRAME_WIDTH, np.int32), info   # whole band is arena
        return None, info
    sky = np.interp(cols, cols[ok], measured[ok]).astype(np.int32)
    if force_open is not None:
        sky[force_open] = 0
    sky = P._sliding_median(sky, P.ARENA_SKY_SMOOTH)
    return np.clip(sky, 0, P.ARENA_BAND_H), info


def _thick_black(cs_slice):
    """mask_black -> vertical CLOSE (seal glare pinholes) -> vertical OPEN (drop any
    run shorter than MIN_WALL_THICK). Identical to production's step 3 preamble."""
    mask_black = cv2.inRange(cs_slice, P.LOWER_BLACK, P.UPPER_BLACK)
    mask_black = cv2.bitwise_and(
        mask_black, cv2.bitwise_not(cv2.inRange(cs_slice, P.LOWER_GREEN, P.UPPER_GREEN))
    )
    sealed = cv2.morphologyEx(mask_black, cv2.MORPH_CLOSE, _WALL_CLOSE_KERNEL)
    return cv2.morphologyEx(sealed, cv2.MORPH_OPEN, _WALL_KERNEL) > 0


def _white(cs_slice):
    return cv2.inRange(cs_slice, np.array([0, 0, WHITE_V_MIN]),
                       np.array([180, WHITE_S_MAX, 255]))


def sky_wall_top(cs_slice):
    """A: the pre-D production scan -- top edge of the near wall's black run.

    Reimplemented here rather than called: build_arena_mask_from_prepared() IS method D
    now, so calling it would compare D against itself. This is that function's step 3
    as it stood at the commit before the switch, so `-n 5` still shows what changed.
    """
    thick_b = _thick_black(cs_slice)
    has = thick_b.any(axis=0)
    lowest = (P.ARENA_BAND_H - 1) - np.argmax(thick_b[::-1], axis=0)
    chain = thick_b | (P._ARENA_ROWS > lowest)
    run = np.minimum.accumulate(chain[::-1], axis=0)[::-1]
    wall_top = P.ARENA_BAND_H - run.sum(axis=0)
    trusted = has & ((lowest - wall_top) <= MAX_WALL_RUN)
    fill = int(np.median(wall_top[trusted])) if trusted.any() else 0
    sky = np.where(has, np.where(trusted, wall_top, fill), 0).astype(np.int32)
    sky = P._sliding_median(sky + P.ARENA_TOP_MARGIN, P.ARENA_SKY_SMOOTH)
    return np.clip(sky, 0, P.ARENA_BAND_H), {'interp': 0.0}


def sky_mat_edge(cs_slice, wall_px=0):
    """B: bottom row of the black run = wall/mat junction."""
    thick_b = _thick_black(cs_slice)
    has = thick_b.any(axis=0)
    # argmax on an all-False column returns 0, so `lowest` is garbage where has=False.
    lowest = (P.ARENA_BAND_H - 1) - np.argmax(thick_b[::-1], axis=0)
    # A run reaching the bottom of the band means the junction is below the band
    # (nose into a wall, hard into a corner) -- no usable measurement either.
    ok = has & (lowest < P.ARENA_BAND_H - 1)
    # No black run AND mat at the top of the band == the wall is above ARENA_Y_TOP,
    # so the whole column is arena. Distinct from "no black run and no mat", which is
    # the grey-room-through-a-gap case and does need a neighbour's value.
    top_is_mat = (_white(cs_slice)[:TOP_PROBE_ROWS] > 0).mean(axis=0) > 0.5
    return _fill_and_smooth(lowest - wall_px, ok, force_open=~has & top_is_mat)


def _floor_blob(cs_slice):
    """The seeded floor blob, holes preserved. Shared by C and D."""
    cand = _white(cs_slice)
    for lo, hi in ((P.LOWER_RED_1, P.UPPER_RED_1), (P.LOWER_RED_2, P.UPPER_RED_2),
                   (P.LOWER_GREEN, P.UPPER_GREEN), (P.LOWER_MAGENTA, P.UPPER_MAGENTA),
                   (P.LOWER_ORANGE, P.UPPER_ORANGE), (P.LOWER_BLUE, P.UPPER_BLUE)):
        # Full-slice, unlike compute_colour_masks_only() which confines orange/blue to
        # the line ROI -- here they are floor pixels and must not be holes in the blob.
        cand = cv2.bitwise_or(cand, cv2.inRange(cs_slice, lo, hi))
    cand = cv2.morphologyEx(cand, cv2.MORPH_CLOSE, P.ARENA_CLOSE_KERNEL)

    contours, _ = cv2.findContours(cand, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    seeded = [c for c in contours
              if cv2.pointPolygonTest(c, P.ARENA_SEED_LOCAL, False) >= 0]
    if not seeded:
        return None
    filled = np.zeros_like(cand)
    cv2.drawContours(filled, [max(seeded, key=cv2.contourArea)], -1, 255, -1)
    # fill-then-AND, exactly as production does at pipeline.py:380 -- the fill closes
    # the outline, the AND puts the interior holes (the wall, black objects) back.
    return cv2.bitwise_and(filled, cand) > 0


def sky_floor_blob(cs_slice):
    """C: blob's topmost pixel per column (np.argmax)."""
    blob = _floor_blob(cs_slice)
    if blob is None:
        return None, {'interp': 1.0}
    top = np.argmax(blob, axis=0)          # first True row; 0 where the column is empty
    # A column whose blob reaches row 0 self-reports "open to the top of the band",
    # so C needs no separate force_open rule -- top==0 already says it.
    return _fill_and_smooth(top.astype(np.int32), blob.any(axis=0))


def sky_floor_contig(cs_slice):
    """D: C, but the blob must be UNBROKEN from the seed row up.

    C's argmax accepts any blob pixel however it got there, so one hairline bridge
    through the wall drags the boundary to row 0 for every column the room touches.
    Walking up from a known-floor row instead means a leak has to be a continuous
    column of floor-coloured pixels through the wall, not merely connected to one
    somewhere else in the frame. Costs one accumulate, needs no black mask.

    The walk starts at each column's OWN lowest floor pixel, not at a fixed row. The
    bottom of the band is the robot's chassis, which is not floor-coloured and would
    stop a bottom-row walk instantly; a fixed seed row is worse still -- when the robot
    is close to a wall the wall covers that row and the column reports nothing, gating
    out the mat that is plainly visible below it.
    """
    blob = _floor_blob(cs_slice)
    if blob is None:
        return None, {'interp': 1.0}
    lowest = (P.ARENA_BAND_H - 1) - np.argmax(blob[::-1], axis=0)
    # The walk must START on our own floor. Where the wall fills the column, the only
    # blob left is whatever leaked in beyond it, and walking up from THAT reports the
    # room as floor. The mat under the nose is always at or below the seed row, so a
    # column whose lowest floor pixel sits above it is not showing us our own floor.
    ok = blob.any(axis=0) & (lowest >= int(P.ARENA_SEED_LOCAL[1]))
    chain = blob | (P._ARENA_ROWS > lowest)      # force everything below `lowest` True
    run = np.minimum.accumulate(chain[::-1], axis=0)[::-1]
    top = P.ARENA_BAND_H - run.sum(axis=0)
    return _fill_and_smooth(top.astype(np.int32), ok, force_open=(top == 0) & ok)


def _draw(frame, sky, colour):
    if sky is None:
        return
    pts = np.stack([np.arange(P.FRAME_WIDTH, dtype=np.int32),
                    sky.astype(np.int32) + P.ARENA_Y_TOP], axis=1)
    cv2.polylines(frame, [pts], False, colour, 2)


class Stats:
    """Per-method tallies.

    `dead`      frames where the method produced no boundary at all -- production
                falls back to a full pass-through mask, so nothing is gated.
    `open`      mean share of columns pinned at the top of the band. Not automatically
                a failure on this footage: the far wall often sits above ARENA_Y_TOP,
                and an open column there is the correct answer.
    `interp`    mean share of columns whose value was borrowed from neighbours.
    `gated`     mean share of the band's area excluded. The number that decides whether
                the mask is doing any work at all.
    `jitter`    mean per-column change frame to frame. A boundary that twitches gates
                blocks in and out between frames.
    `leak`      mean share of walled columns whose boundary sits ABOVE the top of that
                column's black wall run -- i.e. the method admitted pixels that are
                provably beyond the arena wall. Method-agnostic, computed from the
                black mask purely as a referee; no method is told about it.
    """

    def __init__(self):
        self.frames = self.dead = 0
        self.open = []
        self.interp = []
        self.gated = []
        self.jitter = []
        self.leak = []
        self.prev = None

    def add(self, sky, info, leak=None):
        self.frames += 1
        if sky is None:
            self.dead += 1
            self.prev = None
            return
        self.open.append(float((sky <= 0).mean()))
        self.interp.append(info['interp'])
        self.gated.append(float(sky.sum()) / (P.ARENA_BAND_H * sky.size))
        if leak is not None:
            self.leak.append(leak)
        if self.prev is not None:
            self.jitter.append(float(np.mean(np.abs(sky - self.prev))))
        self.prev = sky

    def row(self, name):
        m = lambda a: 100.0 * float(np.mean(a)) if a else float('nan')
        j = float(np.mean(self.jitter)) if self.jitter else float('nan')
        return (f"  {name:<14} dead {100.0 * self.dead / max(self.frames, 1):5.1f}%   "
                f"open cols {m(self.open):5.1f}%   "
                f"interp cols {m(self.interp):5.1f}%   "
                f"band gated {m(self.gated):5.1f}%   "
                f"jitter {j:5.2f} px/frame   "
                f"LEAK {m(self.leak):5.1f}%")


def process(path, out_dir, max_frames, wall_px):
    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        print(f"!! cannot open {path}")
        return None
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    name = os.path.basename(os.path.dirname(path)) or os.path.basename(path)
    out_path = os.path.join(out_dir, f"skyline_{name}.mp4")
    writer = None
    stats = {'A wall-top': Stats(), 'B mat-edge': Stats(),
             'C floor-blob': Stats(), 'D floor-contig': Stats()}

    n = 0
    while max_frames is None or n < max_frames:
        ok, frame = cap.read()
        if not ok:
            break
        n += 1
        cs = P.prepare_colour_slice(frame)
        results = [('A wall-top', sky_wall_top(cs), COL_A),
                   ('B mat-edge', sky_mat_edge(cs, wall_px), COL_B),
                   ('C floor-blob', sky_floor_blob(cs), COL_C),
                   ('D floor-contig', sky_floor_contig(cs), COL_D)]

        # Referee: top of each column's sustained black run. A boundary above it is
        # admitting pixels beyond the wall, whatever method produced it.
        thick = _thick_black(cs)
        walled = thick.any(axis=0)
        wall_top = np.where(walled, np.argmax(thick, axis=0), 0)

        vis = frame.copy()
        for label, (sky, info), colour in results:
            leak = None
            if sky is not None and walled.any():
                leak = float(((sky < wall_top) & walled).sum()) / walled.sum()
            stats[label].add(sky, info, leak)
            _draw(vis, sky, colour)

        for y in (P.ARENA_Y_TOP, P.ARENA_Y_BOTTOM):
            cv2.line(vis, (0, y), (P.FRAME_WIDTH, y), (90, 90, 90), 1)
        cv2.circle(vis, P.ARENA_SEED_PT, 4, (255, 255, 255), -1)
        for i, (label, (sky, _info), colour) in enumerate(results):
            text = label + ("  --" if sky is None else "")
            cv2.putText(vis, text, (8, 16 + 16 * i), cv2.FONT_HERSHEY_SIMPLEX,
                        0.45, colour, 1, cv2.LINE_AA)
        cv2.putText(vis, f"{name}  f{n}", (P.FRAME_WIDTH - 210, 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)

        if writer is None:
            writer = cv2.VideoWriter(out_path, cv2.VideoWriter_fourcc(*'mp4v'),
                                     fps, (vis.shape[1], vis.shape[0]))
        writer.write(vis)

    cap.release()
    if writer is not None:
        writer.release()
    print(f"\n{path}  ({n} frames)  -> {out_path}")
    for label in stats:
        print(stats[label].row(label))
    return stats


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("videos", nargs="*", help="raw.mp4 paths (default: N most recent)")
    ap.add_argument("-n", type=int, default=5, help="how many recent runs (default 5)")
    ap.add_argument("--out", default="skyline_compare", help="output directory")
    ap.add_argument("--max-frames", type=int, default=None, help="stop after N frames")
    ap.add_argument("--wall-height", action="store_true",
                    help="size WALL_BAND_PX for this venue instead of rendering video")
    ap.add_argument("--bench", action="store_true",
                    help="time the methods instead of rendering video")
    ap.add_argument("--wall-px", type=int, default=0,
                    help="method B: px of wall kept above the mat edge (default 0)")
    args = ap.parse_args()

    videos = args.videos
    if not videos:
        found = [p for p in glob.glob("obstacle/*/raw.mp4") if os.path.getsize(p) > 1 << 20]
        videos = sorted(found, key=os.path.getmtime)[-args.n:]
    if not videos:
        sys.exit("No raw.mp4 found under obstacle/.")

    if args.wall_height:
        wall_height(videos)
        return
    if args.bench:
        bench(videos)
        return

    os.makedirs(args.out, exist_ok=True)
    totals = {}
    for path in videos:
        s = process(path, args.out, args.max_frames, args.wall_px)
        if s:
            totals[path] = s

    print("\n=== totals across %d videos ===" % len(totals))
    for label in ('A wall-top', 'B mat-edge', 'C floor-blob', 'D floor-contig'):
        merged = Stats()
        for s in totals.values():
            m = s[label]
            merged.frames += m.frames
            merged.dead += m.dead
            merged.open += m.open
            merged.interp += m.interp
            merged.gated += m.gated
            merged.jitter += m.jitter
            merged.leak += m.leak
        print(merged.row(label))


def bench(videos, reps=30):
    """Time each method, and scale laptop numbers to the Pi using the run logs.

    Absolute numbers off a laptop mean nothing for the Pi, and a fixed fudge factor is
    a guess. Instead: time the WHOLE inline pipeline here, divide the Pi's measured
    `vision p50` into it, and use that ratio for the per-method deltas -- one number,
    derived from the same work on both machines.

    PI_VISION_P50 is the median of `vision p50` across the 2026-08-24 runs'
    obstacle_output.txt. Re-derive it after any pipeline change -- it was measured
    with the OLD wall-top arena mask, so until you have logged a run on the current
    code the scale is a little pessimistic:
        grep -o 'vision p50 *[0-9.]*' obstacle/*/obstacle_output.txt | sort -k3 -n

    Sanity check, taken while the wall-top scan was still live: this predicted 3.4 ms
    for it, against the 3.3 ms pool.py:3 documents. Prediction for the switch to D:
    `vision p50` should land near 8.5 ms, still well inside the 17.9 ms frame period.
    """
    import timeit
    PI_VISION_P50 = 7.75

    frames, slices = [], []
    for path in videos:
        cap = cv2.VideoCapture(path)
        for i in range(60, 600, 60):
            cap.set(cv2.CAP_PROP_POS_FRAMES, i)
            ok, f = cap.read()
            if ok:
                frames.append(f)
                slices.append(P.prepare_colour_slice(f))
        cap.release()
    if not frames:
        sys.exit("no frames to bench")

    pool, P.vision_pool = P.vision_pool, None      # inline, so both branches are timed
    try:
        t = lambda fn, items, r: (1000 * timeit.timeit(lambda: [fn(x) for x in items],
                                                       number=r) / (r * len(items)))
        full = t(P.process_video_frame, frames, max(reps // 2, 5))
    finally:
        P.vision_pool = pool
    scale = PI_VISION_P50 / full
    print(f"{len(frames)} frames x {reps} reps")
    print(f"  full process_video_frame here : {full:6.3f} ms")
    print(f"  Pi vision p50 (from logs)     : {PI_VISION_P50:6.3f} ms  ->  scale x{scale:.2f}\n")

    # The boundary functions return only `sky`; production also allocates the two
    # full-frame masks, so it is ~0.2 ms above bare D. Compare like for like.
    base = t(P.build_arena_mask_from_prepared, slices, reps)
    print(f"  {'production (D)':<22} here {base:6.3f} ms  ->  Pi ~{base * scale:5.2f} ms   "
          f"<- what ships")
    for name, fn in (("A wall-top (was)", sky_wall_top), ("B mat-edge", sky_mat_edge),
                     ("C floor-blob", sky_floor_blob), ("D floor-contig (bare)", sky_floor_contig)):
        v = t(fn, slices, reps)
        print(f"  {name:<22} here {v:6.3f} ms  ->  Pi ~{v * scale:5.2f} ms   "
              f"({(v - base) * scale:+5.2f} ms vs production)")
    print(f"\n  Camera period at 56 fps = {1000 / 56:.1f} ms. The delta lands on the arena")
    print("  worker, the longer of the pool's two branches, so it is a real frame cost.")


def wall_height(videos, max_frames=300):
    """Size WALL_BAND_PX for this venue.

    The wall stands ON the mat, so its black run starts at the floor boundary. Two
    numbers decide the cap: how tall that run actually gets (set the cap above it, or
    you eat real wall and your PD gains are wrong), and how often it escapes the band
    entirely (that is the surround thresholding as black and merging with the wall --
    the thing the cap exists to stop).

    Columns near the top of the band have no headroom, so a short far wall trivially
    "escapes". Only columns with >=120 px above the boundary are counted for that.
    """
    H, R = [], []
    for path in videos:
        cap = cv2.VideoCapture(path)
        for _ in range(max_frames):
            ok, f = cap.read()
            if not ok:
                break
            cs = P.prepare_colour_slice(f)
            blob = _floor_blob(cs)
            if blob is None:
                continue
            black = cv2.inRange(cs, P.LOWER_BLACK, P.UPPER_BLACK) > 0
            lo = (P.ARENA_BAND_H - 1) - np.argmax(blob[::-1], axis=0)
            ft = P.ARENA_BAND_H - np.minimum.accumulate(
                (blob | (P._ARENA_ROWS > lo))[::-1], axis=0)[::-1].sum(axis=0)
            wt = P.ARENA_BAND_H - np.minimum.accumulate(
                (black | (P._ARENA_ROWS >= ft))[::-1], axis=0)[::-1].sum(axis=0)
            h = ft - wt
            m = (ft > 0) & (h > 0)
            H.append(h[m]); R.append(ft[m])
        cap.release()
    if not H:
        sys.exit("no wall runs found")
    H = np.concatenate(H); R = np.concatenate(R)
    room = R >= 120
    escaped = room & (H >= R - 2)
    clean = room & ~escaped
    print(f"{H.size} column-samples; {room.sum()} with >=120px headroom\n")
    print(f"  runs escaping the band (surround merged into the wall): "
          f"{100 * escaped.sum() / max(room.sum(), 1):.1f}%")
    print("     >5% here means the cap is doing real work in this venue.\n")
    q = lambda a, p: np.percentile(a, p)
    print(f"  attached run height, clean runs:  p50 {q(H[clean],50):.0f}  "
          f"p90 {q(H[clean],90):.0f}  p99 {q(H[clean],99):.0f}  max {H[clean].max():.0f}")
    print("\n  cost of each candidate WALL_BAND_PX (share of real wall pixels lost):")
    for c in (50, 80, 100, 120, 160, 200):
        lost = np.clip(H[clean] - c, 0, None).sum() / max(H[clean].sum(), 1)
        print(f"    {c:3d} px -> clips {100 * (H[clean] > c).mean():5.1f}% of columns, "
              f"loses {100 * lost:5.1f}% of wall pixels")
    print("\n  A FLAT cap must clear the near-wall p99 everywhere, which leaves far columns")
    print("  (where a merged background actually runs away) barely capped at all. Scale it")
    print("  with the boundary row instead -- cap = WALL_BAND_SLOPE * row + WALL_BAND_PX:")
    for k, c in ((0.9, 25), (1.1, 15), (0.8, 40)):
        cap = k * R[clean] + c
        at = (k * np.array([40, 120, 190]) + c).astype(int)
        print(f"    SLOPE {k:.1f} PX {c:3d} -> clips {100 * (H[clean] > cap).mean():4.1f}% of real "
              f"wall | cap at rows 40/120/190 = {at}")
    print("\n  Pick the tightest line that still clips ~0%. Too tight and the PD gains, which")
    print("  are tuned against the uncapped area, are integrating a different signal.")


def demo():
    """Self-check: synthetic band, black wall over white mat, left 40 columns blanked.

    Asserts the two things the fill has to get right -- the border columns inherit the
    first measured column rather than defaulting open, and no column is left at 0."""
    band = np.zeros((P.ARENA_BAND_H, P.FRAME_WIDTH, 3), np.uint8)
    band[:, :] = (90, 10, 180)                       # HSV-ish "white mat"
    for x in range(P.FRAME_WIDTH):
        edge = 60 + x // 40                          # mat edge drifts down to the right
        band[:edge, x] = (0, 0, 20)                  # black wall above it
    band[:, :40] = (0, 0, 20)                        # left border: all wall, no mat

    sky, info = sky_mat_edge(band)
    assert sky is not None
    assert (sky > 0).all(), "a column was left fully open"
    assert abs(int(sky[0]) - int(sky[45])) <= 2, f"border not edge-clamped: {sky[:5]} vs {sky[45]}"
    assert 0.0 < info['interp'] < 0.2, f"expected ~6% interpolated columns, got {info}"
    assert sky[-1] > sky[50], "boundary should drift down to the right"

    # ...and the case the force_open rule exists for: no wall in the band at all, mat
    # to the top. Must stay fully open, NOT interpolate a boundary from nowhere.
    allmat = np.zeros((P.ARENA_BAND_H, P.FRAME_WIDTH, 3), np.uint8)
    allmat[:, :] = (90, 10, 180)
    sky2, _ = sky_mat_edge(allmat)
    assert sky2 is not None and (sky2 == 0).all(), "wall-above-band column got gated"
    print(f"demo ok: {info['interp']:.1%} columns filled, sky[0]={sky[0]} sky[639]={sky[-1]}")


if __name__ == "__main__":
    if "--demo" in sys.argv:
        demo()
    else:
        main()

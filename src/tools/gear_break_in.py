#!/usr/bin/env python3
"""
gear_break_in.py

Gear break-in test for the drive motor. Runs the motor at full duty in a repeating
cycle so the gears wear in:

    forward 30 s -> brake -> reverse 30 s -> brake -> forward 30 s -> ...

Runs until Ctrl+C. Shows encoder info, raw (unsmoothed) motor RPM, and a line graph
of RPM from the start of the run until now. The graph is drawn with the kitty
graphics protocol, so it needs to run in kitty (or another terminal that supports
it). Pass --no-graph to skip the image and only print the text readout.

Run from repo root:
    python3 -m src.tools.gear_break_in
    python3 -m src.tools.gear_break_in --no-graph
"""

import sys
import time
import base64
import argparse
import io

from PIL import Image, ImageDraw, ImageFont

from src.motors import motor


# --- Test parameters ---
PHASE_SECONDS = 30.0      # run time per direction
DUTY = 100.0              # full duty
SWITCH_PAUSE_S = 0.5      # brake pause between directions (not recorded in graph)
SPINUP_SKIP_S = 0.5       # after each restart, wheel is spinning up from 0: keep it out of the graph
SAMPLE_HZ = 20.0          # encoder sampling rate; every sample is kept (no smoothing)
DISPLAY_HZ = 2.0          # terminal + graph refresh rate

# --- Graph parameters ---
GRAPH_CELL_W = 10         # pixels per terminal cell (used to size the image)
GRAPH_CELL_H = 20
GRAPH_COLS = 100          # image width in terminal cells
GRAPH_ROWS = 22           # image height in terminal cells
FWD_COLOR = (80, 200, 255)
REV_COLOR = (255, 160, 60)
BG_COLOR = (24, 24, 28)
GRID_COLOR = (60, 60, 68)
TEXT_COLOR = (200, 200, 200)

# Number of text lines printed above the graph (the image is placed below them)
TOP_LINES = 14


def render_graph(ts, rpms, fwd, max_rpm):
    """Renders the RPM-vs-run-time line graph as PNG bytes.

    ts    : active run time per sample (s), breaks excluded
    rpms  : absolute motor RPM per sample (unsmoothed)
    fwd   : True where the sample was taken while driving forward
    Each pixel column draws the min..max envelope of its samples, so short jerks
    and spikes stay visible even when the run is long.
    """
    w = GRAPH_COLS * GRAPH_CELL_W
    h = GRAPH_ROWS * GRAPH_CELL_H
    left, right, top, bottom = 70, 12, 10, 34
    pw = w - left - right
    ph = h - top - bottom

    img = Image.new("RGB", (w, h), BG_COLOR)
    draw = ImageDraw.Draw(img)
    try:
        font = ImageFont.load_default(size=14)
    except TypeError:
        font = ImageFont.load_default()

    t_end = max(ts[-1], 30.0) if ts else 30.0
    y_max = max(max_rpm, max(rpms) * 1.05) if rpms else max_rpm

    def xpix(t):
        return left + (t / t_end) * (pw - 1)

    def ypix(v):
        return top + ph - (v / y_max) * ph

    # Grid + Y labels (0, 25, 50, 75, 100 % of y_max)
    for frac in (0.0, 0.25, 0.5, 0.75, 1.0):
        y = ypix(frac * y_max)
        draw.line([(left, y), (w - right, y)], fill=GRID_COLOR, width=1)
        draw.text((6, y - 8), f"{frac * y_max:6.0f}", fill=TEXT_COLOR, font=font)
    draw.text((6, 4), "rpm", fill=TEXT_COLOR, font=font)

    # X ticks every nice step
    step = 10 if t_end <= 120 else 30 if t_end <= 600 else 60 if t_end <= 1800 else 300
    t_tick = 0
    while t_tick <= t_end:
        x = xpix(t_tick)
        draw.line([(x, top + ph), (x, top + ph + 4)], fill=TEXT_COLOR, width=1)
        draw.text((x - 12, top + ph + 8), f"{t_tick:.0f}", fill=TEXT_COLOR, font=font)
        t_tick += step
    draw.text((w - right - 260, h - 20), "run time, s (breaks excluded)",
              fill=TEXT_COLOR, font=font)

    if ts:
        # Per pixel column: min, max, first, last value and direction
        ncol = pw
        cmin = [None] * ncol
        cmax = [None] * ncol
        cfirst = [None] * ncol
        clast = [None] * ncol
        cfwd = [True] * ncol
        for t, v, f in zip(ts, rpms, fwd):
            c = min(ncol - 1, max(0, int((t / t_end) * (ncol - 1))))
            if cmin[c] is None:
                cmin[c] = cmax[c] = cfirst[c] = v
            else:
                if v < cmin[c]:
                    cmin[c] = v
                if v > cmax[c]:
                    cmax[c] = v
            clast[c] = v
            cfwd[c] = f

        # Vertical envelope per column, then connect each column to the next
        prev = None
        for c in range(ncol):
            if cmin[c] is None:
                continue
            x = left + c
            color = FWD_COLOR if cfwd[c] else REV_COLOR
            y_lo, y_hi = ypix(cmax[c]), ypix(cmin[c])
            if y_hi - y_lo < 1:
                draw.point((x, y_lo), fill=color)
            else:
                draw.line([(x, y_lo), (x, y_hi)], fill=color, width=1)
            if prev is not None:
                draw.line([(x - 1, ypix(prev)), (x, ypix(cfirst[c]))], fill=color, width=1)
            prev = clast[c]

    buf = io.BytesIO()
    img.save(buf, format="PNG", compress_level=1)
    return buf.getvalue()


def kitty_image_sequence(png_bytes, image_id=1):
    """Kitty graphics protocol: delete the previous image, then transmit and display
    the PNG at the cursor, scaled to GRAPH_COLS x GRAPH_ROWS cells."""
    data = base64.standard_b64encode(png_bytes).decode("ascii")
    chunks = [data[i:i + 4096] for i in range(0, len(data), 4096)]
    out = [f"\x1b_Ga=d,d=I,i={image_id},q=2\x1b\\"]
    for i, chunk in enumerate(chunks):
        more = 1 if i < len(chunks) - 1 else 0
        if i == 0:
            ctrl = f"a=T,f=100,i={image_id},c={GRAPH_COLS},r={GRAPH_ROWS},q=2,C=1,m={more}"
        else:
            ctrl = f"m={more}"
        out.append(f"\x1b_G{ctrl};{chunk}\x1b\\")
    return "".join(out)


def draw_screen(state, enc, rpm, cfg, show_graph):
    """Clears the terminal, prints the text readout and the kitty graph below it."""
    now = time.perf_counter()
    elapsed = now - state["start"]
    phase_elapsed = now - state["phase_start"]
    mins, secs = divmod(int(elapsed), 60)
    hrs, mins = divmod(mins, 60)
    label = "BRAKE" if state["braking"] else state["direction"].upper()

    lines = [
        "=" * 78,
        f"  GEAR BREAK-IN   elapsed {hrs:02d}:{mins:02d}:{secs:02d}   "
        f"cycles completed: {state['cycles']}   (Ctrl+C to stop)",
        "=" * 78,
        f"  Phase     : {label:<8} duty {DUTY:.0f}%   "
        f"{phase_elapsed:5.1f} / {PHASE_SECONDS:.0f} s",
        f"  Motor RPM : {rpm:+8.1f}   (raw, unsmoothed; max {motor.MAX_MOTOR_RPM:.0f})",
        f"  Encoder   : pos {enc.position:>10d} counts   dist {enc.distance:>10.1f} mm",
        f"              {cfg['cpr']:.2f} counts/rev   wheel {cfg['wheel_mm']:.1f} mm",
        f"  Revs      : fwd {state['revs']['forward']:>10.2f}   rev {state['revs']['reverse']:>10.2f}",
        f"  Peak RPM  : fwd {state['peak']['forward']:>10.1f}   rev {state['peak']['reverse']:>10.1f}",
        "",
        "  RPM vs run time (start -> now)  cyan = forward, orange = reverse",
    ]
    lines = lines[:TOP_LINES]
    while len(lines) < TOP_LINES:
        lines.append("")

    out = ["\x1b[H\x1b[2J" + "\n".join(lines)]
    if show_graph:
        if state["ts"]:
            png = render_graph(state["ts"], state["rpms"], state["fwd"], motor.MAX_MOTOR_RPM)
        else:
            png = render_graph([0.0], [0.0], [True], motor.MAX_MOTOR_RPM)
        out.append(f"\x1b[{TOP_LINES + 1};1H")
        out.append(kitty_image_sequence(png))
    out.append(f"\x1b[{TOP_LINES + GRAPH_ROWS + 1};1H")
    out.append("  Ctrl+C = stop and brake")
    sys.stdout.write("".join(out))
    sys.stdout.flush()


def start_motor(direction):
    if direction == "forward":
        motor.forward(DUTY)
    else:
        motor.reverse(DUTY)


def run(show_graph):
    print("Initializing motor & encoder...")
    if not motor.initialize():
        print("FATAL: motor driver failed to initialize.")
        return 1
    enc = motor.encoder
    if enc is None:
        print("FATAL: encoder not available, RPM cannot be measured.")
        motor.cleanup()
        return 1

    cfg = {
        "cpr": enc.counts_per_rev,
        "wheel_mm": enc.wheel_diameter_mm,
    }
    print(f"Encoder: {cfg['cpr']:.2f} counts/rev, wheel {cfg['wheel_mm']:.1f} mm, "
          f"max motor RPM {motor.MAX_MOTOR_RPM:.0f}")
    print(f"Plan: {PHASE_SECONDS:.0f} s forward -> brake -> {PHASE_SECONDS:.0f} s reverse "
          f"-> brake -> repeat. Duty {DUTY:.0f}%. Ctrl+C to stop.")
    time.sleep(2.0)

    now = time.perf_counter()
    state = {
        "start": now,
        "phase_start": now,
        "direction": "forward",
        "braking": False,
        "cycles": 0,
        "revs": {"forward": 0.0, "reverse": 0.0},
        "peak": {"forward": 0.0, "reverse": 0.0},
        # Full run history, one entry per sample. Active run time only (no brake pauses).
        "ts": [],
        "rpms": [],
        "fwd": [],
        "skip_until": now + SPINUP_SKIP_S,
    }

    last_pos = enc.position
    last_t = now
    active_t = 0.0
    rpm = 0.0
    next_draw = now
    start_motor(state["direction"])

    try:
        while True:
            now = time.perf_counter()
            pos = enc.position
            dt = now - last_t
            if dt > 0:
                # Raw motor RPM for this sample, no smoothing (wheel rpm -> motor rpm via gear ratio)
                rpm = (pos - last_pos) / cfg["cpr"] / dt * 60.0 / motor.EXTERNAL_GEAR_RATIO
                d = state["direction"]
                state["revs"][d] += abs(pos - last_pos) / cfg["cpr"]
                state["peak"][d] = max(state["peak"][d], abs(rpm))
                # Graph only gets samples once the wheel is back at speed
                if now >= state["skip_until"]:
                    active_t += dt
                    state["ts"].append(active_t)
                    state["rpms"].append(abs(rpm))
                    state["fwd"].append(d == "forward")
                last_pos, last_t = pos, now

            if now - state["phase_start"] >= PHASE_SECONDS:
                # Switch direction: brake, pause (not recorded), flip, resume at full duty
                motor.brake()
                state["braking"] = True
                time.sleep(SWITCH_PAUSE_S)
                if state["direction"] == "forward":
                    state["direction"] = "reverse"
                else:
                    state["direction"] = "forward"
                    state["cycles"] += 1
                state["braking"] = False
                start_motor(state["direction"])
                state["phase_start"] = time.perf_counter()
                state["skip_until"] = state["phase_start"] + SPINUP_SKIP_S
                last_pos = enc.position
                last_t = state["phase_start"]
                rpm = 0.0
                continue

            if now >= next_draw:
                draw_screen(state, enc, rpm, cfg, show_graph)
                next_draw = now + 1.0 / DISPLAY_HZ

            time.sleep(1.0 / SAMPLE_HZ)

    except KeyboardInterrupt:
        pass
    finally:
        motor.brake()
        motor.cleanup()

    print("\n\nStopped and braked.")
    print(f"Cycles completed : {state['cycles']}")
    print(f"Revs forward     : {state['revs']['forward']:.2f}")
    print(f"Revs reverse     : {state['revs']['reverse']:.2f}")
    print(f"Peak fwd RPM     : {state['peak']['forward']:.1f}")
    print(f"Peak rev RPM     : {state['peak']['reverse']:.1f}")
    print(f"Samples recorded : {len(state['ts'])}")
    return 0


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Gear break-in: full duty fwd/rev cycles")
    parser.add_argument("--no-graph", action="store_true",
                        help="text readout only, no kitty graphics")
    args = parser.parse_args()
    sys.exit(run(show_graph=not args.no_graph))

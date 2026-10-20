"""Replay the obstacle-run rules (src/obstacle_challenge/rules.py) with no robot.

Feeds saved frames or recorded detections through the SAME build_world / count_turns /
decide code that main.py runs, and prints, frame by frame, which rule decided and
what it commanded. Use it to check a new or changed rule before driving:

  - does my rule fire on the frames where it should (and not elsewhere)?
  - is the steering angle the right sign and roughly the right size?
  - did my change alter anything I did not mean to? (--changed)

Inputs (pick one):

  python3 -m src.tools.replay obstacle/<run>          # a run recorded with main.py -v
  python3 -m src.tools.replay obstacle/<run>/raw.mp4.overlay.jsonl
  python3 -m src.tools.replay dataset/20260609_013542 # images: files starting with this
  python3 -m src.tools.replay "dataset/*.jpg"         # images: a glob
  python3 -m src.tools.replay my_frames/              # images: a folder

  A recorded run replays the detections the robot logged -- exact. Images run through
  the vision pipeline -- also exact (they are lossless), so this is what to use when
  you changed OBJECTS / HSV ranges / ROIs. Use capture_dataset.py to take images of a
  new object.

Options:

  --open           replay the OPEN challenge (src/open_challenge/rules.py, open
                   config ROIs). Images only -- the open challenge records no raw video.
  --dir cw|ccw     driving direction (default: from the recording, else clockwise;
                   with --open: learned from the first floor line, as on the robot)
  --frames A:B     only replay frames A..B-1 (still runs all frames before A, so the
                   rules' memory is right)
  --rule NAME      only print frames where this rule decided (e.g. red, wall, close:red)
  --changed        recorded runs only: print only frames where the replayed angle
                   differs from what the robot recorded (by more than 1 degree)
  --summary        only print the per-rule summary
  --out FILE.mp4   also write an annotated video (needs raw.mp4 next to a recording)
  --start-s S      seconds into the run at the first replayed frame (default 6.0) --
                   only matters for time-based rules such as CLOSE_SWERVE 'after_s'

Limits -- read these:

  * OPEN LOOP. The frames come from a robot that steered the way the OLD code told
    it to. Replay says "on this picture, the rules would command X". It cannot say
    what happens next, because the robot would have moved differently.
  * Recordings hold every VIDEO_EVERY_N-th frame (default every 2nd), so rule memory
    that counts frames (post-pass clip, D-terms, turn cooldown) runs at half rate.
    Expect small angle differences against the recorded values even for unchanged
    rules; a changed rule shows up as a large or systematic difference.
  * The wall-gap aim of the red/green rules needs pure_black_mask, which recordings
    do not store, so on recordings those rules fall back to the ideal-line steering.
    Use images to test the gap aim.
"""

import argparse
import glob
import json
import os
import sys
from collections import Counter

import cv2

from src.obstacle_challenge import rules
from src.open_challenge import config as open_config
from src.open_challenge import rules as open_rules
from src.obstacle_challenge.tuning import CAMERA_FPS, VIDEO_FOURCC
from src.vision import pipeline as vision
from src.vision.pipeline import annotate_video_frame, process_video_frame, record_to_detections

DIRS = {'cw': 'clockwise', 'ccw': 'counter-clockwise',
        'clockwise': 'clockwise', 'counter-clockwise': 'counter-clockwise'}


def _recorded_value(debug, prefix):
    for item in debug or []:
        if item.startswith(prefix):
            try:
                return float(item[len(prefix):].rstrip('ms'))
            except ValueError:
                return None
    return None


def load_recording(path):
    """Yields (label, frame-or-None, detections, recorded) from an overlay log.
    Direction-detection frames at the start of the run are skipped."""
    if os.path.isdir(path):
        path = os.path.join(path, 'raw.mp4.overlay.jsonl')
    raw_path = path[:-len('.overlay.jsonl')]
    recs = []
    with open(path) as fh:
        for line in fh:
            if line.strip():
                rec = json.loads(line)
                debug = rec.get('debug', [])
                if 'det' in rec and not any(str(d).startswith('StartDir') for d in debug):
                    recs.append(rec)
    direction = recs[-1].get('dir') if recs else None
    return recs, direction, raw_path


def iter_recording(recs, raw_path, want_frames):
    cap = cv2.VideoCapture(raw_path) if want_frames and os.path.exists(raw_path) else None
    pos = 0
    for rec in recs:
        frame = None
        if cap is not None:
            # Records are in video order; read forward to this record's frame.
            while pos <= rec['v']:
                ok, frame = cap.read()
                pos += 1
                if not ok:
                    frame = None
                    break
        cam_frame = _recorded_value(rec.get('debug'), 'F:')
        recorded = {'angle': _recorded_value(rec.get('debug'), 'Angle:'),
                    'rpm': _recorded_value(rec.get('debug'), 'RPM:')}
        yield (f"v{rec['v']}" + (f"/F{int(cam_frame)}" if cam_frame is not None else ''),
               frame, record_to_detections(rec['det']), recorded, cam_frame)
    if cap is not None:
        cap.release()


def image_paths(spec):
    if os.path.isdir(spec):
        paths = glob.glob(os.path.join(spec, '*.jpg')) + glob.glob(os.path.join(spec, '*.png'))
    elif any(c in spec for c in '*?['):
        paths = glob.glob(spec)
    else:
        paths = glob.glob(spec + '*.jpg') + glob.glob(spec + '*.png')
    return sorted(paths)


def iter_images(paths):
    vision.vision_pool = None
    for p in paths:
        frame = cv2.imread(p)
        if frame is None or frame.shape[:2] != (vision.FRAME_HEIGHT, vision.FRAME_WIDTH):
            print(f"  skipping {p}: unreadable or not {vision.FRAME_WIDTH}x{vision.FRAME_HEIGHT}",
                  file=sys.stderr)
            continue
        yield os.path.basename(p), frame, process_video_frame(frame), {}, None


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('source')
    ap.add_argument('--open', action='store_true')
    ap.add_argument('--dir', choices=sorted(DIRS))
    ap.add_argument('--frames', default=None)
    ap.add_argument('--rule', default=None)
    ap.add_argument('--changed', action='store_true')
    ap.add_argument('--summary', action='store_true')
    ap.add_argument('--out', default=None)
    ap.add_argument('--start-s', type=float, default=6.0)
    args = ap.parse_args(argv)

    recorded_dir = None
    R = open_rules if args.open else rules
    if args.open:
        vision.configure(open_config)   # open-challenge ROIs and arena mode
    if args.open and (args.source.endswith('.overlay.jsonl') or os.path.exists(
            os.path.join(args.source, 'raw.mp4.overlay.jsonl'))):
        sys.exit("--open replays images only (the open challenge records no raw video)")
    if args.source.endswith('.overlay.jsonl') or os.path.exists(
            os.path.join(args.source, 'raw.mp4.overlay.jsonl')):
        recs, recorded_dir, raw_path = load_recording(args.source)
        if not recs:
            sys.exit(f"no drive frames with detections in {args.source}")
        frames = iter_recording(recs, raw_path, want_frames=bool(args.out))
        kind = f"recording ({len(recs)} frames)"
    else:
        paths = image_paths(args.source)
        if not paths:
            sys.exit(f"no images match {args.source}")
        frames = iter_images(paths)
        kind = f"images ({len(paths)} files)"
        if args.changed:
            sys.exit("--changed needs a recorded run (images have no recorded angle)")

    direction = DIRS[args.dir] if args.dir else (recorded_dir if recorded_dir in DIRS else 'clockwise')
    lo, hi = 0, None
    if args.frames:
        a, _, b = args.frames.partition(':')
        lo, hi = int(a or 0), (int(b) if b else None)

    if args.open and not args.dir:
        direction = None                # learned from the first floor line
    print(f"replaying {args.source}: {kind}, {'open challenge' if args.open else 'obstacle challenge'}, "
          f"{direction or 'direction from first line'}")
    print("rules:", " > ".join(r.__name__ for r in R.RULES))
    if not args.summary:
        print(f"{'#':>5} {'frame':<26} {'rule':<14} {'angle':>7} {'rpm':>6} {'turns':>5}"
              f" {'rec.angle':>9}  notes")

    writer = None
    mem = R.Memory()
    if args.open:
        mem.direction = direction
    counts, changed = Counter(), 0
    first_cam = None
    for i, (label, frame, det, recorded, cam_frame) in enumerate(frames):
        if hi is not None and i >= hi:
            break
        if cam_frame is not None:
            first_cam = cam_frame if first_cam is None else first_cam
            t = args.start_s + (cam_frame - first_cam) / CAMERA_FPS
        else:
            t = args.start_s + i / CAMERA_FPS
        if args.open:
            world = R.build_world(det, frame, mem, t)
            R.learn_direction(world, mem)
            direction = mem.direction
        else:
            world = R.build_world(det, direction, t)
        new_turn = R.count_turns(world, mem)
        cmd = R.decide(world, mem)
        if i < lo:
            continue

        counts[cmd.rule] += 1
        rec_angle = recorded.get('angle')
        differs = rec_angle is not None and abs(float(cmd.angle) - rec_angle) > 1.0
        changed += differs
        show = (not args.summary
                and (args.rule is None or cmd.rule == args.rule)
                and (not args.changed or differs))
        if show:
            notes = []
            if getattr(cmd, 'swerve', None) is not None:
                notes.append(f"SWERVE {cmd.swerve:+.0f}")
            if new_turn:
                notes.append("TURN")
            if getattr(cmd, 'target_x', None) is not None:
                notes.append(f"aim x={cmd.target_x}")
            if args.open:
                notes.append(f"wallrow={world.wall_row} dir={direction or '?'}"
                             + (" CORNER-SPEED" if cmd.corner_speed else ""))
            notes += [d for d in cmd.debug if 'Clip' in d or 'Cap' in d]
            rec_s = f"{rec_angle:9.0f}" if rec_angle is not None else f"{'':9}"
            print(f"{i:5d} {label:<26.26} {cmd.rule:<14} {float(cmd.angle):+7.1f} {cmd.rpm:6.0f}"
                  f" {mem.turns:5d} {rec_s}{'  *' if differs else '   '}{' '.join(notes)}")

        if args.out and frame is not None:
            if writer is None:
                h, w = frame.shape[:2]
                writer = cv2.VideoWriter(args.out, cv2.VideoWriter_fourcc(*VIDEO_FOURCC), 10.0, (w, h))
            debug = cmd.debug + [f"Angle:{int(cmd.angle)}", f"RPM:{int(cmd.rpm)}",
                                 f"Turns:{mem.turns}", f"Rule:{cmd.rule}", f"#{i}"]
            writer.write(annotate_video_frame(frame, det, direction, debug_info=debug,
                                              visual_target_x=getattr(cmd, 'target_x', None),
                                              visual_target_line=getattr(cmd, 'target_line', None)))
    if writer is not None:
        writer.release()
        print(f"\nwrote {args.out}")
    elif args.out:
        print("\n--out: no frames to write (a recording needs raw.mp4 next to it)")

    total = sum(counts.values())
    print(f"\n{total} frames replayed, {mem.turns} turns counted")
    for rule, n in counts.most_common():
        print(f"  {rule:<14} {n:6d}  ({100.0 * n / max(total, 1):.0f}%)")
    if recorded_dir is not None:
        print(f"  frames where the angle differs from the recording by > 1 deg: {changed}"
              f" (some are expected -- see 'Limits' in this tool's help)")


if __name__ == '__main__':
    try:
        main()
    except BrokenPipeError:      # output piped into `head` -- not an error
        sys.stderr.close()

"""Before/after tests for the surprise-rule refactor.

The refactor moved the obstacle-run decisions out of main.py into rules.py, and made
the vision pipeline table-driven (tuning.OBJECTS). Neither may change what the robot
does. These tests prove it by running the ORIGINAL code -- read straight out of git at
ORIGINAL_REV, so nothing here is a hand-copied reference that could drift -- next to
the new code, on the same inputs, and comparing every frame.

  test_vision_matches_original   pipeline.py at ORIGINAL_REV vs now, on dataset/*.jpg
  test_vision_pool_matches_original   same, through the two-process VisionPool
  test_new_object_entry          adding OBJECTS entries detects new colours (inline and
                                 pooled) without touching the existing detections
  test_custom_entry_open         a kind='custom' entry (own roi) works in the open challenge
  test_target_false_entry        a pillar entry with target=False is never the target
  test_rules_match_original      main.py's per-frame loop at ORIGINAL_REV (motor,
                                 servo and camera stubbed) vs rules.decide() +
                                 main.actuate(), on dataset frames and on recorded
                                 runs in obstacle/*/
  test_open_vision_matches_original   the pipeline configured for the open challenge
  test_open_rules_match_original the open challenge's loop at ORIGINAL_REV vs
                                 open_challenge/rules.py + its main.actuate()

Run on the Pi (CPU only, nothing touches hardware -- but not during a live run):

    python3 -m pytest tests/test_refactor_equivalence.py -v
  or, without pytest:
    python3 -m tests.test_refactor_equivalence

Once the refactored code is the accepted baseline (driven on the robot), these tests
have done their job; they keep passing as long as ORIGINAL_REV is in git history and
nothing changes behaviour. A deliberate behaviour change WILL fail them -- that is
the point: check the diff is the one you meant, then move ORIGINAL_REV forward.
"""

import copy
import glob
import importlib.util
import json
import math
import os
import subprocess
import sys
import types
from collections import deque

import cv2
import numpy as np

from src.obstacle_challenge import main as obstacle_main
from src.obstacle_challenge import rules
from src.obstacle_challenge import tuning as T
from src.open_challenge import config as OC
from src.open_challenge import main as open_main
from src.open_challenge import rules as open_rules
from src.vision import pipeline as P

ORIGINAL_REV = '5a74e19'   # last commit before the refactor

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATASET_GLOB = os.path.join(REPO_ROOT, 'dataset', '*.jpg')
RUN_LOGS_GLOB = os.path.join(REPO_ROOT, 'obstacle', '*', 'raw.mp4.overlay.jsonl')
MAX_VISION_FRAMES = int(os.environ.get('MAX_VISION_FRAMES', 400))
MAX_RUN_LOGS = int(os.environ.get('MAX_RUN_LOGS', 25))


def _git_show(path):
    return subprocess.check_output(
        ['git', '-C', REPO_ROOT, 'show', f'{ORIGINAL_REV}:{path}'], text=True)


def _load_frames(step=1, limit=None):
    out = []
    for p in sorted(glob.glob(DATASET_GLOB))[::step]:
        f = cv2.imread(p)
        if f is not None and f.shape == (P.FRAME_HEIGHT, P.FRAME_WIDTH, 3):
            out.append((os.path.basename(p), f))
        if limit and len(out) >= limit:
            break
    assert out, f"no usable frames at {DATASET_GLOB}"
    return out


# ---------------------------------------------------------------------------
# Vision
# ---------------------------------------------------------------------------

def _original_pipeline():
    """pipeline.py as it was at ORIGINAL_REV, imported as a separate module."""
    spec = importlib.util.spec_from_loader('original_pipeline', loader=None)
    mod = importlib.util.module_from_spec(spec)
    exec(compile(_git_show('src/vision/pipeline.py'), 'original_pipeline.py', 'exec'), mod.__dict__)
    return mod


_DET_FIELDS = ('type', 'color', 'area', 'centroid', 'target_x', 'target_y')
_ORIGINAL_LISTS = ('detected_blocks', 'detected_walls', 'detected_orange', 'detected_blue',
                   'detected_magenta', 'detected_close_black', 'detected_line_roi_wall')


def _assert_same_detections(ref, new, ctx):
    for key in _ORIGINAL_LISTS:
        r_list, n_list = ref.get(key, []), new.get(key, [])
        assert len(r_list) == len(n_list), f"{ctx}: {key} count {len(r_list)} != {len(n_list)}"
        for i, (r, n) in enumerate(zip(r_list, n_list)):
            for f in _DET_FIELDS:
                assert r.get(f) == n.get(f), f"{ctx}: {key}[{i}].{f} {r.get(f)!r} != {n.get(f)!r}"
            assert np.array_equal(r['contour'], n['contour']), f"{ctx}: {key}[{i}].contour"
    for key in ('arena_mask', 'pure_black_mask'):
        assert np.array_equal(ref[key], new[key]), f"{ctx}: {key} differs"
    r_sky, n_sky = ref.get('arena_sky'), new.get('arena_sky')
    assert (r_sky is None) == (n_sky is None), f"{ctx}: arena_sky presence"
    if r_sky is not None:
        assert np.array_equal(r_sky, n_sky), f"{ctx}: arena_sky differs"
    assert ref['arena_seeded'] == new['arena_seeded'], f"{ctx}: arena_seeded"
    assert ref['line_roi_wall_pct'] == new['line_roi_wall_pct'], f"{ctx}: line_roi_wall_pct"


def test_vision_matches_original():
    """Same detections, masks and arena as the original pipeline, frame for frame,
    with and without SWAP_RED_GREEN."""
    old = _original_pipeline()
    old.vision_pool = None
    P.vision_pool = None
    frames = _load_frames(step=max(1, len(glob.glob(DATASET_GLOB)) // MAX_VISION_FRAMES))
    try:
        for swap in (False, True):
            old.SWAP_RED_GREEN = swap
            P.SWAP_RED_GREEN = swap
            for name, frame in (frames if not swap else frames[::5]):
                ctx = f"{name} swap={swap}"
                _assert_same_detections(old.process_video_frame(frame),
                                        P.process_video_frame(frame), ctx)
                m_old, m_new = old.compute_colour_masks(frame), P.compute_colour_masks(frame)
                for k in m_old:
                    assert np.array_equal(m_old[k], m_new[k]), f"{ctx}: mask {k} differs"
    finally:
        old.SWAP_RED_GREEN = P.SWAP_RED_GREEN = False


def test_vision_pool_matches_original():
    """The two-process VisionPool path gives the original (inline) detections."""
    from src.vision.pool import VisionPool
    old = _original_pipeline()
    old.vision_pool = None
    pool = VisionPool()
    pool.start()
    P.vision_pool = pool
    try:
        for name, frame in _load_frames(step=20):
            _assert_same_detections(old.process_video_frame(frame),
                                    P.process_video_frame(frame), f"{name} (pool)")
        assert pool.ok, "vision pool fell back to inline processing"
    finally:
        P.vision_pool = None
        pool.stop()


def _configure_with_objects(objects):
    cfg = types.SimpleNamespace(**{k: getattr(T, k) for k in dir(T) if not k.startswith('_')})
    cfg.OBJECTS = objects
    P.configure(cfg)


def test_new_object_entry():
    """Adding OBJECTS entries is all it takes to detect a new colour -- inline and
    through the pool -- and it leaves the existing detections untouched."""
    from src.vision.pool import VisionPool
    old = _original_pipeline()
    old.vision_pool = None
    nothing = (np.array([255, 255, 255]), np.array([0, 0, 0]))   # lower > upper: matches no pixel
    objects = dict(T.OBJECTS)
    # A pillar colour that is never seen: must change nothing at all.
    objects['yellow'] = dict(kind='pillar', ranges=[nothing], min_area=250, close=True,
                             morph_close=True, not_wall=True, draw=(0, 255, 255))
    # A marker that re-uses green's range: must find exactly where green pillars are.
    objects['green_marker'] = dict(kind='marker', ranges=[(T.LOWER_GREEN, T.UPPER_GREEN)],
                                   min_area=250, morph_close=True, draw=(0, 128, 0))
    # A line colour that re-uses orange's range.
    objects['orange2'] = dict(kind='line', ranges=[(T.LOWER_ORANGE, T.UPPER_ORANGE)],
                              min_area=20, draw=(0, 100, 255))
    # Custom: green's range in the pillar ROI must find exactly what a marker does.
    # (No morph_close: it fills holes differently at the edge of a smaller mask.)
    objects['green_marker_raw'] = dict(kind='marker', ranges=[(T.LOWER_GREEN, T.UPPER_GREEN)],
                                       min_area=250, draw=(0, 128, 0))
    objects['green_custom'] = dict(kind='custom', roi=T.full_frame_roi, min_area=250,
                                   ranges=[(T.LOWER_GREEN, T.UPPER_GREEN)], draw=(0, 200, 0))
    # Custom on the left half, sticking out above the processed rows (gets clipped).
    left_roi = (0, 20, 320, 200)
    objects['green_left'] = dict(kind='custom', roi=left_roi, min_area=50,
                                 ranges=[(T.LOWER_GREEN, T.UPPER_GREEN)], draw=(0, 150, 0))
    pool = None
    try:
        _configure_with_objects(objects)
        assert P.COLOUR_MASK_NAMES[-6:] == ('yellow', 'green_marker', 'orange2', 'green_marker_raw',
                                            'green_custom', 'green_left')
        assert 'detected_green_marker' in P.DETECTION_LISTS
        frames = _load_frames(step=40)
        for use_pool in (False, True):
            if use_pool:
                pool = VisionPool()
                pool.start()
                P.vision_pool = pool
            else:
                P.vision_pool = None
            found = found_left = 0
            for name, frame in frames:
                ref, new = old.process_video_frame(frame), P.process_video_frame(frame)
                ctx = f"{name} pool={use_pool}"
                _assert_same_detections(ref, new, ctx)
                assert not any(b['color'] == 'yellow' for b in new['detected_blocks']), ctx
                greens = sorted(b['centroid'] for b in new['detected_blocks']
                                if b['type'] == 'block' and b['color'] == 'green')
                markers = sorted(m['centroid'] for m in new['detected_green_marker'])
                # Green pillars pass a ground-contact test markers don't, so markers
                # can only be a superset.
                assert set(greens) <= set(markers), ctx
                assert [o['centroid'] for o in new['detected_orange2']] == \
                       [o['centroid'] for o in new['detected_orange']], ctx
                assert sorted(o['centroid'] for o in new['detected_green_custom']) == \
                       sorted(o['centroid'] for o in new['detected_green_marker_raw']), ctx
                for o in new['detected_green_left']:
                    pts = o['contour'].reshape(-1, 2)
                    assert pts[:, 0].max() < 320 and pts[:, 1].min() >= P.GLOBAL_Y_OFFSET, ctx
                found_left += len(new['detected_green_left'])
                found += len(markers)
                rec = P.record_to_detections(P.detections_to_record(new))
                assert len(rec['detected_green_marker']) == len(markers), ctx
                P.annotate_video_frame(frame, new, 'clockwise')
            assert found, "the green_marker entry never detected anything"
            assert found_left, "the green_left custom entry never detected anything"
            if use_pool:
                assert pool.ok, "vision pool fell back to inline processing"
    finally:
        P.vision_pool = None
        if pool is not None:
            pool.stop()
        P.configure(T)


def test_custom_entry_open():
    """A 'custom' entry also works with the open-challenge config: clipped to its
    rows, found in world.seen(), drawn, recorded -- and the open rules still decide."""
    objects = dict(OC.OBJECTS)
    objects['white_patch'] = dict(kind='custom', roi=(200, 60, 240, 300), min_area=50, arena=False,
                                  ranges=[(T.LOWER_WHITE, T.UPPER_WHITE)], draw=(255, 255, 255))
    cfg = types.SimpleNamespace(**{k: getattr(OC, k) for k in dir(OC) if not k.startswith('_')})
    cfg.OBJECTS = objects
    P.vision_pool = None
    P.configure(cfg)
    saved, open_rules.OBJECTS = open_rules.OBJECTS, objects
    try:
        assert P._PATCHES['white_patch'] == (200, 0, 440, P.GLOBAL_Y_END - P.GLOBAL_Y_OFFSET)
        mem, found = open_rules.Memory(), 0
        for name, frame in _load_frames(step=60):
            det = P.process_video_frame(frame)
            world = open_rules.build_world(det, frame, mem, 1.0)
            for o in world.seen('white_patch'):
                pts = o['contour'].reshape(-1, 2)
                assert 200 <= pts[:, 0].min() and pts[:, 0].max() < 440, name
                assert P.GLOBAL_Y_OFFSET <= pts[:, 1].min() and pts[:, 1].max() < P.GLOBAL_Y_END, name
                found += 1
            open_rules.learn_direction(world, mem)
            open_rules.count_turns(world, mem)
            open_rules.decide(world, mem)
            P.annotate_video_frame(frame, det, 'clockwise')
            rec = P.record_to_detections(P.detections_to_record(det))
            assert len(rec['detected_white_patch']) == len(det['detected_white_patch']), name
        assert found, "the custom entry never detected anything"
    finally:
        open_rules.OBJECTS = saved
        P.configure(T)


def test_target_false_entry():
    """A pillar entry with target=False is never world.target_block, but stays visible."""
    def pillar(color, y):
        return {'type': 'block', 'color': color, 'centroid': (320, y), 'area': 1000}
    saved = rules.OBJECTS
    rules.OBJECTS = dict(saved, yellow=dict(kind='pillar', ranges=[], min_area=250, target=False))
    try:
        world = types.SimpleNamespace(blocks=[pillar('yellow', 200), pillar('red', 150)])
        block, _ = rules.pick_target_block(world)
        assert block['color'] == 'red', block
        world = types.SimpleNamespace(blocks=[pillar('yellow', 200)])
        assert rules.pick_target_block(world) == (None, False)
        rules.OBJECTS['yellow'] = dict(rules.OBJECTS['yellow'], target=True)
        world = types.SimpleNamespace(blocks=[pillar('yellow', 200), pillar('red', 150)])
        assert rules.pick_target_block(world)[0]['color'] == 'yellow'
    finally:
        rules.OBJECTS = saved


# ---------------------------------------------------------------------------
# Control loop
# ---------------------------------------------------------------------------

def _sequences():
    """(name, direction, [detections, ...]) in frame order."""
    seqs = []
    # Dataset captures, grouped by session, through the real pipeline (includes
    # pure_black_mask, so the pillar-to-wall gap scan is exercised).
    P.vision_pool = None
    by_session = {}
    for name, frame in _load_frames():
        by_session.setdefault(name.rsplit('_', 1)[0], []).append(frame)
    for session, frames in sorted(by_session.items()):
        if len(frames) >= 20:
            dets = [P.process_video_frame(f) for f in frames]
            for d in ('clockwise', 'counter-clockwise'):
                seqs.append((f'dataset/{session}', d, dets))
    # Recorded runs (`main.py -v`), if any are on this machine.
    logs = [l for l in sorted(glob.glob(RUN_LOGS_GLOB)) if os.path.getsize(l) > 300_000]
    for path in logs[-MAX_RUN_LOGS:]:
        with open(path) as fh:
            recs = [json.loads(x) for x in fh if x.strip()]
        recs = [r for r in recs if 'det' in r]
        if len(recs) < 50:
            continue
        direction = recs[-1].get('dir')
        if direction not in ('clockwise', 'counter-clockwise'):
            continue
        seqs.append((path, direction, [P.record_to_detections(r['det']) for r in recs]))
    return seqs


class _End(Exception):
    pass


class _Recorder:
    """Stands in for motor/servo: records every call (except the read-only ones)."""
    def __init__(self, events, prefix):
        self._events, self._prefix = events, prefix

    def __getattr__(self, name):
        return lambda *a, **k: self._events.append((self._prefix + name,) + a + tuple(sorted(k.items())))

    def get_measured_rpm(self):
        return 0.0


class _Null:
    def __getattr__(self, name):
        return lambda *a, **k: None

    def __bool__(self):
        return False


def _frame_time(i):
    return 6.0 + i / 28.0     # recorded every 2nd frame at 56 fps; > 5 s into the run


def _original_loop_source():
    """The body of main.py's `while True:` drive loop at ORIGINAL_REV, up to (not
    including) the turn-13 parking branch, wrapped in a function."""
    src = _git_show('src/obstacle_challenge/main.py').split('\n')
    end = next(i for i, l in enumerate(src) if 'if turn_counter >= 13:' in l)
    start = max(i for i, l in enumerate(src[:end]) if l == '        while True:')
    body = src[start:end] + [
        '            _capture(angle, commanded_rpm, visual_target_x, visual_target_line, turn_counter)']
    state = ['cooldown_frames', 'turn_counter', 'prevangle', 'prev_wall_error',
             'past_frame_counter', 'frame_counter', 'prev_rpm', 'last_block_target_rpm',
             'frames_since_block_seen', 'tracking_green_block', 'green_post_pass_frames',
             'tracking_red_block', 'red_post_pass_frames', 'prev_target_error']
    return ('def _run():\n    global ' + ', '.join(state) + '\n'
            + '\n'.join(l[4:] for l in body) + '\n')


def run_original(direction, dets):
    out, events = [], []
    clock = [0.0]
    idx = iter(range(len(dets)))
    cur = [0]

    class Camera:
        def get_next_frame(self, past):
            try:
                cur[0] = next(idx)
            except StopIteration:
                raise _End
            clock[0] = _frame_time(cur[0])
            return np.zeros(1, np.uint8), cur[0] + 1, clock[0]

    fake_time = types.SimpleNamespace(monotonic=lambda: clock[0], perf_counter=lambda: clock[0],
                                      sleep=lambda s: None)

    def capture(angle, rpm, tx, tl, turns):
        out.append(dict(angle=float(angle), rpm=float(rpm), target_x=tx, target_line=tl,
                        turns=turns, events=list(events)))
        events.clear()

    g = {k: getattr(T, k) for k in dir(T) if not k.startswith('_')}
    g.update(
        np=np, cv2=cv2, math=math, deque=deque, time=fake_time,
        camera_thread=Camera(), process_video_frame=lambda f: dets[cur[0]],
        motor=_Recorder(events, 'motor.'), servo=_Recorder(events, 'servo.'),
        log=_Null(), perf=_Null(), video_encoder=_Null(), overlay_log=_Null(),
        record_raw=True, _capture=capture, driving_direction=direction, run_start_time=0.0,
        orange_detection_history=deque([False] * T.ORANGE_DETECTION_HISTORY_LENGTH,
                                       maxlen=T.ORANGE_DETECTION_HISTORY_LENGTH),
        cooldown_frames=0, turn_counter=0, prevangle=0, prev_wall_error=0.0,
        past_frame_counter=0, frame_counter=0, frame_debug_throttle=_Null(),
        prev_rpm=T.INITIAL_RPM, last_block_target_rpm=T.INITIAL_RPM,
        frames_since_block_seen=T.BLOCK_TARGET_GRACE_FRAMES,
        tracking_green_block=False, green_post_pass_frames=0,
        tracking_red_block=False, red_post_pass_frames=0, prev_target_error=0.0,
    )
    exec(compile(_original_loop_source(), 'original_main_loop', 'exec'), g)
    try:
        g['_run']()
    except _End:
        pass
    return out


class _patched:
    """Temporarily replace attributes of a module (motor/servo/time/log in a main)."""
    def __init__(self, module, **attrs):
        self.module, self.attrs, self.saved = module, attrs, {}

    def __enter__(self):
        for k, v in self.attrs.items():
            self.saved[k] = getattr(self.module, k)
            setattr(self.module, k, v)

    def __exit__(self, *exc):
        for k, v in self.saved.items():
            setattr(self.module, k, v)


_NO_SLEEP = types.SimpleNamespace(sleep=lambda s: None)


def run_new(direction, dets):
    """rules.py + obstacle main.actuate(), the way main.drive_phase() drives them."""
    out, events = [], []
    mem = rules.Memory()
    prev_angle = 0
    with _patched(obstacle_main, motor=_Recorder(events, 'motor.'),
                  servo=_Recorder(events, 'servo.'), time=_NO_SLEEP, log=_Null()):
        for i, det in enumerate(dets):
            world = rules.build_world(det, direction, _frame_time(i))
            rules.count_turns(world, mem)
            cmd = rules.decide(world, mem)
            prev_angle = obstacle_main.actuate(cmd, prev_angle)
            out.append(dict(angle=float(cmd.angle), rpm=float(cmd.rpm), target_x=cmd.target_x,
                            target_line=cmd.target_line, turns=mem.turns, events=list(events)))
            events.clear()
    return out


def test_rules_match_original():
    """Same steering angle, rpm, motor/servo calls, aim lines and turn count as the
    original main-loop code, on every frame of every sequence."""
    seqs = _sequences()
    assert seqs, "no input sequences"
    frames = 0
    for name, direction, dets in seqs:
        old = run_original(direction, copy.deepcopy(dets))
        new = run_new(direction, copy.deepcopy(dets))
        assert len(old) == len(new), f"{name}: frame count {len(old)} != {len(new)}"
        for i, (o, n) in enumerate(zip(old, new)):
            for k in o:
                assert o[k] == n[k], f"{name} [{direction}] frame {i}: {k} {o[k]!r} != {n[k]!r}"
        frames += len(old)
    print(f"  compared {frames} frames over {len(seqs)} sequences")


# ---------------------------------------------------------------------------
# Open challenge
# ---------------------------------------------------------------------------

def test_open_vision_matches_original():
    """With the open challenge's config, the pipeline matches the original too."""
    old = _original_pipeline()
    old.vision_pool = None
    P.vision_pool = None
    try:
        old.configure(OC)
        P.configure(OC)
        for name, frame in _load_frames(step=10):
            _assert_same_detections(old.process_video_frame(frame),
                                    P.process_video_frame(frame), f"{name} (open config)")
    finally:
        P.configure(T)


def _open_sequences():
    """(name, [(frame, detections), ...]) per dataset session, open-challenge vision."""
    P.vision_pool = None
    by_session = {}
    for name, frame in _load_frames():
        by_session.setdefault(name.rsplit('_', 1)[0], []).append(frame)
    seqs = []
    try:
        P.configure(OC)
        for session, frames in sorted(by_session.items()):
            if len(frames) >= 20:
                seqs.append((f'dataset/{session}', [(f, P.process_video_frame(f)) for f in frames]))
    finally:
        P.configure(T)
    return seqs


def _original_open_loop_source():
    """front_wall_base_row() and the body of the open challenge's frame loop at
    ORIGINAL_REV, up to (not including) its exit conditions."""
    src = _git_show('src/open_challenge/main.py').split('\n')
    k0 = next(i for i, l in enumerate(src) if l.startswith('_FRONT_WALL_KERNEL'))
    k1 = next(i for i, l in enumerate(src) if l.startswith('if __name__'))
    end = next(i for i, l in enumerate(src) if '# --- Exit Conditions ---' in l)
    start = max(i for i, l in enumerate(src[:end]) if l == '        while True:')
    body = src[start:end] + [
        '            _capture(angle, target_rpm, driving_direction, turn_counter)']
    state = ['cooldown_frames', 'turn_counter', 'prev_angle', 'prev_wall_error',
             'past_frame_counter', 'driving_direction', 'target_rpm', 'latest_vibration_deg_s']
    return ('\n'.join(src[k0:k1]) + '\n\ndef _run():\n    global ' + ', '.join(state) + '\n'
            + '\n'.join(l[4:] for l in body) + '\n')


def run_open_original(frames_dets):
    out, events = [], []
    clock = [0.0]
    idx = iter(range(len(frames_dets)))
    cur = [0]

    class Camera:
        def get_next_frame(self, past):
            try:
                cur[0] = next(idx)
            except StopIteration:
                raise _End
            clock[0] = _frame_time(cur[0])
            return frames_dets[cur[0]][0], cur[0] + 1, clock[0]

    fake_time = types.SimpleNamespace(monotonic=lambda: clock[0], perf_counter=lambda: clock[0],
                                      sleep=lambda s: None)

    def capture(angle, target_rpm, direction, turns):
        out.append(dict(angle=float(angle), rpm=float(target_rpm), direction=direction,
                        turns=turns, events=list(events)))
        events.clear()

    from src.open_challenge.schedule import SpeedSchedule
    g = {k: getattr(OC, k) for k in dir(OC) if not k.startswith('_')}
    g.update(
        np=np, cv2=cv2, math=math, deque=deque, time=fake_time, config=OC,
        camera_thread=Camera(), process_video_frame=lambda f: frames_dets[cur[0]][1],
        motor=_Recorder(events, 'motor.'), servo=_Recorder(events, 'servo.'),
        log=_Null(), ilog=_Null(), perf=_Null(), video_encoder=_Null(), imu_thread=_Null(),
        frame_debug_throttle=_Null(), vibration_throttle=_Null(), _capture=capture,
        orange_detection_history=deque([False] * OC.ORANGE_DETECTION_HISTORY_LENGTH,
                                       maxlen=OC.ORANGE_DETECTION_HISTORY_LENGTH),
        cooldown_frames=0, turn_counter=0, prev_angle=0, prev_wall_error=0.0,
        past_frame_counter=0, driving_direction=None, final_run_initiated=False,
        speed_schedule=SpeedSchedule(), target_rpm=OC.STRAIGHT_RPM, latest_vibration_deg_s=0.0,
    )
    exec(compile(_original_open_loop_source(), 'original_open_loop', 'exec'), g)
    try:
        g['_run']()
    except _End:
        pass
    return out


def run_open_new(frames_dets):
    """open_challenge/rules.py + its main.actuate(), the way its drive_phase() drives them."""
    out, events = [], []
    mem = open_rules.Memory()
    prev_angle = 0
    with _patched(open_main, motor=_Recorder(events, 'motor.'),
                  servo=_Recorder(events, 'servo.'), log=_Null()):
        for i, (frame, det) in enumerate(frames_dets):
            world = open_rules.build_world(det, frame, mem, _frame_time(i))
            open_rules.learn_direction(world, mem)
            open_rules.count_turns(world, mem)
            cmd = open_rules.decide(world, mem)
            prev_angle = open_main.actuate(cmd, prev_angle)
            out.append(dict(angle=float(cmd.angle), rpm=float(mem.target_rpm),
                            direction=mem.direction, turns=mem.turns, events=list(events)))
            events.clear()
    return out


def test_open_rules_match_original():
    """Same steering, rpm target changes, motor/servo calls, learned direction and
    turn count as the original open-challenge loop, on every frame."""
    seqs = _open_sequences()
    assert seqs, "no input sequences"
    frames = 0
    for name, frames_dets in seqs:
        old = run_open_original(copy.deepcopy(frames_dets))
        new = run_open_new(copy.deepcopy(frames_dets))
        assert len(old) == len(new), f"{name}: frame count {len(old)} != {len(new)}"
        for i, (o, n) in enumerate(zip(old, new)):
            for k in o:
                assert o[k] == n[k], f"{name} [open] frame {i}: {k} {o[k]!r} != {n[k]!r}"
        frames += len(old)
    print(f"  compared {frames} open-challenge frames over {len(seqs)} sequences")


def main():
    tests = [test_vision_matches_original, test_vision_pool_matches_original,
             test_new_object_entry, test_custom_entry_open, test_target_false_entry, test_rules_match_original,
             test_open_vision_matches_original, test_open_rules_match_original]
    failed = 0
    for t in tests:
        try:
            t()
            print(f"PASS  {t.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL  {t.__name__}: {e}")
    print(f"\n{len(tests) - failed}/{len(tests)} passed")
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()

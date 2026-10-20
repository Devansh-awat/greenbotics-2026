"""What the robot DOES on each camera frame of the obstacle run.

How one frame is decided
------------------------
1. build_world()  turns the camera detections into a World: what is seen NOW.
2. count_turns()  updates the lap counter in Memory.
3. decide()       asks each rule in RULES, in order: "is this your situation?"
                  A rule answers None ("not mine") or a Command. The FIRST rule that
                  answers wins; the rules after it are not asked.
4. FILTERS        then adjust whichever Command won (post-pass clip, speed, ramp).

  World    -- this frame only: pillars, walls, lines, direction, run time.
  Memory   -- what must survive between frames: turn count, previous errors, which
              pillar we were tracking, post-pass frames left, last rpm.
  Command  -- the answer: steering angle, label of the rule, swerve or not, and what
              the FILTERS fill in (rpm).

Nothing in this file touches the motor, servo or camera. main.py's actuate() carries
out the Command. That is what lets src/tools/replay.py run this exact code on
recorded runs with no robot.

To add a behaviour: write rule_xxx(world, mem) returning a Command or None, and
insert it into RULES at the right priority. See SURPRISE.md.

Every number used here lives in tuning.py.
"""

import math
from collections import deque
from dataclasses import dataclass, field

import cv2
import numpy as np

from src.obstacle_challenge.tuning import *


# ---------------------------------------------------------------------------
# World, Memory, Command
# ---------------------------------------------------------------------------

@dataclass
class World:
    """What the camera sees on THIS frame. Built fresh every frame."""
    detections: dict
    direction: str            # 'clockwise' | 'counter-clockwise'
    run_time: float           # seconds since the run started
    blocks: list              # pillars: type 'block' (far) and 'close_block' (in the close ROI)
    magenta: list             # magenta parking walls, sorted in driving order
    orange: list
    blue: list
    wall_left: float          # black wall pixel areas in the four wall ROIs
    wall_right: float
    inner_left: float
    inner_right: float
    close_black_area: float
    line_wall_pct: float      # % of the line ROI that is black wall
    pure_black_mask: object   # slice-sized mask of wall pixels (or None in replay)
    target_block: dict = None # the pillar to steer around (see pick_target_block)
    target_capped: bool = False  # True when it is under its handoff line

    @property
    def clockwise(self):
        return self.direction == 'clockwise'

    @property
    def corner_close(self):
        return (self.close_black_area > CORNER_CLOSE_BLACK_AREA
                or self.line_wall_pct > CORNER_LINE_WALL_PCT)

    def seen(self, name):
        """Every detection of OBJECTS[name] this frame (any kind of object)."""
        if OBJECTS[name]['kind'] == 'pillar':
            return [b for b in self.blocks if b['color'] == name]
        return self.detections.get('detected_' + name, [])


@dataclass
class Memory:
    """Everything that must survive from one frame to the next."""
    turns: int = 0
    turn_cooldown: int = 0
    orange_history: deque = field(default_factory=lambda: deque(
        [False] * ORANGE_DETECTION_HISTORY_LENGTH, maxlen=ORANGE_DETECTION_HISTORY_LENGTH))
    prev_wall_error: float = 0.0
    prev_target_error: float = 0.0
    tracking: str = None              # colour of the pillar being steered around, or None
    post_pass: dict = field(default_factory=lambda: {c: 0 for c in POST_PASS})
    frames_since_block: int = BLOCK_TARGET_GRACE_FRAMES
    last_block_rpm: float = INITIAL_RPM
    prev_rpm: float = INITIAL_RPM
    extra: dict = field(default_factory=dict)   # free space for new rules' own state


@dataclass
class Command:
    """One rule's answer for this frame."""
    angle: float = 0.0
    rule: str = ''                # which rule decided (shown in the video and log)
    swerve: float = None          # set = reverse-and-swerve with this angle before steering
    wall_steering: bool = False   # True when following walls (looser post-green clip)
    block_y: float = None         # y of the pillar being passed (slows down when close)
    speed: float = None           # set = use this rpm instead of the automatic speed (0 = stop)
    rpm: float = 0.0              # filled in by the speed filters
    rpm_changed: bool = False
    target_x: int = None          # aim line, for the video
    target_line: tuple = None     # angle line, for the video
    debug: list = field(default_factory=list)


# ---------------------------------------------------------------------------
# 1. Building the World
# ---------------------------------------------------------------------------

def build_world(detections, direction, run_time):
    # Magenta in driving order (left->right when clockwise). Sorted in place so the
    # recorded detections show the same order.
    if detections.get('detected_magenta'):
        detections['detected_magenta'].sort(key=lambda m: m['centroid'][0],
                                            reverse=(direction != 'clockwise'))

    walls = detections['detected_walls']

    def wall_area(kind):
        return sum(w['area'] for w in walls if w['type'] == kind)

    world = World(
        detections=detections,
        direction=direction,
        run_time=run_time,
        blocks=[b for b in detections['detected_blocks'] if not _is_next_straight_pillar(b)],
        magenta=detections.get('detected_magenta', []),
        orange=detections['detected_orange'],
        blue=detections['detected_blue'],
        wall_left=wall_area('wall_left'),
        wall_right=wall_area('wall_right'),
        inner_left=wall_area('wall_inner_left'),
        inner_right=wall_area('wall_inner_right'),
        close_black_area=sum(o['area'] for o in detections.get('detected_close_black', [])),
        line_wall_pct=detections.get('line_roi_wall_pct', 0),
        pure_black_mask=detections.get('pure_black_mask'),
    )
    world.target_block, world.target_capped = pick_target_block(world)
    return world


def _is_next_straight_pillar(b):
    """A pillar on the NEXT straight, seen over the corner: touches the top of the
    pillar ROI at the far side of the frame."""
    if b.get('contour') is None or len(b['contour']) == 0:
        return False
    x = b.get('centroid', (0, 0))[0]
    far_side = ((b.get('color') == 'green' and x < EDGE_IGNORE_GREEN_MAX_X)
                or (b.get('color') == 'red' and x > EDGE_IGNORE_RED_MIN_X))
    top_y = b['contour'].reshape(-1, 2)[:, 1].min()
    return far_side and top_y <= full_frame_roi[1] + EDGE_IGNORE_TOP_PX


def pick_target_block(world):
    """The pillar to steer around: the lowest (nearest) one, unless it is already
    being passed (under its handoff line) -- then hand over to the next pillar.

    Returns (block or None, capped). capped = we stay on a pillar that is under
    its handoff line, so its rule limits how hard it may steer.

    Pillars whose OBJECTS entry has target=False are never picked (they are still
    in world.blocks / world.seen() for a rule to look at).
    """
    candidates = [b for b in world.blocks
                  if b['type'] == 'block' and OBJECTS.get(b['color'], {}).get('target', True)]
    if not candidates:
        return None, False
    b0 = candidates[0]
    b1 = candidates[1] if len(candidates) > 1 else None
    x, y = b0['centroid']

    # Red handoff line: slanted, from (X_EDGE, Y_EDGE) to (X_CENTER, Y_CENTER).
    if RED_HANDOFF_X_CENTER != RED_HANDOFF_X_EDGE:
        red_line_y = RED_HANDOFF_Y_EDGE + (RED_HANDOFF_Y_CENTER - RED_HANDOFF_Y_EDGE) * (
            (x - RED_HANDOFF_X_EDGE) / (RED_HANDOFF_X_CENTER - RED_HANDOFF_X_EDGE))
    else:
        red_line_y = RED_HANDOFF_Y_CENTER

    if b0['color'] == 'red' and x < RED_HANDOFF_X_CENTER and y >= red_line_y:
        if b1 is not None and b1['color'] == 'green':
            return b0, False
        if b1 is not None and b1['color'] == 'red':
            return b1, False
        return b0, True

    if b0['color'] == 'green' and x > GREEN_HANDOFF_MIN_X and y >= GREEN_HANDOFF_MIN_Y:
        if b1 is not None and b1['color'] == 'green':
            return b1, False
        return b0, True

    return b0, False


# ---------------------------------------------------------------------------
# 2. Turn counting
# ---------------------------------------------------------------------------

def count_turns(world, mem):
    """A turn = orange seen for (HISTORY_LENGTH - 1) frames in a row after a frame
    without it, then ignored for ORANGE_COOLDOWN_FRAMES. Returns True on a new turn."""
    mem.orange_history.append(bool(world.orange))
    if mem.turn_cooldown > 0:
        mem.turn_cooldown -= 1
        return False
    if not mem.orange_history[0] and sum(mem.orange_history) == ORANGE_DETECTION_HISTORY_LENGTH - 1:
        mem.turns += 1
        mem.turn_cooldown = ORANGE_COOLDOWN_FRAMES
        return True
    return False


# ---------------------------------------------------------------------------
# 3. The rules, highest priority first
# ---------------------------------------------------------------------------

def rule_close_block(world, mem):
    """A pillar is right in front of the nose: reverse, then swerve away.

    Only the FIRST pillar in the close ROI is looked at. A colour without an entry
    in CLOSE_SWERVE (e.g. black) never triggers it.
    """
    close = [b for b in world.blocks if b['type'] == 'close_block']
    if not close:
        return None
    colour = close[0]['color']
    swerve = CLOSE_SWERVE.get(colour)
    if swerve is None or world.run_time <= swerve.get('after_s', -math.inf):
        return None

    # Starting again from a crawl after the swerve.
    mem.prev_rpm = INITIAL_RPM
    mem.last_block_rpm = INITIAL_RPM
    mem.frames_since_block = BLOCK_TARGET_GRACE_FRAMES
    if colour in POST_PASS:
        mem.post_pass[colour] = POST_PASS[colour]['frames']
    mem.tracking = None
    angle = swerve[world.direction]
    return Command(angle=angle, rule='close:' + colour, swerve=angle)


def rule_red(world, mem):
    """Pass a red pillar on its RIGHT (the pillar stays on our left)."""
    block = world.target_block
    if block is None or block['color'] != 'red':
        return None
    _start_tracking(mem, 'red')
    bx, by = block['centroid']
    cmd = Command(rule='red', block_y=by)

    # 1. Find the gap to aim at: between the pillar and a magenta wall right of it
    #    (counter-clockwise only), else between the pillar and the black wall.
    aim_x = None
    magenta = _magenta_beside(world, block, 'right', RED_MAGENTA_MAX_DY)
    if magenta is not None and not world.clockwise:
        aim_x = int(MAGENTA_AIM_WEIGHT * magenta['contour'][:, 0, 0].min()
                    + BLOCK_AIM_WEIGHT * block['contour'][:, 0, 0].max())
    elif world.inner_right > 0 or world.wall_right > 0:
        aim_x = _gap_to_wall(world, block, 'right')

    # 2. Steer: at the gap if there is one, else along the ideal line.
    if aim_x is not None:
        cmd.target_x = aim_x
        angle = _steer_to_x(aim_x, mem)
    else:
        mem.prev_target_error = 0.0
        angle, cmd.target_line = _steer_along_line(block, RED_ORIGIN_X, RED_ORIGIN_Y,
                                                   RED_OTHER_X, RED_OTHER_Y)

    # 3. Limits.
    if world.inner_left > BIG_INNER_WALL_AREA:
        angle = np.clip(angle, *AWAY_FROM_LEFT_WALL)
    elif world.inner_right > BIG_INNER_WALL_AREA:
        angle = np.clip(angle, *AWAY_FROM_RIGHT_WALL)
    else:
        angle = np.clip(angle, *RED_ANGLE_RANGE)
    if world.target_capped and world.inner_right <= BIG_INNER_WALL_AREA:
        angle = max(RED_UNDER_LINE_MIN_ANGLE, angle)
        cmd.debug.append(f"RCap:{RED_UNDER_LINE_MIN_ANGLE}")

    cmd.angle = angle
    cmd.debug.append(f"Blk:R({int(bx)},{int(by)})")
    if cmd.target_x is not None:
        cmd.debug.append(f"MidX:{int(cmd.target_x)}")
    cmd.debug.extend([f"IL:{int(world.inner_left)}", f"IR:{int(world.inner_right)}"])
    return cmd


def rule_green(world, mem):
    """Pass a green pillar on its LEFT (the pillar stays on our right)."""
    block = world.target_block
    if block is None or block['color'] != 'green':
        return None
    _start_tracking(mem, 'green')
    bx, by = block['centroid']
    cmd = Command(rule='green', block_y=by)

    # 1. Find the gap: between a magenta wall left of the pillar and the pillar
    #    (clockwise only), else between the black wall and the pillar.
    aim_x = None
    magenta = _magenta_beside(world, block, 'left', GREEN_MAGENTA_MAX_DY)
    if magenta is not None and world.clockwise:
        aim_x = int(MAGENTA_AIM_WEIGHT * magenta['contour'][:, 0, 0].max()
                    + BLOCK_AIM_WEIGHT * block['contour'][:, 0, 0].min())
    elif world.inner_left > 0 or world.wall_left > 0:
        aim_x = _gap_to_wall(world, block, 'left')

    # 2. Steer.
    if aim_x is not None:
        cmd.target_x = aim_x
        angle = _steer_to_x(aim_x, mem)
    else:
        mem.prev_target_error = 0.0
        angle, cmd.target_line = _steer_along_line(block, GREEN_ORIGIN_X, GREEN_ORIGIN_Y,
                                                   GREEN_OTHER_X, GREEN_OTHER_Y)

    # 3. Limits. (Big inner walls only matter when there is no gap to aim at.)
    if cmd.target_x is None and world.inner_left > BIG_INNER_WALL_AREA:
        angle = np.clip(angle, *AWAY_FROM_LEFT_WALL)
    elif cmd.target_x is None and world.inner_right > BIG_INNER_WALL_AREA:
        angle = np.clip(angle, *AWAY_FROM_RIGHT_WALL)
    else:
        angle = np.clip(angle, *GREEN_ANGLE_RANGE)
    if world.target_capped:
        angle = np.clip(angle, -GREEN_UNDER_LINE_MAX_ANGLE, GREEN_UNDER_LINE_MAX_ANGLE)
        cmd.debug.append("GLineCap")

    cmd.angle = angle
    cmd.debug.append(f"Blk:G({int(bx)},{int(by)})")
    if cmd.target_x is not None:
        cmd.debug.append(f"MidX:{int(cmd.target_x)}")
    cmd.debug.extend([f"IL:{int(world.inner_left)}", f"IR:{int(world.inner_right)}"])
    return cmd


def rule_wall_follow(world, mem):
    """No pillar to deal with: keep the left and right wall areas balanced (PD),
    pushing round the corner when it comes. Always answers, so keep it LAST."""
    _stop_tracking(mem)
    if mem.frames_since_block < BLOCK_TARGET_GRACE_FRAMES:
        mem.frames_since_block += 1
    mem.prev_target_error = 0.0

    total_left = world.wall_left + world.inner_left
    total_right = world.wall_right + world.inner_right
    # Only one side visible: exaggerate it so we steer firmly away from it.
    left_gone = total_left < WALL_ONE_SIDE_MIN and total_right > WALL_ONE_SIDE_SEEN
    right_gone = total_right < WALL_ONE_SIDE_MIN and total_left > WALL_ONE_SIDE_SEEN
    if left_gone and right_gone:
        # Both walls weak: ambiguous, so turn the way we drive. Clockwise the inner
        # wall is on the right, so "right gone" is the corner.
        right_gone = world.clockwise
        left_gone = not right_gone
    if left_gone:
        total_right = total_right * 2 + WALL_ONE_SIDE_BOOST
    elif right_gone:
        total_left = total_left * 2 + WALL_ONE_SIDE_BOOST

    cmd = Command(rule='wall', wall_steering=True)
    cmd.debug.extend([f"L:{int(world.wall_left)}", f"R:{int(world.wall_right)}"])
    cmd.debug.extend([f"IL:{int(world.inner_left)}", f"IR:{int(world.inner_right)}"])

    error = total_left - total_right
    angle = error * WALL_KP + (error - mem.prev_wall_error) * WALL_KD + WALL_ANGLE_BIAS
    mem.prev_wall_error = error
    cmd.debug.append(f"Err:{int(error)}")
    cmd.debug.append(f"Wall%:{int(world.line_wall_pct)}")

    if world.corner_close:
        angle += CORNER_PUSH[world.direction]
    no_walls = total_left == 0 and total_right == 0
    if no_walls and (world.orange or world.blue):
        angle += NO_WALL_LINE_PUSH[world.direction]
    if no_walls:
        angle += NO_WALL_PUSH[world.direction]
    elif world.wall_left == 0 and not world.clockwise:
        angle += -OUTER_WALL_MISSING_PUSH
    elif world.wall_right == 0 and world.clockwise:
        angle += OUTER_WALL_MISSING_PUSH

    cmd.angle = angle
    return cmd


# THE PRIORITY LIST. First rule that answers wins.
RULES = [
    rule_close_block,
    rule_red,
    rule_green,
    rule_wall_follow,
]


# ---------------------------------------------------------------------------
# Helpers the rules share
# ---------------------------------------------------------------------------

def _start_tracking(mem, colour):
    """We are steering around a `colour` pillar from this frame on."""
    if mem.tracking is not None and mem.tracking != colour:
        _stop_tracking(mem)
    mem.tracking = colour
    if colour in mem.post_pass:
        mem.post_pass[colour] = 0
    mem.prev_wall_error = 0.0
    mem.frames_since_block = 0


def _stop_tracking(mem):
    """The pillar we were steering around is passed: start its post-pass window."""
    if mem.tracking in POST_PASS:
        mem.post_pass[mem.tracking] = POST_PASS[mem.tracking]['frames']
    mem.tracking = None


def _magenta_beside(world, block, side, max_dy):
    """Nearest magenta wall at least MAGENTA_BESIDE_MIN_DX px to the `side` of the
    pillar and within max_dy px vertically, or None."""
    bx, by = block['centroid']
    best, best_dx = None, None
    for m in world.magenta:
        dx = (m['centroid'][0] - bx) if side == 'right' else (bx - m['centroid'][0])
        if dx >= MAGENTA_BESIDE_MIN_DX and abs(m['target_y'] - by) < max_dy:
            if best is None or dx < best_dx:
                best, best_dx = m, dx
    return best


def _gap_to_wall(world, block, side):
    """x halfway between the pillar's bottom corner on `side` and the first black
    wall pixel beyond it on the same rows, or None if no wall is found."""
    pts = block['contour'].reshape(-1, 2)
    bottom = pts[pts[:, 1] >= pts[:, 1].max() - 2]
    if side == 'right':
        corner_x = int(bottom[:, 0].max()) if len(bottom) > 0 else int(pts[:, 0].max())
    else:
        corner_x = int(bottom[:, 0].min()) if len(bottom) > 0 else int(pts[:, 0].min())
    corner_y = int(pts[:, 1].max())

    mask = world.pure_black_mask
    if mask is None:
        return None
    if side == 'right':
        y_lo = min(inner_right_roi_y, right_roi_y)
        y_hi = max(inner_right_roi_y + inner_right_roi_h, right_roi_y + right_roi_h) - 1
        xs_min = max(1, max(corner_x + 1, inner_right_roi_x))
        xs_max = min(mask.shape[1] - 2, right_roi_x + right_roi_w // 2)
    else:
        y_lo = min(inner_left_roi_y, left_roi_y)
        y_hi = max(inner_left_roi_y + inner_left_roi_h, left_roi_y + left_roi_h) - 1
        xs_min = max(1, left_roi_x + left_roi_w // 2)
        xs_max = min(mask.shape[1] - 2, min(corner_x - 1, inner_left_roi_x + inner_left_roi_w - 1))
    ys_min = max(1, max(y_lo, corner_y - WALL_GAP_SCAN_DY) - GLOBAL_Y_OFFSET)
    ys_max = min(mask.shape[0] - 2, min(y_hi, corner_y + WALL_GAP_SCAN_DY) - GLOBAL_Y_OFFSET)
    if ys_min > ys_max or xs_min > xs_max:
        return None

    # Erode 3x3 so single noisy pixels don't count as wall.
    crop = mask[ys_min - 1:ys_max + 2, xs_min - 1:xs_max + 2]
    eroded = cv2.erode(crop, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))[1:-1, 1:-1]
    cols = np.where(np.any(eroded > 0, axis=0))[0]
    if len(cols) == 0:
        return None
    wall_x = int(xs_min + (cols[0] if side == 'right' else cols[-1]))
    return int((corner_x + wall_x) / 2)


def _steer_to_x(aim_x, mem):
    """PD steering that brings aim_x to the middle of the frame."""
    error = aim_x - FRAME_MIDPOINT_X
    angle = error * TARGET_LINE_KP + (error - mem.prev_target_error) * TARGET_LINE_KD
    mem.prev_target_error = error
    return angle


def _steer_along_line(block, ox, oy, other_x, other_y):
    """Steer so the line origin->pillar matches the ideal line origin->other.
    Returns (angle, line to draw)."""
    bx, by = block['centroid']
    ideal = math.degrees(math.atan2(other_x - ox, oy - other_y))
    current = math.degrees(math.atan2(bx - ox, oy - by))
    return (current - ideal) * BLOCK_ANGLE_GAIN, ((ox, oy), (bx, by), ideal, (other_x, other_y))


# ---------------------------------------------------------------------------
# 4. Filters: applied to whichever Command won, in this order
# ---------------------------------------------------------------------------

def filter_post_pass_clip(cmd, world, mem):
    """Just passed a pillar: limit steering for a few frames so we don't swing
    straight into the wall. Otherwise just the hard STEER_LIMIT."""
    for colour, p in POST_PASS.items():
        left = mem.post_pass.get(colour, 0)
        if left <= 0:
            continue
        tag = 'W' if cmd.wall_steering else ''
        cap = p['wall_max_angle'] if cmd.wall_steering else p['max_angle']
        if p['corner_max_angle'] is not None and world.corner_close:
            cap, tag = p['corner_max_angle'], 'Ovr'
        cmd.angle = np.clip(cmd.angle, -cap, cap)
        cmd.debug.append(f"{colour[0].upper()}Clip{tag}:{left}")
        mem.post_pass[colour] = left - 1
        return
    cmd.angle = np.clip(cmd.angle, -STEER_LIMIT, STEER_LIMIT)


def filter_speed(cmd, world, mem):
    """Target rpm: slower when steering hard, and slower the closer the pillar.
    A rule that sets cmd.speed overrides all of this."""
    if cmd.speed is not None:
        cmd.rpm = cmd.speed
        return
    if not USE_VARIABLE_SPEED:
        cmd.rpm = MIN_RPM
        return
    a = min(1.0, abs(cmd.angle) / SPEED_FULL_STEER_ANGLE)
    f_steering = 1.0 - (SPEED_STEER_SLOWDOWN * a)
    if cmd.block_y is not None:
        # 1.0 at the top of the pillar ROI, 0.0 at SPEED_ROI_BOTTOM_PAD above its bottom.
        roi_y_min = full_frame_roi[1]
        roi_y_max = full_frame_roi[1] + full_frame_roi[3]
        scale_y_max = max(roi_y_min + 1, roi_y_max - SPEED_ROI_BOTTOM_PAD)
        y = np.clip(cmd.block_y, roi_y_min, scale_y_max)
        f_height = 1.0 - (y - roi_y_min) / (scale_y_max - roi_y_min)
        cmd.rpm = MIN_RPM + (MAX_RPM - MIN_RPM) * f_height * f_steering
        mem.last_block_rpm = cmd.rpm
    elif mem.frames_since_block < BLOCK_TARGET_GRACE_FRAMES:
        cmd.rpm = mem.last_block_rpm * f_steering
    else:
        cmd.rpm = MIN_RPM + (MAX_RPM - MIN_RPM) * f_steering


def filter_rpm_ramp(cmd, world, mem):
    """Limit how fast the rpm may change per frame."""
    step = np.clip(cmd.rpm - mem.prev_rpm, -MAX_DECEL_PER_FRAME, MAX_ACCEL_PER_FRAME)
    cmd.rpm = mem.prev_rpm + step
    cmd.rpm_changed = cmd.rpm != mem.prev_rpm
    mem.prev_rpm = cmd.rpm


FILTERS = [
    filter_post_pass_clip,
    filter_speed,
    filter_rpm_ramp,
]


# ---------------------------------------------------------------------------
# decide()
# ---------------------------------------------------------------------------

def decide(world, mem):
    """Ask RULES in order; the first answer wins. Then run FILTERS on it."""
    cmd = None
    for rule in RULES:
        cmd = rule(world, mem)
        if cmd is not None:
            break
    if cmd is None:
        cmd = Command(rule='none')
    for f in FILTERS:
        f(cmd, world, mem)
    return cmd

"""What the robot DOES on each camera frame of the open challenge.

Same shape as src/obstacle_challenge/rules.py:

1. build_world()     turns the camera frame + detections into a World: what is seen NOW.
2. learn_direction() sets the driving direction from the first floor line seen.
3. count_turns()     updates the lap counter in Memory.
4. decide()          asks each rule in RULES, in order: "is this your situation?"
                     A rule answers None ("not mine") or a Command. The FIRST rule that
                     answers wins; the rules after it are not asked.
5. FILTERS           then adjust whichever Command won (steering limit, speed).

  World    -- this frame only: walls, lines, front-wall distance, direction, run time.
  Memory   -- what must survive between frames: turn count, direction, previous
              wall error, the speed schedule, the current rpm target.
  Command  -- the answer: steering angle, label of the rule, optional speed, and
              what the FILTERS fill in (rpm).

Nothing in this file touches the motor, servo or camera. main.py's actuate() carries
out the Command, so src/tools/replay.py can run this exact code on saved images.

To add a behaviour: write rule_xxx(world, mem) returning a Command or None, and
insert it into RULES at the right priority. See SURPRISE.md.

Every number used here lives in src/open_challenge/config.py.
"""

from collections import deque
from dataclasses import dataclass, field

import cv2
import numpy as np

from src.open_challenge.config import *
from src.open_challenge.schedule import SpeedSchedule


# ---------------------------------------------------------------------------
# World, Memory, Command
# ---------------------------------------------------------------------------

@dataclass
class World:
    """What the camera sees on THIS frame. Built fresh every frame."""
    detections: dict
    frame: object             # the camera image (BGR), for anything the pipeline misses
    direction: str            # 'clockwise' | 'counter-clockwise' | None (not known yet)
    run_time: float           # seconds since the run started
    blocks: list              # pillars -- empty unless full_frame_roi is set in config.py
    orange: list
    blue: list
    wall_left: float          # black wall pixel areas in the four wall ROIs
    wall_right: float
    inner_left: float
    inner_right: float
    close_black_area: float
    line_wall_pct: float      # % of the line ROI that is black wall
    wall_row: int             # image row where the wall ahead meets the floor (0 = none)
    new_turn: bool = False    # a turn was counted on this frame

    @property
    def clockwise(self):
        return self.direction == 'clockwise'

    @property
    def corner_close(self):
        return (self.close_black_area > CLOSE_BLACK_AREA_THRESHOLD
                or self.line_wall_pct > LINE_ROI_WALL_PCT_THRESHOLD)

    def seen(self, name):
        """Every detection of OBJECTS[name] this frame (any kind of object)."""
        if OBJECTS[name]['kind'] == 'pillar':
            return [b for b in self.blocks if b['color'] == name]
        return self.detections.get('detected_' + name, [])


@dataclass
class Memory:
    """Everything that must survive from one frame to the next."""
    direction: str = None             # learned from the first floor line; can be set
    turns: int = 0
    turn_cooldown: int = 0
    orange_history: deque = field(default_factory=lambda: deque(
        [False] * ORANGE_DETECTION_HISTORY_LENGTH, maxlen=ORANGE_DETECTION_HISTORY_LENGTH))
    prev_wall_error: float = 0.0
    schedule: SpeedSchedule = field(default_factory=SpeedSchedule)
    target_rpm: float = STRAIGHT_RPM  # the rpm last sent to the motor
    hold_speed: bool = False          # True = leave the motor speed alone (final straight)
    extra: dict = field(default_factory=dict)   # free space for new rules' own state
    last_cmd: 'Command' = field(default_factory=lambda: Command())   # last frame's final Command


@dataclass
class Command:
    """One rule's answer for this frame."""
    angle: float = 0.0
    rule: str = ''                # which rule decided (shown in the video and log)
    speed: float = None           # set = use this rpm instead of the speed schedule (0 = stop)
    rpm: float = 0.0              # filled in by filter_speed
    rpm_changed: bool = False
    corner_speed: bool = False    # filter_speed chose CORNER_RPM
    debug: list = field(default_factory=list)


# ---------------------------------------------------------------------------
# 1. Building the World
# ---------------------------------------------------------------------------

_FRONT_WALL_KERNEL = cv2.getStructuringElement(cv2.MORPH_RECT, (1, FRONT_WALL_MIN_THICK))


def front_wall_base_row(frame):
    """Image row where the wall ahead meets the floor (0 = no wall in sight).

    Median over the centre columns of the bottom of the lowest thick dark run.
    Rises monotonically as the robot closes on a corner, well before the
    close-black / line-ROI corner triggers fire, so it is the cue to slow down.
    """
    crop = frame[FRONT_WALL_Y0:FRONT_WALL_Y1, FRONT_WALL_X0:FRONT_WALL_X1]
    dark = (crop.max(axis=2) <= FRONT_WALL_DARK_V).astype(np.uint8) * 255
    dark = cv2.morphologyEx(dark, cv2.MORPH_OPEN, _FRONT_WALL_KERNEL)
    has = dark.any(axis=0)
    if has.mean() < 0.5:
        return 0
    bottoms = dark.shape[0] - 1 - np.argmax(dark[::-1] > 0, axis=0)
    return int(np.median(bottoms[has])) + FRONT_WALL_Y0


def build_world(detections, frame, mem, run_time):
    walls = detections['detected_walls']

    def wall_area(kind):
        return sum(w['area'] for w in walls if w['type'] == kind)

    return World(
        detections=detections,
        frame=frame,
        direction=mem.direction,
        run_time=run_time,
        blocks=detections.get('detected_blocks', []),
        orange=detections['detected_orange'],
        blue=detections['detected_blue'],
        wall_left=wall_area('wall_left'),
        wall_right=wall_area('wall_right'),
        inner_left=wall_area('wall_inner_left'),
        inner_right=wall_area('wall_inner_right'),
        close_black_area=sum(o['area'] for o in detections.get('detected_close_black', [])),
        line_wall_pct=detections.get('line_roi_wall_pct', 0),
        wall_row=front_wall_base_row(frame),
    )


# ---------------------------------------------------------------------------
# 2. Direction and turn counting
# ---------------------------------------------------------------------------

def learn_direction(world, mem):
    """Orange line seen first -> clockwise, blue first -> counter-clockwise. Only
    while mem.direction is still None. Returns True when it was just learned."""
    if mem.direction is not None:
        return False
    if world.orange:
        mem.direction = 'clockwise'
    elif world.blue:
        mem.direction = 'counter-clockwise'
    else:
        return False
    world.direction = mem.direction
    return True


def count_turns(world, mem):
    """A turn = orange seen for (HISTORY_LENGTH - 1) frames in a row after a frame
    without it, then ignored for ORANGE_COOLDOWN_FRAMES. Returns True on a new turn
    (and sets world.new_turn)."""
    mem.orange_history.append(bool(world.orange))
    if mem.turn_cooldown > 0:
        mem.turn_cooldown -= 1
        return False
    if not mem.orange_history[0] and sum(mem.orange_history) == ORANGE_DETECTION_HISTORY_LENGTH - 1:
        mem.turns += 1
        mem.turn_cooldown = ORANGE_COOLDOWN_FRAMES
        world.new_turn = True
        return True
    return False


# ---------------------------------------------------------------------------
# 3. The rules, highest priority first
# ---------------------------------------------------------------------------

def rule_wall_follow(world, mem):
    """Keep the left and right wall areas balanced (PD), adding a fixed turn when a
    corner is close. Always answers, so keep it LAST."""
    inner_left, inner_right = world.inner_left, world.inner_right
    # Only one inner wall seen: inflate it.
    if inner_left > 0 and inner_right == 0:
        inner_left = inner_left * 2 + INNER_ONE_SIDE_BOOST
    elif inner_right > 0 and inner_left == 0:
        inner_right = inner_right * 2 + INNER_ONE_SIDE_BOOST

    total_left = world.wall_left + inner_left
    total_right = world.wall_right + inner_right
    # One side gone: push the side we CAN see hard, so the robot swings away from
    # the wall it is about to clip rather than drifting on a small error.
    if total_left < WALL_ONE_SIDE_MIN and total_right > WALL_ONE_SIDE_SEEN:
        total_right = total_right * 2 + WALL_ONE_SIDE_BOOST
    elif total_right < WALL_ONE_SIDE_MIN and total_left > WALL_ONE_SIDE_SEEN:
        total_left = total_left * 2 + WALL_ONE_SIDE_BOOST

    error = total_left - total_right
    angle = error * WALL_KP + (error - mem.prev_wall_error) * WALL_KD
    mem.prev_wall_error = error

    # Corner turn: towards the inside of the track. Direction not known yet: turn
    # away from the bigger wall.
    if world.direction == 'counter-clockwise':
        corner_turn = -CORNER_TURN_ANGLE
    elif world.direction == 'clockwise':
        corner_turn = CORNER_TURN_ANGLE
    else:
        corner_turn = -CORNER_TURN_ANGLE if total_left < total_right else CORNER_TURN_ANGLE
    # ADDS to the PD angle rather than overriding it, so the wall error still shapes
    # how tight the turn is.
    if world.corner_close:
        angle += corner_turn
    # Both walls out of frame with a floor line under us: mid-corner, keep turning.
    if total_left == 0 and total_right == 0 and (world.orange or world.blue):
        angle += corner_turn

    cmd = Command(angle=angle, rule='wall')
    cmd.debug.extend([f"L:{int(world.wall_left)}", f"R:{int(world.wall_right)}"])
    cmd.debug.extend([f"IL:{int(inner_left)}", f"IR:{int(inner_right)}"])
    cmd.debug.append(f"Err:{int(error)}")
    cmd.debug.append(f"Wall%:{int(world.line_wall_pct)}")
    return cmd


# THE PRIORITY LIST. First rule that answers wins.
RULES = [
    rule_wall_follow,
]


# ---------------------------------------------------------------------------
# 4. Filters: applied to whichever Command won, in this order
# ---------------------------------------------------------------------------

def filter_steer_limit(cmd, world, mem):
    cmd.angle = np.clip(cmd.angle, -STEER_LIMIT, STEER_LIMIT)


def filter_speed(cmd, world, mem):
    """STRAIGHT_RPM on straights, CORNER_RPM around corners (see schedule.py), or
    the rule's own cmd.speed. Nothing changes while mem.hold_speed is set."""
    # The schedule is fed every frame, even when its answer is not used.
    cmd.corner_speed = mem.schedule.update(world.run_time, world.wall_row,
                                           corner_close=world.corner_close,
                                           turn_counted=world.new_turn)
    if mem.hold_speed:
        cmd.rpm = mem.target_rpm
        return
    if cmd.speed is not None:
        cmd.rpm = cmd.speed
    else:
        cmd.rpm = CORNER_RPM if cmd.corner_speed else STRAIGHT_RPM
    cmd.rpm_changed = cmd.rpm != mem.target_rpm
    mem.target_rpm = cmd.rpm


FILTERS = [
    filter_steer_limit,
    filter_speed,
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
    mem.last_cmd = cmd
    return cmd

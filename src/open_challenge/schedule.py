"""Open-challenge speed schedule: STRAIGHT_RPM on straights, CORNER_RPM around corners.

Pure logic (no hardware, no cv2) so it can be replayed on recorded frames.

Slow down BEFORE the corner: the front wall's base row (see
main.front_wall_base_row) climbs as the wall gets close. Speed up SOON AFTER the
turn: once the orange line is counted the corner speed is held for at least
tail_s, then released only when the wall row has fallen below release_row, i.e. the
robot has swung round and faces away from the wall. (A purely timed 0.2 s release
was tried on 2026-09-30 and made the turns looser: it accelerated with the wall
still ahead, wall row 77-106 at release vs 45-59.) The row cue is also ignored
until it has dropped once since the last turn, so the corner just passed cannot
re-arm the slow-down.
"""

from src.open_challenge import config


class SpeedSchedule:
    def __init__(self, slow_row=None, slow_frames=None, release_row=None, tail_s=None):
        self.slow_row = config.CORNER_SLOW_ROW if slow_row is None else slow_row
        self.slow_frames = config.CORNER_SLOW_FRAMES if slow_frames is None else slow_frames
        self.release_row = config.CORNER_RELEASE_ROW if release_row is None else release_row
        self.tail_s = config.CORNER_HOLD_AFTER_TURN_S if tail_s is None else tail_s
        self.corner_mode = False
        self._frames = 0
        self._hold_until = 0.0
        self._rearm_ready = True

    def update(self, now, wall_row, corner_close=False, turn_counted=False):
        """Feed one frame; returns True while corner speed is wanted."""
        if turn_counted:
            self._hold_until = now + self.tail_s
            self._rearm_ready = False
            self.corner_mode = True
        if wall_row < self.release_row:
            self._rearm_ready = True
        self._frames = self._frames + 1 if (self._rearm_ready and wall_row >= self.slow_row) else 0

        if now < self._hold_until:
            self.corner_mode = True
        elif self._frames >= self.slow_frames or corner_close:
            self.corner_mode = True
        elif self.corner_mode and wall_row < self.release_row:
            self.corner_mode = False   # min hold over AND facing away from the wall: full speed
        if now >= self._hold_until:
            self._hold_until = 0.0
        return self.corner_mode

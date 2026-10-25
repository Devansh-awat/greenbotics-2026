"""Per-frame detection + decision log for the open challenge, for comparing runs.

One JSON object per camera frame, written to <run folder>/open_frames.jsonl. Same
approach as obstacle's OverlayLog: append() only builds a small dict and adds it to a
list in memory (microseconds, no I/O, no lock); the file is written once by close(),
after the run's loop has exited. ~100 bytes of RAM-resident dict per field-set per
frame is a few MB for a whole run.

Keys are short but stable, so two runs can be lined up on "f" (camera frame counter,
the same tag video_encoder stores next to open.mp4) or "t" (seconds since run start):

    f     camera frame counter          t     run time, s
    rule  rule that decided             ang   final steering angle (after filters)
    err   wall-follow error (L - R, after the one-side boosts)
    wl wr il ir   black wall pixel areas: outer left/right, inner left/right
    cb    close-black area              lw    % of line ROI that is wall
    row   front wall base row (0 = none)
    cc    corner_close                  cs    corner speed chosen
    or bl [count, total area] of orange / blue line detections
    dir   driving direction             turns turn counter     nt  turn counted this frame
    rpm   measured wheel rpm            rpt   rpm target
    hd    IMU heading                   vib   gyro vibration, deg/s
    lat   capture->decision latency, ms prc   vision time, ms     skip  frames dropped before this one
"""

import json

from src.logs.setup import log


def _lines(objs):
    return [len(objs), round(sum(o['area'] for o in objs))]


class FrameLog:
    def __init__(self, path):
        self.path = path
        self.written = 0
        self._records = []
        self._open = True

    def append(self, frame_counter, world, mem, cmd, rpm, heading, vibration,
               latency_ms, proc_ms, skipped):
        if not self._open:
            return
        self._records.append({
            "f": frame_counter,
            "t": round(world.run_time, 3),
            "rule": cmd.rule,
            "ang": round(float(cmd.angle), 2),
            "err": round(mem.prev_wall_error, 1),
            "wl": round(world.wall_left), "wr": round(world.wall_right),
            "il": round(world.inner_left), "ir": round(world.inner_right),
            "cb": round(world.close_black_area),
            "lw": round(world.line_wall_pct, 1),
            "row": world.wall_row,
            "cc": world.corner_close, "cs": cmd.corner_speed,
            "or": _lines(world.orange), "bl": _lines(world.blue),
            "dir": mem.direction, "turns": mem.turns, "nt": world.new_turn,
            "rpm": round(rpm, 1), "rpt": round(mem.target_rpm),
            "hd": None if heading is None else round(heading, 1),
            "vib": round(vibration, 2),
            "lat": round(latency_ms, 2), "prc": round(proc_ms, 2),
            "skip": skipped,
        })

    def close(self):
        if not self._open:
            return
        self._open = False
        try:
            with open(self.path, "w") as fh:
                for rec in self._records:
                    fh.write(json.dumps(rec, separators=(",", ":")) + "\n")
                    self.written += 1
        except Exception:
            log.exception("Could not write frame log %s", self.path)
            return
        log.info("Frame log written: %s (%d frames)", self.path, self.written)

    @staticmethod
    def load(path):
        """Records as a list of dicts, e.g. pandas.DataFrame(FrameLog.load(p))."""
        with open(path) as fh:
            return [json.loads(line) for line in fh if line.strip()]

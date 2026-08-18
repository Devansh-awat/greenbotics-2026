"""Run-critical power-mode guard.

`src/tools/idle_power.sh apply` trims idle current between runs, and its one
run-critical knob is the CPU governor: powersave pins the Pi 5 at its minimum
clock, which blows the 17.7 ms frame budget. If a run is started while the trim
is still active, this module undoes it automatically -- call
ensure_performance() early in every main program, before the vision pool spins
up. The other trims (Wi-Fi power save, LEDs, Bluetooth) don't affect the run
and are left however they are.

Best-effort by design: it must never crash or stall a run. Direct sysfs writes
are tried first (work when running as root), then passwordless sudo; if both
fail it logs a loud warning and the run proceeds at whatever clock it has.
"""

import glob
import os
import subprocess

# Must match STATE in src/tools/idle_power.sh.
IDLE_STATE_FILE = "/tmp/greenbotics_idle_power.state"
IDLE_SCRIPT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "idle_power.sh")

_POLICY_GLOB = "/sys/devices/system/cpu/cpufreq/policy*/scaling_governor"
RUN_GOVERNOR = "ondemand"


def _governors():
    govs = {}
    for path in glob.glob(_POLICY_GLOB):
        try:
            with open(path) as f:
                govs[path] = f.read().strip()
        except OSError:
            pass
    return govs


def _set_governor(path, governor):
    try:
        with open(path, "w") as f:
            f.write(governor)
        return True
    except OSError:
        pass
    try:
        r = subprocess.run(
            ["sudo", "-n", "tee", path], input=governor,
            capture_output=True, text=True, timeout=5)
        return r.returncode == 0
    except Exception:
        return False


def ensure_performance(logger=None):
    """Undo the idle-power trim if it is (still) active. Returns True when the
    CPU governor is in a run-ready state on exit, False if it could not be
    fixed (the caller should not abort -- the warning is the action)."""
    def info(msg, *a):
        if logger: logger.info(msg, *a)
    def warn(msg, *a):
        if logger: logger.warning(msg, *a)

    # Full revert via the script when its state file says the trim is applied
    # (also restores LEDs/eth exactly as they were). Needs root or sudo -n.
    if os.path.exists(IDLE_STATE_FILE) and os.path.exists(IDLE_SCRIPT):
        cmd = ["bash", IDLE_SCRIPT, "revert"]
        if os.geteuid() != 0:
            cmd = ["sudo", "-n"] + cmd
        try:
            r = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
            if r.returncode == 0:
                info("Idle-power trim was active; reverted for the run.")
            else:
                warn("idle_power.sh revert failed (%s); falling back to "
                     "governor-only fix", (r.stderr or r.stdout).strip()[:200])
        except Exception as e:
            warn("idle_power.sh revert errored (%s); falling back to "
                 "governor-only fix", e)

    govs = _governors()
    bad = {p: g for p, g in govs.items() if g == "powersave"}
    if not bad:
        if govs:
            info("CPU governor OK for run: %s", sorted(set(govs.values()))[0])
        return True

    ok = all(_set_governor(p, RUN_GOVERNOR) for p in bad)
    if ok:
        info("CPU governor was 'powersave' (idle mode); switched to '%s' "
             "for the run.", RUN_GOVERNOR)
        return True
    warn("CPU governor is 'powersave' and could not be changed -- the Pi is "
         "pinned at its minimum clock and WILL miss the frame budget. "
         "Run: sudo %s revert", IDLE_SCRIPT)
    return False


if __name__ == "__main__":
    import logging
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
    ensure_performance(logging.getLogger("power_mode"))

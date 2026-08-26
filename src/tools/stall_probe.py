"""External stall probe -- finds out WHERE the control loop blocks, without touching it.

Background: obstacle runs occasionally lose ~1.3-1.8s inside a single control-loop
iteration (75+ camera frames skipped), roughly 1 run in 50. The per-frame logs can't
explain it: vision is bounded at 100ms by VisionPool's own timeout, no CLOSE BLOCK
maneuver is involved, the camera keeps capturing throughout, and vcgencmd shows no
throttling. Whatever it is, it blocks ONLY the main thread.

This probe runs as a separate process and samples the kernel's own view of every
thread in the robot process. The key field is thread state in /proc/PID/task/TID/stat:

    D  uninterruptible sleep  -> blocked in disk I/O (the overlay .jsonl write, or
                                 anything else that reaches the SD card)
    R  runnable/running       -> CPU starvation or a long pure-CPU stretch
    S  interruptible sleep    -> waiting on a lock, a condvar, or a pipe/event

So a stall spent in D says "SD card / filesystem", a stall spent in R says "we are
oversubscribed on 4 cores", and a stall spent in S says "waiting on something else"
-- three very different fixes. Nothing here perturbs the control loop: separate
process, read-only /proc access, samples buffered in RAM and written out at exit.

Usage (start it first, it waits for the run to appear):

    python3 -m src.tools.stall_probe --wait --out /tmp/stall.log
    # ... in another terminal, do the run as usual ...
    # Ctrl-C the probe (or it exits when the robot process does)

Then read the summary it prints, or the full trace in --out.
"""

import argparse
import os
import signal
import sys
import time

TARGET_MATCH = "src.obstacle_challenge.main"

# /proc/diskstats stat fields, 0-indexed after "major minor name".
DS_READS_DONE, DS_MS_READING = 0, 3
DS_WRITES_DONE, DS_MS_WRITING = 4, 7
DS_IOS_IN_FLIGHT, DS_IO_TICKS = 8, 9


def find_target(match=TARGET_MATCH):
    """Return the pid of the running obstacle-challenge process, or None."""
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        try:
            with open(f"/proc/{entry}/cmdline", "rb") as fh:
                cmdline = fh.read().replace(b"\0", b" ").decode(errors="replace")
        except OSError:
            continue
        if match in cmdline and "stall_probe" not in cmdline:
            return int(entry)
    return None


def read_thread_states(pid):
    """{tid: (comm, state)} for every thread of `pid`.

    The main thread -- the one running the control loop, and the only one we have a
    stall theory about -- is labelled MAIN. CPython does not push Python thread names
    down to the OS, so every worker thread just reports "python3"; the main thread is
    identified structurally instead, by tid == pid, which is always true on Linux.
    """
    out = {}
    try:
        tids = os.listdir(f"/proc/{pid}/task")
    except OSError:
        return out
    for tid in tids:
        try:
            with open(f"/proc/{pid}/task/{tid}/stat", "rb") as fh:
                raw = fh.read().decode(errors="replace")
        except OSError:
            continue
        # comm is parenthesised and may contain spaces -- split on the last ')'.
        close = raw.rfind(")")
        open_ = raw.find("(")
        if close == -1 or open_ == -1:
            continue
        comm = raw[open_ + 1:close]
        rest = raw[close + 2:].split()
        if not rest:
            continue
        tid_i = int(tid)
        if tid_i == pid:
            comm = f"MAIN[{comm}]"
        out[tid_i] = (comm, rest[0])
    return out


def read_proc_io(pid):
    """Bytes this process has passed to syscalls (wchar) and sent to storage."""
    vals = {}
    try:
        with open(f"/proc/{pid}/io", "rb") as fh:
            for line in fh.read().decode(errors="replace").splitlines():
                k, _, v = line.partition(":")
                vals[k.strip()] = int(v.strip() or 0)
    except OSError:
        pass
    return vals.get("wchar", 0), vals.get("write_bytes", 0)


def read_diskstats(dev):
    try:
        with open("/proc/diskstats", "rb") as fh:
            for line in fh.read().decode(errors="replace").splitlines():
                parts = line.split()
                if len(parts) > 13 and parts[2] == dev:
                    return [int(x) for x in parts[3:]]
    except OSError:
        pass
    return None


def read_dirty():
    dirty = writeback = 0
    try:
        with open("/proc/meminfo", "rb") as fh:
            for line in fh.read().decode(errors="replace").splitlines():
                if line.startswith("Dirty:"):
                    dirty = int(line.split()[1])
                elif line.startswith("Writeback:"):
                    writeback = int(line.split()[1])
    except OSError:
        pass
    return dirty, writeback


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pid", type=int, default=None,
                    help="pid to watch (default: auto-find the obstacle run)")
    ap.add_argument("--wait", action="store_true",
                    help="wait for the target process to appear instead of exiting")
    ap.add_argument("--interval", type=float, default=0.02,
                    help="sample period in seconds (default 0.02 = 50 Hz)")
    ap.add_argument("--disk", default="mmcblk0", help="block device to watch")
    ap.add_argument("--out", default="stall_probe.log", help="where to write the trace")
    ap.add_argument("--stall-ms", type=float, default=150.0,
                    help="flag a thread stuck off-R/S for at least this long")
    args = ap.parse_args()

    pid = args.pid
    while pid is None:
        pid = find_target()
        if pid is None:
            if not args.wait:
                sys.exit(f"No process matching {TARGET_MATCH!r} (use --wait or --pid)")
            time.sleep(0.2)
    print(f"stall_probe: watching pid {pid} at {1/args.interval:.0f} Hz -> {args.out}",
          flush=True)

    samples = []          # (t, {tid: (comm, state)}, wchar, write_bytes, dirty, wb, ds)
    stop = {"now": False}
    signal.signal(signal.SIGINT, lambda *_: stop.__setitem__("now", True))
    signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__("now", True))

    t0 = time.monotonic()
    next_tick = t0
    while not stop["now"]:
        next_tick += args.interval
        now = time.monotonic()
        if not os.path.isdir(f"/proc/{pid}"):
            print("stall_probe: target exited", flush=True)
            break
        states = read_thread_states(pid)
        if not states:
            break
        wchar, wbytes = read_proc_io(pid)
        dirty, writeback = read_dirty()
        samples.append((now - t0, states, wchar, wbytes, dirty, writeback,
                        read_diskstats(args.disk)))
        sleep_for = next_tick - time.monotonic()
        if sleep_for > 0:
            time.sleep(sleep_for)
        else:
            next_tick = time.monotonic()   # we fell behind; resync

    report(samples, args)


def report(samples, args):
    if not samples:
        print("stall_probe: no samples collected")
        return

    with open(args.out, "w") as fh:
        fh.write("# t_s  dirty_kB  wb_kB  d_wchar  d_write_bytes  inflight  "
                 "io_ticks_delta  threads(comm=state,...)\n")
        prev = None
        for s in samples:
            t, states, wchar, wbytes, dirty, wb, ds = s
            if prev is None:
                d_wchar = d_wbytes = 0
                d_ticks = 0
                inflight = ds[DS_IOS_IN_FLIGHT] if ds else -1
            else:
                d_wchar = wchar - prev[2]
                d_wbytes = wbytes - prev[3]
                inflight = ds[DS_IOS_IN_FLIGHT] if ds else -1
                d_ticks = (ds[DS_IO_TICKS] - prev[6][DS_IO_TICKS]) if (ds and prev[6]) else 0
            thr = ",".join(f"{c}={st}" for _, (c, st) in sorted(states.items()))
            fh.write(f"{t:8.3f} {dirty:9d} {wb:6d} {d_wchar:8d} {d_wbytes:13d} "
                     f"{inflight:8d} {d_ticks:6d}  {thr}\n")
            prev = s

    # --- sampler gaps: if WE were stalled too, the whole system hiccuped ----
    gaps = []
    for a, b in zip(samples, samples[1:]):
        gap = b[0] - a[0]
        if gap > max(0.1, args.interval * 5):
            gaps.append((a[0], gap))

    # --- per-thread episodes of "not making progress" -----------------------
    # D and R are treated as ONE contiguous episode rather than two kinds, because a
    # thread blocked on the SD card flickers D->R->D as each I/O completes and the
    # next begins: splitting on that turns a single 1.3s stall into dozens of 20ms
    # episodes that all fall under the threshold. The episode is held open across
    # both states and classified afterwards by which state dominated it.
    episodes = []      # (start, dur, n_D, n_R, tid, comm)
    active = {}        # key -> [start, n_D, n_R, last_t]

    def close(key, end_t):
        start, n_d, n_r, _ = active.pop(key)
        episodes.append((start, end_t - start, n_d, n_r, key[0], key[1]))

    for t, states, *_ in samples:
        for tid, (comm, st) in states.items():
            key = (tid, comm)
            if st in ("D", "R"):
                if key not in active:
                    active[key] = [t, 0, 0, t]
                active[key][1 if st == "D" else 2] += 1
                active[key][3] = t
            elif key in active:
                close(key, t)
        for key in [k for k in active if k[0] not in states]:
            close(key, t)
    last_t = samples[-1][0]
    for key in list(active):
        close(key, last_t)

    thresh = args.stall_ms / 1000.0
    # Sorted WORST-FIRST, not chronologically. Sorting by start time and then
    # truncating the printed list hides the very episode you are hunting: a run with
    # dozens of harmless 150-200ms blips will push the one 4.8s stall off the end of
    # the output, which is exactly what happened the first time this tool was used.
    flagged = sorted((e for e in episodes if e[1] >= thresh),
                     key=lambda e: -e[1])

    dur_s = samples[-1][0]
    print(f"\nstall_probe: {len(samples)} samples over {dur_s:.1f}s "
          f"-> full trace in {args.out}")

    if gaps:
        print(f"\n!! sampler itself was delayed {len(gaps)}x -- system-wide stall, "
              f"not just the robot process:")
        for t, g in gaps[:10]:
            print(f"     t={t:7.3f}s  gap={g*1000:7.1f} ms")

    if not flagged:
        print(f"\nNo thread stayed off-S for >= {args.stall_ms:.0f} ms. "
              "If a stall occurred this run, the thread was in S -- waiting on a lock, "
              "condvar or pipe -- which is neither disk I/O nor CPU starvation.")
        return

    print(f"\nThreads that stopped making progress for >= {args.stall_ms:.0f} ms, "
          f"WORST FIRST\n"
          f"  (D = uninterruptible disk wait, R = on-CPU or starved):\n")
    print(f"  {'t_start':>9}  {'dur_ms':>9}  {'%D':>5}  {'%R':>5}  verdict / thread")
    for start, dur, n_d, n_r, tid, comm in flagged[:40]:
        tot = max(1, n_d + n_r)
        pd, pr = 100.0 * n_d / tot, 100.0 * n_r / tot
        if pd >= 60:
            verdict = "DISK-WAIT (SD card / filesystem)"
        elif pr >= 60:
            verdict = "ON-CPU or CPU-STARVED"
        else:
            verdict = "MIXED disk+cpu"
        print(f"  {start:9.3f}  {dur*1000:9.1f}  {pd:5.0f}  {pr:5.0f}  "
              f"{verdict} -- {comm} (tid {tid})")
    if len(flagged) > 40:
        print(f"  ... and {len(flagged) - 40} more (see {args.out})")


if __name__ == "__main__":
    main()

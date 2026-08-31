#!/usr/bin/env python3
"""Clip glider CSV logs down to just the in-flight segments.

A logging session writes up to five time-aligned CSVs that share one clock
(`cf_time_s`): <prefix>_Motor.csv, _Accelerometer.csv, _Controller.csv,
_Connection.csv, and _Events.csv. Most sessions contain a lot of bench/ground
time (sitting still before launch, between hand-launches, after landing). This
tool finds the windows where the aircraft was actually flying and writes clipped
copies containing only those windows -- one set of files per detected flight.

Each flight is one hand-launch glide, bracketed by a pair of OPPOSITE-direction
acc_x jolts with the flight in between. Detection uses only the accelerometer:
  1. LAUNCH = a BIG impulse in the X acceleration (Accelerometer.csv acc_x) -- at
     least ~3 g. On the ground |acc_x| ~ 0.05 g, and handling/walking only reaches
     ~0.9 g, so a >= ACC_X_SPIKE jolt reliably means a hand-launch throw. The brief
     multi-sample "ringing" of one throw is grouped into a single impulse event via
     SPIKE_CLUSTER_S, whose SIGN is taken from its largest-magnitude sample. A
     flight OPENS at a launch impulse.
  2. LANDING = the airframe abruptly going still. The flight CLOSES at the first
     REST_MIN_S stretch of near-constant acc_z (std < REST_STD) after the launch --
     the aircraft down and waiting to be recovered. On the ground acc_z is
     rock-steady at ~1 g (std ~0.001); in flight it wanders far more, so the abrupt
     drop to steady readings cleanly marks touchdown and pins an accurate duration.
     The launch..landing span is accepted as a flight if it was confirmed EITHER by
     a >= LAND_SPIKE opposite-direction decel (a hard landing brakes the airframe
     with an acc_x jolt opposite the throw; real logs show ~2 g, hence
     LAND_SPIKE < ACC_X_SPIKE) OR by being clearly airborne (acc_z std over the span
     >= AIRBORNE_STD). The airborne branch catches soft/belly landings whose decel
     is too gentle to reach LAND_SPIKE but which were unmistakably in the air.
  3. The next launch is the first strong impulse AFTER that landing, so several
     separate throws in one session become several distinct flights, and a
     mid-flight bump (not followed by rest) never splits one glide. A launch that
     never reaches rest before the log ends is an incomplete glide (logging dropped
     out at touchdown) and is NOT clipped. Each span is finally validated by
     requiring its acc_z std to exceed Z_CONST_STD (a constant-Z span was never
     airborne) and to last >= MIN_FLIGHT_S.

Gyro and throttle are not used. Sessions without a >= ACC_X_SPIKE impulse contain
no detected flight. All thresholds are constants below.

Originals are never modified: clipped files are written to an output folder
(default ./clipped/).

Usage:
    python3 clip_flights.py [DIR] [--out OUTDIR] [--dry-run] [--verbose]
"""
from __future__ import annotations

import argparse
import csv
import glob
import os
from dataclasses import dataclass, field
from datetime import datetime
from typing import Dict, List, Optional, Tuple

# --------------------------------------------------------------------------- #
# Detection tuning (values chosen from the bench-vs-flight gap in real logs)
# --------------------------------------------------------------------------- #
# LAUNCH = a BIG X-acceleration impulse (the hand-throw). Ground/handling/walking
# tops out ~0.9 g, real launch throws hit 4-24 g, so 3 g cleanly marks a launch.
ACC_X_SPIKE = 3.0   # g, |acc_x| jolt strong enough to be a launch throw

# One throw rings for a few samples; spikes within this gap are one impulse event
# (so a single launch isn't counted as several takeoffs).
SPIKE_CLUSTER_S = 2.0

# LANDING confirmation = an OPPOSITE-direction acc_x jolt (the airframe
# decelerating as it hits) just before it comes to rest. These are softer and more
# variable than the throw -- real logs show landing decels of only ~2 g -- so they
# use a lower threshold than a launch. Still far above the ~0.9 g of flight/walking
# noise, so it doesn't false-trigger. (A launch is any jolt >= ACC_X_SPIKE; a
# landing is a >= LAND_SPIKE jolt of the sign OPPOSITE the launch.)
LAND_SPIKE = 1.5    # g, min opposite-direction |acc_x| decel to confirm a landing

# A launch..landing span is only a real flight if its Z acceleration is NOT
# constant. Ground acc_z std ~0.001 g (steady 1 g); flight acc_z std is far higher
# as the aircraft banks/pitches. Spans with acc_z std below this were never
# airborne -> dropped.
Z_CONST_STD = 0.02  # g, minimum acc_z std over a span for it to count as flight

# A flight is confirmed by EITHER a hard opposite-direction landing decel
# (>= LAND_SPIKE, below) OR a clearly-airborne span. This is the airborne test: a
# span whose acc_z std exceeds this was unmistakably in the air (real flights show
# 0.4-1.6 g here; ground is ~0.001 g), so it counts as a flight even when the
# landing was too soft to produce a LAND_SPIKE decel (a belly/grass landing). Set
# well above ground/handling noise but below the softest real flight so soft
# landings are caught without false-triggering on bench handling.
AIRBORNE_STD = 0.30  # g, acc_z std above which a span counts as airborne on its own

# The landing is pinned by the airframe going still right after touchdown (sitting
# on the ground awaiting recovery). "Still" = acc_z std below REST_STD (the same
# rock-steady 1 g as bench ground time) sustained for REST_MIN_S. The onset of that
# stretch is the touchdown time; it's found by sliding a REST_MIN_S window forward
# from the launch in REST_STEP_S steps. A launch that never reaches rest before the
# log ends is an incomplete glide (logging dropped out) and is NOT clipped.
REST_STD = 0.01     # g, max acc_z std for the airframe to count as "sitting still"
REST_MIN_S = 3.0    # s, how long it must stay still to confirm a landing/recovery
REST_STEP_S = 0.5   # s, granularity of the rest-onset search

# Pad each flight window by this much on each side so the launch/landing impulse
# itself and its immediate transient aren't clipped off.
EDGE_PAD_S = 1.0
# Discard detected spans shorter than this -- e.g. the brief ring between two
# samples of the same throw -- as not real flights. Hand-tossed glides can be very
# short (a 2-3 s toss), so this is set just above the throw-ring duration.
MIN_FLIGHT_S = 2.0

# Stream suffix -> whether its rows are keyed on cf_time_s (True) or host time.
STREAMS = {
    "Motor": True,
    "Accelerometer": True,
    "Controller": True,
    "Connection": True,
    "Events": False,   # keyed on host_time_iso; mapped via breakpoints
}


@dataclass
class Sample:
    t: float
    ax: float        # signed acc_x; its magnitude marks impulses, its SIGN tells a
                     # launch throw from an (opposite-direction) landing deceleration
    z: float         # acc_z, used to reject constant-Z (ground) spans between impulses


@dataclass
class Session:
    prefix: str                      # full path prefix (no _Stream.csv)
    # cf_time -> host datetime pairs harvested from "# BREAKPOINT" lines, used to
    # translate flight windows into wall-clock time for the Events stream.
    cf_to_host: List[Tuple[float, datetime]] = field(default_factory=list)


# --------------------------------------------------------------------------- #
# Parsing helpers
# --------------------------------------------------------------------------- #
def _is_comment(row: List[str]) -> bool:
    return bool(row) and row[0].startswith("#")


def _is_header(row: List[str]) -> bool:
    return bool(row) and row[0] in ("cf_time_s", "host_time_iso")


def _parse_host(s: str) -> Optional[datetime]:
    try:
        return datetime.fromisoformat(s)
    except (ValueError, TypeError):
        return None


def _iter_data_rows(path: str):
    """Yield data rows (skipping comments/headers) from a stream CSV."""
    try:
        fh = open(path, newline="")
    except FileNotFoundError:
        return
    with fh:
        for row in csv.reader(fh):
            if not row or _is_comment(row) or _is_header(row):
                continue
            yield row


def _harvest_breakpoints(prefix: str) -> List[Tuple[float, datetime]]:
    """Build cf_time<->host_time anchors from '# BREAKPOINT,host,label' lines in
    the cf_time-keyed streams. A breakpoint line sits between two data rows, so
    its cf_time is approximated by the next data row's cf_time."""
    anchors: List[Tuple[float, datetime]] = []
    for suffix, cf_keyed in STREAMS.items():
        if not cf_keyed:
            continue
        path = f"{prefix}_{suffix}.csv"
        try:
            fh = open(path, newline="")
        except FileNotFoundError:
            continue
        with fh:
            rows = list(csv.reader(fh))
        for i, row in enumerate(rows):
            if not row or not row[0].startswith("# BREAKPOINT"):
                continue
            host = _parse_host(row[1]) if len(row) > 1 else None
            if host is None:
                continue
            # find the next real data row's cf_time
            for nxt in rows[i + 1:]:
                if nxt and not _is_comment(nxt) and not _is_header(nxt):
                    try:
                        anchors.append((float(nxt[0]), host))
                    except (ValueError, IndexError):
                        pass
                    break
    anchors.sort()
    return anchors


def _cf_to_host(anchors: List[Tuple[float, datetime]], t: float) -> Optional[datetime]:
    """Linear-interpolate a cf_time to wall-clock using breakpoint anchors."""
    if not anchors:
        return None
    if len(anchors) == 1:
        cf0, h0 = anchors[0]
        return h0
    # clamp/interp
    if t <= anchors[0][0]:
        (cf0, h0), (cf1, h1) = anchors[0], anchors[1]
    elif t >= anchors[-1][0]:
        (cf0, h0), (cf1, h1) = anchors[-2], anchors[-1]
    else:
        (cf0, h0), (cf1, h1) = anchors[0], anchors[1]
        for j in range(1, len(anchors)):
            if anchors[j][0] >= t:
                (cf0, h0), (cf1, h1) = anchors[j - 1], anchors[j]
                break
    span = cf1 - cf0
    if abs(span) < 1e-9:
        return h0
    frac = (t - cf0) / span
    return h0 + (h1 - h0) * frac


# --------------------------------------------------------------------------- #
# Activity signal + flight-window detection
# --------------------------------------------------------------------------- #
def _build_activity(prefix: str) -> List[Sample]:
    """Read the accelerometer into a time-sorted series carrying signed acc_x (its
    magnitude finds the launch/landing impulses, its sign tells the two apart) and
    acc_z (to reject constant-Z ground spans)."""
    out: List[Sample] = []
    for row in _iter_data_rows(f"{prefix}_Accelerometer.csv"):
        try:
            t = float(row[0])
            ax = float(row[1])
            az = float(row[3])
        except (ValueError, IndexError):
            continue
        out.append(Sample(t, ax, az))
    out.sort(key=lambda s: s.t)
    return out


def _impulse_events(samples: List[Sample]) -> List[Tuple[float, float, int]]:
    """Group samples with |acc_x| >= ACC_X_SPIKE into (start, end, sign) impulse
    events, merging the multi-sample ring of one throw (spikes within
    SPIKE_CLUSTER_S). ``sign`` is +1/-1 for the whole cluster.

    Only spikes of the SAME sign are merged: the ring of one throw is all one
    direction, whereas a launch and its landing deceleration point OPPOSITE ways.
    A sign flip therefore always starts a new event, so a short hand-toss -- whose
    launch (+) and landing (-) spikes can fall within SPIKE_CLUSTER_S of each other
    -- still yields two separate events (launch then landing) instead of being
    collapsed into one, which would hide the flight."""
    events: List[List[float]] = []   # [start, end, peak_abs, peak_signed, sign]
    for s in samples:
        if abs(s.ax) < ACC_X_SPIKE:
            continue
        cur_sign = 1 if s.ax >= 0 else -1
        if (events and s.t - events[-1][1] <= SPIKE_CLUSTER_S
                and events[-1][4] == cur_sign):
            events[-1][1] = s.t
            if abs(s.ax) > events[-1][2]:
                events[-1][2] = abs(s.ax)
                events[-1][3] = s.ax
        else:
            events.append([s.t, s.t, abs(s.ax), s.ax, cur_sign])
    return [(a, b, sg) for a, b, _, _, sg in events]


def _zstd(samples: List[Sample], t0: float, t1: float) -> float:
    """Population std of acc_z over the samples whose time falls in [t0, t1]."""
    zs = [s.z for s in samples if t0 <= s.t <= t1]
    n = len(zs)
    if n < 2:
        return 0.0
    mean = sum(zs) / n
    return (sum((z - mean) ** 2 for z in zs) / n) ** 0.5


def _at_rest(samples: List[Sample], t0: float, t1: float) -> bool:
    """True if acc_z holds effectively constant (std < REST_STD) across [t0, t1],
    i.e. the airframe is sitting still. Needs a few samples so a sparse tail can't
    look 'quiet' just by containing too few points to vary."""
    zs = [s.z for s in samples if t0 <= s.t <= t1]
    n = len(zs)
    if n < 3:
        return False
    mean = sum(zs) / n
    std = (sum((z - mean) ** 2 for z in zs) / n) ** 0.5
    return std < REST_STD


def _rest_onset_after(samples: List[Sample], t: float,
                      session_end: float) -> Optional[float]:
    """Time at which the airframe first settles into a sustained rest stretch at or
    after ``t`` -- i.e. the landing/touchdown time. A stretch is REST_MIN_S of
    near-zero acc_z variation (std < REST_STD), the aircraft sitting on the ground
    awaiting recovery. Returns None when no such stretch exists in the remaining
    data (e.g. logging dropped out right at touchdown), so the caller can close the
    flight at the log's end instead."""
    if session_end - t < REST_MIN_S:
        return None
    start = t
    last_start = session_end - REST_MIN_S
    while start <= last_start:
        if _at_rest(samples, start, start + REST_MIN_S):
            return start
        start += REST_STEP_S
    return None


def _opposite_decel(samples: List[Sample], t0: float, t1: float,
                    launch_sign: int) -> float:
    """Peak magnitude of acc_x in the OPPOSITE direction to the launch over
    [t0, t1] -- i.e. the strength of the landing deceleration. 0 if none."""
    opp = 0.0
    for s in samples:
        if t0 <= s.t <= t1 and (s.ax < 0) == (launch_sign > 0):
            opp = max(opp, abs(s.ax))
    return opp


def _detect_windows(samples: List[Sample]) -> List[Tuple[float, float]]:
    """Each flight is one hand-launch glide, bracketed by a pair of OPPOSITE-
    direction acc_x jolts. It OPENS at a launch throw (a >= ACC_X_SPIKE impulse)
    and CLOSES at the landing, which is pinned where the airframe abruptly goes
    still (a REST_MIN_S stretch of near-constant acc_z -- the aircraft down and
    awaiting recovery) and is confirmed by a >= LAND_SPIKE deceleration in the
    direction OPPOSITE the throw somewhere between the launch and that stillness.
    The next launch is the first strong impulse after the landing, so several
    separate throws become several distinct flights, and a mid-flight bump (not
    followed by rest) never splits one glide. A launch that never reaches rest
    before the log ends is an incomplete glide (logging dropped out) and is NOT
    clipped. Spans too short or with constant acc_z (never airborne) are dropped."""
    if not samples:
        return []
    events = _impulse_events(samples)
    session_end = samples[-1].t
    windows = []
    launch_ok_from = samples[0].t   # earliest time an impulse can count as a launch
    for ev_start, ev_end, sign in events:
        if ev_start < launch_ok_from:
            continue                # inside a prior flight: not a new launch
        a = ev_end                  # launch: the glide opens here
        # Landing = where the airframe first settles into rest after the launch.
        b = _rest_onset_after(samples, a, session_end)
        if b is None:
            continue                # never came to rest: incomplete glide, skip
        launch_ok_from = b          # impulses before the landing aren't new launches
        if b - a < MIN_FLIGHT_S:
            continue
        # Confirm a real flight by EITHER a hard opposite-direction decel at
        # touchdown (a firm landing) OR a clearly-airborne span (acc_z varying far
        # more than the near-constant ground reading). The airborne branch rescues
        # soft/belly landings whose decel stays below LAND_SPIKE but which were
        # plainly in the air. The final Z_CONST_STD gate still rejects a constant-Z
        # span reached only via a hard decel (never actually airborne).
        zst = _zstd(samples, a, b)
        hard_landing = _opposite_decel(samples, a, b, sign) >= LAND_SPIKE
        airborne = zst >= AIRBORNE_STD
        if (hard_landing or airborne) and zst >= Z_CONST_STD:
            windows.append((a - EDGE_PAD_S, b + EDGE_PAD_S))
    return windows


# --------------------------------------------------------------------------- #
# Clipping
# --------------------------------------------------------------------------- #
def _clip_stream(prefix: str, suffix: str, cf_keyed: bool,
                 window: Tuple[float, float], anchors, out_path: str) -> Optional[int]:
    """Write a clipped copy of one stream for one flight window. Returns the number
    of data rows kept, or None if the source file is absent. The output is ALWAYS
    written when the source exists -- even with zero data rows (just the schema and
    column-header lines) -- so every clip has a consistent, complete file set. This
    matters for sparse host-keyed streams like Events: a short flight window may
    contain no events, but the clip should still carry an Events.csv."""
    src = f"{prefix}_{suffix}.csv"
    try:
        fh = open(src, newline="")
    except FileNotFoundError:
        return None
    t0, t1 = window
    host0 = _cf_to_host(anchors, t0)
    host1 = _cf_to_host(anchors, t1)
    kept = 0
    with fh:
        rows = list(csv.reader(fh))
    out_rows: List[List[str]] = []
    for i, row in enumerate(rows):
        if not row:
            continue
        if _is_header(row) or (row[0].startswith("#") and not row[0].startswith("# BREAKPOINT")):
            out_rows.append(row)             # schema + column headers: always keep
            continue
        if row[0].startswith("# BREAKPOINT"):
            # Keep a breakpoint if its position falls inside the window.
            keep = False
            if cf_keyed:
                for nxt in rows[i + 1:]:
                    if nxt and not _is_comment(nxt) and not _is_header(nxt):
                        try:
                            keep = t0 <= float(nxt[0]) <= t1
                        except (ValueError, IndexError):
                            pass
                        break
            else:
                h = _parse_host(row[1]) if len(row) > 1 else None
                keep = h is not None and host0 is not None and host0 <= h <= host1
            if keep:
                out_rows.append(row)
            continue
        # data row
        if cf_keyed:
            try:
                t = float(row[0])
            except (ValueError, IndexError):
                continue
            if t0 <= t <= t1:
                out_rows.append(row); kept += 1
        else:
            h = _parse_host(row[0]) if row else None
            if h is not None and host0 is not None and host0 <= h <= host1:
                out_rows.append(row); kept += 1
    with open(out_path, "w", newline="") as ofh:
        csv.writer(ofh).writerows(out_rows)
    return kept


def _clip_console(prefix: str, window: Tuple[float, float], anchors,
                  out_path: str) -> Optional[int]:
    """Write a copy of the session's Console.txt for one flight window. Returns the
    number of lines written, or None if the console is absent. The console is a
    plain host-time-keyed text log ('<ISO timestamp>  <message>'), so lines are kept
    when their leading timestamp falls in the window's wall-clock span. A line with
    no parseable timestamp (a wrapped continuation of the message above) inherits the
    previous line's keep decision, so multi-line messages stay whole. If the window
    captures NO console lines (common for a short hand-toss flight, whose few seconds
    hold no console output -- setup/arm/PID all print outside it), the FULL session
    console is copied instead, so the clip always carries useful, non-empty context
    rather than a 0-byte file. Always written when the source exists."""
    src = f"{prefix}_Console.txt"
    try:
        with open(src) as fh:
            all_lines = fh.readlines()
    except FileNotFoundError:
        return None
    t0, t1 = window
    host0 = _cf_to_host(anchors, t0)
    host1 = _cf_to_host(anchors, t1)
    kept_lines: List[str] = []
    keep = False
    for line in all_lines:
        stamp = line.split(None, 1)[0] if line.strip() else ""
        h = _parse_host(stamp)
        if h is not None:
            keep = host0 is not None and host0 <= h <= host1
        # else: continuation line -> reuse previous `keep`
        if keep:
            kept_lines.append(line)
    if not kept_lines:
        kept_lines = all_lines   # nothing in-window: keep full session context
    with open(out_path, "w") as ofh:
        ofh.writelines(kept_lines)
    return len(kept_lines)


def process_session(prefix: str, out_dir: str, dry_run: bool,
                    verbose: bool) -> Tuple[int, int]:
    """Detect and clip all flights in one session. Returns (n_flights, n_files)."""
    samples = _build_activity(prefix)
    windows = _detect_windows(samples)
    name = os.path.basename(prefix)
    if not windows:
        if verbose:
            span = f"{samples[0].t:.1f}-{samples[-1].t:.1f}s" if samples else "no data"
            print(f"  {name}: no flight ({span})")
        return (0, 0)
    anchors = _harvest_breakpoints(prefix)
    total = sum(w[1] - w[0] for w in windows)
    print(f"  {name}: {len(windows)} flight(s), {total:.0f}s in air -> "
          + ", ".join(f"[{a:.1f}-{b:.1f}]" for a, b in windows))
    if dry_run:
        return (len(windows), 0)
    os.makedirs(out_dir, exist_ok=True)
    n_files = 0
    for idx, window in enumerate(windows, 1):
        for suffix, cf_keyed in STREAMS.items():
            out_path = os.path.join(out_dir, f"{name}_flight{idx}_{suffix}.csv")
            if _clip_stream(prefix, suffix, cf_keyed, window, anchors, out_path) is not None:
                n_files += 1
        # Console.txt (plain text, not a CSV STREAM) so the clip carries its log.
        con_path = os.path.join(out_dir, f"{name}_flight{idx}_Console.txt")
        if _clip_console(prefix, window, anchors, con_path) is not None:
            n_files += 1
    return (len(windows), n_files)


def find_sessions(directory: str, match: Optional[str] = None) -> List[str]:
    """A session is any prefix that has at least one recognised stream file.
    ``match`` optionally restricts to prefixes whose basename contains it (e.g.
    a date like "20260722")."""
    prefixes = set()
    for suffix in STREAMS:
        # Recurse so sessions grouped into per-day subfolders (logs/YYYYMMDD/...)
        # and Misc_Flights are all found when pointed at the logs root.
        for path in glob.glob(os.path.join(directory, "**", f"*_{suffix}.csv"),
                              recursive=True):
            prefixes.add(path[: -(len(suffix) + 5)])  # strip "_<suffix>.csv"
    # Ignore anything already produced by this tool.
    out = [p for p in prefixes if "_flight" not in os.path.basename(p)]
    if match:
        out = [p for p in out if match in os.path.basename(p)]
    return sorted(out)


def main() -> None:
    ap = argparse.ArgumentParser(description="Clip glider logs to flight-only segments.")
    ap.add_argument("directory", nargs="?", default=".", help="folder of logs (default: .)")
    ap.add_argument("--out", default=None, help="output folder (default: <dir>/clipped)")
    ap.add_argument("--dry-run", action="store_true", help="report detected flights, write nothing")
    ap.add_argument("--verbose", action="store_true", help="also list sessions with no flight")
    ap.add_argument("--match", default=None,
                    help="only process sessions whose name contains this (e.g. a date 20260722)")
    args = ap.parse_args()

    out_dir = args.out or os.path.join(args.directory, "clipped")
    sessions = find_sessions(args.directory, args.match)
    print(f"Scanning {len(sessions)} session(s) in {os.path.abspath(args.directory)}")
    if not args.dry_run:
        print(f"Writing clipped flights to {os.path.abspath(out_dir)}\n")
    else:
        print("(dry run -- no files written)\n")

    tot_flights = tot_files = tot_with = 0
    for prefix in sessions:
        n_flights, n_files = process_session(prefix, out_dir, args.dry_run, args.verbose)
        tot_flights += n_flights
        tot_files += n_files
        tot_with += 1 if n_flights else 0
    print(f"\nDone: {tot_with}/{len(sessions)} sessions contained flight; "
          f"{tot_flights} flight(s) detected"
          + ("" if args.dry_run else f", {tot_files} clipped files written."))


if __name__ == "__main__":
    main()

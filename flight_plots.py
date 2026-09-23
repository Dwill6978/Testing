#!/usr/bin/env python3
"""Plot a single glider flight: gyro + user input, accel, and control-surface
deflections on one stacked, time-synced figure with major flight events marked.

A flight's data lives in the five time-aligned CSV streams that share the
`cf_time_s` clock (see clip_flights.py). This module turns one such set -- either
a raw session (auto-detecting its flight window) or an already-clipped
`*_flightN_*` set -- into a matplotlib Figure with three stacked subplots that
share the x (time) axis:

  Row 1  Gyro rates (roll/pitch/yaw, deg/s) with the pilot's rate setpoints
         overlaid (Controller.csv gyro_* and set_*, both already deg/s).
  Row 2  Accelerometer (acc_x/y/z, g).
  Row 3  Control-surface deflections normalised to -1..+1, translated from the
         raw motor commands (Motor.csv motor_m4=aileron, motor_m2=elevator,
         motor_m1=aileron2; centred at 32767), plus throttle (servo_cmd).

Major flight events (arm, disarm, failsafe/connection-loss, PID applied, plus
derived takeoff/land) are drawn as labelled vertical lines across all three rows.
Events are keyed on host wall-clock time and mapped back onto the cf_time axis
using the breakpoint anchors embedded in the streams.

The module deliberately builds Figures via the object API (no pyplot), so it is
safe to call headless and to embed the returned Figure in a Qt canvas.

CLI:
    python3 flight_plots.py [DIR] [--match SUBSTR] [--save OUTDIR]
"""
from __future__ import annotations

import argparse
import glob
import os
import re
from dataclasses import dataclass, field
from datetime import datetime
from typing import Dict, List, Optional, Tuple

import matplotlib
matplotlib.use("Agg")  # overridden to QtAgg by the GUI before importing this
from matplotlib.figure import Figure

import clip_flights as cf

# Motor command centre and full-scale span -> deflection = (cmd-centre)/span.
SERVO_CENTER = 32767.0
SERVO_SPAN = 32767.0
# servo_cmd (throttle) raw range -> normalise to 0..1.
THROTTLE_MAX = 65535.0

# Radians/sec -> deg/sec. Applies to the gyro_* columns ONLY.
#
# The two halves of Controller.csv do not share a unit, despite sitting on the
# same axes in the plot:
#   gyro_*  <- controller.r_roll/r_pitch/r_yaw, which the firmware stores as
#              radians(sensors->gyro.x) (controller_pid.c:139-141). Genuinely
#              rad/s, so it needs this factor.
#   set_*   <- controller.rollRate/pitchRate/yawRate, i.e. rateDesired.*, which
#              is handed straight to attitudeControllerCorrectRatePID() next to
#              the raw deg/s sensors->gyro.x (controller_pid.c:126-127). Already
#              deg/s -- converting it again inflated every setpoint by 57.3x, so
#              a 45 deg/s command plotted as ~2578.
# The subscription has used these same variable names since the logger was
# written, so this applies uniformly to old and new CSVs alike.
RAD2DEG = 57.29577951308232

# Which event types get a marked line, and how to draw them.
# name -> (colour, label)
MAJOR_EVENTS = {
    "MOTOR_ARM": ("#2ca02c", "arm"),
    "MOTOR_DISARM": ("#d62728", "disarm"),
    "FAILSAFE_DISARM": ("#ff7f0e", "failsafe"),
    "MOTOR_ARM_REJECTED": ("#9467bd", "arm rej"),
    "PID_APPLIED": ("#8c564b", "PID"),
}
# Derived-event styling.
DERIVED_STYLE = {
    "takeoff": ("#1f77b4", "takeoff"),
    "land": ("#7f7f7f", "land"),
}

# Events logged as "# BREAKPOINT" markers rather than typed Events.csv rows. In
# Events.csv these land as `host,BREAKPOINT,<label>,,` so the real name is in the
# label column (row[2]), not the kind column (row[1]). Manual-override and
# autonomous-mode toggles are recorded this way; map the labels of interest here.
# (FLIGHT_START_MOTOR_ARM / FLIGHT_END_MOTOR_DISARM are deliberately omitted: they
# duplicate the typed MOTOR_ARM / MOTOR_DISARM events already drawn via MAJOR_EVENTS.)
BREAKPOINT_EVENTS = {
    "MANUAL_OVERRIDE_ON":  ("#e377c2", "man ovr on"),
    "MANUAL_OVERRIDE_OFF": ("#bcbd22", "man ovr off"),
    "AUTONOMOUS_ENABLED":  ("#17becf", "auto on"),
    "AUTONOMOUS_DISABLED": ("#9edae5", "auto off"),
}


@dataclass
class FlightData:
    name: str
    window: Optional[Tuple[float, float]]         # (t0, t1) in cf_time
    # Row 1
    gyro_t: List[float] = field(default_factory=list)
    gyro: Dict[str, List[float]] = field(default_factory=dict)   # roll/pitch/yaw
    set_t: List[float] = field(default_factory=list)
    setp: Dict[str, List[float]] = field(default_factory=dict)
    # Row 2
    acc_t: List[float] = field(default_factory=list)
    acc: Dict[str, List[float]] = field(default_factory=dict)    # x/y/z
    # Row 3
    mot_t: List[float] = field(default_factory=list)
    surf: Dict[str, List[float]] = field(default_factory=dict)   # ail/elev/ail2
    throttle: List[float] = field(default_factory=list)
    # Events: list of (cf_time, kind, colour, label)
    events: List[Tuple[float, str, str, str]] = field(default_factory=list)
    # Detected flight windows (cf_time) to shade on a full-session plot so you can
    # see where each clip was cut from. Empty for single (clipped/raw) flights.
    clip_windows: List[Tuple[float, float]] = field(default_factory=list)
    # (cf_time) spans where manual override was active. Collected from the events
    # file WITHOUT the window filter, because the override toggle usually happens
    # before takeoff and so falls outside a clipped flight's window.
    override_spans: List[Tuple[float, float]] = field(default_factory=list)


# --------------------------------------------------------------------------- #
# Loading
# --------------------------------------------------------------------------- #
def _in_window(t: float, window: Optional[Tuple[float, float]]) -> bool:
    return window is None or (window[0] <= t <= window[1])


def _load_controller(fd: FlightData, prefix: str) -> None:
    fd.gyro = {"roll": [], "pitch": [], "yaw": []}
    fd.setp = {"roll": [], "pitch": [], "yaw": []}
    for row in cf._iter_data_rows(f"{prefix}_Controller.csv"):
        try:
            t = float(row[0])
            gr, gp, gy = float(row[1]), float(row[2]), float(row[3])
            sr, sp, sy = float(row[4]), float(row[5]), float(row[6])
        except (ValueError, IndexError):
            continue
        if not _in_window(t, fd.window):
            continue
        fd.gyro_t.append(t)
        fd.gyro["roll"].append(gr * RAD2DEG)
        fd.gyro["pitch"].append(gp * RAD2DEG)
        fd.gyro["yaw"].append(gy * RAD2DEG)
        fd.set_t.append(t)
        # No RAD2DEG here: rateDesired.* is already deg/s. See the constant.
        fd.setp["roll"].append(sr)
        fd.setp["pitch"].append(sp)
        fd.setp["yaw"].append(sy)


def _load_accel(fd: FlightData, prefix: str) -> None:
    fd.acc = {"x": [], "y": [], "z": []}
    for row in cf._iter_data_rows(f"{prefix}_Accelerometer.csv"):
        try:
            t = float(row[0])
            ax, ay, az = float(row[1]), float(row[2]), float(row[3])
        except (ValueError, IndexError):
            continue
        if not _in_window(t, fd.window):
            continue
        fd.acc_t.append(t)
        fd.acc["x"].append(ax)
        fd.acc["y"].append(ay)
        fd.acc["z"].append(az)


def _deflection(cmd: float) -> float:
    d = (cmd - SERVO_CENTER) / SERVO_SPAN
    return max(-1.0, min(1.0, d))


def _load_motor(fd: FlightData, prefix: str) -> None:
    # rudder = motor_m3 (surface 3), appended as column 5 in newer logs. Older logs
    # only have 5 columns (through servo_cmd) and no rudder; those simply leave the
    # rudder series empty so it isn't plotted, rather than erroring.
    fd.surf = {"aileron": [], "elevator": [], "aileron2": [], "rudder": []}
    for row in cf._iter_data_rows(f"{prefix}_Motor.csv"):
        try:
            t = float(row[0])
            m4, m1, m2 = float(row[1]), float(row[2]), float(row[3])
            thr = float(row[4])
        except (ValueError, IndexError):
            continue
        if not _in_window(t, fd.window):
            continue
        m3 = None
        if len(row) > 5:
            try:
                m3 = float(row[5])
            except ValueError:
                m3 = None
        fd.mot_t.append(t)
        fd.surf["aileron"].append(_deflection(m4))
        fd.surf["elevator"].append(_deflection(m2))
        fd.surf["aileron2"].append(_deflection(m1))
        if m3 is not None:
            fd.surf["rudder"].append(_deflection(m3))
        fd.throttle.append(max(0.0, min(1.0, thr / THROTTLE_MAX)))


def _host_to_cf(anchors: List[Tuple[float, datetime]], h: datetime) -> Optional[float]:
    """Inverse of clip_flights._cf_to_host: map a wall-clock time back to cf_time
    by linear interpolation between breakpoint anchors."""
    if not anchors:
        return None
    if len(anchors) == 1:
        return anchors[0][0]
    if h <= anchors[0][1]:
        (cf0, h0), (cf1, h1) = anchors[0], anchors[1]
    elif h >= anchors[-1][1]:
        (cf0, h0), (cf1, h1) = anchors[-2], anchors[-1]
    else:
        (cf0, h0), (cf1, h1) = anchors[0], anchors[1]
        for j in range(1, len(anchors)):
            if anchors[j][1] >= h:
                (cf0, h0), (cf1, h1) = anchors[j - 1], anchors[j]
                break
    span = (h1 - h0).total_seconds()
    if abs(span) < 1e-9:
        return cf0
    frac = (h - h0).total_seconds() / span
    return cf0 + (cf1 - cf0) * frac


def _load_events(fd: FlightData, prefix: str) -> None:
    anchors = cf._harvest_breakpoints(prefix)
    for row in cf._iter_data_rows(f"{prefix}_Events.csv"):
        if len(row) < 2:
            continue
        kind = row[1]
        if kind in MAJOR_EVENTS:
            disp_kind, (colour, label) = kind, MAJOR_EVENTS[kind]
        elif kind == "BREAKPOINT" and len(row) >= 3 and row[2] in BREAKPOINT_EVENTS:
            # Manual-override / autonomous toggles carry their real name in row[2].
            disp_kind, (colour, label) = row[2], BREAKPOINT_EVENTS[row[2]]
        else:
            continue
        h = cf._parse_host(row[0])
        if h is None:
            continue
        t = _host_to_cf(anchors, h)
        if t is None or not _in_window(t, fd.window):
            continue
        fd.events.append((t, disp_kind, colour, label))
    fd.events.sort(key=lambda e: e[0])


def _load_override_spans(fd: FlightData, prefix: str) -> None:
    """Collect the (cf_time) spans during which manual override was active.

    Read from ``prefix``'s own events, unwindowed: the ON toggle is usually
    flipped on the ground, so a windowed read would miss the state a flight was
    already in. A clipped flight should be given its parent session's prefix --
    the clip's own events file keeps only the rows inside its window, and the
    ``# BREAKPOINT`` anchor lines needed to map host time to cf_time may not have
    survived the cut at all. Clips copy cf_time verbatim, so spans measured on
    the session apply to its clips unchanged.
    """
    anchors = cf._harvest_breakpoints(prefix)
    if not anchors:
        return
    override_on: Optional[float] = None
    for row in cf._iter_data_rows(f"{prefix}_Events.csv"):
        if len(row) < 3 or row[1] != "BREAKPOINT":
            continue
        h = cf._parse_host(row[0])
        if h is None:
            continue
        t = _host_to_cf(anchors, h)
        if t is None:
            continue
        if row[2] == "MANUAL_OVERRIDE_ON" and override_on is None:
            override_on = t
        elif row[2] == "MANUAL_OVERRIDE_OFF" and override_on is not None:
            fd.override_spans.append((override_on, t))
            override_on = None
    if override_on is not None:
        # Override was never switched back off before the session ended.
        fd.override_spans.append((override_on, float("inf")))


def _drop_override_setpoints(fd: FlightData) -> None:
    """Discard the commanded-rate samples recorded while manual override was on.

    Manual override streams raw servo/motor PWM on the generic setpoint channel.
    That decoder leaves setpoint_t zeroed, and a zeroed mode.roll/pitch is
    modeDisable -- which controller_pid reads as "hold absolute attitude 0", not
    as "no command". So the attitude PID runs against a level target while the
    airframe is hand-flown, and its unclamped output lands in rateDesired, i.e.
    in the controller.rollRate/pitchRate columns these traces come from. Add the
    ground station's interleaved zero-setpoint supervisor keepalive and the
    result is a spike train that swamps the shared y-axis and hides the gyro.

    Those samples were never pilot commands, so they are dropped. Dropped rather
    than NaN-filled so setp stays index-aligned with set_t for every consumer,
    and dropped rather than clamped so nothing fake is drawn. The gyro traces are
    untouched -- they are real measurements throughout.

    Firmware build 6+ fixes this at the source (the decoder now selects
    modeVelocity with a zero rate); this keeps every earlier log readable.
    """
    if not fd.override_spans or not fd.set_t:
        return
    keep = [i for i, t in enumerate(fd.set_t)
            if not any(a <= t <= b for a, b in fd.override_spans)]
    if len(keep) == len(fd.set_t):
        return
    fd.set_t = [fd.set_t[i] for i in keep]
    for axis, vals in fd.setp.items():
        fd.setp[axis] = [vals[i] for i in keep]


def _add_derived_events(fd: FlightData) -> None:
    """Derive takeoff and landing markers with the SAME model clip_flights uses to
    cut the window, so they stay consistent with it: takeoff = the launch throw
    (first |acc_x| >= ACC_X_SPIKE jolt); landing = where the airframe then settles
    into stillness (rest onset) -- the touchdown. Deriving landing from the settle,
    not from a last big spike, keeps a soft landing (whose decel is below the launch
    threshold) placed at the true touchdown instead of being dropped (rawf2) or
    stuck on the throw's own recoil right after takeoff (rawf3)."""
    if not fd.acc_t:
        return
    # Takeoff = the first launch-strength jolt; its sign is the throw direction.
    launch_t = launch_sign = None
    for t, ax in zip(fd.acc_t, fd.acc["x"]):
        if abs(ax) >= cf.ACC_X_SPIKE:
            launch_t = t
            launch_sign = 1 if ax >= 0 else -1
            break
    if launch_t is None:
        return
    c, lbl = DERIVED_STYLE["takeoff"]
    fd.events.append((launch_t, "takeoff", c, lbl))
    # Landing = the touchdown decel: the strongest OPPOSITE-direction acc_x jolt
    # after the launch (the airframe braking as it hits, just before it rests).
    # This lives inside the clipped window (unlike the post-landing rest, which the
    # window truncates) and marks touchdown -- so a soft landing lands at the end of
    # the flight, not dropped (rawf2) or stuck on the throw's recoil (rawf3).
    land_t = None
    land_peak = cf.LAND_SPIKE
    for t, ax in zip(fd.acc_t, fd.acc["x"]):
        if t <= launch_t or (ax < 0) != (launch_sign > 0):
            continue            # before launch, or same direction as the throw
        if abs(ax) >= land_peak:
            land_peak = abs(ax)
            land_t = t
    if land_t is not None and land_t > launch_t:
        c, lbl = DERIVED_STYLE["land"]
        fd.events.append((land_t, "land", c, lbl))
    fd.events.sort(key=lambda e: e[0])


def _add_window_markers(fd: FlightData, windows: List[Tuple[float, float]]) -> None:
    """Full-session variant of _add_derived_events: mark takeoff+landing for EVERY
    detected flight window, not just the first. Each detected window is
    (launch - EDGE_PAD_S, touchdown + EDGE_PAD_S), so the launch/touchdown times are
    recovered by peeling that pad back off -- keeping the markers exactly consistent
    with where clip_flights cut each clip. The windows are also stashed on fd so
    build_figure can shade them."""
    fd.clip_windows = list(windows)
    ct, tlbl = DERIVED_STYLE["takeoff"]
    cl, llbl = DERIVED_STYLE["land"]
    for lo, hi in windows:
        fd.events.append((lo + cf.EDGE_PAD_S, "takeoff", ct, tlbl))
        fd.events.append((hi - cf.EDGE_PAD_S, "land", cl, llbl))
    fd.events.sort(key=lambda e: e[0])


def load_flight(prefix: str, name: Optional[str] = None,
                window: Optional[Tuple[float, float]] = None,
                detect_window: bool = False,
                mark_all_windows: bool = False,
                session_prefix: Optional[str] = None) -> FlightData:
    """Load one flight's streams into a FlightData.

    ``prefix`` is the path prefix shared by the stream files (no ``_Stream.csv``).
    If ``detect_window`` is True and no ``window`` is given, the flight window is
    auto-detected from the fused activity signal (for raw, unclipped sessions).
    Already-clipped ``*_flightN_*`` prefixes need neither -- they contain only the
    flight rows -- so pass ``detect_window=False`` (the default).

    ``mark_all_windows`` is for plotting a whole unclipped session: instead of one
    derived takeoff/landing, mark every detected flight and record the windows so
    build_figure can shade where each clip was cut from.

    ``session_prefix`` is the parent session of a clipped set, used only to read
    manual-override spans that the clip itself cannot resolve (see
    _load_override_spans). Harmless to omit; the masking just degrades.
    """
    if window is None and detect_window:
        windows = cf._detect_windows(cf._build_activity(prefix))
        window = windows[0] if windows else None
    fd = FlightData(name=name or os.path.basename(prefix), window=window)
    _load_controller(fd, prefix)
    _load_accel(fd, prefix)
    _load_motor(fd, prefix)
    _load_events(fd, prefix)
    _load_override_spans(fd, session_prefix or prefix)
    _drop_override_setpoints(fd)
    if mark_all_windows:
        _add_window_markers(fd, cf._detect_windows(cf._build_activity(prefix)))
    else:
        _add_derived_events(fd)
    return fd


# --------------------------------------------------------------------------- #
# Plotting
# --------------------------------------------------------------------------- #
def _t0(fd: FlightData) -> float:
    """Earliest sample time, used to zero the x-axis so it reads seconds-into-flight."""
    cands = [s[0] for s in (fd.gyro_t, fd.acc_t, fd.mot_t) if s]
    return min(cands) if cands else 0.0


def _draw_clip_windows(ax, fd: FlightData, t0: float, label: bool) -> None:
    """Shade each detected flight window so a full-session plot shows exactly where
    every clip was cut from (empty for single clipped/raw flights)."""
    for i, (lo, hi) in enumerate(fd.clip_windows, 1):
        ax.axvspan(lo - t0, hi - t0, color="#1f77b4", alpha=0.07, zorder=0)
        if label:
            ax.annotate(f"f{i}", xy=((lo + hi) / 2 - t0, 0.0),
                        xycoords=("data", "axes fraction"),
                        xytext=(0, 2), textcoords="offset points",
                        ha="center", va="bottom", fontsize=7, color="#1f77b4",
                        alpha=0.8)


def _draw_events(ax, fd: FlightData, t0: float, label: bool) -> None:
    for t, kind, colour, lbl in fd.events:
        ax.axvline(t - t0, color=colour, linestyle="--", linewidth=1.0, alpha=0.7)
        if label:
            ax.annotate(lbl, xy=(t - t0, 1.0), xycoords=("data", "axes fraction"),
                        xytext=(0, 2), textcoords="offset points",
                        ha="center", va="bottom", fontsize=7, color=colour,
                        rotation=90)


def build_figure(fd: FlightData, figsize=(11, 8)) -> Figure:
    """Build the 3-row stacked, shared-x figure for one flight."""
    fig = Figure(figsize=figsize, constrained_layout=True)
    ax1, ax2, ax3 = fig.subplots(3, 1, sharex=True)
    t0 = _t0(fd)

    # Row 1: gyro + setpoints
    gcol = {"roll": "#1f77b4", "pitch": "#2ca02c", "yaw": "#d62728"}
    for axis in ("roll", "pitch", "yaw"):
        if fd.gyro_t and fd.gyro.get(axis):
            xs = [t - t0 for t in fd.gyro_t]
            ax1.plot(xs, fd.gyro[axis], color=gcol[axis], linewidth=0.9,
                     label=f"gyro {axis}")
        if fd.set_t and fd.setp.get(axis) and any(fd.setp[axis]):
            xs = [t - t0 for t in fd.set_t]
            ax1.plot(xs, fd.setp[axis], color=gcol[axis], linewidth=0.9,
                     linestyle=":", alpha=0.8, label=f"cmd {axis}")
    ax1.set_ylabel("rate (deg/s)")
    ax1.legend(fontsize=6, ncol=3, loc="upper right")
    ax1.grid(True, alpha=0.3)

    # Row 2: accel
    acol = {"x": "#1f77b4", "y": "#2ca02c", "z": "#d62728"}
    for axis in ("x", "y", "z"):
        if fd.acc_t and fd.acc.get(axis):
            xs = [t - t0 for t in fd.acc_t]
            ax2.plot(xs, fd.acc[axis], color=acol[axis], linewidth=0.9,
                     label=f"acc {axis}")
    ax2.set_ylabel("accel (g)")
    ax2.legend(fontsize=6, ncol=3, loc="upper right")
    ax2.grid(True, alpha=0.3)

    # Row 3: surface deflections + throttle
    scol = {"aileron": "#1f77b4", "elevator": "#2ca02c", "aileron2": "#17becf",
            "rudder": "#9467bd"}
    for surf in ("aileron", "elevator", "aileron2", "rudder"):
        if fd.mot_t and fd.surf.get(surf):
            xs = [t - t0 for t in fd.mot_t]
            ax3.plot(xs, fd.surf[surf], color=scol[surf], linewidth=0.9,
                     label=surf)
    if fd.mot_t and fd.throttle:
        xs = [t - t0 for t in fd.mot_t]
        ax3.plot(xs, fd.throttle, color="#ff7f0e", linewidth=0.9,
                 linestyle="-", alpha=0.8, label="throttle")
    ax3.set_ylabel("deflection (-1..1)")
    ax3.set_ylim(-1.1, 1.1)
    ax3.set_xlabel("time in flight (s)")
    ax3.legend(fontsize=6, ncol=4, loc="upper right")
    ax3.grid(True, alpha=0.3)

    # Shade detected clip windows behind the traces (full-session plot only);
    # label f1, f2, ... once on the bottom row.
    _draw_clip_windows(ax1, fd, t0, label=False)
    _draw_clip_windows(ax2, fd, t0, label=False)
    _draw_clip_windows(ax3, fd, t0, label=True)

    # Event lines across all rows; label only on the top row.
    _draw_events(ax1, fd, t0, label=True)
    _draw_events(ax2, fd, t0, label=False)
    _draw_events(ax3, fd, t0, label=False)

    fig.suptitle(fd.name, fontsize=10)
    return fig


# --------------------------------------------------------------------------- #
# Flight enumeration (raw sessions + clipped sets)
# --------------------------------------------------------------------------- #
@dataclass
class FlightRef:
    name: str                 # display name
    prefix: str               # path prefix for the streams
    source: str               # "clipped", "raw" (a detected window), or "full"
    detect_window: bool       # whether load_flight should auto-detect the window
    window: Optional[Tuple[float, float]] = None  # explicit cf window (raw multi-flight)
    flight_index: Optional[int] = None  # 1-based index of a raw detected window
    session_prefix: Optional[str] = None  # parent session of a clip, when known


def enumerate_flights(directory: str, match: Optional[str] = None,
                      clipped_dir: Optional[str] = None) -> List[FlightRef]:
    """Enumerate plottable flights: every already-clipped ``*_flightN_*`` set, plus
    every raw session that auto-detects at least one flight window. ``match`` is a
    substring filter on the display name."""
    refs: List[FlightRef] = []
    clip_dir = clipped_dir or os.path.join(directory, "clipped")

    # Scope clips to the scanned folder. Clips are grouped into per-day subfolders
    # (clipped/YYYYMMDD, or clipped/Misc_Flights) mirroring the logs folder, so
    # when a specific day folder is scanned only that day's clips should appear --
    # not every day's. When the scanned directory's basename is such a day token,
    # restrict the search to the matching clip subfolder STRICTLY: if that folder
    # has no clips yet, show none (rather than falling back and leaking other days'
    # clips in). Only when scanning something that isn't a day folder (e.g. the
    # logs root) do we search the whole clip tree so all saved clips reappear.
    day_sub = os.path.basename(os.path.normpath(directory))
    is_day_folder = bool(re.fullmatch(r"\d{8}|Misc_Flights", day_sub))
    clip_root = os.path.join(clip_dir, day_sub) if is_day_folder else clip_dir

    # Clipped flights: prefixes ending in _flightN.
    seen = set()
    clipped: List[FlightRef] = []
    for suffix in cf.STREAMS:
        for path in glob.glob(os.path.join(clip_root, "**", f"*_flight*_{suffix}.csv"),
                              recursive=True):
            prefix = path[: -(len(suffix) + 5)]
            if prefix in seen:
                continue
            seen.add(prefix)
            name = os.path.basename(prefix)
            ref = FlightRef(name, prefix, "clipped", detect_window=False)
            clipped.append(ref)
            refs.append(ref)

    # Raw sessions: the full unclipped session plus each auto-detected window, so
    # you can plot both and see whether detection missed or mis-cut anything.
    sessions = cf.find_sessions(directory, match)
    for prefix in sessions:
        windows = cf._detect_windows(cf._build_activity(prefix))
        base = os.path.basename(prefix)
        refs.append(FlightRef(f"{base} [full]", prefix, "full",
                              detect_window=False, window=None))
        for i, win in enumerate(windows, 1):
            refs.append(FlightRef(f"{base} [raw f{i}]", prefix, "raw",
                                  detect_window=False, window=win,
                                  flight_index=i))

    # Point each clip back at the session it was cut from (clip basenames are
    # "<session>_flightN"). Only used to recover manual-override spans, which a
    # clip cannot always resolve on its own -- see _load_override_spans.
    for ref in clipped:
        base = os.path.basename(ref.prefix)
        for sess in sessions:
            if base.startswith(os.path.basename(sess) + "_flight"):
                ref.session_prefix = sess
                break

    if match:
        refs = [r for r in refs if match in r.name]
    return sorted(refs, key=lambda r: r.name)


def load_flight_ref(ref: FlightRef) -> FlightData:
    return load_flight(ref.prefix, name=ref.name, window=ref.window,
                       detect_window=ref.detect_window,
                       mark_all_windows=(ref.source == "full"),
                       session_prefix=ref.session_prefix)


def session_start_time(prefix: str) -> float:
    """Absolute cf_time of a session's earliest sample -- the zero point of the
    plot x-axis. Add a time read off the plot (seconds-into-flight) to this to get
    the absolute cf_time needed to clip a manual window."""
    return _t0(load_flight(prefix, detect_window=False))


def describe_flight(fd: FlightData) -> str:
    """A clip_flights-style one-block textual readout for the console pane."""
    lines = [f"== {fd.name} =="]
    if fd.window:
        lines.append(f"  window: cf {fd.window[0]:.1f}-{fd.window[1]:.1f}s "
                     f"({fd.window[1] - fd.window[0]:.0f}s)")
    lines.append(f"  gyro samples:  {len(fd.gyro_t)}")
    dropped = len(fd.gyro_t) - len(fd.set_t)
    if dropped > 0:
        lines.append(f"  cmd samples:   {len(fd.set_t)} "
                     f"({dropped} dropped: manual override, not pilot commands)")
    lines.append(f"  accel samples: {len(fd.acc_t)}")
    lines.append(f"  motor samples: {len(fd.mot_t)}")
    if fd.events:
        ev = ", ".join(f"{lbl}@{t - _t0(fd):.1f}s" for t, k, c, lbl in fd.events)
        lines.append(f"  events: {ev}")
    else:
        lines.append("  events: none")
    return "\n".join(lines)


def main() -> None:
    ap = argparse.ArgumentParser(description="Plot glider flights.")
    ap.add_argument("directory", nargs="?", default=".", help="log folder (default: .)")
    ap.add_argument("--match", default=None, help="filter flights by name substring")
    ap.add_argument("--save", default=None, help="save PNGs to this folder instead of showing")
    args = ap.parse_args()

    refs = enumerate_flights(args.directory, args.match)
    print(f"Found {len(refs)} plottable flight(s)")
    if not refs:
        return
    if args.save:
        os.makedirs(args.save, exist_ok=True)
    for ref in refs:
        fd = load_flight_ref(ref)
        print(describe_flight(fd))
        if args.save:
            fig = build_figure(fd)
            out = os.path.join(args.save, f"{ref.name.replace(' ', '_').replace('[', '').replace(']', '')}.png")
            fig.savefig(out, dpi=110)
            print(f"  saved {out}")


if __name__ == "__main__":
    main()

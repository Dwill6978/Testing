"""
gui_GliderControlTest.py

GUI front-end for the Crazyflie Bolt glider control + telemetry logger.
This is a re-packaging of GliderControlTest.py: every CLI prompt / flag is now a
widget, the live matplotlib plots are embedded in a tab, and a "Connect" button
drives the whole session. A Manual Override tab lets you take over the servos /
motors directly through the Crazyflie parameter system (motorPowerSet.* and
servo.servoAngle), optionally driven by a game controller.

Architecture
------------
* GUI thread        : owns all widgets + the matplotlib canvas. A QTimer redraws
                      the plots from thread-safe buffers.
* Worker thread     : owns the SyncCrazyflie connection and the tight control
                      loop (the old _main_control_loop) + pygame polling.
* cflib callbacks   : fire on cflib's threads -> push points into PlotBuffers
                      (lock guarded). The GUI timer drains them.
* GUI -> worker     : a thread-safe command queue (one-shot actions) plus a
                      lock-guarded shared-state object (continuous values).

Dependencies: PySide6 (or PyQt5), matplotlib, pygame, cflib.
    pip install pyside6 matplotlib pygame cflib
"""

import csv
import glob
import json
import math
import os
import queue
import re
import shutil
import struct
import subprocess
import sys
import threading
import time
from collections import deque
from dataclasses import asdict, dataclass, field
from datetime import datetime
from typing import Dict, List, Optional, Tuple

# --------------------------------------------------------------------------- #
# Qt binding compatibility (PySide6 preferred, PyQt5 fallback)
# --------------------------------------------------------------------------- #
try:
    from PySide6 import QtCore, QtGui, QtWidgets
    from PySide6.QtCore import Qt, QTimer, Signal

    QT_BINDING = "PySide6"
except ImportError:  # pragma: no cover - fallback path
    from PyQt5 import QtCore, QtGui, QtWidgets
    from PyQt5.QtCore import Qt, QTimer
    from PyQt5.QtCore import pyqtSignal as Signal

    QT_BINDING = "PyQt5"

import matplotlib

matplotlib.use("QtAgg")
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qtagg import NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure

import flight_plots

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crtp.crtpstack import CRTPPacket, CRTPPort
from cflib.utils import uri_helper

try:
    import pygame
except ImportError:  # controller support optional
    pygame = None


# --------------------------------------------------------------------------- #
# Constants (carried over from GliderControlTest.py)
# --------------------------------------------------------------------------- #
DEFAULT_URI = uri_helper.uri_from_env(default="radio://0/80/2M/E7E7E7E701")
MAX_MOTOR_CMD = 65535
JOYSTICK_DEADBAND = 0.05
# Root-owned copy of reset_controller.sh, run unattended via a NOPASSWD sudoers
# entry to USB re-enumerate the InterLink-X at connect (Parallels passthrough
# wedges it after the first run). Quietly no-ops if not installed / not granted.
RESET_CMD = "/usr/local/sbin/reset_controller.sh"
THROTTLE_STEP = 0.01
# Normalized throttle below this reads as a true 0 (motor off). Covers the RC
# throttle stick's idle jitter so "throttle is zero" is actually reachable.
THROTTLE_INPUT_DEADBAND = 0.03
# Safety interlock: the motor will not arm unless the thrust input is at/below
# this idle level, so arming can never coincide with a spun-up throttle.
ARM_THROTTLE_DEADBAND = 0.05
# Default (unchecked) state of the Setup-tab "Log controller input" checkbox.
# When enabled, the worker echoes live controller axis/command reads to the
# console once per second while a controller-driven mode is active -- a
# diagnostic to confirm the stick is read and which mode is in effect.
CONTROLLER_DEBUG_LOG = False
# Max servo units moved per control-loop tick (~100 Hz) in manual override.
# Non-blocking slew so the loop never stalls; ~3000 units/tick ~= full travel
# in ~0.2 s, smooth without flooding the radio link.
OVERRIDE_SERVO_SLEW = 3000
# Same idea for the propulsion throttle in NORMAL (non-override) flight, which
# also reaches the ESC through the servo.servoAngle param. This used to ramp in a
# BLOCKING inner loop (step 1000 every 5 ms sleep), which both stalled the ~100 Hz
# control loop for ~100 ms per throttle move and queued ~21 acked param writes per
# move -- the cause of motor commands running seconds behind the stick and
# continuing to arrive after a disarm.
#
# Expressed as a RATE rather than a per-tick step so the ramp feels identical no
# matter how the loop rate and the write-coalescing interval interact (a per-tick
# step silently re-scales when either changes). 200 units/ms is exactly the old
# loop's rate: 1000 units per 5 ms sleep, i.e. full 0..65535 travel in ~0.33 s.
THROTTLE_SERVO_SLEW_PER_S = 200_000.0
# Manual override drives the motors/servo through the parameter system, which is
# request/response (each write is acked one at a time) rather than the streaming
# commander channel used by setpoint flight. Pushing all four surfaces + throttle
# every ~10 ms tick overruns that channel, so the writes queue up and the servos
# lag the stick. Coalesce override writes to this interval so the link always
# carries the freshest command instead of a growing backlog. ~60 Hz stays
# responsive for hand flying while fitting inside the param channel's throughput.
OVERRIDE_WRITE_INTERVAL = 1.0 / 60.0
# Commander-fast manual override: raw M1-M4 + propulsion (servo) values streamed
# on the generic setpoint channel (CRTPPort.COMMANDER_GENERIC, channel 0) as a
# type-11 "manualMotor" packet. This rides the same fire-and-forget path as
# normal setpoint flight, so it stays responsive instead of backing up behind the
# request/response param channel that motorPowerSet.* uses. Requires the modded
# firmware (manualMotorType decoder + stabilizer apply). See crtp_commander_generic.c.
MANUAL_MOTOR_SETPOINT_TYPE = 11
GENERIC_SETPOINT_CHANNEL = 0
# Streamed propulsion throttle for NORMAL (non-override) flight. Rides channel 1
# of the same port -- the "meta command" channel -- NOT channel 0 next to the
# manualMotor packet. Channel 0 is the setpoint channel: the firmware memsets the
# setpoint_t and pushes it to the commander for every packet that arrives there,
# so a throttle packet on channel 0 would overwrite the rate setpoint with zeros.
# Interleaved with this GUI's own ~100 Hz rate stream, every other command the
# controller saw would be a zero. Channel 1 does not touch the setpoint at all,
# which is correct: on a fixed wing the ESC is a separate actuator from the
# surfaces, not a component of the attitude command. See propulsionSetDecoder in
# crtp_commander.c. Requires the modded firmware.
META_COMMAND_CHANNEL = 1
META_PROPULSION_TYPE = 1
# Default the propulsion throttle to the streamed path. The servo.servoAngle
# param path is kept switchable (Control tab) so the two can be compared on one
# flight by logging servo.angle against servo_cmd; it is the fallback for stock
# firmware, which has no propulsion meta-command decoder.
PROPULSION_FAST_DEFAULT = True
# Neutral center for the servo trims (matches the firmware defaults).
SERVO_TRIM_CENTER = 32767
# Per-surface trim axis -> firmware param name (stabilizer.trim{Roll,Pitch,Yaw}).
TRIM_PARAMS = {
    "roll": "stabilizer.trimRoll",
    "pitch": "stabilizer.trimPitch",
    "yaw": "stabilizer.trimYaw",
}
# Fixed-wing servo mixer map (firmware fwSurfMap.*). Each motor channel M1-M4
# is assigned a control surface, with an optional per-channel command invert.
SURFACE_MAP_CHANNELS = ("m1", "m2", "m3", "m4")
SURFACE_MAP_SURF_PARAM = {ch: f"fwSurfMap.{ch}Surf" for ch in SURFACE_MAP_CHANNELS}
SURFACE_MAP_INV_PARAM = {ch: f"fwSurfMap.{ch}Inv" for ch in SURFACE_MAP_CHANNELS}
# Surface codes shared with firmware channelServoPwm(): (code, label).
SURFACE_OPTIONS = (
    (0, "Unused"),
    (1, "Aileron (roll)"),
    (2, "Elevator (pitch)"),
    (3, "Rudder (yaw)"),
)
# Per-channel (surface_code, invert) defaults mirroring the firmware fwSurfMap
# defaults, used before a deck read-back populates the real values.
SURFACE_MAP_DEFAULTS = {"m1": (1, 0), "m2": (2, 0), "m3": (3, 1), "m4": (1, 1)}
# Short label + trace colour per surface code for the live motor plot.
SURFACE_PLOT_LABEL = {1: "Aileron", 2: "Elevator", 3: "Rudder"}
SURFACE_PLOT_COLOR = {1: "green", 2: "blue", 3: "red"}
# In-flight tuning ranges for the rear knobs (absolute, clamped). Trim spans the
# full UINT16 servo range; the PID ranges match the rate-loop gain envelopes the
# user tunes within. Kp is spaced across 100..800, Ki 0..40, Kd 0..10.
TRIM_KNOB_RANGE = (0.0, 65535.0)
PID_KP_RANGE = (100.0, 800.0)
PID_KI_RANGE = (0.0, 40.0)
PID_KD_RANGE = (0.0, 10.0)
# A rear knob only "catches" (starts driving the value) once its mapped position
# passes within this fraction of the currently stored value, so entering a mode
# never snaps the surface/gain to wherever the knob happens to be sitting.
KNOB_CATCH_FRACTION = 0.02
# GUI spin-box column indices for PID terms (see MainWindow.pid_spins keys).
PID_TERM_COL = {"kp": 1, "ki": 2, "kd": 3}
# Persistent notes: loaded into the Notes tab on launch, written back on close
# (and on demand via the Save button). Kept next to this script.
NOTES_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "glider_notes.txt")
# Persistent PID rate gains: written by the Control tab's "Save as launch
# defaults" button and reloaded into the spin boxes on every launch, so a tuned
# set carries over between sessions (and is pushed to the deck on Connect).
PID_DEFAULTS_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "glider_pid_defaults.json")
# Persistent Setup-tab settings (log rates/enables, LPF, rate limits, URI, ...).
# Written by "Save as launch defaults" on that tab, reloaded into the widgets at
# startup. Keyed by SessionConfig field name -- see MainWindow._setup_bindings.
SETUP_DEFAULTS_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "glider_setup_defaults.json")
# Persistent user button maps: named per-controller layouts edited on the Mapping
# tab (which command sits on which button index, plus a human label for each
# physical button). Several maps can be stored; one per controller type is marked
# active and applied on Connect. Absent/corrupt file -> the built-in profile.
CONTROLLER_MAPS_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "glider_controller_maps.json")
# Persistent maneuver library: the test card edited on the Maneuvers tab. Each
# entry is one injectable excitation (axis + shape + amplitude + timing).
MANEUVER_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "glider_maneuvers.json")
# Root folder that holds all flight logs. Each session's CSV/Console files are
# written into a per-day subfolder (YYYYMMDD) so logs stay grouped by flight day.
LOGS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
# Root folder for clipped (flight-only) copies. Clips are written into per-day
# subfolders (YYYYMMDD) mirroring LOGS_DIR, or Misc_Flights for undated sessions.
CLIPPED_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "clipped")
# OneDrive mirror target. The Mac's OneDrive is exposed inside this Parallels VM as
# a normal writable folder under the Home share; anything written here is synced to
# OneDrive by the Mac, so it reaches the macOS MATLAB analysis pipeline without a
# manual web upload. Each per-day subfolder of CLIPPED_DIR is mirrored to a matching
# subfolder here. (The sibling "OneDrive - The Ohio State University" path is a
# symlink into /Users, which doesn't resolve inside the VM -- use CloudStorage.)
ONEDRIVE_FLIGHTDATA_DIR = (
    "/media/psf/Home/Library/CloudStorage/"
    "OneDrive-TheOhioStateUniversity/Glider/Data/FlightData")
CSV_SCHEMA_VERSION = "glider_csv_v1"


def resolve_log_prefix(prefix: str) -> str:
    """Return a full path prefix for a session's log files, routed into today's
    per-day folder under LOGS_DIR.

    Searches LOGS_DIR for a folder named after the current day (YYYYMMDD); if one
    exists the logs go there, otherwise it is created. Only the bare filename part
    of ``prefix`` is used, so callers can pass a plain name and it always lands in
    the right day folder.
    """
    day_dir = os.path.join(LOGS_DIR, datetime.now().strftime("%Y%m%d"))
    os.makedirs(day_dir, exist_ok=True)
    return os.path.join(day_dir, os.path.basename(prefix))


def clipped_day_dir(session_name: str) -> str:
    """Return (creating if needed) the clipped subfolder for a session, using the
    same YYYYMMDD grouping as LOGS_DIR. The day is taken from a
    ``flight_YYYYMMDD_...`` session name; sessions without that stamp go to
    Misc_Flights, matching how the logs folder is organized."""
    m = re.match(r"flight_(\d{8})_", session_name)
    sub = m.group(1) if m else "Misc_Flights"
    day_dir = os.path.join(CLIPPED_DIR, sub)
    os.makedirs(day_dir, exist_ok=True)
    return day_dir


# Disarm if the Connection log block goes silent this long. MUST stay several times
# the Connection log period, or a single dropped packet trips a spurious failsafe:
# this was 1.0 s while the saved period was 1000 ms, i.e. ZERO margin, and 54 of 204
# recorded sessions logged a CONNECTION_TELEMETRY_TIMEOUT disarm -- many of which
# were almost certainly this, not real link loss. At 3.0 s against a 500 ms period
# the watchdog needs 6 consecutive misses to fire, so it still catches a genuinely
# dead link but tolerates ordinary radio jitter.
CONNECTION_WATCHDOG_TIMEOUT_S = 3.0
DEFAULT_PLOT_WINDOW_S = 20.0  # how much history each live plot shows

# Re-create the log blocks if NO log packet of any kind has arrived for this long.
#
# The deck tears its own logging down behind our back. In the firmware's log task
# (modules/src/log.c) every block tick checks crtpIsConnected() and, if it is
# false, calls logReset() -- which stops AND DELETES every log block and frees
# all log ops. For the radio that check is just
#   (now - lastPacketTick) < RADIO_ACTIVITY_TIMEOUT_MS
# and the timeout is 1000 ms (hal/src/radiolink.c). So ONE second in which the
# deck receives nothing destroys all logging, and the host is never told: cflib's
# LogConfig objects still report added == started == True, so nothing retries.
# Setpoints keep working because the commander port does not care about log
# state, which is why the symptom is "live plots frozen forever but the aircraft
# still flies" and why it takes a reconnect to clear.
#
# Must stay well above the fastest block period (10 ms here) but below
# CONNECTION_WATCHDOG_TIMEOUT_S, so logging is back before the telemetry
# watchdog mistakes a dead log block for a dead link and disarms.
LOG_STALL_TIMEOUT_S = 2.0
# Minimum spacing between recovery attempts, so a genuinely dead link produces a
# slow retry rather than a CREATE_BLOCK burst every control-loop tick.
LOG_RECOVERY_COOLDOWN_S = 2.0


# --------------------------------------------------------------------------- #
# PID / state dataclasses (carried over)
# --------------------------------------------------------------------------- #
@dataclass
class PidAxisGains:
    kp: float
    ki: float
    kd: float
    kff: float = 0.0


@dataclass
class PidGains:
    pitch: PidAxisGains = field(default_factory=lambda: PidAxisGains(kp=300.0, ki=12.0, kd=0.0, kff=0.0))
    yaw: PidAxisGains = field(default_factory=lambda: PidAxisGains(kp=300.0, ki=12.0, kd=0.0, kff=0.0))
    roll: PidAxisGains = field(default_factory=lambda: PidAxisGains(kp=300.0, ki=12.0, kd=0.0, kff=0.0))


PID_AXES = ("pitch", "yaw", "roll")
PID_TERMS = ("kp", "ki", "kd", "kff")
# How long to wait for the deck to echo the twelve pid_rate writes before giving
# up and logging the readback as incomplete. Generous on purpose: cflib acks
# param writes one at a time, so twelve queued writes behind a busy or lossy link
# can legitimately take a while, and a false "no readback" warning is the exact
# noise this verification was added to remove.
PID_VERIFY_TIMEOUT_S = 3.0


def load_default_gains() -> PidGains:
    """Launch-time gains: the last set saved from the Control tab, falling back
    to the PidGains defaults for anything missing or unreadable."""
    gains = PidGains()
    try:
        with open(PID_DEFAULTS_FILE, "r", encoding="utf-8") as fh:
            saved = json.load(fh)
    except FileNotFoundError:
        return gains
    except (OSError, ValueError):
        return gains
    for axis in PID_AXES:
        stored = saved.get(axis)
        if not isinstance(stored, dict):
            continue
        g = getattr(gains, axis)
        for term in PID_TERMS:
            if term in stored:
                try:
                    setattr(g, term, float(stored[term]))
                except (TypeError, ValueError):
                    pass
    return gains


def save_default_gains(gains: PidGains) -> None:
    """Persist these gains as the launch defaults (raises OSError on failure)."""
    payload = {axis: asdict(getattr(gains, axis)) for axis in PID_AXES}
    with open(PID_DEFAULTS_FILE, "w", encoding="utf-8") as fh:
        json.dump(payload, fh, indent=2)


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


# --------------------------------------------------------------------------- #
# CSV logging (carried over verbatim)
# --------------------------------------------------------------------------- #
class CsvLogBundle:
    def __init__(self, prefix: str):
        self.controller_file = open(f"{prefix}_Controller.csv", "w", newline="")
        self.motor_file = open(f"{prefix}_Motor.csv", "w", newline="")
        self.connection_file = open(f"{prefix}_Connection.csv", "w", newline="")
        self.accelerometer_file = open(f"{prefix}_Accelerometer.csv", "w", newline="")
        self.event_file = open(f"{prefix}_Events.csv", "w", newline="")
        # Plain-text mirror of everything shown in the Console tab (Crazyflie
        # firmware console + this app's own status lines), for post-flight review.
        self.console_file = open(f"{prefix}_Console.txt", "w")

        self.controller = csv.writer(self.controller_file)
        self.motor = csv.writer(self.motor_file)
        self.connection = csv.writer(self.connection_file)
        self.accelerometer = csv.writer(self.accelerometer_file)
        self.event = csv.writer(self.event_file)

        # Console text arrives in arbitrary chunks from two threads (the cflib
        # console callback and the worker's own _log). Buffer partial lines and
        # timestamp each completed line under a lock so writes never interleave.
        self._console_lock = threading.Lock()
        self._console_buf = ""

        self.write_headers()

    def write_headers(self) -> None:
        self.controller.writerow(["# schema_version", CSV_SCHEMA_VERSION, "dataset", "controller"])
        # inj_* is the maneuver-injector contribution to the commanded rate, in
        # the same body axes and deg/s as set_*, APPENDED at the end so readers
        # that index set_yaw at column 6 (loadClip.m, flight_plots) keep working
        # on both old and new logs -- the same convention as motor_m3 /
        # servo_angle / the EKF attitude columns.
        #
        # These three columns are the whole point of logging the injector at all:
        # set_* already carries the TOTAL commanded rate (it comes off the deck
        # as controller.*Rate), so without inj_* there is no way to tell the
        # pilot's command apart from the injected perturbation. With it,
        # pilot = set_* - inj_*, and the excitation is known exactly, per sample.
        # Zero on every row when nothing is being injected.
        self.controller.writerow(["cf_time_s", "gyro_roll", "gyro_pitch", "gyro_yaw",
                                  "set_roll", "set_pitch", "set_yaw",
                                  "inj_roll", "inj_pitch", "inj_yaw"])

        self.motor.writerow(["# schema_version", CSV_SCHEMA_VERSION, "dataset", "motor"])
        # motor_m3 (the rudder surface) is appended last so older readers that index
        # servo_cmd at column 4 keep working; new readers pick up the rudder at 5.
        # servo_angle is appended after it for the same reason. The two servo
        # columns are NOT redundant: servo_cmd is this process's last_servo_value
        # (what the host intended) while servo_angle is logged off the deck (what
        # the deck actually applied). Their difference is the host->deck latency,
        # which is invisible if you only have the first. servo_angle is nan on
        # firmware built before the servo LOG_GROUP existed.
        self.motor.writerow(["cf_time_s", "motor_m4", "motor_m1", "motor_m2", "servo_cmd",
                             "motor_m3", "servo_angle"])

        self.connection.writerow(["# schema_version", CSV_SCHEMA_VERSION, "dataset", "connection"])
        self.connection.writerow(["cf_time_s", "rssi", "vbat"])

        self.accelerometer.writerow(["# schema_version", CSV_SCHEMA_VERSION, "dataset", "accelerometer"])
        # EKF attitude (deg) is APPENDED after acc_z so existing readers that index
        # acc_x at column 1 and acc_z at column 3 -- clip_flights and flight_plots --
        # keep working unchanged on both old and new logs. Same convention as the
        # motor row above.
        self.accelerometer.writerow(["cf_time_s", "acc_x", "acc_y", "acc_z",
                                     "est_roll", "est_pitch", "est_yaw"])

        self.event.writerow(["# schema_version", CSV_SCHEMA_VERSION, "dataset", "events"])
        self.event.writerow(["host_time_iso", "event", "value_1", "value_2", "value_3"])

    def write_breakpoint(self, label: str) -> None:
        host_time = datetime.now().isoformat(timespec="seconds")
        marker = ["# BREAKPOINT", host_time, label]
        self.controller.writerow(marker)
        self.motor.writerow(marker)
        self.connection.writerow(marker)
        self.accelerometer.writerow(marker)
        self.event.writerow([host_time, "BREAKPOINT", label, "", ""])
        self.event_file.flush()

    def write_event(self, event_name: str, v1: object = "", v2: object = "", v3: object = "") -> None:
        host_time = datetime.now().isoformat(timespec="seconds")
        self.event.writerow([host_time, event_name, v1, v2, v3])
        # Flush every event. Events are tiny and rare -- a whole session produces
        # well under the 8 KB stdio buffer -- so without this an Events.csv stays
        # EMPTY ON DISK until close(), and any session that does not exit cleanly
        # loses its entire annotation layer. Session 20260929_093733 did exactly
        # that: 2.8 MB of Controller.csv (big enough to auto-flush) next to a
        # 0-byte Events.csv, which left 3 clipped flights permanently unusable
        # for tuning analysis because their gains are unrecoverable. The
        # telemetry CSVs are self-flushing by volume; this file is not.
        self.event_file.flush()

    def write_console(self, text: str) -> None:
        """Append console text, timestamping each completed line."""
        with self._console_lock:
            self._console_buf += text
            while "\n" in self._console_buf:
                line, self._console_buf = self._console_buf.split("\n", 1)
                stamp = datetime.now().isoformat(timespec="milliseconds")
                self.console_file.write(f"{stamp}  {line}\n")
            self.console_file.flush()

    def close(self) -> None:
        with self._console_lock:
            if self._console_buf:  # flush any trailing partial line
                stamp = datetime.now().isoformat(timespec="milliseconds")
                self.console_file.write(f"{stamp}  {self._console_buf}\n")
                self._console_buf = ""
        self.controller_file.close()
        self.motor_file.close()
        self.connection_file.close()
        self.accelerometer_file.close()
        self.event_file.close()
        self.console_file.close()


# --------------------------------------------------------------------------- #
# Thread-safe telemetry buffers for the embedded plots
# --------------------------------------------------------------------------- #
class PlotBuffers:
    """Producer = cflib callbacks (worker threads). Consumer = GUI redraw timer.

    Each stream's deque is sized so it holds ``window_s`` seconds of history at
    that stream's log period, so every plot shows the same time span regardless
    of its sample rate. Call ``configure()`` before a session starts to match the
    periods the user chose in the Setup tab.
    """

    def __init__(self, window_s: float = DEFAULT_PLOT_WINDOW_S):
        self._lock = threading.Lock()
        self._window_s = window_s
        self._periods_ms = {"controller": 50, "motor": 50, "connection": 50, "accelerometer": 50}
        self._build()

    def _maxlen(self, stream: str) -> int:
        period_s = self._periods_ms.get(stream, 10) / 1000.0
        return max(50, int(self._window_s / period_s))

    def _build(self) -> None:
        d = lambda stream: deque(maxlen=self._maxlen(stream))
        self.t_ctrl, self.gyroroll, self.gyropitch, self.gyroyaw = (d("controller") for _ in range(4))
        self.setroll, self.setpitch, self.setyaw = (d("controller") for _ in range(3))
        self.t_motor, self.motor1, self.motor2, self.motor3, self.motor4, self.thrust = (d("motor") for _ in range(6))
        self.t_conn, self.rssi = d("connection"), d("connection")
        self.t_acc, self.accx, self.accy, self.accz = (d("accelerometer") for _ in range(4))
        # EKF attitude angles (deg). Same block/period as the accelerometer, so the
        # same maxlen keeps them index-aligned with t_acc.
        self.estroll, self.estpitch, self.estyaw = (d("accelerometer") for _ in range(3))

    def configure(self, periods_ms: Dict[str, int], window_s: Optional[float] = None) -> None:
        """Resize the buffers for new log periods. Call before telemetry starts."""
        with self._lock:
            self._periods_ms.update(periods_ms)
            if window_s is not None:
                self._window_s = window_s
            self._build()

    def add_controller(self, ts, gr, gp, gy, sr, sp, sy):
        with self._lock:
            self.t_ctrl.append(ts)
            self.gyroroll.append(gr); self.gyropitch.append(gp); self.gyroyaw.append(gy)
            self.setroll.append(sr); self.setpitch.append(sp); self.setyaw.append(sy)

    def add_motor(self, ts, m1, m2, m3, m4, thrust):
        with self._lock:
            self.t_motor.append(ts)
            self.motor1.append(m1); self.motor2.append(m2)
            self.motor3.append(m3); self.motor4.append(m4); self.thrust.append(thrust)

    def add_connection(self, ts, rssi):
        with self._lock:
            self.t_conn.append(ts); self.rssi.append(rssi)

    def add_accel(self, ts, ax, ay, az, er=None, ep=None, ey=None):
        # EKF attitude rides the same log block as acceleration (one CRTP packet,
        # same timestamp), so it shares t_acc rather than carrying its own clock.
        # The est_* args default to None so a caller that predates the attitude
        # subscription still works.
        with self._lock:
            self.t_acc.append(ts)
            self.accx.append(ax); self.accy.append(ay); self.accz.append(az)
            self.estroll.append(er); self.estpitch.append(ep); self.estyaw.append(ey)

    def snapshot(self) -> Dict[str, list]:
        with self._lock:
            return {name: list(getattr(self, name)) for name in (
                "t_ctrl", "gyroroll", "gyropitch", "gyroyaw", "setroll", "setpitch", "setyaw",
                "t_motor", "motor1", "motor2", "motor3", "motor4", "thrust",
                "t_conn", "rssi",
                "t_acc", "accx", "accy", "accz", "estroll", "estpitch", "estyaw",
            )}


# --------------------------------------------------------------------------- #
# Embedded matplotlib canvas (the original 2x3 grid)
# --------------------------------------------------------------------------- #
class PlotCanvas(FigureCanvas):
    def __init__(self, buffers: PlotBuffers):
        self.buffers = buffers
        self.fig = Figure(figsize=(11, 6))
        super().__init__(self.fig)

        (self.ax, self.ax2, self.ax3), (self.ax4, self.ax5, self.ax6) = self.fig.subplots(2, 3)

        (self.line_gyroroll,) = self.ax.plot([], [], label="Roll Rate", color="blue")
        (self.line_setroll,) = self.ax.plot([], [], label="Roll Setpoint", color="red")
        self.ax.set_ylim(-38, 38); self.ax.set_title("Roll")
        self.ax.set_xlabel("Time (s)"); self.ax.set_ylabel("deg/s")

        (self.line_gyropitch,) = self.ax2.plot([], [], label="Pitch Rate", color="blue")
        (self.line_setpitch,) = self.ax2.plot([], [], label="Pitch Setpoint", color="red")
        self.ax2.set_ylim(-38, 38); self.ax2.set_title("Pitch")
        self.ax2.set_xlabel("Time (s)"); self.ax2.set_ylabel("deg/s")

        (self.line_gyroyaw,) = self.ax3.plot([], [], label="Yaw Rate", color="blue")
        (self.line_setyaw,) = self.ax3.plot([], [], label="Yaw Setpoint", color="red")
        self.ax3.set_ylim(-38, 38); self.ax3.set_title("Yaw")
        self.ax3.set_xlabel("Time (s)"); self.ax3.set_ylabel("deg/s")

        # EKF attitude ANGLE on a twin y-axis of the matching rate plot. The 2x3
        # grid was already full, so rather than shrink all six plots to make a
        # seventh cell, each angle goes on the subplot that already shows that
        # axis' rate -- they are the integral/derivative of each other, so reading
        # them together is what you actually want in flight. Separate y-axis
        # because the units differ (deg/s vs deg) and the ranges differ by ~5x;
        # sharing one axis would squash the rate traces flat.
        self.attitude_axes = {}
        self.attitude_lines = {}
        for key, base in (("roll", self.ax), ("pitch", self.ax2), ("yaw", self.ax3)):
            twin = base.twinx()
            (line,) = twin.plot([], [], label=f"{key.capitalize()} Angle",
                                color="darkgreen", linestyle="--", linewidth=1.0)
            twin.set_ylabel("deg", color="darkgreen")
            twin.tick_params(axis="y", labelcolor="darkgreen", labelsize=8)
            self.attitude_axes[key] = twin
            self.attitude_lines[key] = line
            # One legend per subplot covering BOTH axes: twinx draws a second,
            # overlapping legend otherwise, and the angle trace would be missing
            # from the rate axis' legend entirely.
            handles = base.get_lines() + [line]
            base.legend(handles=handles, labels=[h.get_label() for h in handles],
                        fontsize=6, loc="upper right")

        # One trace per motor channel; its label/colour/visibility follow the
        # configured servo->surface map (set via set_surface_map()).
        self.motor_lines = {}
        for ch in ("m1", "m2", "m3", "m4"):
            (line,) = self.ax4.plot([], [], color="gray")
            self.motor_lines[ch] = line
        (self.line_thrust,) = self.ax4.plot([], [], label="Thrust", color="orange")
        self.ax4.set_ylim(-1.1, 1.1); self.ax4.set_title("Motor Commands")
        self.ax4.set_xlabel("Time (s)"); self.ax4.set_ylabel("deflection / thrust")
        self.surface_map = {ch: surf for ch, (surf, _inv) in SURFACE_MAP_DEFAULTS.items()}
        self._apply_surface_map_labels()

        (self.line_rssi,) = self.ax5.plot([], [], label="RSSI", color="red")
        self.ax5.set_title("RSSI"); self.ax5.set_xlabel("Time (s)")
        self.ax5.set_ylabel("value"); self.ax5.set_ylim(0, 60); self.ax5.legend()

        (self.line_accx,) = self.ax6.plot([], [], label="Acc X", color="red")
        (self.line_accy,) = self.ax6.plot([], [], label="Acc Y", color="blue")
        (self.line_accz,) = self.ax6.plot([], [], label="Acc Z", color="green")
        self.ax6.set_title("Accelerometer"); self.ax6.set_xlabel("Time (s)")
        self.ax6.set_ylabel("g"); self.ax6.set_ylim(-5.0, 5.0); self.ax6.legend()

        self.fig.tight_layout()

    def refresh(self) -> None:
        s = self.buffers.snapshot()
        self.line_gyroroll.set_data(s["t_ctrl"], s["gyroroll"])
        self.line_gyropitch.set_data(s["t_ctrl"], s["gyropitch"])
        self.line_gyroyaw.set_data(s["t_ctrl"], s["gyroyaw"])
        self.line_setroll.set_data(s["t_ctrl"], s["setroll"])
        self.line_setpitch.set_data(s["t_ctrl"], s["setpitch"])
        self.line_setyaw.set_data(s["t_ctrl"], s["setyaw"])

        tm = s["t_motor"]
        defl = lambda v: clamp((v - SERVO_TRIM_CENTER) / SERVO_TRIM_CENTER, -1.0, 1.0)
        for ch, line in self.motor_lines.items():
            if self.surface_map.get(ch, 0) in SURFACE_PLOT_LABEL:
                line.set_data(tm, [defl(v) for v in s[f"motor{ch[1]}"]])
            else:
                line.set_data([], [])  # unused channel: keep off the plot
        self.line_thrust.set_data(tm, [clamp(v / MAX_MOTOR_CMD, 0.0, 1.0) for v in s["thrust"]])

        self.line_rssi.set_data(s["t_conn"], s["rssi"])

        self.line_accx.set_data(s["t_acc"], s["accx"])
        self.line_accy.set_data(s["t_acc"], s["accy"])
        self.line_accz.set_data(s["t_acc"], s["accz"])

        # EKF attitude. Samples logged before the attitude subscription existed are
        # None, so drop those pairs rather than handing None to matplotlib (which
        # would raise on autoscale). Filtering here also means a mid-session
        # resubscribe can't misalign the angle traces from their timestamps.
        for key, line in self.attitude_lines.items():
            pairs = [(t, v) for t, v in zip(s["t_acc"], s[f"est{key}"]) if v is not None]
            line.set_data([t for t, _ in pairs], [v for _, v in pairs])

        for twin in self.attitude_axes.values():
            twin.relim()
            twin.autoscale(enable=True, axis="both", tight=False)

        for axis in (self.ax, self.ax2, self.ax3, self.ax5, self.ax6):
            axis.relim()
            # Autoscale x (fill the full width) AND y (re-enabled even though
            # __init__ called set_ylim, which had turned y autoscaling off).
            axis.autoscale(enable=True, axis="both", tight=False)
        # Motor plot: autoscale x only, keep the deflection/thrust y-axis fixed.
        self.ax4.relim()
        self.ax4.autoscale(enable=True, axis="x", tight=False)
        self.ax4.set_ylim(-1.1, 1.1)
        self.draw_idle()

    def set_surface_map(self, mapping: Dict[str, int]) -> None:
        """Update which surface each motor channel represents and relabel the
        motor plot accordingly."""
        self.surface_map = dict(mapping)
        self._apply_surface_map_labels()

    def _apply_surface_map_labels(self) -> None:
        """Colour/label/show each motor trace by its assigned surface; hide
        unused channels. Rebuilds the motor-plot legend."""
        for ch, line in self.motor_lines.items():
            surf = self.surface_map.get(ch, 0)
            if surf in SURFACE_PLOT_LABEL:
                line.set_visible(True)
                line.set_color(SURFACE_PLOT_COLOR[surf])
                line.set_label(f"{ch.upper()} \u00b7 {SURFACE_PLOT_LABEL[surf]}")
            else:
                line.set_visible(False)
                line.set_label(f"_{ch.upper()} (unused)")
        self.ax4.legend()
        self.draw_idle()


# --------------------------------------------------------------------------- #
# Shared state between GUI and worker
# --------------------------------------------------------------------------- #
@dataclass
class SessionConfig:
    """Captured once from the Setup tab when the user presses Connect."""
    uri: str = DEFAULT_URI
    filename_prefix: str = ""
    use_controller: bool = False
    controller_type: str = "xbox"      # key into CONTROLLER_PROFILES
    debug_controller_log: bool = CONTROLLER_DEBUG_LOG
    fwactlpf_enable: bool = True
    fwactlpf_cutoff_hz: float = 8.0
    roll_rate_limit: float = 90.0
    pitch_rate_limit: float = 90.0
    yaw_rate_limit: float = 90.0
    log_controller: bool = True
    log_motor: bool = True
    log_connection: bool = True
    log_accelerometer: bool = True
    period_controller_ms: int = 50
    period_motor_ms: int = 50
    period_connection_ms: int = 50
    # 20 ms = 50 Hz. This block carries acc + EKF attitude (6 vars, one packet).
    period_accelerometer_ms: int = 20
    plot_window_s: float = DEFAULT_PLOT_WINDOW_S
    gains: PidGains = field(default_factory=PidGains)


@dataclass
class LiveControl:
    """Continuous values the GUI mutates while connected (lock-guarded)."""
    trimmed: bool = False
    motor_armed: bool = False
    autonomous: bool = False
    throttle: float = 0.0
    setpoint_roll: float = 0.0
    setpoint_pitch: float = 0.0
    setpoint_yaw: float = 0.0
    # Manual / direct override of the parameter system
    manual_override: bool = False
    override_with_controller: bool = False
    override_m1: int = 0
    override_m2: int = 0
    override_m3: int = 0
    override_m4: int = 0
    override_servo: int = 0


@dataclass
class ControllerProfile:
    """Maps a physical controller's axes/buttons onto the glider's logical
    controls. Axis/button indices are 0-based; use -1 (or omit a button) to mark
    a control as 'not present', in which case it is skipped at read time.

    *_sign values correct hardware orientation only (default 1.0 keeps the Xbox
    baseline). If a surface moves the wrong way on your controller, flip the
    matching sign here — no other code needs to change. The consumers keep the
    same SDL convention (stick up = negative Y for pitch/throttle)."""
    label: str
    roll_axis: int
    pitch_axis: int
    yaw_axis: int
    throttle_axis: int
    roll_sign: float = 1.0
    pitch_sign: float = 1.0
    yaw_sign: float = 1.0
    throttle_sign: float = 1.0
    throttle_from_axis: bool = False   # True: throttle stick is an absolute axis
    has_hat: bool = True               # False: no D-pad (get_hat is skipped)
    # Throttle-axis calibration (only used when throttle_from_axis): the raw
    # (post-sign) axis value at zero throttle and at full throttle. Defaults map
    # the standard SDL full-scale range; override per device (the InterLink-X
    # throttle idles at ~+0.80 and reads ~-0.83 at full).
    throttle_idle_raw: float = 1.0
    throttle_full_raw: float = -1.0
    # Rear trim/tune knobs (absolute axes). Each doubles as a trim knob and a PID
    # gain knob depending on the active mode: roll-knob=trim roll / Ki,
    # pitch-knob=trim pitch / Kp, yaw-knob=trim yaw / Kd. -1 disables (no knobs).
    trim_roll_axis: int = -1
    trim_pitch_axis: int = -1
    trim_yaw_axis: int = -1
    # Raw axis reading at each knob's physical extremes (down/full-CCW .. up/full-
    # CW). The InterLink-X rear knobs don't reach full-scale: they swing ~-0.7 to
    # ~+0.7, so calibrate here to map that actual travel onto the full value range
    # (otherwise the knob ends can't reach 0 / max). down maps to the range min.
    knob_raw_min: float = -1.0
    knob_raw_max: float = 1.0
    # USB vendor:product id (e.g. "1781:0e59"), used to check via lsusb whether
    # the device is actually attached to this machine's USB bus before trusting a
    # /dev/input/js* node. None disables the check (non-USB / unknown / macOS).
    usb_id: Optional[str] = None
    # command name -> button index; omit a command to disable it on this device
    buttons: Dict[str, int] = field(default_factory=dict)


# Xbox / generic gamepad: the original mapping (right stick = roll/pitch,
# left stick = yaw/throttle, D-pad steps throttle).
XBOX_PROFILE = ControllerProfile(
    label="Xbox / gamepad",
    roll_axis=3, pitch_axis=4, yaw_axis=0, throttle_axis=1,
    throttle_from_axis=False, has_hat=True,
    buttons={"trim_on": 0, "trim_off": 3, "breakpoint": 10,
             "arm": 4, "disarm": 5, "auto_on": 2, "auto_off": 1},
)

# GREAT PLANES InterLink-X / InterLink Elite RC sim controller. Standard Mode-2
# layout: right stick = elevator/aileron, left stick = rudder/throttle. Typical
# Linux/SDL axis order is aileron=0, elevator=1, throttle=2, rudder=3, and the
# device has no hat. Button indices are a best guess and guarded at read time,
# so a wrong/absent index simply disables that command. Breakpoint is dropped
# (not enough buttons), per the "leave it out if it doesn't map" request.
RC_PROFILE = ControllerProfile(
    label="RC sim controller (InterLink-X)",
    # Axis indices verified on real hardware via interlink_tester.py --wizard:
    #   a0 roll (right stick H), a1 pitch (right stick V, up=-),
    #   a5 yaw (left stick H), a2 throttle (left stick V, low=+0.8/high=-0.83).
    # Signs default +1.0 (SDL convention matches the Xbox baseline); flip an
    # individual *_sign to -1.0 here if that surface deflects the wrong way.
    roll_axis=0, pitch_axis=1, yaw_axis=5, throttle_axis=2,
    # Rudder was reversed on this airframe (right yaw command -> left rudder), so
    # the yaw axis is inverted here. Affects both rate-setpoint flight and the
    # manual-override rudder channel (both read via p.yaw_sign).
    yaw_sign=-1.0,
    throttle_from_axis=True, has_hat=False,
    # Throttle stick idles at ~+0.80 (raw) and reads ~-0.83 at full up; calibrate
    # so idle maps to 0.0 (motor off) and full to 1.0.
    throttle_idle_raw=0.80, throttle_full_raw=-0.83,
    # Rear knobs: a3 = aileron/roll trim & Ki, a4 = elevator/pitch trim & Kp,
    # a6 = rudder/yaw trim & Kd (per the 2026-07-20 mapping).
    trim_roll_axis=3, trim_pitch_axis=4, trim_yaw_axis=6,
    # Rear knobs read ~-0.7 (down) to ~+0.7 (up) on this unit, not full-scale.
    knob_raw_min=-0.7, knob_raw_max=0.7,
    usb_id="1781:0e59",
    # Button mapping (2026-07-20 rework). Edge = one action per press; the four
    # latch switches (arm/override/trim-mode/pid-mode) mirror the switch position
    # and are read via _latch_edge, not _edge_cmd.
    buttons={
        # edge (momentary) actions
        "breakpoint": 14,
        "auto_off": 16,     # "Manual control" switch -> autonomous OFF
        "auto_on": 15,      # "Autonomous/Mission mode" switch -> autonomous ON
        "trim_on": 11,      # Trim / lock servos
        "trim_off": 12,     # No-trim / unlock servos
        "pid_sel_roll": 5,  # select Roll for in-flight PID tuning
        "pid_sel_pitch": 6, # select Pitch for in-flight PID tuning
        "pid_sel_yaw": 8,   # select Yaw for in-flight PID tuning
        "save_tune": 2,     # persist current trim/gains to the deck's flash
        # latch (level) switches
        "arm_latch": 1,        # up = armed, down = disarmed
        "override_latch": 0,   # up = servo-PWM (manual override), down = rate setpoints
        "trim_mode": 4,        # in-flight trimming mode
        "pid_mode": 3,         # in-flight PID-tuning mode
    },
)

CONTROLLER_PROFILES = {"xbox": XBOX_PROFILE, "rc": RC_PROFILE}


# --------------------------------------------------------------------------- #
# User-editable button maps (Mapping tab)
# --------------------------------------------------------------------------- #
# Every command the control loop actually reads, in display order. Three kinds:
#   "latch"  the handler mirrors the switch *position* (_latch_edge), so the
#            action follows the switch rather than toggling on each press. Only
#            correct for an input that physically holds its position.
#   "edge"   fires once per 0->1 transition (_edge_cmd).
#   "toggle" fires on the 0->1 transition and flips a state the app owns, so a
#            momentary button behaves like a switch. Use this, not "latch", for
#            anything bound to a spring-return button: a latch read of a
#            momentary button turns the state off again on release.
#
# XBOX_PROFILE also carries "arm"/"disarm" entries, but nothing in
# _poll_controller_buttons reads them -- arming is driven solely by the
# "arm_latch" switch. They are deliberately left out of this list rather than
# offered as mappable, because a button that silently does nothing is exactly
# the kind of thing this tab exists to eliminate. (Worth revisiting separately.)
CONTROLLER_COMMANDS: List[Tuple[str, str, str]] = [
    ("arm_latch",     "Arm / disarm motor",            "latch"),
    ("override_latch", "Manual override on / off",     "latch"),
    ("trim_mode",     "Trim mode (rear knobs = trim)", "latch"),
    ("pid_mode",      "PID-tune mode (rear knobs)",    "latch"),
    ("trim_on",       "Trim ON / lock servos",         "edge"),
    ("trim_off",      "Trim OFF / unlock servos",      "edge"),
    ("auto_on",       "Autonomous mode ON",            "edge"),
    ("auto_off",      "Autonomous mode OFF",           "edge"),
    ("breakpoint",    "Write CSV breakpoint marker",   "edge"),
    ("pid_sel_roll",  "PID tune: select roll",         "edge"),
    ("pid_sel_pitch", "PID tune: select pitch",        "edge"),
    ("pid_sel_yaw",   "PID tune: select yaw",          "edge"),
    ("save_tune",     "Save trim / gains to flash",    "edge"),
    # Maneuver injection. Deliberately absent from both built-in profiles, so
    # they arrive UNMAPPED and have to be bound on the Mapping tab before any
    # button can fire a maneuver -- the logic ships inert.
    ("inject_arm",    "Maneuver injector arm / disarm", "toggle"),
    ("inject_fire",   "Fire the selected maneuver",     "edge"),
    ("inject_abort",  "Abort maneuver injection",       "edge"),
]

# Commands whose loss or misassignment has flight-safety consequences. Leaving
# these unmapped fails safe (you simply cannot arm, or cannot enter override),
# but it should never happen silently -- the Mapping tab warns.
SAFETY_CRITICAL_COMMANDS = {"arm_latch", "override_latch"}

UNMAPPED = -1

# The four primary flight axes, in display order:
#   (key, label, detection prompt, expected logical sign for that gesture)
#
# The last field is the part worth understanding. Detection can tell which axis
# moved and which way it went in *raw SDL* terms, but "which way is correct" is a
# convention this codebase already fixed, and it is NOT uniformly +1:
#
#   roll     RC_PROFILE a0, sign +1. SDL: stick right = +1  -> right is POSITIVE.
#   pitch    RC_PROFILE a1, sign +1, and the profile docstring states the SDL
#            convention "stick up = negative Y for pitch". Stick up therefore
#            reads -1 and stays -1 -> up is NEGATIVE.
#   yaw      SDL: stick right = +1 -> right is POSITIVE. (RC_PROFILE carries
#            yaw_sign=-1, but that is an *airframe* reversal, not the controller
#            convention, so it belongs in `inverted`, not here.)
#
# Throttle is absent on purpose: it does not use a sign at all. It is calibrated
# by raw endpoints (throttle_idle_raw/throttle_full_raw), which detection
# captures directly, so direction falls out of the calibration for free.
AXIS_CONTROLS: List[Tuple[str, str, str, float]] = [
    ("roll",     "Aileron / roll",    "Move the AILERON stick fully RIGHT",  +1.0),
    ("pitch",    "Elevator / pitch",  "Move the ELEVATOR stick fully UP",    -1.0),
    ("yaw",      "Rudder / yaw",      "Move the RUDDER stick fully RIGHT",   +1.0),
    ("throttle", "Throttle",          "Move the THROTTLE stick fully UP",     0.0),
]

# Detection tuning. The capture window has to be long enough to reach a stop
# without feeling like a hang, and the minimum deviation has to reject "the user
# clicked Detect and then nothing happened" -- 0.15 matches interlink_tester.py.
AXIS_DETECT_BASELINE_S = 0.8
AXIS_DETECT_CAPTURE_S = 4.0
AXIS_DETECT_MIN_DEV = 0.15


@dataclass
class ControllerMap:
    """A named, user-editable button layout for one controller type.

    Two independent things live here on purpose. ``commands`` is the mapping
    (which action sits on which button index); ``names`` is a description of the
    *hardware* (what the user calls each physical button). Keeping them separate
    means relabelling a switch never disturbs an assignment, and reassigning an
    action never loses a label.
    """
    name: str
    controller_type: str = "rc"
    commands: Dict[str, int] = field(default_factory=dict)
    names: Dict[int, str] = field(default_factory=dict)
    # Primary flight axes: "roll"/"pitch"/"yaw"/"throttle" -> axis index, plus the
    # sign that orients each one. Detection sets the index and the orientation
    # sign; `inverted` is the user's separate "this surface moves the wrong way on
    # my airframe" flip, kept apart so re-detecting an axis never silently undoes
    # a reversal that was established by actually flying it.
    axes: Dict[str, int] = field(default_factory=dict)
    axis_signs: Dict[str, float] = field(default_factory=dict)
    inverted: Dict[str, bool] = field(default_factory=dict)
    # Throttle-stick calibration captured during detection: the raw axis reading
    # at rest and at full. None -> keep the built-in profile's values.
    throttle_idle_raw: Optional[float] = None
    throttle_full_raw: Optional[float] = None

    def effective_sign(self, control: str) -> float:
        """The sign actually written into the profile: detected orientation
        combined with the user's invert flag."""
        base = float(self.axis_signs.get(control, 1.0))
        return -base if self.inverted.get(control) else base

    def button_label(self, idx: int) -> str:
        """Human text for a button index: the user's name if they gave one,
        otherwise a bare index. Used everywhere a button is shown."""
        if idx is None or idx < 0:
            return "— unmapped —"
        custom = self.names.get(idx, "").strip()
        return f"{idx} — {custom}" if custom else f"button {idx}"

    def conflicts(self) -> Dict[int, List[str]]:
        """Button indices carrying more than one command -> the command names.
        Double-booking is legal (some layouts genuinely want it) but is almost
        always a mistake, so the tab surfaces it rather than blocking it."""
        by_idx: Dict[int, List[str]] = {}
        for cmd, idx in self.commands.items():
            if idx is not None and idx >= 0:
                by_idx.setdefault(idx, []).append(cmd)
        return {i: cmds for i, cmds in by_idx.items() if len(cmds) > 1}

    def warnings(self) -> List[str]:
        """Human-readable problems worth showing before this map is used.
        Warn-but-allow: none of these prevent the map being applied."""
        out: List[str] = []
        for cmd in sorted(SAFETY_CRITICAL_COMMANDS):
            if self.commands.get(cmd, UNMAPPED) < 0:
                label = dict((c, l) for c, l, _ in CONTROLLER_COMMANDS).get(cmd, cmd)
                out.append(f"'{label}' is unmapped — it cannot be triggered from the controller.")
        for idx, cmds in sorted(self.conflicts().items()):
            out.append(f"{self.button_label(idx)} is assigned to {len(cmds)} commands: "
                       + ", ".join(sorted(cmds)))

        # Axes. Unlike buttons, none of these are optional: every one drives a
        # control surface or the motor, so an unset axis is always a fault.
        axis_labels = {k: l for k, l, _, _ in AXIS_CONTROLS}
        by_axis: Dict[int, List[str]] = {}
        for key, _, _, _ in AXIS_CONTROLS:
            idx = self.axes.get(key, UNMAPPED)
            if idx is None or idx < 0:
                out.append(f"'{axis_labels[key]}' has no axis assigned — that control is dead.")
            else:
                by_axis.setdefault(idx, []).append(axis_labels[key])
        for idx, ctrls in sorted(by_axis.items()):
            if len(ctrls) > 1:
                out.append(f"Axis {idx} is shared by {', '.join(sorted(ctrls))} — "
                           f"one stick would move two surfaces.")
        if self.throttle_idle_raw is not None and self.throttle_full_raw is not None:
            span = abs(self.throttle_idle_raw - self.throttle_full_raw)
            if span < 0.2:
                out.append(f"Throttle travel is only {span:.2f} of full scale — "
                           f"re-detect it and move the stick all the way.")
        return out


def default_controller_map(controller_type: str) -> ControllerMap:
    """A ControllerMap seeded from the built-in profile, so a first-time user starts
    from the working layout instead of a blank sheet."""
    prof = CONTROLLER_PROFILES.get(controller_type, XBOX_PROFILE)
    cmds = {cmd: int(prof.buttons.get(cmd, UNMAPPED)) for cmd, _, _ in CONTROLLER_COMMANDS}
    axes = {"roll": prof.roll_axis, "pitch": prof.pitch_axis,
            "yaw": prof.yaw_axis, "throttle": prof.throttle_axis}
    # Seed orientation from the profile's *_sign, but split it: the magnitude-1
    # sign the profile carries is the product of convention and any airframe
    # reversal, and we cannot tell them apart from here. Treat a negative profile
    # sign as the user's invert flag (that is what it was used for -- RC yaw was
    # flipped because the rudder was reversed on this airframe), leaving the
    # detected orientation at the convention default.
    signs = {"roll": prof.roll_sign, "pitch": prof.pitch_sign,
             "yaw": prof.yaw_sign, "throttle": prof.throttle_sign}
    inverted = {k: (v < 0) for k, v in signs.items()}
    return ControllerMap(name=f"{prof.label} (built-in)", controller_type=controller_type,
                         commands=cmds, names={}, axes=axes,
                         axis_signs={k: 1.0 for k in axes}, inverted=inverted,
                         throttle_idle_raw=prof.throttle_idle_raw,
                         throttle_full_raw=prof.throttle_full_raw)


def load_controller_maps() -> Tuple[Dict[str, ControllerMap], Dict[str, str]]:
    """Read every saved map plus the active-map choice per controller type.

    Mirrors the tolerance of load_default_gains/_load_setup_defaults: a missing
    or malformed file, or any single bad entry, degrades to the built-in profile
    rather than raising. A controller config that throws on startup would leave
    the GUI unusable at the flight line.
    """
    maps: Dict[str, ControllerMap] = {}
    active: Dict[str, str] = {}
    try:
        with open(CONTROLLER_MAPS_FILE, "r", encoding="utf-8") as fh:
            saved = json.load(fh)
    except FileNotFoundError:
        return maps, active
    except (OSError, ValueError):
        return maps, active
    if not isinstance(saved, dict):
        return maps, active

    for name, entry in (saved.get("maps") or {}).items():
        if not isinstance(entry, dict):
            continue
        ctype = entry.get("controller_type", "rc")
        if ctype not in CONTROLLER_PROFILES:
            continue
        bm = ControllerMap(name=str(name), controller_type=ctype)
        for cmd, _, _ in CONTROLLER_COMMANDS:
            try:
                bm.commands[cmd] = int((entry.get("commands") or {}).get(cmd, UNMAPPED))
            except (TypeError, ValueError):
                bm.commands[cmd] = UNMAPPED
        # JSON object keys are always strings; button indices are ints here.
        for k, v in (entry.get("names") or {}).items():
            try:
                bm.names[int(k)] = str(v)
            except (TypeError, ValueError):
                continue
        # Axes. A map written before axis support existed simply has no "axes"
        # key; fall back to the built-in profile per control rather than dropping
        # the whole map, so an existing button layout keeps working.
        seed = default_controller_map(ctype)
        for key, _, _, _ in AXIS_CONTROLS:
            try:
                bm.axes[key] = int((entry.get("axes") or {}).get(key, seed.axes[key]))
            except (TypeError, ValueError):
                bm.axes[key] = seed.axes[key]
            try:
                bm.axis_signs[key] = float(
                    (entry.get("axis_signs") or {}).get(key, seed.axis_signs[key]))
            except (TypeError, ValueError):
                bm.axis_signs[key] = 1.0
            bm.inverted[key] = bool(
                (entry.get("inverted") or {}).get(key, seed.inverted[key]))
        for attr in ("throttle_idle_raw", "throttle_full_raw"):
            raw = entry.get(attr)
            try:
                setattr(bm, attr, float(raw) if raw is not None else getattr(seed, attr))
            except (TypeError, ValueError):
                setattr(bm, attr, getattr(seed, attr))
        maps[bm.name] = bm

    for ctype, name in (saved.get("active") or {}).items():
        if ctype in CONTROLLER_PROFILES and isinstance(name, str):
            active[ctype] = name
    return maps, active


def save_controller_maps(maps: Dict[str, ControllerMap], active: Dict[str, str]) -> None:
    """Persist all maps (raises OSError on failure, like save_default_gains)."""
    payload = {
        "version": 1,
        "active": active,
        "maps": {
            bm.name: {
                "controller_type": bm.controller_type,
                "commands": bm.commands,
                "names": {str(k): v for k, v in bm.names.items()},
                "axes": bm.axes,
                "axis_signs": bm.axis_signs,
                "inverted": bm.inverted,
                "throttle_idle_raw": bm.throttle_idle_raw,
                "throttle_full_raw": bm.throttle_full_raw,
            }
            for bm in maps.values()
        },
    }
    with open(CONTROLLER_MAPS_FILE, "w", encoding="utf-8") as fh:
        json.dump(payload, fh, indent=2)


def active_controller_map(controller_type: str) -> Optional[ControllerMap]:
    """The saved map marked active for this controller type, or None to use the
    built-in profile untouched."""
    maps, active = load_controller_maps()
    name = active.get(controller_type)
    if not name:
        return None
    return maps.get(name)


# --------------------------------------------------------------------------- #
# Maneuver injection (flight-test excitation overlaid on the pilot's commands)
# --------------------------------------------------------------------------- #
# WHY THIS EXISTS
# ---------------
# Tuning a rate loop from piloted flight is guesswork, because the pilot and the
# controller are in the loop together: when the aircraft settles you cannot tell
# whether the gains did it or the pilot did. A maneuver injector breaks that
# ambiguity by adding a KNOWN, REPEATABLE perturbation to the rate setpoint while
# the pilot keeps flying. The response to that perturbation is attributable to the
# controller alone, so rise time, overshoot, damping and the kff/kp split can be
# read straight off the log -- and compared across flights, because the input was
# identical every time.
#
# The injection is ADDITIVE on the commanded rate (r), not on the surface. That
# keeps it inside the control loop being measured: the loop sees a step/doublet in
# its own reference, which is exactly the transfer function the analysis fits.
#
# SIGN CONVENTION: body axes, as the firmware sees them (and as controller.*Rate
# is logged): +roll = right roll rate, +pitch = nose-up rate, +yaw = nose-right
# rate. _send_rate_setpoint converts the file's stick convention into this one
# before adding the injection, so a maneuver's amplitude means the same thing
# whichever flight mode is driving.
MANEUVER_AXES = ("roll", "pitch", "yaw")

# (key, label, which extra fields matter) -- the shapes worth having on a glider.
# doublet is the workhorse (zero net attitude change, excites one frequency band);
# 3-2-1-1 is the standard multistep that covers a wide band in one short pass;
# the chirp is for a proper frequency sweep when you want a Bode-style picture.
MANEUVER_SHAPES: List[Tuple[str, str]] = [
    ("step",    "Step (hold A for the duration)"),
    ("doublet", "Doublet (+A then -A, half the duration each)"),
    ("3211",    "3-2-1-1 multistep (+3 -2 +1 -1 pulses)"),
    ("sine",    "Sine dwell (N cycles at one frequency)"),
    ("chirp",   "Chirp (linear frequency sweep f0 -> f1)"),
]
MANEUVER_SHAPE_KEYS = tuple(k for k, _ in MANEUVER_SHAPES)

# ----- safety gates (all enforced on the worker thread, every tick) --------- #
# 1. Amplitude ceiling. The injection may never ask for more than this fraction
#    of the axis's configured rate limit, no matter what the library says. A
#    hand-edited JSON or a fat-fingered spin box therefore cannot command a
#    full-scale rate step; the worst case is a little over half of a limit the
#    user already chose to fly with.
MANEUVER_MAX_AMPLITUDE_FRACTION = 0.6
# 2. Duration ceiling, and a hard watchdog on top of it. Nothing can leave the
#    injector commanding a rate for longer than this even if a shape function
#    misbehaves -- the tick ends any run past its own total duration.
MANEUVER_MAX_DURATION_S = 10.0
# 3. Re-trigger cooldown. Back-to-back doublets contaminate each other's
#    response, and a bouncing button must not fire twice.
MANEUVER_COOLDOWN_S = 1.0
# 4. Pilot override. Stick deflection past this fraction of full travel on the
#    axis being excited aborts the run instantly: moving the stick is the
#    pilot's reflex when something looks wrong, so it must be the abort action
#    rather than something that fights the injection.
MANEUVER_ABORT_STICK = 0.5
# Settle ceiling: the quiet lead-in / lead-out that brackets each run so the log
# contains the trimmed baseline the response is measured against.
MANEUVER_MAX_SETTLE_S = 5.0


@dataclass
class Maneuver:
    """One injectable excitation: an amplitude-scaled unit shape on one axis.

    ``settle_s`` is dead time at zero injection before AND after the shape. It
    is part of the run (the log is annotated across the whole thing) because a
    response is only readable against a known-quiet baseline -- without it the
    fit has nothing to measure the pre-input trim state from.
    """
    name: str = "new maneuver"
    axis: str = "pitch"
    shape: str = "doublet"
    amplitude_dps: float = 20.0      # peak commanded rate, deg/s, body axes
    duration_s: float = 1.0          # length of the shape itself
    settle_s: float = 0.5            # quiet lead-in and lead-out
    cycles: float = 3.0              # sine dwell only
    f_start_hz: float = 0.5          # chirp only
    f_end_hz: float = 4.0            # chirp only
    enabled: bool = True             # part of the advance-through-list sequence

    def sanitized(self) -> "Maneuver":
        """A copy with every field forced into a legal range. Applied on load
        and again when the GUI pushes the library, so neither a hand-edited file
        nor a future widget change can hand the control loop a nonsense run."""
        return Maneuver(
            name=(str(self.name).strip() or "unnamed")[:40],
            axis=self.axis if self.axis in MANEUVER_AXES else "pitch",
            shape=self.shape if self.shape in MANEUVER_SHAPE_KEYS else "doublet",
            # The amplitude is clamped again at run time against the live rate
            # limit (see GliderWorker._injection_tick); this is only the
            # sanity bound on the stored value.
            amplitude_dps=clamp(float(self.amplitude_dps), -180.0, 180.0),
            duration_s=clamp(float(self.duration_s), 0.1, MANEUVER_MAX_DURATION_S),
            settle_s=clamp(float(self.settle_s), 0.0, MANEUVER_MAX_SETTLE_S),
            cycles=clamp(float(self.cycles), 0.5, 20.0),
            f_start_hz=clamp(float(self.f_start_hz), 0.1, 20.0),
            f_end_hz=clamp(float(self.f_end_hz), 0.1, 20.0),
            enabled=bool(self.enabled),
        )

    @property
    def total_duration_s(self) -> float:
        return 2.0 * self.settle_s + self.duration_s

    def unit_value(self, t: float) -> float:
        """The shape at time ``t`` seconds into the run, normalised to +/-1.

        Every shape is defined to return exactly 0.0 outside the active window,
        so a run always begins and ends at zero injection -- there is no step
        discontinuity handed to the aircraft when a maneuver completes.
        """
        t -= self.settle_s
        d = self.duration_s
        if t < 0.0 or t >= d:
            return 0.0
        shape = self.shape
        if shape == "step":
            return 1.0
        if shape == "doublet":
            return 1.0 if t < 0.5 * d else -1.0
        if shape == "3211":
            # Pulse widths 3,2,1,1 in units of d/7, signs + - + -. The classic
            # multistep: one pass excites roughly a decade of frequency, which a
            # single doublet cannot.
            u = d / 7.0
            if t < 3.0 * u:
                return 1.0
            if t < 5.0 * u:
                return -1.0
            if t < 6.0 * u:
                return 1.0
            return -1.0
        if shape == "sine":
            f = self.cycles / d          # exactly N cycles in the window
            return math.sin(2.0 * math.pi * f * t)
        if shape == "chirp":
            # Linear sweep: instantaneous f = f0 + k t, so the phase (its
            # integral) carries the 1/2 k t^2 term. Integrating the frequency
            # rather than evaluating sin(2 pi f(t) t) is what keeps the sweep
            # phase-continuous -- the naive form jumps and injects harmonics.
            k = (self.f_end_hz - self.f_start_hz) / d
            phase = 2.0 * math.pi * (self.f_start_hz * t + 0.5 * k * t * t)
            return math.sin(phase)
        return 0.0

    def describe(self) -> str:
        bits = [f"{self.axis} {self.shape}", f"A={self.amplitude_dps:g} deg/s",
                f"dur={self.duration_s:g}s"]
        if self.shape == "sine":
            bits.append(f"{self.cycles:g} cyc ({self.cycles / self.duration_s:.2f} Hz)")
        elif self.shape == "chirp":
            bits.append(f"{self.f_start_hz:g}->{self.f_end_hz:g} Hz")
        if self.settle_s > 0:
            bits.append(f"settle={self.settle_s:g}s")
        return ", ".join(bits)


def default_maneuvers() -> List[Maneuver]:
    """A starter test card. Deliberately conservative amplitudes: the point of
    the first flight with this feature is to confirm the injection is visible in
    the log and survivable, not to get a good fit."""
    return [
        Maneuver(name="pitch doublet", axis="pitch", shape="doublet",
                 amplitude_dps=20.0, duration_s=1.0, settle_s=0.5),
        Maneuver(name="roll doublet", axis="roll", shape="doublet",
                 amplitude_dps=25.0, duration_s=0.8, settle_s=0.5),
        Maneuver(name="yaw doublet", axis="yaw", shape="doublet",
                 amplitude_dps=20.0, duration_s=1.0, settle_s=0.5),
        Maneuver(name="pitch 3-2-1-1", axis="pitch", shape="3211",
                 amplitude_dps=20.0, duration_s=2.8, settle_s=0.5),
        Maneuver(name="pitch step", axis="pitch", shape="step",
                 amplitude_dps=15.0, duration_s=1.5, settle_s=0.5),
    ]


def load_maneuvers() -> List[Maneuver]:
    """Read the saved test card, degrading field by field like
    load_default_gains: a malformed library must not stop the GUI launching at
    the flight line, and a single bad entry must not take the rest with it."""
    try:
        with open(MANEUVER_FILE, "r", encoding="utf-8") as fh:
            saved = json.load(fh)
    except FileNotFoundError:
        return default_maneuvers()
    except (OSError, ValueError):
        return default_maneuvers()
    entries = saved.get("maneuvers") if isinstance(saved, dict) else saved
    if not isinstance(entries, list):
        return default_maneuvers()
    out: List[Maneuver] = []
    for entry in entries:
        if not isinstance(entry, dict):
            continue
        m = Maneuver()
        for key in vars(m):
            if key in entry:
                try:
                    setattr(m, key, entry[key])
                except (TypeError, ValueError):
                    pass
        try:
            out.append(m.sanitized())
        except (TypeError, ValueError):
            continue
    return out or default_maneuvers()


def save_maneuvers(maneuvers: List[Maneuver]) -> None:
    """Persist the test card (raises OSError on failure, like save_default_gains)."""
    payload = {"version": 1, "maneuvers": [asdict(m) for m in maneuvers]}
    with open(MANEUVER_FILE, "w", encoding="utf-8") as fh:
        json.dump(payload, fh, indent=2)


class ManeuverInjector:
    """Run-state machine for maneuver injection. Pure logic, no Qt, no cflib.

    Lives on and is only touched by the worker thread. It knows nothing about
    the aircraft: the worker decides whether injection is permitted this tick
    (``_injection_block_reason``) and clamps the output against the live rate
    limits. Keeping the gates outside means this class cannot be the reason a
    gate is missed, and it can be reasoned about (and tested) on its own.

    The two-step arm/trigger split is deliberate. ``armed`` is a conscious "I
    intend to inject on this flight" state the pilot sets once; ``trigger`` is
    the per-run action that a controller button will later be bound to. A stray
    button press on an unarmed injector does nothing at all.
    """

    def __init__(self) -> None:
        self.library: List[Maneuver] = []
        self.selected = 0
        self.armed = False
        self.advance = False          # step to the next enabled entry per run
        self._run: Optional[Maneuver] = None
        self._t0 = 0.0
        self._last_end = 0.0
        self._saturated = False
        self._peak_cmd = 0.0
        # (event_name, v1, v2) tuples for the worker to log + echo. Queued here
        # rather than written directly so this class stays free of the logger.
        self._events: List[Tuple[str, str, str]] = []

    # ----- library ---------------------------------------------------------- #
    def set_library(self, maneuvers: List[Maneuver], selected: Optional[int] = None) -> None:
        self.library = [m.sanitized() for m in maneuvers]
        if selected is not None:
            self.selected = selected
        self.selected = max(0, min(self.selected, max(0, len(self.library) - 1)))

    def selected_maneuver(self) -> Optional[Maneuver]:
        if 0 <= self.selected < len(self.library):
            return self.library[self.selected]
        return None

    # ----- state ------------------------------------------------------------ #
    @property
    def active(self) -> bool:
        return self._run is not None

    @property
    def axis(self) -> Optional[str]:
        return self._run.axis if self._run is not None else None

    def drain_events(self) -> List[Tuple[str, str, str]]:
        out, self._events = self._events, []
        return out

    # ----- lifecycle -------------------------------------------------------- #
    def set_armed(self, on: bool, now: float) -> None:
        on = bool(on)
        if on == self.armed:
            return
        self.armed = on
        if not on:
            self.abort("INJECTOR_DISARMED", now)
        self._events.append(("MANEUVER_ARMED" if on else "MANEUVER_DISARMED", "", ""))

    def trigger(self, now: float) -> Tuple[bool, str]:
        """Start the selected maneuver. Returns (started, human message). Every
        refusal is reported rather than silently ignored -- a trigger that does
        nothing without saying why is how you end up flying a pass you think was
        recorded and was not."""
        if not self.armed:
            return False, "injector is not armed"
        if self._run is not None:
            return False, f"'{self._run.name}' is still running"
        if now - self._last_end < MANEUVER_COOLDOWN_S:
            wait = MANEUVER_COOLDOWN_S - (now - self._last_end)
            return False, f"cooldown, {wait:.1f}s left"
        man = self.selected_maneuver()
        if man is None:
            return False, "the library is empty"
        self._run = man
        self._t0 = now
        self._saturated = False
        self._peak_cmd = 0.0
        self._events.append((
            "MANEUVER_START", man.name,
            f"axis={man.axis} shape={man.shape} amp={man.amplitude_dps:g} "
            f"dur={man.duration_s:g} settle={man.settle_s:g}"))
        return True, f"injecting '{man.name}' ({man.describe()})"

    def abort(self, reason: str, now: float) -> bool:
        """End any run early. Returns True if a run was actually stopped."""
        if self._run is None:
            return False
        self._finish(reason, now)
        return True

    def _finish(self, reason: str, now: float) -> None:
        man = self._run
        self._run = None
        self._last_end = now
        if man is not None:
            self._events.append((
                "MANEUVER_END", man.name,
                f"reason={reason} peak={self._peak_cmd:.1f} "
                f"saturated={'1' if self._saturated else '0'}"))

    def note_saturation(self) -> None:
        """Flag that the commanded total hit a rate limit during this run. The
        analysis cares: a clamped sample measures the limit, not the response,
        so a saturated pass should not be fitted."""
        self._saturated = True

    # ----- per-tick --------------------------------------------------------- #
    def tick(self, now: float) -> Tuple[float, float, float]:
        """The injection to add this tick, as (roll, pitch, yaw) deg/s in body
        axes. All zeros when no run is active."""
        man = self._run
        if man is None:
            return (0.0, 0.0, 0.0)
        elapsed = now - self._t0
        # Watchdog: end on duration, and also if the clock ever runs past it (a
        # stalled loop, a suspended process, a shape that misreports its length).
        if elapsed >= man.total_duration_s:
            self._finish("COMPLETED", now)
            if self.advance:
                self._advance()
            return (0.0, 0.0, 0.0)
        value = man.amplitude_dps * man.unit_value(elapsed)
        self._peak_cmd = max(self._peak_cmd, abs(value))
        return (
            value if man.axis == "roll" else 0.0,
            value if man.axis == "pitch" else 0.0,
            value if man.axis == "yaw" else 0.0,
        )

    def _advance(self) -> None:
        """Move the selection to the next enabled entry, wrapping. This is what
        turns one button into a test card: each press flies the next point."""
        n = len(self.library)
        if n == 0:
            return
        for step in range(1, n + 1):
            idx = (self.selected + step) % n
            if self.library[idx].enabled:
                self.selected = idx
                return

    def status(self) -> dict:
        """Snapshot for the GUI status line (plain data, safe to send over a
        signal to the other thread)."""
        man = self._run
        sel = self.selected_maneuver()
        return {
            "armed": self.armed,
            "active": man is not None,
            "running": man.name if man is not None else "",
            "selected": sel.name if sel is not None else "",
            "selected_index": self.selected,
            "saturated": self._saturated,
        }


# --------------------------------------------------------------------------- #
# Worker: connection + control loop (runs on its own thread)
# --------------------------------------------------------------------------- #
class GliderWorker(QtCore.QObject):
    console = Signal(str)        # text for the Console tab
    status = Signal(str)         # status-bar text
    connected = Signal(bool)     # connection established / torn down
    telemetry = Signal(float, float)  # vbat, rssi (for status readout)
    override_state = Signal(int, int, int, int, int)  # m1,m2,m3,m4,servo live cmd
    override_mode = Signal(bool, bool)  # manual_override on/off, drive-with-controller
    trim_value = Signal(str, int)  # (axis, value) trim read back from the deck / live trim
    surface_map = Signal(str, int, int)  # (channel, surface_code, invert) read back from deck
    pid_value = Signal(str, str, float)  # (axis, term, value) live in-flight PID tune
    buttons_state = Signal(object)  # (armed, n_buttons, frozenset of pressed indices)
    axes_state = Signal(object)     # (armed, tuple of raw axis values) for axis detect
    injector_state = Signal(object)  # ManeuverInjector.status() dict for the Maneuvers tab

    def __init__(self, buffers: PlotBuffers):
        super().__init__()
        self.buffers = buffers
        self.config = SessionConfig()
        self.live = LiveControl()
        self._lock = threading.Lock()
        self._cmd_queue: "queue.Queue" = queue.Queue()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

        self.logs: Optional[CsvLogBundle] = None
        self.cf = None
        self.commander = None
        self.joystick = None
        self.axis_centers: List[float] = []  # per-axis rest offsets, captured at connect
        self.profile = XBOX_PROFILE
        self.log_configs: Dict[str, LogConfig] = {}
        self.log_enabled: Dict[str, bool] = {}
        self.last_servo_value = 0
        self.failsafe_active = False
        self.last_connection_seen_at = 0.0
        self.last_button_state: Dict[int, int] = {}
        self.last_hat_state: Tuple[int, int] = (0, 0)
        # Live button state pushed to the Mapping tab. SDL may only be read from
        # the thread that owns the joystick, so the GUI cannot poll it directly --
        # the worker samples it here and emits. Cached so we only signal on an
        # actual change (plus a slow heartbeat), instead of 100 emits a second.
        self._last_buttons_sent: Optional[frozenset] = None
        self._last_buttons_emit = 0.0
        self._last_axes_emit = 0.0
        self.last_override_servo = 0
        # Manual-override state: cache of last param value actually written
        # (dirty-check to avoid flooding the radio) + non-blocking servo slew.
        self._last_sent: Dict[str, int] = {}
        self._override_servo_cmd = 0
        self._last_override_emit: Optional[Tuple[int, int, int, int, int]] = None
        self._last_override_write = 0.0
        self._last_throttle_write: Optional[float] = None
        # Commander-fast override: when True, stream the raw motor/servo command on
        # the setpoint channel (responsive, needs modded firmware) instead of the
        # slower param-based motorPowerSet.* path. Toggled from the override tab.
        self._fast_override = True
        # Set for real in _create_log_configs once the deck's TOC is known; defined
        # here so _on_motor_log cannot raise AttributeError if a callback ever fires
        # before that runs.
        self.log_has_servo_angle = False
        self._last_arm_request = 0.0
        # ----- in-flight trim / PID-tune mode state ----------------------- #
        # Latch switches are level-driven; remember the last observed level so we
        # act only on a physical flip (see _latch_edge).
        self._latch_prev: Dict[str, Optional[bool]] = {}
        self._trim_mode = False       # rear knobs drive surface trims
        self._pid_mode = False        # rear knobs drive selected-axis PID gains
        self._pid_axis = "roll"       # which axis the PID knobs currently tune
        # True once the PID latch was flipped on while armed: mode stays blocked
        # until the switch is cycled off->on again (after disarming).
        self._pid_latch_blocked = False
        # Knob catch/takeover: a knob is ignored until its mapped value passes
        # through the stored value. Keyed "trim:<axis>" / "pid:<axis>:<term>".
        self._knob_caught: Dict[str, bool] = {}
        self._knob_catch_sign: Dict[str, float] = {}
        # ----- log block stall detection ---------------------------------- #
        # Monotonic time of the last log packet of ANY block, stamped by the
        # wrapper in _log_cb. 0.0 means logging has not been started yet, which
        # must not count as a stall. Written on cflib's rx thread and read on the
        # worker thread; a float rebind is atomic in CPython, so no lock.
        self._last_log_rx_at = 0.0
        self._log_recovery_at = 0.0
        self._log_recovery_count = 0
        self._log_cb_errors: Dict[str, float] = {}
        # ----- pid readback verification ---------------------------------- #
        # _pid_echo is written by _pid_echo_cb on cflib's rx thread and read by
        # _pid_verify_tick on the worker thread, hence the lock. _pid_verify is
        # (expected_values, deadline) while an apply is outstanding, else None.
        self._pid_echo: Dict[str, float] = {}
        self._pid_echo_lock = threading.Lock()
        self._pid_verify: Optional[Tuple[Dict[str, float], float]] = None
        # ----- maneuver injection ----------------------------------------- #
        # Propulsion throttle route for normal flight: True = streamed
        # meta-command (fast, needs the modded firmware), False = servo.servoAngle
        # param writes (slow, works on stock). Toggled from the Control tab.
        self.propulsion_fast = PROPULSION_FAST_DEFAULT
        self.injector = ManeuverInjector()
        self.injector.set_library(load_maneuvers())
        # Last injection actually commanded, body axes deg/s. Written here on the
        # worker thread and read by the cflib controller-log callback on ITS
        # thread; a whole-tuple rebind is atomic in CPython, so the reader always
        # sees a consistent triple and no lock is needed (the same reasoning as
        # last_servo_value). This is what lands in the inj_* CSV columns, and it
        # is the only record of what the pilot did NOT command.
        self._inj_cmd: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        # Abort a run when the stick on the excited axis moves past
        # MANEUVER_ABORT_STICK. User-defeatable from the tab, because once the
        # feature is trusted you may well want to inject while holding a turn.
        self._inj_stick_abort = True
        self._last_injector_emit = 0.0
        self._last_injector_status: Optional[tuple] = None

    # ----- public API (called from GUI thread) ----------------------------- #
    def start(self, config: SessionConfig) -> None:
        if self._thread and self._thread.is_alive():
            return
        self.config = config
        base = CONTROLLER_PROFILES.get(config.controller_type, XBOX_PROFILE)
        # Copy the profile (and its buttons dict) before anything can edit it.
        # XBOX_PROFILE / RC_PROFILE are module-level singletons, so applying a
        # user map in place would permanently mutate the built-in layout for the
        # rest of the process -- including the "revert to built-in" path, which
        # would then revert to the edited values.
        self.profile = ControllerProfile(**{**asdict(base), "buttons": dict(base.buttons)})
        saved = active_controller_map(config.controller_type)
        if saved is not None:
            self._apply_controller_map(saved)
        self._stop.clear()
        self.failsafe_active = False
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()

    def post(self, action: str, payload: object = None) -> None:
        """Queue a one-shot command for the worker loop."""
        self._cmd_queue.put((action, payload))

    def _apply_controller_map(self, bmap: "ControllerMap") -> None:
        """Overwrite the active profile's button and axis assignments from a map.

        Only entries present in the map are touched, and UNMAPPED (-1) is stored
        verbatim: _edge_cmd/_latch_edge already treat a negative index as 'not
        present on this device' and skip it, so unmapping is a no-op at read time
        rather than a special case anywhere in the control loop. _axis_c applies
        the same rule to axes.
        """
        for cmd, idx in bmap.commands.items():
            self.profile.buttons[cmd] = int(idx)
        for key, idx in bmap.axes.items():
            attr = f"{key}_axis"
            if hasattr(self.profile, attr):
                setattr(self.profile, attr, int(idx))
                setattr(self.profile, f"{key}_sign", bmap.effective_sign(key))
        if bmap.throttle_idle_raw is not None:
            self.profile.throttle_idle_raw = float(bmap.throttle_idle_raw)
        if bmap.throttle_full_raw is not None:
            self.profile.throttle_full_raw = float(bmap.throttle_full_raw)
        # axis_centers is sampled for EVERY axis at connect, not just the mapped
        # ones, so remapping indices needs no re-sample -- the rest offset for the
        # newly chosen index is already there.
        # Drop stale edge/latch history: a command that just moved to a different
        # button must not inherit the old button's last level, or the first poll
        # after a remap would read a phantom transition and fire the action.
        self.last_button_state.clear()
        self._latch_prev.clear()

    def update_live(self, **kwargs) -> None:
        """Mutate continuous control values atomically."""
        with self._lock:
            for k, v in kwargs.items():
                setattr(self.live, k, v)

    def _live_snapshot(self) -> LiveControl:
        with self._lock:
            return LiveControl(**vars(self.live))

    # ----- thread body ----------------------------------------------------- #
    def _log(self, text: str) -> None:
        self.console.emit(text)
        logs = self.logs  # local ref: avoid race if teardown nulls self.logs
        if logs is not None:
            try:
                logs.write_console(text)
            except Exception:
                pass

    def _run(self) -> None:
        cfg = self.config
        name = cfg.filename_prefix.strip() or f"flight_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        prefix = resolve_log_prefix(name)
        try:
            self.logs = CsvLogBundle(prefix)
            self.logs.write_event("SESSION_START", cfg.uri)
            self._log(f"Logging to prefix: {prefix}\n")

            cflib.crtp.init_drivers()
            self._setup_joystick()

            with SyncCrazyflie(cfg.uri, cf=Crazyflie(rw_cache="./cache")) as scf:
                self.cf = scf.cf
                self.commander = self.cf.commander
                self._bind_connection_callbacks()
                self.cf.console.receivedChar.add_callback(self._console_callback)
                self._log(f"Connected to Crazyflie: {cfg.uri}\n")
                self.connected.emit(True)

                self._configure_flight_controller()
                self._create_log_configs()
                self._bind_log_callbacks()
                self._start_enabled_logs()

                self.cf.param.set_value("usd.logging", "1")
                self.cf.platform.send_arming_request(True)
                self.last_connection_seen_at = time.monotonic()
                self.logs.write_breakpoint("SESSION_READY")
                self.status.emit("Ready")
                self._log("Glider configured and ready.\n")

                self._control_loop()
        except Exception as exc:  # surface any failure to the GUI
            self._log(f"ERROR: {exc}\n")
            self.status.emit(f"Error: {exc}")
        finally:
            self._shutdown()
            self.connected.emit(False)
            self.status.emit("Disconnected")

    # ----- setup ----------------------------------------------------------- #
    def _soft_replug(self) -> None:
        """Best-effort USB re-enumerate of the InterLink-X via the root-owned
        reset script, run with `sudo -n` so it never blocks on a password. If
        the NOPASSWD entry isn't installed it quietly no-ops and the user can
        physically replug instead."""
        if not os.path.exists(RESET_CMD):
            return
        try:
            r = subprocess.run(["sudo", "-n", RESET_CMD],
                               timeout=8, capture_output=True, text=True)
        except Exception:
            return
        if r.returncode != 0:
            return
        self._log("Soft-replugged the controller (USB re-enumerate).\n")
        # Wait for the joystick node to reappear after re-enumeration.
        end = time.monotonic() + 4.0
        while time.monotonic() < end:
            if glob.glob("/dev/input/js*"):
                time.sleep(0.5)
                return
            time.sleep(0.1)

    def _device_attached(self) -> Optional[bool]:
        """Whether the profile's USB id is present on this machine's USB bus per
        lsusb. Returns None when it can't be determined (no usb_id set, or lsusb
        unavailable, e.g. on macOS). Lets us tell 'never passed through to the
        VM' apart from a live device, and avoid trusting a stale js* node."""
        usb_id = getattr(self.profile, "usb_id", None)
        if not usb_id:
            return None
        try:
            out = subprocess.run(["lsusb"], timeout=3, capture_output=True, text=True)
        except Exception:
            return None
        if out.returncode != 0:
            return None
        return usb_id.lower() in out.stdout.lower()

    def _is_streaming(self, js, dwell: float = 1.0) -> bool:
        """True once the joystick reports at least one axis that isn't the SDL
        uninitialized default of -1.0. A device that's present but wedged
        mid-reset reads all axes at exactly -1.0, so this distinguishes a live
        stream from a stale one. Drains events while polling."""
        n = js.get_numaxes()
        if not n:
            return False
        end = time.monotonic() + dwell
        while time.monotonic() < end:
            pygame.event.get()  # drain + refresh
            if not all(js.get_axis(i) == -1.0 for i in range(n)):
                return True
            time.sleep(0.05)
        return False

    def _open_streaming_joystick(self, retries: int = 8, settle: float = 1.5):
        """Open joystick 0 and wait until it's actually streaming, retrying
        across re-enumerations (the node number can change after a reset).
        Returns the live joystick, or None if it never starts streaming.

        Fast path: when no controller is requested and none is present, gives up
        after the first attempt instead of spinning for the full retry budget."""
        for attempt in range(retries):
            attached = self._device_attached()
            pygame.init()
            pygame.joystick.init()
            if pygame.joystick.get_count() > 0:
                js = pygame.joystick.Joystick(0)
                js.init()
                if self._is_streaming(js):
                    return js
                # A js* node exists but no axis is streaming. If lsusb confirms
                # the USB device isn't actually on the bus, this is a stale node
                # left over after the device detached (the classic Parallels
                # passthrough symptom) -- say so plainly rather than "waiting".
                if attached is False:
                    self._log(
                        f"  a /dev/input/js* node exists but the controller "
                        f"({self.profile.usb_id}) is NOT on the VM's USB bus "
                        f"(stale node). Connect it to this VM in Parallels "
                        f"(Devices -> USB & Bluetooth). Attempt "
                        f"{attempt + 1}/{retries}...\n")
                else:
                    self._log(
                        f"  controller present but not streaming yet "
                        f"(attempt {attempt + 1}/{retries}); waiting...\n")
            else:
                if not self.config.use_controller and attached is not True:
                    # No controller wanted and none present: don't stall.
                    return None
                if attached is False:
                    self._log(
                        f"  controller ({self.profile.usb_id}) is NOT attached "
                        f"to the VM's USB bus. Connect it to this VM in Parallels "
                        f"(Devices -> USB & Bluetooth). Attempt "
                        f"{attempt + 1}/{retries}...\n")
                else:
                    self._log(
                        f"  no joystick yet (attempt {attempt + 1}/{retries}); "
                        f"waiting for re-enumeration...\n")
            try:
                pygame.joystick.quit()
                pygame.quit()
            except Exception:
                pass
            time.sleep(settle)
        return None

    def _setup_joystick(self) -> None:
        # Always try to bring up a joystick so the Manual Override "drive with
        # controller" option works even when the Setup-tab controller toggle is
        # off. Only hard-fail when the user explicitly requested a controller.
        if pygame is None:
            if self.config.use_controller:
                raise RuntimeError("Controller requested but pygame is not installed.")
            return
        # Re-enumerate the controller before opening it so a reconnect isn't
        # wedged by Parallels USB passthrough (no-ops if not set up).
        self._soft_replug()
        # Without this hint, SDL only delivers joystick events while its own
        # window has input focus. Since the visible window is Qt's (not
        # pygame's), the joystick would appear "dead" whenever the terminal or
        # GUI has focus. Must be set before pygame.init().
        os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")
        self.joystick = self._open_streaming_joystick()
        if self.joystick is None:
            if self.config.use_controller:
                raise RuntimeError(
                    "Controller enabled but it never started streaming (absent, "
                    "or all axes stuck at -1.0). Physically unplug/replug it and "
                    "reconnect.")
            return
        self._log(f"Using joystick: {self.joystick.get_name()}\n")
        self._log(
            f"  profile={self.profile.label}  axes={self.joystick.get_numaxes()}"
            f"  buttons={self.joystick.get_numbuttons()}"
            f"  hats={self.joystick.get_numhats()}\n")
        self.axis_centers = self._sample_axis_centers()
        p = self.profile
        self._log(
            f"  axis rest centers (roll/pitch/yaw) = "
            f"{self._axis_center(p.roll_axis):+.3f}/"
            f"{self._axis_center(p.pitch_axis):+.3f}/"
            f"{self._axis_center(p.yaw_axis):+.3f}"
            f"  (subtracted so a centered stick commands true zero)\n")

    def _reconnect_controller(self) -> None:
        """Re-open the controller mid-session without dropping the Crazyflie
        link: tears down the SDL joystick, re-enumerates (reset script), and
        retries until streaming. Runs on the worker thread via the command queue,
        so it interleaves safely with the control loop (never concurrent). A
        backup for when Parallels drops the passthrough during a flight."""
        if pygame is None:
            self._log("Reconnect: pygame not installed; cannot open a controller.\n")
            return
        self._log("Reconnecting controller...\n")
        self.status.emit("Reconnecting controller...")
        # Release the current SDL joystick/subsystem so a re-enumerated device
        # (possibly a new js* node) is picked up cleanly.
        if pygame.get_init():
            try:
                if self.joystick is not None:
                    self.joystick.quit()
            except Exception:
                pass
            try:
                pygame.joystick.quit()
                pygame.quit()
            except Exception:
                pass
        self.joystick = None
        # Drop cached input state so stale button/latch levels don't misfire and
        # the latch switches re-sync to their physical positions on first read.
        self.last_button_state.clear()
        self._latch_prev.clear()
        self._knob_caught.clear()
        self._knob_catch_sign.clear()
        self.last_hat_state = (0, 0)
        self._trim_mode = False
        self._pid_mode = False
        self._pid_latch_blocked = False

        self._soft_replug()
        os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")
        self.joystick = self._open_streaming_joystick()
        if self.joystick is None:
            self._log("Reconnect FAILED: controller never started streaming. "
                      "Check that it's connected to this VM in Parallels.\n")
            self.status.emit("Controller reconnect failed")
            return
        self.axis_centers = self._sample_axis_centers()
        p = self.profile
        self._log(
            f"Reconnected: {self.joystick.get_name()}  axes="
            f"{self.joystick.get_numaxes()} buttons={self.joystick.get_numbuttons()}"
            f"  rest(roll/pitch/yaw)={self._axis_center(p.roll_axis):+.3f}/"
            f"{self._axis_center(p.pitch_axis):+.3f}/"
            f"{self._axis_center(p.yaw_axis):+.3f}\n")
        self.status.emit("Controller reconnected")

    def _sample_axis_centers(self, seconds: float = 0.3) -> List[float]:
        """Read the resting position of every axis (sticks should be centered
        at connect) so small idle offsets don't leak into the setpoints. The
        InterLink-X yaw idles near -0.06, which used to exceed the deadband and
        produce a slow phantom yaw."""
        n = self.joystick.get_numaxes()
        centers = [0.0] * n
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            pygame.event.get()  # drain + refresh
            for i in range(n):
                centers[i] = self.joystick.get_axis(i)
            time.sleep(0.02)
        return centers

    def _axis_center(self, idx: int) -> float:
        """Captured rest offset for an axis (0.0 if unknown / absent)."""
        if 0 <= idx < len(self.axis_centers):
            return self.axis_centers[idx]
        return 0.0

    def _configure_flight_controller(self) -> None:
        self._bind_pid_echo_callback()
        self.cf.param.set_value("motorPowerSet.enable", "0")
        self.cf.param.set_value("flightmode.stabModeRoll", "0")
        self.cf.param.set_value("flightmode.stabModePitch", "0")
        self.cf.param.set_value("flightmode.stabModeYaw", "0")
        self._apply_pid_gains(self.config.gains)

        self.cf.param.set_value("fwActLpf.enable", "1" if self.config.fwactlpf_enable else "0")
        self.cf.param.set_value("fwActLpf.cutoffHz", max(0.1, self.config.fwactlpf_cutoff_hz))
        # Read back rather than trusting the write: fwActLpf.* are PARAM_PERSISTENT,
        # so a failed set leaves the previously stored value in place and the filter
        # silently stays on. When the filter state is the variable under test, the
        # log must record what the deck reports, not what the GUI asked for.
        try:
            lpf_on = int(float(self.cf.param.get_value("fwActLpf.enable")))
            lpf_hz = float(self.cf.param.get_value("fwActLpf.cutoffHz"))
        except Exception as exc:
            self._log(f"[lpf] could not read back fwActLpf state: {exc}\n")
            lpf_on, lpf_hz = -1, float("nan")
        self.logs.write_event("FW_ACT_LPF", f"enable={lpf_on}", f"cutoffHz={lpf_hz}")
        self._log(f"Output LPF {'ON' if lpf_on == 1 else 'OFF'} (deck reports "
                  f"enable={lpf_on}, cutoffHz={lpf_hz})\n")
        if lpf_on != int(self.config.fwactlpf_enable):
            self._log("[lpf] WARNING: deck state does not match the Setup tab!\n")

        # Read back the persisted per-surface servo trims (UINT16, center each
        # surface in toServoPwm) so the GUI shows the stored values.
        for axis, param in TRIM_PARAMS.items():
            try:
                trim = int(self.cf.param.get_value(param))
                self._last_sent[param] = trim
                self.trim_value.emit(axis, trim)
                self._log(f"Servo trim {param} = {trim}\n")
            except Exception as exc:
                self._log(f"Could not read {param}: {exc}\n")
        self._write_trim_state()

        # Read back the persisted servo mixer map (which surface each M# drives
        # plus its invert flag) so the Control tab reflects the stored config.
        for ch in SURFACE_MAP_CHANNELS:
            sp, ip = SURFACE_MAP_SURF_PARAM[ch], SURFACE_MAP_INV_PARAM[ch]
            try:
                surf = int(self.cf.param.get_value(sp))
                inv = int(self.cf.param.get_value(ip))
                self._last_sent[sp] = surf
                self._last_sent[ip] = inv
                self.surface_map.emit(ch, surf, inv)
                self._log(f"Surface map {ch.upper()}: surface={surf} invert={inv}\n")
            except Exception as exc:
                self._log(f"Could not read surface map for {ch.upper()}: {exc}\n")
        self._write_surface_map_state()

    # ----- latched-state events -------------------------------------------- #
    # TRIM_APPLIED / SURFACE_MAP_APPLIED mirror PID_APPLIED: they record the
    # complete mixer configuration at an instant, so clip_flights can latch the
    # last one at or before a clip's start and re-emit it as TRIM_STATE /
    # SURFACE_MAP_STATE. Offline analysis needs them because turning a
    # motor_m* count into deflection-about-trim requires
    # pwm = trim + axisSign*u: without trim it cannot locate the saturation
    # (trimPitch=30000 clips positive pitch at 0.916 of travel but negative at
    # 1.0), and without the surface map it cannot even say which channel is
    # which axis. The firmware's documented defaults are NOT this aircraft's
    # config, so only a readback is authoritative.
    #
    # Both emit ALL axes/channels every time, including after a single-axis
    # change. That is the whole point: TRIM_SET already records the per-axis
    # delta, but a latched *delta* would let a clip inherit two fresh axes and
    # one stale one, and a log that confidently states the wrong trim is worse
    # than one that states none -- it silently mislocates every saturation
    # margin downstream. Reading from _last_sent keeps them in step with what
    # was actually pushed to the deck.
    def _write_trim_state(self) -> None:
        self.logs.write_event(
            "TRIM_APPLIED",
            *(f"{axis}={self._last_sent.get(TRIM_PARAMS[axis], 'NA')}"
              for axis in ("roll", "pitch", "yaw")))

    def _write_surface_map_state(self) -> None:
        # 4 channels into 3 value columns, so m1/m2 share the first.
        parts = [f"{ch}:surf={self._last_sent.get(SURFACE_MAP_SURF_PARAM[ch], 'NA')}"
                 f",inv={self._last_sent.get(SURFACE_MAP_INV_PARAM[ch], 'NA')}"
                 for ch in SURFACE_MAP_CHANNELS]
        self.logs.write_event("SURFACE_MAP_APPLIED",
                              " ".join(parts[:2]), *parts[2:4])

    # WHY THE PID READBACK IS ASYNCHRONOUS
    #
    # cflib's param.set_value() does NOT block: it packs a packet and hands it to
    # _ParamUpdater's queue, which drains on its own thread one request at a time
    # (param.py:367). param.get_value() does NOT hit the radio at all: it is a
    # plain dict read of the locally cached TOC values (param.py:381), refreshed
    # only when the deck's echo arrives in _param_updated (param.py:215).
    #
    # So reading back immediately after writing reads the cache as it was BEFORE
    # the writes -- 12 round trips still queued. The comparison then always fails
    # and the warning always fires, which is exactly the "mismatch on first
    # connection and on every gain change" symptom. The warning was crying wolf;
    # the gains were landing fine.
    #
    # The fix cannot simply block until the echoes arrive, because _apply_pid_gains
    # is reachable from _handle_command, which is drained INSIDE the 100 Hz control
    # loop (_control_loop -> _drain_commands). Waiting there for 12 radio round
    # trips would stall the setpoint stream for long enough to trip the firmware's
    # commander timeout -- turning a cosmetic log bug into an in-flight failsafe.
    #
    # Instead: write, then arm a pending-verification record and let the deck's
    # own echoes satisfy it. _pid_echo is filled by a pid_rate group callback on
    # cflib's rx thread; _pid_verify_tick (called from the control loop) does the
    # comparison and the logging, so self.logs stays owned by one thread.
    def _bind_pid_echo_callback(self) -> None:
        self.cf.param.add_update_callback(group="pid_rate", cb=self._pid_echo_cb)

    def _pid_echo_cb(self, complete_name: str, value: str) -> None:
        # Runs on cflib's incoming-packet thread: touch nothing but the dict.
        try:
            parsed = float(value)
        except (TypeError, ValueError):
            return
        with self._pid_echo_lock:
            self._pid_echo[complete_name] = parsed

    def _apply_pid_gains(self, gains: PidGains) -> None:
        # Verify against the deck's echo rather than trusting the writes:
        # pid_rate.* are PARAM_PERSISTENT, so a set that fails leaves the
        # previously STORED gains in force and the aircraft quietly flies the old
        # tune. PID_APPLIED is the record every clipped flight is annotated with
        # (clip_flights.CARRY_FORWARD_EVENTS), and a log that confidently states
        # the wrong gains is worse than one that states none -- it would send a
        # tuning session chasing a change that never took.
        #
        # Expected values are quantized to binary32 first. The deck stores each
        # gain as a C float and echoes that back, so a requested 0.0035 (a float64
        # here) comes home as 0.0035000001080334187. Comparing the raw request
        # would report a "mismatch" on any gain not exactly representable in
        # single precision -- true of most ki/kd values, though not of the current
        # all-integer tune, which is why this has not bitten yet. Quantizing makes
        # the comparison exact in both directions with no epsilon to tune:
        # cflib caches str(float) and Python's float repr round-trips exactly.
        expected = {}
        for axis in PID_AXES:
            for term in PID_TERMS:
                want = float(getattr(getattr(gains, axis), term))
                expected[f"pid_rate.{axis}_{term}"] = struct.unpack(
                    "<f", struct.pack("<f", want))[0]
        # Clear the stale echoes BEFORE writing, not after. The initial TOC
        # download leaves an echo cached for every param, and those pre-write
        # values would satisfy the check instantly and verify nothing. Clearing
        # after the writes would be worse still: it could discard a genuine echo
        # that had already come home, and then the run could only ever time out.
        with self._pid_echo_lock:
            for name in expected:
                self._pid_echo.pop(name, None)
        self._pid_verify = (expected, time.monotonic() + PID_VERIFY_TIMEOUT_S)
        for name in expected:
            axis, term = name.split(".", 1)[1].rsplit("_", 1)
            self.cf.param.set_value(name, getattr(getattr(gains, axis), term))
        self._log("PID gains sent; awaiting readback.\n")

    def _pid_verify_tick(self) -> None:
        """Finish the PID readback once the deck has echoed the writes.

        Called every control-loop tick; a dict identity check and a length
        compare when idle, so it costs nothing. Emits PID_APPLIED either when all
        twelve echoes are in or when the deadline expires -- a timeout still
        writes the record, with the un-echoed terms marked nan, because silence
        about the gains is the one outcome the analysis layer cannot work with.
        """
        if self._pid_verify is None:
            return
        expected, deadline = self._pid_verify
        with self._pid_echo_lock:
            got = {name: self._pid_echo.get(name) for name in expected}
        missing = [name for name, value in got.items() if value is None]
        timed_out = time.monotonic() > deadline
        if missing and not timed_out:
            return
        self._pid_verify = None

        def shown(name: str) -> float:
            value = got[name]
            return float("nan") if value is None else value

        # Terms are labelled so a log stays readable next to older sessions,
        # which recorded a bare (kp,ki,kd) triple and no feed-forward term.
        self.logs.write_event(
            "PID_APPLIED",
            *(f"{axis}(" + ",".join(f"{t}={shown(f'pid_rate.{axis}_{t}')}"
                                    for t in PID_TERMS) + ")"
              for axis in PID_AXES),
        )
        if missing:
            self._log("[pid] WARNING: no readback within "
                      f"{PID_VERIFY_TIMEOUT_S:.1f}s for: "
                      f"{', '.join(n.split('.', 1)[1] for n in missing)}\n")
        mismatched = [name for name, want in expected.items()
                      if got[name] is not None and got[name] != want]
        if mismatched:
            self._log("[pid] WARNING: the deck reports different gains than were "
                      "requested for: "
                      f"{', '.join(n.split('.', 1)[1] for n in mismatched)}\n")
        if not missing and not mismatched:
            self._log("PID gains applied and confirmed by the deck.\n")

    def _create_log_configs(self) -> None:
        lg_controller = LogConfig(name="Controller", period_in_ms=self.config.period_controller_ms)
        for v in ("controller.r_roll", "controller.r_pitch", "controller.r_yaw",
                  "controller.pitchRate", "controller.rollRate", "controller.yawRate"):
            lg_controller.add_variable(v, "float")

        # 4 x uint16 = 8 of the 26 payload bytes, so servo.angle below is free:
        # the firmware sends one packet per block per period regardless of size.
        lg_motor = LogConfig(name="Motor", period_in_ms=self.config.period_motor_ms)
        for v in ("motor.m4", "motor.m1", "motor.m2", "motor.m3"):
            lg_motor.add_variable(v, "uint16_t")

        # servo.angle is only present on firmware carrying the servo deck's
        # LOG_GROUP. Probe the TOC instead of adding it unconditionally, because
        # cf.log.add_config() raises on an unknown variable and that exception
        # would propagate out of _run() and abort the whole session -- losing
        # m1..m4 and every other block too. Degrading one column is the right
        # failure mode; losing all telemetry because the aircraft is running an
        # older build is not. The TOC is already fetched here: _create_log_configs
        # is called from inside the SyncCrazyflie context.
        self.log_has_servo_angle = False
        try:
            if self.cf.log.toc.get_element_by_complete_name("servo.angle") is not None:
                lg_motor.add_variable("servo.angle", "uint16_t")
                self.log_has_servo_angle = True
        except Exception:
            pass
        if not self.log_has_servo_angle:
            self._log("[log] servo.angle not in the deck's TOC -- logging servo_cmd "
                      "(host intent) only. Flash firmware with the servo LOG_GROUP "
                      "to measure host->deck command latency.\n")

        lg_connection = LogConfig(name="Connection", period_in_ms=self.config.period_connection_ms)
        lg_connection.add_variable("radio.rssi", "uint8_t")
        lg_connection.add_variable("pm.vbat", "float")

        # This block carries the EKF attitude estimate alongside raw acceleration.
        # They share one block deliberately: the firmware sends ONE CRTP packet per
        # block per period regardless of payload (log.c LOG_MAX_LEN = 26), so extra
        # variables in an existing block cost zero extra radio packets while a new
        # block would cost a full packet per period. 3 floats (acc) + 3 floats
        # (attitude) = 24 of the 26 available bytes, so this block is now FULL --
        # adding a 7th variable here will fail with E2BIG in the firmware.
        # stateEstimate.* is the estimator output in DEGREES (note: its pitch is
        # inverted, legacy CF2 body frame), distinct from controller.*Rate.
        lg_accel = LogConfig(name="Accelerometer", period_in_ms=self.config.period_accelerometer_ms)
        for v in ("acc.x", "acc.y", "acc.z",
                  "stateEstimate.roll", "stateEstimate.pitch", "stateEstimate.yaw"):
            lg_accel.add_variable(v, "float")

        self.log_configs = {
            "controller": lg_controller,
            "motor": lg_motor,
            "connection": lg_connection,
            "accelerometer": lg_accel,
        }
        self.log_enabled = {
            "controller": self.config.log_controller,
            "motor": self.config.log_motor,
            "connection": self.config.log_connection,
            "accelerometer": self.config.log_accelerometer,
        }

    def _log_cb(self, fn):
        """Wrap a log data callback with arrival stamping and an exception guard.

        The guard is not belt-and-braces. cflib's Caller.call() iterates its
        callbacks with no per-callback try/except (utils/callbacks.py), so one
        raising callback aborts the rest of the chain for that packet; the
        exception surfaces in _IncomingPacketHandler, which reports it with
        logger.error and drops the packet. This process never configures
        logging, so such a traceback goes to stderr only and is invisible in the
        GUI's log pane -- telemetry would appear to half-die for no stated
        reason. Catching here keeps one bad block from taking the others down and
        puts the reason somewhere the pilot will actually see it.

        Stamping happens BEFORE the body, so a callback that raises every time
        still counts as "log data is arriving" and does not trigger the block
        recovery below -- re-creating the block would not fix a host-side bug.
        """
        name = getattr(fn, "__name__", "log_cb")

        def wrapped(timestamp, data, logconf):
            self._last_log_rx_at = time.monotonic()
            try:
                fn(timestamp, data, logconf)
            except Exception as exc:
                # Rate-limited: at 10 ms periods an unguarded report would flood
                # the pane faster than it could be read.
                now = time.monotonic()
                if now - self._log_cb_errors.get(name, 0.0) > 5.0:
                    self._log_cb_errors[name] = now
                    self._log(f"[log] {name} raised: {exc!r} (further reports "
                              "from this callback suppressed for 5s)\n")

        return wrapped

    def _bind_log_callbacks(self) -> None:
        for key, handler in (("controller", self._on_controller_log),
                             ("motor", self._on_motor_log),
                             ("connection", self._on_connection_log),
                             ("accelerometer", self._on_accel_log)):
            self.cf.log.add_config(self.log_configs[key])
            self.log_configs[key].data_received_cb.add_callback(self._log_cb(handler))

    def _start_enabled_logs(self) -> None:
        for key, enabled in self.log_enabled.items():
            if enabled:
                self.log_configs[key].start()
        time.sleep(0.1)
        # Start the stall clock only now, so the time spent connecting and
        # fetching the TOC is not mistaken for a stall on the first tick.
        self._last_log_rx_at = time.monotonic()

    def _log_recovery_tick(self) -> None:
        """Re-create the deck's log blocks if log data has stopped arriving.

        See LOG_STALL_TIMEOUT_S for why the deck silently deletes them. Recovery
        has to force added = False before calling start(): cflib's start() only
        sends CREATE_BLOCK when it believes the block is absent, and otherwise
        sends START_LOGGING for an id the firmware no longer has. The deck's
        CREATE_BLOCK reply is what then re-sends START_LOGGING, and that handler
        is itself gated on `if not block.added` (crazyflie/log.py), so leaving
        the stale True in place means nothing restarts.

        add_config() is deliberately NOT called again: it re-resolves
        default_fetch_as and would append every variable to the config a second
        time. The existing LogConfig objects are reused as-is.

        Correct whether or not the deck actually dropped the blocks: if they
        still exist, CREATE_BLOCK returns EEXIST, which cflib treats as success
        and follows with START_LOGGING anyway.
        """
        if self._last_log_rx_at == 0.0 or self.cf is None:
            return
        if not any(self.log_enabled.values()):
            return
        now = time.monotonic()
        if (now - self._last_log_rx_at) <= LOG_STALL_TIMEOUT_S:
            return
        if self._log_recovery_at and (now - self._log_recovery_at) < LOG_RECOVERY_COOLDOWN_S:
            return
        self._log_recovery_at = now
        self._log_recovery_count += 1
        restarted = []
        for key, enabled in self.log_enabled.items():
            if not enabled:
                continue
            conf = self.log_configs[key]
            try:
                conf.started = False
                conf.added = False
                conf.start()
                restarted.append(key)
            except Exception as exc:
                self._log(f"[log] could not restart the {key} block: {exc}\n")
        if restarted:
            self._log(f"[log] no log data for {LOG_STALL_TIMEOUT_S:.1f}s -- "
                      f"re-creating blocks: {', '.join(restarted)} "
                      f"(attempt {self._log_recovery_count})\n")
            if self.logs is not None:
                self.logs.write_event("LOG_BLOCKS_RESTARTED",
                                      ",".join(restarted),
                                      self._log_recovery_count)

    # ----- log callbacks (run on cflib threads) ---------------------------- #
    def _on_controller_log(self, timestamp, data, _logconf):
        ts = timestamp / 1000.0
        gr = float(data["controller.r_roll"]); gp = float(data["controller.r_pitch"])
        gy = float(data["controller.r_yaw"]); sp = float(data["controller.pitchRate"])
        sr = float(data["controller.rollRate"]); sy = float(data["controller.yawRate"])
        # controller.r_* is rad/s; controller.*Rate is deg/s (see flight_plots
        # .RAD2DEG). The live axes are labelled deg/s, so the gyro side -- and
        # only the gyro side -- is converted before plotting. The CSV keeps the
        # raw firmware values so the on-disk format is unchanged and old logs
        # stay comparable to new ones; flight_plots does the same conversion at
        # load time.
        with self._lock:
            override = self.live.manual_override
        if override:
            # Under manual override the commanded-rate log variables do not hold
            # pilot commands: the override packet bypasses the controller, and on
            # firmware without the modeVelocity fix (crtp_commander_generic.c,
            # manualMotorDecoder) the zeroed setpoint reads as modeDisable, i.e.
            # "hold 0 deg attitude". The attitude PID then runs against level and
            # its unclamped output lands in controller.rollRate/pitchRate, which
            # is what these traces plot. Blanking them keeps the shared deg/s axes
            # scaled to the gyro, which is the signal worth watching here.
            #
            # NaN rather than a dropped sample: the setpoint traces share t_ctrl
            # with the gyro traces, so skipping would misalign every later point.
            # matplotlib renders NaN as a gap, which also makes the override span
            # visible instead of forging a flat zero line.
            # The CSV still gets the raw firmware values: blanking is a display
            # decision, and flight_plots masks these spans at load time using the
            # MANUAL_OVERRIDE breakpoints, so the on-disk format stays unchanged
            # and old logs stay comparable to new ones.
            psr = psp = psy = float("nan")
        else:
            psr, psp, psy = sr, sp, sy
        self.buffers.add_controller(
            ts,
            gr * flight_plots.RAD2DEG, gp * flight_plots.RAD2DEG, gy * flight_plots.RAD2DEG,
            psr, psp, psy,
        )
        # Host-side injection snapshot (see GliderWorker._inj_cmd): read without
        # a lock because the worker rebinds the whole tuple at once.
        inj_r, inj_p, inj_y = self._inj_cmd
        self.logs.controller.writerow([ts, gr, gp, gy, sr, sp, sy,
                                       round(inj_r, 3), round(inj_p, 3), round(inj_y, 3)])

    def _on_motor_log(self, timestamp, data, _logconf):
        ts = timestamp / 1000.0
        m4 = float(data["motor.m4"]); m1 = float(data["motor.m1"])
        m2 = float(data["motor.m2"]); m3 = float(data["motor.m3"])
        sa = float(data["servo.angle"]) if self.log_has_servo_angle else float("nan")
        self.buffers.add_motor(ts, m1, m2, m3, m4, float(self.last_servo_value))
        self.logs.motor.writerow([ts, m4, m1, m2, self.last_servo_value, m3, sa])

    def _on_connection_log(self, timestamp, data, _logconf):
        ts = timestamp / 1000.0
        rssi = float(data["radio.rssi"]); vbat = float(data["pm.vbat"])
        self.last_connection_seen_at = time.monotonic()
        self.buffers.add_connection(ts, rssi)
        self.logs.connection.writerow([ts, rssi, vbat])
        self.telemetry.emit(vbat, rssi)

    def _on_accel_log(self, timestamp, data, _logconf):
        ts = timestamp / 1000.0
        ax = float(data["acc.x"]); ay = float(data["acc.y"]); az = float(data["acc.z"])
        er = float(data["stateEstimate.roll"]); ep = float(data["stateEstimate.pitch"])
        ey = float(data["stateEstimate.yaw"])
        self.buffers.add_accel(ts, ax, ay, az, er, ep, ey)
        self.logs.accelerometer.writerow([ts, ax, ay, az, er, ep, ey])

    # ----- connection failsafe --------------------------------------------- #
    def _bind_connection_callbacks(self) -> None:
        self.cf.connection_lost.add_callback(self._on_link_issue)
        self.cf.connection_failed.add_callback(self._on_link_issue)
        self.cf.disconnected.add_callback(self._on_link_issue)

    def _on_link_issue(self, link_uri, message: str = "") -> None:
        self._failsafe_disarm(f"LINK_ISSUE uri={link_uri} msg={message}")

    def _failsafe_disarm(self, reason: str) -> None:
        if self.failsafe_active:
            return
        self.failsafe_active = True
        self.update_live(motor_armed=False, autonomous=False, throttle=0.0, manual_override=False)
        # Stop injecting and DISARM the injector, not just abort the run. A
        # failsafe means the link or the aircraft is in an unknown state; the
        # pilot should have to consciously re-arm before the next perturbation.
        self.injector.abort("FAILSAFE", time.monotonic())
        self.injector.set_armed(False, time.monotonic())
        self._inj_cmd = (0.0, 0.0, 0.0)
        if self.logs is not None:
            self.logs.write_breakpoint("FAILSAFE_DISARM")
            self.logs.write_event("FAILSAFE_DISARM", reason)
        try:
            if self.commander is not None:
                self.commander.send_setpoint(0.0, 0.0, 0.0, 0)
                self.commander.send_stop_setpoint()
        except Exception:
            pass
        # Cut the ESC on the streamed route too, and FIRST: it is the fast one, so
        # it is the one that can still be carrying throttle. Letting the firmware's
        # fwProp.timeoutMs expire would also stop the motor, but only after the
        # timeout -- an explicit zero stops it on the next control cycle. Sent
        # unconditionally rather than under `if self.propulsion_fast` so that
        # flipping the route mid-flight cannot leave a stale streamed command as
        # the freshest thing the firmware has seen.
        self._send_propulsion_packet(0)
        self.last_servo_value = 0
        try:
            if self.cf is not None:
                self.cf.param.set_value("motorPowerSet.enable", "0")
                self.cf.param.set_value("servo.servoAngle", "0")
                self.cf.platform.send_arming_request(False)
        except Exception:
            pass
        self._log(f"Failsafe disarm engaged: {reason}\n")
        self.status.emit("FAILSAFE")

    # ----- main control loop ----------------------------------------------- #
    def _control_loop(self) -> None:
        while not self._stop.is_set():
            # Refresh SDL joystick state once per tick, unconditionally, so the
            # controller is read every loop regardless of which mode branch runs
            # or whether the debug echo is enabled. (Previously each handler
            # pumped on its own, which coupled reads to code paths.)
            # Use get() (not bare pump()) to DRAIN the event queue: a moving
            # stick floods the queue with JOYAXISMOTION events, and if it's
            # never emptied SDL stops updating and axis reads freeze at their
            # last value (buttons, being rare, still sneak through).
            if self.joystick is not None:
                pygame.event.get()

            self._drain_commands()
            self._pid_verify_tick()
            self._log_recovery_tick()
            self._emit_button_state()
            self._emit_axes_state()

            if (
                self.log_enabled.get("connection", False)
                and self.last_connection_seen_at > 0.0
                and (time.monotonic() - self.last_connection_seen_at) > CONNECTION_WATCHDOG_TIMEOUT_S
            ):
                self._failsafe_disarm("CONNECTION_TELEMETRY_TIMEOUT")

            # Read all discrete controller inputs (buttons/latches/tune modes)
            # up front, before the flight-mode branch, so switches (including the
            # servo-PWM/rate-setpoint override latch itself) are honoured every
            # tick regardless of which continuous path runs below.
            if self.config.use_controller and self.joystick is not None:
                self._poll_controller_buttons(self._live_snapshot())

            live = self._live_snapshot()

            if self.config.debug_controller_log and (
                (live.manual_override and live.override_with_controller)
                or self.config.use_controller
            ):
                self._debug_log_controller(live)

            if live.manual_override:
                self._drive_manual_override(live)
                self._feed_supervisor_keepalive()
                # Setpoints are bypassed in override, so nothing ticks the
                # injector down this branch -- stop any run (see _injection_tick).
                self._injection_tick()
            elif self.config.use_controller:
                self._handle_controller_flight(live)
            else:
                # GUI-driven setpoints (no controller): autonomous or hold-zero.
                # Hold-zero still routes through _send_rate_setpoint so a
                # maneuver can be injected on the bench with no stick attached,
                # which is how you verify the feature before flying it.
                if live.autonomous:
                    self._send_rate_setpoint(live.setpoint_roll, live.setpoint_pitch,
                                             live.setpoint_yaw)
                else:
                    self._send_rate_setpoint(0.0, 0.0, 0.0)
                if live.motor_armed:
                    self._set_bl_motor_throttle(live.throttle)

            time.sleep(0.01)

    def _emit_button_state(self) -> None:
        """Publish which physical buttons are down, for the Mapping tab's live
        indicator and its Learn function.

        Change-driven rather than rate-driven: a press or release emits on the very
        next tick (~10 ms, so Learn feels instant), while a controller sitting still
        costs one emit every half second. The heartbeat exists so a tab opened after
        the fact still gets a starting picture instead of waiting for a press.
        """
        if self.joystick is None:
            self._last_buttons_sent = None
            return
        try:
            n = self.joystick.get_numbuttons()
            pressed = frozenset(i for i in range(n) if self.joystick.get_button(i))
        except Exception:
            # A controller yanked mid-flight raises here; the reconnect path owns
            # recovery, this display is not worth propagating an exception for.
            return
        now = time.monotonic()
        if pressed == self._last_buttons_sent and (now - self._last_buttons_emit) < 0.5:
            return
        self._last_buttons_sent = pressed
        self._last_buttons_emit = now
        self.buttons_state.emit((self._live_snapshot().motor_armed, n, pressed))

    def _emit_axes_state(self) -> None:
        """Publish raw axis values for the Mapping tab's live readout and its
        axis detection.

        Fixed ~25 Hz rather than change-driven: unlike buttons, an axis is never
        still (noise moves the last decimal constantly), so a change filter would
        emit on every tick and save nothing. 25 Hz is smooth to watch and gives
        detection ~100 samples over its capture window -- ample for a peak search.
        """
        if self.joystick is None:
            return
        now = time.monotonic()
        if (now - self._last_axes_emit) < 0.04:
            return
        self._last_axes_emit = now
        try:
            values = tuple(self.joystick.get_axis(i)
                           for i in range(self.joystick.get_numaxes()))
        except Exception:
            return
        self.axes_state.emit((self._live_snapshot().motor_armed, values))

    def _feed_supervisor_keepalive(self) -> None:
        """Keep the firmware supervisor in a motors-allowed state during manual
        override. The override path sends no commander setpoints, so without this
        the setpoint watchdog (COMMANDER_WDT_TIMEOUT_SHUTDOWN, 2 s) fires and the
        supervisor blocks the commander (crtpCommanderBlock). The watchdog checks
        setpoint *age*, not value, so a zero setpoint is enough to pet it; it does
        not fight the override because motorPowerSet.enable overrides the ratios."""
        try:
            self.commander.send_setpoint(0.0, 0.0, 0.0, 0)
            now = time.monotonic()
            if now - self._last_arm_request > 1.0:
                self.cf.platform.send_arming_request(True)
                self._last_arm_request = now
        except Exception:
            pass

    def _drain_commands(self) -> None:
        while True:
            try:
                action, payload = self._cmd_queue.get_nowait()
            except queue.Empty:
                return
            self._handle_command(action, payload)

    def _handle_command(self, action: str, payload: object) -> None:
        if action == "arm":
            # Safety interlock: refuse to arm unless the thrust input is at idle,
            # so arming can never coincide with a live throttle command.
            thrust_in = self._current_thrust_input()
            if thrust_in > ARM_THROTTLE_DEADBAND:
                self.logs.write_event("MOTOR_ARM_REJECTED", round(thrust_in, 3))
                self._log(f"Arm REJECTED: throttle must be at idle (0); it is "
                          f"{thrust_in:.2f}. Lower the thrust stick and re-arm.\n")
                return
            # Arm only enables the thrust axis; it does NOT spin the motor. The
            # motor stays at whatever the throttle input commands (idle == off).
            self.update_live(motor_armed=True)
            self.logs.write_breakpoint("FLIGHT_START_MOTOR_ARM")
            self.logs.write_event("MOTOR_ARM")
            self._log("Motor armed (throttle idle; advance the thrust stick to spin up).\n")
        elif action == "disarm":
            self.update_live(motor_armed=False, throttle=0.0)
            self._set_bl_motor_throttle(0.0)
            self.logs.write_breakpoint("FLIGHT_END_MOTOR_DISARM")
            self.logs.write_event("MOTOR_DISARM")
            self._log("Motor disarmed.\n")
        elif action == "breakpoint":
            self.logs.write_breakpoint(str(payload) if payload else "MANUAL_BREAKPOINT")
            self._log("Breakpoint written.\n")
        elif action == "persist_pid":
            self._persist_pid_gains()
        elif action == "apply_pid":
            self.config.gains = payload
            self._apply_pid_gains(payload)
            self.logs.write_breakpoint("PID_UPDATED")
        elif action == "autonomous":
            self.update_live(autonomous=bool(payload))
            self.logs.write_breakpoint("AUTONOMOUS_ENABLED" if payload else "AUTONOMOUS_DISABLED")
        elif action == "set_trim":
            axis, value = payload
            param = TRIM_PARAMS[axis]
            trim = int(clamp(int(value), 0, MAX_MOTOR_CMD))
            self._set_param_if_changed(param, trim)
            self.logs.write_event("TRIM_SET", axis, trim)
            self._write_trim_state()
        elif action == "persist_trim":
            self._persist_trim()
        elif action == "set_surface_map":
            channel, surf, invert = payload
            surf = int(clamp(int(surf), 0, 3))
            invert = 1 if invert else 0
            self._set_param_if_changed(SURFACE_MAP_SURF_PARAM[channel], surf)
            self._set_param_if_changed(SURFACE_MAP_INV_PARAM[channel], invert)
            self.logs.write_event("SURFACE_MAP_SET", channel, f"surface={surf}", f"invert={invert}")
            self._write_surface_map_state()
        elif action == "persist_surface_map":
            self._persist_surface_map()
        elif action == "manual_override":
            enabled = bool(payload)
            self.update_live(manual_override=enabled)
            # Clean transition: start from a known-zero state and force the next
            # writes through (clear the dirty-check cache) so there is no jump.
            self._last_sent.clear()
            self._override_servo_cmd = 0
            self._last_override_emit = None
            # motorPowerSet.enable routes the *param* override into setMotorRatios.
            # The commander-fast path instead feeds the streamed values through the
            # normal motor pipeline, so it needs enable=0 (a non-zero enable would
            # let the param values fight the streamed ones). Only the legacy path
            # switches it on.
            param_enable = "1" if (enabled and not self._fast_override) else "0"
            self.cf.param.set_value("motorPowerSet.enable", param_enable)
            self.override_state.emit(0, 0, 0, 0, 0)
            self.logs.write_breakpoint("MANUAL_OVERRIDE_ON" if enabled else "MANUAL_OVERRIDE_OFF")
            mode = "fast/streamed" if self._fast_override else "param"
            self._log(f"Manual override {'ON' if enabled else 'OFF'} ({mode}).\n")
        elif action == "fast_override":
            self._fast_override = bool(payload)
            self._log(f"Manual override mode: "
                      f"{'fast/streamed' if self._fast_override else 'param (legacy)'}.\n")
        elif action == "reconnect_controller":
            self._reconnect_controller()
        elif action == "set_controller_map":
            # Runtime remap. This arrives through the same queue as every other
            # GUI->worker change, which is the whole point: the control loop reads
            # profile.buttons ~100 times a second, and applying the new dict here
            # (on the worker thread, between ticks) means neither side needs a lock.
            if payload is None:
                # "Revert to built-in": rebuild the profile from the module-level
                # template rather than trying to undo the edits in place.
                base = CONTROLLER_PROFILES.get(self.config.controller_type, XBOX_PROFILE)
                self.profile = ControllerProfile(
                    **{**asdict(base), "buttons": dict(base.buttons)})
                self.last_button_state.clear()
                self._latch_prev.clear()
                self._log("Button map reverted to the built-in layout.\n")
            else:
                self._apply_controller_map(payload)
                self._log(f"Button map '{payload.name}' applied.\n")
            self.logs.write_event("CONTROLLER_MAP_APPLIED",
                                  payload.name if payload is not None else "built-in")
        elif action == "maneuver_library":
            # The GUI owns the editable library; the worker gets a sanitized
            # copy by value. Nothing on the control loop ever reads a widget,
            # which is the rule that keeps the two threads independent.
            maneuvers, selected = payload
            self.injector.set_library(maneuvers, selected)
            self._emit_injector_state()
        elif action == "maneuver_select":
            self.injector.set_library(self.injector.library, int(payload))
            self._emit_injector_state()
        elif action == "inject_advance":
            self.injector.advance = bool(payload)
        elif action == "inject_stick_abort":
            self._inj_stick_abort = bool(payload)
        elif action == "propulsion_fast":
            want = bool(payload)
            if want != self.propulsion_fast:
                # Zero the route being left behind before switching, so a stale
                # command on it cannot be what the firmware falls back to.
                if self.propulsion_fast:
                    self._send_propulsion_packet(0)
                else:
                    self._set_param_if_changed("servo.servoAngle", 0)
                self.last_servo_value = 0
                self._last_throttle_write = None
                self.propulsion_fast = want
                route = "streamed packet" if want else "servoAngle param"
                if self.logs is not None:
                    self.logs.write_event("PROPULSION_ROUTE", route)
                self._log(f"Propulsion throttle route: {route}\n")
        elif action == "inject_arm":
            self.injector.set_armed(bool(payload), time.monotonic())
            self._drain_injector_events()
            self._log(f"Maneuver injector {'ARMED' if payload else 'disarmed'}.\n")
            self._emit_injector_state()
        elif action == "inject_fire":
            now = time.monotonic()
            man = self.injector.selected_maneuver()
            # Re-run the full gate here as well as in the tick: refusing to
            # START is much better than starting and aborting one tick later,
            # and it gives the pilot a reason instead of a silent no-op.
            blocked = self._injection_block_reason(
                self._live_snapshot(), man.axis if man is not None else None)
            if blocked is not None:
                self.logs.write_event("MANEUVER_REJECTED", blocked,
                                      man.name if man is not None else "")
                self._log(f"Maneuver REJECTED: {blocked}.\n")
            else:
                ok, message = self.injector.trigger(now)
                if not ok:
                    self.logs.write_event("MANEUVER_REJECTED", message,
                                          man.name if man is not None else "")
                self._log(f"{'Maneuver: ' if ok else 'Maneuver REJECTED: '}{message}\n")
                self._drain_injector_events()
            self._emit_injector_state()
        elif action == "inject_abort":
            if self.injector.abort("PILOT_ABORT", time.monotonic()):
                self._log("Maneuver aborted.\n")
            self._inj_cmd = (0.0, 0.0, 0.0)
            self._drain_injector_events()
            self._emit_injector_state()
        elif action == "event":
            self.logs.write_event(*payload)

    # ----- direct parameter-system override -------------------------------- #
    def _drive_manual_override(self, live: LiveControl) -> None:
        """Bypass the flight controller and command motors/servo directly.

        Runs every control-loop tick (~100 Hz). To avoid saturating the radio
        link (the cause of the lag/sputter) every write is dirty-checked and the
        servo is slewed one non-blocking step per tick instead of ramped inline.
        """
        if live.override_with_controller and self.joystick is not None:
            # (SDL state is pumped once per tick at the top of the control loop.)
            # Surface mapping (axis indices come from the active
            # ControllerProfile, so this works for both Xbox and RC):
            #   M4 = roll/aileron   <- roll axis   (aileron moved to M4)
            #   M2 = pitch/elevator <- pitch axis (inverted)
            #   M3 = yaw/rudder     <- yaw axis
            # Each surface is bidirectional so it sits at mid-scale +/- input.
            # Throttle axis -> servo; M1 stays slider-driven.
            p = self.profile
            roll = self._axis_c(p.roll_axis, p.roll_sign)
            pitch = self._axis_c(p.pitch_axis, p.pitch_sign)
            yaw = self._axis_c(p.yaw_axis, p.yaw_sign)
            roll = 0.0 if abs(roll) < JOYSTICK_DEADBAND else roll
            pitch = 0.0 if abs(pitch) < JOYSTICK_DEADBAND else pitch
            yaw = 0.0 if abs(yaw) < JOYSTICK_DEADBAND else yaw
            mid = MAX_MOTOR_CMD // 2
            m1 = int(live.override_m1)          # M1 stays slider-controlled
            m2 = int(clamp(mid - pitch * mid, 0, MAX_MOTOR_CMD))
            m3 = int(clamp(mid + yaw * mid, 0, MAX_MOTOR_CMD))
            m4 = int(clamp(mid + roll * mid, 0, MAX_MOTOR_CMD))
            servo_target = int(clamp(self._throttle_norm(p) * MAX_MOTOR_CMD, 0, MAX_MOTOR_CMD))
        else:
            # Slider-driven direct command.
            m1, m2 = int(live.override_m1), int(live.override_m2)
            m3, m4 = int(live.override_m3), int(live.override_m4)
            servo_target = int(live.override_servo)

        # Arm/disarm governs propulsion even in manual override, for safety: while
        # disarmed the throttle/servo is forced to zero immediately (bypassing the
        # slew ramp for an instant cut), but the control surfaces (M1-M4) stay live
        # so the aircraft is still steerable. Arming again lets the throttle ramp
        # back up smoothly.
        armed = live.motor_armed
        if not armed:
            servo_target = 0
            self._override_servo_cmd = 0
        else:
            # Non-blocking slew toward the servo target so the loop never stalls.
            self._override_servo_cmd = self._slew(
                self._override_servo_cmd, servo_target, OVERRIDE_SERVO_SLEW)

        if self._fast_override:
            # Commander-fast path: stream the raw command on the setpoint channel
            # every tick. It is fire-and-forget (no per-packet ACK), so there is no
            # backlog to coalesce against -- newest packet always wins and the
            # firmware applies it within one 1 kHz stabilizer loop. The firmware
            # auto-expires the stream (fwManual.timeoutMs) and cuts propulsion if
            # it goes stale, so a disarm (servo forced to 0 above) stops the motor
            # on the very next streamed packet.
            self._send_manual_motor_packet(m1, m2, m3, m4, self._override_servo_cmd)
        else:
            # Legacy param path: coalesce the writes to OVERRIDE_WRITE_INTERVAL so
            # the radio link carries the latest command instead of a growing
            # backlog (the cause of the sluggish/laggy servo feel). A disarm always
            # cuts the throttle now, regardless of the write timer.
            now = time.monotonic()
            due = (now - self._last_override_write) >= OVERRIDE_WRITE_INTERVAL
            if due:
                self._set_param_if_changed("motorPowerSet.m1", m1)
                self._set_param_if_changed("motorPowerSet.m2", m2)
                self._set_param_if_changed("motorPowerSet.m3", m3)
                self._set_param_if_changed("motorPowerSet.m4", m4)
                self._set_param_if_changed("servo.servoAngle", self._override_servo_cmd)
                self._last_override_write = now
            elif not armed:
                # Safety: never defer cutting propulsion behind the coalescing timer.
                self._set_param_if_changed("servo.servoAngle", 0)
        self.last_override_servo = self._override_servo_cmd
        self.last_servo_value = self._override_servo_cmd

        # Mirror the live command on the GUI sliders (controller mode only, and
        # only when it changed, to keep cross-thread signal traffic light).
        if live.override_with_controller and self.joystick is not None:
            state = (m1, m2, m3, m4, self._override_servo_cmd)
            if state != self._last_override_emit:
                self._last_override_emit = state
                self.override_state.emit(*state)

    @staticmethod
    def _slew(current: int, target: int, max_step: int) -> int:
        target = int(clamp(target, 0, MAX_MOTOR_CMD))
        if abs(target - current) <= max_step:
            return target
        return current + max_step * (1 if target > current else -1)

    def _send_manual_motor_packet(self, m1: int, m2: int, m3: int, m4: int,
                                  servo: int) -> None:
        """Stream one commander-fast manual-override packet: raw M1-M4 + propulsion
        (servo) on the generic setpoint channel. Matches manualMotorPacket_s in the
        firmware: one type byte then five little-endian uint16 (0..UINT16_MAX)."""
        if self.cf is None:
            return
        pk = CRTPPacket()
        pk.port = CRTPPort.COMMANDER_GENERIC
        pk.channel = GENERIC_SETPOINT_CHANNEL
        pk.data = struct.pack(
            "<BHHHHH", MANUAL_MOTOR_SETPOINT_TYPE,
            int(clamp(m1, 0, MAX_MOTOR_CMD)),
            int(clamp(m2, 0, MAX_MOTOR_CMD)),
            int(clamp(m3, 0, MAX_MOTOR_CMD)),
            int(clamp(m4, 0, MAX_MOTOR_CMD)),
            int(clamp(servo, 0, MAX_MOTOR_CMD)),
        )
        try:
            self.cf.send_packet(pk)
        except Exception:
            pass

    def _send_propulsion_packet(self, esc: int) -> None:
        """Stream one propulsion (ESC) throttle command on the meta-command
        channel. Matches propulsionSetPacket in the firmware: one type byte then a
        single little-endian uint16 (0..UINT16_MAX).

        Fire-and-forget, like the rate setpoints and unlike a param write, so
        this can be called every control-loop tick without building a backlog.
        That is the whole point: the surfaces always had a streaming path and the
        ESC did not.
        """
        if self.cf is None:
            return
        pk = CRTPPacket()
        pk.port = CRTPPort.COMMANDER_GENERIC
        pk.channel = META_COMMAND_CHANNEL
        pk.data = struct.pack("<BH", META_PROPULSION_TYPE,
                              int(clamp(esc, 0, MAX_MOTOR_CMD)))
        try:
            self.cf.send_packet(pk)
        except Exception:
            pass

    def _set_param_if_changed(self, name: str, value: int) -> None:
        value = int(value)
        if self._last_sent.get(name) != value:
            self.cf.param.set_value(name, value)
            self._last_sent[name] = value

    def _persist_trim(self) -> None:
        """Save the per-surface trims to the deck's flash (PARAM_PERSISTENT) so
        they survive a power cycle. persistent_store is async in cflib."""
        def _done(complete_name, success):
            self._log(f"Trim {'saved' if success else 'SAVE FAILED'}"
                      f" ({complete_name}).\n")
        for param in TRIM_PARAMS.values():
            try:
                self.cf.param.persistent_store(param, _done)
            except Exception as exc:
                self._log(f"Trim persist error ({param}): {exc}\n")
        self.logs.write_event("TRIM_PERSIST_REQUEST")

    def _persist_surface_map(self) -> None:
        """Save the servo mixer map (surface assignment + invert per channel) to
        the deck's flash so it survives a power cycle."""
        def _done(complete_name, success):
            self._log(f"Surface map {'saved' if success else 'SAVE FAILED'}"
                      f" ({complete_name}).\n")
        for ch in SURFACE_MAP_CHANNELS:
            for param in (SURFACE_MAP_SURF_PARAM[ch], SURFACE_MAP_INV_PARAM[ch]):
                try:
                    self.cf.param.persistent_store(param, _done)
                except Exception as exc:
                    self._log(f"Surface map persist error ({param}): {exc}\n")
        self.logs.write_event("SURFACE_MAP_PERSIST_REQUEST")

    def _set_bl_motor_throttle(self, throttle: float) -> None:
        """Drive the propulsion ESC toward ``throttle`` (0..1), non-blocking.

        Two routes, selected by self.propulsion_fast:

        FAST (default) -- stream a propulsion meta-command every tick. The
        firmware applies it inside the stabilizer loop via servoSetAngleFast(),
        so the command reaches the ESC timer on the next control cycle. No slew
        limiting and no write coalescing here, because both of those exist only
        to protect the param channel: a fire-and-forget packet cannot build a
        backlog, and rate-limiting the host would just add lag back on top of
        hardware that no longer needs it. The ESC's own ramp is the authority on
        how fast the motor may spool.

        PARAM (fallback) -- the original servo.servoAngle param write, kept for
        stock firmware (which has no propulsion decoder) and so the two paths can
        be compared on a single flight. Everything below about slewing and
        coalescing applies to THIS route only.

        Called every tick of the ~100 Hz control loop, so the ramp is advanced a
        little per call (at THROTTLE_SERVO_SLEW_PER_S) instead of being run to
        completion here. The previous version looped to the target internally with
        a 5 ms sleep per step, which was the root cause of two separate faults:

          * it blocked the control loop for ~100 ms per throttle move, during
            which no setpoints were streamed and no controller input was read;
          * it emitted ~21 param writes per move. Param writes are NOT
            fire-and-forget: cflib serialises them one at a time and waits for the
            deck to echo each one back (param.py's _ParamUpdater: an unbounded
            FIFO, wait_lock, send_packet(expected_reply=...)). Feeding that queue
            faster than the radio round-trip makes it grow without bound, so
            commands arrive seconds late and keep arriving after the stick stops
            -- including after a disarm, because the backlog still has to drain.

        Writes are coalesced to OVERRIDE_WRITE_INTERVAL for the same reason the
        manual-override path coalesces its own: the link should always carry the
        freshest command rather than a queue of stale ones. A cut to zero bypasses
        the timer, so disarming still stops the motor on the very next tick.
        """
        target = int(clamp(throttle, 0.0, 1.0) * MAX_MOTOR_CMD)
        cutting = target == 0
        now = time.monotonic()
        if self.propulsion_fast:
            self._send_propulsion_packet(target)
            self._last_throttle_write = now
            self.last_servo_value = target
            return
        if self._last_throttle_write is None:
            elapsed = OVERRIDE_WRITE_INTERVAL   # first command since connect
        else:
            elapsed = now - self._last_throttle_write
            if not cutting and elapsed < OVERRIDE_WRITE_INTERVAL:
                return                  # too soon: leave last_servo_value alone so
                                        # the next due tick resumes from what the
                                        # deck was actually last told
        # Slew by the time actually elapsed, so the ramp rate is set by
        # THROTTLE_SERVO_SLEW_PER_S alone and not by how often this happens to run.
        # elapsed is capped at a little over one write interval so a stalled loop
        # (or a long gap since the last command) can only ever ramp the ESC by
        # about one write's worth per write -- never a single large throttle jump.
        # Capping makes a stall ramp SLOWER, which is the safe direction.
        step = int(THROTTLE_SERVO_SLEW_PER_S * min(elapsed, OVERRIDE_WRITE_INTERVAL * 1.5))
        value = target if cutting else self._slew(
            self.last_servo_value, target, max(step, 1))
        self._set_param_if_changed("servo.servoAngle", value)
        self._last_throttle_write = now
        self.last_servo_value = value

    # ----- controller input (manual / autonomous flight) ------------------- #
    def _axis(self, idx: int, sign: float = 1.0) -> float:
        """Read a joystick axis via the active profile. Returns 0.0 if the axis
        index is absent (-1 or beyond this device's axis count)."""
        if self.joystick is None or idx < 0 or idx >= self.joystick.get_numaxes():
            return 0.0
        return sign * self.joystick.get_axis(idx)

    def _axis_c(self, idx: int, sign: float = 1.0) -> float:
        """Read a control-surface axis (roll/pitch/yaw) with its captured rest
        offset removed, so a stick that idles slightly off-zero (the InterLink-X
        yaw rests at ~-0.06) still commands a true zero when centered. Throttle
        deliberately uses raw _axis() + its own calibration, not this."""
        if self.joystick is None or idx < 0 or idx >= self.joystick.get_numaxes():
            return 0.0
        return sign * (self.joystick.get_axis(idx) - self._axis_center(idx))

    def _throttle_norm(self, profile: "ControllerProfile") -> float:
        """Normalise the throttle input to 0..1.
        - Absolute throttle sticks (RC): linearly map the calibrated raw range
          [throttle_idle_raw .. throttle_full_raw] onto [0..1] so the stick's
          idle position reads a true 0 (motor off) and full-up reads 1.0. A small
          idle deadband absorbs jitter. Flip profile.throttle_sign to reverse.
        - Otherwise (Xbox baseline): use the magnitude of the stick deflection."""
        raw = self._axis(profile.throttle_axis, profile.throttle_sign)
        if profile.throttle_from_axis:
            span = profile.throttle_idle_raw - profile.throttle_full_raw
            if abs(span) < 1e-6:
                return 0.0
            norm = clamp((profile.throttle_idle_raw - raw) / span, 0.0, 1.0)
            return 0.0 if norm < THROTTLE_INPUT_DEADBAND else norm
        raw = 0.0 if abs(raw) < JOYSTICK_DEADBAND else raw
        return clamp(abs(raw), 0.0, 1.0)

    def _current_thrust_input(self) -> float:
        """The throttle the motor would follow right now, used by the arm
        interlock. For an axis-throttle controller read the stick fresh; else
        fall back to the live throttle value."""
        p = self.profile
        if (self.config.use_controller and self.joystick is not None
                and p.throttle_from_axis):
            return self._throttle_norm(p)
        with self._lock:
            return self.live.throttle

    def _edge_cmd(self, command: str) -> bool:
        """Rising-edge detector for a logical command, resolved to a button via
        the active profile. Commands with no button (index absent or beyond the
        device's button count) are silently disabled."""
        idx = self.profile.buttons.get(command, -1)
        if self.joystick is None or idx < 0 or idx >= self.joystick.get_numbuttons():
            return False
        current = int(self.joystick.get_button(idx))
        previous = self.last_button_state.get(idx, 0)
        self.last_button_state[idx] = current
        return current == 1 and previous == 0

    def _latch_edge(self, command: str) -> Tuple[bool, Optional[bool]]:
        """Read a latch (level) switch resolved via the profile. Returns
        (changed, level); level is None when the switch isn't present on this
        device. The first observation reports changed=True so the app state syncs
        to the physical switch position at connect."""
        idx = self.profile.buttons.get(command, -1)
        if self.joystick is None or idx < 0 or idx >= self.joystick.get_numbuttons():
            return (False, None)
        level = int(self.joystick.get_button(idx)) == 1
        prev = self._latch_prev.get(command)
        self._latch_prev[command] = level
        changed = prev is None or prev != level
        return (changed, level)

    def _knob_norm(self, axis_idx: int) -> float:
        """Absolute rear-knob position normalised to 0..1 over the profile's
        calibrated raw travel (knob_raw_min..knob_raw_max), so a knob that only
        swings ~-0.7..+0.7 still reaches both ends of its value range. Uses the
        raw axis (no rest-centre offset — these are absolute pots, not sticks)."""
        lo, hi = self.profile.knob_raw_min, self.profile.knob_raw_max
        span = hi - lo
        if abs(span) < 1e-6:
            return 0.0
        return clamp((self._axis(axis_idx) - lo) / span, 0.0, 1.0)

    def _knob_value(self, axis_idx: int, lo: float, hi: float) -> float:
        return lo + self._knob_norm(axis_idx) * (hi - lo)

    def _arm_catch(self, key: str, axis_idx: int, lo: float, hi: float,
                   stored: float) -> None:
        """Re-arm the catch/takeover for a knob: it won't drive its value until
        its mapped position passes through ``stored``."""
        self._knob_caught[key] = False
        val = self._knob_value(axis_idx, lo, hi)
        self._knob_catch_sign[key] = 1.0 if (val - stored) >= 0 else -1.0

    def _catch_value(self, key: str, axis_idx: int, lo: float, hi: float,
                     stored: float) -> Optional[float]:
        """Return the knob's value once caught, else None. 'Caught' = the mapped
        value has come within KNOB_CATCH_FRACTION of the stored value or crossed
        past it since the mode was entered (so there's no jump on entry)."""
        val = self._knob_value(axis_idx, lo, hi)
        if self._knob_caught.get(key):
            return val
        diff = val - stored
        init = self._knob_catch_sign.get(key, 0.0)
        crossed = init != 0.0 and (diff >= 0.0) != (init >= 0.0)
        if abs(diff) <= abs(hi - lo) * KNOB_CATCH_FRACTION or crossed:
            self._knob_caught[key] = True
            return val
        return None

    def _debug_log_controller(self, live: LiveControl) -> None:
        """Once-per-second console echo of what the controller is reading, so a
        'nothing happens' report can be pinned to reads vs. mode vs. wiring."""
        now = time.monotonic()
        if now - getattr(self, "_last_ctrl_dbg", 0.0) < 1.0:
            return
        self._last_ctrl_dbg = now
        if self.joystick is None:
            self._log("[ctrl] no joystick open (reads impossible)\n")
            return
        p = self.profile
        mode = ("override+controller" if (live.manual_override and live.override_with_controller)
                else "override(sliders)" if live.manual_override
                else "setpoint" if self.config.use_controller else "idle")
        self._log(
            f"[ctrl] mode={mode} roll(a{p.roll_axis})={self._axis_c(p.roll_axis, p.roll_sign):+.2f} "
            f"pitch(a{p.pitch_axis})={self._axis_c(p.pitch_axis, p.pitch_sign):+.2f} "
            f"yaw(a{p.yaw_axis})={self._axis_c(p.yaw_axis, p.yaw_sign):+.2f} "
            f"thr={self._throttle_norm(p):.2f} armed={live.motor_armed}\n")

    def _poll_controller_buttons(self, live: LiveControl) -> None:
        """Process all discrete controller inputs once per tick: momentary
        (edge) buttons, the four latch switches, and the rear-knob trim/PID-tune
        modes. Runs every loop regardless of flight mode so a switch is honoured
        even while manual override is active."""
        # --- momentary (edge) buttons ------------------------------------- #
        if self._edge_cmd("trim_on"):
            self.update_live(trimmed=True); self.logs.write_event("TRIM_ON")
        if self._edge_cmd("trim_off"):
            self.update_live(trimmed=False); self.logs.write_event("TRIM_OFF")
        if self._edge_cmd("breakpoint"):
            self.logs.write_breakpoint("MANUAL_BREAKPOINT")
        if self._edge_cmd("auto_on"):
            self.update_live(autonomous=True); self.logs.write_breakpoint("AUTONOMOUS_ENABLED")
        if self._edge_cmd("auto_off"):
            self.update_live(autonomous=False); self.logs.write_breakpoint("AUTONOMOUS_DISABLED")

        # --- latch switches (level = state) ------------------------------- #
        armed_changed, armed_level = self._latch_edge("arm_latch")
        if armed_changed and armed_level is not None:
            self._handle_command("arm" if armed_level else "disarm", None)

        ovr_changed, ovr_level = self._latch_edge("override_latch")
        if ovr_changed and ovr_level is not None:
            self._handle_command("manual_override", ovr_level)
            # Flipping override on from the controller defaults to controller-
            # driven, and mirrors the switch state onto the GUI checkboxes.
            if ovr_level:
                self.update_live(override_with_controller=True)
            self.override_mode.emit(bool(ovr_level), True if ovr_level else False)

        # --- maneuver injection (unmapped until bound on the Mapping tab) -- #
        # Routed through _handle_command rather than touching the injector
        # directly, so a controller button and the tab's buttons take exactly
        # the same path -- including the gate checks and the logging.
        # Arm is a TOGGLE, not a latch: it flips the injector's own armed state
        # on each press and ignores the release. Read as a latch it tracked the
        # button level, so the injector was armed only while the button was held
        # -- unflyable, because firing and working the sticks needs that hand.
        # The flip is computed from self.injector.armed rather than from a local
        # copy, so the GUI checkbox, this button and a failsafe disarm all share
        # one source of truth and the next press is always a real inversion of
        # what the aircraft is actually doing.
        if self._edge_cmd("inject_arm"):
            self._handle_command("inject_arm", not self.injector.armed)
        if self._edge_cmd("inject_fire"):
            self._handle_command("inject_fire", None)
        if self._edge_cmd("inject_abort"):
            self._handle_command("inject_abort", None)

        # Re-snapshot so tune-mode logic sees arm/override changes from this tick.
        self._poll_tune_modes(self._live_snapshot())

        # --- save trim / tune to hardware --------------------------------- #
        if self._edge_cmd("save_tune"):
            self._save_tune()

    def _poll_tune_modes(self, live: LiveControl) -> None:
        """Handle the trim-mode / PID-mode latches, PID-axis selection, and the
        rear-knob value application (with catch/takeover). PID mode is blocked
        while the motor is armed and requires an off->on re-flip after disarm."""
        p = self.profile
        armed = live.motor_armed

        # Trim-mode latch: entering re-arms the three trim knobs' catch.
        trim_changed, trim_level = self._latch_edge("trim_mode")
        if trim_changed and trim_level is not None:
            self._trim_mode = trim_level
            if trim_level:
                self._arm_trim_catch()
                self._log("In-flight TRIM mode ON (rear knobs adjust surface trims).\n")
            else:
                self._log("In-flight TRIM mode OFF.\n")

        # PID-mode latch: blocked while armed; requires re-flip after disarm.
        pid_changed, pid_level = self._latch_edge("pid_mode")
        if pid_changed and pid_level is not None:
            if pid_level:
                if armed:
                    self._pid_latch_blocked = True
                    self._log("PID-tune mode REJECTED: motor is armed. Disarm, then "
                              "flip the PID switch off and on again.\n")
                    self.logs.write_event("PID_TUNE_REJECTED_ARMED")
                else:
                    self._pid_mode = True
                    self._pid_latch_blocked = False
                    self._arm_pid_catch()
                    self._log(f"In-flight PID-tune mode ON (axis={self._pid_axis}; "
                              f"knobs set Kp/Ki/Kd).\n")
            else:
                self._pid_mode = False
                self._pid_latch_blocked = False
                self._log("In-flight PID-tune mode OFF.\n")

        # Safety invariant: never tune while armed. If the motor becomes armed
        # while PID mode is live, drop out and require a re-flip after disarm.
        if self._pid_mode and armed:
            self._pid_mode = False
            self._pid_latch_blocked = True
            self._log("PID-tune mode exited: motor was armed.\n")
            self.logs.write_event("PID_TUNE_EXIT_ARMED")

        # PID-axis selection (edge). Re-arm catch so knobs don't jump the gains.
        for axis, cmd in (("roll", "pid_sel_roll"), ("pitch", "pid_sel_pitch"),
                          ("yaw", "pid_sel_yaw")):
            if self._edge_cmd(cmd) and self._pid_axis != axis:
                self._pid_axis = axis
                if self._pid_mode:
                    self._arm_pid_catch()
                self._log(f"PID-tune axis -> {axis}.\n")

        # Knobs do nothing unless a mode is active; trim/PID are meaningless in
        # servo-PWM (manual override), so only apply in rate-setpoint flight.
        if live.manual_override:
            return
        if self._pid_mode:
            self._apply_pid_knobs()
        elif self._trim_mode:
            self._apply_trim_knobs()

    # ----- rear-knob trim / PID application -------------------------------- #
    def _trim_knob_axes(self) -> Dict[str, int]:
        p = self.profile
        return {"roll": p.trim_roll_axis, "pitch": p.trim_pitch_axis,
                "yaw": p.trim_yaw_axis}

    def _pid_term_axes(self) -> Dict[str, int]:
        """Map each PID term to its rear knob: Kp=pitch knob (a4), Ki=roll knob
        (a3), Kd=yaw knob (a6), per the mapping."""
        p = self.profile
        return {"kp": p.trim_pitch_axis, "ki": p.trim_roll_axis,
                "kd": p.trim_yaw_axis}

    def _arm_trim_catch(self) -> None:
        lo, hi = TRIM_KNOB_RANGE
        for axis, idx in self._trim_knob_axes().items():
            if idx < 0:
                continue
            stored = float(self._last_sent.get(TRIM_PARAMS[axis], SERVO_TRIM_CENTER))
            self._arm_catch(f"trim:{axis}", idx, lo, hi, stored)

    def _arm_pid_catch(self) -> None:
        axis = self._pid_axis
        gains = getattr(self.config.gains, axis)
        for term, idx in self._pid_term_axes().items():
            if idx < 0:
                continue
            lo, hi = self._pid_term_range(term)
            self._arm_catch(f"pid:{axis}:{term}", idx, lo, hi, getattr(gains, term))

    @staticmethod
    def _pid_term_range(term: str) -> Tuple[float, float]:
        return {"kp": PID_KP_RANGE, "ki": PID_KI_RANGE, "kd": PID_KD_RANGE}[term]

    def _apply_trim_knobs(self) -> None:
        lo, hi = TRIM_KNOB_RANGE
        changed = False
        for axis, idx in self._trim_knob_axes().items():
            if idx < 0:
                continue
            param = TRIM_PARAMS[axis]
            stored = float(self._last_sent.get(param, SERVO_TRIM_CENTER))
            val = self._catch_value(f"trim:{axis}", idx, lo, hi, stored)
            if val is None:
                continue
            ival = int(clamp(val, lo, hi))
            if self._last_sent.get(param) != ival:
                self._set_param_if_changed(param, ival)
                self.trim_value.emit(axis, ival)
                self.logs.write_event("TRIM_SET", axis, ival)
                changed = True
        # One combined state row per pass, not per axis: this runs in the control
        # loop, so a knob swept across two axes would otherwise emit a burst of
        # near-identical rows.
        if changed:
            self._write_trim_state()

    def _apply_pid_knobs(self) -> None:
        axis = self._pid_axis
        gains = getattr(self.config.gains, axis)
        for term, idx in self._pid_term_axes().items():
            if idx < 0:
                continue
            lo, hi = self._pid_term_range(term)
            stored = float(getattr(gains, term))
            val = self._catch_value(f"pid:{axis}:{term}", idx, lo, hi, stored)
            if val is None:
                continue
            val = clamp(val, lo, hi)
            if abs(val - stored) < (hi - lo) * 1e-3:
                continue
            setattr(gains, term, val)
            self.cf.param.set_value(f"pid_rate.{axis}_{term}", val)
            self.pid_value.emit(axis, term, val)
            self.logs.write_event("PID_TUNE_SET", axis, term, round(val, 4))

    def _save_tune(self) -> None:
        """Persist the currently-tuned values to the deck's flash. In PID mode
        the selected axis' rate gains are stored; otherwise the surface trims."""
        if self._pid_mode:
            self._persist_pid_gains(self._pid_axis)
        else:
            self._persist_trim()

    def _persist_pid_gains(self, axis: Optional[str] = None) -> None:
        """Save rate gains to the deck's flash, one axis or all three.

        Iterates PID_TERMS, which includes kff. This used to be a hardcoded
        ("kp", "ki", "kd") and kff therefore reached flash by no path at all --
        the knobs cannot tune it (see _pid_term_range, three physical pots) and
        nothing else called persistent_store for it. A power cycle left the deck
        holding knob-tuned kp/ki/kd next to a kff of 0.0 from the compiled
        defaults, which is a feed-forward term silently switched off.
        """
        axes = PID_AXES if axis is None else (axis,)

        def _done(name, success):
            self._log(f"PID {'saved' if success else 'SAVE FAILED'} ({name}).\n")
        for ax in axes:
            for term in PID_TERMS:
                param = f"pid_rate.{ax}_{term}"
                try:
                    self.cf.param.persistent_store(param, _done)
                except Exception as exc:
                    self._log(f"PID persist error ({param}): {exc}\n")
        self.logs.write_event("PID_PERSIST_REQUEST", "all" if axis is None else axis)

    def _handle_controller_flight(self, live: LiveControl) -> None:
        """Continuous stick-driven flight: throttle + rate setpoints. Discrete
        buttons/latches are handled separately in _poll_controller_buttons."""
        p = self.profile
        roll_axis = round(self._axis_c(p.roll_axis, p.roll_sign), 3)
        pitch_axis = round(self._axis_c(p.pitch_axis, p.pitch_sign), 3)
        yaw_axis = round(self._axis_c(p.yaw_axis, p.yaw_sign), 3)

        if p.throttle_from_axis:
            # RC: the left stick is an absolute throttle axis -> set directly.
            thr = self._throttle_norm(p)
            with self._lock:
                if abs(thr - self.live.throttle) > 1e-3:
                    self.live.throttle = thr
        elif p.has_hat and self.joystick.get_numhats() > 0:
            # Xbox: D-pad steps the throttle up/down.
            hat = self.joystick.get_hat(0)
            if hat != self.last_hat_state:
                if hat == (0, 1):
                    with self._lock:
                        self.live.throttle = clamp(self.live.throttle + THROTTLE_STEP, 0.0, 1.0)
                        t = self.live.throttle
                    self.logs.write_event("THROTTLE_UP", round(t, 3))
                elif hat == (0, -1):
                    with self._lock:
                        self.live.throttle = clamp(self.live.throttle - THROTTLE_STEP, 0.0, 1.0)
                        t = self.live.throttle
                    self.logs.write_event("THROTTLE_DOWN", round(t, 3))
                self.last_hat_state = hat

        live = self._live_snapshot()
        if live.autonomous:
            self._send_rate_setpoint(live.setpoint_roll, live.setpoint_pitch,
                                     live.setpoint_yaw)
        elif not live.trimmed:
            roll_cmd = roll_axis if abs(roll_axis) > JOYSTICK_DEADBAND else 0.0
            pitch_cmd = pitch_axis if abs(pitch_axis) > JOYSTICK_DEADBAND else 0.0
            yaw_cmd = yaw_axis if abs(yaw_axis) > JOYSTICK_DEADBAND else 0.0
            self._send_rate_setpoint(roll_cmd * self.config.roll_rate_limit,
                                     pitch_cmd * self.config.pitch_rate_limit,
                                     yaw_cmd * self.config.yaw_rate_limit)
        else:
            # Trim-lock: no setpoint goes out, so the injector must be stopped
            # explicitly here. _send_rate_setpoint is the only thing that ticks
            # it, and a path that never calls it would otherwise leave a run
            # frozen mid-shape with its clock stopped.
            self._injection_tick()

        if live.motor_armed:
            self._set_bl_motor_throttle(live.throttle)

    # ----- rate setpoints + maneuver injection ----------------------------- #
    def _send_rate_setpoint(self, roll: float, pitch: float, yaw: float) -> None:
        """The single exit for every rate setpoint this app sends.

        Arguments arrive in the *stick* convention the call sites have always
        used -- i.e. exactly what used to be handed to send_setpoint before its
        -1 on pitch/yaw. Those flips happen here instead, which puts the rest of
        this function in BODY axes (+roll right, +pitch nose-up, +yaw nose-right),
        the same convention the firmware logs controller.*Rate in and the only
        one in which a maneuver amplitude means a fixed physical thing.

        Centralising the send is a safety property, not tidiness. There were
        three send_setpoint call sites (autonomous-with-controller,
        stick-driven, and GUI hold-zero); an injection added at one of them
        would be silently absent in the other two flight modes, and the rate
        clamp that only one of them applied would stay missing at the others.
        One exit means the injection and the limit cannot be bypassed by taking
        a different path through the loop.
        """
        r, p, y = float(roll), -float(pitch), -float(yaw)
        inj_r, inj_p, inj_y = self._injection_tick()
        tgt_r, tgt_p, tgt_y = r + inj_r, p + inj_p, y + inj_y
        lim_r = self.config.roll_rate_limit
        lim_p = self.config.pitch_rate_limit
        lim_y = self.config.yaw_rate_limit
        out_r = clamp(tgt_r, -lim_r, lim_r)
        out_p = clamp(tgt_p, -lim_p, lim_p)
        out_y = clamp(tgt_y, -lim_y, lim_y)
        if self.injector.active and (abs(out_r - tgt_r) > 1e-6
                                     or abs(out_p - tgt_p) > 1e-6
                                     or abs(out_y - tgt_y) > 1e-6):
            # The pilot was already close to a rate limit, so part of the
            # injection was clipped off. Flagged on the run (MANEUVER_END
            # carries saturated=1) because a clamped pass measures the limit,
            # not the response -- the same trap that dragged the flight9 pitch
            # fit from 0.551 to 0.227. Better to know the pass is unusable than
            # to fit it.
            self.injector.note_saturation()
        self.commander.send_setpoint(out_r, out_p, out_y, 10001)

    def _stick_deflection(self, axis: str) -> float:
        """Rest-corrected stick travel on one control axis, 0..1-ish. 0.0 when
        there is no controller, which makes the pilot-override gate inert
        instead of accidentally permissive."""
        if self.joystick is None or not self.config.use_controller:
            return 0.0
        p = self.profile
        idx, sign = {
            "roll": (p.roll_axis, p.roll_sign),
            "pitch": (p.pitch_axis, p.pitch_sign),
            "yaw": (p.yaw_axis, p.yaw_sign),
        }.get(axis, (-1, 1.0))
        return abs(self._axis_c(idx, sign))

    def _injection_block_reason(self, live: LiveControl,
                                axis: Optional[str] = None) -> Optional[str]:
        """Why injection must not be commanded right now, or None if it may be.

        Each of these is a state in which the rate setpoint is either not
        reaching the aircraft or not the pilot's to perturb, so injecting would
        at best burn a test point and at worst add a disturbance nobody asked
        for. Checked every tick, not just at trigger time: a maneuver that was
        legal when it started must still stop the moment the state changes.
        """
        if self.failsafe_active:
            return "FAILSAFE"
        if live.manual_override:
            # Override streams raw motor/servo values and bypasses the
            # controller; the rate loop being measured is not even running.
            return "MANUAL_OVERRIDE"
        if live.trimmed:
            # Trim / lock-servos sends no setpoint at all, so an injection here
            # would advance the shape clock against an aircraft that cannot see it.
            return "TRIM_LOCK"
        if not self.injector.armed:
            return "NOT_ARMED"
        axis = axis or self.injector.axis
        if self._inj_stick_abort and axis is not None:
            if self._stick_deflection(axis) > MANEUVER_ABORT_STICK:
                # Moving the stick is the pilot's reflex when a pass looks
                # wrong, so it has to BE the abort rather than something that
                # fights the injection for authority.
                return "PILOT_OVERRIDE"
        return None

    def _injection_tick(self) -> Tuple[float, float, float]:
        """Advance the injector one tick and return (roll, pitch, yaw) deg/s in
        body axes, after the gates and the hard amplitude ceiling."""
        now = time.monotonic()
        live = self._live_snapshot()
        reason = self._injection_block_reason(live)
        if reason is not None:
            self.injector.abort(reason, now)
            self._inj_cmd = (0.0, 0.0, 0.0)
        else:
            inj = self.injector.tick(now)
            # The ceiling is applied against the LIVE rate limits rather than
            # the stored amplitude, so it tracks whatever the pilot chose to fly
            # with on the Setup tab. A library edited by hand to 500 deg/s still
            # cannot command more than 60% of the limit.
            cap_r = MANEUVER_MAX_AMPLITUDE_FRACTION * self.config.roll_rate_limit
            cap_p = MANEUVER_MAX_AMPLITUDE_FRACTION * self.config.pitch_rate_limit
            cap_y = MANEUVER_MAX_AMPLITUDE_FRACTION * self.config.yaw_rate_limit
            self._inj_cmd = (clamp(inj[0], -cap_r, cap_r),
                             clamp(inj[1], -cap_p, cap_p),
                             clamp(inj[2], -cap_y, cap_y))
        self._drain_injector_events()
        self._emit_injector_state()
        return self._inj_cmd

    def _drain_injector_events(self) -> None:
        """Move the injector's queued events into the flight log and the console.

        MANEUVER_START / MANEUVER_END name the pass and its parameters; they are
        NOT how the analysis finds the input in time. Events.csv is stamped in
        host time to the second, which is useless against a 1 s doublet, so the
        timing comes from the per-sample inj_roll/inj_pitch/inj_yaw columns in
        Controller.csv instead. The events exist to identify and to tell you
        whether the pass saturated.
        """
        if self.logs is None:
            return
        for name, v1, v2 in self.injector.drain_events():
            self.logs.write_event(name, v1, v2)
            detail = " ".join(part for part in (v1, v2) if part)
            self._log(f"[maneuver] {name}{(' ' + detail) if detail else ''}\n")

    def _emit_injector_state(self) -> None:
        """Publish injector state to the Maneuvers tab, on change plus a slow
        heartbeat -- same pattern as _emit_button_state, for the same reason: at
        100 Hz an unconditional emit would be 100 cross-thread signals a second
        to repaint a label that usually has not changed."""
        st = self.injector.status()
        key = (st["armed"], st["active"], st["running"], st["selected_index"],
               st["saturated"])
        now = time.monotonic()
        if key == self._last_injector_status and (now - self._last_injector_emit) < 0.5:
            return
        self._last_injector_status = key
        self._last_injector_emit = now
        self.injector_state.emit(st)

    # ----- teardown -------------------------------------------------------- #
    def _shutdown(self) -> None:
        # Zero the streamed route before the param writes: those are acked and can
        # block, and the motor should stop before anything that might wait.
        self._send_propulsion_packet(0)
        self.last_servo_value = 0
        try:
            if self.cf is not None:
                self.cf.param.set_value("motorPowerSet.enable", "0")
                self.cf.param.set_value("servo.servoAngle", "0")
                self.cf.param.set_value("usd.logging", "0")
        except Exception:
            pass
        for key, cfg in self.log_configs.items():
            try:
                if self.log_enabled.get(key, False):
                    cfg.stop()
            except Exception:
                pass
        try:
            if self.commander is not None:
                self.commander.send_stop_setpoint()
        except Exception:
            pass
        if self.logs is not None:
            try:
                self.logs.write_breakpoint("SESSION_END")
                self.logs.write_event("SESSION_END")
                self.logs.close()
            except Exception:
                pass
            self.logs = None
        # Release the joystick in SDL's preferred order (object -> subsystem ->
        # pygame) so the device isn't left wedged for the next run/connect under
        # Parallels USB passthrough.
        if pygame is not None and pygame.get_init():
            try:
                if self.joystick is not None:
                    self.joystick.quit()
            except Exception:
                pass
            try:
                pygame.joystick.quit()
            except Exception:
                pass
            try:
                pygame.quit()
            except Exception:
                pass
        self.joystick = None
        self.cf = None
        self.commander = None

    def _console_callback(self, text: str) -> None:
        self.console.emit(text)
        logs = self.logs  # local ref: cflib console thread may fire during teardown
        if logs is not None:
            try:
                logs.write_console(text)
            except Exception:
                pass


# --------------------------------------------------------------------------- #
# Main window
# --------------------------------------------------------------------------- #
class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Crazyflie Glider Control")
        # Size to fit the available screen so the window never extends off-screen.
        screen = QtWidgets.QApplication.primaryScreen()
        if screen is not None:
            avail = screen.availableGeometry()
            self.resize(min(1200, avail.width()), min(800, avail.height()))
        else:
            self.resize(1200, 800)

        self.buffers = PlotBuffers()
        self.worker = GliderWorker(self.buffers)
        self.worker.console.connect(self._append_console)
        self.worker.status.connect(self._set_status)
        self.worker.connected.connect(self._on_connection_changed)
        self.worker.telemetry.connect(self._on_telemetry)
        self.worker.override_state.connect(self._on_override_state)
        self.worker.override_mode.connect(self._on_override_mode)
        self.worker.trim_value.connect(self._on_trim_value)
        self.worker.surface_map.connect(self._on_surface_map)
        self.worker.pid_value.connect(self._on_pid_value)
        self.worker.buttons_state.connect(self._on_buttons_state)
        self.worker.axes_state.connect(self._on_axes_state)
        self.worker.injector_state.connect(self._on_injector_state)

        self.tabs = QtWidgets.QTabWidget()
        self.setCentralWidget(self.tabs)
        self.tabs.addTab(self._scrollable(self._build_setup_tab()), "Setup")
        self.tabs.addTab(self._build_plots_tab(), "Live Plots")
        self.tabs.addTab(self._scrollable(self._build_control_tab()), "Control")
        self.tabs.addTab(self._scrollable(self._build_maneuver_tab()), "Maneuvers")
        self.tabs.addTab(self._scrollable(self._build_override_tab()), "Manual Override")
        self.tabs.addTab(self._scrollable(self._build_mapping_tab()), "Mapping")
        self.tabs.addTab(self._build_flightdata_tab(), "Flight Data")
        self.tabs.addTab(self._build_console_tab(), "Console")
        self.tabs.addTab(self._build_notes_tab(), "Notes")

        self.statusBar().showMessage("Disconnected")

        # Plot redraw timer (GUI thread).
        self.plot_timer = QTimer(self)
        self.plot_timer.timeout.connect(self.canvas.refresh)
        self.plot_timer.start(50)

        # After every tab exists: a malformed settings file reports to the
        # Console pane, which must already be built for that to be survivable.
        self._load_setup_defaults()

        self._set_connected_ui(False)

    # ----- tab builders ---------------------------------------------------- #
    def _scrollable(self, widget: QtWidgets.QWidget) -> QtWidgets.QScrollArea:
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(widget)
        return scroll

    def _build_setup_tab(self) -> QtWidgets.QWidget:
        w = QtWidgets.QWidget()
        form = QtWidgets.QFormLayout(w)

        self.uri_edit = QtWidgets.QLineEdit(DEFAULT_URI)
        form.addRow("Crazyflie URI:", self.uri_edit)

        self.filename_edit = QtWidgets.QLineEdit()
        self.filename_edit.setPlaceholderText("blank -> flight_<timestamp>")
        form.addRow("Log filename prefix:", self.filename_edit)

        self.controller_chk = QtWidgets.QCheckBox("Enable game controller input")
        self.controller_type_combo = QtWidgets.QComboBox()
        for label, key in (("Xbox / gamepad", "xbox"),
                           ("RC sim controller (InterLink-X)", "rc")):
            self.controller_type_combo.addItem(label, key)
        # Re-open the controller mid-session without dropping the Crazyflie link
        # -- a backup for when Parallels drops the USB passthrough during a run.
        self.reconnect_btn = QtWidgets.QPushButton("Reconnect controller")
        self.reconnect_btn.setToolTip(
            "Re-run the USB re-enumerate + open sequence for the controller "
            "without disconnecting from the Crazyflie.")
        self.reconnect_btn.clicked.connect(lambda: self.worker.post("reconnect_controller"))
        ctrl_row = QtWidgets.QHBoxLayout()
        ctrl_row.addWidget(self.controller_chk)
        ctrl_row.addWidget(self.controller_type_combo)
        ctrl_row.addWidget(self.reconnect_btn)
        ctrl_row.addStretch(1)
        form.addRow(ctrl_row)

        self.controller_debug_chk = QtWidgets.QCheckBox(
            "Log controller input to console (1 Hz diagnostic)")
        self.controller_debug_chk.setChecked(CONTROLLER_DEBUG_LOG)
        form.addRow(self.controller_debug_chk)

        self.lpf_enable_chk = QtWidgets.QCheckBox("Enable fwActLpf output filter")
        self.lpf_enable_chk.setChecked(True)
        form.addRow(self.lpf_enable_chk)

        self.lpf_cutoff_spin = QtWidgets.QDoubleSpinBox()
        self.lpf_cutoff_spin.setRange(0.1, 1000.0); self.lpf_cutoff_spin.setValue(8.0)
        form.addRow("fwActLpf cutoff (Hz):", self.lpf_cutoff_spin)

        self.roll_limit_spin = self._rate_spin(90.0)
        self.pitch_limit_spin = self._rate_spin(90.0)
        self.yaw_limit_spin = self._rate_spin(90.0)
        form.addRow("Roll rate limit (deg/s):", self.roll_limit_spin)
        form.addRow("Pitch rate limit (deg/s):", self.pitch_limit_spin)
        form.addRow("Yaw rate limit (deg/s):", self.yaw_limit_spin)

        self.log_controller_chk = QtWidgets.QCheckBox("Controller rates/setpoints"); self.log_controller_chk.setChecked(True)
        self.log_motor_chk = QtWidgets.QCheckBox("Motor data"); self.log_motor_chk.setChecked(True)
        self.log_connection_chk = QtWidgets.QCheckBox("RSSI / VBAT"); self.log_connection_chk.setChecked(True)
        self.log_accel_chk = QtWidgets.QCheckBox("Accelerometer"); self.log_accel_chk.setChecked(True)

        # Per-stream log period. Higher period = lower rate = less radio load
        # (helps with "LOG packets drop detected" on a marginal link).
        self.period_controller_spin = self._period_spin(50)
        self.period_motor_spin = self._period_spin(50)
        self.period_connection_spin = self._period_spin(50)
        self.period_accel_spin = self._period_spin(50)

        log_box = QtWidgets.QGroupBox("Logging (checkbox = enable, ms = log period)")
        log_grid = QtWidgets.QGridLayout(log_box)
        for row, (chk, spin) in enumerate((
            (self.log_controller_chk, self.period_controller_spin),
            (self.log_motor_chk, self.period_motor_spin),
            (self.log_connection_chk, self.period_connection_spin),
            (self.log_accel_chk, self.period_accel_spin),
        )):
            log_grid.addWidget(chk, row, 0)
            log_grid.addWidget(spin, row, 1)
            log_grid.addWidget(QtWidgets.QLabel("ms"), row, 2)
        form.addRow(log_box)

        self.plot_window_spin = QtWidgets.QDoubleSpinBox()
        self.plot_window_spin.setRange(2.0, 600.0)
        self.plot_window_spin.setSingleStep(5.0)
        self.plot_window_spin.setSuffix(" s")
        self.plot_window_spin.setValue(DEFAULT_PLOT_WINDOW_S)
        form.addRow("Live plot window:", self.plot_window_spin)

        self.save_setup_btn = QtWidgets.QPushButton("Save settings as launch defaults")
        self.save_setup_btn.setToolTip(
            "Store every field on this tab in glider_setup_defaults.json so it is "
            "restored the next time the GUI starts.")
        self.save_setup_btn.clicked.connect(self._save_setup_defaults)
        form.addRow(self.save_setup_btn)

        self.connect_btn = QtWidgets.QPushButton("Connect")
        self.connect_btn.setStyleSheet("font-weight: bold; padding: 8px;")
        self.connect_btn.clicked.connect(self._toggle_connection)
        form.addRow(self.connect_btn)
        return w

    def _setup_bindings(self) -> Dict[str, QtWidgets.QWidget]:
        """SessionConfig field name -> the Setup-tab widget holding that value.

        Single source of truth for the tab. Collecting a SessionConfig, saving
        and restoring launch defaults, and locking the tab while connected all
        iterate this map, so a new Setup setting only has to be registered in
        one place (plus its widget's construction) to be picked up everywhere."""
        return {
            "uri": self.uri_edit,
            "filename_prefix": self.filename_edit,
            "use_controller": self.controller_chk,
            "controller_type": self.controller_type_combo,
            "debug_controller_log": self.controller_debug_chk,
            "fwactlpf_enable": self.lpf_enable_chk,
            "fwactlpf_cutoff_hz": self.lpf_cutoff_spin,
            "roll_rate_limit": self.roll_limit_spin,
            "pitch_rate_limit": self.pitch_limit_spin,
            "yaw_rate_limit": self.yaw_limit_spin,
            "log_controller": self.log_controller_chk,
            "log_motor": self.log_motor_chk,
            "log_connection": self.log_connection_chk,
            "log_accelerometer": self.log_accel_chk,
            "period_controller_ms": self.period_controller_spin,
            "period_motor_ms": self.period_motor_spin,
            "period_connection_ms": self.period_connection_spin,
            "period_accelerometer_ms": self.period_accel_spin,
            "plot_window_s": self.plot_window_spin,
        }

    @staticmethod
    def _widget_value(wdg: QtWidgets.QWidget) -> object:
        """Read a Setup widget generically. Combos report their userData (the
        stable key like "xbox"), not the display label, so saved files survive
        a wording change in the dropdown."""
        if isinstance(wdg, QtWidgets.QCheckBox):
            return wdg.isChecked()
        if isinstance(wdg, QtWidgets.QComboBox):
            return wdg.currentData()
        if isinstance(wdg, QtWidgets.QLineEdit):
            return wdg.text()
        return wdg.value()

    @staticmethod
    def _set_widget_value(wdg: QtWidgets.QWidget, value: object) -> None:
        """Write a Setup widget generically, coercing to the type Qt demands.
        JSON has no int/float distinction, so a period saved as 50 comes back as
        50 but 50.0 would too -- QSpinBox.setValue rejects a float, hence the
        explicit int()/float() split. An unknown combo key is left alone rather
        than snapping the selection to index 0."""
        if isinstance(wdg, QtWidgets.QCheckBox):
            wdg.setChecked(bool(value))
        elif isinstance(wdg, QtWidgets.QComboBox):
            idx = wdg.findData(value)
            if idx >= 0:
                wdg.setCurrentIndex(idx)
        elif isinstance(wdg, QtWidgets.QLineEdit):
            wdg.setText(str(value))
        elif isinstance(wdg, QtWidgets.QSpinBox):
            wdg.setValue(int(value))
        else:
            wdg.setValue(float(value))

    def _save_setup_defaults(self) -> None:
        """Persist every Setup-tab control as the launch defaults."""
        payload = {field: self._widget_value(wdg)
                   for field, wdg in self._setup_bindings().items()}
        try:
            with open(SETUP_DEFAULTS_FILE, "w", encoding="utf-8") as fh:
                json.dump(payload, fh, indent=2)
        except OSError as exc:
            self._append_console(f"[setup] could not save {SETUP_DEFAULTS_FILE}: {exc}\n")
            return
        self._append_console(f"[setup] saved launch defaults to {SETUP_DEFAULTS_FILE}\n")

    def _load_setup_defaults(self) -> None:
        """Restore saved Setup-tab values at startup. Anything missing, unknown
        or malformed is skipped field by field, so a hand-edited or outdated
        file degrades to the built-in defaults instead of failing to launch."""
        try:
            with open(SETUP_DEFAULTS_FILE, "r", encoding="utf-8") as fh:
                saved = json.load(fh)
        except FileNotFoundError:
            return
        except (OSError, ValueError) as exc:
            self._append_console(f"[setup] could not load {SETUP_DEFAULTS_FILE}: {exc}\n")
            return
        if not isinstance(saved, dict):
            return
        for field, wdg in self._setup_bindings().items():
            if field not in saved:
                continue
            try:
                self._set_widget_value(wdg, saved[field])
            except (TypeError, ValueError):
                self._append_console(f"[setup] ignoring bad saved value for {field}\n")

    def _rate_spin(self, value: float) -> QtWidgets.QDoubleSpinBox:
        s = QtWidgets.QDoubleSpinBox()
        s.setRange(0.0, 500.0); s.setValue(value)
        return s

    def _period_spin(self, value: int) -> QtWidgets.QSpinBox:
        # Crazyflie log periods are multiples of 10 ms.
        s = QtWidgets.QSpinBox()
        s.setRange(10, 1000); s.setSingleStep(10); s.setValue(value)
        return s

    def _build_plots_tab(self) -> QtWidgets.QWidget:
        w = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(w)
        self.canvas = PlotCanvas(self.buffers)
        layout.addWidget(self.canvas)
        return w

    def _build_control_tab(self) -> QtWidgets.QWidget:
        w = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(w)

        # Arm / trim / breakpoint
        btn_row = QtWidgets.QHBoxLayout()
        self.arm_btn = QtWidgets.QPushButton("Arm Motor")
        self.disarm_btn = QtWidgets.QPushButton("Disarm Motor")
        self.bp_btn = QtWidgets.QPushButton("Breakpoint Marker")
        self.arm_btn.clicked.connect(lambda: self.worker.post("arm"))
        self.disarm_btn.clicked.connect(lambda: self.worker.post("disarm"))
        self.bp_btn.clicked.connect(lambda: self.worker.post("breakpoint"))
        for b in (self.arm_btn, self.disarm_btn, self.bp_btn):
            btn_row.addWidget(b)
        layout.addLayout(btn_row)

        # Throttle
        thr_box = QtWidgets.QGroupBox("Throttle")
        thr_outer = QtWidgets.QVBoxLayout(thr_box)
        thr_layout = QtWidgets.QHBoxLayout()
        self.throttle_slider = QtWidgets.QSlider(Qt.Horizontal)
        self.throttle_slider.setRange(0, 100)
        self.throttle_label = QtWidgets.QLabel("0%")
        self.throttle_slider.valueChanged.connect(self._on_throttle_changed)
        thr_layout.addWidget(self.throttle_slider); thr_layout.addWidget(self.throttle_label)
        thr_outer.addLayout(thr_layout)

        self.prop_fast_chk = QtWidgets.QCheckBox(
            "Drive the ESC on the streamed fast path (needs modded firmware)")
        self.prop_fast_chk.setChecked(PROPULSION_FAST_DEFAULT)
        self.prop_fast_chk.setToolTip(
            "ON  (default): throttle is streamed as a propulsion meta-command and\n"
            "applied inside the firmware's stabilizer loop, so it reaches the ESC\n"
            "on the next control cycle.\n"
            "OFF: throttle is written to the servo.servoAngle param. Each write is\n"
            "acked one at a time, so the host has to coalesce and slew-limit the\n"
            "command and the motor runs behind the stick. Needed on stock firmware,\n"
            "which has no propulsion decoder.\n\n"
            "To measure the difference, log servo.angle (what the deck applied)\n"
            "against servo_cmd (what this host asked for) and compare the lag.")
        self.prop_fast_chk.toggled.connect(
            lambda on: self.worker.post("propulsion_fast", bool(on)))
        thr_outer.addWidget(self.prop_fast_chk)
        layout.addWidget(thr_box)

        # Autonomous setpoints
        auto_box = QtWidgets.QGroupBox("Autonomous setpoints (deg/s)")
        auto_layout = QtWidgets.QFormLayout(auto_box)
        self.sp_roll = self._sp_spin(); self.sp_pitch = self._sp_spin(); self.sp_yaw = self._sp_spin()
        auto_layout.addRow("Roll:", self.sp_roll)
        auto_layout.addRow("Pitch:", self.sp_pitch)
        auto_layout.addRow("Yaw:", self.sp_yaw)
        self.autonomous_chk = QtWidgets.QCheckBox("Autonomous mode ENABLED")
        self.autonomous_chk.toggled.connect(lambda on: self.worker.post("autonomous", on))
        send_sp_btn = QtWidgets.QPushButton("Send setpoints")
        send_sp_btn.clicked.connect(self._send_setpoints)
        auto_layout.addRow(self.autonomous_chk)
        auto_layout.addRow(send_sp_btn)
        layout.addWidget(auto_box)

        # PID tuning
        pid_box = QtWidgets.QGroupBox("PID rate gains")
        grid = QtWidgets.QGridLayout(pid_box)
        grid.addWidget(QtWidgets.QLabel("KP"), 0, 1)
        grid.addWidget(QtWidgets.QLabel("KI"), 0, 2)
        grid.addWidget(QtWidgets.QLabel("KD"), 0, 3)
        grid.addWidget(QtWidgets.QLabel("KFF"), 0, 4)
        defaults = load_default_gains()
        self.pid_spins = {}
        for row, (axis, g) in enumerate((("pitch", defaults.pitch), ("yaw", defaults.yaw), ("roll", defaults.roll)), start=1):
            grid.addWidget(QtWidgets.QLabel(axis.capitalize()), row, 0)
            for col, val in enumerate((g.kp, g.ki, g.kd, g.kff), start=1):
                spin = QtWidgets.QDoubleSpinBox()
                spin.setRange(-100000.0, 100000.0); spin.setDecimals(2); spin.setValue(val)
                grid.addWidget(spin, row, col)
                self.pid_spins[(axis, col)] = spin
        apply_pid_btn = QtWidgets.QPushButton("Apply PID")
        apply_pid_btn.clicked.connect(self._apply_pid)
        grid.addWidget(apply_pid_btn, 4, 0, 1, 5)
        save_pid_btn = QtWidgets.QPushButton("Save as launch defaults")
        save_pid_btn.setToolTip("Store these gains in glider_pid_defaults.json so they "
                                "load into these boxes on the next launch and are pushed "
                                "to the deck on Connect.")
        save_pid_btn.clicked.connect(self._save_pid_defaults)
        grid.addWidget(save_pid_btn, 5, 0, 1, 5)
        persist_pid_btn = QtWidgets.QPushButton("Save to deck flash")
        persist_pid_btn.setToolTip(
            "Store the gains the deck is currently running in its own flash "
            "(PARAM_PERSISTENT), so they survive a power cycle.\n"
            "Saves what the deck HAS, not what these boxes show -- press Apply PID "
            "first if you have edited them.\n"
            "Covers all three axes and all four terms including KFF, which "
            "previously could not be saved to flash by any route.")
        persist_pid_btn.clicked.connect(lambda: self.worker.post("persist_pid", None))
        grid.addWidget(persist_pid_btn, 6, 0, 1, 5)
        layout.addWidget(pid_box)

        # Per-surface servo trims: the center each control surface actuates
        # around (stabilizer.trim{Roll,Pitch,Yaw}). Slider + spinbox per axis
        # stay in sync; both apply live to the deck.
        trim_box = QtWidgets.QGroupBox("Servo trims (center per surface)")
        trim_layout = QtWidgets.QGridLayout(trim_box)
        trim_layout.addWidget(QtWidgets.QLabel("Surface"), 0, 0)
        trim_layout.addWidget(QtWidgets.QLabel("Trim"), 0, 1)
        trim_layout.addWidget(QtWidgets.QLabel("Value"), 0, 2)
        self.trim_sliders = {}
        self.trim_spins = {}
        surfaces = (("roll", "Roll (aileron)"), ("pitch", "Pitch (elevator)"), ("yaw", "Yaw (rudder)"))
        for row, (axis, label) in enumerate(surfaces, start=1):
            slider = QtWidgets.QSlider(Qt.Horizontal)
            slider.setRange(0, MAX_MOTOR_CMD); slider.setValue(SERVO_TRIM_CENTER)
            spin = QtWidgets.QSpinBox()
            spin.setRange(0, MAX_MOTOR_CMD); spin.setValue(SERVO_TRIM_CENTER)
            slider.valueChanged.connect(self._make_trim_handler(axis))
            spin.valueChanged.connect(self._make_trim_handler(axis))
            trim_layout.addWidget(QtWidgets.QLabel(label), row, 0)
            trim_layout.addWidget(slider, row, 1)
            trim_layout.addWidget(spin, row, 2)
            self.trim_sliders[axis] = slider
            self.trim_spins[axis] = spin

        trim_center_btn = QtWidgets.QPushButton(f"Center all ({SERVO_TRIM_CENTER})")
        trim_center_btn.clicked.connect(self._center_trims)
        self.trim_save_btn = QtWidgets.QPushButton("Save to deck")
        self.trim_save_btn.clicked.connect(lambda: self.worker.post("persist_trim"))
        trim_layout.addWidget(trim_center_btn, len(surfaces) + 1, 0, 1, 2)
        trim_layout.addWidget(self.trim_save_btn, len(surfaces) + 1, 2)
        layout.addWidget(trim_box)

        # Servo mixer map: assign each motor channel (M1-M4) to a control surface
        # and optionally reverse it (firmware fwSurfMap.*). Applies to stabilized
        # flight; changes are pushed live and can be saved to the deck's flash.
        map_box = QtWidgets.QGroupBox("Servo \u2192 surface map (stabilized flight)")
        map_layout = QtWidgets.QGridLayout(map_box)
        map_layout.addWidget(QtWidgets.QLabel("Channel"), 0, 0)
        map_layout.addWidget(QtWidgets.QLabel("Control surface"), 0, 1)
        map_layout.addWidget(QtWidgets.QLabel("Invert"), 0, 2)
        self.surface_combos = {}
        self.surface_invert_chks = {}
        for row, ch in enumerate(SURFACE_MAP_CHANNELS, start=1):
            map_layout.addWidget(QtWidgets.QLabel(ch.upper()), row, 0)
            combo = QtWidgets.QComboBox()
            for code, label in SURFACE_OPTIONS:
                combo.addItem(label, code)
            invert_chk = QtWidgets.QCheckBox("Reverse")
            # Seed with the firmware defaults before wiring signals so no
            # spurious set_surface_map is posted during construction.
            surf_default, inv_default = SURFACE_MAP_DEFAULTS[ch]
            combo.setCurrentIndex(max(0, combo.findData(surf_default)))
            invert_chk.setChecked(bool(inv_default))
            combo.currentIndexChanged.connect(self._make_surface_map_handler(ch))
            invert_chk.toggled.connect(self._make_surface_map_handler(ch))
            map_layout.addWidget(combo, row, 1)
            map_layout.addWidget(invert_chk, row, 2)
            self.surface_combos[ch] = combo
            self.surface_invert_chks[ch] = invert_chk
        self.surface_map_save_btn = QtWidgets.QPushButton("Save map to deck")
        self.surface_map_save_btn.clicked.connect(lambda: self.worker.post("persist_surface_map"))
        map_layout.addWidget(self.surface_map_save_btn, len(SURFACE_MAP_CHANNELS) + 1, 0, 1, 3)
        layout.addWidget(map_box)

        layout.addStretch(1)
        return w

    def _sp_spin(self) -> QtWidgets.QDoubleSpinBox:
        s = QtWidgets.QDoubleSpinBox(); s.setRange(-500.0, 500.0); return s

    # ----- Maneuvers tab --------------------------------------------------- #
    def _build_maneuver_tab(self) -> QtWidgets.QWidget:
        """Test-card editor + run controls for the maneuver injector.

        The library is edited here and pushed to the worker by value on every
        change; the worker never reads a widget. Buttons on this tab and a (yet
        unbound) controller button post the identical commands, so there is one
        implementation of arm / fire / abort rather than a GUI copy and a
        controller copy that can drift apart.
        """
        w = QtWidgets.QWidget()
        outer = QtWidgets.QVBoxLayout(w)

        self.maneuvers: List[Maneuver] = load_maneuvers()
        # Guard against feedback: populating the editor fires the same
        # valueChanged signals the user's edits do, which would write the
        # half-populated editor straight back into the library.
        self._mv_loading = False

        top = QtWidgets.QHBoxLayout()
        outer.addLayout(top)

        # --- library list ---------------------------------------------------
        lib_box = QtWidgets.QGroupBox("Test card")
        lib_lay = QtWidgets.QVBoxLayout(lib_box)
        self.mv_list = QtWidgets.QListWidget()
        self.mv_list.setMinimumWidth(260)
        self.mv_list.currentRowChanged.connect(self._mv_on_row_changed)
        self.mv_list.itemChanged.connect(self._mv_on_item_changed)
        lib_lay.addWidget(self.mv_list)
        lib_lay.addWidget(QtWidgets.QLabel(
            "The tick box marks an entry as part of the sequence used by\n"
            "\"advance after each run\". Order is the order of this list."))
        row = QtWidgets.QHBoxLayout()
        for label, slot in (("Add", self._mv_add), ("Duplicate", self._mv_duplicate),
                            ("Delete", self._mv_delete)):
            btn = QtWidgets.QPushButton(label)
            btn.clicked.connect(slot)
            row.addWidget(btn)
        lib_lay.addLayout(row)
        row2 = QtWidgets.QHBoxLayout()
        save_btn = QtWidgets.QPushButton("Save library")
        save_btn.setToolTip(f"Write the card to {os.path.basename(MANEUVER_FILE)} "
                            f"so it reloads on the next launch.")
        save_btn.clicked.connect(self._mv_save)
        revert_btn = QtWidgets.QPushButton("Revert to saved")
        revert_btn.clicked.connect(self._mv_revert)
        row2.addWidget(save_btn); row2.addWidget(revert_btn)
        lib_lay.addLayout(row2)
        top.addWidget(lib_box)

        # --- editor ---------------------------------------------------------
        ed_box = QtWidgets.QGroupBox("Selected maneuver")
        ed_lay = QtWidgets.QVBoxLayout(ed_box)
        form = QtWidgets.QFormLayout()
        ed_lay.addLayout(form)

        self.mv_name = QtWidgets.QLineEdit()
        self.mv_name.editingFinished.connect(self._mv_editor_changed)
        form.addRow("Name:", self.mv_name)

        self.mv_axis = QtWidgets.QComboBox()
        for axis in MANEUVER_AXES:
            self.mv_axis.addItem(axis.capitalize(), axis)
        self.mv_axis.currentIndexChanged.connect(self._mv_editor_changed)
        form.addRow("Axis:", self.mv_axis)

        self.mv_shape = QtWidgets.QComboBox()
        for key, label in MANEUVER_SHAPES:
            self.mv_shape.addItem(label, key)
        self.mv_shape.currentIndexChanged.connect(self._mv_shape_changed)
        form.addRow("Shape:", self.mv_shape)

        self.mv_amp = QtWidgets.QDoubleSpinBox()
        self.mv_amp.setRange(-180.0, 180.0); self.mv_amp.setDecimals(1)
        self.mv_amp.setSingleStep(1.0); self.mv_amp.setSuffix(" deg/s")
        self.mv_amp.setToolTip(
            "Peak commanded rate, added to the pilot's command.\n"
            f"Capped in flight at {MANEUVER_MAX_AMPLITUDE_FRACTION:.0%} of that "
            "axis's rate limit from the Setup tab, whatever is typed here.")
        self.mv_amp.valueChanged.connect(self._mv_editor_changed)
        form.addRow("Amplitude:", self.mv_amp)

        self.mv_dur = QtWidgets.QDoubleSpinBox()
        self.mv_dur.setRange(0.1, MANEUVER_MAX_DURATION_S); self.mv_dur.setDecimals(2)
        self.mv_dur.setSingleStep(0.1); self.mv_dur.setSuffix(" s")
        self.mv_dur.valueChanged.connect(self._mv_editor_changed)
        form.addRow("Duration:", self.mv_dur)

        self.mv_settle = QtWidgets.QDoubleSpinBox()
        self.mv_settle.setRange(0.0, MANEUVER_MAX_SETTLE_S); self.mv_settle.setDecimals(2)
        self.mv_settle.setSingleStep(0.1); self.mv_settle.setSuffix(" s")
        self.mv_settle.setToolTip(
            "Quiet time at zero injection before and after the shape, inside the\n"
            "run. The response can only be measured against a known baseline, so\n"
            "this is what gives the fit its pre-input trim state.")
        self.mv_settle.valueChanged.connect(self._mv_editor_changed)
        form.addRow("Settle (each end):", self.mv_settle)

        self.mv_cycles = QtWidgets.QDoubleSpinBox()
        self.mv_cycles.setRange(0.5, 20.0); self.mv_cycles.setDecimals(1)
        self.mv_cycles.setSingleStep(0.5)
        self.mv_cycles.valueChanged.connect(self._mv_editor_changed)
        self.mv_cycles_row = QtWidgets.QLabel("Cycles (sine):")
        form.addRow(self.mv_cycles_row, self.mv_cycles)

        self.mv_f0 = QtWidgets.QDoubleSpinBox()
        self.mv_f0.setRange(0.1, 20.0); self.mv_f0.setDecimals(2)
        self.mv_f0.setSingleStep(0.1); self.mv_f0.setSuffix(" Hz")
        self.mv_f0.valueChanged.connect(self._mv_editor_changed)
        self.mv_f0_row = QtWidgets.QLabel("Chirp start:")
        form.addRow(self.mv_f0_row, self.mv_f0)

        self.mv_f1 = QtWidgets.QDoubleSpinBox()
        self.mv_f1.setRange(0.1, 20.0); self.mv_f1.setDecimals(2)
        self.mv_f1.setSingleStep(0.1); self.mv_f1.setSuffix(" Hz")
        self.mv_f1.valueChanged.connect(self._mv_editor_changed)
        self.mv_f1_row = QtWidgets.QLabel("Chirp end:")
        form.addRow(self.mv_f1_row, self.mv_f1)

        # Preview. Worth the widget: the shape functions are the one part of
        # this that is easy to get subtly wrong (a 3-2-1-1 with the wrong pulse
        # widths still looks plausible in numbers), and this draws exactly what
        # the control loop will command, from the same unit_value() code.
        self.mv_fig = Figure(figsize=(5, 2.0))
        self.mv_ax = self.mv_fig.add_subplot(111)
        self.mv_canvas = FigureCanvas(self.mv_fig)
        self.mv_canvas.setMinimumHeight(170)
        ed_lay.addWidget(self.mv_canvas)
        top.addWidget(ed_box, 1)

        # --- run controls ---------------------------------------------------
        run_box = QtWidgets.QGroupBox("Run")
        run_lay = QtWidgets.QVBoxLayout(run_box)
        btn_row = QtWidgets.QHBoxLayout()
        self.mv_arm_chk = QtWidgets.QCheckBox("Arm injector")
        self.mv_arm_chk.setToolTip(
            "Two-step on purpose: arming is the conscious \"I intend to inject on\n"
            "this flight\" decision, firing is the per-pass action. A stray press\n"
            "of an unarmed injector does nothing.")
        self.mv_arm_chk.toggled.connect(self._mv_arm)
        btn_row.addWidget(self.mv_arm_chk)
        self.mv_fire_btn = QtWidgets.QPushButton("Fire selected")
        self.mv_fire_btn.clicked.connect(self._mv_fire)
        btn_row.addWidget(self.mv_fire_btn)
        self.mv_abort_btn = QtWidgets.QPushButton("Abort")
        self.mv_abort_btn.clicked.connect(self._mv_abort)
        btn_row.addWidget(self.mv_abort_btn)
        btn_row.addStretch(1)
        run_lay.addLayout(btn_row)

        self.mv_advance_chk = QtWidgets.QCheckBox(
            "Advance to the next ticked entry after each run")
        self.mv_advance_chk.setToolTip(
            "Turns one button into a test card: each trigger flies the next point\n"
            "in the list instead of repeating the same one.")
        self.mv_advance_chk.toggled.connect(
            lambda on: self.worker.post("inject_advance", bool(on)))
        run_lay.addWidget(self.mv_advance_chk)

        self.mv_stick_abort_chk = QtWidgets.QCheckBox(
            f"Abort if the stick on that axis moves past "
            f"{MANEUVER_ABORT_STICK:.0%} travel")
        self.mv_stick_abort_chk.setChecked(True)
        self.mv_stick_abort_chk.toggled.connect(
            lambda on: self.worker.post("inject_stick_abort", bool(on)))
        run_lay.addWidget(self.mv_stick_abort_chk)

        self.mv_status = QtWidgets.QLabel("Injector: disarmed")
        self.mv_status.setStyleSheet("font-weight: bold;")
        run_lay.addWidget(self.mv_status)
        outer.addWidget(run_box)

        gates = QtWidgets.QLabel(
            "Safety gates, all re-checked every control tick (~100 Hz):\n"
            f"  - amplitude is capped at {MANEUVER_MAX_AMPLITUDE_FRACTION:.0%} of "
            f"the axis rate limit, and the total command is still clamped to the limit\n"
            f"  - duration is capped at {MANEUVER_MAX_DURATION_S:g} s, with a "
            f"watchdog that ends any run past its own length\n"
            f"  - {MANEUVER_COOLDOWN_S:g} s cooldown between runs, and only one run "
            f"at a time\n"
            "  - a run aborts instantly on failsafe/link loss, manual override "
            "or trim-lock\n"
            f"  - ...and on pilot input past {MANEUVER_ABORT_STICK:.0%} travel on the "
            f"INJECTED axis only (other axes are free; held trim counts toward it)\n"
            "  - a failsafe also DISARMS the injector, so it has to be re-armed "
            "deliberately\n"
            "  - the injection is logged per sample as inj_roll/inj_pitch/inj_yaw "
            "in Controller.csv\n"
            "Controller buttons for arm / fire / abort ship UNMAPPED — bind them on "
            "the Mapping tab.")
        gates.setStyleSheet("color: #555;")
        outer.addWidget(gates)
        outer.addStretch(1)

        self._mv_refresh_list(select=0)
        return w

    # ----- Maneuvers tab: library editing ---------------------------------- #
    def _mv_refresh_list(self, select: Optional[int] = None) -> None:
        """Rebuild the list widget from self.maneuvers."""
        prev = self.mv_list.currentRow() if select is None else select
        self._mv_loading = True
        self.mv_list.clear()
        for man in self.maneuvers:
            item = QtWidgets.QListWidgetItem(f"{man.name}  —  {man.describe()}")
            item.setFlags(item.flags() | Qt.ItemIsUserCheckable)
            item.setCheckState(Qt.Checked if man.enabled else Qt.Unchecked)
            self.mv_list.addItem(item)
        self._mv_loading = False
        if self.maneuvers:
            self.mv_list.setCurrentRow(max(0, min(prev, len(self.maneuvers) - 1)))
        else:
            self._mv_load_editor(None)
        self._mv_push()

    def _mv_current(self) -> Optional[Maneuver]:
        row = self.mv_list.currentRow()
        if 0 <= row < len(self.maneuvers):
            return self.maneuvers[row]
        return None

    def _mv_on_row_changed(self, row: int) -> None:
        self._mv_load_editor(self._mv_current())
        if 0 <= row < len(self.maneuvers) and getattr(self, "_worker_running", False):
            self.worker.post("maneuver_select", row)

    def _mv_on_item_changed(self, item) -> None:
        """Only the tick box can change here (the text is set programmatically),
        so this is the enabled flag."""
        if self._mv_loading:
            return
        row = self.mv_list.row(item)
        if 0 <= row < len(self.maneuvers):
            self.maneuvers[row].enabled = item.checkState() == Qt.Checked
            self._mv_push()

    def _mv_load_editor(self, man: Optional[Maneuver]) -> None:
        self._mv_loading = True
        try:
            enabled = man is not None
            for wdg in (self.mv_name, self.mv_axis, self.mv_shape, self.mv_amp,
                        self.mv_dur, self.mv_settle, self.mv_cycles,
                        self.mv_f0, self.mv_f1):
                wdg.setEnabled(enabled)
            if man is None:
                self.mv_name.setText("")
                self._mv_preview(None)
                return
            self.mv_name.setText(man.name)
            self.mv_axis.setCurrentIndex(max(0, self.mv_axis.findData(man.axis)))
            self.mv_shape.setCurrentIndex(max(0, self.mv_shape.findData(man.shape)))
            self.mv_amp.setValue(man.amplitude_dps)
            self.mv_dur.setValue(man.duration_s)
            self.mv_settle.setValue(man.settle_s)
            self.mv_cycles.setValue(man.cycles)
            self.mv_f0.setValue(man.f_start_hz)
            self.mv_f1.setValue(man.f_end_hz)
        finally:
            self._mv_loading = False
        self._mv_shape_rows(man.shape)
        self._mv_preview(man)

    def _mv_shape_rows(self, shape: str) -> None:
        """Show only the parameters the chosen shape actually uses. Leaving a
        dead 'Chirp end' box live next to a doublet invites someone to tune it
        and wonder why nothing changed."""
        sine = shape == "sine"
        chirp = shape == "chirp"
        for wdg, on in ((self.mv_cycles, sine), (self.mv_cycles_row, sine),
                        (self.mv_f0, chirp), (self.mv_f0_row, chirp),
                        (self.mv_f1, chirp), (self.mv_f1_row, chirp)):
            wdg.setVisible(on)

    def _mv_shape_changed(self) -> None:
        self._mv_shape_rows(str(self.mv_shape.currentData()))
        self._mv_editor_changed()

    def _mv_editor_changed(self) -> None:
        """Write the editor back into the selected maneuver."""
        if self._mv_loading:
            return
        row = self.mv_list.currentRow()
        if not (0 <= row < len(self.maneuvers)):
            return
        man = Maneuver(
            name=self.mv_name.text(),
            axis=str(self.mv_axis.currentData()),
            shape=str(self.mv_shape.currentData()),
            amplitude_dps=self.mv_amp.value(),
            duration_s=self.mv_dur.value(),
            settle_s=self.mv_settle.value(),
            cycles=self.mv_cycles.value(),
            f_start_hz=self.mv_f0.value(),
            f_end_hz=self.mv_f1.value(),
            enabled=self.maneuvers[row].enabled,
        ).sanitized()
        self.maneuvers[row] = man
        item = self.mv_list.item(row)
        if item is not None:
            self._mv_loading = True
            item.setText(f"{man.name}  —  {man.describe()}")
            self._mv_loading = False
        self._mv_preview(man)
        self._mv_push()

    def _mv_preview(self, man: Optional[Maneuver]) -> None:
        """Plot the commanded injection against time, from the same
        unit_value() the control loop uses."""
        self.mv_ax.clear()
        if man is not None:
            total = man.total_duration_s
            n = max(200, int(total * 400))
            ts = [total * i / (n - 1) for i in range(n)]
            vs = [man.amplitude_dps * man.unit_value(t) for t in ts]
            self.mv_ax.plot(ts, vs, lw=1.4)
            self.mv_ax.axhline(0.0, color="0.7", lw=0.8)
            if man.settle_s > 0:
                for x in (man.settle_s, man.settle_s + man.duration_s):
                    self.mv_ax.axvline(x, color="0.8", lw=0.8, ls="--")
            self.mv_ax.set_title(f"{man.name}: injected {man.axis} rate", fontsize=9)
            self.mv_ax.set_xlabel("time into run (s)", fontsize=8)
            self.mv_ax.set_ylabel("deg/s", fontsize=8)
            self.mv_ax.tick_params(labelsize=7)
            self.mv_ax.margins(x=0.02, y=0.2)
        self.mv_fig.tight_layout()
        self.mv_canvas.draw_idle()

    def _mv_push(self) -> None:
        """Send the library (by value) to the worker, if a session is live."""
        if not getattr(self, "_worker_running", False):
            return
        self.worker.post("maneuver_library",
                         (list(self.maneuvers), self.mv_list.currentRow()))

    def _mv_add(self) -> None:
        self.maneuvers.append(Maneuver())
        self._mv_refresh_list(select=len(self.maneuvers) - 1)

    def _mv_duplicate(self) -> None:
        man = self._mv_current()
        if man is None:
            return
        copy = Maneuver(**asdict(man))
        copy.name = f"{man.name} copy"
        self.maneuvers.insert(self.mv_list.currentRow() + 1, copy.sanitized())
        self._mv_refresh_list(select=self.mv_list.currentRow() + 1)

    def _mv_delete(self) -> None:
        row = self.mv_list.currentRow()
        if not (0 <= row < len(self.maneuvers)):
            return
        del self.maneuvers[row]
        self._mv_refresh_list(select=max(0, row - 1))

    def _mv_save(self) -> None:
        try:
            save_maneuvers(self.maneuvers)
        except OSError as exc:
            self._append_console(f"[maneuver] could not save {MANEUVER_FILE}: {exc}\n")
            return
        self._append_console(f"[maneuver] saved {len(self.maneuvers)} maneuvers to "
                             f"{MANEUVER_FILE}\n")

    def _mv_revert(self) -> None:
        self.maneuvers = load_maneuvers()
        self._mv_refresh_list(select=0)
        self._append_console("[maneuver] reverted to the saved test card\n")

    # ----- Maneuvers tab: run controls ------------------------------------- #
    def _mv_arm(self, on: bool) -> None:
        if not getattr(self, "_worker_running", False):
            if on:
                self._append_console("[maneuver] connect before arming the injector\n")
                self.mv_arm_chk.setChecked(False)
            return
        self.worker.post("inject_arm", bool(on))

    def _mv_fire(self) -> None:
        if not getattr(self, "_worker_running", False):
            self._append_console("[maneuver] not connected\n")
            return
        self.worker.post("inject_fire", None)

    def _mv_abort(self) -> None:
        if getattr(self, "_worker_running", False):
            self.worker.post("inject_abort", None)

    def _on_injector_state(self, st: object) -> None:
        """Mirror worker-side injector state into the tab. The worker is the
        authority: it arms/disarms on its own (failsafe, controller latch), so
        the checkbox follows it rather than the other way round."""
        if not isinstance(st, dict):
            return
        armed = bool(st.get("armed"))
        if armed != self.mv_arm_chk.isChecked():
            self.mv_arm_chk.blockSignals(True)
            self.mv_arm_chk.setChecked(armed)
            self.mv_arm_chk.blockSignals(False)
        idx = int(st.get("selected_index", -1))
        if 0 <= idx < self.mv_list.count() and idx != self.mv_list.currentRow():
            # Advance mode moves the selection on the worker side.
            self.mv_list.setCurrentRow(idx)
        if st.get("active"):
            text = f"INJECTING: {st.get('running', '')}"
            colour = "#b35c00"
        elif armed:
            text = f"Armed — next: {st.get('selected', '(none)')}"
            colour = "#006400"
        else:
            text = "Injector: disarmed"
            colour = "#555"
        if st.get("saturated"):
            text += "  [rate-limited: do not fit this pass]"
        self.mv_status.setText(text)
        self.mv_status.setStyleSheet(f"font-weight: bold; color: {colour};")

    def _build_override_tab(self) -> QtWidgets.QWidget:
        w = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(w)

        warn = QtWidgets.QLabel(
            "Manual override bypasses the flight controller and writes motorPowerSet.* / "
            "servo.servoAngle directly. Use with props off until you trust your mapping."
        )
        warn.setWordWrap(True)
        warn.setStyleSheet("color: #b00; font-weight: bold;")
        layout.addWidget(warn)

        self.override_chk = QtWidgets.QCheckBox("Manual override ENABLED (motorPowerSet.enable=1)")
        self.override_chk.toggled.connect(lambda on: self.worker.post("manual_override", on))
        layout.addWidget(self.override_chk)

        self.override_ctrl_chk = QtWidgets.QCheckBox("Drive override with game controller")
        self.override_ctrl_chk.toggled.connect(
            lambda on: self.worker.update_live(override_with_controller=on))
        layout.addWidget(self.override_ctrl_chk)

        self.override_fast_chk = QtWidgets.QCheckBox(
            "Commander-fast override (streamed; needs modded firmware)")
        self.override_fast_chk.setToolTip(
            "Stream the raw motor/servo command on the setpoint channel for "
            "responsive hand flying, instead of the slower motorPowerSet.* param "
            "writes. Requires the modded firmware (manualMotor setpoint decoder). "
            "Uncheck to fall back to the param path on stock firmware.")
        self.override_fast_chk.setChecked(True)
        self.override_fast_chk.toggled.connect(
            lambda on: self.worker.post("fast_override", on))
        layout.addWidget(self.override_fast_chk)

        grid = QtWidgets.QGridLayout()
        self.override_sliders = {}
        self.override_spinboxes = {}
        for row, name in enumerate(("m1", "m2", "m3", "m4", "servo")):
            grid.addWidget(QtWidgets.QLabel(name.upper()), row, 0)
            slider = QtWidgets.QSlider(Qt.Horizontal)
            slider.setRange(0, MAX_MOTOR_CMD)
            spin = QtWidgets.QSpinBox()
            spin.setRange(0, MAX_MOTOR_CMD)
            # Only commit on Enter / focus-out, not on every keystroke.
            spin.setKeyboardTracking(False)
            slider.valueChanged.connect(self._make_override_handler(name, spin))
            spin.valueChanged.connect(self._make_override_spin_handler(name, slider))
            grid.addWidget(slider, row, 1)
            grid.addWidget(spin, row, 2)
            self.override_sliders[name] = slider
            self.override_spinboxes[name] = spin
        layout.addLayout(grid)

        zero_btn = QtWidgets.QPushButton("Zero all channels")
        zero_btn.clicked.connect(self._zero_override)
        layout.addWidget(zero_btn)
        layout.addStretch(1)
        return w

    def _make_override_handler(self, name: str, spin: QtWidgets.QSpinBox):
        def handler(value: int):
            spin.blockSignals(True)
            spin.setValue(value)
            spin.blockSignals(False)
            self.worker.update_live(**{f"override_{name}": value})
        return handler

    def _make_override_spin_handler(self, name: str, slider: QtWidgets.QSlider):
        def handler(value: int):
            slider.blockSignals(True)
            slider.setValue(value)
            slider.blockSignals(False)
            self.worker.update_live(**{f"override_{name}": value})
        return handler

    def _on_override_mode(self, manual_on: bool, with_controller: bool) -> None:
        """Mirror the controller's override latch onto the Manual Override tab.

        Signals are blocked so this state sync does not re-post to the worker
        (which already applied the switch state)."""
        self.override_chk.blockSignals(True)
        self.override_chk.setChecked(manual_on)
        self.override_chk.blockSignals(False)
        if manual_on:
            self.override_ctrl_chk.blockSignals(True)
            self.override_ctrl_chk.setChecked(with_controller)
            self.override_ctrl_chk.blockSignals(False)

    def _on_override_state(self, m1: int, m2: int, m3: int, m4: int, servo: int) -> None:
        """Reflect the worker's live override command on the sliders/spinboxes.

        Signals are blocked while setting the value so this display-only update
        does not feed back into worker.update_live (which would fight the
        controller input)."""
        for name, value in (("m1", m1), ("m2", m2), ("m3", m3),
                            ("m4", m4), ("servo", servo)):
            slider = self.override_sliders[name]
            slider.blockSignals(True)
            slider.setValue(int(value))
            slider.blockSignals(False)
            spin = self.override_spinboxes[name]
            spin.blockSignals(True)
            spin.setValue(int(value))
            spin.blockSignals(False)

    # ----- Mapping tab: editable button layouts ---------------------------- #
    # The tab edits a *draft* ControllerMap held in self._map_draft, never the saved
    # file and never the worker's live profile. Nothing leaves this tab until the
    # user presses Save, which is what makes Revert cheap and makes a half-finished
    # edit harmless if the GUI is closed.
    def _build_mapping_tab(self) -> QtWidgets.QWidget:
        w = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(w)

        self._map_draft: ControllerMap = default_controller_map("rc")
        self._map_dirty = False
        self._map_learn_cmd: Optional[str] = None   # command awaiting a press
        self._map_armed = False                     # last known motor_armed
        self._map_nbuttons = 0                      # buttons the pad reports
        self._map_pressed: frozenset = frozenset()
        self._map_loading = False                   # suppress widget signals
        self._map_axis_values: Tuple[float, ...] = ()
        self._map_detect: Optional[dict] = None     # in-flight axis detection

        # --- row 1: which controller, which saved map ---------------------- #
        top = QtWidgets.QHBoxLayout()
        top.addWidget(QtWidgets.QLabel("Controller:"))
        self.map_ctype_combo = QtWidgets.QComboBox()
        for label, key in (("Xbox / gamepad", "xbox"),
                           ("RC sim controller (InterLink-X)", "rc")):
            self.map_ctype_combo.addItem(label, key)
        self.map_ctype_combo.setCurrentIndex(1)
        self.map_ctype_combo.currentIndexChanged.connect(self._map_on_ctype_changed)
        top.addWidget(self.map_ctype_combo)

        top.addSpacing(16)
        top.addWidget(QtWidgets.QLabel("Saved map:"))
        self.map_name_combo = QtWidgets.QComboBox()
        self.map_name_combo.setMinimumWidth(220)
        self.map_name_combo.currentIndexChanged.connect(self._map_on_name_changed)
        top.addWidget(self.map_name_combo)
        top.addStretch(1)
        layout.addLayout(top)

        # --- row 2: what you can do with it -------------------------------- #
        btns = QtWidgets.QHBoxLayout()
        self.map_save_btn = QtWidgets.QPushButton("Save")
        self.map_save_btn.setToolTip("Overwrite the selected map and apply it now.")
        self.map_save_btn.clicked.connect(self._map_save)
        self.map_saveas_btn = QtWidgets.QPushButton("Save as...")
        self.map_saveas_btn.setToolTip("Store these edits under a new name.")
        self.map_saveas_btn.clicked.connect(self._map_save_as)
        self.map_delete_btn = QtWidgets.QPushButton("Delete")
        self.map_delete_btn.clicked.connect(self._map_delete)
        self.map_revert_btn = QtWidgets.QPushButton("Revert to built-in")
        self.map_revert_btn.setToolTip(
            "Discard edits and reload this controller's factory layout.")
        self.map_revert_btn.clicked.connect(self._map_revert)
        for b in (self.map_save_btn, self.map_saveas_btn,
                  self.map_delete_btn, self.map_revert_btn):
            btns.addWidget(b)
        btns.addStretch(1)
        layout.addLayout(btns)

        # --- banners: arm lock, then validation ---------------------------- #
        self.map_lock_label = QtWidgets.QLabel()
        self.map_lock_label.setStyleSheet(
            "color: #b00020; font-weight: bold; padding: 4px;")
        self.map_lock_label.setVisible(False)
        layout.addWidget(self.map_lock_label)

        self.map_warn_label = QtWidgets.QLabel()
        self.map_warn_label.setWordWrap(True)
        self.map_warn_label.setStyleSheet(
            "color: #8a6d00; background: #fff8e1; border: 1px solid #ffe082; padding: 6px;")
        self.map_warn_label.setVisible(False)
        layout.addWidget(self.map_warn_label)

        # --- the two tables ------------------------------------------------ #
        split = QtWidgets.QSplitter(QtCore.Qt.Horizontal)

        # Left: the hardware. One row per physical button the pad reports, with a
        # live pressed indicator so you can name a switch by flipping it.
        left = QtWidgets.QWidget()
        lv = QtWidgets.QVBoxLayout(left)
        lv.setContentsMargins(0, 0, 0, 0)
        lv.addWidget(QtWidgets.QLabel(
            "<b>Physical buttons</b> — press one on the controller to find it, "
            "then type a name for it."))
        self.map_btn_table = QtWidgets.QTableWidget(0, 4)
        self.map_btn_table.setHorizontalHeaderLabels(
            ["#", "Your name for it", "Now", "Assigned to"])
        self.map_btn_table.verticalHeader().setVisible(False)
        self.map_btn_table.horizontalHeader().setStretchLastSection(True)
        self.map_btn_table.itemChanged.connect(self._map_on_name_edited)
        lv.addWidget(self.map_btn_table)
        split.addWidget(left)

        # Right: the software. One row per command the control loop actually reads.
        right = QtWidgets.QWidget()
        rv = QtWidgets.QVBoxLayout(right)
        rv.setContentsMargins(0, 0, 0, 0)
        rv.addWidget(QtWidgets.QLabel(
            "<b>Actions</b> — choose the button for each, or leave it unmapped."))
        self.map_cmd_table = QtWidgets.QTableWidget(len(CONTROLLER_COMMANDS), 4)
        self.map_cmd_table.setHorizontalHeaderLabels(
            ["Action", "Kind", "Button", ""])
        self.map_cmd_table.verticalHeader().setVisible(False)
        self.map_cmd_table.horizontalHeader().setStretchLastSection(False)
        self.map_cmd_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.map_cmd_combos: Dict[str, QtWidgets.QComboBox] = {}
        self.map_learn_btns: Dict[str, QtWidgets.QPushButton] = {}
        for row, (cmd, label, kind) in enumerate(CONTROLLER_COMMANDS):
            self.map_cmd_table.setItem(row, 0, QtWidgets.QTableWidgetItem(label))
            kind_item = QtWidgets.QTableWidgetItem(
                {"latch": "switch", "toggle": "toggle"}.get(kind, "press"))
            kind_item.setToolTip({
                "latch": "switch: the action follows the switch position "
                         "(on while up). Bind a switch, not a button -- a "
                         "momentary button would turn it off on release.",
                "toggle": "toggle: each press flips the state and the release "
                          "is ignored, so a momentary button acts as a switch.",
            }.get(kind,
                  "press: the action fires once each time the button goes down."))
            self.map_cmd_table.setItem(row, 1, kind_item)
            combo = QtWidgets.QComboBox()
            # Bind cmd by default arg: a bare closure over the loop variable would
            # leave every row pointing at the last command.
            combo.currentIndexChanged.connect(
                lambda _i, c=cmd: self._map_on_assign_changed(c))
            self.map_cmd_table.setCellWidget(row, 2, combo)
            self.map_cmd_combos[cmd] = combo
            learn = QtWidgets.QPushButton("Learn")
            learn.setCheckable(True)
            learn.setToolTip("Click, then press the button you want for this action.")
            learn.clicked.connect(lambda _c=False, c=cmd: self._map_learn(c))
            self.map_cmd_table.setCellWidget(row, 3, learn)
            self.map_learn_btns[cmd] = learn
        self.map_cmd_table.resizeColumnsToContents()
        rv.addWidget(self.map_cmd_table)
        split.addWidget(right)
        split.setSizes([420, 620])
        layout.addWidget(split, 1)

        # --- axes: detect-by-moving -------------------------------------- #
        layout.addWidget(QtWidgets.QLabel(
            "<b>Sticks</b> — click Detect, then move only that control fully in "
            "the named direction. The axis that moves most is locked in."))

        self.map_axis_table = QtWidgets.QTableWidget(len(AXIS_CONTROLS), 5)
        self.map_axis_table.setHorizontalHeaderLabels(
            ["Control", "Axis", "Live", "Reversed", "Detect"])
        self.map_axis_table.verticalHeader().setVisible(False)
        self.map_axis_table.horizontalHeader().setStretchLastSection(False)
        self.map_axis_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.map_axis_combos: Dict[str, QtWidgets.QComboBox] = {}
        self.map_axis_invert: Dict[str, QtWidgets.QCheckBox] = {}
        self.map_axis_detect_btns: Dict[str, QtWidgets.QPushButton] = {}
        self.map_axis_live: Dict[str, QtWidgets.QTableWidgetItem] = {}
        for row, (key, label, prompt, _) in enumerate(AXIS_CONTROLS):
            item = QtWidgets.QTableWidgetItem(label)
            item.setToolTip(prompt)
            self.map_axis_table.setItem(row, 0, item)

            combo = QtWidgets.QComboBox()
            combo.currentIndexChanged.connect(
                lambda _i, k=key: self._map_on_axis_changed(k))
            self.map_axis_table.setCellWidget(row, 1, combo)
            self.map_axis_combos[key] = combo

            live = QtWidgets.QTableWidgetItem("—")
            live.setFlags(live.flags() & ~QtCore.Qt.ItemIsEditable)
            self.map_axis_table.setItem(row, 2, live)
            self.map_axis_live[key] = live

            chk = QtWidgets.QCheckBox()
            chk.setToolTip(
                "Tick if this surface deflects the wrong way on the aircraft.\n"
                "Kept separate from detection, so re-detecting the axis will not "
                "undo a reversal you found by flying it."
                if key != "throttle" else
                "Throttle direction comes from its calibration, not a sign — "
                "re-detect the throttle instead of using this.")
            chk.setEnabled(key != "throttle")
            chk.toggled.connect(lambda on, k=key: self._map_on_invert_changed(k, on))
            holder = QtWidgets.QWidget()
            hl = QtWidgets.QHBoxLayout(holder)
            hl.setContentsMargins(0, 0, 0, 0)
            hl.addWidget(chk)
            hl.setAlignment(QtCore.Qt.AlignCenter)
            self.map_axis_table.setCellWidget(row, 3, holder)
            self.map_axis_invert[key] = chk

            det = QtWidgets.QPushButton("Detect")
            det.setCheckable(True)
            det.setToolTip(prompt)
            det.clicked.connect(lambda _c=False, k=key: self._map_detect_axis(k))
            self.map_axis_table.setCellWidget(row, 4, det)
            self.map_axis_detect_btns[key] = det
        self.map_axis_table.resizeColumnsToContents()
        self.map_axis_table.setMaximumHeight(
            self.map_axis_table.horizontalHeader().height()
            + self.map_axis_table.rowHeight(0) * len(AXIS_CONTROLS) + 4)
        layout.addWidget(self.map_axis_table)

        self.map_axis_status = QtWidgets.QLabel(
            "Centre the sticks before detecting. Changes take effect on the "
            "connected controller as soon as you press Save.")
        self.map_axis_status.setWordWrap(True)
        self.map_axis_status.setStyleSheet("color: #555;")
        layout.addWidget(self.map_axis_status)

        self._map_reload_names()
        return w

    # ----- Mapping tab: state plumbing ------------------------------------- #
    def _map_ctype(self) -> str:
        return self.map_ctype_combo.currentData() or "rc"

    def _map_reload_names(self) -> None:
        """Repopulate the saved-map dropdown for the selected controller type and
        select whichever map is marked active (or the built-in placeholder)."""
        maps, active = load_controller_maps()
        ctype = self._map_ctype()
        self._map_loading = True
        self.map_name_combo.clear()
        # None as item data means "no saved map -- use the built-in profile".
        self.map_name_combo.addItem("<built-in layout>", None)
        for name in sorted(n for n, bm in maps.items() if bm.controller_type == ctype):
            self.map_name_combo.addItem(name, name)
        want = active.get(ctype)
        idx = self.map_name_combo.findData(want) if want else 0
        self.map_name_combo.setCurrentIndex(idx if idx >= 0 else 0)
        self._map_loading = False
        self._map_load_selected()

    def _map_load_selected(self) -> None:
        """Make the dropdown's selection the working draft."""
        name = self.map_name_combo.currentData()
        ctype = self._map_ctype()
        if name is None:
            self._map_draft = default_controller_map(ctype)
        else:
            maps, _ = load_controller_maps()
            self._map_draft = maps.get(name) or default_controller_map(ctype)
        # Remembered separately from the draft's own name, because the built-in
        # placeholder has a display name but no identity in the combo (data None).
        self._map_selected: Optional[str] = name
        self._map_dirty = False
        self._map_cancel_learn()
        self._map_cancel_detect()
        self._map_refresh()

    def _map_on_ctype_changed(self) -> None:
        if self._map_loading:
            return
        self._map_reload_names()

    def _map_on_name_changed(self) -> None:
        if self._map_loading:
            return
        if self._map_dirty and not self._map_confirm_discard():
            return
        self._map_load_selected()

    def _map_confirm_discard(self) -> bool:
        resp = QtWidgets.QMessageBox.question(
            self, "Discard changes?",
            "This map has unsaved changes. Discard them?",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No)
        if resp == QtWidgets.QMessageBox.Yes:
            return True
        # Put the dropdown back on the map still being edited, with signals muted
        # so this handler does not re-enter and ask again.
        self._map_loading = True
        idx = self.map_name_combo.findData(self._map_selected)
        self.map_name_combo.setCurrentIndex(idx if idx >= 0 else 0)
        self._map_loading = False
        return False

    def _map_button_count(self) -> int:
        """How many button rows to show.

        Prefer what the connected pad reports. With nothing connected, fall back to
        the highest index the map or the built-in profile mentions, so an existing
        layout is still fully editable offline instead of showing an empty table.
        """
        if self._map_nbuttons > 0:
            return self._map_nbuttons
        used = [i for i in self._map_draft.commands.values() if i is not None and i >= 0]
        used += [i for i in CONTROLLER_PROFILES[self._map_ctype()].buttons.values()]
        used += list(self._map_draft.names.keys())
        return max(used or [0]) + 1

    def _map_axis_count(self) -> int:
        """Axis rows to offer. Same offline fallback logic as the button count:
        prefer what the device reports, else cover every index already in use."""
        if self._map_axis_values:
            return len(self._map_axis_values)
        prof = CONTROLLER_PROFILES[self._map_ctype()]
        used = [i for i in self._map_draft.axes.values() if i is not None and i >= 0]
        used += [prof.roll_axis, prof.pitch_axis, prof.yaw_axis, prof.throttle_axis,
                 prof.trim_roll_axis, prof.trim_pitch_axis, prof.trim_yaw_axis]
        return max([i for i in used if i >= 0] or [0]) + 1

    def _map_refresh(self) -> None:
        """Rebuild both tables from the draft. Cheap enough to do wholesale."""
        n = self._map_button_count()
        assigned: Dict[int, List[str]] = {}
        labels = {c: l for c, l, _ in CONTROLLER_COMMANDS}
        for cmd, idx in self._map_draft.commands.items():
            if idx is not None and idx >= 0:
                assigned.setdefault(idx, []).append(labels.get(cmd, cmd))

        self._map_loading = True
        self.map_btn_table.setRowCount(n)
        for i in range(n):
            num = QtWidgets.QTableWidgetItem(str(i))
            num.setFlags(num.flags() & ~QtCore.Qt.ItemIsEditable)
            self.map_btn_table.setItem(i, 0, num)
            self.map_btn_table.setItem(
                i, 1, QtWidgets.QTableWidgetItem(self._map_draft.names.get(i, "")))
            live = QtWidgets.QTableWidgetItem("● pressed" if i in self._map_pressed else "")
            live.setFlags(live.flags() & ~QtCore.Qt.ItemIsEditable)
            self.map_btn_table.setItem(i, 2, live)
            use = QtWidgets.QTableWidgetItem(", ".join(sorted(assigned.get(i, []))) or "—")
            use.setFlags(use.flags() & ~QtCore.Qt.ItemIsEditable)
            self.map_btn_table.setItem(i, 3, use)
        self.map_btn_table.resizeColumnsToContents()

        for cmd, combo in self.map_cmd_combos.items():
            combo.blockSignals(True)
            combo.clear()
            combo.addItem("— unmapped —", UNMAPPED)
            for i in range(n):
                combo.addItem(self._map_draft.button_label(i), i)
            cur = self._map_draft.commands.get(cmd, UNMAPPED)
            idx = combo.findData(cur if cur is not None and cur >= 0 else UNMAPPED)
            combo.setCurrentIndex(idx if idx >= 0 else 0)
            combo.blockSignals(False)
        self.map_cmd_table.resizeColumnsToContents()

        n_axes = self._map_axis_count()
        for key, _, _, _ in AXIS_CONTROLS:
            combo = self.map_axis_combos[key]
            combo.blockSignals(True)
            combo.clear()
            combo.addItem("— none —", UNMAPPED)
            for i in range(n_axes):
                combo.addItem(f"axis {i}", i)
            cur = self._map_draft.axes.get(key, UNMAPPED)
            at = combo.findData(cur if cur is not None and cur >= 0 else UNMAPPED)
            combo.setCurrentIndex(at if at >= 0 else 0)
            combo.blockSignals(False)
            chk = self.map_axis_invert[key]
            chk.blockSignals(True)
            chk.setChecked(bool(self._map_draft.inverted.get(key)))
            chk.blockSignals(False)
        self._map_loading = False
        self._map_refresh_axis_live()

        self._map_refresh_warnings()
        self._map_update_enabled()

    def _map_refresh_warnings(self) -> None:
        warns = self._map_draft.warnings()
        self.map_warn_label.setVisible(bool(warns))
        if warns:
            self.map_warn_label.setText("⚠ " + "<br>⚠ ".join(warns))

    def _map_update_enabled(self) -> None:
        """Arm lock: while the motor is armed, the layout is frozen.

        Reassigning arm_latch or override_latch with a live motor could strand the
        aircraft in override with no way back, so the whole tab goes read-only
        rather than trying to allow the 'safe' subset of edits.
        """
        locked = self._map_armed
        self.map_lock_label.setVisible(locked)
        if locked:
            self.map_lock_label.setText(
                "Motor is ARMED — button mapping is locked. Disarm to edit.")
        self.map_btn_table.setEditTriggers(
            QtWidgets.QAbstractItemView.NoEditTriggers if locked
            else QtWidgets.QAbstractItemView.DoubleClicked
            | QtWidgets.QAbstractItemView.SelectedClicked
            | QtWidgets.QAbstractItemView.EditKeyPressed)
        for combo in self.map_cmd_combos.values():
            combo.setEnabled(not locked)
        for b in self.map_learn_btns.values():
            b.setEnabled(not locked)
        for combo in self.map_axis_combos.values():
            combo.setEnabled(not locked)
        for key, chk in self.map_axis_invert.items():
            # Throttle has no invert: its direction lives in the calibration.
            chk.setEnabled(not locked and key != "throttle")
        # Detection asks for full-scale stick movement -- including full throttle
        # -- so it is gated on disarm like everything else on this tab.
        for b in self.map_axis_detect_btns.values():
            b.setEnabled(not locked)
        for b in (self.map_save_btn, self.map_saveas_btn,
                  self.map_delete_btn, self.map_revert_btn):
            b.setEnabled(not locked)
        self.map_ctype_combo.setEnabled(not locked)
        self.map_name_combo.setEnabled(not locked)
        self.map_save_btn.setText("Save *" if self._map_dirty and not locked else "Save")

    # ----- Mapping tab: edits ---------------------------------------------- #
    def _map_on_name_edited(self, item) -> None:
        if self._map_loading or item.column() != 1:
            return
        idx = item.row()
        text = item.text().strip()
        if text:
            self._map_draft.names[idx] = text
        else:
            self._map_draft.names.pop(idx, None)
        self._map_dirty = True
        # The label feeds the assignment dropdowns, so they have to be rebuilt.
        self._map_refresh()

    def _map_on_assign_changed(self, cmd: str) -> None:
        if self._map_loading:
            return
        combo = self.map_cmd_combos[cmd]
        self._map_draft.commands[cmd] = int(combo.currentData())
        self._map_dirty = True
        self._map_refresh()

    def _map_learn(self, cmd: str) -> None:
        """Arm capture-by-press for one command. Clicking an already-armed row
        cancels, so the button is its own escape hatch."""
        if self._map_learn_cmd == cmd:
            self._map_cancel_learn()
            return
        self._map_cancel_learn()
        self._map_learn_cmd = cmd
        b = self.map_learn_btns[cmd]
        b.setChecked(True)
        b.setText("press…")

    def _map_cancel_learn(self) -> None:
        cmd = self._map_learn_cmd
        self._map_learn_cmd = None
        if cmd and cmd in self.map_learn_btns:
            b = self.map_learn_btns[cmd]
            b.setChecked(False)
            b.setText("Learn")

    def _on_buttons_state(self, payload) -> None:
        """Worker-thread sample of the controller's discrete inputs.

        Qt marshals this onto the GUI thread for us, which is the reason the worker
        emits instead of the GUI polling pygame: SDL joystick state belongs to the
        thread that opened the device.
        """
        armed, nbuttons, pressed = payload
        was_pressed, was_n, was_armed = self._map_pressed, self._map_nbuttons, self._map_armed
        self._map_pressed, self._map_nbuttons, self._map_armed = pressed, nbuttons, armed

        # Learn resolves on the rising edge only, so holding a switch down does not
        # keep re-capturing, and a switch already up when you click Learn is ignored
        # until you actually flip it.
        if self._map_learn_cmd and not self._map_armed:
            new = pressed - was_pressed
            if new:
                cmd = self._map_learn_cmd
                self._map_draft.commands[cmd] = min(new)
                self._map_dirty = True
                self._map_cancel_learn()
                self._map_refresh()
                return

        if pressed != was_pressed or nbuttons != was_n:
            self._map_refresh()
        elif armed != was_armed:
            self._map_update_enabled()

    # ----- Mapping tab: axis detection ------------------------------------- #
    def _map_on_axis_changed(self, key: str) -> None:
        if self._map_loading:
            return
        self._map_draft.axes[key] = int(self.map_axis_combos[key].currentData())
        self._map_dirty = True
        self._map_refresh()

    def _map_on_invert_changed(self, key: str, on: bool) -> None:
        if self._map_loading:
            return
        self._map_draft.inverted[key] = bool(on)
        self._map_dirty = True
        self._map_update_enabled()

    def _map_detect_axis(self, key: str) -> None:
        """Start (or cancel) detect-by-moving for one flight axis."""
        if self._map_detect and self._map_detect["key"] == key:
            self._map_cancel_detect("Detection cancelled.")
            return
        self._map_cancel_detect()
        if self._map_armed:
            # Belt and braces: the buttons are already disabled while armed, but
            # detecting the throttle means shoving the stick to full, so this one
            # gets an explicit refusal rather than trusting a widget's state.
            self.map_axis_status.setText("Cannot detect while the motor is ARMED.")
            return
        if not self._map_axis_values:
            self.map_axis_status.setText(
                "No controller data. Connect (and enable the controller) first.")
            self.map_axis_detect_btns[key].setChecked(False)
            return
        prompt = {k: p for k, _, p, _ in AXIS_CONTROLS}[key]
        self._map_detect = {
            "key": key,
            "phase": "baseline",
            "until": time.monotonic() + AXIS_DETECT_BASELINE_S,
            "baseline": list(self._map_axis_values),
            "peak": [0.0] * len(self._map_axis_values),
        }
        self.map_axis_detect_btns[key].setChecked(True)
        self.map_axis_detect_btns[key].setText("cancel")
        self.map_axis_status.setText(f"Hold still — sampling rest position… ({prompt} next)")

    def _map_cancel_detect(self, message: str = "") -> None:
        if self._map_detect:
            btn = self.map_axis_detect_btns[self._map_detect["key"]]
            btn.setChecked(False)
            btn.setText("Detect")
        self._map_detect = None
        if message:
            self.map_axis_status.setText(message)

    def _on_axes_state(self, payload) -> None:
        """~25 Hz axis sample from the worker: live readout plus detection."""
        armed, values = payload
        self._map_axis_values = values
        if armed != self._map_armed:
            self._map_armed = armed
            if armed:
                self._map_cancel_detect("Motor armed — detection stopped.")
            self._map_update_enabled()
        self._map_refresh_axis_live()
        if self._map_detect:
            self._map_detect_step(values)

    def _map_detect_step(self, values: Tuple[float, ...]) -> None:
        """One sample of the detection state machine.

        Driven by incoming samples rather than a QTimer: if the controller stops
        reporting, detection simply stops advancing instead of timing out against
        a clock that keeps running with no data behind it.
        """
        st = self._map_detect
        now = time.monotonic()
        if len(values) != len(st["baseline"]):
            self._map_cancel_detect("Controller axis count changed — detection aborted.")
            return

        if st["phase"] == "baseline":
            st["baseline"] = list(values)
            if now >= st["until"]:
                st["phase"] = "capture"
                st["until"] = now + AXIS_DETECT_CAPTURE_S
            return

        # Rank by *peak deviation from rest*, not by crossing a fixed threshold.
        # This is the lesson already learned in interlink_tester.py: the
        # InterLink-X throttle and rear knobs rest near an end-stop (~+0.8), so
        # they can only travel ~0.2 and would never cross a 0.30 threshold. Peak
        # deviation catches an end-stop axis as readily as a centred one.
        for i, v in enumerate(values):
            d = v - st["baseline"][i]
            if abs(d) > abs(st["peak"][i]):
                st["peak"][i] = d

        lead = max(range(len(st["peak"])), key=lambda i: abs(st["peak"][i]))
        remaining = st["until"] - now
        if remaining > 0:
            self.map_axis_status.setText(
                f"Move it now — {remaining:.1f}s  (leading: axis {lead}, "
                f"deviation {st['peak'][lead]:+.2f})")
            return

        self._map_detect_finish(st, lead)

    def _map_detect_finish(self, st: dict, lead: int) -> None:
        key = st["key"]
        dev = st["peak"][lead]
        label = {k: l for k, l, _, _ in AXIS_CONTROLS}[key]
        if abs(dev) < AXIS_DETECT_MIN_DEV:
            self._map_cancel_detect(
                f"Barely any movement seen (peak {dev:+.2f}) — nothing assigned for "
                f"{label}. Centre the sticks, click Detect, then move it fully.")
            return

        self._map_draft.axes[key] = lead
        if key == "throttle":
            # Throttle carries no sign: it is calibrated by raw endpoints, so
            # direction and non-full-scale travel are captured in one go. Rest is
            # idle (motor off), the far end of the swing is full.
            self._map_draft.throttle_idle_raw = round(st["baseline"][lead], 3)
            self._map_draft.throttle_full_raw = round(st["baseline"][lead] + dev, 3)
            self._map_draft.axis_signs[key] = 1.0
            detail = (f"idle {self._map_draft.throttle_idle_raw:+.2f} → "
                      f"full {self._map_draft.throttle_full_raw:+.2f}")
        else:
            # Orient so the gesture the user was asked to make produces the
            # logical sign this codebase expects (see AXIS_CONTROLS). Note this
            # does NOT touch `inverted` -- an airframe reversal survives a
            # re-detect.
            want = {k: s for k, _, _, s in AXIS_CONTROLS}[key]
            observed = 1.0 if dev > 0 else -1.0
            self._map_draft.axis_signs[key] = want * observed
            detail = f"sign {self._map_draft.axis_signs[key]:+.0f} (peak {dev:+.2f})"

        self._map_dirty = True
        self._map_cancel_detect()
        self._map_refresh()
        self.map_axis_status.setText(f"{label} → axis {lead}, {detail}. Press Save to apply.")

    def _map_refresh_axis_live(self) -> None:
        """Update just the Live column. Called at 25 Hz, so it deliberately does
        not go through the full _map_refresh table rebuild."""
        for key, _, _, _ in AXIS_CONTROLS:
            idx = self._map_draft.axes.get(key, UNMAPPED)
            item = self.map_axis_live.get(key)
            if item is None:
                continue
            if idx is None or idx < 0 or idx >= len(self._map_axis_values):
                item.setText("—")
            else:
                item.setText(f"{self._map_axis_values[idx]:+.2f}")

    # ----- Mapping tab: persistence ---------------------------------------- #
    def _map_commit(self, name: str) -> None:
        """Write the draft to disk under `name`, mark it active, and push it to a
        running worker so the change is live without reconnecting."""
        maps, active = load_controller_maps()
        self._map_draft.name = name
        self._map_draft.controller_type = self._map_ctype()
        maps[name] = self._map_draft
        active[self._map_ctype()] = name
        try:
            save_controller_maps(maps, active)
        except OSError as exc:
            QtWidgets.QMessageBox.warning(
                self, "Save failed", f"Could not write {CONTROLLER_MAPS_FILE}:\n{exc}")
            return
        self._map_dirty = False
        # Only push to a live session, and only if it is using this controller
        # type -- otherwise the edit is for a layout the session is not reading.
        if self._worker_running and self.worker.config.controller_type == self._map_ctype():
            self.worker.post("set_controller_map", self._map_draft)
        self._map_reload_names()

    def _map_save(self) -> None:
        name = self.map_name_combo.currentData()
        if name is None:
            # Nothing to overwrite -- the built-in entry is a placeholder, not a
            # saved map, so this degrades into Save As rather than silently
            # inventing a name.
            self._map_save_as()
            return
        self._map_commit(name)

    def _map_save_as(self) -> None:
        suggested = self._map_draft.name if self.map_name_combo.currentData() else \
            f"{CONTROLLER_PROFILES[self._map_ctype()].label} custom"
        name, ok = QtWidgets.QInputDialog.getText(
            self, "Save button map", "Name for this map:",
            QtWidgets.QLineEdit.Normal, suggested)
        name = (name or "").strip()
        if not ok or not name:
            return
        maps, _ = load_controller_maps()
        if name in maps:
            resp = QtWidgets.QMessageBox.question(
                self, "Overwrite?", f"A map named '{name}' already exists. Replace it?",
                QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
                QtWidgets.QMessageBox.No)
            if resp != QtWidgets.QMessageBox.Yes:
                return
        # Save As copies: keep the draft's assignments but under the new identity.
        self._map_draft = ControllerMap(name=name, controller_type=self._map_ctype(),
                                    commands=dict(self._map_draft.commands),
                                    names=dict(self._map_draft.names))
        self._map_commit(name)

    def _map_delete(self) -> None:
        name = self.map_name_combo.currentData()
        if name is None:
            return
        resp = QtWidgets.QMessageBox.question(
            self, "Delete map", f"Delete the saved map '{name}'?",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No)
        if resp != QtWidgets.QMessageBox.Yes:
            return
        maps, active = load_controller_maps()
        maps.pop(name, None)
        # Deleting the active map falls back to the built-in layout rather than
        # leaving 'active' pointing at something that no longer exists.
        if active.get(self._map_ctype()) == name:
            active.pop(self._map_ctype(), None)
            if self._worker_running:
                self.worker.post("set_controller_map", None)
        try:
            save_controller_maps(maps, active)
        except OSError as exc:
            QtWidgets.QMessageBox.warning(
                self, "Delete failed", f"Could not write {CONTROLLER_MAPS_FILE}:\n{exc}")
            return
        self._map_reload_names()

    def _map_revert(self) -> None:
        resp = QtWidgets.QMessageBox.question(
            self, "Revert to built-in",
            "Load this controller's built-in layout? Saved maps are kept, but "
            "the controller will use the built-in one until you save again.",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No)
        if resp != QtWidgets.QMessageBox.Yes:
            return
        maps, active = load_controller_maps()
        active.pop(self._map_ctype(), None)
        try:
            save_controller_maps(maps, active)
        except OSError:
            pass
        if self._worker_running:
            self.worker.post("set_controller_map", None)
        self._map_reload_names()

    def _build_flightdata_tab(self) -> QtWidgets.QWidget:
        """Offline flight-log viewer: pick clipped/raw flights from the log folder,
        plot each as a stacked gyro/accel/deflection figure in its own sub-tab,
        with a clip_flights-style textual readout in the console pane."""
        w = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(w)

        # Collapse bar: hide the whole selection area to give the plots the tab.
        bar = QtWidgets.QHBoxLayout()
        self.fd_toggle_btn = QtWidgets.QToolButton()
        self.fd_toggle_btn.setText(" Hide selection controls")
        self.fd_toggle_btn.setCheckable(True)
        self.fd_toggle_btn.setToolButtonStyle(QtCore.Qt.ToolButtonTextBesideIcon)
        self.fd_toggle_btn.setArrowType(QtCore.Qt.DownArrow)
        self.fd_toggle_btn.toggled.connect(self._fd_toggle_controls)
        bar.addWidget(self.fd_toggle_btn)
        bar.addStretch(1)
        layout.addLayout(bar)

        # Vertical splitter: drag the divider to trade selection space for plots.
        vsplit = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        layout.addWidget(vsplit, 1)

        # Selection controls live in their own (collapsible) container.
        self.fd_controls = QtWidgets.QWidget()
        sel_layout = QtWidgets.QVBoxLayout(self.fd_controls)
        sel_layout.setContentsMargins(0, 0, 0, 0)

        # Folder row + rescan.
        top = QtWidgets.QHBoxLayout()
        top.addWidget(QtWidgets.QLabel("Log folder:"))
        self.fd_dir_edit = QtWidgets.QLineEdit(LOGS_DIR)
        top.addWidget(self.fd_dir_edit, 1)
        browse_btn = QtWidgets.QPushButton("Browse...")
        browse_btn.clicked.connect(self._fd_browse)
        rescan_btn = QtWidgets.QPushButton("Rescan")
        rescan_btn.clicked.connect(self._fd_rescan)
        top.addWidget(browse_btn)
        top.addWidget(rescan_btn)
        sel_layout.addLayout(top)

        # Filter + selection list.
        filt = QtWidgets.QHBoxLayout()
        filt.addWidget(QtWidgets.QLabel("Filter:"))
        self.fd_match_edit = QtWidgets.QLineEdit()
        self.fd_match_edit.setPlaceholderText("name substring, e.g. a date 20260722")
        self.fd_match_edit.returnPressed.connect(self._fd_rescan)
        filt.addWidget(self.fd_match_edit, 1)
        sel_layout.addLayout(filt)

        self.fd_list = QtWidgets.QListWidget()
        self.fd_list.setSelectionMode(
            QtWidgets.QAbstractItemView.ExtendedSelection)
        sel_layout.addWidget(self.fd_list, 1)

        act = QtWidgets.QHBoxLayout()
        plot_btn = QtWidgets.QPushButton("Plot selected")
        plot_btn.clicked.connect(self._fd_plot_selected)
        selall_btn = QtWidgets.QPushButton("Select all")
        selall_btn.clicked.connect(self.fd_list.selectAll)
        save_btn = QtWidgets.QPushButton("Save auto clips [raw f#]")
        save_btn.setToolTip("Save the selected auto-detected [raw f#] flights to the "
                            "clipped folder (grouped by day). Each clip gets the full "
                            "set of log files (all CSV streams + Console.txt).")
        save_btn.clicked.connect(self._fd_save_clips)
        onedrive_btn = QtWidgets.QPushButton("Sync to OneDrive")
        onedrive_btn.setToolTip(
            "Mirror every per-day folder in the local clipped/ folder to OneDrive "
            "(Glider/Data/FlightData/<day>). A day folder that already exists on "
            "OneDrive is REPLACED so the newest local clips win. Syncs to the Mac's "
            "OneDrive via the Parallels share -- no web upload needed.")
        onedrive_btn.clicked.connect(self._fd_sync_onedrive)
        clear_btn = QtWidgets.QPushButton("Clear plots")
        clear_btn.clicked.connect(self._fd_clear_plots)
        act.addWidget(plot_btn)
        act.addWidget(selall_btn)
        act.addWidget(save_btn)
        act.addWidget(onedrive_btn)
        act.addStretch(1)
        act.addWidget(clear_btn)
        sel_layout.addLayout(act)

        # Series picker: toggle which traces are shown on the plotted figures.
        # Keys are the exact matplotlib line labels used by flight_plots.build_figure,
        # so toggling just flips line visibility on the already-built figures.
        series_box = QtWidgets.QGroupBox("Series to plot")
        series_grid = QtWidgets.QGridLayout(series_box)
        series_groups = (
            ("Rates (deg/s)", (
                "gyro roll", "gyro pitch", "gyro yaw",
                "cmd roll", "cmd pitch", "cmd yaw")),
            ("Accel (g)", ("acc x", "acc y", "acc z")),
            ("Surfaces / throttle", (
                "aileron", "elevator", "aileron2", "rudder", "throttle")),
            # Only present on logs recorded after the EKF attitude columns were
            # added; the checkboxes are harmless on older logs because
            # _fd_apply_series_to_fig matches on label and simply finds no such line.
            ("Attitude (deg)", ("att roll", "att pitch", "att yaw")),
        )
        self.fd_series_checks: Dict[str, QtWidgets.QCheckBox] = {}
        for col, (group_name, labels) in enumerate(series_groups):
            series_grid.addWidget(QtWidgets.QLabel(f"<b>{group_name}</b>"), 0, col)
            for r, lbl in enumerate(labels, start=1):
                cb = QtWidgets.QCheckBox(lbl)
                cb.setChecked(True)
                cb.toggled.connect(self._fd_apply_series_all)
                self.fd_series_checks[lbl] = cb
                series_grid.addWidget(cb, r, col)
        # Select all / clear all convenience row.
        series_btns = QtWidgets.QHBoxLayout()
        fd_all_btn = QtWidgets.QPushButton("All series")
        fd_all_btn.clicked.connect(lambda: self._fd_set_all_series(True))
        fd_none_btn = QtWidgets.QPushButton("No series")
        fd_none_btn.clicked.connect(lambda: self._fd_set_all_series(False))
        series_btns.addWidget(fd_all_btn)
        series_btns.addWidget(fd_none_btn)
        series_btns.addStretch(1)
        # Row index derived from the longest group rather than hardcoded: the
        # checkboxes occupy rows 1..len(labels), so a fixed row silently lands on
        # top of the last checkbox of the tallest column as soon as any group
        # grows. That is what hid "cmd yaw" (6th entry of the rates column) --
        # the box existed and toggled correctly, it was just covered by these
        # buttons, which span every column.
        series_btn_row = max(len(labels) for _, labels in series_groups) + 1
        series_grid.addLayout(series_btns, series_btn_row, 0, 1, len(series_groups))
        sel_layout.addWidget(series_box)

        # Manual clip: for flights the auto-detector misses, read the start/end off
        # a plotted [full] session's x-axis (seconds-into-flight) and save that span
        # as a clip for the selected session.
        man = QtWidgets.QHBoxLayout()
        man.addWidget(QtWidgets.QLabel("Manual clip  start:"))
        self.fd_man_start = QtWidgets.QDoubleSpinBox()
        self.fd_man_start.setRange(0.0, 100000.0)
        self.fd_man_start.setDecimals(1)
        self.fd_man_start.setSuffix(" s")
        # Entering a start/end also snaps the current plot's x-axis to that span.
        self.fd_man_start.editingFinished.connect(self._fd_zoom_to_range)
        man.addWidget(self.fd_man_start)
        man.addWidget(QtWidgets.QLabel("end:"))
        self.fd_man_end = QtWidgets.QDoubleSpinBox()
        self.fd_man_end.setRange(0.0, 100000.0)
        self.fd_man_end.setDecimals(1)
        self.fd_man_end.setSuffix(" s")
        self.fd_man_end.setValue(60.0)
        self.fd_man_end.editingFinished.connect(self._fd_zoom_to_range)
        man.addWidget(self.fd_man_end)
        zoom_btn = QtWidgets.QPushButton("Zoom to range")
        zoom_btn.setToolTip("Set the current plot's x-axis to start..end and rescale y "
                            "to fit (also happens when you enter the times).")
        zoom_btn.clicked.connect(self._fd_zoom_to_range)
        man.addWidget(zoom_btn)
        man_btn = QtWidgets.QPushButton("Save manual clip")
        man_btn.setToolTip("Clip the selected session to this start/end span (in the "
                           "plot's seconds-into-flight axis) and save it.")
        man_btn.clicked.connect(self._fd_save_manual_clip)
        man.addWidget(man_btn)
        man.addStretch(1)
        sel_layout.addLayout(man)

        vsplit.addWidget(self.fd_controls)

        # Output split: nested plot tabs (left) + console readout (right).
        split = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        self.fd_plot_tabs = QtWidgets.QTabWidget()
        split.addWidget(self.fd_plot_tabs)
        self.fd_console = QtWidgets.QPlainTextEdit()
        self.fd_console.setReadOnly(True)
        self.fd_console.setMaximumBlockCount(4000)
        cfont = QtGui.QFont("monospace"); cfont.setStyleHint(QtGui.QFont.Monospace)
        self.fd_console.setFont(cfont)
        split.addWidget(self.fd_console)
        split.setStretchFactor(0, 4)
        split.setStretchFactor(1, 1)
        vsplit.addWidget(split)

        # Let the plots keep any extra height; start with a compact control pane.
        vsplit.setStretchFactor(0, 0)
        vsplit.setStretchFactor(1, 1)
        vsplit.setSizes([320, 680])

        self._fd_refs: List["flight_plots.FlightRef"] = []
        self._fd_rescan()
        return w

    def _fd_browse(self) -> None:
        # Force Qt's own dialog: the native GTK folder picker can hang here (it
        # only appears after a terminal interrupt), so DontUseNativeDialog keeps
        # Browse responsive.
        d = QtWidgets.QFileDialog.getExistingDirectory(
            self, "Select log folder", self.fd_dir_edit.text(),
            QtWidgets.QFileDialog.DontUseNativeDialog)
        if d:
            self.fd_dir_edit.setText(d)
            self._fd_rescan()

    def _fd_rescan(self) -> None:
        """Enumerate plottable flights (clipped sets + raw sessions) into the list."""
        directory = self.fd_dir_edit.text().strip() or "."
        match = self.fd_match_edit.text().strip() or None
        self.fd_list.clear()
        try:
            # Clips always live in the fixed CLIPPED_DIR (grouped by day), not
            # under whatever log folder is being scanned, so saved clips reappear
            # here regardless of the scan directory.
            self._fd_refs = flight_plots.enumerate_flights(
                directory, match, clipped_dir=CLIPPED_DIR)
        except Exception as exc:  # noqa: BLE001 - surface any scan error to the pane
            self._fd_refs = []
            self.fd_console.appendPlainText(f"[scan error] {exc}")
            return
        for ref in self._fd_refs:
            item = QtWidgets.QListWidgetItem(f"[{ref.source}] {ref.name}")
            self.fd_list.addItem(item)
        self.fd_console.appendPlainText(
            f"Scanned {os.path.abspath(directory)}: "
            f"{len(self._fd_refs)} plottable flight(s)")

    def _fd_clear_plots(self) -> None:
        while self.fd_plot_tabs.count():
            widget = self.fd_plot_tabs.widget(0)
            self.fd_plot_tabs.removeTab(0)
            widget.deleteLater()

    def _fd_plot_selected(self) -> None:
        rows = sorted(i.row() for i in self.fd_list.selectedIndexes())
        if not rows:
            self.fd_console.appendPlainText("Select one or more flights to plot.")
            return
        self._fd_clear_plots()
        for row in rows:
            ref = self._fd_refs[row]
            try:
                fd = flight_plots.load_flight_ref(ref)
                self.fd_console.appendPlainText(flight_plots.describe_flight(fd))
                fig = flight_plots.build_figure(fd)
            except Exception as exc:  # noqa: BLE001 - report bad file, keep going
                self.fd_console.appendPlainText(f"[plot error] {ref.name}: {exc}")
                continue
            page = QtWidgets.QWidget()
            vbox = QtWidgets.QVBoxLayout(page)
            canvas = FigureCanvas(fig)
            # Stash the canvas on the page so zoom-to-range can reach the active
            # tab's figure, and enable mouse-wheel zoom (toolbar gives pan/box-zoom).
            page._fd_canvas = canvas
            # Honour the current series selection on this freshly built figure.
            self._fd_apply_series_to_fig(fig)
            canvas.mpl_connect("scroll_event", self._fd_on_scroll)
            toolbar = NavigationToolbar(canvas, page)
            vbox.addWidget(toolbar)
            vbox.addWidget(canvas, 1)
            # Short tab label; full name is the tooltip.
            label = ref.name if len(ref.name) <= 24 else ref.name[:22] + "…"
            idx = self.fd_plot_tabs.addTab(page, label)
            self.fd_plot_tabs.setTabToolTip(idx, ref.name)
        if self.fd_plot_tabs.count():
            self.fd_plot_tabs.setCurrentIndex(0)

    # ---- layout helpers ------------------------------------------------ #
    def _fd_toggle_controls(self, hidden: bool) -> None:
        """Collapse/expand the selection controls so the plots can use the tab."""
        self.fd_controls.setVisible(not hidden)
        self.fd_toggle_btn.setArrowType(
            QtCore.Qt.RightArrow if hidden else QtCore.Qt.DownArrow)
        self.fd_toggle_btn.setText(
            " Show selection controls" if hidden else " Hide selection controls")

    # ---- series selection ---------------------------------------------- #
    def _fd_set_all_series(self, checked: bool) -> None:
        """Check/uncheck every series box at once (toggled signals fire once each,
        but re-applying is cheap and keeps the plots in sync)."""
        for cb in self.fd_series_checks.values():
            cb.blockSignals(True)
            cb.setChecked(checked)
            cb.blockSignals(False)
        self._fd_apply_series_all()

    def _fd_apply_series_all(self) -> None:
        """Apply the current series selection to every open plot tab."""
        for i in range(self.fd_plot_tabs.count()):
            page = self.fd_plot_tabs.widget(i)
            canvas = getattr(page, "_fd_canvas", None)
            if canvas is not None:
                self._fd_apply_series_to_fig(canvas.figure)
                canvas.draw_idle()

    def _fd_apply_series_to_fig(self, fig) -> None:
        """Show/hide traces on one figure per the checkboxes, rebuild each axis's
        legend to only list visible traces, then rescale y to the visible data."""
        enabled = {lbl for lbl, cb in self.fd_series_checks.items() if cb.isChecked()}
        for ax in fig.axes:
            for line in ax.get_lines():
                lbl = line.get_label()
                if lbl in self.fd_series_checks:
                    line.set_visible(lbl in enabled)
            # Rebuild legend from visible, explicitly-labelled traces (event
            # marker lines are auto-labelled with a leading "_" and excluded).
            handles = [ln for ln in ax.get_lines()
                       if ln.get_visible() and not ln.get_label().startswith("_")]
            leg = ax.get_legend()
            if handles:
                ax.legend(handles=handles, fontsize=6,
                          ncol=min(len(handles), 4), loc="upper right")
            elif leg is not None:
                leg.remove()
        self._fd_autoscale_y(fig)

    # ---- interactive zoom helpers -------------------------------------- #
    def _fd_current_canvas(self):
        """The FigureCanvas of the currently shown plot tab, or None."""
        page = self.fd_plot_tabs.currentWidget()
        return getattr(page, "_fd_canvas", None) if page is not None else None

    @staticmethod
    def _fd_autoscale_y(fig) -> None:
        """Rescale each axis's y-limits to fit only the data inside the current
        x-limits (ignoring the vertical event marker lines), so zooming/panning in x
        keeps the traces filling the plot."""
        for ax in fig.axes:
            x0, x1 = ax.get_xlim()
            lo = hi = None
            for line in ax.get_lines():
                if not line.get_visible():
                    continue
                xd = line.get_xdata()
                # Skip vertical event markers (axvline: two points, equal x).
                if len(xd) == 2 and xd[0] == xd[1]:
                    continue
                yd = line.get_ydata()
                for x, y in zip(xd, yd):
                    if x0 <= x <= x1:
                        if lo is None or y < lo:
                            lo = y
                        if hi is None or y > hi:
                            hi = y
            if lo is None or hi is None:
                continue
            if hi == lo:
                pad = 1.0 if hi == 0 else abs(hi) * 0.1
            else:
                pad = (hi - lo) * 0.08
            ax.set_ylim(lo - pad, hi + pad)

    def _fd_on_scroll(self, event) -> None:
        """Mouse-wheel zoom on the shared x-axis, centred on the cursor; y rescales
        to the new window. Scroll up zooms in, down zooms out."""
        ax = event.inaxes
        if ax is None or event.xdata is None:
            return
        fig = ax.figure
        scale = 0.8 if event.button == "up" else 1.25
        x0, x1 = ax.get_xlim()
        xc = event.xdata
        ax.set_xlim(xc - (xc - x0) * scale, xc + (x1 - xc) * scale)  # sharex -> all
        self._fd_autoscale_y(fig)
        fig.canvas.draw_idle()

    def _fd_zoom_to_range(self) -> None:
        """Snap the active plot's x-axis to the manual clip start/end and rescale y
        to fit. Fired when the manual clip times are entered, or via the button."""
        canvas = self._fd_current_canvas()
        if canvas is None:
            return
        start, end = self.fd_man_start.value(), self.fd_man_end.value()
        if end <= start:
            return
        fig = canvas.figure
        for ax in fig.axes:
            ax.set_xlim(start, end)
        self._fd_autoscale_y(fig)
        canvas.draw_idle()

    def _write_clip_files(self, prefix: str, name: str, idx: int, out_dir: str,
                          window) -> Tuple[int, int]:
        """Write every log file for one flight window into out_dir: all CSV streams
        plus the Console.txt. Returns (n_files_written, n_dense_data_rows). Files are
        always written when their source exists (even a header-only Events for a
        flight with no events), so each clip carries a complete, consistent set. The
        dense-row count (accel/motor/etc.) is the 'did we actually capture flight
        data' signal used to warn on an empty manual clip."""
        anchors = flight_plots.cf._harvest_breakpoints(prefix)
        n_files = n_rows = 0
        for suffix, cf_keyed in flight_plots.cf.STREAMS.items():
            out_path = os.path.join(out_dir, f"{name}_flight{idx}_{suffix}.csv")
            kept = flight_plots.cf._clip_stream(
                prefix, suffix, cf_keyed, window, anchors, out_path)
            if kept is not None:
                n_files += 1
                if cf_keyed:
                    n_rows += kept
        con_path = os.path.join(out_dir, f"{name}_flight{idx}_Console.txt")
        if flight_plots.cf._clip_console(prefix, window, anchors, con_path) is not None:
            n_files += 1
        return n_files, n_rows

    def _fd_sync_onedrive(self) -> None:
        """Mirror each per-day folder under CLIPPED_DIR to the OneDrive FlightData
        folder, replacing any same-named day folder there so the newest local clips
        are what ends up on OneDrive. OneDrive lives on the Mac and is exposed to
        this VM through the Parallels share, so the copy syncs to the cloud (and the
        macOS MATLAB pipeline) with no manual web upload."""
        dest_root = ONEDRIVE_FLIGHTDATA_DIR
        # The share only resolves when the Mac is running and the folder is shared.
        share_root = "/media/psf/Home"
        if not os.path.isdir(share_root):
            self.fd_console.appendPlainText(
                "Sync to OneDrive: the Parallels Home share isn't mounted "
                f"({share_root} missing). Is the VM running under Parallels with "
                "Home-folder sharing on?")
            return
        # Only the per-day subfolders (YYYYMMDD or Misc_Flights) are mirrored.
        day_dirs = sorted(
            d for d in glob.glob(os.path.join(CLIPPED_DIR, "*"))
            if os.path.isdir(d)
            and re.fullmatch(r"\d{8}|Misc_Flights", os.path.basename(d)))
        if not day_dirs:
            self.fd_console.appendPlainText(
                "Sync to OneDrive: no day folders in the clipped folder to sync.")
            return
        try:
            os.makedirs(dest_root, exist_ok=True)
        except OSError as exc:
            self.fd_console.appendPlainText(
                f"Sync to OneDrive: can't create the destination folder: {exc}")
            return
        self.fd_console.appendPlainText(
            f"Sync to OneDrive -> {dest_root}")
        synced = failed = 0
        for src in day_dirs:
            day = os.path.basename(src)
            dst = os.path.join(dest_root, day)
            try:
                # Replace the whole day folder so removed/re-clipped flights don't
                # leave stale files behind: the newest local clips fully define it.
                if os.path.exists(dst):
                    shutil.rmtree(dst)
                shutil.copytree(src, dst)
            except OSError as exc:
                self.fd_console.appendPlainText(f"  [sync error] {day}: {exc}")
                failed += 1
                continue
            n = len([f for f in os.listdir(dst)
                     if os.path.isfile(os.path.join(dst, f))])
            self.fd_console.appendPlainText(f"  synced {day}/ ({n} files, replaced)")
            synced += 1
        self.fd_console.appendPlainText(
            f"OneDrive sync done: {synced} day folder(s), {failed} failed. "
            "OneDrive will upload them from the Mac.")

    def _fd_save_clips(self) -> None:
        """Write the selected auto-detected clips to CLIPPED_DIR, grouped into
        per-day subfolders (same scheme as the logs folder)."""
        rows = sorted(i.row() for i in self.fd_list.selectedIndexes())
        if not rows:
            self.fd_console.appendPlainText("Select one or more auto-detected clips to save.")
            return
        saved = skipped = 0
        for row in rows:
            ref = self._fd_refs[row]
            if ref.source != "raw" or ref.window is None:
                # Only the on-the-fly detected windows ("[raw fN]") are savable;
                # full sessions and already-saved clips are skipped.
                self.fd_console.appendPlainText(
                    f"  skip {ref.name}: not an auto-detected clip")
                skipped += 1
                continue
            name = os.path.basename(ref.prefix)
            out_dir = clipped_day_dir(name)
            idx = ref.flight_index or 1
            try:
                n_files, _ = self._write_clip_files(
                    ref.prefix, name, idx, out_dir, ref.window)
            except Exception as exc:  # noqa: BLE001 - report and keep going
                self.fd_console.appendPlainText(f"  [save error] {ref.name}: {exc}")
                skipped += 1
                continue
            rel = os.path.relpath(out_dir, CLIPPED_DIR)
            self.fd_console.appendPlainText(
                f"  saved {name}_flight{idx} ({n_files} files) -> clipped/{rel}/")
            saved += 1
        self.fd_console.appendPlainText(f"Saved {saved} clip(s), skipped {skipped}.")
        if saved:
            self._fd_rescan()

    def _fd_next_flight_index(self, out_dir: str, name: str) -> int:
        """Lowest 1-based flightN index not already present for this session in
        out_dir, so a manual clip never overwrites an existing (auto or manual) one."""
        used = set()
        for path in glob.glob(os.path.join(out_dir, f"{name}_flight*_*.csv")):
            m = re.search(rf"{re.escape(name)}_flight(\d+)_", os.path.basename(path))
            if m:
                used.add(int(m.group(1)))
        idx = 1
        while idx in used:
            idx += 1
        return idx

    def _fd_save_manual_clip(self) -> None:
        """Clip the selected session to the manual start/end span (plot seconds) and
        save it into the clipped date folder."""
        rows = sorted(i.row() for i in self.fd_list.selectedIndexes())
        if not rows:
            self.fd_console.appendPlainText("Select a session (its [full] entry) to clip manually.")
            return
        ref = self._fd_refs[rows[0]]
        if ref.source not in ("full", "raw"):
            self.fd_console.appendPlainText(
                f"  {ref.name}: pick a raw session's [full] entry to clip manually.")
            return
        start = self.fd_man_start.value()
        end = self.fd_man_end.value()
        if end <= start:
            self.fd_console.appendPlainText(
                f"  manual clip needs end > start (got {start:.1f}s .. {end:.1f}s).")
            return
        name = os.path.basename(ref.prefix)
        try:
            t0 = flight_plots.session_start_time(ref.prefix)
            window = (t0 + start, t0 + end)
            out_dir = clipped_day_dir(name)
            idx = self._fd_next_flight_index(out_dir, name)
            n_files, n_rows = self._write_clip_files(
                ref.prefix, name, idx, out_dir, window)
        except Exception as exc:  # noqa: BLE001 - report and stop
            self.fd_console.appendPlainText(f"  [manual clip error] {name}: {exc}")
            return
        if n_rows == 0:
            # Bad/empty range: remove the header-only files just written so no empty
            # clip is left behind (only this flightN's files match, idx being free).
            for path in glob.glob(os.path.join(out_dir, f"{name}_flight{idx}_*")):
                try:
                    os.remove(path)
                except OSError:
                    pass
            self.fd_console.appendPlainText(
                f"  {name}: no sensor data in {start:.1f}s..{end:.1f}s -- nothing saved.")
            return
        rel = os.path.relpath(out_dir, CLIPPED_DIR)
        self.fd_console.appendPlainText(
            f"  saved {name}_flight{idx} ({n_files} files, {start:.1f}s..{end:.1f}s) "
            f"-> clipped/{rel}/")
        self._fd_rescan()

    def _build_console_tab(self) -> QtWidgets.QWidget:
        w = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(w)
        self.console_view = QtWidgets.QPlainTextEdit()
        self.console_view.setReadOnly(True)
        self.console_view.setMaximumBlockCount(2000)
        self.console_view.setLineWrapMode(QtWidgets.QPlainTextEdit.WidgetWidth)
        font = QtGui.QFont("monospace"); font.setStyleHint(QtGui.QFont.Monospace)
        self.console_view.setFont(font)
        layout.addWidget(self.console_view)
        return w

    def _build_notes_tab(self) -> QtWidgets.QWidget:
        """Free-form flight notes that persist across launches. The file is
        loaded into the editor on startup (old notes reshow) and written back
        on close, so each session builds on the last."""
        w = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(w)
        layout.addWidget(QtWidgets.QLabel(f"Notes persist to {NOTES_FILE}"))
        self.notes_edit = QtWidgets.QPlainTextEdit()
        self.notes_edit.setLineWrapMode(QtWidgets.QPlainTextEdit.WidgetWidth)
        self.notes_edit.setPlaceholderText(
            "Flight notes, gain changes, trim values, observations...")
        layout.addWidget(self.notes_edit)

        btn_row = QtWidgets.QHBoxLayout()
        stamp_btn = QtWidgets.QPushButton("Insert timestamp")
        stamp_btn.clicked.connect(self._insert_notes_timestamp)
        save_btn = QtWidgets.QPushButton("Save notes now")
        save_btn.clicked.connect(self._save_notes)
        btn_row.addWidget(stamp_btn)
        btn_row.addStretch(1)
        btn_row.addWidget(save_btn)
        layout.addLayout(btn_row)

        self._load_notes()
        return w

    def _load_notes(self) -> None:
        """Populate the notes editor from the persisted file, if it exists."""
        try:
            with open(NOTES_FILE, "r", encoding="utf-8") as fh:
                self.notes_edit.setPlainText(fh.read())
            self.notes_edit.moveCursor(QtGui.QTextCursor.End)
        except FileNotFoundError:
            pass
        except OSError as exc:
            self._append_console(f"[notes] could not load {NOTES_FILE}: {exc}\n")

    def _save_notes(self) -> None:
        """Write the full notes buffer back to the persisted file."""
        try:
            with open(NOTES_FILE, "w", encoding="utf-8") as fh:
                fh.write(self.notes_edit.toPlainText())
        except OSError as exc:
            self._append_console(f"[notes] could not save {NOTES_FILE}: {exc}\n")

    def _insert_notes_timestamp(self) -> None:
        """Drop a dated header at the cursor to separate entries."""
        stamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.notes_edit.moveCursor(QtGui.QTextCursor.End)
        prefix = "\n" if self.notes_edit.toPlainText() else ""
        self.notes_edit.insertPlainText(f"{prefix}--- {stamp} ---\n")
        self.notes_edit.moveCursor(QtGui.QTextCursor.End)

    # ----- GUI actions ----------------------------------------------------- #
    def _collect_config(self) -> SessionConfig:
        values = {field: self._widget_value(wdg)
                  for field, wdg in self._setup_bindings().items()}
        # Only field needing massaging: a blank/whitespace URI means "default".
        values["uri"] = str(values["uri"]).strip() or DEFAULT_URI
        return SessionConfig(gains=self._gains_from_spins(), **values)

    def _toggle_connection(self) -> None:
        if self.connect_btn.text() == "Connect":
            config = self._collect_config()
            # Size the plot buffers so every stream shows the same time window at
            # its chosen log period (called before the worker starts producing).
            self.buffers.configure({
                "controller": config.period_controller_ms,
                "motor": config.period_motor_ms,
                "connection": config.period_connection_ms,
                "accelerometer": config.period_accelerometer_ms,
            }, window_s=config.plot_window_s)
            self.worker.start(config)
            self.connect_btn.setText("Disconnect")
            self.connect_btn.setEnabled(False)  # re-enabled on connected(True)
            self._set_status("Connecting...")
        else:
            self.worker.stop()
            self.connect_btn.setEnabled(False)

    def _on_throttle_changed(self, value: int) -> None:
        self.throttle_label.setText(f"{value}%")
        self.worker.update_live(throttle=value / 100.0)

    def _send_setpoints(self) -> None:
        self.worker.update_live(
            setpoint_roll=self.sp_roll.value(),
            setpoint_pitch=self.sp_pitch.value(),
            setpoint_yaw=self.sp_yaw.value(),
        )
        self.worker.post("event", ("AUTONOMOUS_SETPOINTS",
                                   self.sp_roll.value(), self.sp_pitch.value(), self.sp_yaw.value()))

    def _gains_from_spins(self) -> PidGains:
        """Read the Control-tab PID grid into a PidGains (columns 1..4 = kp/ki/kd/kff)."""
        gains = PidGains()
        for axis in PID_AXES:
            g = getattr(gains, axis)
            for col, term in enumerate(PID_TERMS, start=1):
                setattr(g, term, self.pid_spins[(axis, col)].value())
        return gains

    def _apply_pid(self) -> None:
        self.worker.post("apply_pid", self._gains_from_spins())

    def _save_pid_defaults(self) -> None:
        """Persist the current grid as the launch defaults. Does not push to the
        deck -- use Apply PID for that; this only changes what loads next time."""
        gains = self._gains_from_spins()
        try:
            save_default_gains(gains)
        except OSError as exc:
            self._append_console(f"[pid] could not save {PID_DEFAULTS_FILE}: {exc}\n")
            return
        self._append_console(f"[pid] saved launch defaults to {PID_DEFAULTS_FILE}\n")

    def _zero_override(self) -> None:
        for slider in self.override_sliders.values():
            slider.setValue(0)

    def _make_trim_handler(self, axis: str):
        """Build a valueChanged slot bound to one surface (roll/pitch/yaw).
        Keeps that axis' slider + spinbox in sync and applies the value live.
        Signals are blocked while mirroring so the two widgets don't ping-pong."""
        def _handler(value: int) -> None:
            value = int(value)
            for wdg in (self.trim_sliders[axis], self.trim_spins[axis]):
                if wdg.value() != value:
                    wdg.blockSignals(True)
                    wdg.setValue(value)
                    wdg.blockSignals(False)
            self.worker.post("set_trim", (axis, value))
        return _handler

    def _center_trims(self) -> None:
        """Reset all three surface trims to the neutral center and apply live."""
        for axis in TRIM_PARAMS:
            self.trim_sliders[axis].setValue(SERVO_TRIM_CENTER)

    def _on_trim_value(self, axis: str, value: int) -> None:
        """Reflect a trim read back from the deck (display only, no re-send)."""
        for wdg in (self.trim_sliders[axis], self.trim_spins[axis]):
            wdg.blockSignals(True)
            wdg.setValue(int(value))
            wdg.blockSignals(False)

    def _current_surface_map(self) -> Dict[str, int]:
        """Channel -> surface code as currently selected in the Control tab."""
        return {ch: int(self.surface_combos[ch].currentData()) for ch in SURFACE_MAP_CHANNELS}

    def _make_surface_map_handler(self, channel: str):
        """Build a slot that pushes this channel's surface + invert to the deck
        whenever its combo box or invert checkbox changes, and relabels the plot."""
        def _handler(*_args) -> None:
            surf = int(self.surface_combos[channel].currentData())
            invert = 1 if self.surface_invert_chks[channel].isChecked() else 0
            self.worker.post("set_surface_map", (channel, surf, invert))
            self.canvas.set_surface_map(self._current_surface_map())
        return _handler

    def _on_surface_map(self, channel: str, surf: int, invert: int) -> None:
        """Reflect the mixer map read back from the deck (display only, no re-send)."""
        combo = self.surface_combos[channel]
        idx = combo.findData(int(surf))
        combo.blockSignals(True)
        combo.setCurrentIndex(idx if idx >= 0 else 0)
        combo.blockSignals(False)
        chk = self.surface_invert_chks[channel]
        chk.blockSignals(True)
        chk.setChecked(bool(invert))
        chk.blockSignals(False)
        self.canvas.set_surface_map(self._current_surface_map())

    def _on_pid_value(self, axis: str, term: str, value: float) -> None:
        """Reflect an in-flight PID-tune knob change in the Control-tab spin box
        (display only, no re-send)."""
        col = PID_TERM_COL.get(term)
        if col is None:
            return
        spin = self.pid_spins.get((axis, col))
        if spin is None:
            return
        spin.blockSignals(True)
        spin.setValue(value)
        spin.blockSignals(False)

    # ----- worker signal handlers ------------------------------------------ #
    def _append_console(self, text: str) -> None:
        self.console_view.moveCursor(QtGui.QTextCursor.End)
        self.console_view.insertPlainText(text)
        self.console_view.moveCursor(QtGui.QTextCursor.End)

    def _set_status(self, text: str) -> None:
        self.statusBar().showMessage(text)

    def _on_telemetry(self, vbat: float, rssi: float) -> None:
        self.statusBar().showMessage(f"VBAT={vbat:.2f}V  RSSI={rssi:.0f}")

    def _on_connection_changed(self, ok: bool) -> None:
        self.connect_btn.setEnabled(True)
        self.connect_btn.setText("Disconnect" if ok else "Connect")
        self._set_connected_ui(ok)

    def _set_connected_ui(self, ok: bool) -> None:
        # Whether the worker loop is live. Posting to a stopped worker is not
        # harmless: the command queue survives, so a map posted now would be
        # applied at the start of the *next* session, overriding whatever was
        # selected then. The Mapping tab checks this before posting.
        self._worker_running = ok
        # Setup fields locked while connected (the Save-defaults button stays
        # live, so a session's settings can still be blessed after connecting).
        for wdg in self._setup_bindings().values():
            wdg.setEnabled(not ok)
        for wdg in (self.arm_btn, self.disarm_btn, self.bp_btn,
                    self.reconnect_btn,
                    self.throttle_slider, self.autonomous_chk,
                    self.override_chk, self.override_ctrl_chk,
                    self.trim_save_btn, self.surface_map_save_btn,
                    # Injector RUN controls only. Editing the test card stays
                    # live offline, which is the whole point of having it in the
                    # GUI -- you build the card at the desk, not at the field.
                    self.mv_arm_chk, self.mv_fire_btn, self.mv_abort_btn,
                    *self.trim_sliders.values(), *self.trim_spins.values(),
                    *self.surface_combos.values(),
                    *self.surface_invert_chks.values()):
            wdg.setEnabled(ok)
        if ok:
            # Hand the fresh worker the current card and run options: it starts
            # from the file-backed library, which may be several edits behind.
            self._mv_push()
            self.worker.post("inject_advance", self.mv_advance_chk.isChecked())
            self.worker.post("inject_stick_abort", self.mv_stick_abort_chk.isChecked())
            # The worker starts at PROPULSION_FAST_DEFAULT; push the checkbox so a
            # route chosen before connecting is the one actually flown.
            self.worker.post("propulsion_fast", self.prop_fast_chk.isChecked())
        else:
            # A new session must start disarmed, no matter how the last one ended.
            self.mv_arm_chk.blockSignals(True)
            self.mv_arm_chk.setChecked(False)
            self.mv_arm_chk.blockSignals(False)
            self._on_injector_state({"armed": False, "active": False,
                                     "selected_index": -1, "selected": "",
                                     "running": "", "saturated": False})

    def closeEvent(self, event) -> None:
        self._save_notes()
        self.worker.stop()
        if self.worker._thread:
            self.worker._thread.join(timeout=3.0)
        event.accept()


def main() -> None:
    app = QtWidgets.QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec() if hasattr(app, "exec") else app.exec_())


if __name__ == "__main__":
    main()

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
from dataclasses import dataclass, field
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


CONNECTION_WATCHDOG_TIMEOUT_S = 1.0
DEFAULT_PLOT_WINDOW_S = 20.0  # how much history each live plot shows


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
        self.controller.writerow(["cf_time_s", "gyro_roll", "gyro_pitch", "gyro_yaw", "set_roll", "set_pitch", "set_yaw"])

        self.motor.writerow(["# schema_version", CSV_SCHEMA_VERSION, "dataset", "motor"])
        # motor_m3 (the rudder surface) is appended last so older readers that index
        # servo_cmd at column 4 keep working; new readers pick up the rudder at 5.
        self.motor.writerow(["cf_time_s", "motor_m4", "motor_m1", "motor_m2", "servo_cmd", "motor_m3"])

        self.connection.writerow(["# schema_version", CSV_SCHEMA_VERSION, "dataset", "connection"])
        self.connection.writerow(["cf_time_s", "rssi", "vbat"])

        self.accelerometer.writerow(["# schema_version", CSV_SCHEMA_VERSION, "dataset", "accelerometer"])
        self.accelerometer.writerow(["cf_time_s", "acc_x", "acc_y", "acc_z"])

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

    def write_event(self, event_name: str, v1: object = "", v2: object = "", v3: object = "") -> None:
        host_time = datetime.now().isoformat(timespec="seconds")
        self.event.writerow([host_time, event_name, v1, v2, v3])

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

    def add_accel(self, ts, ax, ay, az):
        with self._lock:
            self.t_acc.append(ts)
            self.accx.append(ax); self.accy.append(ay); self.accz.append(az)

    def snapshot(self) -> Dict[str, list]:
        with self._lock:
            return {name: list(getattr(self, name)) for name in (
                "t_ctrl", "gyroroll", "gyropitch", "gyroyaw", "setroll", "setpitch", "setyaw",
                "t_motor", "motor1", "motor2", "motor3", "motor4", "thrust",
                "t_conn", "rssi",
                "t_acc", "accx", "accy", "accz",
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
        self.ax.set_xlabel("Time (s)"); self.ax.set_ylabel("deg/s"); self.ax.legend()

        (self.line_gyropitch,) = self.ax2.plot([], [], label="Pitch Rate", color="blue")
        (self.line_setpitch,) = self.ax2.plot([], [], label="Pitch Setpoint", color="red")
        self.ax2.set_ylim(-38, 38); self.ax2.set_title("Pitch")
        self.ax2.set_xlabel("Time (s)"); self.ax2.set_ylabel("deg/s"); self.ax2.legend()

        (self.line_gyroyaw,) = self.ax3.plot([], [], label="Yaw Rate", color="blue")
        (self.line_setyaw,) = self.ax3.plot([], [], label="Yaw Setpoint", color="red")
        self.ax3.set_ylim(-38, 38); self.ax3.set_title("Yaw")
        self.ax3.set_xlabel("Time (s)"); self.ax3.set_ylabel("deg/s"); self.ax3.legend()

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
    period_accelerometer_ms: int = 15
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
        self.last_override_servo = 0
        # Manual-override state: cache of last param value actually written
        # (dirty-check to avoid flooding the radio) + non-blocking servo slew.
        self._last_sent: Dict[str, int] = {}
        self._override_servo_cmd = 0
        self._last_override_emit: Optional[Tuple[int, int, int, int, int]] = None
        self._last_override_write = 0.0
        # Commander-fast override: when True, stream the raw motor/servo command on
        # the setpoint channel (responsive, needs modded firmware) instead of the
        # slower param-based motorPowerSet.* path. Toggled from the override tab.
        self._fast_override = True
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

    # ----- public API (called from GUI thread) ----------------------------- #
    def start(self, config: SessionConfig) -> None:
        if self._thread and self._thread.is_alive():
            return
        self.config = config
        self.profile = CONTROLLER_PROFILES.get(config.controller_type, XBOX_PROFILE)
        self._stop.clear()
        self.failsafe_active = False
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()

    def post(self, action: str, payload: object = None) -> None:
        """Queue a one-shot command for the worker loop."""
        self._cmd_queue.put((action, payload))

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
        self.cf.param.set_value("motorPowerSet.enable", "0")
        self.cf.param.set_value("flightmode.stabModeRoll", "0")
        self.cf.param.set_value("flightmode.stabModePitch", "0")
        self.cf.param.set_value("flightmode.stabModeYaw", "0")
        self._apply_pid_gains(self.config.gains)

        self.cf.param.set_value("fwActLpf.enable", "1" if self.config.fwactlpf_enable else "0")
        self.cf.param.set_value("fwActLpf.cutoffHz", max(0.1, self.config.fwactlpf_cutoff_hz))
        self.logs.write_event("FW_ACT_LPF", f"enable={int(self.config.fwactlpf_enable)}",
                              f"cutoffHz={self.config.fwactlpf_cutoff_hz}")

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

    def _apply_pid_gains(self, gains: PidGains) -> None:
        self.cf.param.set_value("pid_rate.pitch_kp", gains.pitch.kp)
        self.cf.param.set_value("pid_rate.pitch_ki", gains.pitch.ki)
        self.cf.param.set_value("pid_rate.pitch_kd", gains.pitch.kd)
        self.cf.param.set_value("pid_rate.pitch_kff", gains.pitch.kff)
        self.cf.param.set_value("pid_rate.yaw_kp", gains.yaw.kp)
        self.cf.param.set_value("pid_rate.yaw_ki", gains.yaw.ki)
        self.cf.param.set_value("pid_rate.yaw_kd", gains.yaw.kd)
        self.cf.param.set_value("pid_rate.yaw_kff", gains.yaw.kff)
        self.cf.param.set_value("pid_rate.roll_kp", gains.roll.kp)
        self.cf.param.set_value("pid_rate.roll_ki", gains.roll.ki)
        self.cf.param.set_value("pid_rate.roll_kd", gains.roll.kd)
        self.cf.param.set_value("pid_rate.roll_kff", gains.roll.kff)
        self.logs.write_event(
            "PID_APPLIED",
            f"pitch({gains.pitch.kp},{gains.pitch.ki},{gains.pitch.kd})",
            f"yaw({gains.yaw.kp},{gains.yaw.ki},{gains.yaw.kd})",
            f"roll({gains.roll.kp},{gains.roll.ki},{gains.roll.kd})",
        )
        self._log("PID gains applied.\n")

    def _create_log_configs(self) -> None:
        lg_controller = LogConfig(name="Controller", period_in_ms=self.config.period_controller_ms)
        for v in ("controller.r_roll", "controller.r_pitch", "controller.r_yaw",
                  "controller.pitchRate", "controller.rollRate", "controller.yawRate"):
            lg_controller.add_variable(v, "float")

        lg_motor = LogConfig(name="Motor", period_in_ms=self.config.period_motor_ms)
        for v in ("motor.m4", "motor.m1", "motor.m2", "motor.m3"):
            lg_motor.add_variable(v, "uint16_t")

        lg_connection = LogConfig(name="Connection", period_in_ms=self.config.period_connection_ms)
        lg_connection.add_variable("radio.rssi", "uint8_t")
        lg_connection.add_variable("pm.vbat", "float")

        lg_accel = LogConfig(name="Accelerometer", period_in_ms=self.config.period_accelerometer_ms)
        for v in ("acc.x", "acc.y", "acc.z"):
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

    def _bind_log_callbacks(self) -> None:
        self.cf.log.add_config(self.log_configs["controller"])
        self.log_configs["controller"].data_received_cb.add_callback(self._on_controller_log)
        self.cf.log.add_config(self.log_configs["motor"])
        self.log_configs["motor"].data_received_cb.add_callback(self._on_motor_log)
        self.cf.log.add_config(self.log_configs["connection"])
        self.log_configs["connection"].data_received_cb.add_callback(self._on_connection_log)
        self.cf.log.add_config(self.log_configs["accelerometer"])
        self.log_configs["accelerometer"].data_received_cb.add_callback(self._on_accel_log)

    def _start_enabled_logs(self) -> None:
        for key, enabled in self.log_enabled.items():
            if enabled:
                self.log_configs[key].start()
        time.sleep(0.1)

    # ----- log callbacks (run on cflib threads) ---------------------------- #
    def _on_controller_log(self, timestamp, data, _logconf):
        ts = timestamp / 1000.0
        gr = float(data["controller.r_roll"]); gp = float(data["controller.r_pitch"])
        gy = float(data["controller.r_yaw"]); sp = float(data["controller.pitchRate"])
        sr = float(data["controller.rollRate"]); sy = float(data["controller.yawRate"])
        self.buffers.add_controller(ts, gr, gp, gy, sr, sp, sy)
        self.logs.controller.writerow([ts, gr, gp, gy, sr, sp, sy])

    def _on_motor_log(self, timestamp, data, _logconf):
        ts = timestamp / 1000.0
        m4 = float(data["motor.m4"]); m1 = float(data["motor.m1"])
        m2 = float(data["motor.m2"]); m3 = float(data["motor.m3"])
        self.buffers.add_motor(ts, m1, m2, m3, m4, float(self.last_servo_value))
        self.logs.motor.writerow([ts, m4, m1, m2, self.last_servo_value, m3])

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
        self.buffers.add_accel(ts, ax, ay, az)
        self.logs.accelerometer.writerow([ts, ax, ay, az])

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
        if self.logs is not None:
            self.logs.write_breakpoint("FAILSAFE_DISARM")
            self.logs.write_event("FAILSAFE_DISARM", reason)
        try:
            if self.commander is not None:
                self.commander.send_setpoint(0.0, 0.0, 0.0, 0)
                self.commander.send_stop_setpoint()
        except Exception:
            pass
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
            elif self.config.use_controller:
                self._handle_controller_flight(live)
            else:
                # GUI-driven setpoints (no controller): autonomous or hold-zero.
                if live.autonomous:
                    self.commander.send_setpoint(
                        clamp(live.setpoint_roll, -self.config.roll_rate_limit, self.config.roll_rate_limit),
                        -clamp(live.setpoint_pitch, -self.config.pitch_rate_limit, self.config.pitch_rate_limit),
                        -clamp(live.setpoint_yaw, -self.config.yaw_rate_limit, self.config.yaw_rate_limit),
                        10001,
                    )
                else:
                    self.commander.send_setpoint(0.0, 0.0, 0.0, 10001)
                if live.motor_armed:
                    self._set_bl_motor_throttle(live.throttle)

            time.sleep(0.01)

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
        elif action == "persist_trim":
            self._persist_trim()
        elif action == "set_surface_map":
            channel, surf, invert = payload
            surf = int(clamp(int(surf), 0, 3))
            invert = 1 if invert else 0
            self._set_param_if_changed(SURFACE_MAP_SURF_PARAM[channel], surf)
            self._set_param_if_changed(SURFACE_MAP_INV_PARAM[channel], invert)
            self.logs.write_event("SURFACE_MAP_SET", channel, f"surface={surf}", f"invert={invert}")
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
        throttle = clamp(throttle, 0.0, 1.0)
        target = int(throttle * MAX_MOTOR_CMD)
        step = 1000
        if target == self.last_servo_value:
            return
        direction = 1 if target > self.last_servo_value else -1
        for value in range(self.last_servo_value, target, step * direction):
            self.cf.param.set_value("servo.servoAngle", value)
            time.sleep(0.005)
        self.cf.param.set_value("servo.servoAngle", target)
        self.last_servo_value = target

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
            axis = self._pid_axis

            def _done(name, success):
                self._log(f"PID {'saved' if success else 'SAVE FAILED'} ({name}).\n")
            for term in ("kp", "ki", "kd"):
                param = f"pid_rate.{axis}_{term}"
                try:
                    self.cf.param.persistent_store(param, _done)
                except Exception as exc:
                    self._log(f"PID persist error ({param}): {exc}\n")
            self.logs.write_event("PID_PERSIST_REQUEST", axis)
        else:
            self._persist_trim()

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
            self.commander.send_setpoint(
                clamp(live.setpoint_roll, -self.config.roll_rate_limit, self.config.roll_rate_limit),
                -clamp(live.setpoint_pitch, -self.config.pitch_rate_limit, self.config.pitch_rate_limit),
                -clamp(live.setpoint_yaw, -self.config.yaw_rate_limit, self.config.yaw_rate_limit),
                10001,
            )
        elif not live.trimmed:
            roll_cmd = roll_axis if abs(roll_axis) > JOYSTICK_DEADBAND else 0.0
            pitch_cmd = pitch_axis if abs(pitch_axis) > JOYSTICK_DEADBAND else 0.0
            yaw_cmd = yaw_axis if abs(yaw_axis) > JOYSTICK_DEADBAND else 0.0
            self.commander.send_setpoint(
                roll_cmd * self.config.roll_rate_limit,
                -pitch_cmd * self.config.pitch_rate_limit,
                -yaw_cmd * self.config.yaw_rate_limit,
                10001,
            )

        if live.motor_armed:
            self._set_bl_motor_throttle(live.throttle)

    # ----- teardown -------------------------------------------------------- #
    def _shutdown(self) -> None:
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
# Button-mapping reference (shown in the Mapping tab)
# --------------------------------------------------------------------------- #
# Columns: (action, Xbox 360 control, InterLink-X (RC) control)
# Xbox reflects the common SDL2/xpad layout for a wired 360 pad on Linux; the
# InterLink-X column reflects the indices verified with interlink_tester.py
# --wizard (7 axes, 17 buttons, no hat). Indices can differ by driver/OS -- if a
# controller behaves oddly, verify with `jstest /dev/input/js0` (or the tester).
# The active mapping is whichever profile is picked by the Setup-tab dropdown
# (see XBOX_PROFILE / RC_PROFILE).
BUTTON_MAPPING = [
    ("Roll command", "Right stick X (axis 3)", "Right stick X / aileron (axis 0)"),
    ("Pitch command", "Right stick Y (axis 4)", "Right stick Y / elevator (axis 1)"),
    ("Yaw command", "Left stick X (axis 0)", "Left stick X / rudder (axis 5)"),
    ("Throttle", "Left stick Y (axis 1) + D-pad Up/Down", "Left stick / throttle (axis 2, absolute)"),
    ("Trim ON", "A / green (button 0)", "button 12"),
    ("Trim OFF", "Y / yellow (button 3)", "button 11"),
    ("Arm motor", "LB / left bumper (button 4)", "button 5"),
    ("Disarm motor", "RB / right bumper (button 5)", "button 6"),
    ("Enable autonomous mode", "X / blue (button 2)", "button 9"),
    ("Disable autonomous mode", "B / red (button 1)", "button 10"),
    ("Manual CSV breakpoint", "Right stick click / R3 (button 10)", "button 14"),
]


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

        self.tabs = QtWidgets.QTabWidget()
        self.setCentralWidget(self.tabs)
        self.tabs.addTab(self._scrollable(self._build_setup_tab()), "Setup")
        self.tabs.addTab(self._build_plots_tab(), "Live Plots")
        self.tabs.addTab(self._scrollable(self._build_control_tab()), "Control")
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

        self.connect_btn = QtWidgets.QPushButton("Connect")
        self.connect_btn.setStyleSheet("font-weight: bold; padding: 8px;")
        self.connect_btn.clicked.connect(self._toggle_connection)
        form.addRow(self.connect_btn)
        return w

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
        thr_layout = QtWidgets.QHBoxLayout(thr_box)
        self.throttle_slider = QtWidgets.QSlider(Qt.Horizontal)
        self.throttle_slider.setRange(0, 100)
        self.throttle_label = QtWidgets.QLabel("0%")
        self.throttle_slider.valueChanged.connect(self._on_throttle_changed)
        thr_layout.addWidget(self.throttle_slider); thr_layout.addWidget(self.throttle_label)
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
        defaults = PidGains()
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

    def _build_mapping_tab(self) -> QtWidgets.QWidget:
        w = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(w)
        table = QtWidgets.QTableWidget(len(BUTTON_MAPPING), 3)
        table.setHorizontalHeaderLabels(
            ["Action", "Xbox 360", "InterLink-X (RC)"])
        table.horizontalHeader().setStretchLastSection(True)
        table.verticalHeader().setVisible(False)
        table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        for row, (act, xbox, rc) in enumerate(BUTTON_MAPPING):
            table.setItem(row, 0, QtWidgets.QTableWidgetItem(act))
            table.setItem(row, 1, QtWidgets.QTableWidgetItem(xbox))
            table.setItem(row, 2, QtWidgets.QTableWidgetItem(rc))
        table.resizeColumnsToContents()
        layout.addWidget(table)
        return w

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
        series_grid.addLayout(series_btns, 6, 0, 1, len(series_groups))
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
        gains = PidGains()
        for axis in ("pitch", "yaw", "roll"):
            g = getattr(gains, axis)
            g.kp = self.pid_spins[(axis, 1)].value()
            g.ki = self.pid_spins[(axis, 2)].value()
            g.kd = self.pid_spins[(axis, 3)].value()
            g.kff = self.pid_spins[(axis, 4)].value()
        return SessionConfig(
            uri=self.uri_edit.text().strip() or DEFAULT_URI,
            filename_prefix=self.filename_edit.text(),
            use_controller=self.controller_chk.isChecked(),
            controller_type=self.controller_type_combo.currentData(),
            debug_controller_log=self.controller_debug_chk.isChecked(),
            fwactlpf_enable=self.lpf_enable_chk.isChecked(),
            fwactlpf_cutoff_hz=self.lpf_cutoff_spin.value(),
            roll_rate_limit=self.roll_limit_spin.value(),
            pitch_rate_limit=self.pitch_limit_spin.value(),
            yaw_rate_limit=self.yaw_limit_spin.value(),
            log_controller=self.log_controller_chk.isChecked(),
            log_motor=self.log_motor_chk.isChecked(),
            log_connection=self.log_connection_chk.isChecked(),
            log_accelerometer=self.log_accel_chk.isChecked(),
            period_controller_ms=self.period_controller_spin.value(),
            period_motor_ms=self.period_motor_spin.value(),
            period_connection_ms=self.period_connection_spin.value(),
            period_accelerometer_ms=self.period_accel_spin.value(),
            plot_window_s=self.plot_window_spin.value(),
            gains=gains,
        )

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

    def _apply_pid(self) -> None:
        gains = PidGains()
        for axis in ("pitch", "yaw", "roll"):
            g = getattr(gains, axis)
            g.kp = self.pid_spins[(axis, 1)].value()
            g.ki = self.pid_spins[(axis, 2)].value()
            g.kd = self.pid_spins[(axis, 3)].value()
            g.kff = self.pid_spins[(axis, 4)].value()
        self.worker.post("apply_pid", gains)

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
        # Setup fields locked while connected.
        for wdg in (self.uri_edit, self.filename_edit, self.controller_chk,
                    self.controller_type_combo, self.controller_debug_chk,
                    self.lpf_enable_chk, self.lpf_cutoff_spin,
                    self.roll_limit_spin, self.pitch_limit_spin, self.yaw_limit_spin,
                    self.log_controller_chk, self.log_motor_chk,
                    self.log_connection_chk, self.log_accel_chk,
                    self.period_controller_spin, self.period_motor_spin,
                    self.period_connection_spin, self.period_accel_spin,
                    self.plot_window_spin):
            wdg.setEnabled(not ok)
        for wdg in (self.arm_btn, self.disarm_btn, self.bp_btn,
                    self.reconnect_btn,
                    self.throttle_slider, self.autonomous_chk,
                    self.override_chk, self.override_ctrl_chk,
                    self.trim_save_btn, self.surface_map_save_btn,
                    *self.trim_sliders.values(), *self.trim_spins.values(),
                    *self.surface_combos.values(),
                    *self.surface_invert_chks.values()):
            wdg.setEnabled(ok)

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

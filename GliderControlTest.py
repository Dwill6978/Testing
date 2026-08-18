import argparse
import csv
import logging
import time
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime
from typing import Dict, Optional, Tuple

import cflib.crtp
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import pygame
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

URI = uri_helper.uri_from_env(default="radio://0/80/2M/E7E7E7E7E7")
MAX_MOTOR_CMD = 65535
JOYSTICK_DEADBAND = 0.05
THROTTLE_STEP = 0.01
CSV_SCHEMA_VERSION = "glider_csv_v1"
CONNECTION_WATCHDOG_TIMEOUT_S = 1.0

logging.basicConfig(level=logging.ERROR)


@dataclass
class PidAxisGains:
    kp: float
    ki: float
    kd: float
    kff: float = 0.0


@dataclass
class PidGains:
    pitch: PidAxisGains = field(default_factory=lambda: PidAxisGains(kp=-300.0, ki=25.0, kd=0.0, kff=0.0))
    yaw: PidAxisGains = field(default_factory=lambda: PidAxisGains(kp=500.0, ki=25.0, kd=0.0, kff=0.0))
    roll: PidAxisGains = field(default_factory=lambda: PidAxisGains(kp=-500.0, ki=25.0, kd=0.0, kff=0.0))


@dataclass
class RuntimeState:
    trimmed: bool = False
    motor_armed: bool = False
    autonomous: bool = False
    throttle: float = 0.0
    last_servo_value: int = 0
    setpoint_pitch: float = 0.0
    setpoint_roll: float = 0.0
    setpoint_yaw: float = 0.0
    roll_rate_limit: float = 35.0
    pitch_rate_limit: float = 35.0
    yaw_rate_limit: float = 35.0
    failsafe_active: bool = False
    last_connection_seen_at: float = 0.0
    last_button_state: Dict[int, int] = field(default_factory=dict)
    last_hat_state: Tuple[int, int] = (0, 0)
    last_battery_print_at_s: float = 0.0


class CsvLogBundle:
    def __init__(self, prefix: str):
        self.controller_file = open(f"{prefix}_Controller.csv", "w", newline="")
        self.motor_file = open(f"{prefix}_Motor.csv", "w", newline="")
        self.connection_file = open(f"{prefix}_Connection.csv", "w", newline="")
        self.accelerometer_file = open(f"{prefix}_Accelerometer.csv", "w", newline="")
        self.event_file = open(f"{prefix}_Events.csv", "w", newline="")

        self.controller = csv.writer(self.controller_file)
        self.motor = csv.writer(self.motor_file)
        self.connection = csv.writer(self.connection_file)
        self.accelerometer = csv.writer(self.accelerometer_file)
        self.event = csv.writer(self.event_file)

        self.write_headers()

    def write_headers(self) -> None:
        self.controller.writerow(["# schema_version", CSV_SCHEMA_VERSION, "dataset", "controller"])
        self.controller.writerow(["cf_time_s", "gyro_roll", "gyro_pitch", "gyro_yaw", "set_roll", "set_pitch", "set_yaw"])

        self.motor.writerow(["# schema_version", CSV_SCHEMA_VERSION, "dataset", "motor"])
        self.motor.writerow(["cf_time_s", "motor_m4", "motor_m1", "motor_m2", "servo_cmd"])

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

    def close(self) -> None:
        self.controller_file.close()
        self.motor_file.close()
        self.connection_file.close()
        self.accelerometer_file.close()
        self.event_file.close()


class LivePlotter:
    def __init__(self, enabled: bool):
        self.enabled = enabled
        if not enabled:
            return

        self.gyroroll_data = deque(maxlen=200)
        self.gyropitch_data = deque(maxlen=200)
        self.gyroyaw_data = deque(maxlen=200)
        self.setpitch_data = deque(maxlen=200)
        self.setroll_data = deque(maxlen=200)
        self.setyaw_data = deque(maxlen=200)
        self.motor2_data = deque(maxlen=200)
        self.motor4_data = deque(maxlen=200)
        self.motor1_data = deque(maxlen=200)
        self.rssi_data = deque(maxlen=200)
        self.accx_data = deque(maxlen=200)
        self.accy_data = deque(maxlen=200)
        self.accz_data = deque(maxlen=200)
        self.timestamps = deque(maxlen=200)
        self.timestamps2 = deque(maxlen=200)
        self.timestamps3 = deque(maxlen=200)
        self.timestamps4 = deque(maxlen=200)

        self.fig, ((self.ax, self.ax2, self.ax3), (self.ax4, self.ax5, self.ax6)) = plt.subplots(2, 3, sharex=True)
        (self.line_gyroroll,) = self.ax.plot([], [], label="Roll Rate", color="blue")
        (self.line_setroll,) = self.ax.plot([], [], label="Roll Setpoint", color="red")
        self.ax.set_ylim(-38, 38)
        self.ax.set_title("Roll")
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("deg/s")
        self.ax.legend()

        (self.line_gyropitch,) = self.ax2.plot([], [], label="Pitch Rate", color="blue")
        (self.line_setpitch,) = self.ax2.plot([], [], label="Pitch Setpoint", color="red")
        self.ax2.set_ylim(-38, 38)
        self.ax2.set_title("Pitch")
        self.ax2.set_xlabel("Time (s)")
        self.ax2.set_ylabel("deg/s")
        self.ax2.legend()

        (self.line_gyroyaw,) = self.ax3.plot([], [], label="Yaw Rate", color="blue")
        (self.line_setyaw,) = self.ax3.plot([], [], label="Yaw Setpoint", color="red")
        self.ax3.set_ylim(-38, 38)
        self.ax3.set_title("Yaw")
        self.ax3.set_xlabel("Time (s)")
        self.ax3.set_ylabel("deg/s")
        self.ax3.legend()

        (self.line_motor1,) = self.ax4.plot([], [], label="Motor 1", color="red")
        (self.line_motor2,) = self.ax4.plot([], [], label="Motor 2", color="blue")
        (self.line_motor4,) = self.ax4.plot([], [], label="Motor 4", color="green")
        self.ax4.set_ylim(-10, 66000)
        self.ax4.set_title("Motor Commands")
        self.ax4.set_xlabel("Time (s)")
        self.ax4.set_ylabel("cmd")
        self.ax4.legend()

        (self.line_rssi,) = self.ax5.plot([], [], label="RSSI", color="red")
        self.ax5.set_title("RSSI")
        self.ax5.set_xlabel("Time (s)")
        self.ax5.set_ylabel("value")
        self.ax5.set_ylim(0, 60)
        self.ax5.legend()

        (self.line_accx,) = self.ax6.plot([], [], label="Acc X", color="red")
        (self.line_accy,) = self.ax6.plot([], [], label="Acc Y", color="blue")
        (self.line_accz,) = self.ax6.plot([], [], label="Acc Z", color="green")
        self.ax6.set_title("Accelerometer")
        self.ax6.set_xlabel("Time (s)")
        self.ax6.set_ylabel("g")
        self.ax6.set_ylim(-5.0, 5.0)
        self.ax6.legend()

        self.anim = animation.FuncAnimation(self.fig, self.update_plot, interval=20, cache_frame_data=False)
        plt.tight_layout()
        plt.show(block=False)

    def controller_point(self, ts: float, gyro_roll: float, gyro_pitch: float, gyro_yaw: float, set_roll: float, set_pitch: float, set_yaw: float) -> None:
        if not self.enabled:
            return
        self.timestamps.append(ts)
        self.gyroroll_data.append(gyro_roll)
        self.gyropitch_data.append(gyro_pitch)
        self.gyroyaw_data.append(gyro_yaw)
        self.setroll_data.append(set_roll)
        self.setpitch_data.append(set_pitch)
        self.setyaw_data.append(set_yaw)

    def motor_point(self, ts: float, m4: float, m1: float, m2: float) -> None:
        if not self.enabled:
            return
        self.timestamps2.append(ts)
        self.motor4_data.append(m4)
        self.motor1_data.append(m1)
        self.motor2_data.append(m2)

    def connection_point(self, ts: float, rssi: float) -> None:
        if not self.enabled:
            return
        self.timestamps3.append(ts)
        self.rssi_data.append(rssi)

    def accelerometer_point(self, ts: float, ax: float, ay: float, az: float) -> None:
        if not self.enabled:
            return
        self.timestamps4.append(ts)
        self.accx_data.append(ax)
        self.accy_data.append(ay)
        self.accz_data.append(az)

    def update_plot(self, _frame):
        if not self.enabled:
            return []

        self.line_gyroroll.set_data(self.timestamps, self.gyroroll_data)
        self.line_gyropitch.set_data(self.timestamps, self.gyropitch_data)
        self.line_gyroyaw.set_data(self.timestamps, self.gyroyaw_data)
        self.line_setroll.set_data(self.timestamps, self.setroll_data)
        self.line_setpitch.set_data(self.timestamps, self.setpitch_data)
        self.line_setyaw.set_data(self.timestamps, self.setyaw_data)

        self.line_motor1.set_data(self.timestamps2, self.motor1_data)
        self.line_motor2.set_data(self.timestamps2, self.motor2_data)
        self.line_motor4.set_data(self.timestamps2, self.motor4_data)

        self.line_rssi.set_data(self.timestamps3, self.rssi_data)

        self.line_accx.set_data(self.timestamps4, self.accx_data)
        self.line_accy.set_data(self.timestamps4, self.accy_data)
        self.line_accz.set_data(self.timestamps4, self.accz_data)

        for axis in (self.ax, self.ax2, self.ax3, self.ax4, self.ax5, self.ax6):
            axis.relim()
            axis.autoscale_view(scalex=True, scaley=False)

        return [
            self.line_gyroroll,
            self.line_gyropitch,
            self.line_gyroyaw,
            self.line_setroll,
            self.line_setpitch,
            self.line_setyaw,
            self.line_motor1,
            self.line_motor2,
            self.line_motor4,
            self.line_rssi,
            self.line_accx,
            self.line_accy,
            self.line_accz,
        ]

    def tick(self) -> None:
        if self.enabled:
            plt.pause(0.001)

    def close(self) -> None:
        if self.enabled:
            plt.close(self.fig)


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def prompt_float(question: str, default: float) -> float:
    raw = input(f"{question} [{default}]: ").strip()
    if not raw:
        return default
    return float(raw)


def prompt_bool(question: str, default: bool = True) -> bool:
    suffix = "Y/n" if default else "y/N"
    raw = input(f"{question} ({suffix}): ").strip().lower()
    if not raw:
        return default
    return raw in ("y", "yes", "1", "true", "t")


class GliderControlApp:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.logs: Optional[CsvLogBundle] = None
        self.plotter = LivePlotter(enabled=not args.no_plot)
        self.state = RuntimeState()
        self.gains = PidGains()

        self.cf = None
        self.commander = None
        self.joystick = None
        self.log_configs: Dict[str, LogConfig] = {}
        self.log_enabled = {
            "controller": True,
            "motor": True,
            "connection": True,
            "accelerometer": True,
        }

    def run(self) -> None:
        filename = input("Enter filename prefix for logs: ").strip()
        if not filename:
            filename = f"flight_{datetime.now().strftime('%Y%m%d_%H%M%S')}"

        self.logs = CsvLogBundle(filename)
        self.logs.write_event("SESSION_START", URI)

        cflib.crtp.init_drivers()
        self._setup_joystick_if_requested()
        self._print_control_mapping_banner()

        with SyncCrazyflie(URI, cf=Crazyflie(rw_cache="./cache")) as scf:
            self.cf = scf.cf
            self.commander = self.cf.commander
            self._bind_connection_callbacks()
            self.cf.console.receivedChar.add_callback(self._console_callback)
            print("Connected to Crazyflie:", URI)

            self._configure_flight_controller()
            self._prompt_rate_limits()
            self._create_log_configs()
            self._bind_log_callbacks()
            self._prompt_log_selection()
            self._start_enabled_logs()

            self.cf.param.set_value("usd.logging", "1")
            self.cf.platform.send_arming_request(True)
            self.state.last_connection_seen_at = time.monotonic()
            self.logs.write_breakpoint("SESSION_READY")
            print("Glider configured and ready.")

            try:
                self._main_control_loop()
            except KeyboardInterrupt:
                print("Stopping control session.")
            finally:
                self._shutdown()

    def _setup_joystick_if_requested(self) -> None:
        if not self.args.controller:
            return

        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            raise RuntimeError("--controller was set, but no joystick was detected.")

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        print("Using joystick:", self.joystick.get_name())

    def _print_control_mapping_banner(self) -> None:
        print("\n" + "=" * 72)
        print("Crazyflie Glider Control Mapping")
        print("=" * 72)
        print("Axes:")
        print("  Axis 3 -> Roll command")
        print("  Axis 4 -> Pitch command")
        print("  Axis 0 -> Yaw command")
        print("  Hat Up/Down -> Throttle +/- 0.01")
        print("Buttons:")
        print("  0 -> Trim ON (hold current input trim mode)")
        print("  3 -> Trim OFF")
        print("  4 -> Arm motor (logs FLIGHT_START_MOTOR_ARM breakpoint)")
        print("  5 -> Disarm motor (logs FLIGHT_END_MOTOR_DISARM breakpoint)")
        print("  2 -> Enable autonomous mode (prompts setpoints)")
        print("  1 -> Disable autonomous mode")
        print("  9 -> PID tuning prompt")
        print("  10 -> Manual CSV breakpoint marker")
        if self.args.controller:
            print("Controller input is ENABLED (--controller).")
        else:
            print("Controller input is DISABLED (run with --controller to enable).")
        print("=" * 72 + "\n")

    def _configure_flight_controller(self) -> None:
        self.cf.param.set_value("motorPowerSet.enable", "0")
        self.cf.param.set_value("flightmode.stabModeRoll", "0")
        self.cf.param.set_value("flightmode.stabModePitch", "0")
        self.cf.param.set_value("flightmode.stabModeYaw", "0")
        self._apply_pid_gains(self.gains)
        self._configure_output_lpf()

    def _configure_output_lpf(self) -> None:
        default_enable = True
        default_cutoff_hz = 30.0

        if self.args.fwactlpf_enable is not None:
            default_enable = self.args.fwactlpf_enable
        if self.args.fwactlpf_cutoff_hz is not None:
            default_cutoff_hz = self.args.fwactlpf_cutoff_hz

        print("Configure controller output low-pass filter (fwActLpf).")
        enabled = prompt_bool("Enable fwActLpf filter?", default=default_enable)
        cutoff_hz = max(0.1, prompt_float("fwActLpf cutoffHz", default_cutoff_hz))

        self.cf.param.set_value("fwActLpf.enable", "1" if enabled else "0")
        self.cf.param.set_value("fwActLpf.cutoffHz", cutoff_hz)

        self.logs.write_event("FW_ACT_LPF", f"enable={int(enabled)}", f"cutoffHz={cutoff_hz}")
        print(f"fwActLpf configured: enable={int(enabled)} cutoffHz={cutoff_hz}")

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

    def _create_log_configs(self) -> None:
        lg_controller = LogConfig(name="Controller", period_in_ms=10)
        lg_controller.add_variable("controller.r_roll", "float")
        lg_controller.add_variable("controller.r_pitch", "float")
        lg_controller.add_variable("controller.r_yaw", "float")
        lg_controller.add_variable("controller.pitchRate", "float")
        lg_controller.add_variable("controller.rollRate", "float")
        lg_controller.add_variable("controller.yawRate", "float")

        lg_motor = LogConfig(name="Motor", period_in_ms=10)
        lg_motor.add_variable("motor.m4", "uint16_t")
        lg_motor.add_variable("motor.m1", "uint16_t")
        lg_motor.add_variable("motor.m2", "uint16_t")

        lg_connection = LogConfig(name="Connection", period_in_ms=50)
        lg_connection.add_variable("radio.rssi", "uint8_t")
        lg_connection.add_variable("pm.vbat", "float")

        lg_accel = LogConfig(name="Accelerometer", period_in_ms=10)
        lg_accel.add_variable("acc.x", "float")
        lg_accel.add_variable("acc.y", "float")
        lg_accel.add_variable("acc.z", "float")

        self.log_configs = {
            "controller": lg_controller,
            "motor": lg_motor,
            "connection": lg_connection,
            "accelerometer": lg_accel,
        }

    def _bind_connection_callbacks(self) -> None:
        # Callbacks are fired by cflib if the radio link fails or drops unexpectedly.
        self.cf.connection_lost.add_callback(self._on_link_issue)
        self.cf.connection_failed.add_callback(self._on_link_issue)
        self.cf.disconnected.add_callback(self._on_link_issue)

    def _on_link_issue(self, link_uri: str, message: str = "") -> None:
        reason = f"LINK_ISSUE uri={link_uri} msg={message}"
        print(f"Connection issue detected: {reason}")
        self._failsafe_disarm(reason)

    def _failsafe_disarm(self, reason: str) -> None:
        if self.state.failsafe_active:
            return

        self.state.failsafe_active = True
        self.state.motor_armed = False
        self.state.autonomous = False
        self.state.throttle = 0.0

        if self.logs is not None:
            self.logs.write_breakpoint("FAILSAFE_DISARM")
            self.logs.write_event("FAILSAFE_DISARM", reason)

        # Best effort disarm: if link is still partially alive this will stop thrust.
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

        print("Failsafe disarm engaged.")

    def _prompt_rate_limits(self) -> None:
        print("Set commander angular rate limits in deg/s (used for manual and autonomous setpoints).")
        self.state.roll_rate_limit = abs(prompt_float("Roll rate limit", self.state.roll_rate_limit))
        self.state.pitch_rate_limit = abs(prompt_float("Pitch rate limit", self.state.pitch_rate_limit))
        self.state.yaw_rate_limit = abs(prompt_float("Yaw rate limit", self.state.yaw_rate_limit))
        self.logs.write_event(
            "RATE_LIMITS",
            f"roll={self.state.roll_rate_limit}",
            f"pitch={self.state.pitch_rate_limit}",
            f"yaw={self.state.yaw_rate_limit}",
        )

    def _bind_log_callbacks(self) -> None:
        self.cf.log.add_config(self.log_configs["controller"])
        self.log_configs["controller"].data_received_cb.add_callback(self._on_controller_log)

        self.cf.log.add_config(self.log_configs["motor"])
        self.log_configs["motor"].data_received_cb.add_callback(self._on_motor_log)

        self.cf.log.add_config(self.log_configs["connection"])
        self.log_configs["connection"].data_received_cb.add_callback(self._on_connection_log)

        self.cf.log.add_config(self.log_configs["accelerometer"])
        self.log_configs["accelerometer"].data_received_cb.add_callback(self._on_accel_log)

    def _prompt_log_selection(self) -> None:
        self.log_enabled["controller"] = prompt_bool("Log controller rates/setpoints?", default=True)
        self.log_enabled["motor"] = prompt_bool("Log motor data?", default=True)
        self.log_enabled["connection"] = prompt_bool("Log RSSI/VBAT?", default=True)
        self.log_enabled["accelerometer"] = prompt_bool("Log accelerometer data?", default=True)

        self.logs.write_event(
            "LOG_SELECTION",
            f"controller={self.log_enabled['controller']}",
            f"motor={self.log_enabled['motor']}",
            f"connection={self.log_enabled['connection']}",
        )

    def _start_enabled_logs(self) -> None:
        for key, enabled in self.log_enabled.items():
            if enabled:
                self.log_configs[key].start()
        time.sleep(0.1)

    def _on_controller_log(self, timestamp: int, data: Dict[str, float], _logconf: LogConfig) -> None:
        ts = timestamp / 1000.0
        gyro_roll = float(data["controller.r_roll"])
        gyro_pitch = float(data["controller.r_pitch"])
        gyro_yaw = float(data["controller.r_yaw"])
        set_pitch = float(data["controller.pitchRate"])
        set_roll = float(data["controller.rollRate"])
        set_yaw = float(data["controller.yawRate"])

        self.plotter.controller_point(ts, gyro_roll, gyro_pitch, gyro_yaw, set_roll, set_pitch, set_yaw)
        self.logs.controller.writerow([ts, gyro_roll, gyro_pitch, gyro_yaw, set_roll, set_pitch, set_yaw])

    def _on_motor_log(self, timestamp: int, data: Dict[str, float], _logconf: LogConfig) -> None:
        ts = timestamp / 1000.0
        m4 = float(data["motor.m4"])
        m1 = float(data["motor.m1"])
        m2 = float(data["motor.m2"])

        self.plotter.motor_point(ts, m4, m1, m2)
        self.logs.motor.writerow([ts, m4, m1, m2, self.state.last_servo_value])

    def _on_connection_log(self, timestamp: int, data: Dict[str, float], _logconf: LogConfig) -> None:
        ts = timestamp / 1000.0
        rssi = float(data["radio.rssi"])
        vbat = float(data["pm.vbat"])
        self.state.last_connection_seen_at = time.monotonic()

        self.plotter.connection_point(ts, rssi)
        self.logs.connection.writerow([ts, rssi, vbat])

        if (ts - self.state.last_battery_print_at_s) > 60.0 or vbat < 7.0:
            print(f"VBAT={vbat:.2f}V RSSI={rssi:.1f}")
            self.state.last_battery_print_at_s = ts

    def _on_accel_log(self, timestamp: int, data: Dict[str, float], _logconf: LogConfig) -> None:
        ts = timestamp / 1000.0
        ax = float(data["acc.x"])
        ay = float(data["acc.y"])
        az = float(data["acc.z"])

        self.plotter.accelerometer_point(ts, ax, ay, az)
        self.logs.accelerometer.writerow([ts, ax, ay, az])

    def _set_bl_motor_throttle(self, throttle: float) -> None:
        throttle = clamp(throttle, 0.0, 1.0)
        target = int(throttle * MAX_MOTOR_CMD)
        step = 1000

        if target == self.state.last_servo_value:
            return

        delta = target - self.state.last_servo_value
        direction = 1 if delta > 0 else -1
        for value in range(self.state.last_servo_value, target, step * direction):
            self.cf.param.set_value("servo.servoAngle", value)
            time.sleep(0.005)

        self.cf.param.set_value("servo.servoAngle", target)
        self.state.last_servo_value = target

        if self.args.debug:
            print(f"BL motor command: {target}")

    def _edge_pressed(self, button_idx: int) -> bool:
        current = int(self.joystick.get_button(button_idx))
        previous = self.state.last_button_state.get(button_idx, 0)
        self.state.last_button_state[button_idx] = current
        return current == 1 and previous == 0

    def _prompt_autonomous_setpoints(self) -> None:
        self.state.setpoint_pitch = clamp(
            prompt_float("Autonomous pitch setpoint (deg/s)", self.state.setpoint_pitch),
            -self.state.pitch_rate_limit,
            self.state.pitch_rate_limit,
        )
        self.state.setpoint_roll = clamp(
            prompt_float("Autonomous roll setpoint (deg/s)", self.state.setpoint_roll),
            -self.state.roll_rate_limit,
            self.state.roll_rate_limit,
        )
        self.state.setpoint_yaw = clamp(
            prompt_float("Autonomous yaw setpoint (deg/s)", self.state.setpoint_yaw),
            -self.state.yaw_rate_limit,
            self.state.yaw_rate_limit,
        )
        self.logs.write_event(
            "AUTONOMOUS_SETPOINTS",
            self.state.setpoint_roll,
            self.state.setpoint_pitch,
            self.state.setpoint_yaw,
        )

    def _prompt_pid_tuning(self) -> None:
        print("Manual PID tuning: leave empty to keep current value.")
        self.gains.pitch.kp = prompt_float("Pitch KP", self.gains.pitch.kp)
        self.gains.pitch.ki = prompt_float("Pitch KI", self.gains.pitch.ki)
        self.gains.pitch.kd = prompt_float("Pitch KD", self.gains.pitch.kd)

        self.gains.yaw.kp = prompt_float("Yaw KP", self.gains.yaw.kp)
        self.gains.yaw.ki = prompt_float("Yaw KI", self.gains.yaw.ki)
        self.gains.yaw.kd = prompt_float("Yaw KD", self.gains.yaw.kd)

        self.gains.roll.kp = prompt_float("Roll KP", self.gains.roll.kp)
        self.gains.roll.ki = prompt_float("Roll KI", self.gains.roll.ki)
        self.gains.roll.kd = prompt_float("Roll KD", self.gains.roll.kd)

        self._apply_pid_gains(self.gains)
        self.logs.write_breakpoint("PID_UPDATED")

    def _handle_controller_inputs(self) -> None:
        pygame.event.pump()

        roll_axis = round(self.joystick.get_axis(3), 3)
        pitch_axis = round(self.joystick.get_axis(4), 3)
        yaw_axis = round(self.joystick.get_axis(0), 3)
        hat = self.joystick.get_hat(0)

        if self._edge_pressed(0):
            self.state.trimmed = True
            self.logs.write_event("TRIM_ON")
        if self._edge_pressed(3):
            self.state.trimmed = False
            self.logs.write_event("TRIM_OFF")

        if self._edge_pressed(10):
            self.logs.write_breakpoint("MANUAL_BREAKPOINT")

        if self._edge_pressed(4):
            self.state.motor_armed = True
            self.state.throttle = max(self.state.throttle, 0.7)
            self._set_bl_motor_throttle(self.state.throttle)
            self.logs.write_breakpoint("FLIGHT_START_MOTOR_ARM")
            self.logs.write_event("MOTOR_ARM", self.state.throttle)

        if self._edge_pressed(5):
            self.state.motor_armed = False
            self.state.throttle = 0.0
            self._set_bl_motor_throttle(0.0)
            self.logs.write_breakpoint("FLIGHT_END_MOTOR_DISARM")
            self.logs.write_event("MOTOR_DISARM")

        if self._edge_pressed(2):
            self.state.autonomous = True
            self._prompt_autonomous_setpoints()
            self.logs.write_breakpoint("AUTONOMOUS_ENABLED")

        if self._edge_pressed(1):
            self.state.autonomous = False
            self.logs.write_breakpoint("AUTONOMOUS_DISABLED")

        if self._edge_pressed(9):
            self._prompt_pid_tuning()

        if hat != self.state.last_hat_state:
            if hat == (0, 1):
                self.state.throttle = clamp(self.state.throttle + THROTTLE_STEP, 0.0, 1.0)
                self.logs.write_event("THROTTLE_UP", round(self.state.throttle, 3))
            elif hat == (0, -1):
                self.state.throttle = clamp(self.state.throttle - THROTTLE_STEP, 0.0, 1.0)
                self.logs.write_event("THROTTLE_DOWN", round(self.state.throttle, 3))
            self.state.last_hat_state = hat

        if self.state.autonomous:
            self.commander.send_setpoint(
                clamp(self.state.setpoint_roll, -self.state.roll_rate_limit, self.state.roll_rate_limit),
                -clamp(self.state.setpoint_pitch, -self.state.pitch_rate_limit, self.state.pitch_rate_limit),
                -clamp(self.state.setpoint_yaw, -self.state.yaw_rate_limit, self.state.yaw_rate_limit),
                10001,
            )
        elif not self.state.trimmed:
            roll_cmd = roll_axis if abs(roll_axis) > JOYSTICK_DEADBAND else 0.0
            pitch_cmd = pitch_axis if abs(pitch_axis) > JOYSTICK_DEADBAND else 0.0
            yaw_cmd = yaw_axis if abs(yaw_axis) > JOYSTICK_DEADBAND else 0.0
            self.commander.send_setpoint(
                roll_cmd * self.state.roll_rate_limit,
                -pitch_cmd * self.state.pitch_rate_limit,
                -yaw_cmd * self.state.yaw_rate_limit,
                10001,
            )

        if self.state.motor_armed:
            self._set_bl_motor_throttle(self.state.throttle)

    def _main_control_loop(self) -> None:
        while True:
            self.plotter.tick()

            if (
                self.log_enabled.get("connection", False)
                and self.state.last_connection_seen_at > 0.0
                and (time.monotonic() - self.state.last_connection_seen_at) > CONNECTION_WATCHDOG_TIMEOUT_S
            ):
                self._failsafe_disarm("CONNECTION_TELEMETRY_TIMEOUT")

            if self.args.controller:
                self._handle_controller_inputs()
            else:
                self.commander.send_setpoint(0.0, 0.0, 0.0, 10001)

            time.sleep(0.01)

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
            self.logs.write_breakpoint("SESSION_END")
            self.logs.write_event("SESSION_END")
            self.logs.close()

        self.plotter.close()

        if self.args.controller:
            try:
                pygame.quit()
            except Exception:
                pass

    @staticmethod
    def _console_callback(text: str) -> None:
        print(text, end="")


def parse_args() -> argparse.Namespace:
    def parse_optional_bool(raw: str) -> bool:
        value = raw.strip().lower()
        if value in ("1", "true", "t", "yes", "y", "on"):
            return True
        if value in ("0", "false", "f", "no", "n", "off"):
            return False
        raise argparse.ArgumentTypeError("Expected one of: on/off, true/false, 1/0")

    parser = argparse.ArgumentParser(description="Crazyflie Bolt glider control and telemetry logger")
    parser.add_argument("--debug", action="store_true", help="Enable debug printouts")
    parser.add_argument("--controller", action="store_true", help="Enable Xbox controller input")
    parser.add_argument("--no-plot", action="store_true", help="Disable live plotting")
    parser.add_argument(
        "--fwactlpf-enable",
        type=parse_optional_bool,
        default=None,
        help="Default fwActLpf.enable value before startup prompt (on/off)",
    )
    parser.add_argument(
        "--fwactlpf-cutoff-hz",
        type=float,
        default=None,
        help="Default fwActLpf.cutoffHz value before startup prompt",
    )
    return parser.parse_args()


def main() -> None:
    app = GliderControlApp(parse_args())
    app.run()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
Crazyflie logger: Logs stateEstimate.y, stateEstimate.x, stateEstimate.z to a CSV file + live plotting.

Usage examples:
    python cf_log_to_csv_and_plot_stateestimate.py --csv out.csv
    python cf_log_to_csv_and_plot_stateestimate.py --csv out.csv --uri radio://0/80/2M --rate 50 --duration 60
    python cf_log_to_csv_and_plot_stateestimate.py --csv out.csv --plot-window 20
    python cf_log_to_csv_and_plot_stateestimate.py --csv out.csv --no-plot

Optional y-limits:
    python cf_log_to_csv_and_plot_stateestimate.py --csv out.csv --y-ylim -2 2 --x-ylim -2 2 --z-ylim 0 3
"""

import argparse
import csv
import os
import signal
import time
from threading import Event, Lock
from collections import deque

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper

# Matplotlib for live plotting
import matplotlib.pyplot as plt

STOP_EVENT = Event()


def parse_args():
    parser = argparse.ArgumentParser(description="Crazyflie logger to CSV with live plotting (stateEstimate.x/y/z)")
    parser.add_argument("--csv", required=True, help="Output CSV file path")
    parser.add_argument(
        "--uri",
        default=uri_helper.uri_from_env(default="radio://0/80/2M"),
        help="Crazyflie URI (default: env CF_URI or radio://0/80/2M)",
    )
    parser.add_argument(
        "--rate",
        type=int,
        default=10,
        help="Logging rate in Hz (default: 10)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="Optional duration in seconds to log (default: run until Ctrl+C)",
    )
    parser.add_argument(
        "--plot-window",
        type=float,
        default=10.0,
        help="Seconds of data to show in the live plot window (default: 10)",
    )
    parser.add_argument(
        "--no-plot",
        action="store_true",
        help="Disable live plotting (CSV logging only)",
    )
    # Optional fixed y-limits for each subplot
    parser.add_argument(
        "--y-ylim",
        nargs=2,
        type=float,
        default=None,
        metavar=("YMIN", "YMAX"),
        help="Fixed y-limits for stateEstimate.y plot, e.g., --y-ylim -2 2",
    )
    parser.add_argument(
        "--x-ylim",
        nargs=2,
        type=float,
        default=None,
        metavar=("YMIN", "YMAX"),
        help="Fixed y-limits for stateEstimate.x plot, e.g., --x-ylim -2 2",
    )
    parser.add_argument(
        "--z-ylim",
        nargs=2,
        type=float,
        default=None,
        metavar=("YMIN", "YMAX"),
        help="Fixed y-limits for stateEstimate.z plot, e.g., --z-ylim 0 3",
    )
    return parser.parse_args()


class CFCSVLogger:
    """
    Handles linking to the Crazyflie, creating a log block for stateEstimate.{y,x,z},
    streaming data to CSV, and storing samples in a thread-safe buffer for plotting.
    """

    def __init__(self, uri: str, csv_path: str, rate_hz: int):
        self.uri = uri
        self.csv_path = csv_path
        self.period_ms = max(10, int(1000 / max(1, rate_hz)))  # clamp to >=10 ms
        cache_dir = os.path.join(os.path.expanduser("~"), ".cache", "cflib")
        os.makedirs(cache_dir, exist_ok=True)
        self.cf = Crazyflie(rw_cache=cache_dir)

        self.logconf = None
        self._connected_evt = Event()
        self._disconnected_evt = Event()

        self._csv_file = None
        self._csv_writer = None
        self._last_flush_t = 0.0

        # Host time for relative plotting
        self._host_t0 = None

        # Thread-safe buffer for live plotting: (t_rel_s, y, x, z)
        self._buf_lock = Lock()
        self._buf = deque(maxlen=20000)  # ~20k samples

        # Register connection lifecycle callbacks
        self.cf.connected.add_callback(self._on_connected)
        self.cf.disconnected.add_callback(self._on_disconnected)
        self.cf.connection_failed.add_callback(self._on_connection_failed)
        self.cf.connection_lost.add_callback(self._on_connection_lost)

    # -------------------- Public API --------------------

    def open(self):
        print(f"[INFO] Connecting to Crazyflie @ {self.uri} ...")
        self.cf.open_link(self.uri)
        # Wait for connection (or failure)
        self._connected_evt.wait(timeout=10.0)
        if not self._connected_evt.is_set():
            raise RuntimeError("Failed to connect within timeout.")

    def close(self):
        try:
            if self.logconf is not None:
                try:
                    self.logconf.stop()
                except Exception:
                    pass
            self.cf.close_link()
        finally:
            if self._csv_file:
                try:
                    self._csv_file.flush()
                except Exception:
                    pass
                self._csv_file.close()
                self._csv_file = None
            print("\n[INFO] Closed link and file.")

    # -------------------- Callbacks --------------------

    def _on_connected(self, link_uri):
        print(f"[OK] Connected: {link_uri}")
        try:
            self._host_t0 = time.time()
            self._start_csv()
            self._start_logging()
            self._connected_evt.set()
        except Exception as e:
            print(f"[ERR] Setup failed: {e}")
            STOP_EVENT.set()

    def _on_disconnected(self, link_uri):
        print(f"\n[INFO] Disconnected: {link_uri}")
        self._disconnected_evt.set()
        STOP_EVENT.set()

    def _on_connection_failed(self, link_uri, msg):
        print(f"[ERR] Connection failed: {msg}")
        STOP_EVENT.set()

    def _on_connection_lost(self, link_uri, msg):
        print(f"\n[ERR] Connection lost: {msg}")
        STOP_EVENT.set()

    # -------------------- CSV --------------------

    def _start_csv(self):
        # Ensure parent dir exists
        parent = os.path.dirname(os.path.abspath(self.csv_path))
        if parent and not os.path.exists(parent):
            os.makedirs(parent, exist_ok=True)

        self._csv_file = open(self.csv_path, mode="w", newline="")
        self._csv_writer = csv.writer(self._csv_file)
        # Header
        self._csv_writer.writerow(
            ["host_time_s", "cf_timestamp_ms", "stateEstimate.y", "stateEstimate.x", "stateEstimate.z"]
        )
        self._csv_file.flush()
        print(f"[OK] Writing CSV to: {self.csv_path}")

    # -------------------- Logging --------------------

    def _start_logging(self):
        self.logconf = LogConfig(name="pos_xyz_log", period_in_ms=self.period_ms)
        # Replace acc.z -> stateEstimate.y, keep x, and add z
        self.logconf.add_variable("stateEstimate.y", "float")
        self.logconf.add_variable("stateEstimate.x", "float")
        self.logconf.add_variable("stateEstimate.z", "float")

        self.logconf.data_received_cb.add_callback(self._on_log_data)
        self.logconf.error_cb.add_callback(self._on_log_error)

        self.cf.log.add_config(self.logconf)
        self.logconf.start()

        print(f"[OK] Logging started @ ~{int(1000/self.period_ms)} Hz (period={self.period_ms} ms).")

    def _on_log_data(self, timestamp, data, logconf):
        """
        timestamp: CF MCU time in ms
        data: {'stateEstimate.y': float, 'stateEstimate.x': float, 'stateEstimate.z': float}
        """
        try:
            host_t = time.time()
            t_rel = host_t - (self._host_t0 or host_t)

            y_est = float(data.get("stateEstimate.y", float("nan")))
            x_est = float(data.get("stateEstimate.x", float("nan")))
            z_est = float(data.get("stateEstimate.z", float("nan")))

            # Write CSV
            self._csv_writer.writerow([f"{host_t:.6f}", int(timestamp),
                                       f"{y_est:.6f}", f"{x_est:.6f}", f"{z_est:.6f}"])

            # Flush every ~0.5 s
            if host_t - self._last_flush_t >= 0.5:
                self._csv_file.flush()
                self._last_flush_t = host_t

            # Add to plot buffer
            with self._buf_lock:
                self._buf.append((t_rel, y_est, x_est, z_est))

            # Optional compact console status (overwrites line)
            print(f"cf_t={timestamp:>8d} ms  y={y_est:>8.4f}  x={x_est:>8.4f}  z={z_est:>8.4f}", end="\r")

        except Exception as e:
            print(f"\n[ERR] CSV write or buffer update failed: {e}")
            STOP_EVENT.set()

    def _on_log_error(self, logconf, msg):
        print(f"\n[ERR] Log error: {msg}")
        STOP_EVENT.set()

    # -------------------- Plot buffer access --------------------

    def snapshot_buffer(self):
        """Safely return a copy of buffered samples as lists: t, y, x, z."""
        with self._buf_lock:
            if not self._buf:
                return [], [], [], []
            t, y, x, z = zip(*self._buf)
        return list(t), list(y), list(x), list(z)


class LivePlotter:
    """Simple rolling-window live plot for stateEstimate.y, stateEstimate.x, stateEstimate.z."""

    def __init__(self, logger: CFCSVLogger, window_s: float, y_ylim=None, x_ylim=None, z_ylim=None):
        self.logger = logger
        self.window_s = window_s
        self.y_ylim = y_ylim
        self.x_ylim = x_ylim
        self.z_ylim = z_ylim

        self.fig, axes = plt.subplots(3, 1, sharex=True, figsize=(9, 7.5))
        self.ax_y, self.ax_x, self.ax_z = axes
        self.fig.canvas.manager.set_window_title("Crazyflie Live Plot: stateEstimate (y, x, z)")

        # Lines
        (self.line_y,) = self.ax_y.plot([], [], color="tab:blue", lw=1.5, label="stateEstimate.y (m)")
        (self.line_x,) = self.ax_x.plot([], [], color="tab:orange", lw=1.5, label="stateEstimate.x (m)")
        (self.line_z,) = self.ax_z.plot([], [], color="tab:green", lw=1.5, label="stateEstimate.z (m)")

        # Labels & grid
        self.ax_y.set_ylabel("y (m)")
        self.ax_x.set_ylabel("x (m)")
        self.ax_z.set_ylabel("z (m)")
        self.ax_z.set_xlabel("time (s)")
        for ax in axes:
            ax.grid(True, alpha=0.3)
            ax.legend(loc="upper right")

        # Optional fixed y-limits
        if self.y_ylim is not None:
            self.ax_y.set_ylim(self.y_ylim)
        if self.x_ylim is not None:
            self.ax_x.set_ylim(self.x_ylim)
        if self.z_ylim is not None:
            self.ax_z.set_ylim(self.z_ylim)

        plt.tight_layout()
        plt.show(block=False)

    def update(self):
        t, y, x, z = self.logger.snapshot_buffer()
        if not t:
            plt.pause(0.01)
            return

        tmax = t[-1]
        tmin = max(0.0, tmax - self.window_s)

        # Find first index >= tmin
        start_idx = 0
        for i in range(len(t)):
            if t[i] >= tmin:
                start_idx = i
                break

        t_win = t[start_idx:]
        y_win = y[start_idx:]
        x_win = x[start_idx:]
        z_win = z[start_idx:]

        # Update line data
        self.line_y.set_data(t_win, y_win)
        self.line_x.set_data(t_win, x_win)
        self.line_z.set_data(t_win, z_win)

        # X-limits to rolling window
        self.ax_y.set_xlim(max(0.0, tmax - self.window_s), max(self.window_s, tmax))

        # Autoscale Y if not fixed
        def autoscale(ax, data, fixed):
            if fixed is not None or not data:
                return
            y_min, y_max = min(data), max(data)
            if y_min == y_max:
                y_min -= 0.1
                y_max += 0.1
            pad = 0.1 * (y_max - y_min)
            ax.set_ylim(y_min - pad, y_max + pad)

        autoscale(self.ax_y, y_win, self.y_ylim)
        autoscale(self.ax_x, x_win, self.x_ylim)
        autoscale(self.ax_z, z_win, self.z_ylim)

        # Redraw
        self.fig.canvas.draw_idle()
        plt.pause(0.01)


def main():
    args = parse_args()

    # Graceful Ctrl+C
    def handle_sigint(sig, frame):
        print("\n[INFO] Ctrl+C received, stopping ...")
        STOP_EVENT.set()

    signal.signal(signal.SIGINT, handle_sigint)

    # Init Crazyflie drivers
    cflib.crtp.init_drivers(enable_debug_driver=False)

    logger = CFCSVLogger(uri=args.uri, csv_path=args.csv, rate_hz=args.rate)

    plotter = None
    plotting = not args.no_plot

    try:
        logger.open()

        if plotting:
            plotter = LivePlotter(
                logger=logger,
                window_s=args.plot_window,
                y_ylim=tuple(args.y_ylim) if args.y_ylim else None,
                x_ylim=tuple(args.x_ylim) if args.x_ylim else None,
                z_ylim=tuple(args.z_ylim) if args.z_ylim else None,
            )

        # Run until duration reached or Ctrl+C / link loss
        if args.duration and args.duration > 0:
            deadline = time.time() + args.duration
            while not STOP_EVENT.is_set() and time.time() < deadline:
                if plotting:
                    plotter.update()
                else:
                    time.sleep(0.05)
            print("\n[INFO] Duration reached, stopping ...")
        else:
            while not STOP_EVENT.is_set():
                if plotting:
                    plotter.update()
                else:
                    time.sleep(0.05)

    except Exception as e:
        print(f"\n[ERR] {e}")
    finally:
        logger.close()
        if plotting:
            try:
                plt.close('all')
            except Exception:
                pass


if __name__ == "__main__":
    main()


#!/usr/bin/env python3
"""Lidar Backpack Scanner v1.0 — Flask web app for ROS 2 scan control."""

import atexit
import glob as globmod
import json
import os
import shutil
import signal
import socket
import subprocess
import sys
import time
import threading
from datetime import datetime
from pathlib import Path

from flask import Flask, render_template, jsonify, request

# ---------------------------------------------------------------------------
# Motor Controller (reused from v0.2 — Miranda I2C control)
# ---------------------------------------------------------------------------

class MotorController:
    def __init__(self, i2c_bus=None):
        self.dev_addr = 0x28
        self.bus = None
        if i2c_bus is None:
            i2c_bus = self._find_ch341_bus()
        self.i2c_bus = i2c_bus
        if i2c_bus is not None:
            try:
                from smbus2 import SMBus
                self.bus = SMBus(i2c_bus)
                print(f"[Motor] Opened I2C bus {i2c_bus}")
            except Exception as e:
                print(f"[Motor] Could not open I2C bus {i2c_bus}: {e}")
                print("[Motor] Motor control will be unavailable.")
        else:
            print("[Motor] No CH341 I2C adapter found. Motor control unavailable.")

    @staticmethod
    def _find_ch341_bus():
        """Auto-detect the CH341 USB-to-I2C adapter bus number."""
        try:
            result = subprocess.run(
                ["i2cdetect", "-l"], capture_output=True, text=True, timeout=5,
            )
            for line in result.stdout.splitlines():
                if "CH341" in line:
                    bus_num = int(line.split()[0].split("-")[1])
                    print(f"[Motor] Auto-detected CH341 on bus {bus_num}")
                    return bus_num
        except Exception as e:
            print(f"[Motor] Auto-detect failed: {e}")
        return None

    def miranda_write(self, address, command, tx_data):
        if self.bus is None:
            print("[Motor] I2C bus not available, skipping write.")
            return
        try:
            self.bus.write_i2c_block_data(address, command, tx_data)
        except Exception as e:
            print(f"[Motor] Failed to write to device: {e}")

    def start_motor(self, rpm=20):
        msb, lsb = self.rpm_to_counts(rpm)
        self.miranda_write(self.dev_addr, 0x07, [msb, lsb])
        print(f"[Motor] Started at {rpm} RPM")

    def stop_motor(self):
        self.miranda_write(self.dev_addr, 0x07, [0x00, 0x00])
        print("[Motor] Stopped")

    def rpm_to_counts(self, rpm):
        deg_per_sec = rpm * 6
        motor_counts = int(deg_per_sec * 91.01944444444445)
        msb = (motor_counts >> 8) & 0xFF
        lsb = motor_counts & 0xFF
        return msb, lsb


# ---------------------------------------------------------------------------
# Lidar Manager — manages ROS 2 subprocess lifecycle
# ---------------------------------------------------------------------------

class LidarManager:
    # States
    IDLE = "idle"
    STARTING = "starting"
    SCANNING = "scanning"
    STOPPING = "stopping"

    def __init__(self):
        self.state = self.IDLE
        self.lidar_type = "ouster"  # "ouster" or "livox"
        self.message = ""  # Step-by-step feedback for the UI
        self.processes = []
        self.output_dir = None
        self.scan_start_time = None
        self.last_elapsed = None  # Preserved after stop for reference
        self.motor = MotorController()
        self.lock = threading.Lock()

        # Paths
        self.project_dir = Path(__file__).parent.resolve()
        self.pointclouds_dir = Path.home() / "pointclouds"
        self.ros2_ws = Path.home() / "ros2_ws"

        # Ouster sensor hostname (matches driver_params.yaml)
        self.ouster_hostname = "os-122134000147.local"

    def _set_message(self, msg):
        with self.lock:
            self.message = msg
        print(f"[Status] {msg}")

    def _is_active(self):
        """Check if we're in a state where the scan sequence should continue."""
        return self.state in (self.STARTING, self.SCANNING)

    def _wait_countdown(self, seconds, msg_template):
        """Wait with a countdown, checking for abort each second.
        msg_template should contain {remaining} placeholder.
        Returns True if completed, False if aborted."""
        for remaining in range(seconds, 0, -1):
            if not self._is_active():
                return False
            self._set_message(msg_template.format(remaining=remaining))
            time.sleep(1)
        return self._is_active()

    # Health monitoring
    HEALTH_FILE = "/tmp/scan_health.json"

    def _read_health(self):
        """Read health status from the monitor's JSON file."""
        try:
            stat = os.stat(self.HEALTH_FILE)
            if time.time() - stat.st_mtime > 5:
                return {"status": "unknown", "message": "Health data stale"}
            with open(self.HEALTH_FILE, "r") as f:
                return json.load(f)
        except (FileNotFoundError, json.JSONDecodeError, OSError):
            return {"status": "unknown", "message": "No health data"}

    def get_status(self):
        with self.lock:
            elapsed = self.last_elapsed
            if self.scan_start_time and self.state in (self.SCANNING, self.STOPPING):
                elapsed = int(time.time() - self.scan_start_time)
            status = {
                "state": self.state,
                "lidar": self.lidar_type,
                "elapsed": elapsed,
                "output_dir": str(self.output_dir) if self.output_dir else None,
                "message": self.message,
            }
            if self.state in (self.SCANNING, self.STARTING):
                status["health"] = self._read_health()
            return status

    def start_scan(self, lidar_type):
        with self.lock:
            if self.state != self.IDLE:
                return False, f"Cannot start: currently {self.state}"
            self.state = self.STARTING
            self.lidar_type = lidar_type
            self.last_elapsed = None

        thread = threading.Thread(target=self._start_scan_sequence, daemon=True)
        thread.start()
        return True, "Scan starting..."

    def _start_scan_sequence(self):
        try:
            # Create timestamped output directory
            timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            self.output_dir = self.pointclouds_dir / f"{timestamp}_{self.lidar_type}"
            self.output_dir.mkdir(parents=True, exist_ok=True)

            source_cmd = (
                "source /opt/ros/humble/setup.bash && "
                f"source {self.ros2_ws}/install/setup.bash"
            )

            # 1. Launch lidar driver
            if self.lidar_type == "ouster":
                self._set_message("Starting Ouster driver...")
                driver_cmd = (
                    f"{source_cmd} && "
                    "ros2 launch ouster_ros driver.launch.py viz:=False"
                )
            else:
                self._set_message("Starting Livox driver...")
                driver_cmd = (
                    f"{source_cmd} && "
                    "ros2 launch livox_ros_driver2 msg_MID360_launch.py"
                )

            driver_proc = subprocess.Popen(
                driver_cmd, shell=True, executable="/bin/bash",
                preexec_fn=os.setsid,
            )
            self.processes.append(driver_proc)

            # Wait for driver to initialize
            if self.lidar_type == "ouster":
                if not self._wait_countdown(40, "Ouster driver initializing ({remaining}s)..."):
                    return
            else:
                if not self._wait_countdown(5, "Livox driver initializing ({remaining}s)..."):
                    return

            # 2. Launch FAST-LIO
            if self.lidar_type == "ouster":
                config_file = "ouster32_backpack.yaml"
            else:
                config_file = "mid360_backpack.yaml"

            self._set_message("Starting FAST-LIO SLAM...")
            fastlio_cmd = (
                f"{source_cmd} && "
                f"cd {self.output_dir} && "
                f"ros2 launch fast_lio mapping.launch.py "
                f"config_file:={config_file} rviz:=true"
            )

            fastlio_proc = subprocess.Popen(
                fastlio_cmd, shell=True, executable="/bin/bash",
                preexec_fn=os.setsid,
            )
            self.processes.append(fastlio_proc)

            if not self._wait_countdown(5, "FAST-LIO initializing ({remaining}s)..."):
                return

            # 3. Start bag recording
            bag_output = self.output_dir / "bag"
            if self.lidar_type == "ouster":
                topics = "/ouster/imu_packets /ouster/lidar_packets"
            else:
                topics = "/livox/lidar /livox/imu"

            self._set_message("Starting bag recording...")
            bag_cmd = (
                f"{source_cmd} && "
                f"ros2 bag record -o {bag_output} {topics}"
            )

            bag_proc = subprocess.Popen(
                bag_cmd, shell=True, executable="/bin/bash",
                preexec_fn=os.setsid,
            )
            self.processes.append(bag_proc)

            # 3b. Launch scan health monitor
            self._set_message("Starting health monitor...")
            monitor_cmd = (
                f"{source_cmd} && "
                f"/usr/bin/python3 {self.project_dir}/scan_monitor.py"
            )
            monitor_proc = subprocess.Popen(
                monitor_cmd, shell=True, executable="/bin/bash",
                preexec_fn=os.setsid,
            )
            self.processes.append(monitor_proc)

            # 4. Delay motor start so FAST-LIO builds initial map first
            if self.lidar_type == "ouster":
                if not self._wait_countdown(10, "FAST-LIO stabilizing, motor in {remaining}s..."):
                    return
                self._set_message("Starting Miranda motor...")
                self.motor.start_motor(20)

            # 5. Copy Ouster metadata JSON to output dir
            if self.lidar_type == "ouster":
                self._copy_ouster_metadata()

            with self.lock:
                self.state = self.SCANNING
            self.scan_start_time = time.time()
            self._set_message("Scanning — check rviz for live point cloud")

        except Exception as e:
            print(f"[Scan] Error during startup: {e}")
            self._set_message(f"Error: {e}")
            with self.lock:
                self.state = self.IDLE

    def _copy_ouster_metadata(self):
        """Copy the Ouster sensor metadata JSON to the output directory."""
        try:
            candidates = globmod.glob(os.path.expanduser("~/ros2_ws/os-*-metadata.json"))
            candidates += globmod.glob("os-*-metadata.json")
            candidates += globmod.glob(os.path.expanduser("~/.ros/os-*-metadata.json"))
            for src in candidates:
                dst = self.output_dir / Path(src).name
                shutil.copy2(src, dst)
                print(f"[Scan] Copied metadata: {src} -> {dst}")
                return
            print("[Scan] No Ouster metadata JSON found to copy.")
        except Exception as e:
            print(f"[Scan] Failed to copy metadata: {e}")

    def stop_scan(self):
        with self.lock:
            if self.state not in (self.STARTING, self.SCANNING):
                return False, f"Cannot stop: currently {self.state}"
            self.state = self.STOPPING

        thread = threading.Thread(target=self._stop_scan_sequence, daemon=True)
        thread.start()
        return True, "Stopping scan..."

    def _stop_scan_sequence(self):
        try:
            # 1. Set Ouster to standby
            if self.lidar_type == "ouster":
                self._set_message("Setting Ouster to STANDBY...")
                self._set_ouster_standby()

            # 2. Stop motor
            if self.lidar_type == "ouster":
                self._set_message("Stopping motor...")
                self.motor.stop_motor()

            # 3. SIGINT all ROS processes (lets FAST-LIO save PCD)
            self._set_message("Stopping ROS processes (saving PCD)...")
            for proc in self.processes:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                except (ProcessLookupError, OSError):
                    pass

            # 4. Wait for exit
            self._set_message("Waiting for processes to exit...")
            deadline = time.time() + 30
            for proc in self.processes:
                remaining = max(0.1, deadline - time.time())
                try:
                    proc.wait(timeout=remaining)
                except subprocess.TimeoutExpired:
                    print(f"[Stop] Process {proc.pid} didn't exit, sending SIGKILL...")
                    try:
                        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                    except (ProcessLookupError, OSError):
                        pass

            self.processes.clear()
            if self.scan_start_time:
                self.last_elapsed = int(time.time() - self.scan_start_time)
            self.scan_start_time = None

            # Clean up health file
            try:
                os.remove(self.HEALTH_FILE)
            except OSError:
                pass

            self._set_message(f"Scan saved to {self.output_dir}")

        except Exception as e:
            print(f"[Stop] Error during shutdown: {e}")
            self._set_message(f"Stop error: {e}")
        finally:
            with self.lock:
                self.state = self.IDLE

    def force_stop(self):
        """Emergency kill — always works regardless of state."""
        self._set_message("Force stopping all processes...")
        self.motor.stop_motor()
        self._set_ouster_standby()
        for proc in self.processes:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except (ProcessLookupError, OSError):
                pass
        self.processes.clear()
        if self.scan_start_time:
            self.last_elapsed = int(time.time() - self.scan_start_time)
        self.scan_start_time = None
        with self.lock:
            self.state = self.IDLE
        self._set_message("Force stopped.")

    def _set_ouster_standby(self):
        """Best-effort attempt to put Ouster into STANDBY mode."""
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                sock.settimeout(3)
                sock.connect((self.ouster_hostname, 7501))
                sock.sendall(b"set_config_param operating_mode STANDBY\nreinitialize\n")
                time.sleep(0.2)
        except Exception:
            pass

    def emergency_cleanup(self):
        """Called on process exit (signal/atexit) — stop motor & lidar, kill children."""
        print("\n[Cleanup] Emergency cleanup triggered...")
        try:
            self.motor.stop_motor()
        except Exception:
            pass
        try:
            self._set_ouster_standby()
        except Exception:
            pass
        for proc in self.processes:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except (ProcessLookupError, OSError):
                pass


# ---------------------------------------------------------------------------
# Flask Application
# ---------------------------------------------------------------------------

app = Flask(__name__)
manager = LidarManager()


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/api/status")
def api_status():
    return jsonify(manager.get_status())


@app.route("/api/start", methods=["POST"])
def api_start():
    data = request.get_json(silent=True) or {}
    lidar_type = data.get("lidar", "ouster")
    if lidar_type not in ("ouster", "livox"):
        return jsonify({"ok": False, "msg": "Invalid lidar type"}), 400
    ok, msg = manager.start_scan(lidar_type)
    return jsonify({"ok": ok, "msg": msg})


@app.route("/api/stop", methods=["POST"])
def api_stop():
    ok, msg = manager.stop_scan()
    return jsonify({"ok": ok, "msg": msg})


@app.route("/api/force_stop", methods=["POST"])
def api_force_stop():
    """Emergency kill — always works."""
    manager.force_stop()
    return jsonify({"ok": True, "msg": "Force stopped."})


@app.route("/api/exit", methods=["POST"])
def api_exit():
    """Stop everything and shut down the server."""
    if manager.state != manager.IDLE:
        manager.force_stop()
    print("[App] Shutting down...")
    threading.Thread(target=lambda: (time.sleep(1), os._exit(0)), daemon=True).start()
    return jsonify({"ok": True, "msg": "Server shutting down..."})


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    # Register cleanup so motor/lidar stop on any exit (Ctrl-C, kill, crash)
    atexit.register(manager.emergency_cleanup)

    def _signal_handler(signum, frame):
        print(f"\n[Signal] Caught signal {signum}, cleaning up...")
        manager.emergency_cleanup()
        sys.exit(1)

    signal.signal(signal.SIGTERM, _signal_handler)
    signal.signal(signal.SIGHUP, _signal_handler)

    print("=" * 60)
    print("  Lidar Backpack Scanner v1.0")
    print("  Open http://<this-ip>:5000 in your browser")
    print("=" * 60)
    try:
        app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)
    except KeyboardInterrupt:
        print("\n[App] Ctrl-C received, cleaning up...")
        manager.emergency_cleanup()
        sys.exit(0)

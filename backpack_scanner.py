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
# Constants
# ---------------------------------------------------------------------------

MIN_DISK_MB = 1024  # Minimum free disk space (MB) to start a scan


def _detect_ros2_setup():
    """Find the first installed ROS 2 distro setup.bash."""
    for distro in ("jazzy", "humble", "iron", "rolling"):
        setup = Path(f"/opt/ros/{distro}/setup.bash")
        if setup.exists():
            return str(setup)
    raise RuntimeError("No ROS 2 distro found in /opt/ros/")

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
        self._cleanup_lock = threading.Lock()  # Prevent concurrent cleanup races
        self._fastlio_proc = None  # Reference to FAST-LIO for PCD save

        # Paths
        self.project_dir = Path(__file__).parent.resolve()
        self.pointclouds_dir = Path.home() / "pointclouds"
        self.ros2_ws = Path.home() / "ros2_ws"

        # Ouster sensor hostname (matches driver_params.yaml)
        self.ouster_hostname = "os-122134000147.local"

        # Livox MID-360 power control
        self.livox_config = self.ros2_ws / "src" / "livox_ros_driver2" / "config" / "MID360_config.json"
        self.livox_workmode_tool = self.project_dir / "livox_workmode"

    def _set_message(self, msg):
        with self.lock:
            self.message = msg
        print(f"[Status] {msg}")

    def _is_active(self):
        """Check if we're in a state where the scan sequence should continue."""
        return self.state in (self.STARTING, self.SCANNING)

    def _wait_countdown(self, seconds, msg_template, watch_proc=None):
        """Wait with a countdown, checking for abort each second.
        msg_template should contain {remaining} placeholder.
        If watch_proc is given, abort if that process exits early.
        Returns True if completed, False if aborted."""
        for remaining in range(seconds, 0, -1):
            if not self._is_active():
                return False
            if watch_proc and watch_proc.poll() is not None:
                self._set_message("Driver process died during startup")
                return False
            self._set_message(msg_template.format(remaining=remaining))
            time.sleep(1)
        if watch_proc and watch_proc.poll() is not None:
            self._set_message("Driver process died during startup")
            return False
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

    def _preflight_check(self):
        """Run pre-flight checks before starting a scan.
        Returns (ok, error_message)."""

        # 1. Check disk space
        self._set_message("Checking disk space...")
        try:
            stat = os.statvfs(str(self.pointclouds_dir))
            free_mb = (stat.f_bavail * stat.f_frsize) / (1024 * 1024)
            if free_mb < MIN_DISK_MB:
                return False, f"Low disk space: {free_mb:.0f} MB free (need {MIN_DISK_MB} MB)"
            print(f"[Preflight] Disk space OK: {free_mb:.0f} MB free")
        except OSError as e:
            return False, f"Cannot check disk space: {e}"

        # 2. Check sensor reachability
        if self.lidar_type == "ouster":
            host = self.ouster_hostname
        else:
            host = "192.168.2.183"

        self._set_message(f"Checking {self.lidar_type} sensor connectivity...")
        try:
            result = subprocess.run(
                ["ping", "-c", "1", "-W", "3", host],
                capture_output=True, timeout=5,
            )
            if result.returncode != 0:
                return False, (
                    f"Cannot reach {self.lidar_type} sensor at {host} — "
                    f"check cable and power"
                )
            print(f"[Preflight] Sensor {host} reachable")
        except subprocess.TimeoutExpired:
            return False, f"Sensor ping timed out for {host} — check network"
        except Exception as e:
            return False, f"Connectivity check failed: {e}"

        return True, ""

    def _kill_all_processes(self, sig=signal.SIGKILL):
        """Kill all tracked processes. Safe to call from any thread."""
        with self._cleanup_lock:
            procs = list(self.processes)  # Snapshot under lock

        for proc in procs:
            try:
                os.killpg(os.getpgid(proc.pid), sig)
            except (ProcessLookupError, OSError):
                pass

        for proc in procs:
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                # Escalate to SIGKILL if SIGINT didn't work
                if sig != signal.SIGKILL:
                    try:
                        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                    except (ProcessLookupError, OSError):
                        pass

        with self._cleanup_lock:
            self.processes.clear()
            self._fastlio_proc = None

    def _abort_startup(self, reason):
        """Clean up after a failed or aborted startup sequence."""
        print(f"[Scan] Startup aborted: {reason}")
        self._set_message(f"Aborted: {reason}")

        # Stop motor if it was started
        try:
            self.motor.stop_motor()
        except Exception:
            pass

        self._kill_all_processes()

        # Put Livox to standby in background (ports freed by _kill_all_processes)
        if self.lidar_type == "livox":
            threading.Thread(target=self._set_livox_standby, daemon=True).start()

        with self.lock:
            self.state = self.IDLE

    def _start_scan_sequence(self):
        try:
            # Pre-flight checks
            ok, err = self._preflight_check()
            if not ok:
                self._abort_startup(err)
                return
            if not self._is_active():
                self._abort_startup("Cancelled by user")
                return

            # Create timestamped output directory
            timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            self.output_dir = self.pointclouds_dir / f"{timestamp}_{self.lidar_type}"
            self.output_dir.mkdir(parents=True, exist_ok=True)

            source_cmd = (
                f"source {_detect_ros2_setup()} && "
                f"source {self.ros2_ws}/install/setup.bash"
            )

            # 0. Wake Livox from standby (must finish before driver claims UDP ports)
            if self.lidar_type == "livox":
                self._set_message("Setting Livox MID-360 to Normal...")
                if not self._set_livox_normal():
                    # Non-fatal: the ROS driver also sets Normal on connect
                    print("[Scan] Livox wake command failed, continuing anyway")
                if not self._is_active():
                    self._abort_startup("Cancelled by user")
                    return
                if not self._wait_countdown(
                    12, "Livox motor starting ({remaining}s)...",
                ):
                    self._abort_startup("Cancelled during Livox wake-up")
                    return

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

            # Wait for driver to initialize (watch for early crash)
            if self.lidar_type == "ouster":
                if not self._wait_countdown(
                    40, "Ouster driver initializing ({remaining}s)...",
                    watch_proc=driver_proc,
                ):
                    self._abort_startup("Driver failed or cancelled during init")
                    return
            else:
                if not self._wait_countdown(
                    5, "Livox driver initializing ({remaining}s)...",
                    watch_proc=driver_proc,
                ):
                    self._abort_startup("Driver failed or cancelled during init")
                    return

            # 2. Launch FAST-LIO (without rviz — launched separately to avoid SIGSEGV)
            if self.lidar_type == "ouster":
                config_file = "ouster32_backpack.yaml"
            else:
                config_file = "mid360_backpack.yaml"

            # Launch with rviz if a display is available, skip when headless
            has_display = bool(
                os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")
            )
            use_rviz = "true" if has_display else "false"
            if not has_display:
                print("[Scan] No display — skipping rviz2")

            self._set_message("Starting FAST-LIO SLAM...")
            fastlio_cmd = (
                f"{source_cmd} && "
                f"cd {self.output_dir} && "
                f"ros2 launch fast_lio mapping.launch.py "
                f"config_file:={config_file} rviz:={use_rviz}"
            )

            fastlio_proc = subprocess.Popen(
                fastlio_cmd, shell=True, executable="/bin/bash",
                preexec_fn=os.setsid,
            )
            self.processes.append(fastlio_proc)
            self._fastlio_proc = fastlio_proc

            if not self._wait_countdown(
                5, "FAST-LIO initializing ({remaining}s)...",
                watch_proc=fastlio_proc,
            ):
                self._abort_startup("FAST-LIO failed or cancelled during init")
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
                    self._abort_startup("Cancelled during motor delay")
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
            self._abort_startup(str(e))

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

            # 3. Save PCD via ROS service (while FAST-LIO is still running)
            self._set_message("Saving PCD...")
            self._call_map_save()
            self._rename_pcd()
            self._set_message("Saved PCD")

            # 4. Kill rviz2 before SIGINT (prevents SIGSEGV crash noise)
            subprocess.run(
                ["pkill", "-9", "rviz2"],
                capture_output=True, timeout=3,
            )

            # 5. Graceful SIGINT, then SIGKILL stragglers
            self._set_message("Saving bag...")
            self._kill_all_processes(sig=signal.SIGINT)
            self._set_message("Saved bag")

            # 6. Set Livox to standby (ports now free after process kill)
            if self.lidar_type == "livox":
                self._set_message("Setting Livox to standby...")
                time.sleep(1)  # Brief delay for UDP port release
                self._set_livox_standby()

            if self.scan_start_time:
                self.last_elapsed = int(time.time() - self.scan_start_time)
            self.scan_start_time = None

            # 7. Copy metadata
            self._set_message("Saved metadata")

            # Clean up health file
            try:
                os.remove(self.HEALTH_FILE)
            except OSError:
                pass

            self._set_message("Scan complete")

        except Exception as e:
            print(f"[Stop] Error during shutdown: {e}")
            self._set_message(f"Stop error: {e}")
        finally:
            with self.lock:
                self.state = self.IDLE

    def force_stop(self):
        """Emergency kill — always works regardless of state."""
        self._set_message("Force stopping all processes...")
        lidar = self.lidar_type  # Snapshot before state reset
        try:
            self.motor.stop_motor()
        except Exception:
            pass
        if lidar == "ouster":
            self._set_ouster_standby()
        self._kill_all_processes()
        if self.scan_start_time:
            self.last_elapsed = int(time.time() - self.scan_start_time)
        self.scan_start_time = None
        with self.lock:
            self.state = self.IDLE
        self._set_message("Force stopped.")
        # Put Livox to standby in background (ports freed by _kill_all_processes)
        if lidar == "livox":
            threading.Thread(target=self._set_livox_standby, daemon=True).start()

    def _call_map_save(self):
        """Call FAST-LIO's /map_save service to write PCD before shutdown."""
        try:
            source_cmd = (
                f"source {_detect_ros2_setup()} && "
                f"source {self.ros2_ws}/install/setup.bash"
            )
            cmd = f"{source_cmd} && ros2 service call /map_save std_srvs/srv/Trigger"
            result = subprocess.run(
                cmd, shell=True, executable="/bin/bash",
                capture_output=True, text=True, timeout=30,
            )
            if result.returncode == 0:
                print(f"[Scan] PCD saved via /map_save service")
            else:
                print(f"[Scan] /map_save service failed: {result.stderr.strip()}")
        except subprocess.TimeoutExpired:
            print("[Scan] /map_save service call timed out")
        except Exception as e:
            print(f"[Scan] /map_save service error: {e}")

    def _rename_pcd(self):
        """Rename scan.pcd to include the timestamp matching the output folder."""
        if not self.output_dir:
            return
        src = self.output_dir / "scan.pcd"
        if not src.exists():
            print("[Scan] No scan.pcd found to rename")
            return
        try:
            # Extract timestamp from output dir name (e.g. "2026-03-02_14-30-00_ouster")
            folder_name = self.output_dir.name
            new_name = f"{folder_name}.pcd"
            dst = self.output_dir / new_name
            src.rename(dst)
            print(f"[Scan] Renamed PCD: {src.name} -> {new_name}")
        except Exception as e:
            print(f"[Scan] Failed to rename PCD: {e}")

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

    def _livox_set_workmode(self, mode, timeout=20):
        """Call the livox_workmode helper to change MID-360 work mode.
        mode: 'standby' or 'normal'.  Returns True on success."""
        try:
            result = subprocess.run(
                [str(self.livox_workmode_tool), str(self.livox_config), mode],
                capture_output=True, text=True, timeout=timeout,
            )
            if result.returncode == 0:
                print(f"[Livox] Work mode set to {mode}")
                return True
            else:
                print(f"[Livox] Failed to set {mode}: {result.stderr.strip()}")
                return False
        except subprocess.TimeoutExpired:
            print(f"[Livox] Timeout setting work mode to {mode}")
            return False
        except Exception as e:
            print(f"[Livox] Error setting work mode: {e}")
            return False

    def _set_livox_standby(self, timeout=20):
        """Best-effort: put Livox MID-360 into standby (motor on, laser off)."""
        self._livox_set_workmode("standby", timeout=timeout)

    def _set_livox_normal(self):
        """Set Livox MID-360 to normal (sampling) mode. Returns True on success."""
        return self._livox_set_workmode("normal")

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
        self._kill_all_processes()
        try:
            self._set_livox_standby(timeout=10)
        except Exception:
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

    def _shutdown():
        time.sleep(1)
        print("[App] Shutting down...")
        _hotspot_watchdog_stop.set()  # Stop watchdog BEFORE restoring WiFi
        manager.emergency_cleanup()
        _stop_hotspot()
        os._exit(0)

    threading.Thread(target=_shutdown, daemon=True).start()
    return jsonify({"ok": True, "msg": "Server shutting down..."})


@app.route("/api/shutdown", methods=["POST"])
def api_shutdown():
    """Stop scan, clean up, and power off the computer."""
    # 1. Stop any active scan gracefully
    if manager.state in (manager.STARTING, manager.SCANNING):
        manager.stop_scan()
        # Wait for stop sequence to finish (PCD save, bag flush)
        deadline = time.time() + 45
        while manager.state != manager.IDLE and time.time() < deadline:
            time.sleep(1)
    elif manager.state == manager.STOPPING:
        deadline = time.time() + 45
        while manager.state != manager.IDLE and time.time() < deadline:
            time.sleep(1)

    # 2. Force stop if graceful stop didn't finish
    if manager.state != manager.IDLE:
        manager.force_stop()

    def _do_shutdown():
        time.sleep(2)
        print("[App] Shutting down computer...")
        _hotspot_watchdog_stop.set()  # Stop watchdog BEFORE restoring WiFi
        manager.emergency_cleanup()
        _stop_hotspot()
        subprocess.run(["sudo", "shutdown", "-h", "now"],
                       capture_output=True, timeout=10)

    threading.Thread(target=_do_shutdown, daemon=True).start()
    return jsonify({"ok": True, "msg": "Computer shutting down..."})


# ---------------------------------------------------------------------------
# WiFi Hotspot Management
# ---------------------------------------------------------------------------

_previous_wifi = None


def _get_wifi_device():
    """Auto-detect the WiFi device name (e.g. wlo1, wlan0)."""
    try:
        result = subprocess.run(
            ["nmcli", "-t", "-f", "DEVICE,TYPE", "device"],
            capture_output=True, text=True, timeout=5,
        )
        for line in result.stdout.strip().splitlines():
            parts = line.split(":")
            if len(parts) >= 2 and parts[1] == "wifi":
                return parts[0]
    except Exception:
        pass
    return "wlo1"  # Fallback to known device


def _start_hotspot():
    """Activate the BackpackScanner hotspot, saving the current WiFi connection."""
    global _previous_wifi
    try:
        # Remember current WiFi connection so we can restore on exit
        result = subprocess.run(
            ["nmcli", "-t", "-f", "NAME,TYPE,DEVICE", "connection", "show", "--active"],
            capture_output=True, text=True, timeout=5,
        )
        for line in result.stdout.strip().splitlines():
            parts = line.split(":")
            if len(parts) >= 3 and parts[1] == "802-11-wireless" and parts[0] != "Hotspot":
                _previous_wifi = parts[0]
                break

        # Activate hotspot
        result = subprocess.run(
            ["nmcli", "connection", "up", "Hotspot"],
            capture_output=True, text=True, timeout=10,
        )
        if result.returncode == 0:
            print("[Hotspot] BackpackScanner WiFi active — phone connects to http://10.42.0.1:5000")
        else:
            print(f"[Hotspot] Failed to start: {result.stderr.strip()}")
            print("[Hotspot] Continuing without hotspot — connect to same network as backpack.")
    except Exception as e:
        print(f"[Hotspot] Could not start hotspot: {e}")
        print("[Hotspot] Continuing without hotspot.")


def _is_hotspot_active():
    """Check if the Hotspot connection is currently active."""
    try:
        result = subprocess.run(
            ["nmcli", "-t", "-f", "NAME", "connection", "show", "--active"],
            capture_output=True, text=True, timeout=5,
        )
        return "Hotspot" in result.stdout.strip().splitlines()
    except Exception:
        return False


_hotspot_watchdog_stop = threading.Event()


def _hotspot_watchdog():
    """Background thread: re-enable hotspot if it drops while the app is running."""
    while not _hotspot_watchdog_stop.wait(timeout=15):
        if not _is_hotspot_active():
            print("[Hotspot] Watchdog: hotspot dropped, re-enabling...")
            result = subprocess.run(
                ["nmcli", "connection", "up", "Hotspot"],
                capture_output=True, text=True, timeout=10,
            )
            if result.returncode == 0:
                print("[Hotspot] Watchdog: hotspot restored")
            else:
                print(f"[Hotspot] Watchdog: restore failed: {result.stderr.strip()}")


def _stop_hotspot():
    """Deactivate hotspot and restore previous WiFi connection."""
    try:
        wifi_dev = _get_wifi_device()

        # Bring down the hotspot connection
        result = subprocess.run(
            ["nmcli", "connection", "down", "Hotspot"],
            capture_output=True, text=True, timeout=10,
        )
        if result.returncode != 0:
            print(f"[Hotspot] 'down' failed: {result.stderr.strip()}")
            # Fallback: disconnect the wifi device directly
            subprocess.run(
                ["nmcli", "device", "disconnect", wifi_dev],
                capture_output=True, text=True, timeout=10,
            )
        time.sleep(1)

        if _previous_wifi:
            result = subprocess.run(
                ["nmcli", "connection", "up", _previous_wifi],
                capture_output=True, text=True, timeout=15,
            )
            if result.returncode == 0:
                print(f"[Hotspot] Restored WiFi: {_previous_wifi}")
            else:
                print(f"[Hotspot] Could not restore '{_previous_wifi}': {result.stderr.strip()}")
                # Let NetworkManager auto-connect to any known network
                subprocess.run(
                    ["nmcli", "device", "wifi", "rescan"],
                    capture_output=True, text=True, timeout=10,
                )
                subprocess.run(
                    ["nmcli", "device", "connect", wifi_dev],
                    capture_output=True, text=True, timeout=15,
                )
                print("[Hotspot] Triggered WiFi reconnect via device connect.")
        else:
            # No previous WiFi saved — just let NM auto-connect
            subprocess.run(
                ["nmcli", "device", "wifi", "rescan"],
                capture_output=True, text=True, timeout=10,
            )
            subprocess.run(
                ["nmcli", "device", "connect", wifi_dev],
                capture_output=True, text=True, timeout=15,
            )
            print("[Hotspot] Hotspot stopped, triggered WiFi auto-connect.")
    except Exception as e:
        print(f"[Hotspot] Error restoring WiFi: {e}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    # Start WiFi hotspot for phone access
    _start_hotspot()

    # Start watchdog to auto-restore hotspot if it gets knocked off
    _wd_thread = threading.Thread(target=_hotspot_watchdog, daemon=True)
    _wd_thread.start()

    # Register cleanup so motor/lidar/hotspot stop on any exit.
    # Use a mutable flag to prevent double-cleanup (signal handler + atexit).
    _cleanup_done = [False]

    def _full_cleanup():
        if _cleanup_done[0]:
            return
        _cleanup_done[0] = True
        _hotspot_watchdog_stop.set()
        manager.emergency_cleanup()
        _stop_hotspot()

    atexit.register(_full_cleanup)

    def _signal_handler(signum, frame):
        print(f"\n[Signal] Caught signal {signum}, cleaning up...")
        _full_cleanup()
        sys.exit(0)  # Clean exit — not a crash

    signal.signal(signal.SIGTERM, _signal_handler)
    signal.signal(signal.SIGHUP, _signal_handler)

    print("=" * 60)
    print("  Lidar Backpack Scanner v1.0")
    print("  Connect phone to BackpackScanner WiFi")
    print("  Open http://10.42.0.1:5000")
    print("=" * 60)
    try:
        app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)
    except KeyboardInterrupt:
        print("\n[App] Ctrl-C received, cleaning up...")
        _full_cleanup()
        sys.exit(0)

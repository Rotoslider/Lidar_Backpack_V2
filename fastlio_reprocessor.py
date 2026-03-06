#!/usr/bin/python3
"""FAST-LIO Reprocessor GUI — replay bags with tunable parameters."""

import json
import os
import signal
import subprocess
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import yaml
from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QFont, QPalette, QColor
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QLabel, QPushButton, QComboBox, QDoubleSpinBox,
    QSpinBox, QCheckBox, QTabWidget, QGroupBox, QFileDialog,
    QLineEdit, QSplitter, QFrame, QInputDialog, QMessageBox,
    QSizePolicy, QDialog, QTextEdit, QDialogButtonBox,
)

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

POINTCLOUDS_DIR = Path.home() / "pointclouds"
REPROCESS_DIR = POINTCLOUDS_DIR / "reprocess"
PRESET_DIR = Path(__file__).parent / "reprocessor_presets"
ROS2_WS = Path.home() / "ros2_ws"
PROJECT_DIR = Path(__file__).parent.resolve()
FASTLIO_CONFIG_DIR = ROS2_WS / "src" / "FAST_LIO_ROS2" / "config"
FASTLIO_LOG_DIR = ROS2_WS / "src" / "FAST_LIO_ROS2" / "Log"
HEALTH_FILE = "/tmp/scan_health.json"

SOURCE_CMD = (
    "source /opt/ros/humble/setup.bash && "
    f"source {ROS2_WS}/install/setup.bash"
)

# ---------------------------------------------------------------------------
# Parameter Schema — single source of truth for UI, YAML I/O, and tooltips
# ---------------------------------------------------------------------------

# Each entry: (yaml_path, label, type, default, tooltip, extra)
#   yaml_path: dot-separated YAML path under ros__parameters
#   type: "float", "int", "bool", "choice", "vector3", "matrix3x3"
#   extra: dict with min/max/step/choices as appropriate

PARAM_SCHEMA = [
    # --- Resolution ---
    {
        "category": "Resolution",
        "path": "filter_size_surf",
        "label": "Surf Voxel Size",
        "type": "float",
        "default": 0.15,
        "tooltip": "ICP voxel grid size (m). Smaller = more detail, more CPU. 0.15 = high detail, 0.5 = fast/coarse.",
        "min": 0.01, "max": 5.0, "step": 0.05,
    },
    {
        "category": "Resolution",
        "path": "filter_size_map",
        "label": "Map Voxel Size",
        "type": "float",
        "default": 0.15,
        "tooltip": "Map voxel grid size (m). Should generally match surf size.",
        "min": 0.01, "max": 5.0, "step": 0.05,
    },
    {
        "category": "Resolution",
        "path": "point_filter_num",
        "label": "Point Filter N",
        "type": "int",
        "default": 1,
        "tooltip": "Keep every Nth point. 1 = all points (highest quality), 2 = half, 4 = quarter.",
        "min": 1, "max": 8, "step": 1,
    },
    {
        "category": "Resolution",
        "path": "max_iteration",
        "label": "Max IKF Iterations",
        "type": "int",
        "default": 4,
        "tooltip": "Iterated Kalman Filter iterations per frame. Higher = tighter pose convergence, more CPU.",
        "min": 1, "max": 20, "step": 1,
    },
    {
        "category": "Resolution",
        "path": "cube_side_length",
        "label": "Cube Side Length",
        "type": "float",
        "default": 1000.0,
        "tooltip": "Local map cube side (m). 1000 = large outdoor scans. Reduce for memory-constrained systems.",
        "min": 50.0, "max": 5000.0, "step": 100.0,
    },
    # --- Range ---
    {
        "category": "Range",
        "path": "preprocess.blind",
        "label": "Min Range (blind)",
        "type": "float",
        "default": 1.5,
        "tooltip": "Minimum range in meters. Filters backpack body / close objects. Ouster: 1.5, Livox: 0.5.",
        "min": 0.0, "max": 10.0, "step": 0.1,
    },
    {
        "category": "Range",
        "path": "mapping.det_range",
        "label": "Max Range",
        "type": "float",
        "default": 80.0,
        "tooltip": "Maximum mapping range in meters. Points beyond this are discarded.",
        "min": 5.0, "max": 500.0, "step": 5.0,
    },
    {
        "category": "Range",
        "path": "mapping.fov_degree",
        "label": "FOV Degrees",
        "type": "float",
        "default": 360.0,
        "tooltip": "Field-of-view in degrees. 360 for spinning/Ouster, narrower for directional.",
        "min": 30.0, "max": 360.0, "step": 10.0,
    },
    # --- IMU Noise ---
    {
        "category": "IMU Noise",
        "path": "mapping.acc_cov",
        "label": "Accel Covariance",
        "type": "float",
        "default": 0.1,
        "tooltip": "Accelerometer noise covariance. Higher = trust IMU less, rely more on lidar.",
        "min": 0.0001, "max": 10.0, "step": 0.01, "decimals": 4,
    },
    {
        "category": "IMU Noise",
        "path": "mapping.gyr_cov",
        "label": "Gyro Covariance",
        "type": "float",
        "default": 0.1,
        "tooltip": "Gyroscope noise covariance. Higher = trust IMU less.",
        "min": 0.0001, "max": 10.0, "step": 0.01, "decimals": 4,
    },
    {
        "category": "IMU Noise",
        "path": "mapping.b_acc_cov",
        "label": "Accel Bias Cov",
        "type": "float",
        "default": 0.0001,
        "tooltip": "Accelerometer bias random-walk covariance.",
        "min": 0.000001, "max": 1.0, "step": 0.0001, "decimals": 6,
    },
    {
        "category": "IMU Noise",
        "path": "mapping.b_gyr_cov",
        "label": "Gyro Bias Cov",
        "type": "float",
        "default": 0.0001,
        "tooltip": "Gyroscope bias random-walk covariance.",
        "min": 0.000001, "max": 1.0, "step": 0.0001, "decimals": 6,
    },
    # --- Extrinsics ---
    {
        "category": "Extrinsics",
        "path": "mapping.extrinsic_est_en",
        "label": "Online Calibration",
        "type": "bool",
        "default": False,
        "tooltip": "Enable online lidar-IMU extrinsic calibration. Usually false for Ouster (identity), true for Livox.",
    },
    {
        "category": "Extrinsics",
        "path": "mapping.extrinsic_T",
        "label": "Translation [x,y,z]",
        "type": "vector3",
        "default": [0.0, 0.0, 0.0],
        "tooltip": "Lidar-to-IMU translation vector in meters.",
        "min": -5.0, "max": 5.0, "step": 0.001, "decimals": 4,
    },
    {
        "category": "Extrinsics",
        "path": "mapping.extrinsic_R",
        "label": "Rotation Matrix",
        "type": "matrix3x3",
        "default": [1, 0, 0, 0, 1, 0, 0, 0, 1],
        "tooltip": "Lidar-to-IMU rotation matrix (row-major, 9 elements). Identity for os_sensor frame.",
        "min": -1.0, "max": 1.0, "step": 0.001, "decimals": 4,
    },
    # --- Preprocess ---
    {
        "category": "Preprocess",
        "path": "preprocess.lidar_type",
        "label": "Lidar Type",
        "type": "choice",
        "default": 3,
        "tooltip": "1 = Livox serial, 2 = Velodyne, 3 = Ouster.",
        "choices": [(1, "Livox"), (2, "Velodyne"), (3, "Ouster")],
    },
    {
        "category": "Preprocess",
        "path": "preprocess.scan_line",
        "label": "Scan Lines",
        "type": "int",
        "default": 32,
        "tooltip": "Number of lidar scan lines. Ouster OS0-32 = 32, Livox MID-360 = 4.",
        "min": 1, "max": 128, "step": 1,
    },
    {
        "category": "Preprocess",
        "path": "preprocess.timestamp_unit",
        "label": "Timestamp Unit",
        "type": "choice",
        "default": 3,
        "tooltip": "Point timestamp unit. 0=sec, 1=ms, 2=us, 3=ns.",
        "choices": [(0, "sec"), (1, "ms"), (2, "us"), (3, "ns")],
    },
    {
        "category": "Preprocess",
        "path": "feature_extract_enable",
        "label": "Feature Extract",
        "type": "bool",
        "default": False,
        "tooltip": "Enable feature extraction (edge/plane). Usually false for direct registration.",
    },
    # --- Publish ---
    {
        "category": "Publish",
        "path": "publish.path_en",
        "label": "Publish Path",
        "type": "bool",
        "default": True,
        "tooltip": "Publish odometry path for visualization.",
    },
    {
        "category": "Publish",
        "path": "publish.scan_publish_en",
        "label": "Publish Scan",
        "type": "bool",
        "default": True,
        "tooltip": "Publish current scan point cloud.",
    },
    {
        "category": "Publish",
        "path": "publish.dense_publish_en",
        "label": "Dense Publish",
        "type": "bool",
        "default": True,
        "tooltip": "Publish dense (all points) vs sparse (filtered) cloud. True = full resolution output.",
    },
    {
        "category": "Publish",
        "path": "publish.scan_bodyframe_pub_en",
        "label": "Body Frame Pub",
        "type": "bool",
        "default": True,
        "tooltip": "Publish scan in body frame (for scan matching visualization).",
    },
    # --- PCD ---
    {
        "category": "PCD",
        "path": "pcd_save.pcd_save_en",
        "label": "PCD Save Enable",
        "type": "bool",
        "default": True,
        "tooltip": "Enable PCD file saving via /map_save service.",
    },
    {
        "category": "PCD",
        "path": "pcd_save.interval",
        "label": "PCD Save Interval",
        "type": "int",
        "default": -1,
        "tooltip": "Auto-save interval in scans. -1 = manual only (via /map_save service).",
        "min": -1, "max": 1000, "step": 1,
    },
    {
        "category": "PCD",
        "path": "map_file_path",
        "label": "PCD File Path",
        "type": "string",
        "default": "./scan.pcd",
        "tooltip": "Output PCD file path (relative to working directory).",
    },
]


def _get_yaml_value(data, dotpath):
    """Get a value from nested dict using dot-separated path."""
    keys = dotpath.split(".")
    node = data
    for k in keys:
        if not isinstance(node, dict) or k not in node:
            return None
        node = node[k]
    return node


def _set_yaml_value(data, dotpath, value):
    """Set a value in nested dict using dot-separated path, creating intermediate dicts."""
    keys = dotpath.split(".")
    node = data
    for k in keys[:-1]:
        if k not in node or not isinstance(node[k], dict):
            node[k] = {}
        node = node[k]
    node[keys[-1]] = value


# ---------------------------------------------------------------------------
# Bag info parser
# ---------------------------------------------------------------------------

def parse_bag_info(scan_dir):
    """Parse bag metadata.yaml and detect sensor type."""
    bag_dir = scan_dir / "bag"
    meta_path = bag_dir / "metadata.yaml"
    info = {
        "duration_s": 0,
        "message_count": 0,
        "sensor": "unknown",
        "has_metadata": False,
        "bag_path": str(bag_dir) if bag_dir.exists() else None,
        "topics": [],
    }

    if not meta_path.exists():
        return info

    try:
        with open(meta_path) as f:
            meta = yaml.safe_load(f)

        bag_info = meta.get("rosbag2_bagfile_information", {})
        duration_ns = bag_info.get("duration", {}).get("nanoseconds", 0)
        info["duration_s"] = duration_ns / 1e9
        info["message_count"] = bag_info.get("message_count", 0)

        topics = bag_info.get("topics_with_message_count", [])
        topic_names = []
        for t in topics:
            name = t.get("topic_metadata", {}).get("name", "")
            topic_names.append(name)
        info["topics"] = topic_names

        if any("/ouster/" in t for t in topic_names):
            info["sensor"] = "ouster"
        elif any("/livox/" in t for t in topic_names):
            info["sensor"] = "livox"

    except Exception as e:
        print(f"[BagInfo] Failed to parse {meta_path}: {e}")

    # Check for Ouster metadata JSON
    import glob as globmod
    metadata_files = globmod.glob(str(scan_dir / "os-*-metadata.json"))
    info["has_metadata"] = len(metadata_files) > 0
    if metadata_files:
        info["metadata_path"] = metadata_files[0]

    return info


# ---------------------------------------------------------------------------
# Process Manager
# ---------------------------------------------------------------------------

class ProcessManager:
    """Manages FAST-LIO replay subprocesses."""

    IDLE = "idle"
    RUNNING = "running"
    STOPPING = "stopping"
    SAVING = "saving"
    COMPLETED = "completed"  # Bag finished, FAST-LIO still alive for PCD save
    CRASHED = "crashed"      # FAST-LIO died unexpectedly

    def __init__(self):
        self.state = self.IDLE
        self.processes = {}  # name -> Popen
        self.lock = threading.Lock()
        self.start_time = None
        self.frozen_elapsed = None  # Elapsed time frozen when bag finishes
        self.bag_duration = 0  # expected duration in seconds
        self.play_rate = 1.0
        self.start_at_offset = 0  # bag-seconds to skip at start
        self.output_dir = None
        self._stop_event = threading.Event()
        self._time_monitor = None
        self.status_log = []  # [(timestamp, message)] for status history

    def _log(self, msg):
        """Append a timestamped entry to the status log."""
        ts = datetime.now().strftime("%H:%M:%S")
        self.status_log.append((ts, msg))
        print(f"[Replay] {msg}")

    def _fastlio_alive(self):
        """Check if the FAST-LIO process is still running."""
        with self.lock:
            proc = self.processes.get("fastlio")
        return proc is not None and proc.poll() is None

    def start_replay(self, scan_dir, bag_info, config_yaml, rate=1.0,
                     stop_at=0, start_at=0, open_rviz=True):
        """Start the full replay pipeline."""
        with self.lock:
            if self.state not in (self.IDLE, self.COMPLETED, self.CRASHED):
                return False, f"Already {self.state}"
            self.state = self.RUNNING
            self._stop_event.clear()
            self.frozen_elapsed = None

        self.status_log.clear()
        self.bag_duration = bag_info["duration_s"]
        self.play_rate = rate
        self.start_at_offset = start_at

        thread = threading.Thread(
            target=self._run_pipeline,
            args=(scan_dir, bag_info, config_yaml, rate, stop_at, start_at,
                  open_rviz),
            daemon=True,
        )
        thread.start()
        return True, "Starting replay..."

    def _run_pipeline(self, scan_dir, bag_info, config_yaml, rate, stop_at,
                      start_at, open_rviz):
        try:
            sensor = bag_info["sensor"]
            scan_name = scan_dir.name

            # Create output directory: {reprocess_time}_{original_scan_name}
            timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            out_name = f"{timestamp}_{scan_name}"
            self.output_dir = REPROCESS_DIR / out_name
            self.output_dir.mkdir(parents=True, exist_ok=True)

            # Set PCD filename based on scan name
            pcd_name = f"reprocess_{scan_name}.pcd"
            config_yaml["/**"]["ros__parameters"]["map_file_path"] = f"./{pcd_name}"

            # Write config YAML to output dir
            config_path = self.output_dir / "fastlio_config.yaml"
            with open(config_path, "w") as f:
                yaml.dump(config_yaml, f, default_flow_style=False)

            # 1. Start FAST-LIO first
            self._log("Starting FAST-LIO...")
            fastlio_cmd = (
                f"{SOURCE_CMD} && "
                f"cd {self.output_dir} && "
                f"ros2 launch fast_lio mapping.launch.py "
                f"config_path:={self.output_dir} "
                f"config_file:=fastlio_config.yaml "
                f"rviz:=false use_sim_time:=true"
            )
            self._start_process("fastlio", fastlio_cmd)

            if self._stop_event.wait(5):
                return

            # 2. Start scan monitor
            self._log("Starting scan monitor...")
            monitor_cmd = (
                f"{SOURCE_CMD} && "
                f"/usr/bin/python3 {PROJECT_DIR}/scan_monitor.py"
            )
            self._start_process("monitor", monitor_cmd)

            # 3. Start RViz (optional)
            if open_rviz:
                self._log("Starting RViz...")
                rviz_cmd = (
                    f"{SOURCE_CMD} && "
                    f"ros2 run rviz2 rviz2 -d "
                    f"$(ros2 pkg prefix fast_lio)/share/fast_lio/rviz/fastlio.rviz "
                    f"--ros-args -p use_sim_time:=true"
                )
                self._start_process("rviz", rviz_cmd)

            if self._stop_event.wait(3):
                return

            # 4. Start replay
            offset_msg = f" (starting at {start_at}s)" if start_at > 0 else ""
            self._log(f"Starting {sensor} bag replay at {rate}x{offset_msg}...")
            bag_path = bag_info["bag_path"]
            start_offset_arg = f" --start-offset {start_at}" if start_at > 0 else ""

            if sensor == "ouster":
                # Start ouster driver nodes (without bag play)
                metadata_path = bag_info.get("metadata_path", "")
                driver_cmd = (
                    f"{SOURCE_CMD} && "
                    f"ros2 launch ouster_ros replay.composite.launch.xml "
                    f"bag_file:=b "
                    f"metadata:={metadata_path} "
                    f"viz:=false "
                    f"use_system_default_qos:=true "
                    f"point_cloud_frame:=os_sensor "
                    f"point_type:=original"
                )
                self._start_process("ouster_driver", driver_cmd)
                if self._stop_event.wait(5):
                    return

                # Start bag play separately (supports --start-offset)
                qos_path = (
                    ROS2_WS / "install" / "ouster_ros" / "share"
                    / "ouster_ros" / "config" / "metadata-qos-override.yaml"
                )
                replay_cmd = (
                    f"{SOURCE_CMD} && "
                    f"ros2 bag play {bag_path} --clock --rate {rate}"
                    f"{start_offset_arg}"
                    f" --remap /os_node/metadata:=/ouster/metadata"
                    f" /os_node/imu_packets:=/ouster/imu_packets"
                    f" /os_node/lidar_packets:=/ouster/lidar_packets"
                    f" --qos-profile-overrides-path {qos_path}"
                )
            else:
                replay_cmd = (
                    f"{SOURCE_CMD} && "
                    f"ros2 bag play {bag_path} --clock --rate {rate}"
                    f"{start_offset_arg}"
                )

            self.start_time = time.time()
            self._start_process("replay", replay_cmd)

            # 5. If stop_at > 0, start time monitor
            if stop_at > 0:
                self._time_monitor = threading.Thread(
                    target=self._watch_time, args=(stop_at, start_at, rate),
                    daemon=True,
                )
                self._time_monitor.start()

            # 6. Wait for replay OR FAST-LIO to finish (whichever comes first)
            replay_proc = self.processes.get("replay")
            fastlio_proc = self.processes.get("fastlio")
            while self.state == self.RUNNING:
                # Check FAST-LIO crash
                if fastlio_proc and fastlio_proc.poll() is not None:
                    rc = fastlio_proc.returncode
                    self._log(f"FAST-LIO crashed (exit code {rc})")
                    with self.lock:
                        if self.start_time:
                            self.frozen_elapsed = (
                                self.start_at_offset +
                                (time.time() - self.start_time) * self.play_rate
                            )
                        self.state = self.CRASHED
                    return

                # Check replay finished
                if replay_proc and replay_proc.poll() is not None:
                    break

                time.sleep(0.5)

            # If not stopped by user, replay finished naturally
            with self.lock:
                if self.state == self.RUNNING:
                    # Freeze elapsed time at bag duration
                    self.frozen_elapsed = self.bag_duration
                    self.state = self.COMPLETED
                    self._log("Bag replay finished — ready to save PCD")

                    # Give FAST-LIO a moment to process remaining frames
                    time.sleep(3)

        except Exception as e:
            self._log(f"Pipeline error: {e}")
        finally:
            with self.lock:
                # Only go idle if we weren't set to COMPLETED/CRASHED
                if self.state == self.RUNNING:
                    self.state = self.IDLE

    def _watch_time(self, stop_at, start_at, rate):
        """Monitor elapsed time and stop replay when stop_at is reached."""
        # Wall time needed = (stop_at - start_at) bag-seconds at given rate
        play_duration = max(stop_at - start_at, 1)
        wall_stop = play_duration / rate
        while not self._stop_event.wait(1):
            if self.start_time is None:
                continue
            elapsed = time.time() - self.start_time
            if elapsed >= wall_stop:
                self._log(f"Stop-at time reached ({stop_at}s of bag)")
                self.save_pcd()
                # Kill only the replay process — let FAST-LIO flush
                self._kill_process("replay")
                time.sleep(3)
                self.stop()
                return

    def _start_process(self, name, cmd):
        proc = subprocess.Popen(
            cmd, shell=True, executable="/bin/bash",
            preexec_fn=os.setsid,
        )
        with self.lock:
            self.processes[name] = proc
        return proc

    def _kill_process(self, name, sig=signal.SIGINT):
        with self.lock:
            proc = self.processes.get(name)
        if proc is None:
            return
        try:
            os.killpg(os.getpgid(proc.pid), sig)
        except (ProcessLookupError, OSError):
            pass

    def _copy_logs_to_output(self):
        """Copy FAST-LIO log files (trajectory, etc.) to the output directory."""
        if self.output_dir is None:
            return
        import shutil
        for name in ("pos_log.txt", "mat_out.txt"):
            src = FASTLIO_LOG_DIR / name
            if src.is_file():
                try:
                    shutil.copy2(src, self.output_dir / name)
                    self._log(f"Copied {name} to output")
                except Exception as e:
                    self._log(f"Failed to copy {name}: {e}")

    def save_pcd(self):
        """Call FAST-LIO /map_save service. Skips if FAST-LIO is dead."""
        if not self._fastlio_alive():
            self._log("Cannot save PCD — FAST-LIO is not running")
            return False

        with self.lock:
            prev_state = self.state
            self.state = self.SAVING

        self._log("Saving PCD via /map_save...")
        ok = False
        try:
            cmd = f"{SOURCE_CMD} && ros2 service call /map_save std_srvs/srv/Trigger"
            result = subprocess.run(
                cmd, shell=True, executable="/bin/bash",
                capture_output=True, text=True, timeout=30,
            )
            if result.returncode == 0:
                self._log("PCD saved successfully")
                self._copy_logs_to_output()
                ok = True
            else:
                self._log(f"/map_save failed: {result.stderr.strip()}")
        except subprocess.TimeoutExpired:
            self._log("/map_save timed out — FAST-LIO may be unresponsive")
        except Exception as e:
            self._log(f"/map_save error: {e}")
        finally:
            with self.lock:
                if self.state == self.SAVING:
                    if prev_state == self.COMPLETED:
                        self.state = self.COMPLETED
                    elif any(p.poll() is None for p in self.processes.values()):
                        self.state = self.RUNNING
                    else:
                        self.state = self.IDLE
        return ok

    def stop(self, save_first=True):
        """Stop all processes. Saves PCD first unless save_first=False."""
        with self.lock:
            if self.state == self.IDLE:
                return
            prev_state = self.state
            self.state = self.STOPPING
        self._stop_event.set()

        # Save PCD before killing FAST-LIO (unless crashed, already saved, or skipped)
        if save_first and prev_state in (self.RUNNING, self.COMPLETED):
            if self._fastlio_alive():
                self._log("Saving PCD before stopping...")
                try:
                    cmd = f"{SOURCE_CMD} && ros2 service call /map_save std_srvs/srv/Trigger"
                    result = subprocess.run(
                        cmd, shell=True, executable="/bin/bash",
                        capture_output=True, text=True, timeout=30,
                    )
                    if result.returncode == 0:
                        self._log("PCD saved")
                        self._copy_logs_to_output()
                    else:
                        self._log(f"PCD save failed: {result.stderr.strip()}")
                except subprocess.TimeoutExpired:
                    self._log("PCD save timed out — FAST-LIO may be unresponsive")
                except Exception as e:
                    self._log(f"PCD save error: {e}")
            else:
                self._log("Skipping PCD save — FAST-LIO is not running")

        # Kill RViz first to avoid SIGSEGV
        self._kill_process("rviz", signal.SIGKILL)
        time.sleep(0.5)

        # SIGINT the rest
        for name in ["replay", "ouster_driver", "monitor", "fastlio"]:
            self._kill_process(name, signal.SIGINT)

        # Wait with timeout
        deadline = time.time() + 10
        with self.lock:
            procs = list(self.processes.values())
        for proc in procs:
            remaining = max(0.1, deadline - time.time())
            try:
                proc.wait(timeout=remaining)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except (ProcessLookupError, OSError):
                    pass

        # Freeze elapsed if not already frozen
        if self.frozen_elapsed is None and self.start_time is not None:
            self.frozen_elapsed = self.start_at_offset + (time.time() - self.start_time) * self.play_rate

        with self.lock:
            self.processes.clear()
            self.state = self.IDLE
            self.start_time = None

        # Clean up health file
        try:
            os.remove(HEALTH_FILE)
        except OSError:
            pass

        self._log("All processes stopped.")

    def get_elapsed_bag_seconds(self):
        """Get elapsed time in bag-seconds (adjusted for rate and start offset)."""
        if self.frozen_elapsed is not None:
            return self.frozen_elapsed
        if self.start_time is None:
            return self.start_at_offset if self.start_at_offset else 0
        return self.start_at_offset + (time.time() - self.start_time) * self.play_rate

    def is_active(self):
        """True if processes are running or bag completed (awaiting save/stop)."""
        return self.state in (self.RUNNING, self.SAVING, self.COMPLETED, self.CRASHED)


# ---------------------------------------------------------------------------
# Main Window
# ---------------------------------------------------------------------------

class ReprocessorWindow(QMainWindow):
    # Signal to update status from worker threads
    status_signal = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("FAST-LIO Reprocessor")
        self.setMinimumSize(900, 650)

        self.process_mgr = ProcessManager()
        self.param_widgets = {}  # path -> widget
        self.scan_dirs = []
        self.current_bag_info = None
        self._prev_log_len = 0  # Track log growth for status updates
        self._prev_health_status = None  # Track health changes for logging
        self._prev_health_msg = None
        self._last_health_issue_time = 0.0  # When the last warn/error started

        self._build_ui()
        self._load_scan_list()
        self._populate_presets()

        # Status timer — poll health + process state
        self.status_timer = QTimer()
        self.status_timer.timeout.connect(self._update_status)
        self.status_timer.start(500)

        self.status_signal.connect(self._set_status_text)

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setSpacing(6)
        main_layout.setContentsMargins(8, 8, 8, 8)

        # --- Row 1: Scan selector ---
        scan_group = QGroupBox("Scan Selection")
        scan_layout = QHBoxLayout(scan_group)

        scan_layout.addWidget(QLabel("Scan:"))
        self.scan_combo = QComboBox()
        self.scan_combo.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.scan_combo.currentIndexChanged.connect(self._on_scan_changed)
        scan_layout.addWidget(self.scan_combo, 1)

        browse_btn = QPushButton("Browse...")
        browse_btn.clicked.connect(self._browse_scan)
        scan_layout.addWidget(browse_btn)

        self.bag_info_label = QLabel("")
        self.bag_info_label.setStyleSheet("color: #aaa;")
        scan_layout.addWidget(self.bag_info_label)

        main_layout.addWidget(scan_group)

        # --- Row 2: Parameters + Health (splitter) ---
        splitter = QSplitter(Qt.Horizontal)

        # Left: parameter editor
        param_widget = QWidget()
        param_layout = QVBoxLayout(param_widget)
        param_layout.setContentsMargins(0, 0, 0, 0)

        # Preset row
        preset_row = QHBoxLayout()
        preset_row.addWidget(QLabel("Preset:"))
        self.preset_combo = QComboBox()
        self.preset_combo.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        preset_row.addWidget(self.preset_combo, 1)

        load_btn = QPushButton("Load")
        load_btn.clicked.connect(self._load_preset)
        preset_row.addWidget(load_btn)

        save_btn = QPushButton("Save As...")
        save_btn.clicked.connect(self._save_preset)
        preset_row.addWidget(save_btn)

        param_layout.addLayout(preset_row)

        # Parameter tabs
        self.param_tabs = QTabWidget()
        self._build_param_tabs()
        param_layout.addWidget(self.param_tabs)

        splitter.addWidget(param_widget)

        # Right: health monitor
        health_widget = QWidget()
        health_layout = QVBoxLayout(health_widget)
        health_layout.setContentsMargins(0, 0, 0, 0)

        health_group = QGroupBox("Health Monitor")
        health_grid = QGridLayout(health_group)
        health_grid.setSpacing(4)

        row = 0
        self.health_status = QLabel("Idle")
        self.health_status.setFont(QFont("monospace", 10, QFont.Bold))
        health_grid.addWidget(QLabel("Status:"), row, 0)
        health_grid.addWidget(self.health_status, row, 1)
        row += 1

        self.health_labels = {}
        for key, label in [
            ("pos_cov", "Pos Cov:"),
            ("rot_cov", "Rot Cov:"),
            ("speed", "Speed:"),
            ("points", "Points:"),
            ("msg_rate", "Msg Rate:"),
        ]:
            lbl = QLabel("—")
            lbl.setFont(QFont("monospace", 10))
            health_grid.addWidget(QLabel(label), row, 0)
            health_grid.addWidget(lbl, row, 1)
            self.health_labels[key] = lbl
            row += 1

        self.elapsed_label = QLabel("00:00 / 00:00")
        self.elapsed_label.setFont(QFont("monospace", 10))
        health_grid.addWidget(QLabel("Elapsed:"), row, 0)
        health_grid.addWidget(self.elapsed_label, row, 1)

        health_layout.addWidget(health_group)
        health_layout.addStretch()

        splitter.addWidget(health_widget)
        splitter.setSizes([550, 350])

        main_layout.addWidget(splitter, 1)

        # --- Row 3: Playback controls ---
        playback_group = QGroupBox("Playback")
        pb_layout = QHBoxLayout(playback_group)

        pb_layout.addWidget(QLabel("Rate:"))
        self.rate_spin = QDoubleSpinBox()
        self.rate_spin.setRange(0.1, 10.0)
        self.rate_spin.setValue(1.0)
        self.rate_spin.setSingleStep(0.1)
        self.rate_spin.setSuffix("x")
        pb_layout.addWidget(self.rate_spin)

        pb_layout.addWidget(QLabel("Start at:"))
        self.start_at_spin = QDoubleSpinBox()
        self.start_at_spin.setRange(0, 99999)
        self.start_at_spin.setValue(0)
        self.start_at_spin.setSuffix("s")
        self.start_at_spin.setToolTip("Skip this many bag-seconds at the start. Useful if the scanner was being picked up or jostled. 0 = start from beginning.")
        pb_layout.addWidget(self.start_at_spin)

        pb_layout.addWidget(QLabel("Stop at:"))
        self.stop_at_spin = QDoubleSpinBox()
        self.stop_at_spin.setRange(0, 99999)
        self.stop_at_spin.setValue(0)
        self.stop_at_spin.setSuffix("s")
        self.stop_at_spin.setToolTip("Stop replay after this many bag-seconds. 0 = play full bag.")
        pb_layout.addWidget(self.stop_at_spin)

        self.rviz_check = QCheckBox("Open RViz")
        self.rviz_check.setChecked(True)
        pb_layout.addWidget(self.rviz_check)

        pb_layout.addStretch()
        main_layout.addWidget(playback_group)

        # --- Row 4: Action buttons + status ---
        action_layout = QHBoxLayout()

        self.process_btn = QPushButton("  PROCESS  ")
        self.process_btn.setToolTip(
            "Start replaying the selected bag through FAST-LIO with the current parameters."
        )
        self.process_btn.setStyleSheet(
            "QPushButton { background-color: #2d7d46; color: white; font-weight: bold; "
            "padding: 8px 20px; font-size: 13px; }"
            "QPushButton:hover { background-color: #35915a; }"
            "QPushButton:disabled { background-color: #555; color: #888; }"
        )
        self.process_btn.clicked.connect(self._start_processing)
        action_layout.addWidget(self.process_btn)

        self.save_btn = QPushButton("  SAVE PCD  ")
        self.save_btn.setToolTip(
            "Save the current point cloud to scan.pcd via FAST-LIO's /map_save service.\n"
            "You can save at any time during or after replay finishes."
        )
        self.save_btn.setStyleSheet(
            "QPushButton { background-color: #2d5f7d; color: white; font-weight: bold; "
            "padding: 8px 20px; font-size: 13px; }"
            "QPushButton:hover { background-color: #357191; }"
            "QPushButton:disabled { background-color: #555; color: #888; }"
        )
        self.save_btn.setEnabled(False)
        self.save_btn.clicked.connect(self._save_pcd)
        action_layout.addWidget(self.save_btn)

        self.stop_btn = QPushButton("  STOP  ")
        self.stop_btn.setToolTip(
            "Save PCD and stop all processes (FAST-LIO, replay, RViz).\n"
            "PCD is always saved automatically before shutdown."
        )
        self.stop_btn.setStyleSheet(
            "QPushButton { background-color: #8b2020; color: white; font-weight: bold; "
            "padding: 8px 20px; font-size: 13px; }"
            "QPushButton:hover { background-color: #a52a2a; }"
            "QPushButton:disabled { background-color: #555; color: #888; }"
        )
        self.stop_btn.setEnabled(False)
        self.stop_btn.clicked.connect(self._stop_processing)
        action_layout.addWidget(self.stop_btn)

        self.log_btn = QPushButton("  LOG  ")
        self.log_btn.setToolTip("Show a log of all status changes and events.")
        self.log_btn.setStyleSheet(
            "QPushButton { background-color: #555; color: white; font-weight: bold; "
            "padding: 8px 14px; font-size: 13px; }"
            "QPushButton:hover { background-color: #666; }"
        )
        self.log_btn.clicked.connect(self._show_log)
        action_layout.addWidget(self.log_btn)

        action_layout.addStretch()

        self.status_label = QLabel("Status: Idle")
        self.status_label.setFont(QFont("monospace", 10))
        action_layout.addWidget(self.status_label)

        main_layout.addLayout(action_layout)

    def _build_param_tabs(self):
        """Create parameter editor tabs from PARAM_SCHEMA."""
        categories = {}
        for param in PARAM_SCHEMA:
            cat = param["category"]
            if cat not in categories:
                categories[cat] = []
            categories[cat].append(param)

        for cat_name, params in categories.items():
            tab = QWidget()
            layout = QGridLayout(tab)
            layout.setSpacing(4)

            for row, param in enumerate(params):
                label = QLabel(param["label"])
                label.setToolTip(param["tooltip"])
                layout.addWidget(label, row, 0)

                widget = self._create_param_widget(param)
                if param["type"] in ("vector3", "matrix3x3"):
                    # These return a container widget
                    layout.addWidget(widget, row, 1, 1, 2)
                else:
                    layout.addWidget(widget, row, 1)
                    # Add unit hint if applicable
                    if param["type"] == "float" and param.get("min", 0) >= 0:
                        pass  # no unit label needed

                self.param_widgets[param["path"]] = widget

            layout.setRowStretch(len(params), 1)
            self.param_tabs.addTab(tab, cat_name)

    def _create_param_widget(self, param):
        """Create the appropriate widget for a parameter."""
        ptype = param["type"]

        if ptype == "float":
            w = QDoubleSpinBox()
            w.setRange(param.get("min", -1e9), param.get("max", 1e9))
            w.setSingleStep(param.get("step", 0.01))
            w.setDecimals(param.get("decimals", 3))
            w.setValue(param["default"])
            w.setToolTip(param["tooltip"])
            return w

        elif ptype == "int":
            w = QSpinBox()
            w.setRange(param.get("min", -999999), param.get("max", 999999))
            w.setSingleStep(param.get("step", 1))
            w.setValue(param["default"])
            w.setToolTip(param["tooltip"])
            return w

        elif ptype == "bool":
            w = QCheckBox()
            w.setChecked(param["default"])
            w.setToolTip(param["tooltip"])
            return w

        elif ptype == "choice":
            w = QComboBox()
            for val, text in param["choices"]:
                w.addItem(text, val)
            # Set default
            idx = w.findData(param["default"])
            if idx >= 0:
                w.setCurrentIndex(idx)
            w.setToolTip(param["tooltip"])
            return w

        elif ptype == "string":
            w = QLineEdit()
            w.setText(str(param["default"]))
            w.setToolTip(param["tooltip"])
            return w

        elif ptype == "vector3":
            container = QWidget()
            layout = QHBoxLayout(container)
            layout.setContentsMargins(0, 0, 0, 0)
            layout.setSpacing(2)
            spins = []
            for i, val in enumerate(param["default"]):
                s = QDoubleSpinBox()
                s.setRange(param.get("min", -1e6), param.get("max", 1e6))
                s.setSingleStep(param.get("step", 0.001))
                s.setDecimals(param.get("decimals", 4))
                s.setValue(val)
                s.setToolTip(param["tooltip"])
                layout.addWidget(s)
                spins.append(s)
            container._spins = spins
            return container

        elif ptype == "matrix3x3":
            container = QWidget()
            layout = QGridLayout(container)
            layout.setContentsMargins(0, 0, 0, 0)
            layout.setSpacing(2)
            spins = []
            for i in range(9):
                s = QDoubleSpinBox()
                s.setRange(param.get("min", -1e6), param.get("max", 1e6))
                s.setSingleStep(param.get("step", 0.001))
                s.setDecimals(param.get("decimals", 4))
                s.setValue(param["default"][i])
                s.setToolTip(param["tooltip"])
                layout.addWidget(s, i // 3, i % 3)
                spins.append(s)
            container._spins = spins
            return container

        # Fallback
        return QLabel(f"<{ptype}>")

    def _get_param_value(self, param):
        """Read current value from a parameter widget."""
        widget = self.param_widgets.get(param["path"])
        if widget is None:
            return param["default"]

        ptype = param["type"]
        if ptype == "float":
            return widget.value()
        elif ptype == "int":
            return widget.value()
        elif ptype == "bool":
            return widget.isChecked()
        elif ptype == "choice":
            return widget.currentData()
        elif ptype == "string":
            return widget.text()
        elif ptype == "vector3":
            return [s.value() for s in widget._spins]
        elif ptype == "matrix3x3":
            return [s.value() for s in widget._spins]
        return param["default"]

    def _set_param_value(self, param, value):
        """Set a parameter widget to a given value."""
        widget = self.param_widgets.get(param["path"])
        if widget is None:
            return

        ptype = param["type"]
        try:
            if ptype == "float":
                widget.setValue(float(value))
            elif ptype == "int":
                widget.setValue(int(value))
            elif ptype == "bool":
                widget.setChecked(bool(value))
            elif ptype == "choice":
                idx = widget.findData(value)
                if idx >= 0:
                    widget.setCurrentIndex(idx)
            elif ptype == "string":
                widget.setText(str(value))
            elif ptype == "vector3":
                for i, s in enumerate(widget._spins):
                    if i < len(value):
                        s.setValue(float(value[i]))
            elif ptype == "matrix3x3":
                for i, s in enumerate(widget._spins):
                    if i < len(value):
                        s.setValue(float(value[i]))
        except (TypeError, ValueError, IndexError):
            pass

    # --- Scan selection ---

    def _load_scan_list(self):
        """Populate scan dropdown from ~/pointclouds/."""
        self.scan_combo.clear()
        self.scan_dirs = []

        if not POINTCLOUDS_DIR.exists():
            return

        for d in sorted(POINTCLOUDS_DIR.iterdir(), reverse=True):
            if d.is_dir() and (d / "bag").is_dir():
                self.scan_dirs.append(d)
                self.scan_combo.addItem(d.name)

    def _on_scan_changed(self, index):
        """Update bag info when scan selection changes."""
        if index < 0 or index >= len(self.scan_dirs):
            self.bag_info_label.setText("")
            self.current_bag_info = None
            return

        scan_dir = self.scan_dirs[index]
        info = parse_bag_info(scan_dir)
        self.current_bag_info = info

        dur = info["duration_s"]
        mins, secs = divmod(int(dur), 60)
        meta_icon = "\u2713" if info["has_metadata"] else "\u2717"
        self.bag_info_label.setText(
            f"{mins}m{secs:02d}s | {info['message_count']:,} msgs | "
            f"{info['sensor']} | metadata: {meta_icon}"
        )

    def _browse_scan(self):
        """Open file dialog to select a scan directory."""
        path = QFileDialog.getExistingDirectory(
            self, "Select Scan Directory", str(POINTCLOUDS_DIR)
        )
        if path:
            scan_dir = Path(path)
            if (scan_dir / "bag").is_dir():
                # Add to list if not already there
                if scan_dir not in self.scan_dirs:
                    self.scan_dirs.insert(0, scan_dir)
                    self.scan_combo.insertItem(0, scan_dir.name)
                self.scan_combo.setCurrentIndex(
                    self.scan_dirs.index(scan_dir)
                )
            else:
                QMessageBox.warning(
                    self, "Invalid Scan",
                    "Selected directory does not contain a 'bag' subdirectory."
                )

    # --- Preset management ---

    def _populate_presets(self):
        """Fill preset dropdown with built-in + custom presets."""
        self.preset_combo.clear()

        # Built-in presets from FAST-LIO config dir
        if FASTLIO_CONFIG_DIR.exists():
            for f in sorted(FASTLIO_CONFIG_DIR.glob("*.yaml")):
                self.preset_combo.addItem(f"[built-in] {f.stem}", str(f))

        # Custom presets
        if PRESET_DIR.exists():
            for f in sorted(PRESET_DIR.glob("*.yaml")):
                self.preset_combo.addItem(f"[custom] {f.stem}", str(f))

    def _load_preset(self):
        """Load selected preset YAML into parameter widgets."""
        path = self.preset_combo.currentData()
        if not path:
            return
        self._load_yaml_file(path)

    def _load_yaml_file(self, path):
        """Load a YAML config file into the parameter widgets."""
        try:
            with open(path) as f:
                raw = yaml.safe_load(f)

            # Navigate to ros__parameters
            data = raw
            if "/**" in data:
                data = data["/**"]
            if "ros__parameters" in data:
                data = data["ros__parameters"]

            for param in PARAM_SCHEMA:
                val = _get_yaml_value(data, param["path"])
                if val is not None:
                    self._set_param_value(param, val)

            print(f"[Preset] Loaded: {path}")
        except Exception as e:
            QMessageBox.warning(self, "Load Error", f"Failed to load preset:\n{e}")

    def _save_preset(self):
        """Save current parameters as a custom preset."""
        name, ok = QInputDialog.getText(
            self, "Save Preset", "Preset name:"
        )
        if not ok or not name.strip():
            return

        PRESET_DIR.mkdir(parents=True, exist_ok=True)
        path = PRESET_DIR / f"{name.strip()}.yaml"

        config = self._build_config_yaml()
        try:
            with open(path, "w") as f:
                yaml.dump(config, f, default_flow_style=False)
            print(f"[Preset] Saved: {path}")
            self._populate_presets()
        except Exception as e:
            QMessageBox.warning(self, "Save Error", f"Failed to save preset:\n{e}")

    def _build_config_yaml(self):
        """Build a complete FAST-LIO config YAML from current widget values."""
        data = {}
        for param in PARAM_SCHEMA:
            val = self._get_param_value(param)
            _set_yaml_value(data, param["path"], val)

        # Ensure topics are set based on sensor type
        if self.current_bag_info and self.current_bag_info["sensor"] == "livox":
            _set_yaml_value(data, "common.lid_topic", "/livox/lidar")
            _set_yaml_value(data, "common.imu_topic", "/livox/imu")
        else:
            _set_yaml_value(data, "common.lid_topic", "/ouster/points")
            _set_yaml_value(data, "common.imu_topic", "/ouster/imu")

        # Defaults that aren't in the schema but are required
        data.setdefault("runtime_pos_log_enable", True)
        if "common" not in data:
            data["common"] = {}
        data["common"].setdefault("time_sync_en", False)
        data["common"].setdefault("time_offset_lidar_to_imu", 0.0)

        # Wrap in ROS 2 parameter format
        return {"/**": {"ros__parameters": data}}

    # --- Processing ---

    def _start_processing(self):
        """Start the replay pipeline."""
        # If currently in COMPLETED/CRASHED state, stop old processes first
        if self.process_mgr.state in (
            self.process_mgr.COMPLETED, self.process_mgr.CRASHED,
        ):
            self.process_mgr.stop(save_first=False)

        idx = self.scan_combo.currentIndex()
        if idx < 0 or idx >= len(self.scan_dirs):
            QMessageBox.warning(self, "No Scan", "Please select a scan first.")
            return

        scan_dir = self.scan_dirs[idx]
        bag_info = parse_bag_info(scan_dir)

        if not bag_info["bag_path"]:
            QMessageBox.warning(self, "No Bag", "Selected scan has no bag directory.")
            return

        if bag_info["sensor"] == "ouster" and not bag_info["has_metadata"]:
            QMessageBox.warning(
                self, "Missing Metadata",
                "Ouster bag requires metadata JSON for replay.\n"
                "No os-*-metadata.json found in the scan directory."
            )
            return

        config_yaml = self._build_config_yaml()
        rate = self.rate_spin.value()
        start_at = self.start_at_spin.value()
        stop_at = self.stop_at_spin.value()
        open_rviz = self.rviz_check.isChecked()

        # Validate start/stop
        if stop_at > 0 and start_at >= stop_at:
            QMessageBox.warning(
                self, "Invalid Range",
                f"Start time ({start_at}s) must be less than stop time ({stop_at}s)."
            )
            return

        ok, msg = self.process_mgr.start_replay(
            scan_dir, bag_info, config_yaml, rate, stop_at, start_at, open_rviz,
        )

        if ok:
            self._prev_health_status = None
            self._prev_health_msg = None
            self._last_health_issue_time = 0.0
            self.process_btn.setEnabled(False)
            self.stop_btn.setEnabled(True)
            self.save_btn.setEnabled(True)
            self._set_status_text("Starting...")
        else:
            QMessageBox.warning(self, "Cannot Start", msg)

    def _stop_processing(self):
        """Save PCD and stop the replay pipeline."""
        self._set_status_text("Saving PCD & stopping...")

        def _do_stop():
            self.process_mgr.stop(save_first=True)
            self.status_signal.emit("Stopped — PCD saved")

        threading.Thread(target=_do_stop, daemon=True).start()

    def _save_pcd(self):
        """Save PCD while running."""
        self._set_status_text("Saving PCD...")

        def _do_save():
            self.process_mgr.save_pcd()
            self.status_signal.emit("PCD saved")

        threading.Thread(target=_do_save, daemon=True).start()

    def _set_status_text(self, text):
        self.status_label.setText(f"Status: {text}")

    def _show_log(self):
        """Show a dialog with the full status change log."""
        dlg = QDialog(self)
        dlg.setWindowTitle("Status Log")
        dlg.setMinimumSize(500, 400)
        layout = QVBoxLayout(dlg)

        text = QTextEdit()
        text.setReadOnly(True)
        text.setFont(QFont("monospace", 10))
        if self.process_mgr.status_log:
            lines = [f"[{ts}] {msg}" for ts, msg in self.process_mgr.status_log]
            text.setPlainText("\n".join(lines))
        else:
            text.setPlainText("No log entries yet.")
        layout.addWidget(text)

        buttons = QDialogButtonBox(QDialogButtonBox.Close)
        buttons.rejected.connect(dlg.close)
        layout.addWidget(buttons)

        dlg.exec_()

    def _update_status(self):
        """Timer callback — update health display and button states."""
        mgr = self.process_mgr
        state = mgr.state

        # Button states
        self.process_btn.setEnabled(state in (mgr.IDLE, mgr.COMPLETED, mgr.CRASHED))
        self.stop_btn.setEnabled(state in (
            mgr.RUNNING, mgr.SAVING, mgr.COMPLETED, mgr.STOPPING, mgr.CRASHED,
        ))
        self.save_btn.setEnabled(state in (mgr.RUNNING, mgr.COMPLETED))

        # Pick up new log entries for status bar
        log = mgr.status_log
        if len(log) > self._prev_log_len:
            latest = log[-1][1]
            self._set_status_text(latest)
            self._prev_log_len = len(log)

        # Update elapsed time
        bag_elapsed = mgr.get_elapsed_bag_seconds()
        total = mgr.bag_duration
        if self.stop_at_spin.value() > 0:
            total = min(total, self.stop_at_spin.value())

        # Cap elapsed at total so display doesn't count past the end
        display_elapsed = min(bag_elapsed, total)
        b_min, b_sec = divmod(int(display_elapsed), 60)
        t_min, t_sec = divmod(int(total), 60)

        if state == mgr.COMPLETED:
            self.elapsed_label.setText(
                f"{t_min:02d}:{t_sec:02d} / {t_min:02d}:{t_sec:02d}"
            )
        elif state == mgr.CRASHED:
            c_min, c_sec = divmod(int(bag_elapsed), 60)
            self.elapsed_label.setText(
                f"{c_min:02d}:{c_sec:02d} / {t_min:02d}:{t_sec:02d} (crashed)"
            )
        elif state in (mgr.RUNNING, mgr.SAVING):
            self.elapsed_label.setText(
                f"{b_min:02d}:{b_sec:02d} / {t_min:02d}:{t_sec:02d}"
            )

        # Health monitor display — terminal states
        if state == mgr.COMPLETED:
            self.health_status.setText("Bag completed — ready to save PCD")
            self.health_status.setStyleSheet("color: #4caf50;")
            return

        if state == mgr.CRASHED:
            self.health_status.setText("FAST-LIO crashed — check log for details")
            self.health_status.setStyleSheet("color: #f44336;")
            return

        if state == mgr.IDLE:
            return

        # Read health file for RUNNING/SAVING/STOPPING states
        try:
            hstat = os.stat(HEALTH_FILE)
            if time.time() - hstat.st_mtime < 5:
                with open(HEALTH_FILE) as f:
                    health = json.load(f)

                status = health.get("status", "unknown")
                message = health.get("message", status)
                colors = {"ok": "#4caf50", "warn": "#ff9800", "error": "#f44336"}
                color = colors.get(status, "#888")
                self.health_status.setText(message)
                self.health_status.setStyleSheet(f"color: {color};")

                # Log health changes (jumps, drifts, warnings)
                if message != self._prev_health_msg:
                    if status in ("warn", "error"):
                        if self._prev_health_status not in ("warn", "error"):
                            self._last_health_issue_time = time.time()
                        mgr._log(f"Health {status}: {message}")
                    elif self._prev_health_status in ("warn", "error"):
                        # Only log recovery if the issue lasted more than 5s
                        if time.time() - self._last_health_issue_time > 5:
                            mgr._log("Health recovered: ok")
                    self._prev_health_status = status
                    self._prev_health_msg = message

                for key in ("pos_cov", "rot_cov", "speed", "points", "msg_rate"):
                    val = health.get(key, "\u2014")
                    if isinstance(val, float):
                        if key in ("pos_cov", "rot_cov"):
                            self.health_labels[key].setText(f"{val:.6f}")
                        else:
                            self.health_labels[key].setText(f"{val:.1f}")
                    else:
                        self.health_labels[key].setText(str(val))
                    self.health_labels[key].setStyleSheet(f"color: {color};")

                if state == mgr.RUNNING:
                    self._set_status_text(f"Running ({status})")
            else:
                if state == mgr.RUNNING:
                    self.health_status.setText("Waiting for data...")
                    self.health_status.setStyleSheet("color: #888;")
        except (FileNotFoundError, json.JSONDecodeError, OSError):
            if state == mgr.RUNNING:
                self.health_status.setText("Waiting for data...")
                self.health_status.setStyleSheet("color: #888;")

    def closeEvent(self, event):
        """Clean up on window close."""
        if self.process_mgr.is_active():
            reply = QMessageBox.question(
                self, "Processes Running",
                "FAST-LIO is still running. Save PCD, stop all processes, and exit?",
                QMessageBox.Yes | QMessageBox.No,
            )
            if reply == QMessageBox.No:
                event.ignore()
                return

        self.status_timer.stop()
        if self.process_mgr.state != self.process_mgr.IDLE:
            self.process_mgr.stop(save_first=True)
        event.accept()


# ---------------------------------------------------------------------------
# Dark theme
# ---------------------------------------------------------------------------

def apply_dark_theme(app):
    app.setStyle("Fusion")
    palette = QPalette()
    palette.setColor(QPalette.Window, QColor(45, 45, 48))
    palette.setColor(QPalette.WindowText, QColor(212, 212, 212))
    palette.setColor(QPalette.Base, QColor(30, 30, 30))
    palette.setColor(QPalette.AlternateBase, QColor(45, 45, 48))
    palette.setColor(QPalette.ToolTipBase, QColor(60, 60, 60))
    palette.setColor(QPalette.ToolTipText, QColor(212, 212, 212))
    palette.setColor(QPalette.Text, QColor(212, 212, 212))
    palette.setColor(QPalette.Button, QColor(55, 55, 58))
    palette.setColor(QPalette.ButtonText, QColor(212, 212, 212))
    palette.setColor(QPalette.BrightText, QColor(255, 255, 255))
    palette.setColor(QPalette.Link, QColor(42, 130, 218))
    palette.setColor(QPalette.Highlight, QColor(42, 130, 218))
    palette.setColor(QPalette.HighlightedText, QColor(255, 255, 255))
    palette.setColor(QPalette.Disabled, QPalette.Text, QColor(128, 128, 128))
    palette.setColor(QPalette.Disabled, QPalette.ButtonText, QColor(128, 128, 128))
    app.setPalette(palette)

    app.setStyleSheet("""
        QGroupBox {
            border: 1px solid #555;
            border-radius: 4px;
            margin-top: 8px;
            padding-top: 8px;
            font-weight: bold;
        }
        QGroupBox::title {
            subcontrol-origin: margin;
            left: 10px;
            padding: 0 4px;
        }
        QTabWidget::pane { border: 1px solid #555; }
        QTabBar::tab {
            background: #3a3a3d;
            color: #ccc;
            padding: 5px 12px;
            margin-right: 2px;
            border-top-left-radius: 4px;
            border-top-right-radius: 4px;
        }
        QTabBar::tab:selected { background: #505053; color: white; }
        QToolTip {
            background-color: #3a3a3d;
            color: #ddd;
            border: 1px solid #666;
            padding: 4px;
        }
        QSplitter::handle { background-color: #555; width: 2px; }
    """)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    app = QApplication(sys.argv)
    apply_dark_theme(app)

    window = ReprocessorWindow()
    window.show()

    # Clean shutdown on Ctrl-C
    signal.signal(signal.SIGINT, lambda *_: (window.close(), app.quit()))
    # Allow Python signal handling inside Qt event loop
    timer = QTimer()
    timer.timeout.connect(lambda: None)
    timer.start(200)

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()

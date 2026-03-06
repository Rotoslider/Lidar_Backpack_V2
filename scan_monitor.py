#!/usr/bin/env python3
"""Scan health monitor — subscribes to /Odometry and /cloud_registered to detect drift."""

import json
import math
import os
import sys
import tempfile
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2


HEALTH_FILE = "/tmp/scan_health.json"

# Covariance thresholds
POS_COV_WARN = 0.5
POS_COV_ERROR = 2.0
ROT_COV_WARN = 0.1
ROT_COV_ERROR = 0.5

# Speed thresholds (m/s)
SPEED_WARN = 3.0     # walking ~1.5 m/s
SPEED_ERROR = 8.0    # impossible on foot

# Position jump — single-step displacement (meters)
JUMP_WARN = 0.3      # 30cm in one odom step is suspicious
JUMP_ERROR = 0.8     # 80cm in one step is a re-localization failure

# How long to keep jump warning active after the last jump (seconds)
JUMP_COOLDOWN = 10.0

# Point count thresholds (registered cloud points per message)
POINTS_WARN = 50     # very few features being matched
POINTS_ERROR = 10    # effectively blind

# Consecutive low-point scans before alerting (avoid single-frame flicker)
POINTS_CONSEC_WARN = 3
POINTS_CONSEC_ERROR = 3

# Message timeout
MSG_TIMEOUT = 3.0    # seconds without odometry → error


class ScanMonitor(Node):
    def __init__(self):
        super().__init__("scan_monitor")

        # Odometry tracking
        self.prev_pos = None
        self.prev_time = None
        self.last_msg_time = None
        self.msg_count = 0
        self.msg_rate_start = time.time()
        self.last_write_time = 0.0

        # Latest metrics (updated by callbacks, read at write time)
        self.pos_cov = 0.0
        self.rot_cov = 0.0
        self.speed = 0.0
        self.jump = 0.0

        # Jump tracking — peak between timer writes and session totals
        self.peak_jump = 0.0       # largest jump since last health write
        self.jump_count = 0        # total jumps this session
        self.last_jump_time = 0.0  # time of most recent jump
        self.max_jump = 0.0        # largest jump this session

        # Point count tracking
        self.cloud_points = -1  # -1 = no data yet
        self.low_point_streak = 0
        self.no_point_streak = 0

        self.sub_odom = self.create_subscription(
            Odometry, "/Odometry", self.odom_callback, 10
        )
        self.sub_cloud = self.create_subscription(
            PointCloud2, "/cloud_registered", self.cloud_callback, 10
        )

        # Timer to detect message timeout and do periodic writes (1 Hz)
        self.create_timer(1.0, self.timer_callback)

        self.get_logger().info(
            "Scan monitor started, subscribing to /Odometry and /cloud_registered"
        )

    def odom_callback(self, msg):
        now = time.time()
        self.last_msg_time = now
        self.msg_count += 1

        # Extract position
        pos = msg.pose.pose.position
        px, py, pz = pos.x, pos.y, pos.z

        # Extract covariance diagonal
        cov = msg.pose.covariance  # 36-element array (6x6 row-major)
        self.pos_cov = max(cov[0], cov[7], cov[14])
        self.rot_cov = max(cov[21], cov[28], cov[35])

        # Compute speed and jump from position delta
        self.speed = 0.0
        self.jump = 0.0
        if self.prev_pos is not None and self.prev_time is not None:
            dt = now - self.prev_time
            if dt > 0:
                dx = px - self.prev_pos[0]
                dy = py - self.prev_pos[1]
                dz = pz - self.prev_pos[2]
                dist = math.sqrt(dx * dx + dy * dy + dz * dz)
                self.speed = dist / dt
                self.jump = dist

                # Track significant jumps
                if dist > JUMP_WARN:
                    self.jump_count += 1
                    self.last_jump_time = now
                    self.max_jump = max(self.max_jump, dist)

                # Keep peak jump between timer writes (odom is much faster
                # than the 1 Hz timer, so a jump could be missed otherwise)
                self.peak_jump = max(self.peak_jump, dist)

        self.prev_pos = (px, py, pz)
        self.prev_time = now

    def cloud_callback(self, msg):
        """Monitor /cloud_registered point count — direct measure of SLAM input quality."""
        # PointCloud2: width * height = total points
        n = msg.width * msg.height
        self.cloud_points = n

        if n <= POINTS_ERROR:
            self.no_point_streak += 1
            self.low_point_streak += 1
        elif n <= POINTS_WARN:
            self.no_point_streak = 0
            self.low_point_streak += 1
        else:
            self.no_point_streak = 0
            self.low_point_streak = 0

    def timer_callback(self):
        """Periodic evaluation and file write (1 Hz)."""
        # Check odometry timeout first
        if self.last_msg_time is None:
            self._write_health("warn", "Waiting for odometry data...")
            return

        gap = time.time() - self.last_msg_time
        if gap > MSG_TIMEOUT:
            self._write_health(
                "error",
                f"No odometry for {gap:.0f}s — SLAM may have crashed",
            )
            return

        # Evaluate all signals
        status, message = self._evaluate()
        self._write_health(status, message)

        # Reset peak jump for next evaluation interval
        self.peak_jump = 0.0

    def _evaluate(self):
        """Evaluate health status from all available metrics."""
        time_since_jump = (
            time.time() - self.last_jump_time
            if self.last_jump_time > 0 else float("inf")
        )
        # Use peak jump since last health write (catches jumps between 1Hz ticks)
        jump_val = self.peak_jump

        # --- ERROR conditions (red) ---

        # Point starvation: sensor blocked/covered
        if self.no_point_streak >= POINTS_CONSEC_ERROR and self.cloud_points >= 0:
            return "error", (
                f"NO EFFECTIVE POINTS — sensor blocked? "
                f"({self.cloud_points} pts)"
            )

        # Position jump: current interval
        if jump_val > JUMP_ERROR:
            return "error", (
                f"POSITION JUMP — {jump_val:.2f}m shift "
                f"({self.jump_count} jumps total)"
            )

        # Recent large jump still in cooldown
        if time_since_jump < JUMP_COOLDOWN and self.max_jump > JUMP_ERROR:
            return "error", (
                f"POSITION JUMP — {self.max_jump:.2f}m max "
                f"({self.jump_count} jumps total)"
            )

        # Covariance blowup
        if self.pos_cov > POS_COV_ERROR:
            return "error", f"DRIFT DETECTED — position covariance {self.pos_cov:.3f}"
        if self.rot_cov > ROT_COV_ERROR:
            return "error", f"DRIFT DETECTED — rotation covariance {self.rot_cov:.3f}"

        # Impossible speed
        if self.speed > SPEED_ERROR:
            return "error", f"DRIFT DETECTED — impossible speed {self.speed:.1f} m/s"

        # --- WARNING conditions (yellow) ---

        # Low point count: degraded input
        if self.low_point_streak >= POINTS_CONSEC_WARN and self.cloud_points >= 0:
            return "warn", (
                f"Low point count — {self.cloud_points} pts, "
                f"scan quality degraded"
            )

        # Position jump: current interval
        if jump_val > JUMP_WARN:
            return "warn", (
                f"Position jump — {jump_val:.2f}m "
                f"({self.jump_count} jumps total)"
            )

        # Recent jumps still in cooldown
        if time_since_jump < JUMP_COOLDOWN and self.jump_count > 0:
            return "warn", (
                f"Position unstable — {self.jump_count} jumps, "
                f"{self.max_jump:.2f}m max"
            )

        # Rising covariance
        if self.pos_cov > POS_COV_WARN:
            return "warn", f"Covariance rising — position {self.pos_cov:.3f}"
        if self.rot_cov > ROT_COV_WARN:
            return "warn", f"Covariance rising — rotation {self.rot_cov:.3f}"

        # High speed
        if self.speed > SPEED_WARN:
            return "warn", f"High velocity — {self.speed:.1f} m/s"

        return "ok", "Scan healthy"

    def _write_health(self, status, message):
        """Atomically write health JSON."""
        elapsed = time.time() - self.msg_rate_start
        msg_rate = self.msg_count / elapsed if elapsed > 0 else 0.0

        data = {
            "status": status,
            "message": message,
            "pos_cov": round(self.pos_cov, 6),
            "rot_cov": round(self.rot_cov, 6),
            "speed": round(self.speed, 2),
            "points": self.cloud_points,
            "msg_rate": round(msg_rate, 1),
            "jump_count": self.jump_count,
            "max_jump": round(self.max_jump, 3),
            "timestamp": time.time(),
        }
        try:
            fd, tmp_path = tempfile.mkstemp(dir="/tmp", suffix=".json")
            with os.fdopen(fd, "w") as f:
                json.dump(data, f)
            os.replace(tmp_path, HEALTH_FILE)
        except Exception as e:
            self.get_logger().warn(f"Failed to write health file: {e}")


def main():
    rclpy.init()
    node = ScanMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        try:
            os.remove(HEALTH_FILE)
        except OSError:
            pass


if __name__ == "__main__":
    main()

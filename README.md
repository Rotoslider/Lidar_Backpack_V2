# Lidar Backpack Scanner — Application Guide

Flask web application for controlling dual-lidar SLAM scans via a phone browser.
Manages the full scan lifecycle: lidar drivers, FAST-LIO2 SLAM, bag recording,
motor control (Ouster), and real-time drift detection.

---

## Architecture

```
Phone Browser (http://<ip>:5000)
        |
        | HTTP JSON API (polling every 2s)
        v
  backpack_scanner.py  (Flask, main process)
        |
        |-- subprocess: Lidar driver (Ouster or Livox ROS 2 node)
        |-- subprocess: FAST-LIO2 (SLAM)
        |-- subprocess: RViz2 (separate process group — killed cleanly on stop)
        |-- subprocess: ros2 bag record
        |-- subprocess: scan_monitor.py (health monitor ROS 2 node)
        |
        +-- I2C: Miranda motor controller (Ouster only)
        +-- TCP:  Ouster standby control (port 7501)
```

### Key Design Decisions

- **Flask, not ROS 2 in the main process.** The web app doesn't need `rclpy` — it
  launches ROS nodes as subprocesses and communicates via the filesystem and HTTP.
  This keeps the phone UI responsive and independent of ROS lifecycle.

- **Separate health monitor script.** `scan_monitor.py` uses `rclpy` to subscribe to
  ROS topics. It writes health status to `/tmp/scan_health.json`, which Flask reads
  and serves to the phone. This avoids mixing ROS 2 context into the Flask process.

- **Process group management.** All subprocesses are launched with `preexec_fn=os.setsid`
  so they get their own process groups. RViz2 is launched in its own process group
  (separate from FAST-LIO) so it can be killed cleanly without crashing. On stop,
  the `/map_save` service is called while FAST-LIO is still running to save the PCD,
  then SIGINT is sent to each group, with a SIGKILL fallback after 30 seconds.

---

## Files

| File | Purpose |
|------|---------|
| `backpack_scanner.py` | Main Flask app — scan lifecycle, motor control, API |
| `scan_monitor.py` | ROS 2 node — subscribes to `/Odometry` and `/cloud_registered`, writes health JSON |
| `templates/index.html` | Phone-optimized web UI |
| `backpack-scanner.service` | systemd unit file for autostart on boot |
| `ouster_standby.json` | Ouster config for STANDBY mode (legacy, not currently used) |
| `ouster_normal.json` | Ouster config for NORMAL mode (legacy, not currently used) |

---

## Usage

### Autostart (Headless)

The app starts automatically on boot via a systemd service. Just power on the
backpack computer, connect your phone to the **BackpackScanner** WiFi, and
open `http://10.42.0.1:5000`.

```bash
# Install/update the service (one-time, or after changing the .service file)
sudo cp ~/projects/Lidar_Backpack_V2/backpack-scanner.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable backpack-scanner.service

# Useful commands
sudo systemctl status backpack-scanner    # check if running
sudo journalctl -u backpack-scanner -f    # tail logs live
sudo systemctl stop backpack-scanner      # stop manually
sudo systemctl restart backpack-scanner   # restart after code changes
```

### Manual Start (for development/debugging)

```bash
cd ~/projects/Lidar_Backpack_V2
python3 backpack_scanner.py
```

Open `http://<computer-ip>:5000` on your phone.

### Scan Workflow

1. **Select lidar** — tap Ouster OS0-32 or Livox MID-360
2. **Start Scan** — the app launches processes in sequence:
   - Lidar driver (40s warmup for Ouster, 5s for Livox)
   - FAST-LIO2 SLAM with RViz (5s warmup)
   - Bag recording (raw packets for Ouster, point cloud for Livox)
   - Scan health monitor
   - Miranda motor at 20 RPM (Ouster only, after 10s FAST-LIO stabilization)
3. **Monitor** — watch the Health indicator on your phone:
   - Green = healthy
   - Yellow = degraded (slow down, check environment)
   - Red = drift detected (stop and restart scan)
4. **Stop Scan** — gracefully stops everything:
   - Ouster set to STANDBY (saves power/heat)
   - Motor stopped
   - RViz2 killed (separate process, no data to save)
   - PCD saved via `/map_save` ROS service (while FAST-LIO is still running)
   - SIGINT to remaining ROS processes
   - WiFi hotspot deactivated, previous WiFi restored
   - Health file cleaned up

### Force Stop

If the normal stop hangs, tap **Force Stop (Kill All)**. This sends SIGKILL to all
processes immediately. FAST-LIO will NOT save a PCD file in this case.

### Exit

Tap **Exit** to stop everything and shut down the Flask server.

---

## API Endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/` | GET | Serve the web UI |
| `/api/status` | GET | Current state, elapsed time, health data, messages |
| `/api/start` | POST | Start scan. Body: `{"lidar": "ouster"}` or `{"lidar": "livox"}` |
| `/api/stop` | POST | Graceful stop (SIGINT, saves PCD) |
| `/api/force_stop` | POST | Emergency kill (SIGKILL, no PCD save) |
| `/api/exit` | POST | Stop everything and shut down server |

### Status Response

```json
{
  "state": "scanning",
  "lidar": "ouster",
  "elapsed": 145,
  "output_dir": "/home/lapanda/pointclouds/2026-03-01_14-30-00_ouster",
  "message": "Scanning — check rviz for live point cloud",
  "health": {
    "status": "ok",
    "message": "Scan healthy",
    "pos_cov": 0.0012,
    "rot_cov": 0.0001,
    "speed": 0.45,
    "points": 3842,
    "msg_rate": 10.2,
    "timestamp": 1709312400.0
  }
}
```

---

## Scan Health Monitor (`scan_monitor.py`)

### What It Monitors

The monitor subscribes to two FAST-LIO topics:

**`/Odometry` (nav_msgs/Odometry)** — EKF state output at ~10-20 Hz
- **Position covariance** — diagonal elements [0,7,14] of the 6x6 pose covariance matrix.
  Represents XYZ position uncertainty from the EKF. Grows when SLAM degrades.
- **Rotation covariance** — diagonal elements [21,28,35]. Rotation uncertainty.
- **Velocity** — computed from position deltas between messages. Detects impossible speeds
  that indicate the EKF state has jumped.
- **Position jumps** — raw displacement per odometry step. Catches sudden re-localization
  failures (e.g., 1-meter shift when sensor recovers from occlusion).

**`/cloud_registered` (sensor_msgs/PointCloud2)** — registered point cloud
- **Point count** — `width * height` of each cloud message. This is the most direct
  measure of SLAM input quality. When the sensor is blocked, covered, or in a
  featureless environment, the point count drops to near-zero.

### Health Status Levels

| Status | Color | Meaning | Action |
|--------|-------|---------|--------|
| `ok` | Green | All metrics normal | Continue scanning |
| `warn` | Yellow | Degraded input or elevated uncertainty | Slow down, check environment |
| `error` | Red (pulsing) | Drift detected or sensor failure | Stop scan, restart |
| `unknown` | Grey | No health data (monitor not running) | Check if scan started properly |

### Detection Thresholds

| Metric | Warning | Error | Notes |
|--------|---------|-------|-------|
| Position covariance | > 0.5 | > 2.0 | EKF uncertainty in meters^2 |
| Rotation covariance | > 0.1 | > 0.5 | EKF uncertainty in radians^2 |
| Speed | > 3.0 m/s | > 8.0 m/s | Walking ~1.5 m/s, running ~3 m/s |
| Position jump | > 0.3m | > 0.8m | Single-step displacement |
| Point count | < 50 | < 10 | Per-frame registered points |
| Message timeout | — | > 3s | No odometry = SLAM crashed |

Point count warnings require 3 consecutive low-count frames to avoid single-frame flicker.

### How It Works

1. Callbacks update metric values on every message
2. A 1 Hz timer evaluates all metrics against thresholds
3. Status is written atomically to `/tmp/scan_health.json` (write to temp, then `os.replace`)
4. Flask reads the JSON file when the phone polls `/api/status`
5. File staleness > 5 seconds triggers "unknown" status

### Important: System Python Requirement

The monitor MUST run with `/usr/bin/python3` (system Python 3.10), NOT pyenv's Python 3.11.
ROS 2 Humble's `rclpy` C extensions are built for Python 3.10. The launch command in
`backpack_scanner.py` uses the full path `/usr/bin/python3` for this reason.

---

## Motor Controller

The Miranda motor rotates the Ouster lidar at 20 RPM for 360-degree scanning.

- Connected via CH341 USB-to-I2C adapter (auto-detected at startup)
- I2C address: `0x28`
- RPM-to-counts conversion: `counts = RPM * 6 * 91.019`
- Motor starts 10 seconds after FAST-LIO to allow initial map stabilization
- Motor stops immediately on scan stop

If the CH341 adapter is not found, the app starts without motor control (useful for
Livox-only operation or bench testing).

---

## Ouster Standby Control

The Ouster is put into STANDBY mode when scanning stops to reduce power consumption
and heat. This is done via direct TCP to port 7501 (more reliable than the ROS service
during shutdown):

```
set_config_param operating_mode STANDBY
reinitialize
```

The sensor must be woken up again when the next scan starts — the Ouster driver handles
this automatically during its initialization.

---

## Output Directory Structure

Each scan creates a timestamped directory under `~/pointclouds/`:

```
~/pointclouds/2026-03-01_14-30-00_ouster/
  scan.pcd              # FAST-LIO accumulated point cloud (saved on graceful stop)
  bag/                  # ROS 2 bag with raw sensor data
    metadata.yaml
    *.db3
  os-122134000147-metadata.json   # Ouster sensor metadata (if available)
```

The bag contains raw packets (Ouster) or raw point cloud + IMU (Livox), allowing
offline reprocessing with different FAST-LIO parameters.

---

## Troubleshooting

| Problem | Cause | Fix |
|---------|-------|-----|
| Health shows "No health data" | Monitor failed to start (Python version mismatch) | Verify `/usr/bin/python3 -c "import rclpy"` works with ROS 2 sourced |
| Health shows "Waiting for odometry data..." | FAST-LIO hasn't started publishing yet | Wait for FAST-LIO initialization (normal during startup) |
| Health stays green when sensor is blocked | Point count detection not working | Check that `/cloud_registered` topic is publishing (`ros2 topic hz /cloud_registered`) |
| Motor doesn't spin | CH341 adapter not detected or I2C error | Check `i2cdetect -l` for CH341 bus, verify wiring |
| Ouster won't wake from standby | Sensor hostname wrong or network issue | Ping `os-122134000147.local`, check ethernet connection |
| PCD file not saved | Used Force Stop instead of Stop | Force Stop sends SIGKILL — use normal Stop for PCD save |
| App doesn't start on boot | systemd service not installed/enabled | Run the install commands in the Autostart section above |
| WiFi hotspot stays on after exit | Cleanup failed | `nmcli connection down Hotspot` then `nmcli device connect wlan0` |
| Scan directory empty | FAST-LIO crashed during scan | Check terminal for FAST-LIO errors, review health history |

---

## Configuration

### Changing Health Thresholds

Edit the constants at the top of `scan_monitor.py`:

```python
POS_COV_WARN = 0.5    # position covariance warning threshold
POS_COV_ERROR = 2.0   # position covariance error threshold
POINTS_WARN = 50      # minimum point count before warning
POINTS_ERROR = 10     # minimum point count before error
SPEED_WARN = 3.0      # m/s — warning velocity
SPEED_ERROR = 8.0     # m/s — error velocity
```

These are conservative initial values. After field testing, you may want to:
- Lower `POINTS_WARN` if your environment naturally has sparse features
- Adjust `SPEED_WARN` based on typical walking speed with the backpack
- Tune covariance thresholds based on observed values during good scans

### Changing Motor Speed

In `backpack_scanner.py`, the motor speed is set in `_start_scan_sequence()`:

```python
self.motor.start_motor(20)  # RPM
```

### Changing Polling Interval

In `templates/index.html`, the status polling interval:

```javascript
setInterval(updateStatus, 2000);  // milliseconds
```

---

## Dependencies

- Python 3.11 (Flask app via pyenv)
- Python 3.10 (scan_monitor via system Python, for rclpy compatibility)
- Flask (`pip install flask`)
- smbus2 (`pip install smbus2`, for motor I2C control)
- ROS 2 Humble (system install)
- Ouster ROS 2 driver
- Livox ROS 2 driver
- FAST-LIO2 ROS 2

# Lidar Backpack Scanner — Application Guide

Flask web application for controlling dual-lidar SLAM scans via a phone browser.
Manages the full scan lifecycle: lidar drivers, FAST-LIO2 SLAM, bag recording,
motor control (Ouster), and real-time drift detection.

---

## Architecture

```
Phone Browser (http://10.42.0.1:5000)
        |
        | HTTP JSON API (polling every 2s)
        v
  backpack_scanner.py  (Flask, main process)
        |
        |-- subprocess: Lidar driver (Ouster or Livox ROS 2 node)
        |-- subprocess: FAST-LIO2 (SLAM + RViz when display available)
        |-- subprocess: ros2 bag record
        |-- subprocess: scan_monitor.py (health monitor ROS 2 node)
        |
        +-- I2C: Miranda motor controller (Ouster only)
        +-- TCP:  Ouster standby control (port 7501)
        +-- UDP:  Livox standby control (via livox_workmode + Livox SDK2)
        +-- Hotspot watchdog (auto-restores WiFi AP if it drops)
```

### Key Design Decisions

- **Flask, not ROS 2 in the main process.** The web app doesn't need `rclpy` — it
  launches ROS nodes as subprocesses and communicates via the filesystem and HTTP.
  This keeps the phone UI responsive and independent of ROS lifecycle.

- **Separate health monitor script.** `scan_monitor.py` uses `rclpy` to subscribe to
  ROS topics. It writes health status to `/tmp/scan_health.json`, which Flask reads
  and serves to the phone. This avoids mixing ROS 2 context into the Flask process.

- **Process group management.** All subprocesses are launched with `preexec_fn=os.setsid`
  so they get their own process groups. On stop, the `/map_save` service is called while
  FAST-LIO is still running to save the PCD, then SIGINT is sent to each group, with a
  SIGKILL escalation after 5 seconds. A `_cleanup_lock` prevents race conditions when
  Stop and Force Stop are pressed in quick succession.

- **Pre-flight checks.** Before launching any ROS process, the app pings the sensor and
  checks disk space. Fails fast with a clear message instead of waiting 40 seconds for
  a driver timeout.

---

## Files

| File | Purpose |
|------|---------|
| `backpack_scanner.py` | Main Flask app — scan lifecycle, motor control, API |
| `scan_monitor.py` | ROS 2 node — subscribes to `/Odometry` and `/cloud_registered`, writes health JSON |
| `templates/index.html` | Phone-optimized web UI |
| `backpack-scanner.service` | systemd unit file for autostart on boot |
| `toggle_server.sh` | Desktop/GPIO script to start or stop the service |
| `Backpack Scanner.desktop` | GNOME desktop shortcut (installed to `~/Desktop/`) |
| `50-backpack-network.pkla` | PolicyKit rule for NetworkManager access from systemd |
| `backpack-scanner-sudoers` | Sudoers rule for passwordless service start/stop and computer shutdown |
| `livox_workmode.cpp` | C++ tool source — sets Livox MID-360 work mode via SDK2 |
| `livox_workmode` | Compiled binary (build: `g++ -o livox_workmode livox_workmode.cpp -llivox_lidar_sdk_shared -lpthread`) |
| `ouster_standby.json` | Ouster config for STANDBY mode (legacy, not currently used) |
| `ouster_normal.json` | Ouster config for NORMAL mode (legacy, not currently used) |

---

## Installation

### First-Time Setup

```bash
# 1. Install the systemd service
sudo cp ~/projects/Lidar_Backpack_V2/backpack-scanner.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable backpack-scanner.service

# 2. Install PolicyKit rule (allows service to manage WiFi hotspot)
sudo cp ~/projects/Lidar_Backpack_V2/50-backpack-network.pkla \
     /etc/polkit-1/localauthority/50-local.d/

# 3. Install sudoers rule (passwordless service start/stop from desktop button)
sudo cp ~/projects/Lidar_Backpack_V2/backpack-scanner-sudoers \
     /etc/sudoers.d/backpack-scanner
sudo chmod 440 /etc/sudoers.d/backpack-scanner

# 4. Desktop shortcut is already at ~/Desktop/Backpack Scanner.desktop
# Right-click → Allow Launching (if GNOME prompts)
```

### After Code Changes

```bash
# Restart the running service to pick up changes
sudo systemctl restart backpack-scanner
```

### Network Configuration

Both lidars connect through a single ethernet interface (`enp2s0`) with two addresses
in one NetworkManager profile ("Lidar Backpack"):

| Address | Subnet | Purpose |
|---------|--------|---------|
| `192.168.2.5` | `/24` | Livox MID-360 (static IP `192.168.2.183`) |
| `169.254.1.1` | `/16` | Ouster OS0-32 (link-local, hostname `os-122134000147.local`) |

This is a single persistent profile — both addresses come up together on every boot.
The Ouster web interface is accessible at `http://os-122134000147.local/`.

---

## Usage

### Starting the Scanner

**Option A: Automatic (headless field use)**
Power on the backpack. The service starts on boot, the WiFi hotspot comes up.
Connect your phone to **BackpackScanner** WiFi and open `http://10.42.0.1:5000`.

**Option B: Desktop button (with monitor)**
Double-click **Backpack Scanner** on the desktop. A terminal shows the service
starting and confirms the hotspot is active. Double-click again to stop.

**Option C: Command line**
```bash
sudo systemctl start backpack-scanner    # start
sudo systemctl stop backpack-scanner     # stop
sudo systemctl status backpack-scanner   # check status
journalctl -u backpack-scanner -f        # tail logs live
```

### Scan Workflow

1. **Select lidar** — tap Ouster OS0-32 or Livox MID-360
2. **Start Scan** — the app runs pre-flight checks then launches processes in sequence:
   - Pre-flight: disk space check, sensor ping
   - Livox: wake from standby → 12s motor spin-up wait (Livox only)
   - Lidar driver (40s warmup for Ouster, 5s for Livox)
   - FAST-LIO2 SLAM (with RViz if display available, headless otherwise)
   - Bag recording (raw packets for Ouster, point cloud + IMU for Livox)
   - Scan health monitor
   - Miranda motor at 20 RPM (Ouster only, after 10s FAST-LIO stabilization)
3. **Monitor** — watch the Health indicator on your phone:
   - Green = healthy
   - Yellow = degraded (slow down, check environment)
   - Red = drift detected (stop and restart scan)
4. **Stop Scan** — gracefully saves data and stops everything. Activity shows
   each step as it happens:
   - Ouster: set to STANDBY, motor stopped
   - Saving PCD... → Saved PCD (renamed with timestamp)
   - Saving bag... → Saved bag
   - Livox: set to standby (motor and laser off)
   - Saved metadata
   - Scan complete

### Force Stop

If the normal stop hangs, tap **Force Stop (Kill All)**. This sends SIGKILL to all
processes immediately. FAST-LIO will NOT save a PCD file in this case.

### Exit

Tap **Exit** to stop everything, shut down the Flask server, and restore the previous
WiFi connection. Use this when you want to connect the backpack to a local network
(e.g. Starlink for file transfer).

### Shutdown

Tap **Shutdown** to power off the backpack computer. Requires two confirmations.
If a scan is running, it is stopped gracefully first (PCD and bag saved, motor stopped)
before the computer powers off. The phone displays "Safe to disconnect power in 30
seconds." You will need physical access to the power button to turn the computer back on.

### WiFi Behavior

| State | WiFi | Phone connects to |
|-------|------|-------------------|
| Service running | Hotspot (guarded by watchdog) | `BackpackScanner` WiFi → `http://10.42.0.1:5000` |
| Service stopped / Exit pressed | Previous WiFi restored | Same network as backpack, use backpack's LAN IP |
| Reboot | Hotspot auto-starts | `BackpackScanner` WiFi → `http://10.42.0.1:5000` |

The hotspot watchdog checks every 15 seconds and re-enables the hotspot if it drops
unexpectedly (e.g. NetworkManager glitch). It stops during clean shutdown so it
doesn't fight the WiFi restore.

---

## API Endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/` | GET | Serve the web UI |
| `/api/status` | GET | Current state, elapsed time, health data, messages |
| `/api/start` | POST | Start scan. Body: `{"lidar": "ouster"}` or `{"lidar": "livox"}` |
| `/api/stop` | POST | Graceful stop (saves PCD and bag) |
| `/api/force_stop` | POST | Emergency kill (SIGKILL, no PCD save) |
| `/api/exit` | POST | Stop everything and shut down server |
| `/api/shutdown` | POST | Graceful stop, cleanup, and power off computer |

---

## Scan Health Monitor (`scan_monitor.py`)

### What It Monitors

The monitor subscribes to two FAST-LIO topics:

**`/Odometry` (nav_msgs/Odometry)** — EKF state output at ~10-20 Hz
- **Position covariance** — diagonal elements [0,7,14] of the 6x6 pose covariance matrix
- **Rotation covariance** — diagonal elements [21,28,35]
- **Velocity** — computed from position deltas between messages
- **Position jumps** — raw displacement per odometry step

**`/cloud_registered` (sensor_msgs/PointCloud2)** — registered point cloud
- **Point count** — `width * height` of each cloud message

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
| Point count | < 50 | < 10 | Per-frame registered points (3 consecutive) |
| Message timeout | — | > 3s | No odometry = SLAM crashed |

### Important: System Python Requirement

The monitor MUST run with `/usr/bin/python3` (system Python 3.10), NOT pyenv's Python 3.11.
ROS 2 Humble's `rclpy` C extensions are built for Python 3.10. The launch command in
`backpack_scanner.py` uses the full path `/usr/bin/python3` for this reason.

---

## Output Directory Structure

Each scan creates a timestamped directory under `~/pointclouds/`:

```
~/pointclouds/2026-03-02_14-30-00_ouster/
  2026-03-02_14-30-00_ouster.pcd   # FAST-LIO point cloud (timestamped name)
  bag/                              # ROS 2 bag with raw sensor data
    metadata.yaml
    *.db3
  os-122134000147-metadata.json     # Ouster sensor metadata (if available)
```

The bag contains raw packets (Ouster) or raw point cloud + IMU (Livox), allowing
offline reprocessing with different FAST-LIO parameters.

---

## Motor Controller

The Miranda motor rotates the Ouster lidar at 20 RPM for 360-degree scanning.

- Connected via CH341 USB-to-I2C adapter (auto-detected at startup)
- I2C address: `0x28`, device group: `ch341` (user `lapanda` has access)
- RPM-to-counts conversion: `counts = RPM * 6 * 91.019`
- Motor starts 10 seconds after FAST-LIO to allow initial map stabilization
- Motor stops immediately on scan stop or abort

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

## Livox MID-360 Standby Control

The Livox MID-360 defaults to Sampling mode on power-on (motor running, laser firing).
When not scanning, the app puts it into standby mode to stop the motor and laser,
reducing power draw and motor wear.

This uses a small C++ tool (`livox_workmode`) that links against the Livox SDK2 shared
library (`liblivox_lidar_sdk_shared.so`) to send work-mode commands over UDP:

```bash
./livox_workmode <config.json> standby   # motor off, laser off
./livox_workmode <config.json> normal    # motor on, laser on (sampling)
```

**MID-360 work modes:**

| SDK Value | Mode | Motor | Laser | Power |
|-----------|------|-------|-------|-------|
| `0x01` (Normal) | Sampling | Running | Firing | Full (~6.5W) |
| `0x02` (WakeUp) | Standby | Off | Off | Low |
| `0x03` (Sleep) | — | — | — | Not supported by MID-360 firmware |

**Important:** The tool and the Livox ROS 2 driver share the same UDP ports
(56100–56501). They cannot run simultaneously. The scanner sequences them:
tool finishes and releases ports before the driver launches, and the driver is
killed before the tool runs again.

**Timing on scan start:** The tool takes ~5s (SDK init + lidar discovery + command),
then a 12s countdown waits for the motor to spin up before launching the driver.

**Rebuild after SDK update:**
```bash
g++ -o livox_workmode livox_workmode.cpp -llivox_lidar_sdk_shared -lpthread
```

---

## Troubleshooting

| Problem | Cause | Fix |
|---------|-------|-----|
| "Cannot reach sensor" on scan start | Sensor not powered or cable disconnected | Check power and ethernet. Ping `os-122134000147.local` (Ouster) or `192.168.2.183` (Livox) |
| "Low disk space" on scan start | Disk full | Free space on the pointclouds partition (need 1 GB minimum) |
| "Driver process died during startup" | ROS driver crashed | Check `journalctl -u backpack-scanner` for driver errors |
| Health shows "No health data" | Monitor failed to start (Python version mismatch) | Verify `/usr/bin/python3 -c "import rclpy"` works with ROS 2 sourced |
| Health shows "Waiting for odometry data..." | FAST-LIO hasn't started publishing yet | Wait for FAST-LIO initialization (normal during startup) |
| Motor doesn't spin | CH341 adapter not detected or I2C error | Check `i2cdetect -l` for CH341 bus, verify wiring |
| PCD file not saved | Used Force Stop instead of Stop | Force Stop sends SIGKILL — use normal Stop for PCD save |
| Hotspot doesn't start | PolicyKit rule not installed | Install `50-backpack-network.pkla` (see Installation above) |
| Can't delete old scan folders | Files owned by root (from old service config) | `sudo chown -R lapanda:lapanda ~/pointclouds/<folder>` |
| Shutdown button doesn't power off | Sudoers file not installed or outdated | Re-install: `sudo install -m 440 -o root -g root backpack-scanner-sudoers /etc/sudoers.d/backpack-scanner` |
| WiFi hotspot stays on after exit | Cleanup failed | `nmcli connection down Hotspot` then `nmcli device connect wlo1` |

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

### Changing Motor Speed

In `backpack_scanner.py`, the motor speed is set in `_start_scan_sequence()`:

```python
self.motor.start_motor(20)  # RPM
```

### Changing Minimum Disk Space

In `backpack_scanner.py`:

```python
MIN_DISK_MB = 1024  # Minimum free disk space (MB) to start a scan
```

---

## Dependencies

- Python 3.11 (Flask app via pyenv)
- Python 3.10/3.12 (scan_monitor via system Python, for rclpy compatibility)
- Flask (`pip install flask`)
- smbus2 (`pip install smbus2`, for motor I2C control)
- ROS 2 Humble or Jazzy (auto-detected at runtime)
- Ouster ROS 2 driver
- Livox ROS 2 driver
- Livox SDK2 (`liblivox_lidar_sdk_shared.so` — for livox_workmode tool)
- FAST-LIO2 ROS 2

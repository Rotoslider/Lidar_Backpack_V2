# FAST-LIO Reprocessor

PyQt5 desktop GUI for replaying scan bags through FAST-LIO with different parameters.
Enables rapid A/B comparison of SLAM settings without going back to the field.

## Quick Start

```bash
/usr/bin/python3 ~/projects/Lidar_Backpack_V2/fastlio_reprocessor.py
```

## What It Does

Scan bags recorded by the backpack scanner contain raw sensor data. This tool
replays those bags through FAST-LIO with tunable parameters, so you can
experiment with voxel sizes, IMU noise, range filters, and other settings to
find the best quality for each scan.

```
┌─────────────────────────────────────────────────────────┐
│  Select a scan bag  →  Tune parameters  →  PROCESS     │
│                                                         │
│  Watch the point cloud build in RViz                    │
│  Monitor health metrics (covariance, speed, points)     │
│                                                         │
│  SAVE PCD  →  Compare results across runs               │
└─────────────────────────────────────────────────────────┘
```

## Replay Pipeline

**Ouster bags** contain raw packets, not decoded point clouds. The replay
pipeline decodes them through the Ouster driver in replay mode:

```
replay.composite.launch.xml (Ouster driver replay mode)
  ├─ os_replay node  →  reads metadata JSON, publishes /ouster/metadata
  ├─ os_cloud node   →  decodes packets → /ouster/points + /ouster/imu
  └─ ros2 bag play   →  replays raw packets with --clock
       ↓
FAST-LIO (use_sim_time:=true)
  ├─ subscribes to /ouster/points + /ouster/imu
  ├─ runs SLAM, accumulates point cloud
  └─ /map_save service → writes scan.pcd
```

**Livox bags** already contain decoded topics and replay directly with
`ros2 bag play --clock`.

## GUI Overview

### Scan Selection

Dropdown auto-populates from `~/pointclouds/`. Shows bag duration, message
count, sensor type, and whether Ouster metadata JSON is present. Use "Browse"
for scan directories in other locations.

### Parameter Editor

27 parameters across 7 tabs, all with tooltips:

| Tab | Key Parameters |
|-----|---------------|
| **Resolution** | `filter_size_surf`, `filter_size_map`, `point_filter_num`, `max_iteration` |
| **Range** | `blind` (min range), `det_range` (max range), `fov_degree` |
| **IMU Noise** | `acc_cov`, `gyr_cov`, `b_acc_cov`, `b_gyr_cov` |
| **Extrinsics** | `gravity_align_en`, `extrinsic_est_en`, translation vector, rotation matrix |
| **Preprocess** | `lidar_type`, `scan_line`, `timestamp_unit` |
| **Publish** | Path, scan, dense publish toggles |
| **PCD** | Save enable, interval, output path |

### Presets

- **Built-in** (read-only): Loaded from `~/ros2_ws/src/FAST_LIO_ROS2/config/`
- **Custom**: Saved to `reprocessor_presets/` via "Save As"

### Playback Controls

- **Rate**: Playback speed multiplier (0.1x–10x)
- **Stop at**: Auto-stop after N bag-seconds (saves PCD first). 0 = full bag.
- **Open RViz**: Launch RViz as a separate window

### Action Buttons

- **PROCESS**: Start the replay pipeline
- **SAVE PCD**: Save current point cloud via `/map_save` (works during or after replay)
- **STOP**: Saves PCD automatically, then kills all processes
- **LOG**: Show timestamped history of all status changes and events

### Health Monitor

Reads from `scan_monitor.py` at 2 Hz. Displays:

- Status (ok/warn/error with color coding)
- Position and rotation covariance
- Speed, registered point count, message rate
- Elapsed bag-time / total duration

Terminal states:
- **Bag completed**: Replay finished, FAST-LIO alive, ready to save PCD
- **FAST-LIO crashed**: Process died (e.g. VoxelGrid overflow), check log

## Output

```
~/pointclouds/reprocess/
└── 2026-03-02_14-30-35_ouster_reprocess_20260302_160000/
    ├── fastlio_config.yaml   # Exact config used (for reproducibility)
    └── scan.pcd              # FAST-LIO output
```

## Troubleshooting

### VoxelGrid overflow ("Leaf size is too small")

PCL's VoxelGrid filter overflows int32 indices when the accumulated map is
too large for the voxel size. Solutions:

- Increase `filter_size_surf` and `filter_size_map` (try 0.3 or 0.5)
- Use `stop_at` to process a shorter segment
- Increase `point_filter_num` to reduce point density

The GUI detects this crash and shows "FAST-LIO crashed" with the exit code
in the log, instead of hanging.

### No points / skipping scans

- Check that the correct preset is loaded (Ouster vs Livox)
- Verify `blind` isn't filtering all points (Ouster: 1.5m, Livox: 0.5m)
- Ensure Ouster metadata JSON exists in the scan directory

### PCD save timeout

- If FAST-LIO crashed, PCD save is skipped automatically
- If FAST-LIO is alive but unresponsive, the 30s timeout will fire and be logged

## ROS 2 Compatibility

The reprocessor auto-detects the installed ROS 2 distro (Jazzy, Humble, etc.)
and sources the correct setup files. No manual configuration needed.

### FAST-LIO MultiThreadedExecutor

FAST-LIO has been modified to use a 2-thread `MultiThreadedExecutor` with
separate callback groups. This ensures that point cloud and IMU data is
received continuously on one thread while SLAM processing runs on another.
Without this, the original single-threaded executor blocked subscription
callbacks during heavy IKF/ICP processing, causing frame drops and position
jumps — especially with dense Ouster point clouds.

This fix applies to both replay (reprocessor) and live operation (backpack).
It works on both Humble and Jazzy.

### Ouster Replay Specifics

Ouster bags are replayed through the `ouster_ros` driver with:
- `organized:=false` — prevents NaN points from reaching FAST-LIO
- `--read-ahead-queue-size 1000` — prevents the bag player from dropping messages
- QoS overrides for metadata topic (transient_local/reliable)

### Gravity Alignment Toggle

The **Extrinsics** tab includes a "Gravity Alignment" checkbox. When enabled
(default), FAST-LIO rotates the initial pose so Z points up using IMU
accelerometer data during initialization. This produces level scans
regardless of sensor mounting angle.

Disable it if you see position jumps or SLAM instability. The original
ROS 1 FAST-LIO behavior is equivalent to gravity alignment OFF.

## Dependencies

- Python 3.10+ with PyQt5 and PyYAML (via venv on Ubuntu 24.04)
- ROS 2 Humble or Jazzy (ouster_ros, fast_lio, livox_ros_driver2)
- scan_monitor.py (from this repo)

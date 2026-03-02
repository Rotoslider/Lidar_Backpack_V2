# ROS 2 Humble + SLAM Setup Guide — Ubuntu 22.04

Complete setup for the Lidar Backpack V2 with Ouster OS0-32 (Gen 5) and Livox lidar on
ROS 2 Humble (replacing the previous ROS 1 Noetic setup).

---

## Table of Contents

1. [Prerequisites](#1-prerequisites)
2. [Install pyenv and Python 3.11](#2-install-pyenv-and-python-311)
3. [Install ROS 2 Humble](#3-install-ros-2-humble)
4. [Create the Colcon Workspace](#4-create-the-colcon-workspace)
5. [Install System Dependencies](#5-install-system-dependencies)
6. [Build the Ouster ROS 2 Driver](#6-build-the-ouster-ros-2-driver)
7. [Build Livox SDK2 and ROS 2 Driver](#7-build-livox-sdk2-and-ros-2-driver)
8. [Build FAST-LIO2 (ROS 2)](#8-build-fast-lio2-ros-2)
9. [Configure FAST-LIO2 for Your Sensors](#9-configure-fast-lio2-for-your-sensors)
10. [Launch and Verify](#10-launch-and-verify)
11. [Troubleshooting](#11-troubleshooting)
12. [WiFi Hotspot for Field Use](#12-wifi-hotspot-for-field-use)
13. [Backpack Scanner Web App](#13-backpack-scanner-web-app)
14. [FAST-LIO2 Backpack Modifications](#14-fast-lio2-backpack-modifications)

---

## 1. Prerequisites

- Ubuntu 22.04 (Jammy) fresh install
- Internet connection
- Ouster OS0-32 Gen 5 updated to **firmware 2.5.3** (latest for Rev 5 hardware)
- Livox lidar (MID-360, Avia, HAP, etc.)

---

## 2. Install pyenv and Python 3.11

Install build dependencies:

```bash
sudo apt-get install -y build-essential libssl-dev zlib1g-dev libbz2-dev \
  libreadline-dev libsqlite3-dev wget curl llvm libncurses5-dev libncursesw5-dev \
  xz-utils tk-dev libffi-dev liblzma-dev python3-openssl git
```

Install pyenv:

```bash
curl https://pyenv.run | bash
```

Add to `~/.bashrc`:

```bash
export PYENV_ROOT="$HOME/.pyenv"
[[ -d $PYENV_ROOT/bin ]] && export PATH="$PYENV_ROOT/bin:$PATH"
eval "$(pyenv init - bash)"
eval "$(pyenv virtualenv-init -)"
```

Then:

```bash
source ~/.bashrc
pyenv install 3.11.9
pyenv global 3.11.9
python3 --version   # should show 3.11.9
```

---

## 3. Install ROS 2 Humble

### 3a. Set locale

```bash
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### 3b. Add ROS 2 apt repository

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu jammy main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### 3c. Install Humble desktop

```bash
sudo apt update
sudo apt install ros-humble-desktop-full -y
```

### 3d. Source ROS 2 and add to bashrc

```bash
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### 3e. Install dev tools

```bash
sudo apt install -y python3-colcon-common-extensions python3-rosdep2 python3-vcstool
sudo rosdep init
rosdep update
```

### 3f. Verify

```bash
printenv | grep ROS
```

You should see `ROS_DISTRO=humble`.

---

## 4. Create the Colcon Workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source install/setup.bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

> **Note:** `colcon build` replaces `catkin_make`. The built outputs go to `install/` not `devel/`.

### 4a. Fix pyenv / ROS 2 Python conflict

ROS 2 Humble is built against the **system Python 3.10**. If pyenv is set to a different
version globally (e.g. 3.11), ROS builds will fail with `ModuleNotFoundError: No module named 'em'`
because pyenv's Python doesn't have the ROS Python packages.

Fix this by pinning the ROS workspace to use system Python:

```bash
cd ~/ros2_ws
pyenv local system
```

This creates a `.python-version` file so pyenv automatically uses system Python 3.10
inside the workspace. Your global Python 3.11 still works everywhere else.

---

## 5. Install System Dependencies

```bash
sudo apt install -y \
  build-essential cmake \
  libeigen3-dev libpcl-dev libpcap-dev \
  libjsoncpp-dev libspdlog-dev libcurl4-openssl-dev \
  ros-humble-pcl-ros \
  ros-humble-pcl-conversions \
  ros-humble-rviz2 \
  ros-humble-navigation2 \
  ros-humble-robot-localization \
  ros-humble-robot-state-publisher \
  ros-humble-cv-bridge \
  python3-pip \
  openssh-client
```

---

## 6. Build the Ouster ROS 2 Driver

### 6a. Clone

```bash
cd ~/ros2_ws/src
git clone --recurse-submodules -b ros2 https://github.com/ouster-lidar/ouster-ros.git
```

### 6b. Install dependencies and build

```bash
cd ~/ros2_ws
rosdep install --from-paths src/ouster-ros --ignore-src -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --packages-select ouster_ros ouster_sensor_msgs
source install/setup.bash
```

> **Important:** Always build with `-DCMAKE_BUILD_TYPE=Release` for the Ouster driver.
> Debug/unoptimized builds will drop packets and perform poorly.
>
> **Note:** This builds only the Ouster packages for early testing. The full workspace
> build (including FAST-LIO2 and Livox) happens in step 8b using `build.sh`.

### 6c. Increase UDP receive buffer (prevents packet drops)

```bash
sudo sysctl -w net.core.rmem_max=134217728
sudo sysctl -w net.core.rmem_default=134217728
```

Make it permanent:

```bash
echo -e "net.core.rmem_max=134217728\nnet.core.rmem_default=134217728" | sudo tee /etc/sysctl.d/90-ouster.conf
sudo sysctl --system
```

> Without this you will see: `Failed to set desired SO_RCVBUF size` warnings and may
> experience packet drops.

### 6d. Configure the sensor hostname

The Ouster driver reads parameters from a YAML file, **not** from command-line arguments.
Edit the default config to set your sensor's hostname or IP:

```bash
# Config file location:
~/ros2_ws/src/ouster-ros/ouster-ros/config/driver_params.yaml
```

Change the `sensor_hostname` line from `''` to your sensor's mDNS hostname or IP:

```yaml
    sensor_hostname: os-122134000147.local
```

> **Tip:** The mDNS hostname is printed on the sensor and follows the format
> `os-<serial>.local`. You can also access the sensor dashboard at
> `http://os-<serial>.local/` in a browser.

Since we built with `--symlink-install`, config changes take effect immediately
without rebuilding.

### 6e. Launch and verify

Connect your OS0-32 via ethernet. The sensor uses DHCP by default, or you can
configure a static IP on your network interface to match the sensor's subnet.

```bash
ros2 launch ouster_ros driver.launch.py
```

You should see output confirming the connection:

```
product: OS-0-32-U0, sn: 122134000147, firmware ver: 2.5.3
lidar mode: 1024x10, lidar udp profile: LEGACY, imu udp profile: LEGACY
```

In another terminal, verify topics are publishing:

```bash
source ~/ros2_ws/install/setup.bash
ros2 topic list | grep ouster
```

Expected topics:

```
/ouster/imu
/ouster/imu_packets
/ouster/lidar_packets
/ouster/metadata
/ouster/nearir_image
/ouster/points
/ouster/range_image
/ouster/reflec_image
/ouster/scan
/ouster/signal_image
/ouster/telemetry
```

Check the point cloud rate (should be ~10 Hz for 1024x10 mode):

```bash
ros2 topic hz /ouster/points
```

The driver also launches rviz2 automatically — you should see the point cloud displayed.

---

## 7. Build Livox SDK2 and ROS 2 Driver

### 7a. Build and install Livox-SDK2

```bash
cd ~
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2
mkdir build && cd build
cmake .. && make -j$(nproc)
sudo make install
sudo ldconfig
```

This installs the library to `/usr/local/lib` and headers to `/usr/local/include`.
The `ldconfig` step is required so the dynamic linker can find `liblivox_lidar_sdk_shared.so`.
Without it, the ROS driver will crash with `cannot open shared object file`.

### 7b. Clone the Livox ROS 2 driver

```bash
cd ~/ros2_ws/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
```

### 7c. Build

```bash
cd ~/ros2_ws/src/livox_ros_driver2
source /opt/ros/humble/setup.bash
./build.sh humble
```

> **Note:** The Livox driver uses its own `build.sh` script instead of a direct `colcon build`
> from the workspace root. This is intentional — it sets up the correct workspace structure.

### 7d. Source and verify

```bash
source ~/ros2_ws/install/setup.bash
ros2 pkg list | grep livox
```

You should see `livox_ros_driver2`.

### 7e. Configure networking for the MID-360

The MID-360 and your WiFi network may both default to the `192.168.1.x` subnet,
which causes routing conflicts. To avoid this, the MID-360 has been moved to the
`192.168.2.x` subnet.

**Sensor settings (configured via Livox Viewer 2):**

| Setting | Value |
|---------|-------|
| Lidar IP | 192.168.2.183 |
| Gateway | 192.168.2.1 |
| Subnet Mask | 255.255.255.0 |
| Points IP | 192.168.2.5 |
| IMU IP | 192.168.2.5 |
| Serial | 47MDN7K0030083 |

> To change these: open Livox Viewer 2, connect to the sensor, go to Lidar Setting,
> update the fields, click **Confirm**, then **Reboot**.

**Host ethernet setup (one-time, persists across reboots):**

```bash
nmcli connection modify "Profile 1" ipv4.method manual ipv4.addresses "192.168.2.5/24"
nmcli connection up "Profile 1"
```

Verify:

```bash
ping 192.168.2.183
```

> **Why 192.168.2.x?** The default `192.168.1.x` conflicts with most home WiFi networks.
> Since the backpack needs WiFi for remote access (NoMachine), the MID-360 must be on a
> separate subnet. The Ouster uses mDNS/link-local on the same ethernet port and is
> unaffected by this static IP.

### 7f. Configure the MID-360 ROS 2 driver

You must edit the config in **both** locations — the Livox build script copies (not
symlinks) config files, so the `install/` copy is what the driver actually reads:

```bash
# Source config (edit this first):
~/ros2_ws/src/livox_ros_driver2/config/MID360_config.json

# Installed config (this is what the driver actually loads):
~/ros2_ws/install/livox_ros_driver2/share/livox_ros_driver2/config/MID360_config.json
```

> **Tip:** After editing the source config, you can re-run `./build.sh humble` to copy it
> to `install/`, or just edit both files directly.

The key fields that must match your network setup:

```json
{
  "host_net_info" : {
    "cmd_data_ip" : "192.168.2.5",
    "push_msg_ip": "192.168.2.5",
    "point_data_ip": "192.168.2.5",
    "imu_data_ip" : "192.168.2.5"
  },
  "lidar_configs" : [
    {
      "ip" : "192.168.2.183"
    }
  ]
}
```

> All the `host_net_info` IPs = your computer's ethernet IP (`192.168.2.5`).
> The `lidar_configs` → `ip` = the MID-360's IP (`192.168.2.183`).
> Leave all port numbers at their defaults.
> If the driver fails with `bind failed`, the IPs in the **installed** config don't match
> an actual IP on your ethernet interface.

### 7g. Test launch

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch livox_ros_driver2 rviz_MID360_launch.py
```

In another terminal:

```bash
ros2 topic list | grep livox
```

You should see `/livox/lidar` and `/livox/imu`.

Check the point cloud rate:

```bash
ros2 topic hz /livox/lidar
```

---

## 8. Build FAST-LIO2 (ROS 2)

### 8a. Clone

```bash
cd ~/ros2_ws/src
git clone --recursive https://github.com/Rotoslider/FAST_LIO_ROS2.git
```

> This is our fork with backpack-specific fixes (Ouster ring field, gravity alignment,
> MID-360 support, backpack configs). See `CHANGES_BACKPACK.md` in the repo for details.
> The upstream repo is [Ericsii/FAST_LIO_ROS2](https://github.com/Ericsii/FAST_LIO_ROS2).

### 8b. Install dependencies and build the entire workspace

The Livox driver's `build.sh` script builds the entire workspace, so use it as the
single build command for all packages (FAST-LIO2, Ouster, and Livox):

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -y
cd ~/ros2_ws/src/livox_ros_driver2
./build.sh humble
```

> **Important:** Do NOT use `colcon build --symlink-install` after running `build.sh`.
> The two build modes create conflicting artifacts (directories vs symlinks) and you'll
> get `existing path cannot be removed: Is a directory` errors. If you need to rebuild
> after a clean, use `build.sh` again. If you hit symlink errors, clean the affected
> build directories: `rm -rf ~/ros2_ws/build/<package_name>` and rebuild.

### 8c. Verify

```bash
source ~/ros2_ws/install/setup.bash
ros2 pkg list | grep -E "fast_lio|ouster|livox"
```

You should see all four packages:

```
fast_lio
livox_ros_driver2
ouster_ros
ouster_sensor_msgs
```

### 8d. Set up upstream remote (for pulling future updates)

Our fork tracks the original FAST_LIO_ROS2 repo. To pull in upstream bug fixes later:

```bash
cd ~/ros2_ws/src/FAST_LIO_ROS2

# Add the upstream remote (one-time setup, skip if already exists)
git remote add upstream https://github.com/Ericsii/FAST_LIO_ROS2.git

# Fetch latest upstream changes
git fetch upstream

# Merge upstream into your local branch
git merge upstream/ros2

# Push the merged result to your fork
git push origin ros2
```

If the merge has conflicts (upstream changed the same lines we did), git will tell you
which files need manual resolution. Edit those files, then:

```bash
git add <conflicted-files>
git commit
git push origin ros2
```

> **Tip:** Check `git remote -v` to see your remotes. `origin` = your fork,
> `upstream` = the original repo.

---

## 9. Configure FAST-LIO2 for Your Sensors

Config files are in `~/ros2_ws/src/FAST_LIO_ROS2/config/`. You need one per sensor.

### 9a. Ouster OS0-32 config

Copy the existing Ouster config and adjust for 32 channels:

```bash
cp ~/ros2_ws/src/FAST_LIO_ROS2/config/ouster64.yaml \
   ~/ros2_ws/src/FAST_LIO_ROS2/config/ouster32.yaml
```

Edit `ouster32.yaml` — key values to change:

```yaml
common:
    lid_topic: "/ouster/points"
    imu_topic: "/ouster/imu"
    time_sync_en: false

preprocess:
    lidar_type: 3                  # 3 = Ouster
    scan_line: 32                  # 32 channels for OS0-32
    timestamp_unit: 3              # nanoseconds
    blind: 0.5                     # OS0 minimum range is ~0.3m

mapping:
    fov_degree: 360
    det_range: 100.0               # OS0-32 range is ~50m but set higher for margin
    extrinsic_est_en: false
    extrinsic_T: [0.0, 0.0, 0.0]
    extrinsic_R: [1.0, 0.0, 0.0,
                  0.0, 1.0, 0.0,
                  0.0, 0.0, 1.0]
```

### 9b. Livox config

Use the included Livox configs as a starting point. For example, `avia.yaml` or `mid360.yaml`.
The key difference from Ouster:

```yaml
common:
    lid_topic: "/livox/lidar"
    imu_topic: "/livox/imu"

preprocess:
    lidar_type: 1                  # 1 = Livox serial LiDARs
    scan_line: 6                   # varies by model (Avia=6, MID-360=4)
    blind: 4.0
```

---

## 10. Launch and Verify

### 10a. Running with Ouster OS0-32

Terminal 1 — Ouster driver:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch ouster_ros driver.launch.py
```

> Sensor hostname is read from `driver_params.yaml` (see step 6d).

Terminal 2 — FAST-LIO2:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch fast_lio mapping.launch.py config_file:=ouster32.yaml
```

Terminal 3 — Visualization:

```bash
rviz2
```

In rviz2, add displays for:
- `PointCloud2` on topic `/cloud_registered`
- `Path` on topic `/path`
- Set fixed frame to `camera_init`

### 10b. Running with Livox

Terminal 1 — Livox driver:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

Terminal 2 — FAST-LIO2:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml
```

Terminal 3 — rviz2 (same as above).

### 10c. Useful diagnostic commands

```bash
# List all active topics
ros2 topic list

# Check topic publishing rate
ros2 topic hz /ouster/points
ros2 topic hz /livox/lidar

# Check IMU data
ros2 topic echo /ouster/imu --once

# View TF tree
ros2 run tf2_tools view_frames
```

---

## 11. Troubleshooting

### ROS 2 Humble Installation

| Problem | Solution |
|---------|----------|
| `apt update` fails after adding ROS 2 repo | Check the signed-by keyring path is correct: `/usr/share/keyrings/ros-archive-keyring.gpg` |
| `ros-humble-desktop-full` not found | Make sure you have `jammy main` (not `focal`) in `/etc/apt/sources.list.d/ros2.list` |
| `rosdep init` says already initialized | This is fine — just run `rosdep update` |
| Qt5 errors during install | Run `sudo apt install -y qtbase5-dev` then retry |
| `ModuleNotFoundError: No module named 'em'` during build | pyenv is overriding system Python. Run `pyenv local system` in `~/ros2_ws` (see step 4a). |

### Ouster Driver

| Problem | Solution |
|---------|----------|
| "Must specify a sensor hostname" | The driver reads from `driver_params.yaml`, **not** command-line args. Edit `~/ros2_ws/src/ouster-ros/ouster-ros/config/driver_params.yaml` and set `sensor_hostname`. |
| No topics published | Check ethernet connection. Ping the sensor: `ping os-<serial>.local`. Verify your host IP is on the same subnet. |
| `SO_RCVBUF` warning / packet drops | Run: `echo -e "net.core.rmem_max=134217728\nnet.core.rmem_default=134217728" \| sudo tee /etc/sysctl.d/90-ouster.conf && sudo sysctl --system` |
| Packet drops / poor performance | Make sure you built with `-DCMAKE_BUILD_TYPE=Release`. Also apply the receive buffer fix above. |
| "Unknown sensor firmware version" warning | Update sensor firmware to 2.5.3. This is a non-fatal warning but FW 2.5.3 is recommended. |
| Driver crashes on start | Check that firmware is >= 2.4.0. SDK 0.16+ dropped support for older firmware. |
| Point cloud looks distorted in rviz2 | Set the fixed frame to the correct frame (`os_sensor` or `os_lidar`). Check `ros2 topic echo /ouster/metadata --once` for sensor info. |

### Livox Driver

| Problem | Solution |
|---------|----------|
| Build fails: cannot find Livox-SDK2 | Make sure you ran `sudo make install` in the Livox-SDK2 build directory. Check that `/usr/local/lib/liblivox_lidar_sdk_shared.so` exists. |
| Crash: `cannot open shared object file: liblivox_lidar_sdk_shared.so` | Run `sudo ldconfig` to update the dynamic linker cache after installing Livox-SDK2. |
| `./build.sh humble` fails | Make sure you sourced `/opt/ros/humble/setup.bash` first. Also verify `python3-colcon-common-extensions` is installed. |
| `bind failed` / `Failed to init livox lidar sdk` | The `host_net_info` IPs in the config don't match any IP on your system. Check the **installed** config at `install/livox_ros_driver2/share/livox_ros_driver2/config/MID360_config.json` — it must have `192.168.2.5`, not `192.168.1.5`. |
| No topics from Livox sensor | Check all 3 things: (1) `MID360_config.json` IPs match your setup, (2) ethernet has the correct static IP (`ip addr show enp2s0`), (3) sensor is pingable. |
| "Livox lidar not connected" | Verify `ping 192.168.2.183` works. If not, check `nmcli connection show "Profile 1"` for correct IP. Run `nmcli connection up "Profile 1"` to reactivate. |
| WiFi subnet conflict (192.168.1.x) | The MID-360 must NOT be on the same subnet as your WiFi. Change the sensor IP in Livox Viewer 2 to `192.168.2.x` (see step 7e). |
| Livox Viewer can't find sensor after IP change | The sensor and your host must be on the same subnet. Set ethernet to `192.168.2.5/24` first, then open Livox Viewer. |
| Config changes not taking effect | The `livox_ros_driver2` build script may copy configs. After editing `src/livox_ros_driver2/config/MID360_config.json`, re-run `./build.sh humble` or manually copy to `install/`. |

### FAST-LIO2

| Problem | Solution |
|---------|----------|
| Build fails: missing `livox_ros_driver2` | Make sure the Livox driver was built and sourced before building FAST-LIO2. Run `source ~/ros2_ws/install/setup.bash` then rebuild. |
| Build fails: `ikd-Tree` or submodule errors | You forgot `--recursive` when cloning. Run: `cd ~/ros2_ws/src/FAST_LIO_ROS2 && git submodule update --init --recursive` |
| "Failed to find match for field 'time'" | The point cloud is missing per-point timestamps. For Ouster, make sure you are using the official `ouster-ros` driver (not a bag conversion). |
| SLAM drifts badly | Check IMU-lidar time synchronization. Verify the extrinsic calibration in the config YAML. Make sure `scan_line` matches your sensor (32 for OS0-32). |
| Initializes then crashes (exit code -6) | Usually a memory issue. Try reducing `det_range` or increasing `point_filter_num` to reduce point density. |
| Map flies away on first movement | The extrinsic rotation between IMU and lidar is wrong. Measure or calibrate the actual mounting orientation and update `extrinsic_R` in the config. |
| Point cloud topic mismatch | Check that `lid_topic` and `imu_topic` in the config YAML match the actual topics from your driver. Use `ros2 topic list` to verify. |

### General ROS 2 Tips

| Problem | Solution |
|---------|----------|
| `colcon build` can't find a package | Source the workspace first: `source ~/ros2_ws/install/setup.bash`, then rebuild. |
| `existing path cannot be removed: Is a directory` | Build mode conflict between `build.sh` and `colcon build --symlink-install`. Fix: `rm -rf ~/ros2_ws/build/<package_name>` then rebuild with the same method. Use `build.sh` for the full workspace (see step 8b). |
| Changes to config files not picked up | Livox configs are copied (not symlinked) — edit both `src/` and `install/` copies, or re-run `build.sh`. Ouster configs are symlinked if built with `--symlink-install`. |
| QoS incompatibility warnings | Some nodes use "best effort" and others use "reliable" QoS. Check with `ros2 topic info -v <topic>` and adjust. |
| Multiple workspaces conflicting | Check your `~/.bashrc` — make sure you are not sourcing conflicting workspaces. Only source one ROS distro. |

---

## Ouster OS0-32 Gen 5 — Important Notes

- **Max firmware:** 2.5.3 (Rev 5 hardware cannot run FW 3.x)
- **Min firmware for current SDK:** 2.4.0
- FW 2.5.3 fixes a bug causing automatic restarts after 36-118 days of continuous operation
- Rev 5 support has a finite window — Ouster's SDK minimum firmware floor keeps rising
- Monitor: https://github.com/ouster-lidar/ouster-sdk/discussions/532

## Key Differences from ROS 1 Noetic

| ROS 1 (Noetic) | ROS 2 (Humble) |
|-----------------|-----------------|
| `catkin_make` | `colcon build` |
| `source devel/setup.bash` | `source install/setup.bash` |
| `roslaunch` | `ros2 launch` |
| `rostopic` | `ros2 topic` |
| `rosrun` | `ros2 run` |
| `rosmsg` | `ros2 interface` |
| `ros-noetic-*` | `ros-humble-*` |
| `package.xml` format 2 | `package.xml` format 3 |
| `CMakeLists.txt` with catkin | `CMakeLists.txt` with ament_cmake |




---

## 12. WiFi Hotspot for Field Use

In the field (jungle, remote sites) there's no WiFi network for the phone to connect to.
The backpack creates its own WiFi hotspot so the phone can connect directly.

> **Note:** The LattePanda has one WiFi radio — it can either be a hotspot OR connect
> to regular WiFi, not both. Use hotspot mode in the field, regular WiFi at home.

### 12a. Create the hotspot (one-time setup)

```bash
nmcli device wifi hotspot ifname wlo1 ssid "BackpackScanner" password "scanforest"
```

This creates a saved connection profile called "Hotspot". Disable auto-start so it
doesn't hijack your WiFi at home:

```bash
nmcli connection modify Hotspot connection.autoconnect no
```

Then reconnect to your normal WiFi:

```bash
nmcli device wifi connect "Big_starlink"
```

### 12b. Field usage

Before heading into the field:

```bash
# Activate the hotspot
nmcli connection up Hotspot
```

On your phone:
1. Connect to WiFi network **BackpackScanner** (password: `scanforest`)
2. Open **http://10.42.0.1:5000** in the browser

> The backpack IP is always `10.42.0.1` in hotspot mode.

### 12c. Back at home

```bash
# Turn off hotspot and reconnect to normal WiFi
nmcli connection down Hotspot
nmcli device wifi connect "Big_starlink"
```

### 12d. Quick reference

| Mode | Command | Phone connects to | Scanner URL |
|------|---------|-------------------|-------------|
| Field (hotspot) | `nmcli connection up Hotspot` | BackpackScanner WiFi | http://10.42.0.1:5000 |
| Home (WiFi) | `nmcli connection down Hotspot` | Same network as backpack | http://\<backpack-ip\>:5000 |

---

## 13. Backpack Scanner Web App

The backpack scanner is a Flask web app (`backpack_scanner.py`) that controls the full
scan lifecycle from a phone browser. It lives in `~/projects/Lidar_Backpack_V2/`.

### 13a. Install Python dependencies

```bash
pip install flask smbus2
```

### 13b. Run the app

```bash
cd ~/projects/Lidar_Backpack_V2
python3 backpack_scanner.py
```

Open `http://<computer-ip>:5000` on your phone. The app manages:

- Lidar driver launch (Ouster or Livox)
- FAST-LIO2 SLAM with RViz
- ROS 2 bag recording
- Miranda motor control (Ouster only, 20 RPM)
- Ouster standby mode on stop (saves power/heat)
- Scan health monitoring with drift detection

### 13c. Scan health monitor

The app launches `scan_monitor.py` as a subprocess during scans. This is a lightweight
ROS 2 node that subscribes to `/Odometry` and `/cloud_registered` to detect SLAM
degradation in real time. Health status is displayed on the phone UI:

- **Green** — scan healthy, all metrics normal
- **Yellow** — degraded (low point count, rising covariance, high speed)
- **Red (pulsing)** — drift detected (sensor blocked, position jump, SLAM crash)

Monitored metrics:
- Point count from `/cloud_registered` (detects blocked/covered sensor)
- Position covariance from `/Odometry` EKF state
- Velocity and position jumps (detects impossible motion / re-localization failures)
- Odometry message rate (detects SLAM crash)

> **Important:** The scan monitor must run with system Python 3.10 (`/usr/bin/python3`),
> not pyenv's Python 3.11. ROS 2 Humble's `rclpy` C extensions require Python 3.10.
> The app handles this automatically.

### 13d. Output

Scans are saved to `~/pointclouds/<timestamp>_<lidar>/`:
- `scan.pcd` — accumulated point cloud (saved on graceful stop)
- `bag/` — raw sensor data for offline reprocessing
- Ouster metadata JSON (if available)

For full documentation, see `docs/README_Backpack_Scanner.md`.

---

## 14. FAST-LIO2 Backpack Modifications

Several modifications were made to the upstream FAST_LIO_ROS2 fork for backpack use.
These are documented in detail in `~/ros2_ws/src/FAST_LIO_ROS2/CHANGES_BACKPACK.md`.

Key changes:
1. **Ring field fix** (`src/preprocess.h`) — Changed Ouster ring from `uint8_t` to
   `uint16_t`. Without this, scan line ordering is corrupted and SLAM drifts badly.
2. **Gravity alignment** (`src/IMU_Processing.hpp`) — Auto-aligns to gravity at startup.
   Handles arbitrary sensor mount angles (vertical Ouster, tilted Livox).
3. **MID-360 support** (`src/preprocess.h`, `src/preprocess.cpp`) — Added point type
   definitions and handler for the Livox MID-360.
4. **First lidar guard** (`src/laserMapping.cpp`) — Prevents false "loop back" on startup.
5. **Backpack configs** (`config/ouster32_backpack.yaml`, `config/mid360_backpack.yaml`)

To rebuild after changes:

```bash
cd ~/ros2_ws
colcon build --packages-select fast_lio
source install/setup.bash
```


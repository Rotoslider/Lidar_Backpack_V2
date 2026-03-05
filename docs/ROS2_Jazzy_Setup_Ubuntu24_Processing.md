# ROS 2 Jazzy + FAST-LIO Setup — Ubuntu 24.04 (Processing Only)

Setup guide for a **bag reprocessing workstation** running Ubuntu 24.04 (Noble).
This machine replays scan bags through FAST-LIO to produce point clouds — no lidar
hardware, no motor, no hotspot needed.

> **Key differences from the Ubuntu 22.04 backpack setup:**
>
> | | Ubuntu 22.04 (Backpack) | Ubuntu 24.04 (Processing) |
> |---|---|---|
> | ROS 2 distro | Humble | **Jazzy** |
> | System Python | 3.10 | **3.12** |
> | GCC | 11 | **13** (stricter) |
> | PCL (apt) | 1.12 (C++14) | **1.14 (C++14)** — needs fix for C++17 |
> | Packages | `ros-humble-*` | `ros-jazzy-*` |
> | Hardware | Lidars, motor, I2C | **None** |

---

## Table of Contents

1. [Prerequisites](#1-prerequisites)
2. [Install pyenv and Python 3.11](#2-install-pyenv-and-python-311)
3. [Install ROS 2 Jazzy](#3-install-ros-2-jazzy)
4. [Create the Colcon Workspace](#4-create-the-colcon-workspace)
5. [Install System Dependencies](#5-install-system-dependencies)
6. [Build the Ouster ROS 2 Driver](#6-build-the-ouster-ros-2-driver)
7. [Build Livox SDK2 and ROS 2 Driver](#7-build-livox-sdk2-and-ros-2-driver)
8. [Build FAST-LIO2 (ROS 2)](#8-build-fast-lio2-ros-2)
9. [Fix PCL / Eigen C++17 Alignment Issue](#9-fix-pcl--eigen-c17-alignment-issue)
10. [Copy the Processing Tools](#10-copy-the-processing-tools)
11. [Test with a Bag](#11-test-with-a-bag)
12. [Troubleshooting](#12-troubleshooting)

---

## 1. Prerequisites

- Ubuntu 24.04 (Noble) fresh install
- Internet connection
- At least 8 GB RAM (FAST-LIO map building is memory-intensive)
- Scan bags copied from the backpack (`~/pointclouds/`)

---

## 2. Install pyenv and Python 3.11

The reprocessor GUI (`fastlio_reprocessor.py`) uses PyQt5 which works best under
Python 3.11. System Python 3.12 is used for ROS 2.

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

## 3. Install ROS 2 Jazzy

> Jazzy is the LTS ROS 2 distro for Ubuntu 24.04, same role as Humble for 22.04.

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
  http://packages.ros.org/ros2/ubuntu noble main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

> **Note:** This uses `noble` instead of `jammy`. You **cannot** install Humble on 24.04.

### 3c. Install Jazzy desktop

```bash
sudo apt update
sudo apt install ros-jazzy-desktop -y
```

### 3d. Source ROS 2 and add to bashrc

```bash
source /opt/ros/jazzy/setup.bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
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

You should see `ROS_DISTRO=jazzy`.

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

### 4a. Fix pyenv / ROS 2 Python conflict

ROS 2 Jazzy is built against **system Python 3.12**. Pin the workspace to system Python:

```bash
cd ~/ros2_ws
pyenv local system
```

This ensures `colcon build` uses system Python 3.12 (which has `rclpy`), while your
global Python 3.11 still works for the reprocessor GUI.

---

## 5. Install System Dependencies

```bash
sudo apt install -y \
  build-essential cmake \
  libeigen3-dev libpcl-dev libpcap-dev \
  libjsoncpp-dev libspdlog-dev libcurl4-openssl-dev \
  libtins-dev libzip-dev \
  ros-jazzy-pcl-ros \
  ros-jazzy-pcl-conversions \
  ros-jazzy-rviz2 \
  ros-jazzy-cv-bridge \
  ros-jazzy-tf2-eigen \
  python3-pip \
  openssh-client
```

> **Note:** `ros-jazzy-navigation2` and `ros-jazzy-robot-localization` are NOT needed
> for processing only. Install them if you want them for other projects.

---

## 6. Build the Ouster ROS 2 Driver

The Ouster driver officially supports Jazzy. Needed for replaying Ouster bags
(raw packets go through the driver to produce point clouds).

### 6a. Clone

```bash
cd ~/ros2_ws/src
git clone --recurse-submodules -b ros2 https://github.com/ouster-lidar/ouster-ros.git
```

### 6b. Build

```bash
cd ~/ros2_ws
rosdep install --from-paths src/ouster-ros --ignore-src -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --packages-select ouster_ros ouster_sensor_msgs
source install/setup.bash
```

---

## 7. Build Livox SDK2 and ROS 2 Driver

Needed because FAST-LIO depends on `livox_ros_driver2` message types, even for
Ouster-only reprocessing. The official Livox driver does not support Jazzy out of
the box.

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

### 7b. Clone the Livox ROS 2 driver

Use the **TU Darmstadt fork** which has proper CMake for Jazzy:

```bash
cd ~/ros2_ws/src
git clone https://github.com/tu-darmstadt-ros-pkg/livox_ros_driver2.git
```

> If the TU Darmstadt fork doesn't work, fall back to the official repo with the build
> flag workaround:
> ```bash
> git clone https://github.com/Livox-SDK/livox_ros_driver2.git
> cd livox_ros_driver2
> cp package_ROS2.xml package.xml
> # Then build with: colcon build --cmake-args -DROS_EDITION=ROS2 -DHUMBLE_ROS=humble
> ```

### 7c. Build

```bash
cd ~/ros2_ws
rosdep install --from-paths src/livox_ros_driver2 --ignore-src -y
colcon build --packages-select livox_ros_driver2
source install/setup.bash
```

> If this fails with CMake errors, see [Troubleshooting](#livox-driver-build-fails).

---

## 8. Build FAST-LIO2 (ROS 2)

### 8a. Clone

```bash
cd ~/ros2_ws/src
git clone --recursive https://github.com/Rotoslider/FAST_LIO_ROS2.git
```

### 8b. Apply Jazzy compatibility patches

FAST-LIO was written for Humble. Jazzy has breaking API changes that must be fixed
before building.

**Patch 1: Subscriber callback signatures** — Jazzy removed non-const shared_ptr callbacks.

```bash
cd ~/ros2_ws/src/FAST_LIO_ROS2
```

In `src/laserMapping.cpp`, find any subscriber callbacks that use
`std::shared_ptr<MsgType>` (non-const) and change to `std::shared_ptr<const MsgType>`.

Look for patterns like:

```cpp
// BEFORE (Humble — won't compile on Jazzy):
void imu_cbk(const sensor_msgs::msg::Imu::SharedPtr msg)
void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)

// AFTER (Jazzy compatible — also works on Humble):
void imu_cbk(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr msg)
```

> `ConstSharedPtr` is an alias for `std::shared_ptr<const MsgType>`.

**Patch 2: tf2 header paths** — Jazzy removed legacy `.h` headers.

In any file that includes tf2 headers, change `.h` to `.hpp`:

```cpp
// BEFORE:
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

// AFTER:
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
```

> If FAST-LIO doesn't use these headers directly, skip this patch.

### 8c. Build

```bash
cd ~/ros2_ws
rosdep install --from-paths src/FAST_LIO_ROS2 --ignore-src -y
colcon build --packages-select fast_lio
source install/setup.bash
```

### 8d. Verify all packages

```bash
ros2 pkg list | grep -E "fast_lio|ouster|livox"
```

Expected:

```
fast_lio
livox_ros_driver2
ouster_ros
ouster_sensor_msgs
```

---

## 9. Fix PCL / Eigen C++17 Alignment Issue

> **This section may or may not be needed.** Try building and running first. If you get
> runtime crashes (`free(): invalid next size`, segfaults in PCL functions), this is the fix.

Ubuntu 24.04's apt PCL (1.14) was compiled with C++14, but ROS 2 Jazzy uses C++17.
This creates an Eigen memory alignment mismatch that causes crashes.

### Option A: Add PCL_NO_PRECOMPILE (quick fix)

Add this define **before** any PCL includes in FAST-LIO source files:

```cpp
#define PCL_NO_PRECOMPILE
```

Files to check: `src/laserMapping.cpp`, `src/preprocess.cpp`, `include/common_lib.h`

Then rebuild:

```bash
cd ~/ros2_ws && colcon build --packages-select fast_lio
```

### Option B: Build PCL from source with C++17 (thorough fix)

```bash
cd ~
git clone -b pcl-1.14.1 https://github.com/PointCloudLibrary/pcl.git
cd pcl && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=17
make -j$(nproc)
sudo make install
sudo ldconfig
```

Then rebuild the ROS workspace:

```bash
cd ~/ros2_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

---

## 10. Copy the Processing Tools

Copy these files from the backpack to the processing machine:

```
~/projects/Lidar_Backpack_V2/
  fastlio_reprocessor.py       # Reprocessor GUI
  scan_monitor.py              # Health monitor (ROS 2 node)
  templates/                   # Not needed for reprocessor
  os-*-metadata.json           # Ouster metadata (copy if not in scan dirs)
  reprocessor_presets/          # Saved parameter presets
```

Install Python dependencies for the reprocessor GUI (under pyenv Python 3.11):

```bash
pip install PyQt5 pyyaml flask
```

> `flask` is listed because the reprocessor imports some shared utilities. If it doesn't,
> you can skip it.

### Copy scan bags

Copy scan directories from the backpack:

```bash
# On the backpack:
rsync -avz ~/pointclouds/ user@processing-machine:~/pointclouds/

# Or use a USB drive, network share, etc.
```

Each scan directory should contain:

```
~/pointclouds/2026-03-02_14-30-35_ouster/
  bag/                          # ROS 2 bag files
  os-122134000147-metadata.json # Ouster metadata (required for replay)
```

---

## 11. Test with a Bag

### 11a. Using the Reprocessor GUI

```bash
cd ~/projects/Lidar_Backpack_V2
python3 fastlio_reprocessor.py
```

Select a scan, adjust parameters, click PROCESS.

### 11b. Manual replay (for debugging)

Terminal 1 — Start FAST-LIO:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch fast_lio mapping.launch.py \
  config_file:=ouster32_backpack.yaml rviz:=true use_sim_time:=true
```

Terminal 2 — Start Ouster replay driver:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch ouster_ros replay.composite.launch.xml \
  bag_file:=b \
  metadata:=/path/to/os-122134000147-metadata.json \
  viz:=false use_system_default_qos:=true \
  point_cloud_frame:=os_sensor point_type:=original
```

Terminal 3 — Play the bag:

```bash
source ~/ros2_ws/install/setup.bash
ros2 bag play ~/pointclouds/<scan>/bag --clock --rate 0.5 \
  --remap /os_node/metadata:=/ouster/metadata \
  /os_node/imu_packets:=/ouster/imu_packets \
  /os_node/lidar_packets:=/ouster/lidar_packets
```

---

## 12. Troubleshooting

### FAST-LIO won't compile on Jazzy

| Error | Fix |
|-------|-----|
| `no matching function for call to 'create_subscription'` with SharedPtr | Change callback param from `MsgType::SharedPtr` to `MsgType::ConstSharedPtr` (see step 8b) |
| `tf2_sensor_msgs/tf2_sensor_msgs.h: No such file` | Change `.h` to `.hpp` in the include |
| `error: 'QOS_EVENT'` or `qos_event.hpp not found` | Change `#include <rclcpp/qos_event.hpp>` to `#include <rclcpp/event_handler.hpp>` |

### PCL crashes at runtime

| Symptom | Fix |
|---------|-----|
| `free(): invalid next size` or segfault in PCL | C++14/C++17 alignment mismatch. See [section 9](#9-fix-pcl--eigen-c17-alignment-issue) |
| `SIGABRT` in `pcl::KdTreeFLANN` | Same fix — PCL_NO_PRECOMPILE or build PCL from source |

### Livox driver build fails

| Error | Fix |
|-------|-----|
| `LIVOX_INTERFACES_INCLUDE_DIRECTORIES is set to NOTFOUND` | Use the TU Darmstadt fork (step 7b) |
| CMake can't find Livox SDK2 | Verify `/usr/local/lib/liblivox_lidar_sdk_shared.so` exists. Run `sudo ldconfig`. |
| `build.sh humble` fails on Jazzy | Don't use `build.sh`. Use `colcon build --packages-select livox_ros_driver2` directly (with the TU Darmstadt fork) |

### Reprocessor GUI issues

| Problem | Fix |
|---------|-----|
| `ModuleNotFoundError: No module named 'rclpy'` | The script that imports rclpy (`scan_monitor.py`) must run with system Python 3.12 (`/usr/bin/python3`), not pyenv. The reprocessor handles this automatically. |
| PyQt5 import error | Install under pyenv: `pip install PyQt5` |
| Can't find scan directories | Scans must be in `~/pointclouds/`. Each subdirectory needs a `bag/` folder. |

### General

| Problem | Fix |
|---------|-----|
| `colcon build` uses wrong Python | Run `pyenv local system` in `~/ros2_ws` |
| `ModuleNotFoundError: No module named 'em'` | Same as above — pyenv conflict |
| GCC 13 new warnings breaking build | Add `-Wno-dangling-reference` to cmake args: `colcon build --cmake-args -DCMAKE_CXX_FLAGS="-Wno-dangling-reference"` |

---

## What's NOT Needed (vs. the backpack)

These are in the Ubuntu 22.04 guide but **skipped** for processing-only:

- CH341 I2C driver and udev rules (Miranda motor)
- `smbus2` Python package
- WiFi hotspot setup
- Ethernet static IP configuration (192.168.2.x for Livox, link-local for Ouster)
- UDP receive buffer tuning (no live sensor traffic)
- `backpack_scanner.py` web app
- PolicyKit / sudoers rules
- systemd service files
- Ouster standby mode configs (`ouster_standby.json`, `ouster_normal.json`)

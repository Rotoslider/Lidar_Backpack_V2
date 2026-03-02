Computer: Latte Panda Delta3
Motor: Overview Miranda Servotorq OVU20012
I2C interface: ch341A all in one board (usb)
Lidar: Ouster OS0-32 (os-122134000147)
Backpack: Aleon
Battery: Makita (BL1850B)

Login: user: lidar pw: lidar



remove Thunderbird
sudo apt-get purge thunderbird*
if you launced it before remove folder otherwise your done
rm -r /home/lidar/.thunderbird

------------------------------------------

use pyenv so you can run everything with python 3.10.14 and not 3.8. using pyenv prevents ubuntu from breaking
prereqs:
sudo apt-get install -y build-essential libssl-dev zlib1g-dev libbz2-dev \
libreadline-dev libsqlite3-dev wget curl llvm libncurses5-dev libncursesw5-dev \
xz-utils tk-dev libffi-dev liblzma-dev python3-openssl git
curl https://pyenv.run | bash
# Load pyenv automatically by appending
# the following to 
# and ~/.bashrc (for interactive shells) :                                                   
                  
# Load pyenv automatically
export PYENV_ROOT="$HOME/.pyenv"
[[ -d $PYENV_ROOT/bin ]] && export PATH="$PYENV_ROOT/bin:$PATH"
eval "$(pyenv init - bash)"
eval "$(pyenv virtualenv-init -)"

  Then:
  source ~/.bashrc
  pyenv install 3.11.9
  pyenv global 3.11.9

verify with:
python3 --version

----------------------------------------------------------------------------------
****install new cmake first or you will need to do everything again when you remove it and reinstall it*****
sudo apt install python3-pip
python3 -m pip install --upgrade pip
sudo apt remove --purge cmake
sudo apt autoremove
python3 -m pip install cmake

-----------------------------------
setup nemo file manager:
sudo apt install nemo
make it the defualt:
xdg-mime default nemo.desktop inode/directory application/x-gnome-saved-search
gsettings set org.gnome.desktop.background show-desktop-icons false
gsettings set org.nemo.desktop show-desktop-icons true

install usefull tools:
sudo apt-get update -y
sudo apt install pluma
sudo apt install nano
python3 -m pip install pyyaml
sudo apt-get install -y sharutils

-----------------------------------
    Install other dependenices:
    
sudo apt install -y         \
    build-essential         \
    libeigen3-dev           \
    libjsoncpp-dev          \
    libspdlog-dev           \
    libcurl4-openssl-dev    
       
-------------------------------------------------------
you will need tcpdump for the veldoyne part of the app.
sudo apt-get install tcpdump
-------------------------------------------------------



Install Ros Noetic running Ubuntu 22.04:

---------------------------------------------
    
Install Ouster SDK:
python3 -m pip install --upgrade pip

***close terminal and open a new one then continue****
python3 -m pip install protobuf==4.23.4
python3 -m pip install --upgrade pip setuptools
python3 -m pip install 'ouster-sdk[examples]'

echo 'export PYTHONPATH="/home/lidar/.pyenv/versions/3.11.9/lib/python3.11/site-packages:$PYTHONPATH"' >> ~/.bashrc
or for the AGX:
echo 'export PYTHONPATH="/home/lidar_agx/.pyenv/versions/3.11.9/lib/python3.11/site-packages:$PYTHONPATH"' >> ~/.bashrc
  
cd ~/catkin_ws/src
git clone --recurse-submodules https://github.com/ouster-lidar/ouster-ros.git

source /opt/ros/noetic/setup.bash
cd ..
catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release
rosdep install --from-paths /home/lidar/catkin_ws/build/ouster-ros/ouster_example
or for AGX:
rosdep install --from-paths /home/lidar_agx/catkin_ws/build/ouster-ros/ouster_example
then run this again:
catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release

source devel/setup.bash
run the following to write to bash file
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

-----------------------------------------------
Setup Ouster ethernet connection
Set eth0 to link local under ip4
use Firefox to connect to lidar using serial # (os-122134000147) to verify function
os-122134000147.local/
or my os)-32:
os-122135000245.local/

Ip address is 169.254.181.131
or:
169.254.171.90
169.254.201.29 (my os0-32)
UDP Destination Address:  169.254.125.136

firewall may not be installed but incse do allow udp packets thru firewall:
sudo ufw allow 7502/udp

To discover sensor on local network use:
ouster-cli discover

to record a pcap and use CTRL+C to stop
ouster-cli source os-122135000245.local record
===========================================================================

ouster-cli source os-122135000245.local viz

Ouster SDK SLAM option:
install kiss icp to run:
python3 -m pip install --force-reinstall -v "kiss-icp==0.2.10"

live (Error: No such command 'mapping'. Need to figure this out):
ouster-cli mapping slam os-122135000245.local -o test1.osf viz
from a pcap:
ouster-cli mapping slam test1.pcap -o test1.osf viz

Convert to pointcloud
need to install first http://www.open3d.org/docs/release/arm.html:
sudo apt-get update -y
sudo apt-get install -y ccache
sudo apt-get install -y xorg-dev libglu1-mesa-dev
sudo apt-get install -y libblas-dev liblapack-dev liblapacke-dev
sudo apt-get install -y libsdl2-dev libc++-7-dev libc++abi-7-dev libxi-dev
sudo apt-get install -y clang-7
python3 -m pip install open3d-python -U --no-cache-dir


ouster now uses point cloud tools but the install fails on the jetson:
python3 -m pip install point-cloud-utils


The convert command converts the SLAM-generated OSF file to a point cloud data file format such as LAS (.las), PLY (.ply), or PCD (.pcd). The output file format depends on the extension of the output filename. Let’s use the OSF file generated from the SLAM command and convert it to a PLY file:

ouster-cli mapping convert test1.osf test1.ply


-------------------------------------------------
https://docs.nvidia.com/jetson/archives/r36.2/DeveloperGuide/HR/ConfiguringTheJetsonExpansionHeaders.html?highlight=jetson%20io#running-jetson-io
to setup the gpio and CSI connector or M2 slot you can use:
sudo /opt/nvidia/jetson-io/jetson-io.py
-----------------------------------------------------------------------------

-----------------------------------------------------------------------------
=============================================================================
-----------------------------------------------------------------------------

++**Setup Miranda**++

set UDP buffer size
sudo sysctl -w net.core.rmem_max=100000
sudo sysctl -w net.core.rmem_default=100000

Install Jetson.GPIO
python3 -m pip install Jetson.GPIO

add to bashrc
export PYTHONPATH="/usr/local/lib/python3.8/dist-packages/Jetson.GPIO-2.1.6-py3.8.egg:$PYTHONPATH"
export PYTHONPATH="/usr/local/lib/python3.11.9/dist-packages/Jetson.GPIO-2.1.6-py3.11.9.egg:$PYTHONPATH"


Install smbus2 (https://pypi.org/project/smbus2/)
python3 -m pip install smbus2
sudo pip3 install smbus2

python3 -m pip install more-itertools
sudo pip3 install more-itertools

Setting User Permissions https://github.com/JetsonHacksNano/ServoKit/blob/master/scripts/setPermissions.sh

sudo groupadd -f -r gpio
sudo usermod -a -G gpio $USER
sudo usermod -aG i2c $USER

sudo cp .pyenv/versions/3.11.9/lib/python3.11/site-packages/Jetson/GPIO/99-gpio.rules /etc/udev/rules.d/

sudo udevadm control --reload-rules && sudo udevadm trigger

add user to sudoers so you can shutdown without password from code:
sudo visudo
lidar ALL=(ALL) NOPASSWD: /sbin/shutdown

Install I2C support
sudo apt-get install libi2c-dev i2c-tools

REBOOT:

detect the motor on bus 7
sudo i2cdetect -y -r 7  Orin Nano


When Jumper is set to ON the device address is 0x29 if its set to one its 0x28

------------------------------------------------
the bus speed of the Miranda motor is 100000. This works well with the Jetson AGX. It does not work well with the Jetson NANO. You must set the bus speed of the NANO to 100k. It defaults to 400k. 
A temp solution is to open the file/sys/bus/i2c/devices/i2c-0/bus_clk_rate as root and change the value from 400000 to 100000. This file will overwrite itself after reboot so you will need to do it each time. Or rewrite the kernal.
--------------------------------------------------
setup bus to correct speed each boot:

1. **Create a systemd service file:**
    Open a terminal and type:
sudo nano /etc/systemd/system/change-bus-clk-rate.service

    This will create a new systemd service file that you can edit.

2. **Add Service Content:**
    In the text editor, copy-paste the following content:
    [Unit]
    Description=Change bus_clk_rate on i2c-0

    [Service]
    Type=oneshot
    ExecStart=/bin/sh -c "echo '100000' > /sys/bus/i2c/devices/i2c-0/bus_clk_rate"

    [Install]
    WantedBy=multi-user.target

    Make sure to save the file (`Ctrl+O`) and exit (`Ctrl+X`).

3. **Enable and Start the Service:**
    Now let's enable the service so it runs at boot, and start it to make the initial change:

sudo systemctl enable change-bus-clk-rate.service
sudo systemctl start change-bus-clk-rate.service
 

4. **Verify:**
    Reboot your Jetson Nano and check the file to make sure the value has changed to `100000`. You can do this by running:

cat /sys/bus/i2c/devices/i2c-0/bus_clk_rate
 

Run the Miranda script from a terminal where it resides:

sudo python3 miranda_tls_v1.6.py
sudo python3 orin_miranda_tls_v1.7.py
sudo python3 orin_miranda_control_v0.2.py
or if in an environment:
sudo /home/lidar/.pyenv/versions/3.11.9/bin/python3 orin_miranda_tls_v1.7.py
or for the AGX:
sudo /home/lidar/.pyenv/versions/3.11.9/bin/python3 orin_miranda_tls_v1.7.py
______________________________________________________________________________
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

SDK run to view:
ouster-cli source 169.254.201.29 viz
ouster-cli source os-122135000245.local viz
---------------------------------------------------------------------- 
ouster-cli source test1.pcap info

look at your pcap:
ouster-cli source <PCAP_FILE> info
tells us the number of packets belonging to each port captured in the pcap, and the associated size.

To visualize the pcap at 2x speed while looping back:
ouster-cli source <PCAP_FILE.pcap> viz -r 2.0 -e loop

You can check check out all the available options by typing --help after ouster-cli source <PCAP_FILE.pcap> viz.

===========================================================

install fastlio:
Prerequisites:
PCL and Eigen
sudo apt-get install python3-pcl pcl-tools
sudo apt-get install libeigen3-dev
sudo apt-get install libyaml-cpp-dev

Livox SDK:
git clone https://github.com/Livox-SDK/Livox-SDK.git
cd Livox-SDK
cd build && cmake ..
make
sudo make install

livox_ros_driver:
cd ~/catkin_ws/src
git clone https://github.com/Livox-SDK/livox_ros_driver.git ws_livox/src
cd ws_livox
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
source ./devel/setup.sh


cd ~/catkin_ws/src
git clone https://github.com/hku-mars/FAST_LIO.git
cd FAST_LIO
git submodule update --init   
cd ../..
catkin_make
source devel/setup.bash
    
copy the config, launch, plot.py and rviz_cfg files from your backup folder to the proper folders in your fast_lio directory. 


uses GUI to turn on lidar and launch FastLIO, It will start a bag recording and start the motor at 20rpm and run till you press stop scan:
/home/lidar/.pyenv/versions/3.11.9/bin/python3 orin_scanner_v0.3.py
or for the AGX:
/home/lidar/.pyenv/versions/3.11.9/bin/python3 orin_scanner_v0.3.py  


==============================================================
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
=============================================================

---------------------
*** MANUAL METHOD ***
---------------------
cd to catkin_ws
use this open a ros node and shows lidar points:
roslaunch ouster_ros sensor.launch sensor_hostname:=169.254.201.29
-----------------------------------------------------------------------------------------------
to use Fast_Lio (LIVE) you must launch the ouster first with rviz off then launch Fast_Lio:

roslaunch ouster_ros sensor.launch viz:=false sensor_hostname:=169.254.201.29
roslaunch fast_lio mapping_ouster32.launch sensor_hostname:=169.254.201.29

roslaunch ouster_ros sensor.launch viz:=false sensor_hostname:=os-122134000147
roslaunch fast_lio mapping_ouster32.launch sensor_hostname:=os-122134000147


rosbag record --duration 1m -o /home/lidar/pointclouds/O1m /ouster/imu_packets /ouster/lidar_packets
or for the AGX:
rosbag record --duration 1m -o /home/lidar_agx/pointclouds/O1m /ouster/imu_packets /ouster/lidar_packets

you can run miranda motor control for now to spin the motor:
sudo /home/lidar/.pyenv/versions/3.11.9/bin/python3 orin_miranda_control_v0.2.py
sudo /home/lidar_agx/.pyenv/versions/3.11.9/bin/python3 orin_miranda_control_v0.2.py

example bag record:
Record bag file:
roslaunch ouster_ros record.launch      \
    sensor_hostname:=169.254.201.29  \
    metadata:=$PWD/test1     \
    bag_file:=$PWD/test1
saves to root of working directory i.e catkin_ws
-------------------------------------------------------
---------------------------------------------------------------------------
Recording Data

To record raw sensor output you may use the provided record.launch file as follows:

roslaunch ouster_ros record.launch      \
    sensor_hostname:=169.254.201.29  \
    metadata:=<json file name>          \
    bag_file:=<optional bag file name>

This will connect to the specified sensor, write the sensor metadata to a file and start recording imu and lidar packets to the specified bag_file once the sensor is connected.

It is necessary that you provide a name for the metadata file and maintain this file along with the recorded bag_file otherwise you won’t be able to play the file correctly.

If no bag_file is specified then a name will be generated based on the current date/time.

By default ROS saves all files to $ROS_HOME, if you want to have these files saved in the current directory, simply give the absolute path to each file. For example:

roslaunch ouster_ros record.launch      \
    sensor_hostname:=<sensor hostname>  \
    metadata:=$PWD/<json file name>     \
    bag_file:=$PWD/<bag file name>
----------------------------------------------------------------------
If a bag was not closed cleanly, then the index information may be corrupted. Two steps are needed to repair the bag:

rosbag reindex *.bag.active

rosbag fix *.bag.active repaired.bag 
-----------------------------------------------------------------------
Playing Back Recorded Data using Ouster ROS player

You may use the replay.launch file to repalay previously captured sensor data. Simply invoke the launch file with the following parameters:

roslaunch ouster_ros replay.launch      \
    metadata:=/home/lidar/pointclouds/copperfalls/ouster1.json          \
    bag_file:=/home/lidar/pointclouds/copperfalls/copper5.bag

roslaunch ouster_ros replay.launch viz:=true metadata:=/home/lidar/pointclouds/ouster1.json bag_file:=/home/lidar/pointclouds/three.bag
roslaunch ouster_ros replay.launch viz:=false metadata:=/home/lidar/pointclouds/SpinnyO/ouster1.json bag_file:=/home/lidar/pointclouds/SpinnyO/dense_brush1.bag        
    ----------------------------------------------------------------------
    edit ouster_ros replay.launch to add function for starting a set amount of time into a bag file and to set the playback rate
    <node if="$(arg _use_bag_file_name)" pkg="rosbag" type="play" name="rosbag_play_recording"
    launch-prefix="bash -c 'sleep 3; $0 $@' "
    output="screen" required="true"
    args="--clock $(arg bag_file) --start 1 --rate 4"/> <!-- use any other values -->
    
  origianl code below:
  <node if="$(arg _use_bag_file_name)" pkg="rosbag" type="play" name="rosbag_play_recording"
    launch-prefix="bash -c 'sleep 3; $0 $@' "
    output="screen" required="true"
    args="--clock $(arg bag_file)"/>
         
=================================================================
   
   To create a favorites icon create a desktop entry 
-----------------------------------------------------------

Edit desktop icons and bash scripts to the proper username etc

Place .launchscripts into:
/home/lidar/catkin_ws/.launchscripts

or for the AGX:
/home/lidar_agx/catkin_ws/.launchscripts

Place desktop icons into:
user/share/applications

[Desktop Entry]
Type=Application
Terminal=false $ true opens extra gnome-terminal
Name=Ouster1m
Icon=/home/lidar/catkin_ws/.launchscripts/bagrecord.jpg
Exec=gnome-terminal -e "bash -c '/home/lidar/catkin_ws/.launchscripts/ouster1m.sh;'"
Name[en_US]=ouster1m

==========================================================
Examples of start scripts called by the favorites:

LaunchOuster.sh:
roslaunch ouster_ros sensor.launch sensor_hostname:=os-122135000245.local metadata:=/home/lidar/pointclouds/ouster1.json

ouster1m.sh:
rosbag record --duration 1m -o /home/lidar/pointclouds/O1m /ouster/imu_packets /ouster/lidar_packets
ouster5m.sh:
rosbag record --duration 5m -o /home/lidar/pointclouds/O5m /ouster/imu_packets /ouster/lidar_packets
ouster10m.sh:
rosbag record --duration 10m -o /home/lidar/pointclouds/O10m /ouster/imu_packets /ouster/lidar_packets
ouster15m.sh:
rosbag record --duration 15m -o /home/lidar/pointclouds/O15m /ouster/imu_packets /ouster/lidar_packets

====================================================================================

Fast_Lio
to process prerecorded Ouster bag files:
roslaunch fast_lio mapping_ouster32.launch

rosbag play does not work for Ouster. You must load the metadata file also using ouster_ros replay.launch:

roslaunch ouster_ros replay.launch viz:=false metadata:=/home/lidar/pointclouds/ouster1.json bag_file:=/home/lidar/pointclouds/02trees_meadow5m.bag

roslaunch ouster_ros replay.launch viz:=false metadata:=/home/lidar_agx/pointclouds/wag.json bag_file:=/home/lidar_agx/pointclouds/wag1.bag

roslaunch ouster_ros replay.launch viz:=true metadata:=/home/lidar/pointclouds/mato/mato.json bag_file:=/home/lidar/pointclouds/mato/mato.bag

If slam fails run bag file a couple seconds before and pause with spacebar. Then launch Fast_Lio
Default Fast_Lio setup:
PARAMETERS
 * /common/imu_topic: /ouster/imu
 * /common/lid_topic: ouster/points
 * /common/time_offset_lidar_to_imu: 0.0
 * /common/time_sync_en: False
 * /cube_side_length: 1000.0
 * /feature_extract_enable: False
 * /filter_size_map: 0.5
 * /filter_size_surf: 0.5
 * /mapping/acc_cov: 0.1
 * /mapping/b_acc_cov: 0.0001
 * /mapping/b_gyr_cov: 0.0001
 * /mapping/det_range: 50.0
 * /mapping/extrinsic_R: [1, 0, 0, 0, 1, 0...
 * /mapping/extrinsic_T: [0.0, 0.0, 0.0]
 * /mapping/extrinsic_est_en: False
 * /mapping/fov_degree: 360
 * /mapping/gyr_cov: 0.1
 * /max_iteration: 3
 * /pcd_save/interval: -1
 * /pcd_save/pcd_save_en: True
 * /point_filter_num: 4
 * /preprocess/blind: 2
 * /preprocess/lidar_type: 3
 * /preprocess/scan_line: 32
 * /preprocess/timestamp_unit: 3
 * /publish/dense_publish_en: True
 * /publish/path_en: True
 * /publish/scan_bodyframe_pub_en: True
 * /publish/scan_publish_en: True
 * /rosdistro: noetic
 * /rosversion: 1.16.0
 * /runtime_pos_log_enable: True

NODES
  /
    laserMapping (fast_lio/fastlio_mapping)
    rviz (rviz/rviz)
--------------------------------------------------------------
filter_size_surf: Downsample the points in a new scan. It is recommended that 0.05~0.15 for indoor scenes, 0.3~0.5 for outdoor scenes.
filter_size_map: Downsample the points in LiDAR global map. It is recommended that 0.15~0.3 for indoor scenes, 0.4~0.5 for outdoor scenes.
------------------------------------------------------------

Set RViz to 3rd person follow taget frame body

++++++++++++++++++++++++++++++++++++++++++++
to use the plot.py in log folder you will need to copy over custom version of plot.py
 
then run
python3 plot.py
---------------------------------------------------- 

to view PCD files saved by Fast_Lio use:
sudo apt install pcl-tools
run from where the PCD is save such as pointclouds folder or provide path
pcl_viewer scans.pcd
/home/lidar/catkin_ws/src/FAST_LIO/PCD/sans.pcd is the old default save location of Fast_Lio. The PCD files are now saved to the pointclouds folder with a timestamp to match the bag file.
commands:
help = h
exit = e
screenshot = j
change what to visualize/color by pressing keyboard 1,2,3,4,5 when pcl_viewer is running.
    1 is all random
    2 is X values
    3 is Y values
    4 is Z values
    5 is intensity

==================================================================================
   

---------------------
Fast_LIO config that works currenty with legacy and new "RNG19_RFL8_SIG16_NIR16" modes:
    lid_topic:  "ouster/points"
    imu_topic:  "/ouster/imu"
    
other fast lio config ouser settings which do not work with my sensor:
lid_topic:  "/os_cloud_node/points"
imu_topic:  "/os_cloud_node/imu"
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

Setting config modes for ouster lidar
Lidar must be running. Either ouster_launch or ouster_live_launch
create a standby json config file then call it from a second terminal
rosservice call /ouster/set_config "config_file: /home/lidar/catkin_ws/.launchscripts/ouster_standby.json"
or to make run after it is put in standby:
rosservice call /ouster/set_config "config_file: /home/lidar/catkin_ws/.launchscripts/ouster_run.json"
Or maybe simpler just close the ouster launch terminal and turn off the switch to the lidar. Could be controlled by a relay and the agx later

+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++=
To read settings and info from a terminal use nc:
nc 169.254.201.29 7501
get_sensor_info
get_time_info
set_config_param lidar_mode 512x10
set_config_param lidar_mode 1024x10

or to stop the lidar:
nc 169.254.201.29 7501
set_config_param operating_mode STANDBY
reinitialize

or to turn it on use:
set_config_param operating_mode NORMAL

or use web browser
169.254.201.29/api/v1/sensor/metadata/sensor_info
169.254.181.131/api/v1/sensor/metadata/sensor_info
169.254.181.131/api/v1/time/system
169.254.181.131/api/v1/time/sensor

++++++++++++++++++++++++++++++++++++++++++++++++++++++++==
only use this if you force wrong python version to system
maybe how to fix removing default python

Make symbolic link

ls -la /usr/bin/python3
sudo rm /usr/bin/python3
sudo ln -s python3.10 /usr/bin/python3

enter terminal ctrl + alt + f3  to return its ctrl + alt + f7

switch back:
ls -la /usr/bin/python3
sudo rm /usr/bin/python3
sudo ln -s python3.8 /usr/bin/python3

verify:
python3 --version



=======================================================================================
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
=======================================================================================

+++Setup camera+++

Arducam B0249 12mp IMX477
https://docs.arducam.com/Nvidia-Jetson-Camera/Native-Camera/Quick-Start-Guide/#Software
install drivers:
wget https://github.com/ArduCAM/MIPI_Camera/releases/download/v0.0.3/install_full.sh
chmod +x install_full.sh
./install_full.sh -m imx477

install support software:
sudo apt update; sudo apt-install -y v4l-utils

v4l2-ctl -d /dev/video0 --all
ls /dev/video*
dmesg | grep -E "imx477|imx219|arducam"
v4l2-ctl --list-devices

NVIDIA Tegra Video Input Device (platform:tegra-camrtc-ca):
	/dev/media0

vi-output, imx477 10-001a (platform:tegra-capture-vi:2):
	/dev/video0

install Gstreamer:
sudo apt-get install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-bad1.0-dev gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio


Run the following command to capture a video, and check the recoded video file ( output.mp4 ):
gst-launch-1.0 nvarguscamerasrc ! \
     'video/x-raw(memory:NVMM), width=(int)1920, height=(int)1080, \
     format=(string)NV12, framerate=(fraction)30/1' ! nvvidconv ! \
     video/x-raw, format=I420 ! x264enc ! \
     h264parse ! qtmux ! filesink \
     location=output.mp4 -e
=============================================================================================
Waveshare 5.5" touch screen Amoled
sudo apt install xserver-xorg-input-synaptics
sudo mkdir /etc/X11/xorg.conf.d
sudo cp /usr/share/X11/xorg.conf.d/40-libinput.conf /etc/X11/xorg.conf.d/
sudo nano /etc/X11/xorg.conf.d/40-libinput.conf

Find the part of the touchscreen, add the following statement inside, Between MatchIsTouchscreen "ON" and MatchDevicePath, and then save the file. ctrl-O, enter then ctrx-x
Option "CalibrationMatrix" "0 1 0 -1 0 1 0 0 1"

sudo reboot

Complete the above steps for a 90 degree rotation.

Note:

0 degrees of rotation parameters:  Option "CalibrationMatrix" "1 0 0 0 1 0 0 0 1"

90 degrees of rotation parameters:  Option "CalibrationMatrix" "0 1 0 0 -1 1 0 0 1"

180 degrees of rotation parameters:  Option "CalibrationMatrix" "-1 0 1 0 -1 1 0 0 1"

270 degrees of rotation parameters:  Option "CalibrationMatrix" "0-1 1 1 0 0 0 0 1" 
 ---------------------------------------------------------------------    
 Backlight control

You can control the backlight brightness by long pressing ON/OFF on the back of the LCD.

Note: If you increase the brightness, it may cause insufficient power of the LCD by getting power through the USB interface. To solve this problem, you can input 5V/2A power through the Power interface on the back of the LCD. 

==========================================================================================
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
==========================================================================================

***other installs that need work***

FAST_LIO_LC:

You must remove Fast_Lio and ws_livox from the src folder before build this version
dependencies:
ROS
sudo apt-get install -y ros-noetic-navigation
sudo apt-get install -y ros-noetic-robot-localization
sudo apt-get install -y ros-noetic-robot-state-publisher
sudo apt-get install -q -y openssh-client
sudo apt-get install -q -y python3-pip
sudo apt-get install -q -y ros-noetic-cv-bridge

*Prerequisites*

CERES Solver
gtsam (see staticmaping)
cd ~/catkin_ws/src
git clone https://github.com/yanliang-wang/FAST_LIO_LC.git

fix issue with opencv in PGO/src/scanRegistration.cpp
 change #include <opencv/cv.h> to:
#include <opencv2/opencv.hpp>

cd ..
catkin_make -j1

add these lines to bash.rc if not already there:
export PATH=/usr/local/cuda-11.7/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda-11.7/lib64:$LD_LIBRARY_PATH
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
source ~/catkin_ws/src/ws_livox/devel/setup.bash
------------------------------------------------------

roslaunch ouster_ros replay.launch viz:=false metadata:=/home/lidar/pointclouds/ouster1.json bag_file:=/home/lidar/pointclouds/three.bag
roslaunch fast_lio mapping_ouster32.launch
roslaunch aloam_velodyne fastlio_velodyne_VLP_16.launch

====================================================================================

FastLivo:

Fix for Sophus so2.cpp by commenting out the two lines
  unit_complex_.real() = 1.;
  unit_complex_.imag() = 0.;
and replacing with:
unit_complex_ = std::complex<double>(1,0);

or maybe 
   unit_complex_.real(1.);
   unit_complex_.imag(0.); 
  
====================================================================================
docker:
source devel/setup.sh

roslaunch ouster_ros replay.launch viz:=true metadata:=/data/ouster1.json bag_file:=/data/three.bag
roslaunch ouster_ros replay.launch viz:=true metadata:=${PWD}/data/ouster1.json bag_file:=${PWD}/data/three.bag
sudo 
============================================================================================

Point Lio:

git clone https://github.com/hku-mars/Point-LIO.git
    cd Point-LIO
    git submodule update --init
    cd ../..
    catkin_make
    source devel/setup.bash
    
    
    For frame [aft_mapped]: Frame [aft_mapped] does not exist
    
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++==
Ouster Topics non legacy mode. Ran with noviz true:
/os_node/imu_packets
/os_node/lidar_packets
/ouster/imu
/ouster/imu_packets
/ouster/lidar_packets
/ouster/nearir_image
/ouster/os_nodelet_mgr/bond
/ouster/points
/ouster/range_image
/ouster/reflec_image
/ouster/signal_image
/rosout
/rosout_agg
/tf   
__________________________________________________________________________________
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
____________________________________________________________________________________

new ouster ros stuff
     

Recording Data

To record raw sensor output you may use the provided record.launch file as follows:

roslaunch ouster_ros record.launch      \
    sensor_hostname:=<sensor hostname>  \
    metadata:=<json file name>          \
    bag_file:=<optional bag file name>

This will connect to the specified sensor, write the sensor metadata to a file and start recording imu and lidar packets to the specified bag_file once the sensor is connected.

It is necessary that you provide a name for the metadata file and maintain this file along with the recorded bag_file otherwise you won't be able to play the file correctly.

If no bag_file is specified then a name will be generated based on the current date/time.

By default ROS saves all files to $ROS_HOME, if you want to have these files saved in the current directory, simply give the absolute path to each file. For example:

roslaunch ouster_ros record.launch      \
    sensor_hostname:=169.254.201.29  \
    metadata:=/home/lidar/pointclouds/my_os0.json  \
    bag_file:=/home/lidar/pointclouds/OS0_32



Alternatively, you may connect to the sensor using the roslaunch ouster_ros sensor.launch .. command and then use the rosbag command in a separate terminal to start recording lidar packets at any time using the following command:

rosbag record /ouster/imu_packets /ouster/lidar_packets

For more information on rosbag functionality refer to rosbag record.

Warning

When recording a bag file directly via the rosbag record, you need to save the metadata information of the sensor you are connected to. This can be achieved by supplying a path to the metadata argument of the sensor.launch. You will need the metadata file information to properly replay the recorded bag file.


Please note all commands in the TCP API are in planned obsolescence. Please consider using the HTTP API instead. The TCP API is subject to deprecation shortly. Please refer to Firmware User Manual for more information or reach out to Ouster Customer support.



------------------------------------

claude app with ouster and livox
Verification

 1. Start the app: python3 backpack_scanner.py
 2. Open http://<backpack-ip>:5000 on phone browser
 3. Select Ouster, press Start — verify:
   - Ouster driver launches (check /ouster/points topic)
   - FAST-LIO launches with rviz showing the map building
   - Bag recording starts
   - Miranda motor spins (if connected)
 4. Press Stop — verify:
   - Motor stops
   - Ouster goes to standby
   - PCD file saved in ~/pointclouds/<timestamp>/scan.pcd
   - Bag files present in ~/pointclouds/<timestamp>/bag/
 5. Repeat with Livox selection
 6. Test from phone browser over WiFi



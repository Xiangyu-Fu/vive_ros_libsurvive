# HTC VIVE ROS [still in development]
This is a ROS package for HTC VIVE. It is based on the [libsurvive](https://github.com/cntools/libsurvive) library.

![](images/Screenshot%20from%202024-10-27%2015-31-07.png)

### Advantages:
- No need for SteamVR for running
- simple setup
- ...

## Installation
1.install libsurvive
use the command
```bash
./install
```
OR
```bash
cd
mkdir repos
cd repos
git clone https://github.com/cntools/libsurvive.git --recursive
cd libsurvive
sudo cp ./useful_files/81-vive.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo apt update && sudo apt install build-essential zlib1g-dev libx11-dev libusb-1.0-0-dev freeglut3-dev liblapacke-dev libopenblas-dev libatlas-base-dev cmake
make
```

2.clone this repository
```bash
cd ~/your_ws/src
git clone
cd ..
catkin build
```

## Notice
If you use [Dongle](https://tundra-labs.com/?srsltid=AfmBOopHBek6D9z68HtG2R4wa5UTrAS4LQfLOqxPkXbbKOc_oVi--y93), please pair the tracker to the Dongle using SteamVR first, then you can use the tracker without the HTC VIVE.

## Usage
1. Run the libsurvive node
```bash
roslaunch vive_ros_libsurvive libsurvive_pub.launch RVIZ_ON:=True PATH_ON:=True
```

2. Troubleshooting
if you find the tracker auto calibrating to some strange position, please use
```bash
./rm_cal.sh
```

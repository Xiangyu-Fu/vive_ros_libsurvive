# HTC VIVE ROS [still in development]
This is a ROS package for HTC VIVE. It is based on the [libsurvive](https://github.com/cntools/libsurvive) library.

TODOs:
- [ ] add  RViz visualization
- [ ] add more publish topics


### Advantages:
- No need for SteamVR
- simple setup
- ...

## Installation
1.install libsurvive
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

## Usage
1. Run the libsurvive node
```bash
rosrun vive_ros_libsurvive libsurvive_pub_node
```


Current:

process[vive_ros_libsurvive-2]: started with pid [270194]
[ INFO] [1729842810.977103263]: Starting Libsurvive publisher node...
[ INFO] [1729842810.985371130]: Publishing pose and button events...
Info: Loaded drivers: GlobalSceneSolver, HTCVive
Info: Adding tracked object KN0 from HTC
Info: Device KN0 has watchman FW version 1637337510 and FPGA version 538/7/2; named '                   WMBUILD-W64$'. Hardware id 0xf1040009 Board rev: 3 (len 56)
Warning: The detected version for device KN0 is 1637337510; the latest that is verified to work is 1632556731. You may have to upgrade libsurvive to support this device.
Info: Detected LH gen 2 system.
Warning: Configuration was valid for gen 1; resetting BSD positions and OOTX
Info: LightcapMode (KN0) 1 -> 2 (ff)
^C[vive_ros_libsurvive-2] killing on exit
Info: MPFIT stats for KN0:
Info:   seed runs         0 / 0
Info:   error failures    0
[rosout-1] killing on exit
[master] killing on exit
shutting down processing monitor...
... shutting down processing monitor complete
done
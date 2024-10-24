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
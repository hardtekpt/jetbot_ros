# Jetbot_ros

This repository contains a **ROS** package made to run on a jetbot that exposes motor control, OLED screen control and camera access on ROS topics.

## Setup

This package was tested on **JetPack 4.6.1** with **L4T 32.7.1** running on an NVIDIA Jetson Nano (4Gb) with the **sparkfun qwiic pHat** and the **LI-IMX219-MIPI-FF-NANO-H136** camera ([sparkfun jetbot](https://www.sparkfun.com/products/18486)). ROS melodic was installed using [this](https://github.com/jetsonhacks/installROS) convenience script.

### Camera Dependencies

To access the CSI camera these packages are required:

- [dusty-nv/jetson-utils](https://github.com/dusty-nv/jetson-utils)
- [rt-net/jetson_nano_cuda_csi_cam](https://github.com/rt-net/jetson_nano_cuda_csi_cam_ros)

Also, before installing the **Jetson-utils** package these packages were installed:

```bash
sudo apt-get install -y --no-install-recommends \
    dialog \
    libglew-dev \
    glew-utils \
    gstreamer1.0-libav \
    gstreamer1.0-nice \
    libgstreamer1.0-dev \
    libgstrtspserver-1.0-dev \
    libglib2.0-dev \
    libsoup2.4-dev \
    libjson-glib-dev \
    python3-pip \
    python3-packaging \
    qtbase5-dev
```

## Using this repository

### Download and install

Clone the repository into the workspace and build it.

```bash
cd ~/catkin_ws/src
git clone https://github.com/hardtekpt/jetbot_ros
cd ~/catkin_ws
catkin_make
```

Install the motor and OLED screen drivers.

```bash
sudo python2 -m pip install sparkfun-qwiic-i2c sparkfun-qwiic-oled-base
sudo python2 -m pip install sparkfun-qwiic-scmd sparkfun-qwiic-micro-oled
```

### Usage

Two main launch files are available. The ```run.launch``` file can be used to launch the jetbot in either simulation or with the real vehicle:

```bash
cd ~/catkin_ws/src
source devel/setup.bash
roslaunch jetbot_ros run.launch sim:=<true,false>
```

This launch file will make the motors, screen and camera available through ROS topics. To get more information use ```rostopic list``` and ```rostopic info```. Additionally, it is possible to control the jetbot via teleop by running in a new terminal:

```bash
cd ~/catkin_ws/src
source devel/setup.bash
roslaunch jetbot_ros teleop.launch sim:=<true,false>
```

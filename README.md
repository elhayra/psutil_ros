# psutil_ros

ROS Wrapper for psutil. Get CPU, virtual memory and tempartures easily

## Installation

1. This package is based on psutil. So before using this package, install it:

```
pip install psutil
```
2. Download this package into your catkin workspace
3. Compile your catkin workspace in order to generate psutil_ros ROS messages

## Usage

1. Optional - open the executable node file (psutil_ros_node.py) and edit the parameters on the top according to your needs.

2. Run the package node:

```
rosrun psutil_ros psutil_ros_node.py
```

* You can find psutil documentation here: http://psutil.readthedocs.io/en/latest/#
* If you want me to expose more psutil features to ROS, open an issue


## Package, Installation, Running

### Debian Packaging for Ubuntu Bionic / ROS Melodic

Generate a debian package for easy installation using `apt`.


```
sudo apt update && sudo apt-get install -y debhelper
debian/rules clean && debian/rules binary
```

Should finish with the following line identifying the available package:

```
...
dpkg-deb: building package 'ros-melodic-psutil-ros' in './ros-melodic-psutil-ros_0.2.0-0bionic_amd64.deb'.
...
```

Install the file into Ubuntu:

```
dpkg -i ros-melodic-psutil-ros_0.2.0-0bionic_amd64.deb
```

Run with roslaunch:
```
roslaunch psutil_ros psutil_ros.launch
```

Echo the CPU topic, for example:
```
rostopic echo /psutil_ros/cpu -c
```

Expect to see similar:

```
usage: 2.59999990463
physical_cores: 6
cores: 12
min_freq: 800
max_freq: 4500
current_freq: 800
---
```
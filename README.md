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

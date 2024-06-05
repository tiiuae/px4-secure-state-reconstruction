# Installation

*NOTE* This has only been tested with ROS 2 Humble. Any other version of ROS may not be compatible. Use with caution.

## Pre-requisites

ROS 2 Humble Hawksbill

PX4-Autopilot (and dependencies)

## Build

Create a ROS 2 Workspace

```bash
mkdir ros2_ws && cd ros2_ws
```

Clone and build the workspace

```bash
git clone --recursive https://github.com/juniorsundar-tii/px4-secure-state-reconstruction.git src
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/local_setup.bash
```

# Running

*TODO*

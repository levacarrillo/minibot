# Minibot - Mobile Robot

ToDo: Description

**IMPORTANT:** This repo contains several modules to run the minibot and its simulation.

## Prerrequisites

1. ROS 2 Jazzy Jalisco or later
2. Dependencies
    1. `xterm`
    2. `python3`
    3. `navigation2`
    4. `nav2-bringup`

## Build

1. Clone inside your `ros2_ws/src/` directory
2. Run the following commands
```
colcon build
source /opt/ros/${ROS_DISTRO}/setup.bash
source install/setup.bash

## Run with
ros2 launch motion_planner motion_planner.launch.py
```

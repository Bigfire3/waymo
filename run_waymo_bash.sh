#!/bin/bash
colcon build --packages-select waymo
source /opt/ros/humble/setup.bash
source /home/lubun/coding/ros2ws/src/install/setup.bash
ros2 launch waymo waymo_launch.py


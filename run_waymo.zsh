#!/bin/zsh

colcon build --packages-select waymo

source ~/ros2_ws/install/setup.zsh

ros2 launch waymo waymo_launch.py
#!/bin/bash

colcon build --packages-select waymo

source ~/ros2_ws/install/setup.bash

ros2 launch waymo waymo_launch.py


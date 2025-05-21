#!/bin/bash
cd ~/ros2_ws || { echo "Fehler: ros2_ws-Verzeichnis nicht gefunden."; exit 1; }

colcon build --packages-select waymo

source /opt/ros/humble/setup.bash

source ~/ros2_ws/install/local_setup.bash

ros2 launch waymo waymo_launch.py
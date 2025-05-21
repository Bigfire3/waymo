# Robotics Project Summer Semester 2025: Team Waymo

This repository contains ROS2 nodes in Python for basic lane following, frontal obstacle detection, automated obstacle passing maneuvers, and automated parking maneuvers inspired by autonomous driving functions. The package is designed for ROS2 Humble.

## Easy Usage

You can start the package and the keyboard handler with the following steps:

1. Open two terminal windows.  
   Then start the keyboard handler with these commands in one of the terminals:

   ```bash
   cd ~/ros2_ws/src/waymo
   source ~/ros2_ws/install/setup.bash
   ./run_keyboard_handler_node.bash
   ```

2. Now start all required nodes using these commands in the other terminal window.
   After running the script, make sure to switch the focus towards the terminal with the running keyboard handler.

   ```bash
   cd ~/ros2_ws/src/waymo
   source ~ros2_ws/install/setup.bash
   ./run_waymo.bash
   ```

## Projekt Details

You can find detailed descriptions in the [wiki section](https://github.com/Bigfire3/waymo/wiki).

You can access the pages below to get a better understanding how the waymo package works:

- ### [Milestones](https://github.com/Bigfire3/waymo/wiki/Project-Milestones)

- ### [Usage](https://github.com/Bigfire3/waymo/wiki/Usage)

  - [Prerequisites / Installation](https://github.com/Bigfire3/waymo/wiki/Usage#prerequisites--installation)
  - [Workspace Setup](https://github.com/Bigfire3/waymo/wiki/Usage#workspace-setup)
  - [Running the Package](https://github.com/Bigfire3/waymo/wiki/Usage#running-the-package)
  - [Running the Keyboard Handler Node (Manual Pause & Debug Toggle)](https://github.com/Bigfire3/waymo/wiki/Usage#running-the-keyboard-handler-node-manual-pause--debug-toggle)
  - [Modifying Parameters](https://github.com/Bigfire3/waymo/wiki/Usage#modifying-parameters)
  - [Testing Functionality / Monitoring Topics](https://github.com/Bigfire3/waymo/wiki/Usage#testing-functionality--monitoring-topics)

- ### [Node Descriptions](https://github.com/Bigfire3/waymo/wiki/Node-Descriptions)

  - [lane_detection_node](https://github.com/Bigfire3/waymo/wiki/Node-Descriptions#lane_detection_node-view-code)
  - [obstacle_detection_node](https://github.com/Bigfire3/waymo/wiki/Node-Descriptions#obstacle_detection_node-view-code)
  - [passing_obstacle_node](https://github.com/Bigfire3/waymo/wiki/Node-Descriptions#passing_obstacle_node-view-code)
  - [traffic_light_detection_node](https://github.com/Bigfire3/waymo/wiki/Node-Descriptions#traffic_light_detection_node-view-code)
  - [sign_detection_node](https://github.com/Bigfire3/waymo/wiki/Node-Descriptions#sign_detection_node-view-code)
  - [parking_node](https://github.com/Bigfire3/waymo/wiki/Node-Descriptions#parking_node-view-code)
  - [state_manager_node](https://github.com/Bigfire3/waymo/wiki/Node-Descriptions#state_manager_node-view-code)
  - [gui_debug_node](https://github.com/Bigfire3/waymo/wiki/Node-Descriptions#gui_debug_node-view-code)
  - [keyboard_handler_node](https://github.com/Bigfire3/waymo/wiki/Node-Descriptions#keyboard_handler_nodeview-code)

- ### [Parameter & Key Constants](https://github.com/Bigfire3/waymo/wiki/Parameter-&-Key-Constants)

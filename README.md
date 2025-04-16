## Robotics Project Summer Semester 2025: Team Waymo

1. Step:

* follow street
* react to obstacles (stop->continue/ignore)

2. Step:

* react to traffic light

3. Step:

* pass obstacles on other street side
* ignore obstacles next to street

4. Step:

* recognize parking sign
* park in/out

5. Step:

* follow street with false road markings

6. Step:

* handle intersections

# Task 1: Lane Following and Obstacle Detection

This repository contains ROS2 nodes in Python for basic lane and obstacle detection, inspired by autonomous driving functions. The package is designed for ROS2 Humble.

---

## Node Descriptions

This package consists of several ROS2 nodes that work together:

### `lane_detection_node` ([View Code](https://github.com/Bigfire3/waymo/blob/development/waymo/lane_detection_node.py))

* **Functionality:**
    * Subscribes to raw image data from the topic `/image_raw/compressed`.
    * Uses OpenCV for image processing to detect lane lines (utilizes helper modules `lane.py` and `edge_detection.py`).
    * Applies a perspective transformation to get a top-down view of the lane.
    * Filters detected lines based on their thickness.
    * Calculates the lateral offset of the robot from the lane center.
    * Publishes the annotated image (with detected lines and ROI) on the topic `/lane/image_annotated`.
    * Publishes the calculated offset from the lane center on the topic `/lane/center_offset` (type `std_msgs/Float64`).
* **Dependencies:** OpenCV, NumPy, cv_bridge.

### `obstacle_detection_node` ([View Code](https://github.com/Bigfire3/waymo/blob/development/waymo/obstacle_detection_node.py))

* **Functionality:**
    * Subscribes to laser scan data from the topic `/scan` (type `sensor_msgs/LaserScan`).
    * Checks distance measurements within a configurable angular range in front of the robot.
    * Detects if an obstacle is closer than a defined stopping distance.
    * Publishes a boolean value on the topic `/obstacle/blocked` (type `std_msgs/Bool`), which is `true` if an obstacle is too close.
* **Dependencies:** -

### `state_manager_node` ([View Code](https://github.com/Bigfire3/waymo/blob/development/waymo/state_manager_node.py))

* **Functionality:**
    * Acts as a state machine for the robot.
    * Subscribes to the obstacle status from `/obstacle/blocked` and the lane offset from `/lane/center_offset`.
    * Determines the current state of the robot (e.g., `WAYMO_STARTED`, `STOPPED`, `FOLLOW_LANE`).
    * Publishes the current state on the topic `/robot/state` (type `std_msgs/String`).
    * Sends velocity commands (forward and rotation) to the robot via the topic `/cmd_vel` (type `geometry_msgs/Twist`), based on the state and lane offset.
* **Dependencies:** -

### `gui_debug_node` ([View Code](https://github.com/Bigfire3/waymo/blob/development/waymo/gui_debug_node.py))

* **Functionality:**
    * Subscribes to the annotated image from `/lane/image_annotated` and the robot state from `/robot/state`.
    * Displays the annotated camera image in an OpenCV window named `Kamerabild Debug`.
    * Prints changes in the robot's state to the console.
    * Primarily serves for debugging and visualization.
* **Dependencies:** OpenCV, cv_bridge.

---

## Parameters

Several nodes allow configuration via ROS2 parameters:

### `lane_detection_node`

* **Thresholding & Filtering:**
    * `block_size`: Size of the neighborhood area for adaptive thresholding (Default: `11`).
    * `c_value`: Constant subtracted from the mean or weighted mean (Default: `20`).
    * `min_thickness`: Minimum average thickness (area/perimeter) for valid line contours (Default: `2.5`).
    * `max_thickness`: Maximum average thickness for valid line contours (Default: `5.0`).
* **Center Calculation:**
    * `center_factor`: Factor for scaling the pixel offset into a control command (Default: `0.025`).
* **Perspective Transform ROI (Region of Interest):**
    * `roi_top_left_w`, `roi_top_left_h`: Relative coordinates (width, height) of the top-left corner of the ROI (Default: `0.2`, `0.65`).
    * `roi_top_right_w`, `roi_top_right_h`: Top-right corner (Default: `0.8`, `0.65`).
    * `roi_bottom_left_w`, `roi_bottom_left_h`: Bottom-left corner (Default: `0.0`, `1.0`).
    * `roi_bottom_right_w`, `roi_bottom_right_h`: Bottom-right corner (Default: `1.0`, `1.0`).
    * `desired_roi_padding_factor`: Relative distance from the edge in the transformed image (Default: `0.25`).

### `obstacle_detection_node`

* `distance_to_stop`: Minimum distance (in meters) at which an obstacle is detected as blocking (Default: `0.25`).
* `angle_range_min`: Minimum angle (in radians, relative to the robot's front) for obstacle detection (Default: `-0.2 rad`, approx. -11.5°).
* `angle_range_max`: Maximum angle (in radians) for obstacle detection (Default: `0.2 rad`, approx. +11.5°).

### `state_manager_node`

* `drivingspeed`: Base speed for driving (Default: `0.15 m/s`). *`Note: The code seems to partially hardcode this value to 0.15, check the implementation.`*

---

## Usage

### Prerequisites / Installation

1.  **ROS2 Humble:** Ensure ROS2 Humble is installed.
    * [Ubuntu Installation Guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

2.  **Colcon:** Install the build tool Colcon.
    ```bash
    sudo apt update
    sudo apt install python3-colcon-common-extensions
    ```

3.  **Python Dependencies (OpenCV, cv_bridge):** These are necessary for image processing and the GUI.
    * **cv_bridge:** Essential for converting between ROS image messages and OpenCV images.
    * **OpenCV:** Required for image processing algorithms.

    First, try installing via ROS packages (recommended on Ubuntu):
    ```bash
    sudo apt update
    sudo apt install ros-humble-cv-bridge python3-opencv
    ```

### Workspace Setup

1.  Create a ROS2 workspace (if not already done):
    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    ```

2.  Clone the waymo package from GitHub into your src folder:
    * with ssh:
        ```bash
        git clone git@github.com:Bigfire3/waymo.git
        ```
    * with https:
        ```bash
        git clone [https://github.com/Bigfire3/waymo.git](https://github.com/Bigfire3/waymo.git)
        ```

3.  **Build the package:**
    ```bash
    cd ~/ros2_ws
    colcon build --packages-select waymo
    ```

4.  **Source the workspace:**
    ```bash
    source ~/ros2_ws/install/setup.bash
    ```
    *`Add this command to your .bashrc (or .zshrc) to avoid running it manually every time.`*

### Running the Package

The nodes can be most easily started together using the provided launch file:

1.  **Source the workspace** (if not already done):
    ```bash
    source ~/ros2_ws/install/setup.bash
    ```

2.  **Start the launch file:**
    ```bash
    ros2 launch waymo waymo_launch.py
    ```
    This will start `lane_detection_node`, `obstacle_detection_node`, `state_manager_node`, and `gui_debug_node`.

#### Alternative: `run_waymo.sh` Script

The package also includes a script `run_waymo.sh` that automates the build process (only for `waymo`) and launching.

1.  Ensure the script is executable:
    ```bash
    cd ~/ros2_ws/src/waymo
    chmod +x run_waymo.sh
    ```

2.  Run the script:
    ```bash
    ./run_waymo.sh
    ```
    *`(Paths in the script might need adjustment, e.g., the path to the workspace cd ~/ros2_ws and the source command)`*

### Modifying Parameters

Parameters can be changed at runtime via the ROS2 CLI. Open a new terminal (and source the workspace again: `source ~/ros2_ws/install/setup.bash`).

**Examples:**

* Change the stopping distance for obstacles:
    ```bash
    ros2 param set /obstacle_detection_node distance_to_stop 0.3
    ```
* Adjust the thresholding parameter for lane detection:
    ```bash
    ros2 param set /lane_detection_node c_value 25
    ```
* Change the base driving speed:
    ```bash
    ros2 param set /state_manager_node drivingspeed 0.1
    ```
* Adjust the ROI for lane detection:
    ```bash
    ros2 param set /lane_detection_node roi_top_left_h 0.6
    ```

### Testing Functionality / Monitoring Topics

Open new terminals (and source the workspace) to check the functionality:

1.  **`Display motion commands (/cmd_vel):`**
    ```bash
    ros2 topic echo /cmd_vel
    ```
    *`Shows the velocity commands sent by state_manager_node.`*

2.  **`Display robot state (/robot/state):`**
    ```bash
    ros2 topic echo /robot/state
    ```
    *`Shows the current state (WAYMO_STARTED, STOPPED, FOLLOW_LANE).`*

3.  **`Display obstacle status (/obstacle/blocked):`**
    ```bash
    ros2 topic echo /obstacle/blocked
    ```
    *`Shows true or false, depending on whether an obstacle was detected.`*

4.  **`Display lane offset (/lane/center_offset):`**
    ```bash
    ros2 topic echo /lane/center_offset
    ```
    *Shows the calculated lateral offset from the lane center.*

5.  **Annotated Image (GUI):**
    * If `gui_debug_node` is running and the GUI libraries are functioning correctly, a window named `Kamerabild Debug` should appear, showing the processed camera image.

6.  **Visualize Node Graph:**
    ```bash
    rqt_graph
    ```
    *`Shows how the nodes are connected and which topics they use (requires rqt).`*

---

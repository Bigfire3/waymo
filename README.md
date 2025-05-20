# Robotics Project Summer Semester 2025: Team Waymo

## **Project Milestones:**

1. **[DONE]** Basic Lane Following & Obstacle Reaction
    * Follow detected lane lines.
    * React to frontal obstacles (stop).
2. **[DONE]** Traffic Light Detection (Color-Based)
    * Detects a specific predefined color (acting as 'Stop' signal).
    * Stops if configured color is detected initially, proceeds otherwise.
3. **[DONE]** Obstacle Passing & Lateral Obstacle Handling
    * Pass stationary obstacles detected on the current lane by switching to the adjacent lane.
    * Ignore obstacles detected far off to the side of the road (handled implicitly by detection ranges).
4. **[DONE]** Parking Maneuver
    * Recognize a designated parking sign. (Handled by `sign_detection_node` and utilized by `parking_node`)
    * Execute an automated parking maneuver (e.g., parallel parking). (Handled by `parking_node`)
    * Execute an automated maneuver to leave the parking spot. (Handled by `parking_node`)
5. **[WORK IN PROGRESS]** Robust Lane Following
    * Handle faded, incorrect, or missing lane markings.
6. **[WORK IN PROGRESS]** Intersection Handling
    * Detect intersections.
    * Execute turns (e.g., based on predefined route or commands).
7. **[DONE]** Manual Pause Functionality
    * Added ability to pause and resume operation via keyboard command.
8. **[DONE]** Debug Canva Functionality
    * Added functionality to toggle the Debug Canva via the keyboard handler.

---

This repository contains ROS2 nodes in Python for basic lane following, frontal obstacle detection, automated obstacle passing maneuvers, and automated parking maneuvers inspired by autonomous driving functions. The package is designed for ROS2 Humble.

---

## **Node Descriptions**

This package consists of several ROS2 nodes that work together:

### `lane_detection_node` ([View Code](/waymo/lane_detection_node.py))

* **Functionality:**

  * Subscribes to raw image data (`/image_raw/compressed`).
  * Uses OpenCV for image processing to detect lane lines (`lane.py`, `edge_detection.py`).
  * Applies perspective transformation for a top-down view.
  * Filters lines based on thickness.
  * Calculates the robot's lateral offset from the lane center.
  * Publishes annotated image (`/lane/image_annotated`).
  * Publishes calculated center offset (`/lane/center_offset` - `std_msgs/Float64`).

* **Dependencies:** OpenCV, NumPy, cv_bridge.

### `obstacle_detection_node` ([View Code](/waymo/obstacle_detection_node.py))

* **Functionality:**
  * Subscribes to laser scan data (`/scan` - `sensor_msgs/LaserScan`).
  * Checks distance measurements in a frontal angular range.
  * Detects if an obstacle is closer than `distance_to_stop`.
  * Publishes obstacle status (`/obstacle/blocked` - `std_msgs/Bool`).
* **Dependencies:** -

### `passing_obstacle_node` ([View Code](/waymo/passing_obstacle_node.py))

* **Functionality:**
  * Handles the automated obstacle passing maneuver.
  * Activated by `state_manager_node` when state becomes `PASSING_OBSTACLE`.
  * Takes exclusive control of `/cmd_vel` during the maneuver.
  * Subscribes to odometry (`/odom`) for orientation (yaw).
  * Subscribes to laser scan (`/scan`) to check if the side is clear of the obstacle.
  * Subscribes to lane offset (`/lane/center_offset`) to perform lane following on the adjacent lane during the pass.
  * Executes a multi-step sequence: 90° turn left, move sideways (following adjacent lane), 90° turn right, drive straight (following adjacent lane) until the side scan indicates the obstacle is passed, 90° turn right, move sideways back to original lane (following lane), 90° turn left.
  * Uses internal constants for maneuver parameters (turn angles, sideways distance, side scan angles, clear distance threshold).
  * Publishes a signal upon completion (`/obstacle/passed` - `std_msgs/Bool`) to notify `state_manager_node`.
* **Dependencies:** NumPy, SciPy (for quaternion conversion).

### `traffic_light_detection_node` ([View Code](/waymo/traffic_light_detection_node.py))

* **Functionality:**
  * Subscribes to raw image data (`/image_raw/compressed` - QoS: Best Effort).
  * Detects a specific predefined color range in the HSV color space (currently configured for dark pink/magenta: H=[160-180], S=[50-255], V=[50-255]).
  * Filters detections based on minimum blob area (currently 100 pixels).
  * Publishes the detection status on `/traffic_light` (`std_msgs/Bool` - QoS: Reliable).
  * **Note:** Publishes `False` if the configured color *is* detected (interpreted as "Stop") and `True` if the color is *not* detected (interpreted as "Go").
  * Includes optional OpenCV debug windows (`show_debug_windows` flag in code).
* **Dependencies:** OpenCV, NumPy.

### `sign_detection_node` ([View Code](/waymo/sign_detection_node.py))

* **Functionality:**
  * Subscribes to raw image data (`/image_raw/compressed`).
  * Loads traffic sign template images (e.g., for parking signs) from the `waymo/traffic_signs` directory within the package.
  * Uses OpenCV's template matching (`cv2.matchTemplate`) to detect configured signs in the camera feed.
  * Publishes a string message (e.g., "parking_sign_detected") on the `/sign` topic when a parking sign is identified.
  * Optionally publishes debug images:
    * `/debug/cam/template_matching` (CompressedImage): Shows the result of the template matching process.
    * `/debug/cam/binary_sign` (CompressedImage): Shows the binarized image used for sign detection.
* **Dependencies:** OpenCV, NumPy, cv_bridge.

### `parking_node` ([View Code](/waymo/parking_node.py))

* **Functionality:**
  * Manages the automated parking and unparking maneuver.
  * Activated by the `state_manager_node` when the robot's state changes to `PARKING`.
  * Takes exclusive control over `/cmd_vel` during the parking sequence.
  * Subscribes to:
    * `/robot/state` (`std_msgs/String`): To monitor the overall robot state.
    * `/scan` (`sensor_msgs/LaserScan`): For detecting the initial parking spot opening using side-mounted laser scan checks and ensuring the area is clear during movements.
    * `/odom` (`nav_msgs/Odometry`): To get the current orientation (yaw) for precise turns.
    * `/lane/center_offset` (`std_msgs/Float64`): Used for lane following when driving to potential parking spots.
    * Executes a multi-phase parking sequence:
      1. `DRIVING_FOR_INITIAL_LASER_SCAN`: Drives forward slowly, following the lane, until an initial laser scan indicates a potential area (e.g., passed the visual sign).
      2. `INITIAL_SIGN_STOPPING` & `INITIAL_SIGN_WAITING`: Stops briefly after the initial laser detection.
      3. `DRIVING_TO_SCAN_POINT`: Drives a predefined distance to the first (or next) potential parking spot, following the lane.
      4. `STOPPING_BEFORE_SCAN` & `SCANNING_FOR_SPOT`: Stops and uses laser scans to check if the adjacent parking spot is clear. If not clear, and `MAX_PARKING_ATTEMPTS` not reached, it moves to the next potential spot.
      5. `TURNING_RIGHT_FOR_PARKING`: Executes a 90° right turn towards the spot.
      6. `MOVING_INTO_SPOT`: Drives a short distance forward into the parking spot.
      7. `TURNING_LEFT_IN_SPOT_TO_PARK`: Executes a 90° left turn to align parallel in the spot.
      8. `WAITING_IN_SPOT`: Pauses for a defined duration in the parked position.
      9. `TURNING_LEFT_IN_SPOT_TO_GET_OUT`: Executes a 90° left turn to prepare for exiting.
      10. `MOVING_OUT_OF_SPOT`: Drives a short distance forward to leave the spot.
      11. `TURNING_RIGHT_FOR_LANE_FOLLOWING`: Executes a 90° right turn to align with the lane again.
      12. `MANEUVER_COMPLETE`: Signals completion.
  * Publishes a `std_msgs/Bool` message on `/parking/finished` topic (True when the full maneuver is completed or aborted after max attempts).
  * **Dependencies:** NumPy, SciPy (for quaternion to Euler conversion).

### `state_manager_node` ([View Code](/waymo/state_manager_node.py))

* **Functionality:**
  * Acts as the main state machine. Key states include: `STATE_STOPPED_AT_TRAFFIC_LIGHT`, `STATE_FOLLOW_LANE`, `STATE_STOPPED_AT_OBSTACLE`, `STATE_PASSING_OBSTACLE`, `MANUAL_PAUSE`.
  * Starts in the `STATE_STOPPED_AT_TRAFFIC_LIGHT` state.
  * Subscribes to:
    * Obstacle status (`/obstacle/blocked` - Bool, QoS: Best Effort).
    * Lane offset (`/lane/center_offset` - Float64, QoS: Best Effort).
    * Obstacle passed signal (`/obstacle/passed` - Bool, QoS: Reliable).
    * Traffic light status (`/traffic_light` - Bool, QoS: Reliable).
    * Keyboard commands (`/keyboard_command` - String, QoS: Reliable).
  * Determines the robot's current operational state based on inputs and internal logic.
  * **Initial Traffic Light Check:** Waits in `STATE_STOPPED_AT_TRAFFIC_LIGHT` until the `/traffic_light` topic sends `True` (indicating the configured stop-color is *not* detected). Once `True` is received, the initial traffic light check is considered done (subscription is destroyed).
  * **Manual Pause:** Handles the `toggle_pause` command from `/keyboard_command` to enter/exit the `MANUAL_PAUSE` state. Stops the robot immediately upon pausing. Transitions directly to `STATE_FOLLOW_LANE` upon resuming.
  * Publishes the current state (`/robot/state` - String, QoS: Reliable).
  * Sends velocity commands (`/cmd_vel` - `geometry_msgs/Twist`) based on state (e.g., lane following uses P-control on offset with a 200Hz timer).
  * Relinquishes control of `/cmd_vel` during the `PASSING_OBSTACLE` state.
* **Dependencies:** NumPy.

### `gui_debug_node` ([View Code](/waymo/gui_debug_node.py))

* **Functionality:**
  * Subscribes to annotated image (`/lane/image_annotated`) and robot state (`/robot/state`).
  * Displays the annotated camera image in an OpenCV window.
  * Prints changes in the robot's state (including `PASSING_OBSTACLE`) to the console.
  * Primarily serves for debugging and visualization.
* **Dependencies:** OpenCV, cv_bridge.

### `keyboard_handler_node`([View Code](/waymo/keyboard_handler_node.py))

* **Functionality:**
  * Listens for keyboard input in its terminal (requires interactive TTY).
  * Detects the 's' key press.
  * Publishes a `"toggle_pause"` message (`std_msgs/String`, Reliable QoS) on the `/keyboard_command` topic.
  * Used to remotely toggle pause/resume for the `state_manager_node`.
  * Restores terminal settings on exit.

* **Usage:**
  * **Important:** Must be run manually in a **separate, interactive terminal**
  * In the new terminal, source your workspace: `source install/setup.bash` (or `.zsh`).
  * Run the node: `ros2 run waymo keyboard_handler_node`
  * Ensure the terminal running the node has keyboard focus.
  * Press `s` to toggle the pause state (`MANUAL_PAUSE` <-> `FOLLOW_LANE`).
  * Press `Ctrl+C` in the handler's terminal to stop it cleanly.

---

## Parameters & Key Constants

Nodes can be configured via ROS2 parameters or internal constants:

### `lane_detection_node`

* See code for detailed parameter descriptions (`block_size`, `c_value`, `min_thickness`, `max_thickness`, `center_factor`, ROI parameters).
* `center_factor`: Crucial for scaling the lane offset into a steering command (Default: `0.03`). Tuning this affects lane following smoothness and ability to take curves.

### `obstacle_detection_node`

* `distance_to_stop`: Minimum frontal distance to trigger stop (Default: `0.25` m).
* `angle_range_min`/`max`: Defines the frontal detection cone (Defaults correspond to approx. +/- 11.5°).

### `passing_obstacle_node`

* Uses internal constants defined at the top of the file:
  * `LINEAR_SPEED`, `ANGULAR_SPEED`: Speeds during the maneuver.
  * `SIDEWAYS_DISTANCE`: Target lateral distance for lane change.
  * `SIDE_SCAN_ANGLE_MIN_DEG`, `SIDE_SCAN_ANGLE_MAX_DEG`: Angular range for checking if the obstacle is passed (Note: Current values might reflect left side based on user testing environment).
  * `SIDE_SCAN_CLEAR_DISTANCE`: Distance threshold to consider the side clear.
  * `WAIT_DURATION_BEFORE_CHECK`: Pause duration before starting the side check.

### `traffic_light_detection_node` (SpecificColorDetector)

* Uses **internal, hardcoded** values for detection (see code):
  * `HSV Color Range`: Currently targets dark pink/magenta ([160, 50, 50] to [180, 255, 255]). *Should be made ROS parameters for flexibility.*
  * `min_blob_area`: Minimum area in pixels for a detected color blob to be considered valid (Currently: 100). *Should be made a ROS parameter.*
  * `ROI`: Image cropping factor (Currently: Top 50%). *Should be made a ROS parameter.*
  * `show_debug_windows`: Boolean flag in code to enable/disable OpenCV debug windows (Default: `False`).

### `sign_detection_node`

* `publish_template_matching` (Boolean, Default: `True`): Enables/disables publishing the debug image for template matching.
* `publish_binary_sign` (Boolean, Default: `True`): Enables/disables publishing the debug image of the binarized frame used for detection.
* **Internal:** Templates are loaded from `share/waymo/traffic_signs/ *.png`. The `setup.py` ensures these are installed.

### `parking_node`

* Uses **internal, hardcoded constants** defined at the top of the `parking_node.py` file for maneuver parameters:
  * `PARKING_LINEAR_SPEED`: Uniform linear speed for most forward movements.
  * `MANEUVER_ANGULAR_SPEED_TURN`: Angular speed for turns.
  * `MAX_ANGULAR_Z_LANE_FOLLOW`: Maximum angular velocity when using lane offset for guidance.
  * Laser Scan Parameters for Spot Detection:
    * `INITIAL_SCAN_ANGLE_MIN/MAX_DEG`: Angular range for the first laser check (after visual sign).
    * `INITIAL_SPOT_DETECTION_DISTANCE`: Distance threshold for the first laser check.
    * `SPOT_SCAN_ANGLE_MIN/MAX_DEG`: Angular range for checking if a parking spot is clear.
    * `PARKING_SPOT_CLEAR_DISTANCE`: Distance threshold to consider a parking spot clear.
  * Durations and Distances:
    * `INITIAL_STOP_DURATION`: How long to wait after initial laser detection.
    * `DRIVE_TO_FIRST_SPOT_DURATION`: Time to drive to the first potential parking spot.
    * `DRIVE_TO_NEXT_SPOT_DURATION`: Time to drive to subsequent potential parking spots.
    * `STOP_BEFORE_SCAN_DURATION`: Brief stop before performing the detailed spot scan.
    * `PARKING_DURATION`: How long to wait once parked.
    * `MOVE_SPOT_DISTANCE`: Distance to move into/out of the parking spot.
  * Turn Control:
    * `TURN_ANGLE_90_DEG`: Target angle for 90-degree turns (can be slightly adjusted).
    * `GOAL_TOLERANCE_ANGLE_RAD`: Tolerance for completing a turn.
  * `MAX_PARKING_ATTEMPTS`: How many parking spots to try before aborting.

### `state_manager_node`

* `drivingspeed`: Base driving speed during `FOLLOW_LANE` (Default: `0.15` m/s).
* (Internal) `control_loop_period`: Timer frequency for control loop (Default: 0.005s -> 200 Hz).

---

## Usage

### Prerequisites / Installation

1. **ROS2 Humble:** Ensure ROS2 Humble is installed.
    * [Ubuntu Installation Guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

2. **Colcon:** Install the build tool Colcon.

    ```bash
    sudo apt update
    sudo apt install python3-colcon-common-extensions
    ```

3. **Python Dependencies:** OpenCV, cv_bridge, NumPy, SciPy.
    * Install via ROS packages if possible:

        ```bash
        sudo apt update
        sudo apt install ros-humble-cv-bridge python3-opencv python3-scipy python3-numpy
        ```

    * Alternatively, use pip within your environment (e.g., Conda): `pip install numpy scipy opencv-python` (cv_bridge usually comes with ROS).

---

### Workspace Setup

1. Create a ROS2 workspace (if not already done):

    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    ```

2. Clone the waymo package from GitHub into your src folder:
    * with ssh:

        ```bash
        git clone git@github.com:Bigfire3/waymo.git
        ```

    * with https:

        ```bash
        git clone https://github.com/Bigfire3/waymo.git
        ```

3. **Build the package:**

    ```bash
    cd ~/ros2_ws
    colcon build --packages-select waymo
    ```

4. **Source the workspace:**

    ```bash
    source ~/ros2_ws/install/setup.bash
    ```

    *`Add this command to your .bashrc (or .zshrc) to avoid running it manually every time.`*

---

### Running the Package

The nodes can be most easily started together using the provided launch file:

1. **Source the workspace** (if not already done):

    ```bash
    source ~/ros2_ws/install/setup.bash
    ```

2. **Start the launch file:**

    ```bash
    ros2 launch waymo waymo_launch.py
    ```

    This will start `lane_detection_node`, `obstacle_detection_node`, `state_manager_node`, and `gui_debug_node`.

#### Alternative: Shell Script

The package also includes several scripts that automate the build process and launch of `waymo`.
There is a script for bash, zsh and conda environment.

Here are the steps for bash:

1. Ensure the script is executable:

    ```bash
    cd ~/ros2_ws/src/waymo
    chmod +x run_waymo.bash
    ```

2. Run the script:

    ```bash
    ./run_waymo_bash.sh
    ```

    *`(Paths in the script might need adjustment, e.g., the path to the workspace cd ~/ros2_ws and the source command)`*

### Running the Keyboard Handler Node (Manual Pause & Debug Toggle)

The `keyboard_handler_node` allows you to manually pause/resume the robot's operation and toggle the debug image canvas.

* **Important:** This node must be run in a **separate, interactive terminal**.
* In the new terminal, source your workspace:
  * For bash:

    ```bash
    source ~/ros2_ws/install/setup.bash
    ```

  * For zsh:
  
    ```bash
    source ~/ros2_ws/install/setup.zsh
    ```

* Run the node using the provided Zsh script (recommended):

    ```bash
    cd ~/ros2_ws/src/waymo
    chmod +x run_keyboard_handler_node.zsh
    ./run_keyboard_handler_node.zsh
    ```

    Alternatively, run it directly (ensure your Conda environment, if used, is active):

    ```bash
    ros2 run waymo keyboard_handler_node
    ```

* Ensure the terminal running the node has keyboard focus.
* Press `s` to toggle the pause state (`MANUAL_PAUSE` in `state_manager_node`).
* Press `d` to toggle the debug image canvas visibility in `gui_debug_node`.
* Press `Ctrl+C` in the handler's terminal to stop it cleanly.

---

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

---

### Testing Functionality / Monitoring Topics

Open new terminals (and source the workspace) to check the functionality:

1. **`Display motion commands (/cmd_vel):`**

    ```bash
    ros2 topic echo /cmd_vel
    ```

    *`Shows the velocity commands sent by state_manager_node.`*

2. **`Display robot state (/robot/state):`**

    ```bash
    ros2 topic echo /robot/state
    ```

    *`Shows the current state (WAYMO_STARTED, STOPPED, FOLLOW_LANE).`*

3. **`Display obstacle status (/obstacle/blocked):`**

    ```bash
    ros2 topic echo /obstacle/blocked
    ```

    *`Shows true or false, depending on whether an obstacle was detected.`*

4. **`Display traffic light status (/traffic_light):`**

    ```bash
    ros2 topic echo /traffic_light
    ```

    *Shows the boolean status from the color detector (`false` if specific color detected, `true` otherwise).*

5. **`Display lane offset (/lane/center_offset):`**

    ```bash
    ros2 topic echo /lane/center_offset
    ```

    *Shows the calculated lateral offset from the lane center.*

6. **Visualize Node Graph:**

    ```bash
    rqt_graph
    ```

    *`Shows how the nodes are connected and which topics they use (requires rqt).`*

7. **`Display detected sign messages (/sign):`**

    ```bash
    ros2 topic echo /sign
    ```

    *`Shows string messages when signs (e.g., "parking_sign_detected") are detected by sign_detection_node.`*

8. **`Display parking maneuver status (/parking/finished):`**

    ```bash
    ros2 topic echo /parking/finished
    ```

    *`Shows true when the parking_node has completed (or aborted) the parking maneuver.`*

9. **Debug Images from `sign_detection_node` (GUI or `rqt_image_view`):**
    * `/debug/cam/template_matching` (CompressedImage): Visual output of the template matching process.
    * `/debug/cam/binary_sign` (CompressedImage): Shows the binarized image used for sign detection.

10. **Debug Images from `gui_debug_node` (Canvas):**
    * The `gui_debug_node` (toggled via 'd' with `keyboard_handler_node`) can display various debug image topics, including those from sign detection if configured. Check `gui_debug_node.py` for the list of subscribed topics.

---

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, CompressedImage
from std_msgs.msg import String, Bool, Float64
import math
import numpy as np
import time
import sys
import traceback
from enum import Enum

import cv2
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation as R
from rcl_interfaces.msg import (
    ParameterDescriptor,
    ParameterType,
    FloatingPointRange,
    IntegerRange,
)

NODE_NAME = "intersection_handling_node"

# --- Topics & States ---
(
    CMD_VEL_TOPIC,
    LASERSCAN_TOPIC,
    ROBOT_STATE_TOPIC,
    ODOM_TOPIC,
    IMAGE_TOPIC,
    CENTER_OFFSET_TOPIC,
    INTERSECTION_FINISHED_TOPIC,
    RECOMMENDED_SPEED_TOPIC,
    DEBUG_IMAGE_TOPIC,
) = (
    "/cmd_vel",
    "/scan",
    "/robot/state",
    "/odom",
    "/image_raw/compressed",
    "/lane/center_offset",
    "/intersection/finished",
    "/robot/recommended_speed",
    "/debug/cam/intersection_analysis",
)

(
    STATE_INTERSECTION_DRIVING_STRAIGHT,
    STATE_INTERSECTION_TURNING_RIGHT,
    STATE_INTERSECTION_TURNING_LEFT,
) = ("INTERSEC_DRIVING_STRAIGHT", "INTERSEC_TURNING_RIGHT", "INTERSEC_TURNING_LEFT")

# --- Default-Werte ---
DEFAULT_APPROACH_SPEED = 0.2  # m/s
DEFAULT_STRAIGHT_SPEED_PART1 = 0.2
DEFAULT_STRAIGHT_SPEED_FINAL = 0.2
DEFAULT_TURN_FORWARD_SPEED = 0.125
DEFAULT_TURN_ANGULAR_SPEED_RIGHT = 0.45
DEFAULT_TURN_ANGULAR_SPEED_LEFT = 0.25
VISUAL_CORRECTION_ANGULAR_SPEED = 0.2  # Angular speed for visual correction in rad/s
(
    DEFAULT_STRAIGHT_DISTANCE_PART1,
    DEFAULT_STRAIGHT_DISTANCE_FINAL,
    DEFAULT_POST_TURN_STRAIGHT_DISTANCE,
) = (0.4, 0.45, 0.1)
(
    DEFAULT_SIDE_SIGN_SCAN_TIMEOUT,
    DEFAULT_WAIT_AT_REFERENCE_DURATION,
    DEFAULT_PRE_ANALYSIS_WAIT_DURATION,
) = (10.0, 0.0, 0.1)
(
    DEFAULT_FINAL_WAIT_DURATION,
    DEFAULT_RIGHT_SIDE_SCAN_ANGLE_MIN_DEG,
    DEFAULT_RIGHT_SIDE_SCAN_ANGLE_MAX_DEG,
) = (0.0, 88.0, 92.0)
DEFAULT_RIGHT_SIDE_SCAN_DISTANCE, DEFAULT_TURN_ANGLE_90_DEG = 0.25, math.pi / 2
DEFAULT_GOAL_TOLERANCE_ANGLE_RAD, DEFAULT_ODOM_DISTANCE_TOLERANCE = (
    math.radians(1.5),
    0.01,
)


class IntersectionPhase(Enum):
    (
        IDLE,
        DRIVING_TO_SIDE_SIGN_REFERENCE,
        WAITING_AT_REFERENCE_POINT,
        EXECUTING_STRAIGHT_MANEUVER_PART1,
        PRE_ANALYSIS_WAIT,
        CORRECTING_ANGLE_WITH_VISUAL_FEEDBACK,
        EXECUTING_STRAIGHT_MANEUVER_FINAL,
        EXECUTING_LEFT_TURN_MANEUVER_TURN,
        EXECUTING_LEFT_TURN_MANEUVER_STRAIGHT,
        EXECUTING_RIGHT_TURN_MANEUVER_TURN,
        EXECUTING_RIGHT_TURN_MANEUVER_STRAIGHT,
        FINAL_WAIT,
        INTERSECTION_FINISHED,
        ABORTING,
    ) = range(14)


class IntersectionHandlingNode(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.bridge = CvBridge()

        def float_desc(desc, min_val=-100.0, max_val=100.0, step=0.001):
            return ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description=desc,
                floating_point_range=[
                    FloatingPointRange(from_value=min_val, to_value=max_val, step=step)
                ],
            )

        def deg_angle_desc(desc, min_val=0.0, max_val=360.0, step=1.0):
            return ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description=desc,
                floating_point_range=[
                    FloatingPointRange(from_value=min_val, to_value=max_val, step=step)
                ],
            )

        def int_desc(desc, min_val=0, max_val=50000, step=1):
            return ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description=desc,
                integer_range=[
                    IntegerRange(from_value=min_val, to_value=max_val, step=step)
                ],
            )

        def percent_desc(desc):
            return ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description=desc,
                floating_point_range=[
                    FloatingPointRange(from_value=0.0, to_value=100.0, step=0.1)
                ],
            )

        self.declare_parameter(
            "approach_speed", DEFAULT_APPROACH_SPEED, float_desc("...")
        )
        self.declare_parameter(
            "straight_speed_part1", DEFAULT_STRAIGHT_SPEED_PART1, float_desc("...")
        )
        self.declare_parameter(
            "straight_speed_final", DEFAULT_STRAIGHT_SPEED_FINAL, float_desc("...")
        )
        self.declare_parameter(
            "turn_forward_speed", DEFAULT_TURN_FORWARD_SPEED, float_desc("...")
        )
        self.declare_parameter(
            "turn_angular_speed_right",
            DEFAULT_TURN_ANGULAR_SPEED_RIGHT,
            float_desc("..."),
        )
        self.declare_parameter(
            "turn_angular_speed_left",
            DEFAULT_TURN_ANGULAR_SPEED_LEFT,
            float_desc("..."),
        )
        self.declare_parameter(
            "straight_distance_part1",
            DEFAULT_STRAIGHT_DISTANCE_PART1,
            float_desc("..."),
        )
        self.declare_parameter(
            "straight_distance_final",
            DEFAULT_STRAIGHT_DISTANCE_FINAL,
            float_desc("..."),
        )
        self.declare_parameter(
            "post_turn_straight_distance",
            DEFAULT_POST_TURN_STRAIGHT_DISTANCE,
            float_desc("..."),
        )
        self.declare_parameter(
            "side_sign_scan_timeout", DEFAULT_SIDE_SIGN_SCAN_TIMEOUT, float_desc("...")
        )
        self.declare_parameter(
            "wait_at_reference_duration",
            DEFAULT_WAIT_AT_REFERENCE_DURATION,
            float_desc("..."),
        )
        self.declare_parameter(
            "pre_analysis_wait_duration",
            DEFAULT_PRE_ANALYSIS_WAIT_DURATION,
            float_desc("..."),
        )
        self.declare_parameter(
            "final_wait_duration", DEFAULT_FINAL_WAIT_DURATION, float_desc("...")
        )
        self.declare_parameter(
            "right_side_scan_angle_min_deg",
            DEFAULT_RIGHT_SIDE_SCAN_ANGLE_MIN_DEG,
            deg_angle_desc("..."),
        )
        self.declare_parameter(
            "right_side_scan_angle_max_deg",
            DEFAULT_RIGHT_SIDE_SCAN_ANGLE_MAX_DEG,
            deg_angle_desc("..."),
        )
        self.declare_parameter(
            "right_side_scan_distance",
            DEFAULT_RIGHT_SIDE_SCAN_DISTANCE,
            float_desc("..."),
        )
        self.declare_parameter(
            "img_analysis_crop_top_percent", 60.0, percent_desc("...")
        )
        self.declare_parameter(
            "img_analysis_crop_bottom_percent", 75.0, percent_desc("...")
        )
        self.declare_parameter(
            "img_analysis_binary_threshold", 128, int_desc("...", max_val=255)
        )
        self.declare_parameter(
            "histogram_valley_threshold", 500, int_desc("...", step=10)
        )
        self.declare_parameter(
            "visual_centering_offset_correction", 30.0, float_desc("...")
        )
        self.declare_parameter(
            "visual_correction_angular_speed",
            VISUAL_CORRECTION_ANGULAR_SPEED,
            float_desc(
                "Angular speed for turning in place to correct offset", max_val=1.0
            ),
        )
        self.declare_parameter(
            "visual_correction_tolerance_pixels",
            2.0,
            float_desc("Pixel tolerance for visual correction maneuver"),
        )
        self.declare_parameter("lane_follow_p_gain", 1.0, float_desc("..."))
        self.declare_parameter("max_angular_z_lane_follow", 1.0, float_desc("..."))

        self.approach_speed = self.get_parameter("approach_speed").value
        (
            self.latest_image_frame,
            self.latest_image_timestamp,
            self.current_visual_offset,
        ) = (None, None, None)
        self.current_phase = IntersectionPhase.IDLE
        self.active_intersection_state, self.maneuver_active_by_statemgr = None, False
        (
            self.current_pos_x,
            self.current_pos_y,
            self.current_yaw,
            self.current_center_offset,
        ) = (0.0, 0.0, 0.0, 0.0)
        (
            self.start_pos_x_segment,
            self.start_pos_y_segment,
            self.start_yaw_for_turn,
            self.target_yaw_for_turn,
        ) = (0.0, 0.0, 0.0, 0.0)
        self.phase_start_time, self.side_sign_detected_by_laser = 0.0, False

        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.image_subscriber = self.create_subscription(
            CompressedImage, IMAGE_TOPIC, self.image_callback, qos_sensor
        )
        self.debug_image_publisher = self.create_publisher(
            CompressedImage, DEBUG_IMAGE_TOPIC, qos_sensor
        )
        self.cmd_vel_publisher = self.create_publisher(
            Twist, CMD_VEL_TOPIC, qos_reliable
        )
        self.recommended_speed_subscriber = self.create_subscription(
            Float64, RECOMMENDED_SPEED_TOPIC, self.speed_callback, qos_reliable
        )
        self.odom_subscriber = self.create_subscription(
            Odometry, ODOM_TOPIC, self.odom_callback, qos_sensor
        )
        self.scan_subscriber = self.create_subscription(
            LaserScan, LASERSCAN_TOPIC, self.scan_callback, qos_sensor
        )
        self.state_subscriber = self.create_subscription(
            String, ROBOT_STATE_TOPIC, self.robot_state_manager_callback, qos_reliable
        )
        self.center_offset_subscriber = self.create_subscription(
            Float64, CENTER_OFFSET_TOPIC, self.center_offset_callback, qos_sensor
        )
        self.intersection_finished_publisher = self.create_publisher(
            Bool, INTERSECTION_FINISHED_TOPIC, qos_reliable
        )
        self.control_timer = self.create_timer(0.01, self.run_intersection_maneuver)

    def image_callback(self, msg: CompressedImage):
        if not self.maneuver_active_by_statemgr:
            return
        try:
            self.latest_image_timestamp = msg.header.stamp
            self.latest_image_frame = self.bridge.compressed_imgmsg_to_cv2(
                msg, desired_encoding="bgr8"
            )
            if (
                self.current_phase
                == IntersectionPhase.CORRECTING_ANGLE_WITH_VISUAL_FEEDBACK
            ):
                self.calculate_current_visual_offset()
        except Exception as e:
            self.get_logger().warn(
                f"Error decoding image: {e}", throttle_duration_sec=5
            )
            self.latest_image_frame = None

    def speed_callback(self, msg: Float64):
        if not self.maneuver_active_by_statemgr:
            return
        # self.approach_speed = msg.data
        self.approach_speed = 0.2  # Temporarily set to a fixed value for testing

    def odom_callback(self, msg: Odometry):
        self.current_pos_x, self.current_pos_y = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
        )
        q = msg.pose.pose.orientation
        try:
            self.current_yaw = R.from_quat([q.x, q.y, q.z, q.w]).as_euler(
                "xyz", degrees=False
            )[2]
        except Exception:
            pass

    def scan_callback(self, msg: LaserScan):
        if (
            self.current_phase != IntersectionPhase.DRIVING_TO_SIDE_SIGN_REFERENCE
            or self.side_sign_detected_by_laser
        ):
            return
        try:
            angle_min_deg, angle_max_deg, dist = (
                self.get_parameter("right_side_scan_angle_min_deg").value,
                self.get_parameter("right_side_scan_angle_max_deg").value,
                self.get_parameter("right_side_scan_distance").value,
            )
            if self._check_laser_zone(
                msg, math.radians(angle_min_deg), math.radians(angle_max_deg), dist
            ):
                self.side_sign_detected_by_laser = True
                self.stop_robot()
                self.change_phase(IntersectionPhase.WAITING_AT_REFERENCE_POINT)
        except rclpy.exceptions.ParameterNotDeclaredException as e:
            self.get_logger().error(
                f"FATAL: Parameter not declared in scan_callback: {e}", once=True
            )

    def robot_state_manager_callback(self, msg: String):
        new_state = msg.data
        is_intersection_now = new_state in [
            STATE_INTERSECTION_DRIVING_STRAIGHT,
            STATE_INTERSECTION_TURNING_LEFT,
            STATE_INTERSECTION_TURNING_RIGHT,
        ]
        if is_intersection_now and not self.maneuver_active_by_statemgr:
            # self.get_logger().info(f"Intersection handling ACTIVATED with state: {new_state}")
            self.maneuver_active_by_statemgr, self.active_intersection_state = (
                True,
                new_state,
            )
            self.change_phase(IntersectionPhase.DRIVING_TO_SIDE_SIGN_REFERENCE)
        elif not is_intersection_now and self.maneuver_active_by_statemgr:
            # self.get_logger().info("Intersection handling DEACTIVATED.")
            self.maneuver_active_by_statemgr = False
            self.change_phase(IntersectionPhase.IDLE)

    def center_offset_callback(self, msg: Float64):
        self.current_center_offset = msg.data

    def run_intersection_maneuver(self):
        if not self.maneuver_active_by_statemgr:
            if self.current_phase != IntersectionPhase.IDLE:
                self.change_phase(IntersectionPhase.IDLE)
            return
        elapsed_phase_time = (
            self.get_clock().now().nanoseconds / 1e9 - self.phase_start_time
        )

        if self.current_phase == IntersectionPhase.DRIVING_TO_SIDE_SIGN_REFERENCE:
            if elapsed_phase_time > self.get_parameter("side_sign_scan_timeout").value:
                self.change_phase(IntersectionPhase.ABORTING)
                return
            # self.drive_with_lane_follow(self.get_parameter('approach_speed').value)
            self.move_robot(self.approach_speed, 0.0)
        elif self.current_phase == IntersectionPhase.WAITING_AT_REFERENCE_POINT:
            if (
                elapsed_phase_time
                >= self.get_parameter("wait_at_reference_duration").value
            ):
                if (
                    self.active_intersection_state
                    == STATE_INTERSECTION_DRIVING_STRAIGHT
                ):
                    self.change_phase(
                        IntersectionPhase.EXECUTING_STRAIGHT_MANEUVER_PART1
                    )
                elif self.active_intersection_state == STATE_INTERSECTION_TURNING_LEFT:
                    self.change_phase(
                        IntersectionPhase.EXECUTING_LEFT_TURN_MANEUVER_TURN
                    )
                elif self.active_intersection_state == STATE_INTERSECTION_TURNING_RIGHT:
                    self.change_phase(
                        IntersectionPhase.EXECUTING_RIGHT_TURN_MANEUVER_TURN
                    )
                else:
                    self.change_phase(IntersectionPhase.ABORTING)
        elif self.current_phase == IntersectionPhase.EXECUTING_STRAIGHT_MANEUVER_PART1:
            if self.drive_distance_straight(
                self.get_parameter("straight_distance_part1").value,
                self.get_parameter("straight_speed_part1").value,
            ):
                self.change_phase(IntersectionPhase.PRE_ANALYSIS_WAIT)
        elif self.current_phase == IntersectionPhase.PRE_ANALYSIS_WAIT:
            self.stop_robot()
            if (
                elapsed_phase_time
                >= self.get_parameter("pre_analysis_wait_duration").value
            ):
                self.change_phase(
                    IntersectionPhase.CORRECTING_ANGLE_WITH_VISUAL_FEEDBACK
                )
        elif (
            self.current_phase
            == IntersectionPhase.CORRECTING_ANGLE_WITH_VISUAL_FEEDBACK
        ):
            tolerance = self.get_parameter("visual_correction_tolerance_pixels").value
            if self.current_visual_offset is None:
                # self.get_logger().warn("Waiting for visual offset to be calculated...", throttle_duration_sec=1)
                self.stop_robot()
                return
            if abs(self.current_visual_offset) <= tolerance:
                # self.get_logger().info("Visual correction finished. Offset is within tolerance.")
                self.stop_robot()
                self.change_phase(IntersectionPhase.EXECUTING_STRAIGHT_MANEUVER_FINAL)
            else:
                speed = self.get_parameter("visual_correction_angular_speed").value
                turn_direction = -1.0 if self.current_visual_offset > 0 else 1.0
                self.move_robot(0.0, turn_direction * speed)
        elif self.current_phase == IntersectionPhase.EXECUTING_STRAIGHT_MANEUVER_FINAL:
            if self.drive_distance_straight(
                self.get_parameter("straight_distance_final").value,
                self.get_parameter("straight_speed_final").value,
            ):
                self.change_phase(IntersectionPhase.FINAL_WAIT)
        elif "TURN_MANEUVER_TURN" in self.current_phase.name:
            fwd_speed, ang_speed_param = self.get_parameter(
                "turn_forward_speed"
            ).value, (
                "turn_angular_speed_left"
                if "LEFT" in self.current_phase.name
                else "turn_angular_speed_right"
            )
            angular_speed = (
                self.get_parameter(ang_speed_param).value
                if "LEFT" in self.current_phase.name
                else -self.get_parameter(ang_speed_param).value
            )
            if self.turn_to_target_yaw(
                self.target_yaw_for_turn, angular_speed, fwd_speed
            ):
                next_phase = (
                    IntersectionPhase.EXECUTING_LEFT_TURN_MANEUVER_STRAIGHT
                    if "LEFT" in self.current_phase.name
                    else IntersectionPhase.EXECUTING_RIGHT_TURN_MANEUVER_STRAIGHT
                )
                self.change_phase(next_phase)
        elif "TURN_MANEUVER_STRAIGHT" in self.current_phase.name:
            if self.drive_distance_with_lane_follow(
                self.get_parameter("post_turn_straight_distance").value,
                self.get_parameter("turn_forward_speed").value,
            ):
                self.change_phase(IntersectionPhase.FINAL_WAIT)
        elif self.current_phase == IntersectionPhase.FINAL_WAIT:
            if elapsed_phase_time >= self.get_parameter("final_wait_duration").value:
                self.change_phase(IntersectionPhase.INTERSECTION_FINISHED)

    def calculate_current_visual_offset(self):
        if self.latest_image_frame is None:
            self.current_visual_offset = None
            return
        output_frame = self.latest_image_frame.copy()
        h, w, _ = output_frame.shape

        crop_top = int(
            h * self.get_parameter("img_analysis_crop_top_percent").value / 100.0
        )
        crop_bottom = int(
            h * self.get_parameter("img_analysis_crop_bottom_percent").value / 100.0
        )
        binary_threshold = self.get_parameter("img_analysis_binary_threshold").value
        valley_threshold = self.get_parameter("histogram_valley_threshold").value
        offset_correction = self.get_parameter(
            "visual_centering_offset_correction"
        ).value

        cropped_frame = output_frame[crop_top:crop_bottom, :]
        if cropped_frame.size == 0:
            self.current_visual_offset = None
            return

        gray_frame = cv2.cvtColor(cropped_frame, cv2.COLOR_BGR2GRAY)
        _, binary_frame = cv2.threshold(
            gray_frame, binary_threshold, 255, cv2.THRESH_BINARY
        )
        histogram = np.sum(binary_frame, axis=0)
        is_valley = histogram < valley_threshold
        diff = np.diff(is_valley.astype(int))
        starts = np.where(diff == 1)[0] + 1
        ends = np.where(diff == -1)[0]
        if is_valley[0]:
            starts = np.insert(starts, 0, 0)
        if is_valley[-1]:
            ends = np.append(ends, len(is_valley) - 1)

        road_center_pixel, corrected_offset_px = -1, None
        if len(starts) > 0 and len(ends) > 0 and len(starts) == len(ends):
            lengths = ends - starts
            if np.max(lengths) > 0:
                longest_block_idx = np.argmax(lengths)
                road_start, road_end = (
                    starts[longest_block_idx],
                    ends[longest_block_idx],
                )
                road_center_pixel = (road_start + road_end) // 2
                raw_offset_px = float(road_center_pixel - (w // 2))
                corrected_offset_px = raw_offset_px + offset_correction

        self.current_visual_offset = corrected_offset_px

        cv2.line(output_frame, (0, crop_top), (w, crop_top), (0, 255, 255), 1)
        cv2.line(output_frame, (0, crop_bottom), (w, crop_bottom), (0, 255, 255), 1)
        if road_center_pixel != -1:
            cv2.line(
                output_frame,
                (road_center_pixel, crop_top),
                (road_center_pixel, crop_bottom),
                (255, 0, 0),
                2,
            )
            if self.current_visual_offset is not None:
                cv2.putText(
                    output_frame,
                    f"Live Offset: {self.current_visual_offset:.1f}",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (0, 0, 0),
                    3,
                )
                cv2.putText(
                    output_frame,
                    f"Live Offset: {self.current_visual_offset:.1f}",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (255, 255, 255),
                    2,
                )
        self._publish_debug_image(output_frame, self.latest_image_timestamp)

    def move_robot(self, linear_x: float, angular_z: float):
        twist_msg = Twist()
        twist_msg.linear.x = linear_x
        twist_msg.angular.z = float(angular_z)
        self.cmd_vel_publisher.publish(twist_msg)

    def stop_robot(self):
        self.move_robot(0.0, 0.0)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def drive_distance_straight(self, target_distance, speed) -> bool:
        traveled = math.sqrt(
            (self.current_pos_x - self.start_pos_x_segment) ** 2
            + (self.current_pos_y - self.start_pos_y_segment) ** 2
        )
        if traveled >= target_distance - DEFAULT_ODOM_DISTANCE_TOLERANCE:
            self.stop_robot()
            return True
        self.move_robot(speed, 0.0)
        return False

    def drive_with_lane_follow(self, speed):
        p_gain = self.get_parameter("lane_follow_p_gain").value
        max_angular = self.get_parameter("max_angular_z_lane_follow").value
        angular_z = np.clip(
            self.current_center_offset * p_gain, -max_angular, max_angular
        )
        self.move_robot(speed, angular_z)

    def turn_to_target_yaw(self, target_yaw, angular_speed_cmd, forward_speed) -> bool:
        angle_diff = self.normalize_angle(target_yaw - self.current_yaw)
        if abs(angle_diff) < DEFAULT_GOAL_TOLERANCE_ANGLE_RAD:
            self.stop_robot()
            return True
        self.move_robot(forward_speed, angular_speed_cmd)
        return False

    def drive_distance_with_lane_follow(self, target_distance, speed) -> bool:
        traveled = math.sqrt(
            (self.current_pos_x - self.start_pos_x_segment) ** 2
            + (self.current_pos_y - self.start_pos_y_segment) ** 2
        )
        if traveled >= target_distance - DEFAULT_ODOM_DISTANCE_TOLERANCE:
            self.stop_robot()
            return True
        self.drive_with_lane_follow(speed)
        return False

    def _publish_debug_image(self, image, timestamp):
        try:
            msg = self.bridge.cv2_to_compressed_imgmsg(image, dst_format="jpeg")
            if timestamp is not None:
                msg.header.stamp = timestamp
            self.debug_image_publisher.publish(msg)
        except Exception as e:
            self.get_logger().warn(
                f"Error publishing debug image: {e}", throttle_duration_sec=5
            )

    def _check_laser_zone(self, msg, min_rad, max_rad, dist_thresh) -> bool:
        if msg.angle_increment <= 0.0:
            return False
        try:
            start_idx, end_idx = max(
                0, int((min_rad - msg.angle_min) / msg.angle_increment)
            ), min(
                len(msg.ranges) - 1,
                int((max_rad - msg.angle_min) / msg.angle_increment),
            )
            if start_idx > end_idx:
                return False
            for i in range(start_idx, end_idx + 1):
                if (
                    not math.isinf(msg.ranges[i])
                    and not math.isnan(msg.ranges[i])
                    and msg.range_min <= msg.ranges[i] < dist_thresh
                ):
                    return True
        except (ValueError, IndexError):
            return False
        return False

    def change_phase(self, new_phase: IntersectionPhase):
        if self.current_phase == new_phase:
            return
        # self.get_logger().info(f"Phase change: {self.current_phase.name} -> {new_phase.name}")
        self.current_phase, self.phase_start_time = (
            new_phase,
            self.get_clock().now().nanoseconds / 1e9,
        )
        if "STRAIGHT_MANEUVER_PART" in new_phase.name or "FINAL" in new_phase.name:
            self.start_pos_x_segment, self.start_pos_y_segment = (
                self.current_pos_x,
                self.current_pos_y,
            )
        elif new_phase == IntersectionPhase.DRIVING_TO_SIDE_SIGN_REFERENCE:
            self.side_sign_detected_by_laser = False
        elif new_phase == IntersectionPhase.CORRECTING_ANGLE_WITH_VISUAL_FEEDBACK:
            self.current_visual_offset = None
        elif "TURN_MANEUVER_TURN" in new_phase.name:
            self.start_yaw_for_turn = self.current_yaw
            angle = (
                DEFAULT_TURN_ANGLE_90_DEG
                if "LEFT" in new_phase.name
                else -DEFAULT_TURN_ANGLE_90_DEG
            )
            self.target_yaw_for_turn = self.normalize_angle(self.current_yaw + angle)
        elif new_phase in [
            IntersectionPhase.IDLE,
            IntersectionPhase.ABORTING,
            IntersectionPhase.INTERSECTION_FINISHED,
        ]:
            self.stop_robot()
            if new_phase != IntersectionPhase.IDLE:
                self.intersection_finished_publisher.publish(
                    Bool(data=(new_phase == IntersectionPhase.INTERSECTION_FINISHED))
                )
            if new_phase != IntersectionPhase.INTERSECTION_FINISHED:
                self.change_phase(IntersectionPhase.IDLE)
            self.maneuver_active_by_statemgr = False


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = IntersectionHandlingNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Bool, Float64, Float32 # <<< KORREKTUR: Float32 importiert
import math
import numpy as np
import time
import sys
import traceback
from enum import Enum, auto

from scipy.spatial.transform import Rotation as R
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, FloatingPointRange, IntegerRange

NODE_NAME = 'intersection_handling_node'

# --- Standardwerte für Parameter (unverändert) ---
DEFAULT_APPROACH_SPEED = 0.15
DEFAULT_INITIAL_DRIVE_SPEED = 0.125
DEFAULT_STRAIGHT_SPEED = 0.125
DEFAULT_TURN_FORWARD_SPEED = 0.125
DEFAULT_TURN_ANGULAR_SPEED_RIGHT = 0.45
DEFAULT_TURN_ANGULAR_SPEED_LEFT = 0.25
DEFAULT_STRAIGHT_DISTANCE = 0.8
DEFAULT_POST_TURN_STRAIGHT_DISTANCE = 0.1
DEFAULT_SIDE_SIGN_SCAN_TIMEOUT = 10.0
DEFAULT_WAIT_AT_REFERENCE_DURATION = 0.0
DEFAULT_FINAL_WAIT_DURATION = 0.0
DEFAULT_RIGHT_SIDE_SCAN_ANGLE_MIN_DEG = 88.0
DEFAULT_RIGHT_SIDE_SCAN_ANGLE_MAX_DEG = 92.0
DEFAULT_RIGHT_SIDE_SCAN_DISTANCE = 0.25
DEFAULT_TURN_ANGLE_90_DEG = math.pi / 2
DEFAULT_GOAL_TOLERANCE_ANGLE_RAD = math.radians(2.5)
DEFAULT_ODOM_DISTANCE_TOLERANCE = 0.01

class IntersectionPhase(Enum): # Unverändert
    IDLE = auto()
    DRIVING_TO_SIDE_SIGN_REFERENCE = auto()
    WAITING_AT_REFERENCE_POINT = auto()
    EXECUTING_STRAIGHT_MANEUVER = auto()
    EXECUTING_LEFT_TURN_MANEUVER_TURN = auto()
    EXECUTING_LEFT_TURN_MANEUVER_STRAIGHT = auto()
    EXECUTING_RIGHT_TURN_MANEUVER_TURN = auto()
    EXECUTING_RIGHT_TURN_MANEUVER_STRAIGHT = auto()
    FINAL_WAIT = auto()
    INTERSECTION_FINISHED = auto()
    ABORTING = auto()

class IntersectionHandlingNode(Node):
    def __init__(self): # Weitgehend unverändert
        super().__init__(NODE_NAME)

        def float_desc(desc, min_val=0.0, max_val=10.0, step=0.001):
            return ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE, description=desc,
                                       floating_point_range=[FloatingPointRange(from_value=min_val, to_value=max_val, step=step)])
        def deg_angle_desc(desc, min_val=0.0, max_val=360.0, step=1.0):
            return ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE, description=desc,
                                       floating_point_range=[FloatingPointRange(from_value=min_val, to_value=max_val, step=step)])

        self.declare_parameter('approach_speed', DEFAULT_APPROACH_SPEED, float_desc("Speed when driving to find side sign", max_val=0.5))
        self.declare_parameter('initial_drive_speed', DEFAULT_INITIAL_DRIVE_SPEED, float_desc("Speed when driving to find side sign", max_val=0.5))
        self.declare_parameter('straight_speed', DEFAULT_STRAIGHT_SPEED, float_desc("Speed for straight intersection maneuver", max_val=0.5))
        self.declare_parameter('max_angular_z_lane_follow', 1.0, float_desc("Max angular speed for lane following", max_val=2.0))
        self.declare_parameter('turn_forward_speed', DEFAULT_TURN_FORWARD_SPEED, float_desc("Forward speed during turning maneuver", max_val=0.5))
        self.declare_parameter('turn_angular_speed_right', DEFAULT_TURN_ANGULAR_SPEED_RIGHT, float_desc("Angular speed for right turn", max_val=1.5))
        self.declare_parameter('turn_angular_speed_left', DEFAULT_TURN_ANGULAR_SPEED_LEFT, float_desc("Angular speed for left turn", max_val=1.5))
        self.declare_parameter('straight_distance', DEFAULT_STRAIGHT_DISTANCE, float_desc("Distance for straight maneuver", max_val=2.0))
        self.declare_parameter('post_turn_straight_distance', DEFAULT_POST_TURN_STRAIGHT_DISTANCE, float_desc("Short straight distance after turning", max_val=0.5))
        self.declare_parameter('side_sign_scan_timeout', DEFAULT_SIDE_SIGN_SCAN_TIMEOUT, float_desc("Timeout for detecting side sign", max_val=20.0))
        self.declare_parameter('wait_at_reference_duration', DEFAULT_WAIT_AT_REFERENCE_DURATION, float_desc("Wait duration after side sign detected", max_val=5.0))
        self.declare_parameter('final_wait_duration', DEFAULT_FINAL_WAIT_DURATION, float_desc("Final wait duration after maneuver", max_val=5.0))
        self.declare_parameter('right_side_scan_angle_min_deg', DEFAULT_RIGHT_SIDE_SCAN_ANGLE_MIN_DEG, deg_angle_desc("Min angle for right side scan (degrees, e.g., 88 for right based on parking_node)", min_val=0.0, max_val=180.0))
        self.declare_parameter('right_side_scan_angle_max_deg', DEFAULT_RIGHT_SIDE_SCAN_ANGLE_MAX_DEG, deg_angle_desc("Max angle for right side scan (degrees, e.g., 92 for right based on parking_node)", min_val=0.0, max_val=180.0))
        self.declare_parameter('right_side_scan_distance', DEFAULT_RIGHT_SIDE_SCAN_DISTANCE, float_desc("Detection distance for right side sign", max_val=1.0))

        self.current_phase = IntersectionPhase.IDLE
        self.active_intersection_state = None
        self.maneuver_active_by_statemgr = False
        self.current_pos_x = 0.0
        self.current_pos_y = 0.0
        self.current_yaw = 0.0
        self.current_center_offset = 0.0
        self.start_pos_x_segment = 0.0
        self.start_pos_y_segment = 0.0
        self.start_yaw_for_turn = 0.0
        self.target_yaw_for_turn = 0.0
        self.phase_start_time = 0.0
        self.side_sign_detected_by_laser = False

        qos_sensor = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        qos_reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1)

        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, qos_sensor)
        self.scan_subscriber = self.create_subscription(LaserScan, LASERSCAN_TOPIC, self.scan_callback, qos_sensor)
        self.state_subscriber = self.create_subscription(String, ROBOT_STATE_TOPIC, self.robot_state_manager_callback, qos_reliable)
        # <<< KORREKTUR: Fehlenden Subscriber hinzugefügt
        self.center_offset_subscriber = self.create_subscription(Float32, '/lane/center_offset', self.center_offset_callback, qos_sensor)
        self.cmd_vel_publisher = self.create_publisher(Twist, CMD_VEL_TOPIC, qos_reliable)
        self.intersection_finished_publisher = self.create_publisher(Bool, '/intersection/finished', qos_reliable)
        self.control_timer_period = 0.005 # 200 Hz
        self.control_timer = self.create_timer(self.control_timer_period, self.run_intersection_maneuver)

    # <<< KORREKTUR: Datentyp von Float64 auf Float32 geändert
    def center_offset_callback(self, msg: Float32):
        self.current_center_offset = msg.data

    def robot_state_manager_callback(self, msg: String):
        new_state_from_manager = msg.data
        is_intersection_state_now = new_state_from_manager in [
            STATE_INTERSECTION_DRIVING_STRAIGHT, STATE_INTERSECTION_TURNING_LEFT, STATE_INTERSECTION_TURNING_RIGHT
        ]

        if is_intersection_state_now:
            if not self.maneuver_active_by_statemgr:
                self.maneuver_active_by_statemgr = True
                self.active_intersection_state = new_state_from_manager
                if self.current_phase == IntersectionPhase.IDLE:
                    self.side_sign_detected_by_laser = False
                    self.change_phase(IntersectionPhase.DRIVING_TO_SIDE_SIGN_REFERENCE)
                else:
                    self.side_sign_detected_by_laser = False
                    self.change_phase(IntersectionPhase.DRIVING_TO_SIDE_SIGN_REFERENCE)
            elif self.maneuver_active_by_statemgr and self.active_intersection_state != new_state_from_manager:
                self.active_intersection_state = new_state_from_manager
                self.side_sign_detected_by_laser = False
                self.change_phase(IntersectionPhase.DRIVING_TO_SIDE_SIGN_REFERENCE)

        elif not is_intersection_state_now:
            if self.maneuver_active_by_statemgr:
                self.maneuver_active_by_statemgr = False
                if self.current_phase != IntersectionPhase.IDLE:
                    self.change_phase(IntersectionPhase.IDLE)

    def odom_callback(self, msg: Odometry):
        self.current_pos_x = msg.pose.pose.position.x
        self.current_pos_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        try:
            r = R.from_quat([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
            self.current_yaw = r.as_euler('xyz', degrees=False)[2]
        except Exception as e:
            self.get_logger().warn(f"Fehler bei Quaternion -> Euler Umwandlung: {e}", throttle_duration_sec=5)

    def scan_callback(self, msg: LaserScan):
        if not self.maneuver_active_by_statemgr or self.current_phase != IntersectionPhase.DRIVING_TO_SIDE_SIGN_REFERENCE or self.side_sign_detected_by_laser:
            return
        angle_min_param_deg = self.get_parameter('right_side_scan_angle_min_deg').value
        angle_max_param_deg = self.get_parameter('right_side_scan_angle_max_deg').value
        detection_dist_param = self.get_parameter('right_side_scan_distance').value
        target_angle_min_rad = math.radians(angle_min_param_deg)
        target_angle_max_rad = math.radians(angle_max_param_deg)
        obstacle_detected = self._check_laser_zone(msg, target_angle_min_rad, target_angle_max_rad, detection_dist_param)
        if obstacle_detected:
            self.side_sign_detected_by_laser = True
            self.stop_robot()
            self.change_phase(IntersectionPhase.WAITING_AT_REFERENCE_POINT)

    def _check_laser_zone(self, scan_msg: LaserScan, angle_min_rad_target: float, angle_max_rad_target: float, detection_distance: float) -> bool:
        if scan_msg.angle_increment <= 0.0:
            self.get_logger().warn("Ungültiges angle_increment im Laserscan.", throttle_duration_sec=10)
            return False
        
        actual_scan_angle_max_rad = scan_msg.angle_min + (len(scan_msg.ranges) - 1) * scan_msg.angle_increment
        adj_target_min_rad = max(angle_min_rad_target, scan_msg.angle_min)
        adj_target_max_rad = min(angle_max_rad_target, actual_scan_angle_max_rad)

        if adj_target_min_rad >= adj_target_max_rad:
            return False

        start_index = max(0, int((adj_target_min_rad - scan_msg.angle_min) / scan_msg.angle_increment))
        end_index = min(len(scan_msg.ranges) - 1, int((adj_target_max_rad - scan_msg.angle_min) / scan_msg.angle_increment))
        
        if start_index > end_index:
            return False

        for i in range(start_index, end_index + 1):
            dist = scan_msg.ranges[i]
            if not math.isinf(dist) and not math.isnan(dist) and \
               dist >= scan_msg.range_min and dist <= scan_msg.range_max and \
               dist < detection_distance:
                return True
        return False

    def change_phase(self, new_phase: IntersectionPhase):
        if self.current_phase == new_phase and (self.current_phase == IntersectionPhase.IDLE and not self.maneuver_active_by_statemgr) :
             return
        
        self.current_phase = new_phase
        self.phase_start_time = self.get_clock().now().nanoseconds / 1e9

        if new_phase == IntersectionPhase.DRIVING_TO_SIDE_SIGN_REFERENCE:
            self.side_sign_detected_by_laser = False
        elif new_phase == IntersectionPhase.EXECUTING_STRAIGHT_MANEUVER:
            self.start_pos_x_segment = self.current_pos_x
            self.start_pos_y_segment = self.current_pos_y
        elif new_phase == IntersectionPhase.EXECUTING_LEFT_TURN_MANEUVER_TURN:
            self.start_yaw_for_turn = self.current_yaw
            self.target_yaw_for_turn = self.normalize_angle(self.current_yaw + DEFAULT_TURN_ANGLE_90_DEG)
        elif new_phase == IntersectionPhase.EXECUTING_RIGHT_TURN_MANEUVER_TURN:
            self.start_yaw_for_turn = self.current_yaw
            self.target_yaw_for_turn = self.normalize_angle(self.current_yaw - DEFAULT_TURN_ANGLE_90_DEG)
        elif new_phase == IntersectionPhase.EXECUTING_LEFT_TURN_MANEUVER_STRAIGHT or \
             new_phase == IntersectionPhase.EXECUTING_RIGHT_TURN_MANEUVER_STRAIGHT:
            self.start_pos_x_segment = self.current_pos_x
            self.start_pos_y_segment = self.current_pos_y
        
        elif new_phase == IntersectionPhase.IDLE:
            self.stop_robot()
            self.maneuver_active_by_statemgr = False
            self.active_intersection_state = None
            self.side_sign_detected_by_laser = False
        
        elif new_phase == IntersectionPhase.ABORTING:
            self.stop_robot()
            finished_msg = Bool(); finished_msg.data = False
            self.intersection_finished_publisher.publish(finished_msg)
            self.change_phase(IntersectionPhase.IDLE)
        
        elif new_phase == IntersectionPhase.INTERSECTION_FINISHED:
            self.stop_robot()
            finished_msg = Bool(); finished_msg.data = True
            self.intersection_finished_publisher.publish(finished_msg)
            self.change_phase(IntersectionPhase.IDLE)

    def run_intersection_maneuver(self):
        if not self.maneuver_active_by_statemgr:
            if self.current_phase != IntersectionPhase.IDLE:
                self.change_phase(IntersectionPhase.IDLE)
            return

        if self.current_phase == IntersectionPhase.IDLE and self.maneuver_active_by_statemgr:
            return

        current_time = self.get_clock().now().nanoseconds / 1e9
        elapsed_phase_time = current_time - self.phase_start_time

        if self.current_phase == IntersectionPhase.DRIVING_TO_SIDE_SIGN_REFERENCE:
            scan_timeout = self.get_parameter('side_sign_scan_timeout').value
            if elapsed_phase_time > scan_timeout:
                self.change_phase(IntersectionPhase.ABORTING)
                return
            self.follow_lane_to_reference()

        elif self.current_phase == IntersectionPhase.WAITING_AT_REFERENCE_POINT:
            self.stop_robot()
            wait_duration = self.get_parameter('wait_at_reference_duration').value
            if elapsed_phase_time >= wait_duration:
                if self.active_intersection_state == STATE_INTERSECTION_DRIVING_STRAIGHT:
                    self.change_phase(IntersectionPhase.EXECUTING_STRAIGHT_MANEUVER)
                elif self.active_intersection_state == STATE_INTERSECTION_TURNING_LEFT:
                    self.change_phase(IntersectionPhase.EXECUTING_LEFT_TURN_MANEUVER_TURN)
                elif self.active_intersection_state == STATE_INTERSECTION_TURNING_RIGHT:
                    self.change_phase(IntersectionPhase.EXECUTING_RIGHT_TURN_MANEUVER_TURN)
                else:
                    self.change_phase(IntersectionPhase.ABORTING)
        
        elif self.current_phase == IntersectionPhase.EXECUTING_STRAIGHT_MANEUVER:
            target_dist = self.get_parameter('straight_distance').value
            speed = self.get_parameter('straight_speed').value
            if self.drive_straight_distance(target_dist, speed):
                self.change_phase(IntersectionPhase.FINAL_WAIT)

        elif self.current_phase == IntersectionPhase.EXECUTING_LEFT_TURN_MANEUVER_TURN:
            turn_speed = self.get_parameter('turn_angular_speed_left').value
            fwd_speed = self.get_parameter('turn_forward_speed').value
            if self.turn_to_target_yaw(self.target_yaw_for_turn, turn_speed, fwd_speed):
                self.change_phase(IntersectionPhase.EXECUTING_LEFT_TURN_MANEUVER_STRAIGHT)
        
        elif self.current_phase == IntersectionPhase.EXECUTING_LEFT_TURN_MANEUVER_STRAIGHT:
            target_dist = self.get_parameter('post_turn_straight_distance').value
            speed = self.get_parameter('turn_forward_speed').value
            if self.drive_straight_distance(target_dist, speed):
                self.change_phase(IntersectionPhase.FINAL_WAIT)

        elif self.current_phase == IntersectionPhase.EXECUTING_RIGHT_TURN_MANEUVER_TURN:
            turn_speed = -self.get_parameter('turn_angular_speed_right').value
            fwd_speed = self.get_parameter('turn_forward_speed').value
            if self.turn_to_target_yaw(self.target_yaw_for_turn, turn_speed, fwd_speed):
                self.change_phase(IntersectionPhase.EXECUTING_RIGHT_TURN_MANEUVER_STRAIGHT)

        elif self.current_phase == IntersectionPhase.EXECUTING_RIGHT_TURN_MANEUVER_STRAIGHT:
            target_dist = self.get_parameter('post_turn_straight_distance').value
            speed = self.get_parameter('turn_forward_speed').value
            if self.drive_straight_distance(target_dist, speed):
                self.change_phase(IntersectionPhase.FINAL_WAIT)

        elif self.current_phase == IntersectionPhase.FINAL_WAIT:
            self.stop_robot()
            wait_duration = self.get_parameter('final_wait_duration').value
            if elapsed_phase_time >= wait_duration:
                self.change_phase(IntersectionPhase.INTERSECTION_FINISHED)

    def move_robot(self, linear_x: float, angular_z: float):
        twist = Twist(); twist.linear.x = linear_x; twist.angular.z = float(angular_z)
        self.cmd_vel_publisher.publish(twist)

    def stop_robot(self):
        self.move_robot(0.0, 0.0)

    def normalize_angle(self, angle: float) -> float:
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

    def drive_straight_distance(self, target_distance: float, speed: float) -> bool:
        traveled_distance = math.sqrt(
            (self.current_pos_x - self.start_pos_x_segment)**2 +
            (self.current_pos_y - self.start_pos_y_segment)**2
        )
        if traveled_distance >= target_distance - DEFAULT_ODOM_DISTANCE_TOLERANCE:
            self.stop_robot()
            return True
        else:
            self.move_robot(speed, 0.0)
            return False
    
    def follow_lane_to_reference(self):
        """
        Creates and publishes a Twist message to follow the lane based on center_offset
        and a fixed approach speed parameter.
        """
        twist_msg = Twist()
        
        speed = self.get_parameter('approach_speed').value
        twist_msg.linear.x = speed
        
        p_gain = 1.0 # P-Gain kann hier als Parameter hinzugefügt werden
        raw_angular_z = self.current_center_offset * p_gain
        
        max_angular_z = self.get_parameter('max_angular_z_lane_follow').value
        # <<< KORREKTUR: Begrenzung der Winkelgeschwindigkeit hinzugefügt
        clipped_angular_z = np.clip(raw_angular_z, -max_angular_z, max_angular_z)
        twist_msg.angular.z = clipped_angular_z
        
        self.cmd_vel_publisher.publish(twist_msg)

    def turn_to_target_yaw(self, target_yaw: float, angular_speed_cmd: float, forward_speed: float) -> bool:
        angle_diff = self.normalize_angle(target_yaw - self.current_yaw)
        if abs(angle_diff) < DEFAULT_GOAL_TOLERANCE_ANGLE_RAD:
            self.stop_robot()
            return True
        else:
            self.move_robot(forward_speed, angular_speed_cmd)
            return False

    def destroy_node(self):
        super().destroy_node()

STATE_INTERSECTION_DRIVING_STRAIGHT = 'INTERSEC_DRIVING_STRAIGHT'
STATE_INTERSECTION_TURNING_RIGHT = 'INTERSEC_TURNING_RIGHT'
STATE_INTERSECTION_TURNING_LEFT = 'INTERSEC_TURNING_LEFT'
CMD_VEL_TOPIC = '/cmd_vel'
LASERSCAN_TOPIC = '/scan'
ROBOT_STATE_TOPIC = '/robot/state'

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = IntersectionHandlingNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if node: node.get_logger().error(f"FATAL ERROR in {NODE_NAME}: {e}\n{traceback.format_exc()}")
        else: print(f"FATAL ERROR in {NODE_NAME} (pre-init): {e}\n{traceback.format_exc()}", file=sys.stderr)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
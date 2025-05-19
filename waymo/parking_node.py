# parking_node.py
#!/usr/bin/env python3
import sys
import traceback
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float64, Bool
import math
import numpy as np
import time # Beibehalten für phase_start_time
from enum import Enum, auto

# Für Quaternionen-Euler-Umwandlung
from scipy.spatial.transform import Rotation as R


# --- Konstanten ---
NODE_NAME = 'parking_node'
STATE_PARKING = 'PARKING' 
CMD_VEL_TOPIC = '/cmd_vel'
LASERSCAN_TOPIC = '/scan'
ODOM_TOPIC = '/odom'
ROBOT_STATE_TOPIC = '/robot/state'
LANE_OFFSET_TOPIC = '/lane/center_offset'
PARKING_FINISHED_TOPIC = '/parking/finished'

# Fahrparameter
PARKING_LINEAR_SPEED = 0.1 # Einheitliche lineare Geschwindigkeit
MANEUVER_ANGULAR_SPEED_TURN = 0.8
MAX_ANGULAR_Z_LANE_FOLLOW = 0.35

# Laserscan Parameter
INITIAL_SCAN_ANGLE_MIN_DEG = 88.0
INITIAL_SCAN_ANGLE_MAX_DEG = 92.0
INITIAL_SCAN_ANGLE_MIN_RAD = math.radians(INITIAL_SCAN_ANGLE_MIN_DEG)
INITIAL_SCAN_ANGLE_MAX_RAD = math.radians(INITIAL_SCAN_ANGLE_MAX_DEG)
INITIAL_SPOT_DETECTION_DISTANCE = 0.25

SPOT_SCAN_ANGLE_MIN_DEG = 75.0
SPOT_SCAN_ANGLE_MAX_DEG = 105.0
SPOT_SCAN_ANGLE_MIN_RAD = math.radians(SPOT_SCAN_ANGLE_MIN_DEG)
SPOT_SCAN_ANGLE_MAX_RAD = math.radians(SPOT_SCAN_ANGLE_MAX_DEG)
PARKING_SPOT_CLEAR_DISTANCE = 0.4

# Zeit- und Distanzparameter
INITIAL_STOP_DURATION = 1.0
DRIVE_TO_FIRST_SPOT_DURATION = 5.2
DRIVE_TO_NEXT_SPOT_DURATION = 3.35
STOP_BEFORE_SCAN_DURATION = 1.0
PARKING_DURATION = 3.0
MOVE_SPOT_DISTANCE = 0.3
TURN_ANGLE_90_DEG = math.radians(90.0 - 2.0)
GOAL_TOLERANCE_ANGLE_RAD = math.radians(2.5)

MAX_PARKING_ATTEMPTS = 3


class ParkingPhase(Enum):
    IDLE = auto()
    DRIVING_FOR_INITIAL_LASER_SCAN = auto()
    INITIAL_SIGN_STOPPING = auto()
    INITIAL_SIGN_WAITING = auto()
    DRIVING_TO_SCAN_POINT = auto()
    STOPPING_BEFORE_SCAN = auto()
    SCANNING_FOR_SPOT = auto()
    TURNING_RIGHT_FOR_PARKING = auto()
    MOVING_INTO_SPOT = auto()
    TURNING_LEFT_IN_SPOT_TO_PARK = auto()
    WAITING_IN_SPOT = auto()
    TURNING_LEFT_IN_SPOT_TO_GET_OUT = auto()
    MOVING_OUT_OF_SPOT = auto()
    TURNING_RIGHT_FOR_LANE_FOLLOWING = auto()
    MANEUVER_COMPLETE = auto()


class ParkingNode(Node):
    def __init__(self):
        super().__init__(NODE_NAME)

        self.current_robot_state_from_manager = ""
        self.parking_phase = ParkingPhase.IDLE
        self.current_center_offset = 0.0
        self.initial_sign_laser_detected = False

        self.current_yaw = 0.0
        self.start_yaw_for_turn = 0.0
        self.target_yaw_for_turn = 0.0
        
        self.phase_start_time = 0.0
        self.maneuver_drive_duration = 0.0

        self.parking_attempts_count = 0

        qos_reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1)
        qos_best_effort = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)

        self.state_subscriber = self.create_subscription(String, ROBOT_STATE_TOPIC, self.robot_state_manager_callback, qos_reliable)
        self.scan_subscriber = self.create_subscription(LaserScan, LASERSCAN_TOPIC, self.scan_callback, qos_best_effort)
        self.offset_subscriber = self.create_subscription(Float64, LANE_OFFSET_TOPIC, self.lane_offset_callback, qos_best_effort)
        self.odom_subscriber = self.create_subscription(Odometry, ODOM_TOPIC, self.odom_callback, qos_best_effort)

        self.cmd_vel_publisher = self.create_publisher(Twist, CMD_VEL_TOPIC, qos_reliable)
        self.parking_finished_publisher = self.create_publisher(Bool, PARKING_FINISHED_TOPIC, qos_reliable)

        self.control_timer_period = 0.005
        self.control_timer = self.create_timer(self.control_timer_period, self.parking_sequence_controller)

    def robot_state_manager_callback(self, msg: String):
        new_state_from_manager = msg.data
        if self.current_robot_state_from_manager != new_state_from_manager:
            self.current_robot_state_from_manager = new_state_from_manager

            if self.current_robot_state_from_manager == STATE_PARKING and \
               (self.parking_phase == ParkingPhase.IDLE or self.parking_phase == ParkingPhase.MANEUVER_COMPLETE):
                self.initial_sign_laser_detected = False 
                self.parking_attempts_count = 0
                self.change_parking_phase(ParkingPhase.DRIVING_FOR_INITIAL_LASER_SCAN)
            elif self.current_robot_state_from_manager != STATE_PARKING and self.parking_phase != ParkingPhase.IDLE:
                self.reset_parking_sequence()

    def reset_parking_sequence(self):
        self.change_parking_phase(ParkingPhase.IDLE)
        self.initial_sign_laser_detected = False
        self.parking_attempts_count = 0
        self.stop_robot() 

    def lane_offset_callback(self, msg: Float64):
        self.current_center_offset = msg.data

    def odom_callback(self, msg: Odometry):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        try:
            r = R.from_quat(orientation_list)
            euler = r.as_euler('xyz', degrees=False)
            self.current_yaw = euler[2] 
        except Exception as e:
            self.get_logger().warn(f"Error in odom_callback quaternion conversion: {e}", throttle_duration_sec=5)
            pass

    def scan_callback(self, msg: LaserScan):
        if self.current_robot_state_from_manager != STATE_PARKING:
            return

        if self.parking_phase == ParkingPhase.DRIVING_FOR_INITIAL_LASER_SCAN and not self.initial_sign_laser_detected:
            obstacle_detected_at_initial_spot = self.check_laser_zone(
                msg, INITIAL_SCAN_ANGLE_MIN_RAD, INITIAL_SCAN_ANGLE_MAX_RAD, INITIAL_SPOT_DETECTION_DISTANCE
            )
            if obstacle_detected_at_initial_spot:
                self.initial_sign_laser_detected = True
                self.change_parking_phase(ParkingPhase.INITIAL_SIGN_STOPPING)

        elif self.parking_phase == ParkingPhase.SCANNING_FOR_SPOT:
            is_spot_clear = not self.check_laser_zone( 
                msg, SPOT_SCAN_ANGLE_MIN_RAD, SPOT_SCAN_ANGLE_MAX_RAD, PARKING_SPOT_CLEAR_DISTANCE
            )
            if is_spot_clear:
                self.change_parking_phase(ParkingPhase.TURNING_RIGHT_FOR_PARKING)
            else:
                self.parking_attempts_count += 1
                if self.parking_attempts_count >= MAX_PARKING_ATTEMPTS:
                    self.change_parking_phase(ParkingPhase.MANEUVER_COMPLETE) 
                else:
                    self.change_parking_phase(ParkingPhase.DRIVING_TO_SCAN_POINT)

    def check_laser_zone(self, scan_msg: LaserScan, angle_min_rad_target: float, angle_max_rad_target: float, detection_distance: float) -> bool:
        if scan_msg.angle_increment <= 0.0:
            self.get_logger().warn("Invalid angle_increment in laser scan.", throttle_duration_sec=10)
            return False
        
        actual_scan_angle_max_rad = scan_msg.angle_min + (len(scan_msg.ranges) - 1) * scan_msg.angle_increment
        adj_target_min = max(angle_min_rad_target, scan_msg.angle_min)
        adj_target_max = min(angle_max_rad_target, actual_scan_angle_max_rad)

        if adj_target_min >= adj_target_max:
            return False

        start_index = max(0, int((adj_target_min - scan_msg.angle_min) / scan_msg.angle_increment))
        end_index = min(len(scan_msg.ranges) - 1, int((adj_target_max - scan_msg.angle_min) / scan_msg.angle_increment))

        if start_index > end_index:
            return False

        for i in range(start_index, end_index + 1):
            dist = scan_msg.ranges[i]
            if not math.isinf(dist) and not math.isnan(dist) and \
               dist >= scan_msg.range_min and dist <= scan_msg.range_max and \
               dist < detection_distance:
                return True 
        return False

    def change_parking_phase(self, new_phase: ParkingPhase):
        if self.parking_phase != new_phase:
            self.parking_phase = new_phase
            self.phase_start_time = self.get_clock().now().nanoseconds / 1e9

            if \
            new_phase == ParkingPhase.TURNING_RIGHT_FOR_PARKING or \
            new_phase == ParkingPhase.TURNING_LEFT_IN_SPOT_TO_PARK or \
            new_phase == ParkingPhase.TURNING_LEFT_IN_SPOT_TO_GET_OUT or \
            new_phase == ParkingPhase.TURNING_RIGHT_FOR_LANE_FOLLOWING:
                self.start_yaw_for_turn = self.current_yaw
                if new_phase == ParkingPhase.TURNING_RIGHT_FOR_PARKING or new_phase == ParkingPhase.TURNING_RIGHT_FOR_LANE_FOLLOWING:
                    angle_to_turn = -TURN_ANGLE_90_DEG 
                else: 
                    angle_to_turn = TURN_ANGLE_90_DEG  
                self.target_yaw_for_turn = self.normalize_angle(self.start_yaw_for_turn + angle_to_turn)
            elif new_phase == ParkingPhase.MOVING_INTO_SPOT or new_phase == ParkingPhase.MOVING_OUT_OF_SPOT:
                if PARKING_LINEAR_SPEED > 0:
                    self.maneuver_drive_duration = MOVE_SPOT_DISTANCE / PARKING_LINEAR_SPEED
                else:
                    self.maneuver_drive_duration = float('inf') 
                    self.get_logger().error("PARKING_LINEAR_SPEED is 0, cannot calculate maneuver_drive_duration for MOVING_INTO_SPOT.")
            elif new_phase == ParkingPhase.DRIVING_TO_SCAN_POINT:
                if self.parking_attempts_count == 0: 
                     self.maneuver_drive_duration = DRIVE_TO_FIRST_SPOT_DURATION
                else: 
                     self.maneuver_drive_duration = DRIVE_TO_NEXT_SPOT_DURATION
            elif new_phase == ParkingPhase.WAITING_IN_SPOT:
                self.maneuver_drive_duration = PARKING_DURATION
            
            if new_phase in [ParkingPhase.INITIAL_SIGN_STOPPING,
                               ParkingPhase.INITIAL_SIGN_WAITING,
                               ParkingPhase.STOPPING_BEFORE_SCAN,
                               ParkingPhase.SCANNING_FOR_SPOT,
                               ParkingPhase.MANEUVER_COMPLETE,
                               ParkingPhase.IDLE]:
                self.stop_robot()

    def parking_sequence_controller(self):
        if self.current_robot_state_from_manager != STATE_PARKING:
            if self.parking_phase != ParkingPhase.IDLE:
                self.reset_parking_sequence()
            return 

        if self.parking_phase == ParkingPhase.MANEUVER_COMPLETE:
            self.stop_robot()
            self.parking_finished_publisher.publish(Bool(data=True)) # Signalisiere Abschluss
            return
        
        current_time = self.get_clock().now().nanoseconds / 1e9
        elapsed_phase_time = current_time - self.phase_start_time
        
        if self.parking_phase == ParkingPhase.IDLE:
            self.stop_robot()
            return

        if self.parking_phase == ParkingPhase.DRIVING_FOR_INITIAL_LASER_SCAN:
            if not self.initial_sign_laser_detected:
                self.follow_lane_slowly()
            else:
                self.stop_robot()
            return 

        if self.parking_phase == ParkingPhase.INITIAL_SIGN_STOPPING:
            self.stop_robot() 
            self.change_parking_phase(ParkingPhase.INITIAL_SIGN_WAITING)

        elif self.parking_phase == ParkingPhase.INITIAL_SIGN_WAITING:
            self.stop_robot() 
            if elapsed_phase_time >= INITIAL_STOP_DURATION:
                self.parking_attempts_count = 0 
                self.change_parking_phase(ParkingPhase.DRIVING_TO_SCAN_POINT)

        elif self.parking_phase == ParkingPhase.DRIVING_TO_SCAN_POINT:
            if elapsed_phase_time < self.maneuver_drive_duration:
                self.follow_lane_slowly()
            else:
                self.stop_robot() 
                self.change_parking_phase(ParkingPhase.STOPPING_BEFORE_SCAN)
        
        elif self.parking_phase == ParkingPhase.STOPPING_BEFORE_SCAN: 
            self.stop_robot()
            if elapsed_phase_time >= STOP_BEFORE_SCAN_DURATION:
                self.change_parking_phase(ParkingPhase.SCANNING_FOR_SPOT)

        elif self.parking_phase == ParkingPhase.SCANNING_FOR_SPOT:
            self.stop_robot() 
            pass 

        elif self.parking_phase == ParkingPhase.TURNING_RIGHT_FOR_PARKING:
            if self.turn_to_target(self.target_yaw_for_turn, MANEUVER_ANGULAR_SPEED_TURN):
                self.stop_robot() 
                self.change_parking_phase(ParkingPhase.MOVING_INTO_SPOT)
        
        elif self.parking_phase == ParkingPhase.MOVING_INTO_SPOT:
            if elapsed_phase_time < self.maneuver_drive_duration:
                self.move_straight(PARKING_LINEAR_SPEED) 
            else:
                self.stop_robot() 
                self.change_parking_phase(ParkingPhase.TURNING_LEFT_IN_SPOT_TO_PARK)

        elif self.parking_phase == ParkingPhase.TURNING_LEFT_IN_SPOT_TO_PARK:
            if self.turn_to_target(self.target_yaw_for_turn, MANEUVER_ANGULAR_SPEED_TURN):
                self.stop_robot()
                self.change_parking_phase(ParkingPhase.WAITING_IN_SPOT)

        elif self.parking_phase == ParkingPhase.WAITING_IN_SPOT:
            self.stop_robot()
            if elapsed_phase_time >= self.maneuver_drive_duration:
                self.change_parking_phase(ParkingPhase.TURNING_LEFT_IN_SPOT_TO_GET_OUT)

        elif self.parking_phase == ParkingPhase.TURNING_LEFT_IN_SPOT_TO_GET_OUT:
            if self.turn_to_target(self.target_yaw_for_turn, MANEUVER_ANGULAR_SPEED_TURN):
                self.stop_robot()
                self.change_parking_phase(ParkingPhase.MOVING_OUT_OF_SPOT)
        
        elif self.parking_phase == ParkingPhase.MOVING_OUT_OF_SPOT:
            if elapsed_phase_time < self.maneuver_drive_duration:
                self.move_straight(PARKING_LINEAR_SPEED)
            else:
                self.stop_robot()
                self.change_parking_phase(ParkingPhase.TURNING_RIGHT_FOR_LANE_FOLLOWING)
        
        elif self.parking_phase == ParkingPhase.TURNING_RIGHT_FOR_LANE_FOLLOWING:
            if self.turn_to_target(self.target_yaw_for_turn, MANEUVER_ANGULAR_SPEED_TURN):
                self.stop_robot()
                if elapsed_phase_time >= self.maneuver_drive_duration:
                    self.change_parking_phase(ParkingPhase.MANEUVER_COMPLETE)
        
    def follow_lane_slowly(self):
        twist_msg = Twist()
        twist_msg.linear.x = PARKING_LINEAR_SPEED
        raw_angular_z = self.current_center_offset 
        twist_msg.angular.z = float(np.clip(raw_angular_z, -MAX_ANGULAR_Z_LANE_FOLLOW, MAX_ANGULAR_Z_LANE_FOLLOW))
        self.cmd_vel_publisher.publish(twist_msg)

    def move_straight(self, speed: float):
        twist_msg = Twist()
        twist_msg.linear.x = speed
        twist_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist_msg)

    def turn_robot(self, angular_speed: float):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = angular_speed
        self.cmd_vel_publisher.publish(twist_msg)
        
    def stop_robot(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist_msg)

    def normalize_angle(self, angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def turn_to_target(self, target_yaw: float, angular_speed_abs: float) -> bool:
        angle_diff = self.normalize_angle(target_yaw - self.current_yaw)
        
        if abs(angle_diff) < GOAL_TOLERANCE_ANGLE_RAD:
            self.stop_robot() 
            return True
        else:
            turn_speed_actual = angular_speed_abs
            if angular_speed_abs == 0:
                 # self.get_logger().warn("turn_to_target called with angular_speed_abs = 0. Cannot turn.") # Wie gewünscht auskommentiert
                 self.stop_robot()
                 return False 

            if angle_diff > 0: 
                self.turn_robot(turn_speed_actual)
            else: 
                self.turn_robot(-turn_speed_actual)
            return False

    def destroy_node(self):
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    parking_node = None
    try:
        parking_node = ParkingNode()
        rclpy.spin(parking_node)
    except KeyboardInterrupt:
        pass 
    except Exception as e:
        if parking_node: 
            parking_node.get_logger().error(f"FATAL ERROR in {NODE_NAME}: {e}\n{traceback.format_exc()}")
        else: 
            print(f"FATAL ERROR in {NODE_NAME} (pre-init or during init): {e}\n{traceback.format_exc()}", file=sys.stderr)
    finally:
        if parking_node:
            parking_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
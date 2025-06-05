import rclpy
import rclpy.node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Bool, Float64 # Float64 hinzugefügt
import math
import time
import sys
import numpy as np # Für np.clip
import traceback # Für Exception-Ausgabe in Main
from scipy.spatial.transform import Rotation as R

# --- Konstanten (Werte aus deinem letzten Code) ---
LINEAR_SPEED = 0.15
ANGULAR_SPEED = 0.8
SIDEWAYS_DISTANCE = 0.28
TURN_ANGLE_90_DEG = math.pi / 2
RECOMMENDED_SPEED_TOPIC = '/robot/recommended_speed'

# --- SCAN-WINKEL (POSITIV - wie in deinem Code) ---
SIDE_SCAN_ANGLE_MIN_DEG = 70.0 # Grad
SIDE_SCAN_ANGLE_MAX_DEG = 135.0 # Grad
SIDE_SCAN_ANGLE_MIN = math.radians(SIDE_SCAN_ANGLE_MIN_DEG)
SIDE_SCAN_ANGLE_MAX = math.radians(SIDE_SCAN_ANGLE_MAX_DEG)

# --- Abstandsschwellenwert (wie in deinem Code) ---
SIDE_SCAN_CLEAR_DISTANCE = 0.40 # Meter

# --- Wait Duration (wie in deinem Code) ---
WAIT_DURATION_BEFORE_CHECK = 0.0 # Sekunden

GOAL_TOLERANCE_ANGLE = 0.05
MOVE_DURATION_SIDEWAYS = SIDEWAYS_DISTANCE / LINEAR_SPEED
LOG_THROTTLE_DURATION = 1.0 # Wird nicht mehr gebraucht ohne Logs
MAX_ANGULAR_Z_PASSING = 0.8

class ManeuverState:
    IDLE = 0; TURNING_LEFT_1 = 1; MOVING_SIDEWAYS_1 = 2; TURNING_RIGHT_1 = 3
    WAIT_BEFORE_CHECKING = 4; CHECKING_SIDE = 5; TURNING_RIGHT_2 = 6
    MOVING_SIDEWAYS_2 = 7; TURNING_LEFT_2 = 8; COMPLETE = 9; ABORTED = 10

class PassingObstacleNode(rclpy.node.Node):

    def __init__(self):
        super().__init__('passing_obstacle_node')

        self.declare_parameter('fallback_passing_speed', 0.1)

        self.maneuver_state = ManeuverState.IDLE
        self.current_yaw = 0.0
        self.start_yaw = 0.0
        self.target_yaw = 0.0
        self.start_time = 0.0
        self.wait_start_time = 0.0
        self.side_is_clear = False
        self.maneuver_active = False
        # self.last_log_time = 0.0 # Nicht mehr gebraucht
        self.current_center_offset = 0.0

        self.recommended_speed = self.get_parameter('fallback_passing_speed').value

        # QoS Profile (Gemischt)
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1
        )
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=1
        )

        # Subscribers
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, qos_sensor)
        self.scan_subscriber = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_sensor)
        self.state_subscriber = self.create_subscription(String, '/robot/state', self.robot_state_callback, qos_reliable)
        self.offset_subscription = self.create_subscription(Float64,'/lane/center_offset',self.lane_offset_callback, qos_sensor)

        self.recommended_speed_subscriber = self.create_subscription(Float64, RECOMMENDED_SPEED_TOPIC, self.recommended_speed_callback, qos_reliable)

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', qos_reliable)
        self.passed_publisher = self.create_publisher(Bool, '/obstacle/passed', qos_reliable)

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_maneuver)

        # Initiale Logs entfernt

    def recommended_speed_callback(self, msg: Float64):
        self.recommended_speed = msg.data
    
    def odom_callback(self, msg: Odometry):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        try: r = R.from_quat(orientation_list); euler = r.as_euler('xyz', degrees=False); self.current_yaw = euler[2]
        except Exception as e:
            # Error Log entfernt
            self.current_yaw = 0.0

    def scan_callback(self, msg: LaserScan):
        """Prüft Seite basierend auf POSITIVEN Winkeln."""
        if self.maneuver_state != ManeuverState.CHECKING_SIDE: return
        target_angle_min_rad = SIDE_SCAN_ANGLE_MIN; target_angle_max_rad = SIDE_SCAN_ANGLE_MAX
        min_index = -1 # Initialisieren für Fehlerbehandlung
        max_index = -1
        try:
            scan_angle_min_rad = msg.angle_min; scan_angle_max_rad = msg.angle_max
            angle_increment_rad = msg.angle_increment; num_ranges = len(msg.ranges)
            if angle_increment_rad <= 0.0: self.side_is_clear = False; return # Fehler ohne Log
            adj_target_min = max(target_angle_min_rad, scan_angle_min_rad); adj_target_max = min(target_angle_max_rad, scan_angle_max_rad)
            min_index = max(0, int((adj_target_min - scan_angle_min_rad) / angle_increment_rad))
            max_index = min(num_ranges - 1, int((adj_target_max - scan_angle_min_rad) / angle_increment_rad))
            if min_index > max_index:
                 self.side_is_clear = False; return # Fehler ohne Log
            min_dist_in_range = float('inf'); valid_ranges_found = False
            for i in range(min_index, max_index + 1):
                dist = msg.ranges[i]
                if not math.isinf(dist) and not math.isnan(dist) and dist > 0.01: valid_ranges_found = True; min_dist_in_range = min(min_dist_in_range, dist)
            # previously_clear = self.side_is_clear # Nicht mehr für Log gebraucht
            clear_distance_threshold = SIDE_SCAN_CLEAR_DISTANCE
            if not valid_ranges_found:
                 self.side_is_clear = False
                 # Warn Log entfernt
            elif min_dist_in_range < clear_distance_threshold:
                 self.side_is_clear = False
                 # Info/Debug Logs entfernt
            else:
                 self.side_is_clear = True
                 # Info/Debug Logs entfernt
        except IndexError: # Fehler ohne Log
             self.side_is_clear = False
        except Exception as e: # Fehler ohne Log
             self.side_is_clear = False

    def lane_offset_callback(self, msg: Float64):
        """Speichert den aktuellen Spurversatz."""
        self.current_center_offset = msg.data

    def robot_state_callback(self, msg: String):
        """Verarbeitet Zustandsänderungen vom State Manager."""
        new_state = msg.data
        if new_state == 'PASSING_OBSTACLE' and not self.maneuver_active:
            # Info Log entfernt
            self.maneuver_active = True; self.maneuver_state = ManeuverState.TURNING_LEFT_1
            passed_msg = Bool(); passed_msg.data = False; self.passed_publisher.publish(passed_msg)
            self.start_yaw = self.current_yaw; self.target_yaw = self.normalize_angle(self.start_yaw + TURN_ANGLE_90_DEG)
            # Info Log entfernt
        elif new_state != 'PASSING_OBSTACLE' and self.maneuver_active:
            # Warn Log entfernt
            self.reset_maneuver()

    def run_maneuver(self):
        """Hauptlogik der Zustandsmaschine."""
        if not self.maneuver_active or self.maneuver_state == ManeuverState.IDLE:
             if self.maneuver_state == ManeuverState.IDLE: self.stop_robot()
             return
        current_time = time.time() # state_changed nicht mehr nötig

        # --- Zustandsmaschine (ohne Logs) ---
        if self.maneuver_state == ManeuverState.TURNING_LEFT_1:
            if self.turn_to_target(self.target_yaw, ANGULAR_SPEED): self.maneuver_state = ManeuverState.MOVING_SIDEWAYS_1; self.start_time = current_time
        elif self.maneuver_state == ManeuverState.MOVING_SIDEWAYS_1:
            if self.move_forward_for_duration(LINEAR_SPEED, MOVE_DURATION_SIDEWAYS, current_time): self.maneuver_state = ManeuverState.TURNING_RIGHT_1; self.start_yaw = self.current_yaw; self.target_yaw = self.normalize_angle(self.start_yaw - TURN_ANGLE_90_DEG)
        elif self.maneuver_state == ManeuverState.TURNING_RIGHT_1:
            if self.turn_to_target(self.target_yaw, -ANGULAR_SPEED): self.maneuver_state = ManeuverState.WAIT_BEFORE_CHECKING; self.wait_start_time = current_time; self.stop_robot()
        elif self.maneuver_state == ManeuverState.WAIT_BEFORE_CHECKING:
            if current_time - self.wait_start_time >= WAIT_DURATION_BEFORE_CHECK: self.maneuver_state = ManeuverState.CHECKING_SIDE; self.side_is_clear = False; # self.last_log_time = 0.0
        elif self.maneuver_state == ManeuverState.CHECKING_SIDE:
            angular_z = self.current_center_offset
            angular_z = np.clip(angular_z, -MAX_ANGULAR_Z_PASSING, MAX_ANGULAR_Z_PASSING)
            self.move_robot(self.recommended_speed, angular_z) # Lane Following
            if self.side_is_clear:
                self.stop_robot(); self.maneuver_state = ManeuverState.TURNING_RIGHT_2
                self.start_yaw = self.current_yaw; self.target_yaw = self.normalize_angle(self.start_yaw - TURN_ANGLE_90_DEG)
        elif self.maneuver_state == ManeuverState.TURNING_RIGHT_2:
             if self.turn_to_target(self.target_yaw, -ANGULAR_SPEED): self.maneuver_state = ManeuverState.MOVING_SIDEWAYS_2; self.start_time = current_time
        elif self.maneuver_state == ManeuverState.MOVING_SIDEWAYS_2:
            if self.move_forward_for_duration(LINEAR_SPEED, MOVE_DURATION_SIDEWAYS, current_time): self.maneuver_state = ManeuverState.TURNING_LEFT_2; self.start_yaw = self.current_yaw; self.target_yaw = self.normalize_angle(self.start_yaw + TURN_ANGLE_90_DEG)
        elif self.maneuver_state == ManeuverState.TURNING_LEFT_2:
            if self.turn_to_target(self.target_yaw, ANGULAR_SPEED): self.maneuver_state = ManeuverState.COMPLETE
        elif self.maneuver_state == ManeuverState.COMPLETE:
            self.stop_robot(); passed_msg = Bool(); passed_msg.data = True; self.passed_publisher.publish(passed_msg); self.reset_maneuver()


    def turn_to_target(self, target_yaw, angular_speed):
        angle_diff = self.normalize_angle(target_yaw - self.current_yaw)
        if abs(angle_diff) < GOAL_TOLERANCE_ANGLE:
            self.stop_robot(); return True
        else:
            actual_speed = angular_speed
            if abs(angle_diff) < 0.3: actual_speed *= 0.5
            self.move_robot(0.0, actual_speed); return False

    def move_forward_for_duration(self, linear_speed, duration, current_time):
        elapsed_time = current_time - self.start_time
        if elapsed_time < duration:
            self.move_robot(linear_speed, 0.0); return False
        else: self.stop_robot(); return True

    def move_robot(self, linear_x, angular_z):
        twist = Twist(); twist.linear.x = linear_x; twist.angular.z = float(angular_z)
        self.cmd_vel_publisher.publish(twist)

    def stop_robot(self):
        self.move_robot(0.0, 0.0)

    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

    def reset_maneuver(self):
        self.stop_robot(); self.maneuver_state = ManeuverState.IDLE; self.maneuver_active = False; self.side_is_clear = False
        # Info Log entfernt

    def destroy_node(self):
        try:
            if rclpy.ok() and self.context.ok(): self.stop_robot(); time.sleep(0.1)
        except Exception as e: pass # Fehler ignorieren
        # Info Log entfernt
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args); node = None
    try: node = PassingObstacleNode(); rclpy.spin(node)
    except KeyboardInterrupt: pass # Log entfernt
    except Exception as e:
        # Log to stderr instead of ROS logger
        print(f"Unhandled exception in PassingObstacleNode: {e}", file=sys.stderr); traceback.print_exc()
        # Error Log entfernt
    finally:
        if node is not None: node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()
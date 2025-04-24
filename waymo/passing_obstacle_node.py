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
from scipy.spatial.transform import Rotation as R

# --- Konstanten ---
LINEAR_SPEED = 0.15
ANGULAR_SPEED = 0.8
SIDEWAYS_DISTANCE = 0.28
TURN_ANGLE_90_DEG = math.pi / 2

# --- SCAN-WINKEL (POSITIV - wie vom User gewünscht) ---
SIDE_SCAN_ANGLE_MIN_DEG = 70.0 # Grad - Vorne Links (laut User-Wunsch)
SIDE_SCAN_ANGLE_MAX_DEG = 135.0 # Grad - Hinten Links (laut User-Wunsch)
SIDE_SCAN_ANGLE_MIN = math.radians(SIDE_SCAN_ANGLE_MIN_DEG)
SIDE_SCAN_ANGLE_MAX = math.radians(SIDE_SCAN_ANGLE_MAX_DEG)

# --- Abstandsschwellenwert (wie vom User gewünscht) ---
SIDE_SCAN_CLEAR_DISTANCE = 0.35 # Meter

GOAL_TOLERANCE_ANGLE = 0.05
MOVE_DURATION_SIDEWAYS = SIDEWAYS_DISTANCE / LINEAR_SPEED
LOG_THROTTLE_DURATION = 1.0
WAIT_DURATION_BEFORE_CHECK = 1.0
MAX_ANGULAR_Z_PASSING = 0.8

class ManeuverState:
    IDLE = 0; TURNING_LEFT_1 = 1; MOVING_SIDEWAYS_1 = 2; TURNING_RIGHT_1 = 3
    WAIT_BEFORE_CHECKING = 4; CHECKING_SIDE = 5; TURNING_RIGHT_2 = 6
    MOVING_SIDEWAYS_2 = 7; TURNING_LEFT_2 = 8; COMPLETE = 9; ABORTED = 10

class PassingObstacleNode(rclpy.node.Node):

    def __init__(self):
        super().__init__('passing_obstacle_node')

        self.maneuver_state = ManeuverState.IDLE
        self.current_yaw = 0.0
        self.start_yaw = 0.0
        self.target_yaw = 0.0
        self.start_time = 0.0
        self.wait_start_time = 0.0
        self.side_is_clear = False
        self.maneuver_active = False
        self.last_log_time = 0.0
        self.current_center_offset = 0.0

        # --- QoS Profile (Korrekte gemischte Profile) ---
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1
        )
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=1
        )
        # --- Ende QoS ---

        # Subscribers
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, qos_sensor)
        self.scan_subscriber = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_sensor)
        self.state_subscriber = self.create_subscription(String, '/robot/state', self.robot_state_callback, qos_reliable)
        self.offset_subscription = self.create_subscription(Float64,'/lane/center_offset',self.lane_offset_callback, qos_sensor)

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', qos_reliable)
        self.passed_publisher = self.create_publisher(Bool, '/obstacle/passed', qos_reliable)

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_maneuver)

        self.get_logger().info(f"Passing Obstacle Node initialized (with Lane Following).")
        # Logge die Parameter, die jetzt verwendet werden
        self.get_logger().info(f"--- Side Scan Angles: {SIDE_SCAN_ANGLE_MIN_DEG:.1f}° to {SIDE_SCAN_ANGLE_MAX_DEG:.1f}°")
        self.get_logger().info(f"--- Side Scan Clear Distance: {SIDE_SCAN_CLEAR_DISTANCE:.2f}m")
        self.get_logger().info(f"--- Sideways Move Distance: {SIDEWAYS_DISTANCE:.2f}m (Duration: {MOVE_DURATION_SIDEWAYS:.2f}s)")
        self.get_logger().info(f"--- Wait Before Check: {WAIT_DURATION_BEFORE_CHECK:.1f}s")

    def odom_callback(self, msg: Odometry):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        try:
            r = R.from_quat(orientation_list)
            euler = r.as_euler('xyz', degrees=False)
            self.current_yaw = euler[2]
        except Exception as e:
            self.get_logger().error(f"Error converting quaternion to Euler: {e}")
            self.current_yaw = 0.0

    def scan_callback(self, msg: LaserScan):
        """Prüft Seite basierend auf POSITIVEN Winkeln."""
        if self.maneuver_state != ManeuverState.CHECKING_SIDE: return

        target_angle_min_rad = SIDE_SCAN_ANGLE_MIN # Sollte jetzt radians(45.0) sein
        target_angle_max_rad = SIDE_SCAN_ANGLE_MAX # Sollte jetzt radians(100.0) sein

        try:
            scan_angle_min_rad = msg.angle_min
            scan_angle_max_rad = msg.angle_max
            angle_increment_rad = msg.angle_increment
            num_ranges = len(msg.ranges)

            if angle_increment_rad <= 0.0: self.get_logger().error("Invalid angle_increment."); self.side_is_clear = False; return

            adj_target_min = max(target_angle_min_rad, scan_angle_min_rad)
            adj_target_max = min(target_angle_max_rad, scan_angle_max_rad)
            min_index = max(0, int((adj_target_min - scan_angle_min_rad) / angle_increment_rad))
            max_index = min(num_ranges - 1, int((adj_target_max - scan_angle_min_rad) / angle_increment_rad))

            # Bei positiven Winkeln ist min > max normal, wenn sie über 0 gehen! Korrektur nötig? Nein, Indexberechnung sollte stimmen.
            # ABER: Wir prüfen trotzdem sicherheitshalber, ob der Bereich gültig ist (max_index >= min_index)
            if min_index > max_index:
                 # Diese Warnung sollte bei korrekter Indexberechnung nicht mehr auftreten, es sei denn, der Bereich ist komplett außerhalb des Scans
                 log_now = time.time() - self.last_log_time > LOG_THROTTLE_DURATION
                 if log_now: self.get_logger().warn(f"Invalid angle indices {min_index} > {max_index}. Check angles/scan data."); self.last_log_time = time.time()
                 self.side_is_clear = False; return

            min_dist_in_range = float('inf'); valid_ranges_found = False
            for i in range(min_index, max_index + 1):
                dist = msg.ranges[i]
                if not math.isinf(dist) and not math.isnan(dist) and dist > 0.01:
                    valid_ranges_found = True
                    min_dist_in_range = min(min_dist_in_range, dist)

            previously_clear = self.side_is_clear
            clear_distance_threshold = SIDE_SCAN_CLEAR_DISTANCE # Jetzt 0.35m

            if not valid_ranges_found:
                 self.side_is_clear = False
                 if previously_clear: self.get_logger().warn(f"No valid ranges {min_index}-{max_index}. BLOCKED.")
            elif min_dist_in_range < clear_distance_threshold:
                 self.side_is_clear = False # Blockiert
                 log_now = time.time() - self.last_log_time > LOG_THROTTLE_DURATION
                 if previously_clear: self.get_logger().info(f"Side->BLOCKED ({min_dist_in_range:.2f}m<{clear_distance_threshold:.2f}m idx {min_index}-{max_index})"); self.last_log_time = time.time()
                 elif log_now: self.get_logger().info(f"[Chk] Still BLOCKED ({min_dist_in_range:.2f}m idx {min_index}-{max_index})"); self.last_log_time = time.time()
            else:
                 self.side_is_clear = True # Frei
                 log_now = time.time() - self.last_log_time > LOG_THROTTLE_DURATION
                 if not previously_clear: self.get_logger().info(f"Side->CLEAR ({min_dist_in_range:.2f}m>={clear_distance_threshold:.2f}m idx {min_index}-{max_index})"); self.last_log_time = time.time()
                 elif log_now: self.get_logger().info(f"[Chk] Still CLEAR ({min_dist_in_range:.2f}m idx {min_index}-{max_index})"); self.last_log_time = time.time()

        except IndexError: self.get_logger().error(f"IndexError {min_index},{max_index} vs {num_ranges}."); self.side_is_clear = False
        except Exception as e: self.get_logger().error(f"Scan error: {e}", exc_info=True); self.side_is_clear = False

    def lane_offset_callback(self, msg: Float64):
        """Speichert den aktuellen Spurversatz."""
        self.current_center_offset = msg.data

    def robot_state_callback(self, msg: String):
        """Verarbeitet Zustandsänderungen vom State Manager."""
        new_state = msg.data
        if new_state == 'PASSING_OBSTACLE' and not self.maneuver_active:
            self.get_logger().info("Received PASSING_OBSTACLE state. Starting maneuver.")
            self.maneuver_active = True; self.maneuver_state = ManeuverState.TURNING_LEFT_1
            passed_msg = Bool(); passed_msg.data = False; self.passed_publisher.publish(passed_msg)
            self.start_yaw = self.current_yaw; self.target_yaw = self.normalize_angle(self.start_yaw + TURN_ANGLE_90_DEG)
            self.get_logger().info(f"Target Yaw Left 1: {math.degrees(self.target_yaw):.1f}°")
        elif new_state != 'PASSING_OBSTACLE' and self.maneuver_active:
            self.get_logger().warn(f"Received external state '{new_state}'. Aborting."); self.reset_maneuver()

    def run_maneuver(self):
        """Hauptlogik der Zustandsmaschine."""
        if not self.maneuver_active or self.maneuver_state == ManeuverState.IDLE:
             if self.maneuver_state == ManeuverState.IDLE: self.stop_robot()
             return
        current_time = time.time(); state_changed = False

        # --- Zustandsmaschine ---
        if self.maneuver_state == ManeuverState.TURNING_LEFT_1:
            if self.turn_to_target(self.target_yaw, ANGULAR_SPEED): self.get_logger().info("Turn Left 1 completed."); self.maneuver_state = ManeuverState.MOVING_SIDEWAYS_1; state_changed = True; self.start_time = current_time
        elif self.maneuver_state == ManeuverState.MOVING_SIDEWAYS_1:
            if self.move_forward_for_duration(LINEAR_SPEED, MOVE_DURATION_SIDEWAYS, current_time): self.get_logger().info("Move Sideways 1 completed."); self.maneuver_state = ManeuverState.TURNING_RIGHT_1; state_changed = True; self.start_yaw = self.current_yaw; self.target_yaw = self.normalize_angle(self.start_yaw - TURN_ANGLE_90_DEG); self.get_logger().info(f"Target Yaw Right 1: {math.degrees(self.target_yaw):.1f}°")
        elif self.maneuver_state == ManeuverState.TURNING_RIGHT_1:
            if self.turn_to_target(self.target_yaw, -ANGULAR_SPEED): self.get_logger().info("Turn Right 1 completed."); self.maneuver_state = ManeuverState.WAIT_BEFORE_CHECKING; state_changed = True; self.wait_start_time = current_time; self.stop_robot(); self.get_logger().info(f"Entering WAIT state for {WAIT_DURATION_BEFORE_CHECK:.1f}s...")
        elif self.maneuver_state == ManeuverState.WAIT_BEFORE_CHECKING:
            if current_time - self.wait_start_time >= WAIT_DURATION_BEFORE_CHECK: self.get_logger().info("Wait finished. Entering CHECKING_SIDE."); self.maneuver_state = ManeuverState.CHECKING_SIDE; state_changed = True; self.side_is_clear = False; self.last_log_time = 0.0
        elif self.maneuver_state == ManeuverState.CHECKING_SIDE:
            angular_z = self.current_center_offset
            angular_z = np.clip(angular_z, -MAX_ANGULAR_Z_PASSING, MAX_ANGULAR_Z_PASSING)
            self.move_robot(LINEAR_SPEED, angular_z) # Lane Following
            if self.side_is_clear:
                self.get_logger().info("Side is reported CLEAR. Proceeding to turn back.")
                self.stop_robot(); self.maneuver_state = ManeuverState.TURNING_RIGHT_2; state_changed = True
                self.start_yaw = self.current_yaw; self.target_yaw = self.normalize_angle(self.start_yaw - TURN_ANGLE_90_DEG)
                self.get_logger().info(f"Target Yaw Right 2: {math.degrees(self.target_yaw):.1f}°")
        elif self.maneuver_state == ManeuverState.TURNING_RIGHT_2:
             if self.turn_to_target(self.target_yaw, -ANGULAR_SPEED): self.get_logger().info("Turn Right 2 completed."); self.maneuver_state = ManeuverState.MOVING_SIDEWAYS_2; state_changed = True; self.start_time = current_time
        elif self.maneuver_state == ManeuverState.MOVING_SIDEWAYS_2:
            if self.move_forward_for_duration(LINEAR_SPEED, MOVE_DURATION_SIDEWAYS, current_time): self.get_logger().info("Move Sideways 2 completed."); self.maneuver_state = ManeuverState.TURNING_LEFT_2; state_changed = True; self.start_yaw = self.current_yaw; self.target_yaw = self.normalize_angle(self.start_yaw + TURN_ANGLE_90_DEG); self.get_logger().info(f"Target Yaw Left 2: {math.degrees(self.target_yaw):.1f}°")
        elif self.maneuver_state == ManeuverState.TURNING_LEFT_2:
            if self.turn_to_target(self.target_yaw, ANGULAR_SPEED): self.get_logger().info("Turn Left 2 completed. Maneuver finished."); self.maneuver_state = ManeuverState.COMPLETE; state_changed = True
        elif self.maneuver_state == ManeuverState.COMPLETE:
            self.stop_robot(); passed_msg = Bool(); passed_msg.data = True; self.passed_publisher.publish(passed_msg); self.get_logger().info("Published passed=True (Reliable)."); self.reset_maneuver()

        # if state_changed: self.get_logger().debug(f"Maneuver state: {self.maneuver_state}")

    def turn_to_target(self, target_yaw, angular_speed):
        """Dreht den Roboter zum Ziel-Gierwinkel."""
        # Korrekte Version mit 'if'
        angle_diff = self.normalize_angle(target_yaw - self.current_yaw)
        if abs(angle_diff) < GOAL_TOLERANCE_ANGLE:
            self.stop_robot()
            return True
        else:
            actual_speed = angular_speed
            if abs(angle_diff) < 0.3: # Vorher war hier ein Tippfehler "If"
                 actual_speed *= 0.5
            self.move_robot(0.0, actual_speed)
            return False

    def move_forward_for_duration(self, linear_speed, duration, current_time):
        """Fährt für eine bestimmte Dauer geradeaus."""
        elapsed_time = current_time - self.start_time
        if elapsed_time < duration:
            self.move_robot(linear_speed, 0.0)
            return False
        else:
            self.stop_robot()
            return True

    def move_robot(self, linear_x, angular_z):
        """Sendet einen Twist-Befehl."""
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_vel_publisher.publish(twist) # Sollte jetzt Reliable sein

    def stop_robot(self):
        """Stoppt den Roboter."""
        self.move_robot(0.0, 0.0)

    def normalize_angle(self, angle):
        """Normalisiert Winkel auf [-pi, pi]."""
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

    def reset_maneuver(self):
        """Setzt Manöverstatus zurück."""
        self.stop_robot(); self.maneuver_state = ManeuverState.IDLE; self.maneuver_active = False; self.side_is_clear = False; self.get_logger().info("Maneuver reset.")

    def destroy_node(self):
        """Aufräumarbeiten."""
        try:
            if rclpy.ok() and self.context.ok(): self.stop_robot(); time.sleep(0.1)
        except Exception as e:
             try: self.get_logger().error(f"Error stopping robot: {e}")
             except Exception: print(f"Error stopping robot: {e}", file=sys.stderr)
        try:
             if hasattr(self, 'get_logger'): self.get_logger().info("Shutting down Passing Node.")
        except Exception: print("Shutting down Passing Node.", file=sys.stderr)
        super().destroy_node()

def main(args=None):
    """Hauptfunktion."""
    rclpy.init(args=args); node = None
    try: node = PassingObstacleNode(); rclpy.spin(node)
    except KeyboardInterrupt:
        # Korrekte Version mit 'if'
        if node: node.get_logger().info("Interrupt received, shutting down.")
    except Exception as e:
        # Korrekte Version mit 'if'
        if node: node.get_logger().error(f"Exception in main loop: {e}", exc_info=True)
        else: print(f"Exception before node init: {e}", file=sys.stderr); import traceback; traceback.print_exc()
    finally:
        if node is not None: node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()
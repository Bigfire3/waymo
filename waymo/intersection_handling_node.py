# intersection_handling_node.py
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Bool, Float64
import math
import time
import sys
import traceback
from enum import Enum, auto

from scipy.spatial.transform import Rotation as R
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, FloatingPointRange, IntegerRange

NODE_NAME = 'intersection_handling_node'

# --- Standardwerte für Parameter ---
# Geschwindigkeiten
DEFAULT_INITIAL_DRIVE_SPEED = 0.1  # m/s, beim Suchen des Seitenschilds
DEFAULT_STRAIGHT_SPEED = 0.1       # m/s, für Geradeausmanöver
DEFAULT_TURN_FORWARD_SPEED = 0.1   # m/s, Vorwärtskomponente beim Abbiegen
DEFAULT_TURN_ANGULAR_SPEED_RIGHT = 0.7 # rad/s
DEFAULT_TURN_ANGULAR_SPEED_LEFT = 0.35 # rad/s

# Distanzen
DEFAULT_STRAIGHT_DISTANCE = 0.70   # Meter
DEFAULT_POST_TURN_STRAIGHT_DISTANCE = 0.05 # Meter (5cm)

# Zeiten
DEFAULT_SIDE_SIGN_SCAN_TIMEOUT = 5.0 # Sekunden
DEFAULT_WAIT_AT_REFERENCE_DURATION = 1.0 # Sekunden
DEFAULT_FINAL_WAIT_DURATION = 1.0      # Sekunden

# Laserscan für rechte Seite (Winkel relativ zur Roboterfront, 0° = vorne, positive Winkel = rechts)
DEFAULT_RIGHT_SIDE_SCAN_ANGLE_MIN_DEG = -88.0 # Grad
DEFAULT_RIGHT_SIDE_SCAN_ANGLE_MAX_DEG = -92.0 # Grad
DEFAULT_RIGHT_SIDE_SCAN_DISTANCE = 0.25    # Meter (Distanz zum Schild)

# Manöver-Parameter
DEFAULT_TURN_ANGLE_90_DEG = math.pi / 2
DEFAULT_GOAL_TOLERANCE_ANGLE_RAD = math.radians(2.5) # Toleranz für Drehmanöver
DEFAULT_ODOM_DISTANCE_TOLERANCE = 0.01     # Meter (1cm)

class IntersectionPhase(Enum):
    IDLE = auto()
    DRIVING_TO_SIDE_SIGN_REFERENCE = auto()
    WAITING_AT_REFERENCE_POINT = auto()
    EXECUTING_STRAIGHT_MANEUVER = auto()
    EXECUTING_LEFT_TURN_MANEUVER_TURN = auto()
    EXECUTING_LEFT_TURN_MANEUVER_STRAIGHT = auto()
    EXECUTING_RIGHT_TURN_MANEUVER_TURN = auto()
    EXECUTING_RIGHT_TURN_MANEUVER_STRAIGHT = auto()
    # POST_MANEUVER_STRAIGHT_DRIVE ist jetzt Teil der LEFT/RIGHT Phasen
    FINAL_WAIT = auto()
    INTERSECTION_FINISHED = auto() # Publiziert Ergebnis und geht zu IDLE
    ABORTING = auto() # Für Fehlerfälle, die False publizieren

class IntersectionHandlingNode(Node):
    def __init__(self):
        super().__init__(NODE_NAME)

        # --- Parameterdeklaration ---
        def float_desc(desc, min_val=0.0, max_val=10.0, step=0.01):
            return ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE, description=desc,
                                       floating_point_range=[FloatingPointRange(from_value=min_val, to_value=max_val, step=step)])
        def int_desc(desc, min_val=0, max_val=360, step=1):
            return ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER, description=desc,
                                       integer_range=[IntegerRange(from_value=min_val, to_value=max_val, step=step)])

        self.declare_parameter('initial_drive_speed', DEFAULT_INITIAL_DRIVE_SPEED, float_desc("Speed when driving to find side sign", max_val=0.5))
        self.declare_parameter('straight_speed', DEFAULT_STRAIGHT_SPEED, float_desc("Speed for straight intersection maneuver", max_val=0.5))
        self.declare_parameter('turn_forward_speed', DEFAULT_TURN_FORWARD_SPEED, float_desc("Forward speed during turning maneuver", max_val=0.5))
        self.declare_parameter('turn_angular_speed_right', DEFAULT_TURN_ANGULAR_SPEED_RIGHT, float_desc("Angular speed for right turn", max_val=1.5))
        self.declare_parameter('turn_angular_speed_left', DEFAULT_TURN_ANGULAR_SPEED_LEFT, float_desc("Angular speed for left turn", max_val=1.5))

        self.declare_parameter('straight_distance', DEFAULT_STRAIGHT_DISTANCE, float_desc("Distance for straight maneuver", max_val=2.0))
        self.declare_parameter('post_turn_straight_distance', DEFAULT_POST_TURN_STRAIGHT_DISTANCE, float_desc("Short straight distance after turning", max_val=0.5))

        self.declare_parameter('side_sign_scan_timeout', DEFAULT_SIDE_SIGN_SCAN_TIMEOUT, float_desc("Timeout for detecting side sign", max_val=20.0))
        self.declare_parameter('wait_at_reference_duration', DEFAULT_WAIT_AT_REFERENCE_DURATION, float_desc("Wait duration after side sign detected", max_val=5.0))
        self.declare_parameter('final_wait_duration', DEFAULT_FINAL_WAIT_DURATION, float_desc("Final wait duration after maneuver", max_val=5.0))

        self.declare_parameter('right_side_scan_angle_min_deg', DEFAULT_RIGHT_SIDE_SCAN_ANGLE_MIN_DEG, float_desc("Min angle for right side scan (degrees, negative for right)", min_val=-180.0, max_val=0.0))
        self.declare_parameter('right_side_scan_angle_max_deg', DEFAULT_RIGHT_SIDE_SCAN_ANGLE_MAX_DEG, float_desc("Max angle for right side scan (degrees, negative for right)", min_val=-180.0, max_val=0.0))
        self.declare_parameter('right_side_scan_distance', DEFAULT_RIGHT_SIDE_SCAN_DISTANCE, float_desc("Detection distance for right side sign", max_val=1.0))

        # Interne Variablen
        self.current_phase = IntersectionPhase.IDLE
        self.active_intersection_state = None # Speichert den auslösenden State (STRAIGHT, LEFT, RIGHT)
        self.maneuver_active_by_statemgr = False # True, wenn StateManager einen Intersection State vorgibt

        self.current_pos_x = 0.0
        self.current_pos_y = 0.0
        self.current_yaw = 0.0
        self.start_pos_x_segment = 0.0
        self.start_pos_y_segment = 0.0
        self.start_yaw_for_turn = 0.0
        self.target_yaw_for_turn = 0.0
        
        self.phase_start_time = 0.0
        self.side_sign_detected_by_laser = False

        # QoS Profile
        qos_sensor = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        qos_reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1)

        # Subscribers
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, qos_sensor)
        self.scan_subscriber = self.create_subscription(LaserScan, LASERSCAN_TOPIC, self.scan_callback, qos_sensor) # LASERSCAN_TOPIC from parking_node
        self.state_subscriber = self.create_subscription(String, ROBOT_STATE_TOPIC, self.robot_state_manager_callback, qos_reliable) # ROBOT_STATE_TOPIC from parking_node

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, CMD_VEL_TOPIC, qos_reliable) # CMD_VEL_TOPIC from parking_node
        self.intersection_finished_publisher = self.create_publisher(Bool, '/intersection/finished', qos_reliable)

        self.control_timer_period = 0.005 # 200 Hz
        self.control_timer = self.create_timer(self.control_timer_period, self.run_intersection_maneuver)

        # self.get_logger().info(f"{NODE_NAME} initialisiert und bereit.")

    def robot_state_manager_callback(self, msg: String):
        new_state_from_manager = msg.data
        
        is_intersection_state = new_state_from_manager in [
            STATE_INTERSECTION_DRIVING_STRAIGHT,
            STATE_INTERSECTION_TURNING_LEFT,
            STATE_INTERSECTION_TURNING_RIGHT
        ]

        if is_intersection_state and not self.maneuver_active_by_statemgr :
            self.get_logger().info(f"Intersection Manöver gestartet durch StateManager: {new_state_from_manager}")
            self.maneuver_active_by_statemgr = True
            self.active_intersection_state = new_state_from_manager
            # Nur starten, wenn wir uns im IDLE-Zustand befinden, um Konflikte zu vermeiden
            if self.current_phase == IntersectionPhase.IDLE:
                 self.change_phase(IntersectionPhase.DRIVING_TO_SIDE_SIGN_REFERENCE)
            else:
                 self.get_logger().warn(f"StateManager will Intersection starten, aber Node ist nicht IDLE (Phase: {self.current_phase.name}). Ignoriere Start.")


        elif not is_intersection_state and self.maneuver_active_by_statemgr:
            self.get_logger().info(f"Intersection Manöver extern abgebrochen (StateManager nicht mehr im Intersection State: {new_state_from_manager}). Gehe zu IDLE.")
            self.maneuver_active_by_statemgr = False
            self.active_intersection_state = None
            self.change_phase(IntersectionPhase.IDLE) # Führt zum Stoppen und Reset
            # Nicht False auf /intersection/finished publishen, da dies ein externer Abbruch ist

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
        if self.current_phase != IntersectionPhase.DRIVING_TO_SIDE_SIGN_REFERENCE or self.side_sign_detected_by_laser:
            return

        angle_min_param = self.get_parameter('right_side_scan_angle_min_deg').value
        angle_max_param = self.get_parameter('right_side_scan_angle_max_deg').value
        detection_dist_param = self.get_parameter('right_side_scan_distance').value

        # Konvertiere Grad in Radiant für den Scan
        target_angle_min_rad = math.radians(angle_min_param)
        target_angle_max_rad = math.radians(angle_max_param)
        
        # Sicherstellen, dass min < max (da negative Winkel verwendet werden)
        if target_angle_min_rad > target_angle_max_rad:
            target_angle_min_rad, target_angle_max_rad = target_angle_max_rad, target_angle_min_rad


        obstacle_detected = self.check_laser_zone_right(msg, target_angle_min_rad, target_angle_max_rad, detection_dist_param)

        if obstacle_detected:
            self.get_logger().info("Seitenschild für Kreuzungsreferenz per Laser erkannt.")
            self.side_sign_detected_by_laser = True
            self.stop_robot() # Anhalten, da Schild erkannt
            self.change_phase(IntersectionPhase.WAITING_AT_REFERENCE_POINT)


    def check_laser_zone_right(self, scan_msg: LaserScan, angle_min_rad_target: float, angle_max_rad_target: float, detection_distance: float) -> bool:
        """ Prüft einen Laser-Scan Bereich auf der rechten Seite des Roboters.
            Winkel sind im LaserScan-Format (-Pi bis Pi, 0 vorne, negative Winkel rechts).
        """
        if scan_msg.angle_increment <= 0.0:
            self.get_logger().warn("Ungültiges angle_increment im Laserscan.", throttle_duration_sec=10)
            return False
        
        # Direkte Umrechnung der Zielwinkel (die bereits im Laserscan-Format sein sollten) in Indizes
        # Stelle sicher, dass die Zielwinkel innerhalb des Scanbereichs liegen
        # angle_min ist der Startwinkel des Scans (z.B. -pi), angle_max der Endwinkel (z.B. pi)
        
        scan_start_angle = scan_msg.angle_min
        
        # Indizes berechnen
        # Der Index für angle_min_rad_target
        start_index_float = (angle_min_rad_target - scan_start_angle) / scan_msg.angle_increment
        # Der Index für angle_max_rad_target
        end_index_float = (angle_max_rad_target - scan_start_angle) / scan_msg.angle_increment

        # Runden und sicherstellen, dass Indizes im gültigen Bereich liegen
        start_index = max(0, int(round(start_index_float)))
        end_index = min(len(scan_msg.ranges) - 1, int(round(end_index_float)))

        if start_index > end_index:
            # Dies kann passieren, wenn der Zielbereich außerhalb des Scans liegt oder ungültig ist
            # self.get_logger().debug(f"Ungültiger Indexbereich für Scan: {start_index} > {end_index}")
            return False

        for i in range(start_index, end_index + 1):
            dist = scan_msg.ranges[i]
            if not math.isinf(dist) and not math.isnan(dist) and \
               scan_msg.range_min < dist < scan_msg.range_max and \
               dist < detection_distance:
                return True # Hindernis im Zielbereich gefunden
        return False


    def change_phase(self, new_phase: IntersectionPhase):
        if self.current_phase != new_phase:
            self.get_logger().info(f"IntersectionPhase Wechsel: {self.current_phase.name} -> {new_phase.name}")
            self.current_phase = new_phase
            self.phase_start_time = self.get_clock().now().nanoseconds / 1e9

            # Phasen-spezifische Initialisierungen
            if new_phase == IntersectionPhase.DRIVING_TO_SIDE_SIGN_REFERENCE:
                self.side_sign_detected_by_laser = False
                # Startzeit wird oben gesetzt (wichtig für Timeout)

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
                self.maneuver_active_by_statemgr = False # Reset, da Manöver vorbei oder abgebrochen
                self.active_intersection_state = None
                self.side_sign_detected_by_laser = False

            elif new_phase == IntersectionPhase.ABORTING:
                self.stop_robot()
                self.get_logger().warn("Intersection Manöver abgebrochen (z.B. Timeout).")
                # Sende False und gehe dann zu IDLE
                finished_msg = Bool(); finished_msg.data = False
                self.intersection_finished_publisher.publish(finished_msg)
                self.change_phase(IntersectionPhase.IDLE) # Geht sofort in IDLE

            elif new_phase == IntersectionPhase.INTERSECTION_FINISHED:
                self.stop_robot()
                self.get_logger().info("Intersection Manöver erfolgreich beendet.")
                finished_msg = Bool(); finished_msg.data = True
                self.intersection_finished_publisher.publish(finished_msg)
                self.change_phase(IntersectionPhase.IDLE) # Geht sofort in IDLE


    def run_intersection_maneuver(self):
        if not self.maneuver_active_by_statemgr and self.current_phase != IntersectionPhase.IDLE:
            # Dies sollte durch robot_state_manager_callback abgedeckt sein, aber als Sicherheit
            self.change_phase(IntersectionPhase.IDLE)
            return
            
        if self.current_phase == IntersectionPhase.IDLE:
            # Nichts zu tun, warte auf Aktivierung durch StateManager
            return

        current_time = self.get_clock().now().nanoseconds / 1e9
        elapsed_phase_time = current_time - self.phase_start_time

        # --- Phasenlogik ---
        if self.current_phase == IntersectionPhase.DRIVING_TO_SIDE_SIGN_REFERENCE:
            if self.side_sign_detected_by_laser: # Sollte durch Scan-Callback passieren
                self.change_phase(IntersectionPhase.WAITING_AT_REFERENCE_POINT)
                return
            
            scan_timeout = self.get_parameter('side_sign_scan_timeout').value
            if elapsed_phase_time > scan_timeout:
                self.get_logger().warn(f"Timeout ({scan_timeout}s) beim Suchen des Seitenschilds.")
                self.change_phase(IntersectionPhase.ABORTING)
                return
            
            # Geradeaus fahren mit initial_drive_speed
            speed = self.get_parameter('initial_drive_speed').value
            self.move_robot(speed, 0.0)

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
                    self.get_logger().error(f"Ungültiger active_intersection_state: {self.active_intersection_state} in WAITING_AT_REFERENCE_POINT. Gehe zu ABORTING.")
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
            speed = self.get_parameter('turn_forward_speed').value # Kann auch straight_speed sein
            if self.drive_straight_distance(target_dist, speed):
                self.change_phase(IntersectionPhase.FINAL_WAIT)

        elif self.current_phase == IntersectionPhase.EXECUTING_RIGHT_TURN_MANEUVER_TURN:
            turn_speed = -self.get_parameter('turn_angular_speed_right').value # Negativ für Rechtsdrehung
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
        
        # INTERSECTION_FINISHED und ABORTING werden direkt in change_phase behandelt und führen zu IDLE

    def move_robot(self, linear_x: float, angular_z: float):
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_vel_publisher.publish(twist)

    def stop_robot(self):
        self.move_robot(0.0, 0.0)

    def normalize_angle(self, angle: float) -> float:
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

    def drive_straight_distance(self, target_distance: float, speed: float) -> bool:
        """ Fährt eine bestimmte Distanz geradeaus. Gibt True zurück, wenn erreicht. """
        traveled_distance = math.sqrt(
            (self.current_pos_x - self.start_pos_x_segment)**2 +
            (self.current_pos_y - self.start_pos_y_segment)**2
        )
        if traveled_distance >= target_distance - DEFAULT_ODOM_DISTANCE_TOLERANCE:
            self.get_logger().debug(f"Zieldistanz ({target_distance:.2f}m) erreicht. Gefahren: {traveled_distance:.2f}m.")
            self.stop_robot()
            return True
        else:
            self.move_robot(speed, 0.0)
            return False

    def turn_to_target_yaw(self, target_yaw: float, angular_speed_cmd: float, forward_speed: float) -> bool:
        """ Dreht zum Ziel-Yaw mit optionaler Vorwärtsbewegung. Gibt True zurück, wenn erreicht. """
        angle_diff = self.normalize_angle(target_yaw - self.current_yaw)

        if abs(angle_diff) < DEFAULT_GOAL_TOLERANCE_ANGLE_RAD:
            self.get_logger().debug(f"Zieldrehung erreicht. Diff: {angle_diff:.3f}rad")
            self.stop_robot() # Stoppt Drehbewegung, Vorwärtsbewegung für nächste Phase
            return True
        else:
            # Sanfte Annäherung an den Zielwinkel, falls gewünscht (hier nicht implementiert)
            # angular_speed_actual = angular_speed_cmd
            # if abs(angle_diff) < 0.2: # Wenn nah dran, langsamer drehen
            #    angular_speed_actual *= (abs(angle_diff) / 0.2) # Proportionale Verlangsamung
            
            self.move_robot(forward_speed, angular_speed_cmd)
            return False

    def destroy_node(self):
        # self.get_logger().info(f"Shutting down {NODE_NAME}.")
        # self.stop_robot()
        super().destroy_node()

# --- Konstanten aus state_manager für Klarheit (sollten importiert werden, wenn möglich) ---
# Diese müssen mit den Definitionen in state_manager_node.py übereinstimmen
STATE_INTERSECTION_DRIVING_STRAIGHT = 'INTERSEC_DRIVING_STRAIGHT'
STATE_INTERSECTION_TURNING_RIGHT = 'INTERSEC_TURNING_RIGHT'
STATE_INTERSECTION_TURNING_LEFT = 'INTERSEC_TURNING_LEFT'

# --- Konstanten aus parking_node für Topics ---
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
        else: print(f"FATAL ERROR in {NODE_NAME} (vor/während Init): {e}\n{traceback.format_exc()}", file=sys.stderr)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
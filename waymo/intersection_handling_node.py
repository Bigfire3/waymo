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

# --- Standardwerte für Parameter (unverändert) ---
DEFAULT_INITIAL_DRIVE_SPEED = 0.125
DEFAULT_STRAIGHT_SPEED = 0.125
DEFAULT_TURN_FORWARD_SPEED = 0.125
DEFAULT_TURN_ANGULAR_SPEED_RIGHT = 0.45
DEFAULT_TURN_ANGULAR_SPEED_LEFT = 0.25
DEFAULT_STRAIGHT_DISTANCE = 0.80
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
        self.declare_parameter('right_side_scan_angle_min_deg', DEFAULT_RIGHT_SIDE_SCAN_ANGLE_MIN_DEG, deg_angle_desc("Min angle for right side scan (degrees, e.g., 88 for right based on parking_node)", min_val=0.0, max_val=180.0))
        self.declare_parameter('right_side_scan_angle_max_deg', DEFAULT_RIGHT_SIDE_SCAN_ANGLE_MAX_DEG, deg_angle_desc("Max angle for right side scan (degrees, e.g., 92 for right based on parking_node)", min_val=0.0, max_val=180.0))
        self.declare_parameter('right_side_scan_distance', DEFAULT_RIGHT_SIDE_SCAN_DISTANCE, float_desc("Detection distance for right side sign", max_val=1.0))

        self.current_phase = IntersectionPhase.IDLE
        self.active_intersection_state = None
        self.maneuver_active_by_statemgr = False
        self.current_pos_x = 0.0
        self.current_pos_y = 0.0
        self.current_yaw = 0.0
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
        self.cmd_vel_publisher = self.create_publisher(Twist, CMD_VEL_TOPIC, qos_reliable)
        self.intersection_finished_publisher = self.create_publisher(Bool, '/intersection/finished', qos_reliable)
        self.control_timer_period = 0.005 # 200 Hz
        self.control_timer = self.create_timer(self.control_timer_period, self.run_intersection_maneuver)
        # self.get_logger().info(f"{NODE_NAME} initialisiert.")

    def robot_state_manager_callback(self, msg: String):
        new_state_from_manager = msg.data
        is_intersection_state_now = new_state_from_manager in [
            STATE_INTERSECTION_DRIVING_STRAIGHT, STATE_INTERSECTION_TURNING_LEFT, STATE_INTERSECTION_TURNING_RIGHT
        ]

        if is_intersection_state_now:
            if not self.maneuver_active_by_statemgr:
                # SM will ein Manöver starten, und wir waren bisher nicht aktiv (oder dachten es zumindest)
                # self.get_logger().info(f"IH-Node: Manöver START-Signal von SM (State: {new_state_from_manager}). IH-Phase: {self.current_phase.name}")
                self.maneuver_active_by_statemgr = True
                self.active_intersection_state = new_state_from_manager
                if self.current_phase == IntersectionPhase.IDLE:
                    self.side_sign_detected_by_laser = False # Wichtig für den Neustart der Schildsuche
                    self.change_phase(IntersectionPhase.DRIVING_TO_SIDE_SIGN_REFERENCE)
                else:
                    # SM will starten, aber wir sind nicht IDLE. Das ist unerwartet.
                    # Wir sollten das aktuelle Manöver sauber abbrechen und dann neu starten.
                    # self.get_logger().warn(f"IH-Node: SM will Intersection starten ({new_state_from_manager}), aber IH-Node war nicht IDLE (Phase: {self.current_phase.name}). Breche ab und starte Manöver neu.")
                    # Setze interne Zustände zurück, als ob ein neues Manöver beginnt
                    self.side_sign_detected_by_laser = False # Wichtig für den Neustart der Schildsuche
                    self.change_phase(IntersectionPhase.DRIVING_TO_SIDE_SIGN_REFERENCE)
            # else: SM ist im Intersection State, und wir wissen das schon (maneuver_active_by_statemgr ist True).
            # Hier könnte man prüfen, ob sich der Typ des Intersection States geändert hat, aber das ist für später.
            # Vorerst: Wenn wir aktiv sind und der SM immer noch einen Intersection-State sendet, ist alles konsistent.
            elif self.maneuver_active_by_statemgr and self.active_intersection_state != new_state_from_manager:
                # SM hat den Typ des Intersection-Manövers geändert, während wir aktiv waren.
                # Das ist ein komplexer Fall. Fürs Erste: Altes Manöver abbrechen und neues starten.
                # self.get_logger().warn(f"IH-Node: SM hat Intersection-Typ geändert von {self.active_intersection_state} zu {new_state_from_manager} während aktiv. Starte neu.")
                self.active_intersection_state = new_state_from_manager
                self.side_sign_detected_by_laser = False
                self.change_phase(IntersectionPhase.DRIVING_TO_SIDE_SIGN_REFERENCE)

        elif not is_intersection_state_now: # SM ist NICHT mehr in einem Intersection State
            if self.maneuver_active_by_statemgr:
                # SM hat das Manöver beendet/verlassen, während wir noch dachten, wir wären aktiv.
                # self.get_logger().info(f"IH-Node: Externer Abbruch durch SM (neuer SM-State: {new_state_from_manager}). Aktuelle IH-Phase: {self.current_phase.name}")
                self.maneuver_active_by_statemgr = False # Wichtig: SOFORT als nicht mehr aktiv markieren
                if self.current_phase != IntersectionPhase.IDLE:
                    self.change_phase(IntersectionPhase.IDLE) 
            # else: SM ist nicht im Intersection State, und wir wissen das schon (maneuver_active_by_statemgr ist False). -> nichts tun.

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
            # self.get_logger().info("IH-Node: Seitenschild für Kreuzungsreferenz per Laser erkannt.")
            self.side_sign_detected_by_laser = True
            self.stop_robot()
            self.change_phase(IntersectionPhase.WAITING_AT_REFERENCE_POINT)

    def _check_laser_zone(self, scan_msg: LaserScan, angle_min_rad_target: float, angle_max_rad_target: float, detection_distance: float) -> bool:
        # Stellt sicher, dass angle_increment gültig ist, um Division durch Null oder Endlosschleifen zu vermeiden
        if scan_msg.angle_increment <= 0.0:
            self.get_logger().warn("Ungültiges angle_increment im Laserscan.", throttle_duration_sec=10)
            return False
        
        # Berechne den tatsächlichen maximalen Winkel des Scans
        actual_scan_angle_max_rad = scan_msg.angle_min + (len(scan_msg.ranges) - 1) * scan_msg.angle_increment
        
        # Stelle sicher, dass der Zielbereich innerhalb des Scanbereichs liegt
        adj_target_min_rad = max(angle_min_rad_target, scan_msg.angle_min)
        adj_target_max_rad = min(angle_max_rad_target, actual_scan_angle_max_rad)

        # Wenn der angepasste Bereich ungültig ist (min >= max), gibt es keine gültigen Indizes
        if adj_target_min_rad >= adj_target_max_rad:
            #self.get_logger().debug(f"Angepasster Scanbereich ungültig: min_rad={adj_target_min_rad}, max_rad={adj_target_max_rad}")
            return False

        # Konvertiere Winkel in Array-Indizes
        # Runden auf den nächsten Index oder int() verwenden (abschneiden) kann je nach Anforderung variieren.
        # Hier verwenden wir int() für den Start und stellen sicher, dass der Endindex nicht überschritten wird.
        start_index = max(0, int((adj_target_min_rad - scan_msg.angle_min) / scan_msg.angle_increment))
        end_index = min(len(scan_msg.ranges) - 1, int((adj_target_max_rad - scan_msg.angle_min) / scan_msg.angle_increment))
        
        # Erneute Prüfung, ob die Indizes nach Anpassung und Konvertierung gültig sind
        if start_index > end_index:
            #self.get_logger().debug(f"Startindex {start_index} > Endindex {end_index} nach Indexberechnung.")
            return False

        #self.get_logger().debug(f"Scanning zone from index {start_index} to {end_index} for distance < {detection_distance:.2f}m.")
        for i in range(start_index, end_index + 1):
            dist = scan_msg.ranges[i]
            # Prüfe auf gültige Distanzwerte (nicht unendlich, nicht NaN)
            # und ob sie innerhalb des gültigen Bereichs des Sensors liegen
            # und kleiner als die Zieldistanz sind.
            if not math.isinf(dist) and not math.isnan(dist) and \
               dist >= scan_msg.range_min and dist <= scan_msg.range_max and \
               dist < detection_distance:
                #self.get_logger().debug(f"Hindernis bei Index {i} auf {dist:.2f}m erkannt (Ziel < {detection_distance:.2f}m).")
                return True # Hindernis im Zielbereich und innerhalb der Distanz gefunden
        return False # Kein Hindernis im Zielbereich gefunden

    def change_phase(self, new_phase: IntersectionPhase):
        # Verhindere Log-Spam für wiederholte IDLE-Wechsel, wenn wir schon IDLE sind und nicht mehr aktiv sein sollen.
        if self.current_phase == new_phase and (self.current_phase == IntersectionPhase.IDLE and not self.maneuver_active_by_statemgr) :
             return
        
        # Wenn ein Manöver abgeschlossen/abgebrochen wird und zu IDLE wechselt, immer loggen.
        log_this_change = True
        if self.current_phase == new_phase and self.current_phase == IntersectionPhase.IDLE: # Verhindere Log-Spam für IDLE->IDLE
            log_this_change = False 

        # if log_this_change:
            # self.get_logger().info(f"IH-Node: Phasenwechsel: {self.current_phase.name} -> {new_phase.name}")
        
        self.current_phase = new_phase
        self.phase_start_time = self.get_clock().now().nanoseconds / 1e9

        if new_phase == IntersectionPhase.DRIVING_TO_SIDE_SIGN_REFERENCE:
            self.side_sign_detected_by_laser = False
            # self.maneuver_active_by_statemgr sollte hier schon True sein (gesetzt im Callback)
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
            self.stop_robot() # Sicherstellen, dass der Roboter stoppt
            # if log_this_change: # Nur loggen, wenn es ein echter Phasenwechsel ist
                #  self.get_logger().warn("IH-Node: Manöver ABGEBROCHEN (z.B. Timeout). Sende False.")
            finished_msg = Bool(); finished_msg.data = False
            self.intersection_finished_publisher.publish(finished_msg)
            # maneuver_active_by_statemgr wird hier *nicht* sofort false, 
            # damit der robot_state_manager_callback ggf. noch reagieren kann, falls SM noch aktiv ist.
            # Stattdessen wird direkt zu IDLE gewechselt, wo es dann auf False gesetzt wird.
            self.change_phase(IntersectionPhase.IDLE) 
        
        elif new_phase == IntersectionPhase.INTERSECTION_FINISHED:
            self.stop_robot() # Sicherstellen, dass der Roboter stoppt
            # if log_this_change:
                # self.get_logger().info("IH-Node: Manöver ERFOLGREICH BEENDET. Sende True.")
            finished_msg = Bool(); finished_msg.data = True
            self.intersection_finished_publisher.publish(finished_msg)
            self.change_phase(IntersectionPhase.IDLE)

    def run_intersection_maneuver(self):
        # **Haupt-Gatekeeper**
        # Wenn der StateManager uns nicht mehr als aktiv für ein Intersection-Manöver sieht,
        # sollen wir nichts tun und sicherstellen, dass wir in IDLE sind.
        if not self.maneuver_active_by_statemgr:
            if self.current_phase != IntersectionPhase.IDLE:
                # Dieser Fall tritt ein, wenn der SM-Callback maneuver_active_by_statemgr auf False gesetzt hat,
                # aber der Phasenwechsel zu IDLE noch nicht vollständig durch diesen Timer-Loop reflektiert wurde.
                # Loggen ist hier meist nicht nötig, da der Callback schon geloggt hat.
                self.change_phase(IntersectionPhase.IDLE)
            return # Absolut nichts weiter tun, wenn nicht aktiv. Roboter sollte stehen (durch change_phase(IDLE)).

        # Wenn wir hier sind, ist self.maneuver_active_by_statemgr == True.
        # Die IH-Node glaubt also, sie *sollte* ein Manöver ausführen.

        # Sicherheitsnetz: Wenn aktiv, aber Phase ist IDLE, stimmt was nicht.
        # Normalerweise sollte der robot_state_manager_callback von IDLE in eine Arbeitsphase wechseln.
        if self.current_phase == IntersectionPhase.IDLE and self.maneuver_active_by_statemgr:
            # self.get_logger().warn(f"IH-Node: run_maneuver - Aktiv vom SM, aber Phase ist IDLE. "
                                #    f"Warte auf korrekten Phasenwechsel durch SM-Callback für State '{self.active_intersection_state}'.")
            # Hier nicht automatisch starten, um Konflikte mit dem Callback zu vermeiden.
            # Der Callback ist dafür zuständig, von IDLE in DRIVING_TO_SIDE_SIGN_REFERENCE zu wechseln.
            return

        current_time = self.get_clock().now().nanoseconds / 1e9
        elapsed_phase_time = current_time - self.phase_start_time

        # --- Eigentliche Phasenlogik (nur wenn self.maneuver_active_by_statemgr == True) ---
        if self.current_phase == IntersectionPhase.DRIVING_TO_SIDE_SIGN_REFERENCE:
            # (scan_callback kümmert sich um den Wechsel bei Schilderkennung)
            scan_timeout = self.get_parameter('side_sign_scan_timeout').value
            if elapsed_phase_time > scan_timeout:
                # self.get_logger().warn(f"Timeout ({scan_timeout}s) beim Suchen des Seitenschilds.")
                self.change_phase(IntersectionPhase.ABORTING)
                return
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
                    # self.get_logger().error(f"Ungültiger active_intersection_state: {self.active_intersection_state} in WAITING_AT_REFERENCE_POINT. Gehe zu ABORTING.")
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
        
        # Phasen ABORTING und INTERSECTION_FINISHED werden direkt in change_phase zu IDLE weitergeleitet

    def move_robot(self, linear_x: float, angular_z: float): # Unverändert
        twist = Twist(); twist.linear.x = linear_x; twist.angular.z = angular_z
        self.cmd_vel_publisher.publish(twist)

    def stop_robot(self): # Unverändert
        self.move_robot(0.0, 0.0)

    def normalize_angle(self, angle: float) -> float: # Unverändert
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

    def drive_straight_distance(self, target_distance: float, speed: float) -> bool: # Unverändert
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

    def turn_to_target_yaw(self, target_yaw: float, angular_speed_cmd: float, forward_speed: float) -> bool: # Unverändert
        angle_diff = self.normalize_angle(target_yaw - self.current_yaw)
        if abs(angle_diff) < DEFAULT_GOAL_TOLERANCE_ANGLE_RAD:
            self.stop_robot()
            return True
        else:
            self.move_robot(forward_speed, angular_speed_cmd)
            return False

    def destroy_node(self): # Unverändert zur letzten Version
        # self.get_logger().info(f"Shutting down {NODE_NAME}.")
        # if self.maneuver_active_by_statemgr or self.current_phase != IntersectionPhase.IDLE : 
            #  self.get_logger().info(f"Stopping robot in destroy_node as current phase is {self.current_phase.name} and active_flag is {self.maneuver_active_by_statemgr}")
            #  self.stop_robot()
        super().destroy_node()

STATE_INTERSECTION_DRIVING_STRAIGHT = 'INTERSEC_DRIVING_STRAIGHT'
STATE_INTERSECTION_TURNING_RIGHT = 'INTERSEC_TURNING_RIGHT'
STATE_INTERSECTION_TURNING_LEFT = 'INTERSEC_TURNING_LEFT'
CMD_VEL_TOPIC = '/cmd_vel'
LASERSCAN_TOPIC = '/scan'
ROBOT_STATE_TOPIC = '/robot/state'

def main(args=None): # Unverändert
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
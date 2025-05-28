# parking_node.py
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
import time # Beibehalten für phase_start_time und Timeouts
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
PARKING_LINEAR_SPEED = 0.1 # Einheitliche lineare Geschwindigkeit für die meisten Manöver
MANEUVER_ANGULAR_SPEED_TURN = 0.8
MAX_ANGULAR_Z_LANE_FOLLOW = 0.35

# Laserscan Parameter
INITIAL_SCAN_ANGLE_MIN_DEG = 88.0
INITIAL_SCAN_ANGLE_MAX_DEG = 92.0
INITIAL_SCAN_ANGLE_MIN_RAD = math.radians(INITIAL_SCAN_ANGLE_MIN_DEG)
INITIAL_SCAN_ANGLE_MAX_RAD = math.radians(INITIAL_SCAN_ANGLE_MAX_DEG)
INITIAL_SPOT_DETECTION_DISTANCE = 0.25 # Distanz, um das initiale Schild zu erkennen

SPOT_SCAN_ANGLE_MIN_DEG = 70.0
SPOT_SCAN_ANGLE_MAX_DEG = 110.0
SPOT_SCAN_ANGLE_MIN_RAD = math.radians(SPOT_SCAN_ANGLE_MIN_DEG)
SPOT_SCAN_ANGLE_MAX_RAD = math.radians(SPOT_SCAN_ANGLE_MAX_DEG)
PARKING_SPOT_CLEAR_DISTANCE = 0.4 # Distanz, um zu prüfen, ob eine Parklücke frei ist

# Zeit- und Distanzparameter (Odometrie basiert)
INITIAL_STOP_DURATION = 0.0 # Kurzer Stopp nach Schilderkennung (kann beibehalten werden)
# DRIVE_TO_FIRST_SPOT_DURATION -> Ersetzt durch DISTANCE_AFTER_INITIAL_SIGN
# DRIVE_TO_NEXT_SPOT_DURATION -> Ersetzt durch DISTANCE_BETWEEN_SPOTS
STOP_BEFORE_SCAN_DURATION = 0.0 # Kurzer Stopp vor dem Scannen der Lücke
PARKING_DURATION = 10.0 # Wie lange im Parkplatz gewartet wird
MOVE_SPOT_DISTANCE = 0.28 # Distanz für das Ein- und Ausparken in die Lücke (nicht die Fahrt zur Lücke)
TURN_ANGLE_90_DEG = math.radians(90.0) # Korrekturwinkel für 90 Grad Drehungen
GOAL_TOLERANCE_ANGLE_RAD = math.radians(2.0) # Toleranz für Drehmanöver

# Neue Odometrie-basierte Distanzen
DISTANCE_AFTER_INITIAL_SIGN = 0.5  # Meter, 51cm nach dem Schild
DISTANCE_BETWEEN_SPOTS = 0.34       # Meter, 33cm von Parklücke zu Parklücke
ODOM_DISTANCE_TOLERANCE = 0.01      # Meter, Toleranz für das Erreichen der Zieldistanz (2cm)

# Timeouts für Odometrie-basierte Fahrten
TIMEOUT_INITIAL_DISTANCE = 15.0     # Sekunden, Timeout für die 51cm Fahrt
TIMEOUT_BETWEEN_SPOTS = 10.0        # Sekunden, Timeout für die 33cm Fahrten
TIMEOUT_MOVE_SPOT = 5.0          # Sekunden, Timeout für das Ein- und Ausparken in die Lücke

MAX_PARKING_ATTEMPTS = 3


class ParkingPhase(Enum):
    IDLE = auto()
    DRIVING_FOR_INITIAL_LASER_SCAN = auto() # Fährt langsam, bis Laserscan Schild rechts erkennt
    INITIAL_SIGN_STOPPING = auto()          # Stoppt kurz nach Schilderkennung
    INITIAL_SIGN_WAITING = auto()           # Wartet kurz nach Schilderkennung
    DRIVING_INITIAL_DISTANCE_ODOM = auto()  # Fährt die ersten 51cm per Odometrie
    DRIVING_TO_NEXT_SPOT_ODOM = auto()      # Fährt 33cm zur nächsten Parklücke per Odometrie
    STOPPING_BEFORE_SCAN = auto()           # Stoppt kurz vor dem Scannen der Parklücke
    SCANNING_FOR_SPOT = auto()              # Scannt die Parklücke
    TURNING_RIGHT_FOR_PARKING = auto()      # Dreht nach rechts zum Einparken
    MOVING_INTO_SPOT = auto()               # Fährt in die Parklücke
    TURNING_LEFT_IN_SPOT_TO_PARK = auto()   # Korrektur Drehung links in der Parklücke
    WAITING_IN_SPOT = auto()                # Wartet in der Parklücke
    TURNING_LEFT_IN_SPOT_TO_GET_OUT = auto() # Dreht links zum Ausparken
    MOVING_OUT_OF_SPOT = auto()             # Fährt aus der Parklücke
    TURNING_RIGHT_FOR_LANE_FOLLOWING = auto()# Dreht rechts zurück zur Fahrspur
    MANEUVER_COMPLETE = auto()              # Parkmanöver abgeschlossen (oder fehlgeschlagen)


class ParkingNode(Node):
    def __init__(self):
        super().__init__(NODE_NAME)

        self.current_robot_state_from_manager = ""
        self.parking_phase = ParkingPhase.IDLE
        self.current_center_offset = 0.0
        self.initial_sign_laser_detected = False

        # Odometrie Daten
        self.current_pos_x = 0.0
        self.current_pos_y = 0.0
        self.current_yaw = 0.0

        # Für distanzbasierte Fahrten
        self.start_pos_x_segment = 0.0
        self.start_pos_y_segment = 0.0
        self.target_distance_segment = 0.0
        self.current_segment_max_duration = 0.0 # Timeout für das aktuelle Odom-Fahrsegment

        # Für Drehungen
        self.start_yaw_for_turn = 0.0
        self.target_yaw_for_turn = 0.0

        self.phase_start_time = 0.0 # Zeitstempel beim Start einer Phase
        self.maneuver_drive_duration = 0.0 # Für zeitbasierte Teile wie Einparken (MOVE_SPOT_DISTANCE)

        self.parking_attempts_count = 0 # Zählt Versuche, eine freie Lücke zu finden/anzufahren

        qos_reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1)
        qos_best_effort = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)

        self.state_subscriber = self.create_subscription(String, ROBOT_STATE_TOPIC, self.robot_state_manager_callback, qos_reliable)
        self.scan_subscriber = self.create_subscription(LaserScan, LASERSCAN_TOPIC, self.scan_callback, qos_best_effort)
        self.offset_subscriber = self.create_subscription(Float64, LANE_OFFSET_TOPIC, self.lane_offset_callback, qos_best_effort)
        self.odom_subscriber = self.create_subscription(Odometry, ODOM_TOPIC, self.odom_callback, qos_best_effort)

        self.cmd_vel_publisher = self.create_publisher(Twist, CMD_VEL_TOPIC, qos_reliable)
        self.parking_finished_publisher = self.create_publisher(Bool, PARKING_FINISHED_TOPIC, qos_reliable)

        self.control_timer_period = 0.005 # 200 Hz für den Kontrollloop
        self.control_timer = self.create_timer(self.control_timer_period, self.parking_sequence_controller)

        # self.get_logger().info(f"{NODE_NAME} initialisiert.")

    def robot_state_manager_callback(self, msg: String):
        new_state_from_manager = msg.data
        if self.current_robot_state_from_manager != new_state_from_manager:
            # self.get_logger().info(f"Robotermanager änderte Zustand von '{self.current_robot_state_from_manager}' zu '{new_state_from_manager}'")
            self.current_robot_state_from_manager = new_state_from_manager

            if self.current_robot_state_from_manager == STATE_PARKING and \
               (self.parking_phase == ParkingPhase.IDLE or self.parking_phase == ParkingPhase.MANEUVER_COMPLETE):
                # self.get_logger().info("Parksequenz gestartet durch Robotermanager.")
                self.initial_sign_laser_detected = False
                self.parking_attempts_count = 0
                self.change_parking_phase(ParkingPhase.DRIVING_FOR_INITIAL_LASER_SCAN)
            elif self.current_robot_state_from_manager != STATE_PARKING and self.parking_phase != ParkingPhase.IDLE:
                # self.get_logger().info("Parksequenz abgebrochen durch Robotermanager (Zustand nicht mehr PARKING).")
                self.reset_parking_sequence()

    def reset_parking_sequence(self):
        # self.get_logger().info("Parksequenz wird zurückgesetzt.")
        self.change_parking_phase(ParkingPhase.IDLE)
        self.initial_sign_laser_detected = False
        self.parking_attempts_count = 0
        self.stop_robot()

    def lane_offset_callback(self, msg: Float64):
        self.current_center_offset = msg.data

    def odom_callback(self, msg: Odometry):
        # Position speichern
        self.current_pos_x = msg.pose.pose.position.x
        self.current_pos_y = msg.pose.pose.position.y

        # Orientierung (Yaw) extrahieren
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        try:
            r = R.from_quat(orientation_list)
            euler = r.as_euler('xyz', degrees=False) # 'xyz' for yaw around z
            self.current_yaw = euler[2]
        except Exception as e:
            # self.get_logger().warn(f"Fehler bei Quaternion -> Euler Umwandlung in odom_callback: {e}", throttle_duration_sec=5)
            pass

    def scan_callback(self, msg: LaserScan):
        if self.current_robot_state_from_manager != STATE_PARKING:
            return # Nur im Parkzustand Scans verarbeiten

        # Logik für das initiale Scannen des Parkschilds
        if self.parking_phase == ParkingPhase.DRIVING_FOR_INITIAL_LASER_SCAN and not self.initial_sign_laser_detected:
            obstacle_detected_at_initial_spot = self.check_laser_zone(
                msg, INITIAL_SCAN_ANGLE_MIN_RAD, INITIAL_SCAN_ANGLE_MAX_RAD, INITIAL_SPOT_DETECTION_DISTANCE
            )
            if obstacle_detected_at_initial_spot:
                # self.get_logger().info("Initiales Parkschild per Laser erkannt.")
                self.initial_sign_laser_detected = True
                self.change_parking_phase(ParkingPhase.INITIAL_SIGN_STOPPING)

        # Logik für das Scannen der Parklücke
        elif self.parking_phase == ParkingPhase.SCANNING_FOR_SPOT:
            is_spot_clear = not self.check_laser_zone(
                msg, SPOT_SCAN_ANGLE_MIN_RAD, SPOT_SCAN_ANGLE_MAX_RAD, PARKING_SPOT_CLEAR_DISTANCE
            )
            if is_spot_clear:
                # self.get_logger().info(f"Parklücke (Versuch {self.parking_attempts_count + 1}) ist frei.")
                self.change_parking_phase(ParkingPhase.TURNING_RIGHT_FOR_PARKING)
            else:
                # self.get_logger().info(f"Parklücke (Versuch {self.parking_attempts_count + 1}) ist besetzt.")
                self.parking_attempts_count += 1
                if self.parking_attempts_count >= MAX_PARKING_ATTEMPTS:
                    # self.get_logger().warn(f"Maximale Anzahl ({MAX_PARKING_ATTEMPTS}) an Parkversuchen erreicht. Breche Parken ab.")
                    self.change_parking_phase(ParkingPhase.MANEUVER_COMPLETE)
                else:
                    # self.get_logger().info("Fahre zur nächsten potenziellen Parklücke.")
                    self.change_parking_phase(ParkingPhase.DRIVING_TO_NEXT_SPOT_ODOM) # Odometrie-basierte Fahrt zur nächsten Lücke

    def check_laser_zone(self, scan_msg: LaserScan, angle_min_rad_target: float, angle_max_rad_target: float, detection_distance: float) -> bool:
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

    def change_parking_phase(self, new_phase: ParkingPhase):
        if self.parking_phase != new_phase:
            # self.get_logger().info(f"ParkPhase Wechsel: {self.parking_phase.name} -> {new_phase.name}")
            self.parking_phase = new_phase
            self.phase_start_time = self.get_clock().now().nanoseconds / 1e9 # Zeitstempel für die neue Phase

            # Phasen-spezifische Initialisierungen
            if new_phase == ParkingPhase.DRIVING_INITIAL_DISTANCE_ODOM:
                self.start_pos_x_segment = self.current_pos_x
                self.start_pos_y_segment = self.current_pos_y
                self.target_distance_segment = DISTANCE_AFTER_INITIAL_SIGN
                self.current_segment_max_duration = TIMEOUT_INITIAL_DISTANCE
                # self.get_logger().info(f"Starte Fahrt für {self.target_distance_segment}m (max. {self.current_segment_max_duration}s). Start Odom: ({self.start_pos_x_segment:.2f}, {self.start_pos_y_segment:.2f})")
            elif new_phase == ParkingPhase.DRIVING_TO_NEXT_SPOT_ODOM:
                self.start_pos_x_segment = self.current_pos_x
                self.start_pos_y_segment = self.current_pos_y
                self.target_distance_segment = DISTANCE_BETWEEN_SPOTS
                self.current_segment_max_duration = TIMEOUT_BETWEEN_SPOTS
                # self.get_logger().info(f"Starte Fahrt für {self.target_distance_segment}m (max. {self.current_segment_max_duration}s). Start Odom: ({self.start_pos_x_segment:.2f}, {self.start_pos_y_segment:.2f})")
            elif \
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
                # self.get_logger().info(f"Starte Drehung. Aktueller Yaw: {self.current_yaw:.2f}rad, Ziel-Yaw: {self.target_yaw_for_turn:.2f}rad")
            elif new_phase == ParkingPhase.MOVING_INTO_SPOT or new_phase == ParkingPhase.MOVING_OUT_OF_SPOT:
                self.start_pos_x_segment = self.current_pos_x
                self.start_pos_y_segment = self.current_pos_y
                self.target_distance_segment = MOVE_SPOT_DISTANCE
                self.current_segment_max_duration = TIMEOUT_MOVE_SPOT
            elif new_phase == ParkingPhase.WAITING_IN_SPOT:
                self.maneuver_drive_duration = PARKING_DURATION # Wiederverwendung der Variable für die Wartezeit

            # Stoppe den Roboter bei bestimmten Phasenübergängen
            if new_phase in [ParkingPhase.INITIAL_SIGN_STOPPING,
                               ParkingPhase.INITIAL_SIGN_WAITING,
                               ParkingPhase.STOPPING_BEFORE_SCAN,
                               ParkingPhase.SCANNING_FOR_SPOT, # Stoppt, um den Scan in Ruhe auszuwerten
                               ParkingPhase.MANEUVER_COMPLETE,
                               ParkingPhase.IDLE]:
                self.stop_robot()

    def parking_sequence_controller(self):
        # Haupt-Kontrollschleife, wird periodisch aufgerufen
        if self.current_robot_state_from_manager != STATE_PARKING:
            if self.parking_phase != ParkingPhase.IDLE: # Wenn nicht bereits IDLE, dann resetten
                self.reset_parking_sequence()
            return # Verlasse die Funktion, wenn nicht im Parkzustand

        if self.parking_phase == ParkingPhase.MANEUVER_COMPLETE:
            self.stop_robot()
            # Hier wird einmalig signalisiert, dass das Parken beendet ist.
            # Der StateManager sollte dies empfangen und den Zustand des Roboters ändern.
            # Um mehrfaches Senden zu vermeiden, könnte man ein Flag setzen.
            # Fürs Erste ist es okay, da der Subscriber im StateManager dies handhaben sollte.
            finished_msg = Bool(); finished_msg.data = True
            self.parking_finished_publisher.publish(finished_msg)
            return

        current_time = self.get_clock().now().nanoseconds / 1e9
        elapsed_phase_time = current_time - self.phase_start_time

        # --- Phasenlogik ---
        if self.parking_phase == ParkingPhase.IDLE:
            self.stop_robot()
            return

        elif self.parking_phase == ParkingPhase.DRIVING_FOR_INITIAL_LASER_SCAN:
            if not self.initial_sign_laser_detected:
                self.follow_lane_slowly() # Langsam fahren und auf Laser-Schilderkennung warten
            else:
                # Sollte durch scan_callback bereits in INITIAL_SIGN_STOPPING gewechselt sein
                self.stop_robot()
            return

        elif self.parking_phase == ParkingPhase.INITIAL_SIGN_STOPPING:
            self.stop_robot() # Stellt sicher, dass der Roboter steht
            self.change_parking_phase(ParkingPhase.INITIAL_SIGN_WAITING) # Gehe zur Wartephase

        elif self.parking_phase == ParkingPhase.INITIAL_SIGN_WAITING:
            self.stop_robot()
            if elapsed_phase_time >= INITIAL_STOP_DURATION:
                self.parking_attempts_count = 0 # Reset für den ersten Parkversuch
                self.change_parking_phase(ParkingPhase.DRIVING_INITIAL_DISTANCE_ODOM) # Starte 70cm Fahrt

        elif self.parking_phase == ParkingPhase.DRIVING_INITIAL_DISTANCE_ODOM:
            traveled_distance = math.sqrt(
                (self.current_pos_x - self.start_pos_x_segment)**2 +
                (self.current_pos_y - self.start_pos_y_segment)**2
            )
            if traveled_distance >= self.target_distance_segment - ODOM_DISTANCE_TOLERANCE:
                # self.get_logger().info(f"Zieldistanz ({self.target_distance_segment:.2f}m) erreicht. Gefahren: {traveled_distance:.2f}m.")
                self.stop_robot()
                self.change_parking_phase(ParkingPhase.STOPPING_BEFORE_SCAN)
            elif elapsed_phase_time >= self.current_segment_max_duration:
                # self.get_logger().warn(f"Timeout! Konnte Zieldistanz ({self.target_distance_segment:.2f}m) nicht in {self.current_segment_max_duration:.1f}s erreichen. Gefahren: {traveled_distance:.2f}m.")
                self.stop_robot()
                self.change_parking_phase(ParkingPhase.MANEUVER_COMPLETE) # Parken fehlgeschlagen
            else:
                self.follow_lane_slowly() # Weiterfahren mit Spurhaltung

        elif self.parking_phase == ParkingPhase.DRIVING_TO_NEXT_SPOT_ODOM:
            traveled_distance = math.sqrt(
                (self.current_pos_x - self.start_pos_x_segment)**2 +
                (self.current_pos_y - self.start_pos_y_segment)**2
            )
            if traveled_distance >= self.target_distance_segment - ODOM_DISTANCE_TOLERANCE:
                # self.get_logger().info(f"Zieldistanz ({self.target_distance_segment:.2f}m) zur nächsten Lücke erreicht. Gefahren: {traveled_distance:.2f}m.")
                self.stop_robot()
                self.change_parking_phase(ParkingPhase.STOPPING_BEFORE_SCAN)
            elif elapsed_phase_time >= self.current_segment_max_duration:
                # self.get_logger().warn(f"Timeout! Konnte Zieldistanz ({self.target_distance_segment:.2f}m) zur nächsten Lücke nicht in {self.current_segment_max_duration:.1f}s erreichen. Gefahren: {traveled_distance:.2f}m.")
                self.stop_robot()
                self.parking_attempts_count += 1 # Dieser Versuch, die Lücke zu erreichen, zählt als fehlgeschlagen
                # self.get_logger().info(f"Parkversuch {self.parking_attempts_count} (von {MAX_PARKING_ATTEMPTS}) wegen Timeout beim Anfahren gescheitert.")
                if self.parking_attempts_count >= MAX_PARKING_ATTEMPTS:
                    # self.get_logger().warn(f"Maximale Anzahl ({MAX_PARKING_ATTEMPTS}) an Parkversuchen erreicht (Timeout beim Anfahren).")
                    self.change_parking_phase(ParkingPhase.MANEUVER_COMPLETE)
                else:
                    # Versuche, die nächste Lücke anzufahren (springt quasi über die nicht erreichte Lücke)
                    self.change_parking_phase(ParkingPhase.DRIVING_TO_NEXT_SPOT_ODOM)
            else:
                self.follow_lane_slowly() # Weiterfahren mit Spurhaltung

        elif self.parking_phase == ParkingPhase.STOPPING_BEFORE_SCAN:
            self.stop_robot()
            if elapsed_phase_time >= STOP_BEFORE_SCAN_DURATION: # Auch wenn 0, geht direkt weiter
                self.change_parking_phase(ParkingPhase.SCANNING_FOR_SPOT)

        elif self.parking_phase == ParkingPhase.SCANNING_FOR_SPOT:
            self.stop_robot() # Stehenbleiben während des Scannens (Scan-Callback übernimmt Phasenwechsel)
            pass # Warte auf Scan-Ergebnis im scan_callback

        elif self.parking_phase == ParkingPhase.TURNING_RIGHT_FOR_PARKING:
            if self.turn_to_target(self.target_yaw_for_turn, MANEUVER_ANGULAR_SPEED_TURN):
                self.stop_robot()
                self.change_parking_phase(ParkingPhase.MOVING_INTO_SPOT)

        elif self.parking_phase == ParkingPhase.MOVING_INTO_SPOT:
            traveled_distance = math.sqrt(
                (self.current_pos_x - self.start_pos_x_segment)**2 +
                (self.current_pos_y - self.start_pos_y_segment)**2
            )
            if traveled_distance >= self.target_distance_segment - ODOM_DISTANCE_TOLERANCE:
                # self.get_logger().info(f"Zieldistanz ({self.target_distance_segment:.2f}m) zur nächsten Lücke erreicht. Gefahren: {traveled_distance:.2f}m.")
                self.stop_robot()
                self.change_parking_phase(ParkingPhase.TURNING_LEFT_IN_SPOT_TO_PARK)
            elif elapsed_phase_time >= self.current_segment_max_duration:
                self.stop_robot()
                self.change_parking_phase(ParkingPhase.MANEUVER_COMPLETE) # Parken fehlgeschlagen
            else:
                self.move_straight(PARKING_LINEAR_SPEED)

        elif self.parking_phase == ParkingPhase.TURNING_LEFT_IN_SPOT_TO_PARK:
            if self.turn_to_target(self.target_yaw_for_turn, MANEUVER_ANGULAR_SPEED_TURN):
                self.stop_robot()
                self.change_parking_phase(ParkingPhase.WAITING_IN_SPOT)

        elif self.parking_phase == ParkingPhase.WAITING_IN_SPOT:
            self.stop_robot()
            if elapsed_phase_time >= self.maneuver_drive_duration: # Hier ist maneuver_drive_duration die PARKING_DURATION
                self.change_parking_phase(ParkingPhase.TURNING_LEFT_IN_SPOT_TO_GET_OUT)

        elif self.parking_phase == ParkingPhase.TURNING_LEFT_IN_SPOT_TO_GET_OUT:
            if self.turn_to_target(self.target_yaw_for_turn, MANEUVER_ANGULAR_SPEED_TURN):
                self.stop_robot()
                self.change_parking_phase(ParkingPhase.MOVING_OUT_OF_SPOT)

        elif self.parking_phase == ParkingPhase.MOVING_OUT_OF_SPOT:
            traveled_distance = math.sqrt(
                (self.current_pos_x - self.start_pos_x_segment)**2 +
                (self.current_pos_y - self.start_pos_y_segment)**2
            )
            if traveled_distance >= self.target_distance_segment - ODOM_DISTANCE_TOLERANCE:
                # self.get_logger().info(f"Zieldistanz ({self.target_distance_segment:.2f}m) zur nächsten Lücke erreicht. Gefahren: {traveled_distance:.2f}m.")
                self.stop_robot()
                self.change_parking_phase(ParkingPhase.TURNING_RIGHT_FOR_LANE_FOLLOWING)
            elif elapsed_phase_time >= self.current_segment_max_duration:
                self.stop_robot()
                self.change_parking_phase(ParkingPhase.MANEUVER_COMPLETE) # Parken fehlgeschlagen
            else:
                self.move_straight(PARKING_LINEAR_SPEED)

        elif self.parking_phase == ParkingPhase.TURNING_RIGHT_FOR_LANE_FOLLOWING:
            if self.turn_to_target(self.target_yaw_for_turn, MANEUVER_ANGULAR_SPEED_TURN):
                self.stop_robot()
                # Kein Zeit-Check hier, Drehung ist abgeschlossen, wenn True zurückgegeben wird
                self.change_parking_phase(ParkingPhase.MANEUVER_COMPLETE)


    def follow_lane_slowly(self):
        """Fährt langsam geradeaus und versucht, den Lane-Offset auszugleichen."""
        twist_msg = Twist()
        twist_msg.linear.x = PARKING_LINEAR_SPEED # Einheitliche langsame Geschwindigkeit
        # Begrenze den Drehwinkel basierend auf dem Offset, um Übersteuern zu vermeiden
        raw_angular_z = self.current_center_offset * 1.0 # P-Regler für Offset; Faktor ggf. anpassen
        twist_msg.angular.z = float(np.clip(raw_angular_z, -MAX_ANGULAR_Z_LANE_FOLLOW, MAX_ANGULAR_Z_LANE_FOLLOW))
        self.cmd_vel_publisher.publish(twist_msg)

    def move_straight(self, speed: float):
        """Fährt mit der gegebenen Geschwindigkeit geradeaus."""
        twist_msg = Twist()
        twist_msg.linear.x = speed
        twist_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist_msg)

    def turn_robot(self, angular_speed: float):
        """Dreht den Roboter mit der gegebenen Winkelgeschwindigkeit."""
        twist_msg = Twist()
        twist_msg.linear.x = 0.0 # Keine Vorwärtsbewegung beim Drehen
        twist_msg.angular.z = angular_speed
        self.cmd_vel_publisher.publish(twist_msg)

    def stop_robot(self):
        """Stoppt alle Bewegungen des Roboters."""
        #self.get_logger().debug("Roboter gestoppt.")
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist_msg)

    def normalize_angle(self, angle: float) -> float:
        """Normalisiert einen Winkel auf den Bereich [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def turn_to_target(self, target_yaw: float, angular_speed_abs: float) -> bool:
        """Dreht den Roboter zu einem Ziel-Yaw-Winkel.
        Gibt True zurück, wenn das Ziel erreicht ist, sonst False.
        """
        angle_diff = self.normalize_angle(target_yaw - self.current_yaw)

        if abs(angle_diff) < GOAL_TOLERANCE_ANGLE_RAD:
            #self.get_logger().debug(f"Zieldrehung erreicht. Diff: {angle_diff:.3f}rad")
            self.stop_robot()
            return True # Ziel erreicht
        else:
            if angular_speed_abs == 0:
                # self.get_logger().warn("turn_to_target mit angular_speed_abs = 0 aufgerufen. Kann nicht drehen.", throttle_duration_sec=10)
                self.stop_robot()
                return False # Kann nicht drehen, wenn Geschwindigkeit 0 ist

            # Bestimme Drehrichtung
            turn_speed_actual = angular_speed_abs
            if angle_diff < 0: # Ziel ist links (gegen den Uhrzeigersinn für positive Winkel)
                turn_speed_actual = -angular_speed_abs
            # self.get_logger().debug(f"Drehe... Diff: {angle_diff:.3f}rad, Speed: {turn_speed_actual:.2f}rad/s")
            self.turn_robot(turn_speed_actual)
            return False # Ziel noch nicht erreicht

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
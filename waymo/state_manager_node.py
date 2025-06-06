# state_manager_node.py

import rclpy
import rclpy.node
import numpy as np
import time
import sys
import traceback

from std_msgs.msg import String, Float64, Bool
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# --- Zustand für manuelle Pause ---
MANUAL_PAUSE_STATE = 'MANUAL_PAUSE'
KEYBOARD_COMMAND_TOPIC = '/keyboard_command'

# --- Roboter Zustände ---
STATE_FOLLOW_LANE = 'FOLLOW_LANE'
STATE_STOPPED_AT_OBSTACLE = 'STOPPED_AT_OBSTACLE'
STATE_PASSING_OBSTACLE = 'PASSING_OBSTACLE'
STATE_STOPPED_AT_TRAFFIC_LIGHT = 'STOPPED_AT_TRAFFIC_LIGHT'
STATE_PARKING = 'PARKING'
STATE_INTERSECTION_DRIVING_STRAIGHT = 'INTERSEC_DRIVING_STRAIGHT'
STATE_INTERSECTION_TURNING_RIGHT = 'INTERSEC_TURNING_RIGHT'
STATE_INTERSECTION_TURNING_LEFT = 'INTERSEC_TURNING_LEFT'


class StateMachine(rclpy.node.Node):

    def __init__(self):
        super().__init__('state_manager_node')
        self.declare_parameter('fallback_drivingspeed', 0.1)  # Fallback-Geschwindigkeit in m/s

        # Interne Variablen
        self.center_offset = 0.0
        self.state = STATE_STOPPED_AT_TRAFFIC_LIGHT
        self.manual_pause_active = False
        self.obstacle_is_blocking = False
        self.traffic_light_is_red = True 
        self.obstacle_just_passed = False
        self.initial_traffic_light_check_done = False
        
        self.parking_sign_visually_detected = False
        self.parking_maneuver_finished = False
        
        self.straight_sign_visually_detected = False
        self.right_sign_visually_detected = False
        self.left_sign_visually_detected = False
        self.intersection_maneuver_finished = False

        self.parking_sign_visually_detected = False # Um zu wissen, dass das Schild mal gesehen wurde
        self.parking_maneuver_finished = False # Flag, um den Abschluss des Parkens zu erkennen

        self.recommended_speed = self.get_parameter('fallback_drivingspeed').value # Initialwert

        # QoS Profile
        qos_sensor = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        qos_reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1)

        self.obstacle_subscription = self.create_subscription(Bool, '/obstacle/blocked', self.obstacle_detection_callback, qos_sensor)
        self.offset_subscription = self.create_subscription(Float64, '/lane/center_offset', self.lane_detection_callback, qos_sensor)
        self.passed_subscription = self.create_subscription(Bool, '/obstacle/passed', self.obstacle_passed_callback, qos_reliable)
        self.traffic_light_subscription = self.create_subscription(Bool, '/traffic_light', self.traffic_light_callback, qos_reliable)
        self.keyboard_cmd_subscription = self.create_subscription(String, KEYBOARD_COMMAND_TOPIC, self.keyboard_command_callback, qos_reliable)
        self.sign_subscription = self.create_subscription(String, '/sign', self.sign_detection_callback, qos_reliable)
        self.parking_finished_subscription = self.create_subscription(Bool, '/parking/finished', self.parking_finished_callback, qos_reliable)
        self.intersection_finished_sub = self.create_subscription(Bool, '/intersection/finished', self.intersection_finished_callback, qos_reliable) # Name war vorher doppelt

        self.recommended_speed_subscription = self.create_subscription(Float64, '/robot/recommended_speed', self.recommended_speed_callback, qos_reliable)

        self.state_publisher_ = self.create_publisher(String, 'robot/state', qos_reliable)
        self.twist_publisher_ = self.create_publisher(Twist, 'cmd_vel', qos_reliable)

        control_loop_period = 0.005 
        self.control_timer = self.create_timer(control_loop_period, self.control_loop_callback)

        self.publish_current_state()
        self.send_cmd_vel(0.0, 0.0)

    def recommended_speed_callback(self, msg: Float64):
        self.recommended_speed = msg.data

    def keyboard_command_callback(self, msg: String):
        if msg.data == 'toggle_pause':
            self.toggle_manual_pause()

    def toggle_manual_pause(self):
        if not self.manual_pause_active:
            self.manual_pause_active = True
            self.send_cmd_vel(0.0, 0.0) 
            pause_state_msg = String(); pause_state_msg.data = MANUAL_PAUSE_STATE
            try:
                 if rclpy.ok() and self.context.ok(): self.state_publisher_.publish(pause_state_msg)
            except Exception: pass
        else:
            self.manual_pause_active = False

    def obstacle_detection_callback(self, msg: Bool):
        if self.manual_pause_active: return
        self.obstacle_is_blocking = msg.data

    def lane_detection_callback(self, msg: Float64):
        self.center_offset = msg.data

    def obstacle_passed_callback(self, msg: Bool):
        if self.manual_pause_active: return
        if msg.data and self.state == STATE_PASSING_OBSTACLE or self.state == STATE_FOLLOW_LANE:
             self.obstacle_just_passed = True # Wird in control_loop verarbeitet

    def traffic_light_callback(self, msg: Bool):
        if self.manual_pause_active: return
        if self.initial_traffic_light_check_done: return
        is_red = not msg.data
        if self.traffic_light_is_red and not is_red :
            self.traffic_light_is_red = False
            self.initial_traffic_light_check_done = True
            if self.traffic_light_subscription:
                self.destroy_subscription(self.traffic_light_subscription)
                self.traffic_light_subscription = None
        elif not self.traffic_light_is_red and is_red: 
            self.traffic_light_is_red = True

    def sign_detection_callback(self, msg: String):
        if self.manual_pause_active: return

        # Temporäre Variablen für die empfangene Nachricht
        _straight_detected = False
        _right_detected = False
        _left_detected = False
        _parking_detected = False

        if msg.data == "straight_sign_detected":
            _straight_detected = True
        elif msg.data == "right_sign_detected":
            _right_detected = True
        elif msg.data == "left_sign_detected":
            _left_detected = True
        elif msg.data == "parking_sign_detected":
            _parking_detected = True

        # Parkschild-Logik (kann parallel zu Kreuzungsschildern existieren oder diese überschreiben, je nach Anforderung)
        # Hier: Parken wird separat behandelt und beeinflusst Kreuzungsschilder nicht direkt beim Setzen.
        if _parking_detected:
            self.parking_sign_visually_detected = True
            # Optional: Wenn Parkschild erkannt, andere Schilder ignorieren/zurücksetzen?
            # Fürs Erste lassen wir das so, da Parken eine höhere Priorität in der control_loop hat.

        # Exklusives Setzen für Kreuzungsschilder
        if _straight_detected:
            # self.get_logger().info("Visuelles Geradeaus-Schild Signal empfangen.")
            self.straight_sign_visually_detected = True
            self.right_sign_visually_detected = False
            self.left_sign_visually_detected = False
        elif _right_detected:
            # self.get_logger().info("Visuelles Rechts-Schild Signal empfangen.")
            self.right_sign_visually_detected = True
            self.straight_sign_visually_detected = False
            self.left_sign_visually_detected = False
        elif _left_detected:
            # self.get_logger().info("Visuelles Links-Schild Signal empfangen.")
            self.left_sign_visually_detected = True
            self.straight_sign_visually_detected = False
            self.right_sign_visually_detected = False
        
        # Wenn kein Kreuzungsschild aktiv erkannt wurde, aber eines der Flags noch True ist,
        # könnte es hier ein Problem geben. Normalerweise werden sie in der control_loop zurückgesetzt.
        # Die obige Logik stellt sicher, dass immer nur ein Kreuzungsflag aktiv ist basierend auf der *letzten* Nachricht.

    def parking_finished_callback(self, msg: Bool):
        if self.manual_pause_active: return
        if msg.data:
            self.parking_maneuver_finished = True
    
    def intersection_finished_callback(self, msg: Bool):
        if self.manual_pause_active: return
        # msg.data ist True, wenn Manöver erfolgreich, False bei Abbruch durch intersection_node
        if msg.data:
            self.intersection_maneuver_finished = True
            # self.get_logger().info("Intersection_finished_callback: Manöver erfolgreich beendet (True).")
        else:
            # Manöver fehlgeschlagen oder von intersection_node abgebrochen
            self.intersection_maneuver_finished = True # Trotzdem als "finished" markieren, um aus dem Zustand rauszukommen
            # self.get_logger().warn("Intersection_finished_callback: Manöver NICHT erfolgreich (False erhalten). Setze trotzdem finished.")
            # Die Flags straight/left/right_sign_visually_detected werden in der control_loop (l) zurückgesetzt.

    def control_loop_callback(self):
        if self.manual_pause_active:
             self.send_cmd_vel(0.0, 0.0)
             self.publish_current_state()
             return

        current_internal_state = self.state
        next_state = current_internal_state

        if not self.initial_traffic_light_check_done:
            if self.traffic_light_is_red:
                next_state = STATE_STOPPED_AT_TRAFFIC_LIGHT
            else: 
                self.initial_traffic_light_check_done = True
                next_state = STATE_FOLLOW_LANE
        else:
            # --- DEBUG LOGGING VOR KREUZUNGSENTSCHEIDUNG ---
            # self.get_logger().debug(f"Control Loop Check: Current State: {current_internal_state}, "
            #                        f"Straight: {self.straight_sign_visually_detected}, "
            #                        f"Right: {self.right_sign_visually_detected}, "
            #                        f"Left: {self.left_sign_visually_detected}, "
            #                        f"IntersectionFinished: {self.intersection_maneuver_finished}")
            # ---------------------------------------------

            if current_internal_state == STATE_PASSING_OBSTACLE and self.obstacle_just_passed:
                next_state = STATE_FOLLOW_LANE
                self.obstacle_just_passed = False
                self.obstacle_is_blocking = False
            elif current_internal_state == STATE_PASSING_OBSTACLE:
                self.obstacle_is_blocking = False # Annahme: Wir sind in der Umfahrung, also ist das Hindernis nicht mehr blockierend
                pass # Bleibe in PASSING_OBSTACLE, Steuerung liegt bei passing_obstacle_node

            # c) Parkschild erkannt und bereit zum Parken?
            #    (Nicht während PASSING_OBSTACLE oder wenn an Ampel gestoppt)
            elif self.parking_sign_visually_detected and \
                 current_internal_state not in [STATE_PASSING_OBSTACLE, STATE_PARKING, STATE_STOPPED_AT_TRAFFIC_LIGHT,
                                                STATE_INTERSECTION_DRIVING_STRAIGHT, STATE_INTERSECTION_TURNING_LEFT, STATE_INTERSECTION_TURNING_RIGHT]: # Intersection States auch ausschließen
                next_state = STATE_PARKING
                self.parking_sign_visually_detected = False 
                self.parking_maneuver_finished = False 
            elif current_internal_state == STATE_PARKING and self.parking_maneuver_finished:
                next_state = STATE_FOLLOW_LANE
                self.parking_sign_visually_detected = False 
                self.parking_maneuver_finished = False 
            elif current_internal_state == STATE_PARKING and not self.parking_maneuver_finished:
                pass
            elif self.obstacle_is_blocking and \
                 current_internal_state not in [STATE_PASSING_OBSTACLE, STATE_PARKING, STATE_STOPPED_AT_OBSTACLE]:
                next_state = STATE_STOPPED_AT_OBSTACLE
            elif current_internal_state == STATE_STOPPED_AT_OBSTACLE and self.obstacle_is_blocking:
                 next_state = STATE_PASSING_OBSTACLE
            
            # Kreuzungslogik:
            # l) Manöver abgeschlossen
            elif (current_internal_state == STATE_INTERSECTION_DRIVING_STRAIGHT or \
                  current_internal_state == STATE_INTERSECTION_TURNING_RIGHT or \
                  current_internal_state == STATE_INTERSECTION_TURNING_LEFT) and \
                 self.intersection_maneuver_finished:
                # self.get_logger().info(f"Zustand {current_internal_state} BEENDET, intersection_maneuver_finished={self.intersection_maneuver_finished}. Gehe zu FOLLOW_LANE.")
                next_state = STATE_FOLLOW_LANE
                self.intersection_maneuver_finished = False 
                self.straight_sign_visually_detected = False 
                self.right_sign_visually_detected = False    
                self.left_sign_visually_detected = False     
            
            # i) Geradeaus-Schild
            elif self.straight_sign_visually_detected and \
                    current_internal_state not in [
                        STATE_PASSING_OBSTACLE, STATE_PARKING, STATE_STOPPED_AT_TRAFFIC_LIGHT,
                        STATE_INTERSECTION_DRIVING_STRAIGHT, STATE_INTERSECTION_TURNING_LEFT, STATE_INTERSECTION_TURNING_RIGHT
                    ]:
                # self.get_logger().info(f"Bedingung (i) WAHR: straight_sign={self.straight_sign_visually_detected}. Wechsel zu INTERSEC_DRIVING_STRAIGHT.")
                next_state = STATE_INTERSECTION_DRIVING_STRAIGHT
                self.intersection_maneuver_finished = False 
                self.straight_sign_visually_detected = False 
            
            # j) Rechts-Schild
            elif self.right_sign_visually_detected and \
                 current_internal_state not in [
                        STATE_PASSING_OBSTACLE, STATE_PARKING, STATE_STOPPED_AT_TRAFFIC_LIGHT,
                        STATE_INTERSECTION_DRIVING_STRAIGHT, STATE_INTERSECTION_TURNING_LEFT, STATE_INTERSECTION_TURNING_RIGHT
                    ]:
                # self.get_logger().info(f"Bedingung (j) WAHR: right_sign={self.right_sign_visually_detected}. Wechsel zu INTERSEC_TURNING_RIGHT.")
                next_state = STATE_INTERSECTION_TURNING_RIGHT
                self.intersection_maneuver_finished = False 
                self.right_sign_visually_detected = False 

            # k) Links-Schild
            elif self.left_sign_visually_detected and \
                 current_internal_state not in [
                        STATE_PASSING_OBSTACLE, STATE_PARKING, STATE_STOPPED_AT_TRAFFIC_LIGHT,
                        STATE_INTERSECTION_DRIVING_STRAIGHT, STATE_INTERSECTION_TURNING_LEFT, STATE_INTERSECTION_TURNING_RIGHT
                    ]:
                # self.get_logger().info(f"Bedingung (k) WAHR: left_sign={self.left_sign_visually_detected}. Wechsel zu INTERSEC_TURNING_LEFT.")
                next_state = STATE_INTERSECTION_TURNING_LEFT
                self.intersection_maneuver_finished = False 
                self.left_sign_visually_detected = False 

            # m) Bereits in einem Intersection State und Manöver läuft noch
            elif (current_internal_state == STATE_INTERSECTION_DRIVING_STRAIGHT or \
                  current_internal_state == STATE_INTERSECTION_TURNING_RIGHT or \
                  current_internal_state == STATE_INTERSECTION_TURNING_LEFT) and \
                 not self.intersection_maneuver_finished:
                pass # Steuerung liegt bei intersection_node
            
            # h) Kein Hindernis und nicht in Spezialzustand -> FOLLOW_LANE
            elif not self.obstacle_is_blocking and current_internal_state not in [ 
                STATE_PASSING_OBSTACLE, STATE_PARKING, STATE_FOLLOW_LANE, 
                STATE_INTERSECTION_DRIVING_STRAIGHT, STATE_INTERSECTION_TURNING_RIGHT, 
                STATE_INTERSECTION_TURNING_LEFT]:
                next_state = STATE_FOLLOW_LANE
            
            # n) Fallback auf FOLLOW_LANE wenn in FOLLOW_LANE und nichts anderes zutrifft
            elif current_internal_state == STATE_FOLLOW_LANE and not (
                    self.obstacle_is_blocking or \
                    self.parking_sign_visually_detected or \
                    self.straight_sign_visually_detected or \
                    self.right_sign_visually_detected or \
                    self.left_sign_visually_detected): # intersection_maneuver_finished ist hier nicht relevant
                pass # Bleibe in FOLLOW_LANE

        if next_state != current_internal_state:
            self.change_state(next_state)

        if self.state == STATE_FOLLOW_LANE:
            driving_speed = self.recommended_speed
            angular_z = self.center_offset
            angular_z = np.clip(angular_z, -1.0, 1.0)
            self.send_cmd_vel(driving_speed, angular_z)
        elif self.state in [STATE_STOPPED_AT_OBSTACLE, STATE_STOPPED_AT_TRAFFIC_LIGHT]:
            self.send_cmd_vel(0.0, 0.0)
        elif self.state == STATE_PASSING_OBSTACLE:
            pass
        elif self.state == STATE_PARKING:
            pass
        elif self.state in [STATE_INTERSECTION_DRIVING_STRAIGHT, STATE_INTERSECTION_TURNING_RIGHT, STATE_INTERSECTION_TURNING_LEFT]:
            pass

        self.publish_current_state()

    def change_state(self, new_state):
        if self.state != new_state:
            # self.get_logger().info(f"Zustandswechsel: {self.state} -> {new_state}")
            self.state = new_state
            if self.state in [STATE_STOPPED_AT_OBSTACLE, STATE_STOPPED_AT_TRAFFIC_LIGHT]:
                 self.send_cmd_vel(0.0, 0.0)
            self.publish_current_state()

    def publish_current_state(self):
        state_to_publish = MANUAL_PAUSE_STATE if self.manual_pause_active else self.state
        state_msg = String(); state_msg.data = state_to_publish
        try:
             if rclpy.ok() and self.context.ok(): self.state_publisher_.publish(state_msg)
        except Exception: pass

    def send_cmd_vel(self, linear_x, angular_z):
       is_stop_command = abs(linear_x) < 0.001 and abs(angular_z) < 0.001
       if self.manual_pause_active and not is_stop_command:
            return 
       twist = Twist(); twist.linear.x = linear_x; twist.angular.z = float(angular_z)
       try:
           if rclpy.ok() and self.context.ok(): self.twist_publisher_.publish(twist)
       except Exception: pass

    def destroy_node(self):
        try:
             if rclpy.ok() and self.context.ok():
                  original_pause_state = self.manual_pause_active
                  self.manual_pause_active = False 
                  self.send_cmd_vel(0.0, 0.0)
                  time.sleep(0.1) 
                  self.manual_pause_active = original_pause_state
        except Exception as e:
            self.get_logger().warn(f"Fehler beim Senden des Stoppbefehls in destroy_node: {e}")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args); node = None
    try:
        node = StateMachine()
        try: time.sleep(0.5) 
        except KeyboardInterrupt: raise 
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if node: node.get_logger().error(f"FATAL ERROR [SM] in main: {e}\n{traceback.format_exc()}")
        else: print(f"FATAL ERROR [SM] vor Node-Init: {e}\n{traceback.format_exc()}", file=sys.stderr)
    finally:
        if node is not None: node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()
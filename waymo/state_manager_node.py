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


class StateMachine(rclpy.node.Node):

    def __init__(self):
        super().__init__('state_manager_node')
        self.declare_parameter('fallback_drivingspeed', 0.1)  # Fallback-Geschwindigkeit in m/s

        # Interne Variablen
        self.center_offset = 0.0
        # Startzustand bleibt STOPPED_AT_TRAFFIC_LIGHT
        self.state = STATE_STOPPED_AT_TRAFFIC_LIGHT
        self.manual_pause_active = False
        self.obstacle_is_blocking = False
        self.traffic_light_is_red = True # Annahme: Ampel ist initial rot
        self.obstacle_just_passed = False
        self.initial_traffic_light_check_done = False
        self.parking_sign_visually_detected = False # Um zu wissen, dass das Schild mal gesehen wurde
        self.parking_maneuver_finished = False # Flag, um den Abschluss des Parkens zu erkennen

        self.recommended_speed = self.get_parameter('fallback_drivingspeed').value # Initialwert

        # QoS Profile
        qos_sensor = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        qos_reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1)

        # Subscribers
        self.obstacle_subscription = self.create_subscription(Bool, '/obstacle/blocked', self.obstacle_detection_callback, qos_sensor)
        self.offset_subscription = self.create_subscription(Float64, '/lane/center_offset', self.lane_detection_callback, qos_sensor)
        self.passed_subscription = self.create_subscription(Bool, '/obstacle/passed', self.obstacle_passed_callback, qos_reliable)
        self.traffic_light_subscription = self.create_subscription(Bool, '/traffic_light', self.traffic_light_callback, qos_reliable)
        self.keyboard_cmd_subscription = self.create_subscription(String, KEYBOARD_COMMAND_TOPIC, self.keyboard_command_callback, qos_reliable)
        self.sign_subscription = self.create_subscription(String, '/sign', self.sign_detection_callback, qos_reliable)
        self.parking_finished_subscription = self.create_subscription(Bool, '/parking/finished', self.parking_finished_callback, qos_reliable)

        self.recommended_speed_subscription = self.create_subscription(Float64, '/robot/recommended_speed', self.recommended_speed_callback, qos_reliable)

        # Publishers
        self.state_publisher_ = self.create_publisher(String, 'robot/state', qos_reliable)
        self.twist_publisher_ = self.create_publisher(Twist, 'cmd_vel', qos_reliable)

        # Control Timer
        control_loop_period = 0.005 # 200 Hz
        self.control_timer = self.create_timer(control_loop_period, self.control_loop_callback)

        # Initiale Zustandsmeldung und Stopp
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
            self.send_cmd_vel(0.0, 0.0) # Sofort anhalten
            pause_state_msg = String(); pause_state_msg.data = MANUAL_PAUSE_STATE
            try:
                 if rclpy.ok() and self.context.ok(): self.state_publisher_.publish(pause_state_msg)
            except Exception: pass
            # self.get_logger().info("Roboter manuell pausiert.")
        else:
            self.manual_pause_active = False
            # Beim Verlassen der Pause nicht direkt in FOLLOW_LANE, sondern den Zustand
            # durch die control_loop Logik neu evaluieren lassen.
            # Die publish_current_state() wird im Loop aufgerufen.
            # self.get_logger().info("Manuelle Pause beendet. Evaluierung des Zustands...")
            # Stelle sicher, dass der interne Zustand nicht MANUAL_PAUSE ist,
            # damit die control_loop wieder normal arbeitet.
            # Der vorherige Zustand (vor Pause) wird nicht explizit gespeichert,
            # die Logik entscheidet neu. Wenn er in PARKING war, bleibt er es.
            # Falls er in FOLLOW_LANE soll, wird die control_loop das entscheiden.


    def obstacle_detection_callback(self, msg: Bool):
        if self.manual_pause_active: return

        self.obstacle_is_blocking = msg.data
        # Der Übergang zu PASSING_OBSTACLE wird in der control_loop gehandhabt.

    def lane_detection_callback(self, msg: Float64):
        self.center_offset = msg.data

    def obstacle_passed_callback(self, msg: Bool):
        if self.manual_pause_active: return
        if msg.data and self.state == STATE_PASSING_OBSTACLE or self.state == STATE_FOLLOW_LANE:
             self.obstacle_just_passed = True # Wird in control_loop verarbeitet

    def traffic_light_callback(self, msg: Bool):
        if self.manual_pause_active: return
        # Dieser Callback wird nur einmal relevant sein (bis initial_traffic_light_check_done True ist)
        if self.initial_traffic_light_check_done: return

        is_red = not msg.data # traffic_light_node sendet True für "Go" (kein Rot), False für "Stop" (Rot erkannt)
        
        if self.traffic_light_is_red and not is_red : # War rot, ist jetzt grün
            # self.get_logger().info("Ampel ist GRÜN. Initiale Prüfung abgeschlossen.")
            self.traffic_light_is_red = False
            self.initial_traffic_light_check_done = True
            # Subscription kann hier deregistriert werden, um Ressourcen zu sparen
            if self.traffic_light_subscription:
                self.destroy_subscription(self.traffic_light_subscription)
                self.traffic_light_subscription = None
                # self.get_logger().info("Traffic light subscription wurde beendet.")
        elif not self.traffic_light_is_red and is_red: # War grün, ist jetzt rot (sollte initial nicht passieren)
            self.traffic_light_is_red = True
            # self.get_logger().warn("Ampel wurde ROT, obwohl sie schon grün war (vor initial check done).")
        # else: Kein relevanter Wechsel für die initiale Logik


    def sign_detection_callback(self, msg: String):
        if self.manual_pause_active: return

        if msg.data == "parking_sign_detected":
            # self.get_logger().info("Visuelles Parkschild Signal empfangen.")
            self.parking_sign_visually_detected = True
            # Der eigentliche Zustandswechsel zu PARKING wird in der control_loop entschieden,
            # um Prioritäten (z.B. nicht während PASSING_OBSTACLE) zu berücksichtigen.
        # Andere Schilder könnten hier behandelt werden, falls die msg.data erweitert wird.

    def parking_finished_callback(self, msg: Bool):
        if self.manual_pause_active: return

        if msg.data:
            self.parking_maneuver_finished = True

    def control_loop_callback(self):
        if self.manual_pause_active:
             # Im manuellen Pause-Zustand wird immer gestoppt und der Zustand MANUAL_PAUSE publiziert
             self.send_cmd_vel(0.0, 0.0)
             self.publish_current_state() # Stellt sicher, dass MANUAL_PAUSE gesendet wird
             return

        current_internal_state = self.state # Der von der State Machine verwaltete Zustand
        next_state = current_internal_state

        # --- Zustandsübergangslogik ---

        # 1. Höchste Priorität: Initiale Ampelphase (wenn noch nicht abgeschlossen)
        if not self.initial_traffic_light_check_done:
            if self.traffic_light_is_red:
                next_state = STATE_STOPPED_AT_TRAFFIC_LIGHT
            else: # Ampel ist grün geworden (sollte durch callback passieren, aber als Fallback)
                self.initial_traffic_light_check_done = True
                # self.get_logger().info("Ampel grün in control_loop erkannt (Fallback).")
                # Nach Ampel standardmäßig in FOLLOW_LANE, es sei denn, andere Bedingungen greifen sofort
                next_state = STATE_FOLLOW_LANE
        
        # 2. Nach initialer Ampelphase:
        else:
            # a) Hindernisumfahrung abgeschlossen?
            if current_internal_state == STATE_PASSING_OBSTACLE and self.obstacle_just_passed:
                # self.get_logger().info("Hindernisumfahrung abgeschlossen. Wechsel zu FOLLOW_LANE.")
                next_state = STATE_FOLLOW_LANE
                self.obstacle_just_passed = False # Reset Flag
                self.obstacle_is_blocking = False # Annahme: Nach dem Passieren ist der Weg erstmal frei

            # b) Aktive Hindernisumfahrung (hat Vorrang vor Parken oder normalem Fahren)
            elif current_internal_state == STATE_PASSING_OBSTACLE:
                self.obstacle_is_blocking = False # Annahme: Wir sind in der Umfahrung, also ist das Hindernis nicht mehr blockierend
                pass # Bleibe in PASSING_OBSTACLE, Steuerung liegt bei passing_obstacle_node

            # c) Parkschild erkannt und bereit zum Parken?
            #    (Nicht während PASSING_OBSTACLE oder wenn an Ampel gestoppt)
            elif self.parking_sign_visually_detected and \
                 current_internal_state not in [STATE_PASSING_OBSTACLE, STATE_PARKING, STATE_STOPPED_AT_TRAFFIC_LIGHT]:
                # self.get_logger().info("Visuelles Parkschild wurde erkannt. Wechsel zu STATE_PARKING.")
                next_state = STATE_PARKING
                self.parking_sign_visually_detected = False # Reset Flag, da der Zustand erreicht wird
                self.parking_maneuver_finished = False # Reset, da wir jetzt parken wollen
            
            # d) Im Zustand PARKING und Parkmanöver abgeschlossen:
            elif current_internal_state == STATE_PARKING and self.parking_maneuver_finished:
                next_state = STATE_FOLLOW_LANE
                self.parking_sign_visually_detected = False # Reset des Schild-Flags
                self.parking_maneuver_finished = False # Reset des Abschluss-Flags

            # e) Im Zustand PARKING, aber noch nicht abgeschlossen:
            elif current_internal_state == STATE_PARKING and not self.parking_maneuver_finished:
                # Die parking_node steuert die Bewegung und das Anhalten.
                # Der state_manager bleibt in diesem Zustand, bis die parking_finished_callback aufgerufen wird.
                pass

            # f) Blockierendes Hindernis erkannt (und nicht schon in Umfahrung oder Parken)
            elif self.obstacle_is_blocking and \
                 current_internal_state not in [STATE_PASSING_OBSTACLE, STATE_PARKING, STATE_STOPPED_AT_OBSTACLE]:
                # self.get_logger().info("Hindernis blockiert den Weg. Wechsel zu STOPPED_AT_OBSTACLE.")
                next_state = STATE_STOPPED_AT_OBSTACLE
            
            # g) Von STOPPED_AT_OBSTACLE zu PASSING_OBSTACLE (wenn Hindernis immer noch da)
            #    Diese Logik war in deinem obstacle_detection_callback, ist hier besser aufgehoben.
            elif current_internal_state == STATE_STOPPED_AT_OBSTACLE and self.obstacle_is_blocking:
                #  self.get_logger().info("Hindernis immer noch da. Starte PASSING_OBSTACLE.")
                 next_state = STATE_PASSING_OBSTACLE
            
            # h) Kein Hindernis und nicht in Spezialzustand -> FOLLOW_LANE
            #    (Auch wenn von STOPPED_AT_OBSTACLE kommend und Hindernis weg ist)
            elif not self.obstacle_is_blocking and \
                 current_internal_state not in [STATE_PASSING_OBSTACLE, STATE_PARKING, STATE_FOLLOW_LANE]:
                # self.get_logger().info("Kein Hindernis (mehr) und nicht in Spezialzustand. Wechsel zu FOLLOW_LANE.")
                next_state = STATE_FOLLOW_LANE
            
            # i) Wenn bereits in FOLLOW_LANE und keine anderen Bedingungen zutreffen, bleibe dabei.
            elif current_internal_state == STATE_FOLLOW_LANE and not self.obstacle_is_blocking and not self.parking_sign_visually_detected and not self.parking_maneuver_finished:
                pass


        # Zustandswechsel durchführen, wenn nötig
        if next_state != current_internal_state:
            self.change_state(next_state)

        # --- Aktionen basierend auf dem aktuellen Zustand ---
        if self.state == STATE_FOLLOW_LANE:
            driving_speed = self.recommended_speed
            angular_z = self.center_offset
            angular_z = np.clip(angular_z, -1.0, 1.0)
            self.send_cmd_vel(driving_speed, angular_z)
        elif self.state in [STATE_STOPPED_AT_OBSTACLE, STATE_STOPPED_AT_TRAFFIC_LIGHT]:
            self.send_cmd_vel(0.0, 0.0)
        elif self.state == STATE_PASSING_OBSTACLE:
            # Keine Geschwindigkeitsbefehle senden, wird von passing_obstacle_node übernommen
            pass
        elif self.state == STATE_PARKING:
            # Keine Geschwindigkeitsbefehle senden, wird von parking_node übernommen
            pass


        self.publish_current_state() # Zustand periodisch senden

    def change_state(self, new_state):
        # Diese Funktion wird nur aufgerufen, wenn sich der interne Zustand wirklich ändert.
        # Die manuelle Pause wird in toggle_manual_pause und im control_loop separat behandelt.
        if self.state != new_state:
            # self.get_logger().info(f"Zustandswechsel: {self.state} -> {new_state}")
            self.state = new_state
            
            # Sende sofort einen Stoppbefehl bei bestimmten Zustandswechseln,
            # bevor die jeweilige Node die Kontrolle übernimmt oder der Loop reagiert.
            if self.state in [STATE_STOPPED_AT_OBSTACLE, STATE_STOPPED_AT_TRAFFIC_LIGHT]:
                 self.send_cmd_vel(0.0, 0.0)
            # Bei Wechsel zu PASSING_OBSTACLE oder PARKING wird die Steuerung an die dedizierten Nodes übergeben.
            # Ein initialer Stopp ist nicht unbedingt nötig, da diese Nodes ihre eigene Logik haben.
            
            self.publish_current_state() # Den neuen Zustand sofort publizieren

    def publish_current_state(self):
        # Wählt den zu publizierenden Zustand: MANUAL_PAUSE hat Vorrang vor dem internen Zustand.
        state_to_publish = MANUAL_PAUSE_STATE if self.manual_pause_active else self.state
        state_msg = String(); state_msg.data = state_to_publish
        try:
             if rclpy.ok() and self.context.ok(): self.state_publisher_.publish(state_msg)
        except Exception: pass # Fehler beim Publizieren ignorieren, um den Loop nicht zu stören

    def send_cmd_vel(self, linear_x, angular_z):
       # Sendet nur, wenn nicht manuell pausiert (es sei denn, es ist ein Stoppbefehl)
       is_stop_command = abs(linear_x) < 0.001 and abs(angular_z) < 0.001
       if self.manual_pause_active and not is_stop_command:
            # self.get_logger().debug("Sende keinen cmd_vel wegen manueller Pause (außer Stopp).")
            return 
       
       twist = Twist(); twist.linear.x = linear_x; twist.angular.z = float(angular_z)
       try:
           if rclpy.ok() and self.context.ok(): self.twist_publisher_.publish(twist)
       except Exception: pass

    def destroy_node(self):
        # self.get_logger().info("State Manager Node wird heruntergefahren.")
        # Sicherstellen, dass der Roboter beim Beenden stoppt
        try:
             if rclpy.ok() and self.context.ok():
                  # Wichtig: Manuelle Pause aufheben, damit Stoppbefehl durchgeht, falls aktiv
                  original_pause_state = self.manual_pause_active
                  self.manual_pause_active = False 
                  self.send_cmd_vel(0.0, 0.0)
                  time.sleep(0.1) # Kurz warten, damit der Befehl gesendet werden kann
                  self.manual_pause_active = original_pause_state # Zustand wiederherstellen (optional)
                #   self.get_logger().info("Stoppbefehl beim Herunterfahren gesendet.")
        except Exception as e:
            self.get_logger().warn(f"Fehler beim Senden des Stoppbefehls in destroy_node: {e}")
        super().destroy_node()

# --- main Funktion mit Delay ---
def main(args=None):
    rclpy.init(args=args); node = None
    try:
        node = StateMachine()
        # Kurze Pause nach Initialisierung, um sicherzustellen, dass andere Nodes evtl. schon publishen
        try: time.sleep(0.5) 
        except KeyboardInterrupt: raise # KeyboardInterrupt hier schon abfangen
        rclpy.spin(node)
    except KeyboardInterrupt:
        # self.get_logger().info("KeyboardInterrupt erhalten, State Manager wird beendet.") # Geht nicht, da node ggf. None
        pass
    except Exception as e:
        # Logge Fehler, bevor Node zerstört wird
        if node: node.get_logger().error(f"FATAL ERROR [SM] in main: {e}\n{traceback.format_exc()}")
        else: print(f"FATAL ERROR [SM] vor Node-Init: {e}\n{traceback.format_exc()}", file=sys.stderr)
    finally:
        if node is not None: node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()
import rclpy
import rclpy.node
import numpy as np
import time
import sys
import traceback

from std_msgs.msg import String, Float64, Bool
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.duration import Duration # Für Timer

# --- Neuer Zustand ---
MANUAL_PAUSE_STATE = 'MANUAL_PAUSE'
COMMAND_TOPIC = '/keyboard_command' # Selber Topic Name wie im Handler

class StateMachine(rclpy.node.Node):

    def __init__(self):
        super().__init__('state_manager_node')
        self.declare_parameter('drivingspeed', 0.15)
        self.center_offset = 0.0 # Wird von lane_detection_callback aktualisiert
        self.state = 'WAYMO_STARTED'
        self.manual_pause_active = False # Flag für manuelle Pause

        # QoS Profile (Gemischt - Empfohlen)
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1
        )
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=1
        )

        # Subscribers
        self.obstacle_subscription = self.create_subscription(Bool, 'obstacle/blocked', self.obstacle_detection_callback, qos_sensor)
        self.offset_subscription = self.create_subscription(Float64, 'lane/center_offset', self.lane_detection_callback, qos_sensor)
        self.passed_subscription = self.create_subscription(Bool, '/obstacle/passed', self.obstacle_passed_callback, qos_reliable)
        self.keyboard_cmd_subscription = self.create_subscription(String, COMMAND_TOPIC, self.keyboard_command_callback, qos_reliable)

        # Publishers
        self.state_publisher_ = self.create_publisher(String, 'robot/state', qos_reliable)
        self.twist_publisher_ = self.create_publisher(Twist, 'cmd_vel', qos_reliable)

        # Control Timer
        control_loop_period = 0.005 # 200 Hz
        self.control_timer = self.create_timer(control_loop_period, self.control_loop_callback)

        # self.get_logger().info('State Machine Node started (Timer-based P-Controller, Keyboard Pause Enabled)')
        self.publish_current_state() # Initialen Zustand senden
        self.send_cmd_vel(0.0, 0.0) # Sicherstellen, dass Roboter initial steht

    def keyboard_command_callback(self, msg: String):
        """Verarbeitet Befehle vom Keyboard Handler Node."""
        if msg.data == 'toggle_pause':
            self.toggle_manual_pause()

    def toggle_manual_pause(self):
        """Schaltet den manuellen Pause-Zustand um."""
        if not self.manual_pause_active:
            # --- PAUSE AKTIVIEREN ---
            self.manual_pause_active = True
            self.send_cmd_vel(0.0, 0.0)
            # Den aktuellen internen Zustand NICHT ändern, nur den veröffentlichten
            pause_state_msg = String()
            pause_state_msg.data = MANUAL_PAUSE_STATE
            self.state_publisher_.publish(pause_state_msg)
        else:
            # --- PAUSE DEAKTIVIEREN ---
            self.manual_pause_active = False
            # Wechsle Zwangsläufig zu FOLLOW_LANE (ruft auch publish_current_state)
            self.change_state('FOLLOW_LANE')

    def control_loop_callback(self):
        """Wird vom Timer aufgerufen, berechnet Steuerung und publiziert Zustand."""
        # --- KEINE AKTION WENN PAUSIERT (ausser State Publish) ---
        if self.manual_pause_active:
             # Publish 'MANUAL_PAUSE' state periodically while paused
             self.publish_current_state()
             return

        current_state = self.state
        driving_speed = 0.0
        angular_z = 0.0

        if current_state == 'FOLLOW_LANE':
            driving_speed = self.get_parameter('drivingspeed').get_parameter_value().double_value
            angular_z = self.center_offset
            max_angular_z = 1.0
            angular_z = np.clip(angular_z, -max_angular_z, max_angular_z)
            self.send_cmd_vel(driving_speed, angular_z)
        # In anderen Zuständen (STOPPED, WAYMO_STARTED) wird Stop von change_state gesendet
        # In PASSING_OBSTACLE wird nichts gesendet

        # --- NEU: Zustand periodisch publishen ---
        # Rufe dies auf, nachdem die Logik für den aktuellen Zustand durchlaufen wurde
        # (ausser wenn pausiert, das wird oben behandelt)
        self.publish_current_state()
        # --- Ende Zustands-Publishing ---

    def change_state(self, new_state):
        """Ändert den Zustand, publiziert ihn und sendet initialen Befehl."""
        if self.manual_pause_active: return # Keine automatischen Änderungen wenn pausiert

        if self.state != new_state:
            # old_state = self.state # Nicht mehr für Log gebraucht
            self.state = new_state
            # self.get_logger().info(f"State change from {old_state} to {self.state}") # Entfernt
            self.publish_current_state() # Neuen Zustand sofort publizieren

            if self.state == 'STOPPED' or self.state == 'WAYMO_STARTED':
                 self.send_cmd_vel(0.0, 0.0) # Stop-Befehl senden

    def obstacle_detection_callback(self, msg: Bool):
        """Verarbeitet Hinderniserkennung und löst Zustandswechsel aus."""
        if self.manual_pause_active: return
        blocked = msg.data
        current_state = self.state
        if blocked:
            if current_state == 'FOLLOW_LANE': self.change_state('STOPPED')
            elif current_state == 'STOPPED': self.change_state('PASSING_OBSTACLE')
        else: # Not blocked
             if current_state == 'STOPPED': self.change_state('FOLLOW_LANE')
             elif current_state == 'WAYMO_STARTED' and current_state != 'FOLLOW_LANE': self.change_state('FOLLOW_LANE')

    def lane_detection_callback(self, msg: Float64):
        """Aktualisiert NUR den Offset-Wert."""
        self.center_offset = msg.data
        # Steuerung wird vom Timer übernommen

    def obstacle_passed_callback(self, msg: Bool):
        """Verarbeitet Signal, dass Hindernis passiert wurde."""
        if self.manual_pause_active: return
        passed = msg.data
        if passed and self.state == 'PASSING_OBSTACLE': self.change_state('FOLLOW_LANE')

    def publish_current_state(self):
        """Publisht den aktuellen oder den Pause-Zustand."""
        state_to_publish = self.state if not self.manual_pause_active else MANUAL_PAUSE_STATE
        state_msg = String(); state_msg.data = state_to_publish
        try:
             # Prüfe ob Kontext noch gültig ist vor dem Publishen
             if rclpy.ok() and self.context.ok(): self.state_publisher_.publish(state_msg)
        except Exception: pass # Ignoriere Fehler beim Herunterfahren

    def send_cmd_vel(self, linear_x, angular_z):
       """Sendet Bewegungsbefehle, aber nicht wenn pausiert."""
       if self.manual_pause_active: return
       twist = Twist(); twist.linear.x = linear_x; twist.angular.z = float(angular_z)
       try:
           if rclpy.ok() and self.context.ok(): self.twist_publisher_.publish(twist)
       except Exception: pass

    def destroy_node(self):
        """Aufräumarbeiten."""
        # self.get_logger().info("Shutting down State Machine Node.") # Entfernt
        try:
             if rclpy.ok() and self.context.ok():
                  self.manual_pause_active = False # Deaktiviere Pause für Stopp-Befehl
                  self.send_cmd_vel(0.0, 0.0); time.sleep(0.1)
        except Exception as e: pass # Fehler ignorieren
            # self.get_logger().warn(f"Could not send stop cmd: {e}") # Entfernt
        super().destroy_node()

# --- main bleibt unverändert ---
def main(args=None):
    rclpy.init(args=args); node = None
    try:
        node = StateMachine()
        rclpy.spin(node)
    except KeyboardInterrupt:
        # if node: node.get_logger().info("KeyboardInterrupt received, shutting down.") # Entfernt
        pass
    except Exception as e:
        print(f"Unhandled exception in StateMachine: {e}", file=sys.stderr)
        traceback.print_exc()
        # if node: node.get_logger().error(f"Unhandled exception in StateMachine: {e}", exc_info=True) # Entfernt
    finally:
        if node is not None: node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()
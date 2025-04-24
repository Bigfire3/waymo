import rclpy
import rclpy.node
import numpy as np
import time
import sys
import traceback

from std_msgs.msg import String, Float64, Bool
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.duration import Duration

# --- Neuer Zustand ---
MANUAL_PAUSE_STATE = 'MANUAL_PAUSE'
COMMAND_TOPIC = '/keyboard_command' # Selber Topic Name wie im Handler

class StateMachine(rclpy.node.Node):

    def __init__(self):
        super().__init__('state_manager_node')
        self.declare_parameter('drivingspeed', 0.15)
        self.center_offset = 0.0
        self.state = 'WAYMO_STARTED'
        self.manual_pause_active = False # Flag für manuelle Pause

        # QoS Profile (Gemischt)
        qos_sensor = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        qos_reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1)

        # Subscribers
        self.obstacle_subscription = self.create_subscription(Bool, 'obstacle/blocked', self.obstacle_detection_callback, qos_sensor)
        self.offset_subscription = self.create_subscription(Float64, 'lane/center_offset', self.lane_detection_callback, qos_sensor)
        self.passed_subscription = self.create_subscription(Bool, '/obstacle/passed', self.obstacle_passed_callback, qos_reliable)
        # --- NEU: Keyboard Command Subscriber ---
        self.keyboard_cmd_subscription = self.create_subscription(
            String,
            COMMAND_TOPIC,
            self.keyboard_command_callback,
            qos_reliable # Befehle sollten Reliable sein
        )
        # --- Ende Keyboard Command Subscriber ---

        # Publishers
        self.state_publisher_ = self.create_publisher(String, 'robot/state', qos_reliable)
        self.twist_publisher_ = self.create_publisher(Twist, 'cmd_vel', qos_reliable)

        # Control Timer
        control_loop_period = 0.005 # 200 Hz
        self.control_timer = self.create_timer(control_loop_period, self.control_loop_callback)

        # self.get_logger().info('State Machine Node started (Timer-based P-Controller, Keyboard Pause Enabled)')
        self.publish_current_state()
        self.send_cmd_vel(0.0, 0.0)

    # --- NEU: Callback für Tastaturbefehle ---
    def keyboard_command_callback(self, msg: String):
        """Verarbeitet Befehle vom Keyboard Handler Node."""
        if msg.data == 'toggle_pause':
            self.toggle_manual_pause()
    # --- Ende Keyboard Command Callback ---

    # --- NEU: Funktion zum Umschalten der Pause ---
    def toggle_manual_pause(self):
        """Schaltet den manuellen Pause-Zustand um."""
        if not self.manual_pause_active:
            # --- PAUSE AKTIVIEREN ---
            self.manual_pause_active = True
            # self.get_logger().info("Keyboard command received - Pausing robot and state logic!")
            self.send_cmd_vel(0.0, 0.0) # Roboter sofort stoppen
            # Zustand setzen (wird von Callbacks ignoriert)
            # Wir brauchen change_state nicht direkt aufrufen, nur publizieren
            pause_state_msg = String()
            pause_state_msg.data = MANUAL_PAUSE_STATE
            self.state_publisher_.publish(pause_state_msg) # Zustand mitteilen
        else:
            # --- PAUSE DEAKTIVIEREN ---
            self.manual_pause_active = False
            # self.get_logger().info("Keyboard command received - Resuming operation -> FOLLOW_LANE")
            # Wechsle Zwangsläufig zu FOLLOW_LANE
            self.change_state('FOLLOW_LANE')
            # Der control_timer übernimmt ab jetzt wieder
    # --- Ende Pause Umschaltfunktion ---


    def control_loop_callback(self):
        """Wird vom Timer aufgerufen, berechnet und sendet Steuerung."""
        # --- KEINE AKTION WENN PAUSIERT ---
        if self.manual_pause_active:
            return # Tue nichts, wenn manuell pausiert

        current_state = self.state
        driving_speed = 0.0
        angular_z = 0.0

        if current_state == 'FOLLOW_LANE':
            driving_speed = self.get_parameter('drivingspeed').get_parameter_value().double_value
            angular_z = self.center_offset
            max_angular_z = 1.0
            angular_z = np.clip(angular_z, -max_angular_z, max_angular_z)
            self.send_cmd_vel(driving_speed, angular_z)
        # Stopp-Befehle für andere Zustände werden von change_state gehandhabt

    def change_state(self, new_state):
        """Ändert den Zustand, publiziert ihn und sendet initialen Befehl für neuen Zustand."""
        # --- KEINE Automatische Zustandsänderung WENN PAUSIERT ---
        # Nur der Keyboard-Callback darf die Pause aufheben
        if self.manual_pause_active:
             return

        if self.state != new_state:
            old_state = self.state
            self.state = new_state
            # self.get_logger().info(f"State change from {old_state} to {self.state}")
            self.publish_current_state()

            if self.state == 'STOPPED' or self.state == 'WAYMO_STARTED':
                 self.send_cmd_vel(0.0, 0.0)
            # Wenn der neue Zustand FOLLOW_LANE ist, startet der Timer automatisch die Bewegung

    def obstacle_detection_callback(self, msg: Bool):
        """Verarbeitet Hinderniserkennung und löst Zustandswechsel aus."""
        # --- IGNORIEREN WENN PAUSIERT ---
        if self.manual_pause_active: return

        blocked = msg.data
        current_state = self.state

        if blocked:
            if current_state == 'FOLLOW_LANE':
                self.change_state('STOPPED')
            elif current_state == 'STOPPED':
                 self.change_state('PASSING_OBSTACLE')
        else: # Not blocked
             if current_state == 'STOPPED':
                 self.change_state('FOLLOW_LANE')
             elif current_state == 'WAYMO_STARTED' and current_state != 'FOLLOW_LANE':
                  self.change_state('FOLLOW_LANE')

    def lane_detection_callback(self, msg: Float64):
        """Aktualisiert NUR den Offset-Wert."""
        # Offset immer speichern, auch wenn pausiert
        self.center_offset = msg.data
        # Die Steuerung wird vom Timer übernommen (wenn nicht pausiert)

    def obstacle_passed_callback(self, msg: Bool):
        """Verarbeitet Signal, dass Hindernis passiert wurde."""
        # --- IGNORIEREN WENN PAUSIERT ---
        if self.manual_pause_active: return

        passed = msg.data
        if passed and self.state == 'PASSING_OBSTACLE':
             self.change_state('FOLLOW_LANE')

    def publish_current_state(self):
        """Publisht den aktuellen Zustand."""
        # Sende den internen Zustand, es sei denn wir sind pausiert
        state_to_publish = self.state if not self.manual_pause_active else MANUAL_PAUSE_STATE
        state_msg = String(); state_msg.data = state_to_publish
        try:
             if rclpy.ok() and self.context.ok(): self.state_publisher_.publish(state_msg)
        except Exception: pass

    def send_cmd_vel(self, linear_x, angular_z):
       """Sendet Bewegungsbefehle."""
       # Sende nur, wenn nicht pausiert! (Doppelte Sicherheit)
       if self.manual_pause_active:
           # Sende einmalig Stop beim Pausieren, aber nicht kontinuierlich
           # Dieser Check verhindert das Senden, während pausiert
           return

       twist = Twist(); twist.linear.x = linear_x; twist.angular.z = float(angular_z)
       try:
           if rclpy.ok() and self.context.ok(): self.twist_publisher_.publish(twist)
       except Exception: pass

    def destroy_node(self):
        """Aufräumarbeiten."""
        # self.get_logger().info("Shutting down State Machine Node.")
        # Keyboard Listener wird im anderen Node beendet
        try:
             if rclpy.ok() and self.context.ok():
                  # Stelle sicher, dass der letzte Befehl Stop ist
                  self.manual_pause_active = False # Deaktiviere Pause für Stopp-Befehl
                  self.send_cmd_vel(0.0, 0.0); time.sleep(0.1)
        except Exception as e: self.get_logger().warn(f"Could not send stop cmd: {e}")
        super().destroy_node()


# --- main bleibt unverändert ---
def main(args=None):
    rclpy.init(args=args); node = None
    try:
        node = StateMachine()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node: node.get_logger().info("KeyboardInterrupt received, shutting down.")
    except Exception as e:
        if node: node.get_logger().error(f"Unhandled exception in StateMachine: {e}", exc_info=True)
        else: print(f"Unhandled exception before StateMachine init: {e}", file=sys.stderr); traceback.print_exc()
    finally:
        if node is not None: node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()
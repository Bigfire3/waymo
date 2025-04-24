import rclpy
import rclpy.node
import numpy as np
import time
import sys # Für sys.stderr in Fehlerbehandlung

from std_msgs.msg import String, Float64, Bool
from geometry_msgs.msg import Twist
# QoS Importe
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class StateMachine(rclpy.node.Node):

    def __init__(self):
        super().__init__('state_manager_node') # Korrekter Node-Name
        self.declare_parameter('drivingspeed', 0.15)
        self.center_offset = 0.0
        self.state = 'WAYMO_STARTED'

        # --- QoS Profile (Korrekte gemischte Profile) ---
        qos_sensor = QoSProfile( # Für Sensordaten
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        qos_reliable = QoSProfile( # Für Befehle/Status
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        # --- Ende QoS ---

        # Subscribers
        self.obstacle_subscription = self.create_subscription(
            Bool,
            'obstacle/blocked',
            self.obstacle_detection_callback,
            qos_sensor # Sensor = Best Effort
        )
        self.offset_subscription = self.create_subscription(
            Float64,
            'lane/center_offset',
            self.lane_detection_callback,
            qos_sensor # Sensor = Best Effort
        )
        self.passed_subscription = self.create_subscription(
            Bool,
            '/obstacle/passed',
            self.obstacle_passed_callback,
            qos_reliable # Status = Reliable
        )

        # Publisher
        self.state_publisher_ = self.create_publisher(
            String,
            'robot/state',
            qos_reliable # Status = Reliable
        )
        self.twist_publisher_ = self.create_publisher(
            Twist,
            'cmd_vel',
            qos_reliable # Befehl = Reliable
        )

        # Timer
        publish_state_period = 1.0
        self.state_publish_timer = self.create_timer(publish_state_period, self.publish_current_state)
        self.get_logger().info('State Machine Node started')

    def logic_function(self):
        """Bestimmt Aktionen basierend auf dem Zustand."""
        new_state = self.state
        driving_speed = 0.0
        angular_z = 0.0

        if new_state == 'WAYMO_STARTED':
            driving_speed = 0.0; angular_z = 0.0
        elif new_state == 'STOPPED':
            driving_speed = 0.0; angular_z = 0.0
        elif new_state == 'FOLLOW_LANE':
            driving_speed = self.get_parameter('drivingspeed').get_parameter_value().double_value
            angular_z = self.center_offset
            max_angular_z = 1.0
            angular_z = np.clip(angular_z, -max_angular_z, max_angular_z)
        elif new_state == 'PASSING_OBSTACLE':
            pass # Übergibt Kontrolle

        if new_state != 'PASSING_OBSTACLE':
            self.send_cmd_vel(driving_speed, angular_z)
        # Zustand immer publizieren, nachdem Logik durchlaufen wurde
        self.publish_current_state()

    def obstacle_detection_callback(self, msg: Bool):
        """Verarbeitet Hinderniserkennung."""
        blocked = msg.data
        current_state = self.state
        state_changed = False

        if blocked:
            if current_state == 'FOLLOW_LANE':
                self.get_logger().info("Obstacle -> STOPPED")
                self.state = 'STOPPED'; state_changed = True
            elif current_state == 'STOPPED':
                 self.get_logger().info("Obstacle still present -> PASSING_OBSTACLE")
                 self.state = 'PASSING_OBSTACLE'; state_changed = True
        else: # Not blocked
             if current_state == 'STOPPED':
                 self.get_logger().info("Obstacle gone while stopped -> FOLLOW_LANE")
                 self.state = 'FOLLOW_LANE'; state_changed = True
             elif current_state == 'WAYMO_STARTED':
                 # Erster Start ohne Hindernis
                 self.state = 'FOLLOW_LANE'; state_changed = True
                 # Keine Log-Ausgabe hier, wird in publish_current_state behandelt

        if state_changed:
             self.logic_function() # Nur ausführen bei Zustandsänderung

    def lane_detection_callback(self, msg: Float64):
        """Verarbeitet Spurversatz."""
        self.center_offset = msg.data
        if self.state == 'FOLLOW_LANE':
            self.logic_function()

    def obstacle_passed_callback(self, msg: Bool):
        """Verarbeitet Signal, dass Hindernis passiert wurde."""
        passed = msg.data
        # Nur reagieren, wenn das Signal positiv ist UND wir im PASSING_OBSTACLE Zustand sind
        if passed and self.state == 'PASSING_OBSTACLE':
            self.get_logger().info("Obstacle passed signal received -> FOLLOW_LANE")
            self.state = 'FOLLOW_LANE'
            self.logic_function()

    def publish_current_state(self):
        """Publisht den aktuellen Zustand."""
        state_msg = String()
        state_msg.data = self.state
        self.state_publisher_.publish(state_msg)

    def send_cmd_vel(self, linear_x, angular_z):
       """Sendet Bewegungsbefehle."""
       twist = Twist()
       twist.linear.x = linear_x
       twist.angular.z = angular_z
       self.twist_publisher_.publish(twist)

    def destroy_node(self):
        """Aufräumarbeiten."""
        self.get_logger().info("Shutting down State Machine Node.")
        try:
             # Sicherstellen, dass der Roboter stoppt
             if rclpy.ok() and self.context.ok():
                  self.send_cmd_vel(0.0, 0.0)
                  # Kurze Pause geben, damit Nachricht evtl. noch gesendet wird
                  time.sleep(0.1)
        except Exception as e:
            # Fehler nur loggen, falls Kontext bereits ungültig
            self.get_logger().warn(f"Could not send stop command during shutdown: {e}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = StateMachine()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node: node.get_logger().info("KeyboardInterrupt received, shutting down.")
    except Exception as e:
        # Logge den Fehler, falls Node existiert
        if node: node.get_logger().error(f"Unhandled exception in StateMachine: {e}", exc_info=True)
        else: # Falls Fehler vor Node-Init passiert
            print(f"Unhandled exception before StateMachine init: {e}", file=sys.stderr)
            import traceback
            traceback.print_exc()
    finally:
        # Sauberes Herunterfahren
        if node is not None and isinstance(node, rclpy.node.Node):
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
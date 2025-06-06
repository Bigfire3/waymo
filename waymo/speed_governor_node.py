# waymo/speed_governor_node.py

import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange, SetParametersResult
from rclpy.parameter import ParameterType
import numpy as np # Für np.clip

class SpeedGovernorNode(Node):
    def __init__(self):
        super().__init__('speed_governor_node')

        # --- Parameter Descriptors ---
        def float_desc(description, min_val=0.0, max_val=1.0, step=0.01):
            return ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description=description,
                floating_point_range=[FloatingPointRange(from_value=min_val, to_value=max_val, step=step)]
            )

        # --- Parameter Deklarationen ---
        self.declare_parameter('max_straight_speed', 0.2,
                               float_desc("Maximum speed on a straight line (m/s)", 0.0, 0.5, 0.01))
        # Faktor zur Reduzierung der Geschwindigkeit basierend auf dem Offset.
        # Ein größerer Faktor führt zu stärkerer Reduzierung bei gleichem Offset.
        # Dieser Wert muss wahrscheinlich experimentell angepasst werden.
        self.declare_parameter('speed_reduction_factor', 0.5,
                               float_desc("Factor to reduce speed based on lane offset", 0.0, 2.0, 0.001))
        self.declare_parameter('min_speed', 0.05,
                               float_desc("Minimum allowed speed during movement (m/s)", 0.0, 0.1, 0.005))

        # Initialen Parameterwert loggen
        # self.log_parameters()

        # QoS Profile
        qos_sensor_data = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1
        )
        qos_control_data = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE, # Wichtig für Steuerbefehle
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # --- Subscriber ---
        self.center_offset_subscription = self.create_subscription(
            Float64,
            '/lane/center_offset',
            self.center_offset_callback,
            qos_sensor_data
        )

        # --- Publisher ---
        self.recommended_speed_publisher = self.create_publisher(
            Float64,
            '/robot/recommended_speed',
            qos_control_data
        )

        # Timer, um regelmäßig die Geschwindigkeit zu publizieren, auch wenn kein neuer Offset kommt
        # (kann nützlich sein, wenn der lane_detection_node mal ausfällt, dann wird die letzte berechnete Geschwindigkeit gehalten oder eine Standardgeschwindigkeit)
        # Für den Anfang reicht es, bei jedem neuen Offset-Wert zu publizieren.
        # Ein Timer könnte hinzugefügt werden, falls ein "Heartbeat" für die Geschwindigkeit gewünscht ist.

        self.current_center_offset = 0.0
        # self.get_logger().info('Speed Governor Node gestartet.')
        # self.get_logger().info('Parameter-Hinweis: max_straight_speed, speed_reduction_factor, min_speed können via rqt_reconfigure angepasst werden.')


    # def log_parameters(self):
    #     self.get_logger().info(f"Initial parameters: "
    #                            f"max_straight_speed: {self.get_parameter('max_straight_speed').value}, "
    #                            f"speed_reduction_factor: {self.get_parameter('speed_reduction_factor').value}, "
    #                            f"min_speed: {self.get_parameter('min_speed').value}")

    def center_offset_callback(self, msg: Float64):
        self.current_center_offset = msg.data
        self.calculate_and_publish_speed()

    def calculate_and_publish_speed(self):
        max_speed = self.get_parameter('max_straight_speed').value
        reduction_factor = self.get_parameter('speed_reduction_factor').value
        min_allowed_speed = self.get_parameter('min_speed').value

        # Berechne die Reduktion basierend auf dem Betrag des Offsets
        # Je größer der Offset, desto größer die Reduktion
        speed_reduction = reduction_factor * abs(self.current_center_offset)

        # Berechne die Zielgeschwindigkeit
        target_speed = max_speed - speed_reduction

        # Stelle sicher, dass die Geschwindigkeit nicht unter die Mindestgeschwindigkeit fällt
        # und nicht über der Maximalgeschwindigkeit liegt (obwohl das durch die Berechnung schon gegeben sein sollte)
        recommended_speed_value = np.clip(target_speed, min_allowed_speed, max_speed)

        # Erstelle und publiziere die Nachricht
        speed_msg = Float64()
        speed_msg.data = recommended_speed_value
        self.recommended_speed_publisher.publish(speed_msg)

        # Optional: Log der berechneten Geschwindigkeit für Debugging-Zwecke
        # self.get_logger().debug(f"Offset: {self.current_center_offset:.3f}, "
        #                        f"Reduction: {speed_reduction:.3f}, "
        #                        f"Target: {target_speed:.3f}, "
        #                        f"Published: {recommended_speed_value:.3f}")

    def destroy_node(self):
        # self.get_logger().info("Speed Governor Node wird heruntergefahren.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SpeedGovernorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if node:
            node.get_logger().fatal(f"FATAL ERROR in SpeedGovernorNode: {e}\n{rclpy.traceback.format_exc()}")
        else:
            print(f"FATAL ERROR before SpeedGovernorNode init: {e}\n{rclpy.traceback.format_exc()}", file=sys.stderr)
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
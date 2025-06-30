
import rclpy
import rclpy.node
from sensor_msgs.msg import LaserScan
import math

class LaserScanDebuggerNode(rclpy.node.Node):

    def __init__(self):
        super().__init__("laser_scan_debugger_node")
        self.get_logger().info("LaserScan Debugger Node gestartet.")
        
        qos_policy = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.subscription = self.create_subscription(
            LaserScan, "scan", self.scanner_callback, qos_profile=qos_policy
        )

    def scanner_callback(self, msg):
        closest_distance = float("inf")
        closest_angle = 0.0

        for i, range_value in enumerate(msg.ranges):
            if not math.isinf(range_value) and not math.isnan(range_value) and range_value > 0.0:
                if range_value < closest_distance:
                    closest_distance = range_value
                    angle = msg.angle_min + i * msg.angle_increment
                    closest_angle = math.degrees(angle)

        if not math.isinf(closest_distance):
            self.get_logger().info(f"Nähestes Objekt: Distanz={closest_distance:.2f}m, Winkel={closest_angle:.2f}°")

def main(args=None):
    rclpy.init(args=args)
    node = LaserScanDebuggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()

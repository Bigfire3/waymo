import rclpy
import rclpy.node
from std_msgs.msg import Bool, String
from sensor_msgs.msg import LaserScan
import math


class ObstacleDetectionNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("obstacle_detection_node")

        self.declare_parameter("distance_to_stop", 0.25)

        # The front of the robot is at the +/- 180 degree seam.
        # We scan a 20-degree cone to the left [-180, -160] and to the right [160, 180].
        self.scan_range_left = (-180.0, -165.0)
        self.scan_range_right = (165.0, 180.0)

        self.closest_distance = float("inf")
        self.blocked = None
        self.current_state = "STATE_STOPPED_AT_TRAFFIC_LIGHT"  # Startwert

        qos_policy = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.state_subscriber = self.create_subscription(
            String, "/robot/state", self.state_callback, qos_policy
        )
        self.subscription = self.create_subscription(
            LaserScan, "scan", self.scanner_callback, qos_profile=qos_policy
        )

        self.blocked_publisher_ = self.create_publisher(
            Bool, "obstacle/blocked", qos_policy
        )

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def state_callback(self, msg: String):
        self.current_state = msg.data

    def scanner_callback(self, msg: LaserScan):
        if self.current_state != "FOLLOW_LANE":
            self.closest_distance = float("inf")
            return
        # Reset closest distance for each new scan
        self.closest_distance = float("inf")

        for i, dist in enumerate(msg.ranges):
            # Skip invalid range values
            if math.isinf(dist) or math.isnan(dist) or dist <= 0.0:
                continue

            angle_deg = math.degrees(msg.angle_min + i * msg.angle_increment)

            # Check if the angle is within the left or right frontal cone
            in_left_range = (
                self.scan_range_left[0] <= angle_deg <= self.scan_range_left[1]
            )
            in_right_range = (
                self.scan_range_right[0] <= angle_deg <= self.scan_range_right[1]
            )

            if in_left_range or in_right_range:
                if dist < self.closest_distance:
                    self.closest_distance = dist

    def timer_callback(self):
        if self.current_state != "FOLLOW_LANE":
            # Wenn der Node nicht aktiv ist, stelle sicher, dass der letzte gesendete Status "nicht blockiert" ist.
            if self.blocked is not False:
                self.blocked = False
                msg = Bool()
                msg.data = self.blocked
                self.blocked_publisher_.publish(msg)
            return

        distance_stop = self.get_parameter("distance_to_stop").value

        # Determine if blocked based on the closest distance found in the scan
        # If no obstacle was found, closest_distance remains 'inf'
        is_currently_blocked = self.closest_distance <= distance_stop

        # Publish only if the state changes
        if self.blocked != is_currently_blocked:
            self.blocked = is_currently_blocked
            msg = Bool()
            msg.data = self.blocked
            self.blocked_publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

import rclpy
import rclpy.node
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
import math


class Obstacle_Detection(rclpy.node.Node):

    def __init__(self):
        super().__init__('drive_with_scanner')

        self.declare_parameter('distance_to_stop', 0.25)
        # In radians, angle range -180° to +180° (LIDAR format)
        self.declare_parameter('angle_range_min', -(0.2) + math.pi)
        self.declare_parameter('angle_range_max', (0.2) + math.pi)

        # Variable for the last sensor reading
        self.closest_distance = float('inf')
        self.closest_angle = 0.0
        self.blocked = None

        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)

        # Subscribe to laser scan data
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scanner_callback,
            qos_profile=qos_policy)
        self.subscription

        # Publisher for current state
        self.blocked_publisher_ = self.create_publisher(Bool, 'obstacle/blocked', qos_policy)

        # Create a timer to periodically update / publish the current state
        timer_period = 0.2  # seconds
        self.my_timer = self.create_timer(timer_period, self.timer_callback)

    def scanner_callback(self, msg):
        # Find the closest object within the angle range
        self.closest_distance = float('inf')
        self.closest_angle = 0.0
        # Get the parameters for angle range
        angle_min = msg.angle_min  # -π (or -180°)
        angle_max = msg.angle_max  # +π (or +180°)
        angle_increment = msg.angle_increment

        angle_range_min = self.get_parameter(
            'angle_range_min').get_parameter_value().double_value
        angle_range_max = self.get_parameter(
            'angle_range_max').get_parameter_value().double_value

        num_ranges = len(msg.ranges)
        min_index = max(
            0, int((angle_range_min - angle_min) / angle_increment))
        max_index = min(
            num_ranges - 1, int((angle_range_max - angle_min) / angle_increment))

        for i in range(min_index, max_index + 1):
            range_value = msg.ranges[i]
            if math.isinf(range_value) or range_value == 0.0 or math.isnan(range_value):
                continue
            # Calculate the actual angle of this reading
            angle = angle_min + i * angle_increment

            # Correct the angle by adding π to map it properly (if needed)
            if angle < 0:  # If angle is negative (left side)
                angle += math.pi  # Add 180 degrees to map it to the proper front-facing frame
            else:
                angle -= math.pi  # Subtract 180 degrees for right-side angles to align

            # Check if this is the closest obstacle
            if range_value < self.closest_distance:
                self.closest_distance = range_value
                self.closest_angle = angle

    # Logic for determining the current state
    def timer_callback(self):
        blocked = self.blocked
        distance_stop = self.get_parameter('distance_to_stop').get_parameter_value().double_value
        
        # If the closest object is too close, stop
        if self.closest_distance <= distance_stop:
            blocked = True
        else:
            blocked = False

        if (self.blocked == blocked):
            return

        msg = Bool()
        msg.data = blocked
        self.blocked_publisher_.publish(msg)

    def destroy_node(self):
        # self.get_logger().info("Destroying Obstacle Detection Node...")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Obstacle_Detection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

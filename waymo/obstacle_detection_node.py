

import rclpy
import rclpy.node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math


class Obstacle_Detection(rclpy.node.Node):

    def __init__(self):
        super().__init__('drive_with_scanner')

        # Declare parameters that can be changed at runtime
        self.declare_parameter('distance_to_stop', 0.3)
        # In radians, e.g., -30 degrees
        self.declare_parameter('angle_range_min', -1.0)
        # In radians, e.g., 30 degrees
        self.declare_parameter('angle_range_max', 1.0)

        # Variable for the last sensor reading
        self.closest_distance = float('inf')
        self.closest_angle = 0.0

        # QoS for subscriber
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
        self.publisher_ = self.create_publisher(String, 'state_machine', 1)

        self.get_logger().info('Obstacle Detection Node started (dynamic parameters available)')

        # Create a timer to periodically update / publish the current state
        timer_period = 0.2  # seconds
        self.my_timer = self.create_timer(timer_period, self.timer_callback)

    # Handling received laser scan data
    def scanner_callback(self, msg):
        # Find the closest object within the angle range
        self.closest_distance = float('inf')
        self.closest_angle = 0.0
        # Get the parameters for angle range
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_increment = msg.angle_increment

        # Get the parameter values for the angle range
        angle_range_min = self.get_parameter(
            'angle_range_min').get_parameter_value().double_value
        angle_range_max = self.get_parameter(
            'angle_range_max').get_parameter_value().double_value

        # Calculate the index range corresponding to the angle range
        min_index = int((angle_range_min - angle_min) / angle_increment)
        max_index = int((angle_range_max - angle_min) / angle_increment)

        # Iterate through the laser scan ranges within the given angle range
        for i in range(min_index, max_index + 1):
            range_value = msg.ranges[i]
            # Ignore invalid readings (NaN or infinite)
            if range_value == float('inf') or range_value == 0.0:
                continue

            # Update closest object
            if range_value < self.closest_distance:
                self.closest_distance = range_value
                self.closest_angle = angle_min + i * angle_increment

        # print(
            # f'Closest Distance: {self.closest_distance}, Angle: {self.closest_angle}')

    # Logic for determining the current state
    def timer_callback(self):
        # Caching parameter for clarity
        distance_stop = self.get_parameter(
            'distance_to_stop').get_parameter_value().double_value
        # If the closest object is too close, stop
        if self.closest_distance <= distance_stop:
            state = "STOPPED"
        else:
            state = "FOLLOW_LANE"
        # Create the String message to send to the robot
        msg = String()
        msg.data = state
        # Publish the message
        self.publisher_.publish(msg)

    def destroy_node(self):
        self.get_logger().info("Destroying Obstacle Detection Node...")
        super().destroy_node


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

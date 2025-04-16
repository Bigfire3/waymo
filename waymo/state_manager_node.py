import rclpy
import rclpy.node
import numpy as np
import time

from std_msgs.msg import String, Float64, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError


class StateMachine(rclpy.node.Node):

    def __init__(self):
        super().__init__('StateMachine')
        self.declare_parameter('drivingspeed', 0.15)
        self.center_offset = 0.0
        self.driving_speed = 0.0
        self.angular_z = 0.0
        self.state = 'WAYMO_STARTED'
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)
        self.obstacle_subscription = self.create_subscription(
            Bool, 'obstacle/blocked', self.obstacle_detection_callback, qos_profile=qos_policy)
        self.offset_subscription = self.create_subscription(
            Float64, 'lane/center_offset', self.lane_detection_callback, qos_profile=qos_policy)

        self.state_publisher_ = self.create_publisher(String, 'robot/state', 1)
        self.twist_publisher_ = self.create_publisher(Twist, 'cmd_vel', 1)
        # self.get_logger().info('State Machine Node started')

    def logic_function(self):
        match self.state:
            case 'WAYMO_STARTED':
                self.driving_speed = 0.0
                self.angular_z = 0.0
                self.send_cmd_vel(self.driving_speed, self.angular_z)

            case 'STOPPED':
                self.driving_speed = 0.0
                self.angular_z = 0.0
                self.send_cmd_vel(self.driving_speed, self.angular_z)

            case 'FOLLOW_LANE':
                self.driving_speed = 0.15
                self.angular_z = self.center_offset
                self.send_cmd_vel(self.driving_speed, self.angular_z)

        self.state_publisher_.publish(String(data=self.state))

    def obstacle_detection_callback(self, msg):
        blocked = msg.data
        if (blocked):
            self.state = 'STOPPED'
        else:
            self.state = 'FOLLOW_LANE'
        self.logic_function()

    def lane_detection_callback(self, msg):
        self.center_offset = msg.data
        self.logic_function()

    def destroy_node(self):
        super().destroy_node()

    def send_cmd_vel(self, linear_x, angular_z):
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        # self.twist_publisher_.publish(twist)
        # self.get_logger().info(f'Sent cmd_vel: linear_x={linear_x}, angular_z={angular_z}')


def main(args=None):
    rclpy.init(args=args)
    node = StateMachine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

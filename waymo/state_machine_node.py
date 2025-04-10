import rclpy
import rclpy.node
import cv2
import numpy as np
import time

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError


class StateMachine(rclpy.node.Node):

    def __init__(self):
        super().__init__('StateMachine')
        self.declare_parameter('drivingspeed', 1.0)
        self.driving_speed = 0.0
        self.state = 'DEFAULT'
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)
        self.subscription = self.create_subscription(
            String,
            'obstacle',
            self.ObstacleDetectionCallback,
            qos_profile=qos_policy)
        self.subscription  # prevent unused variable warning

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 1)
        self.get_logger().info('State Machine Node started')

    def LogicFunction(self):
        driving_speed = self.driving_speed
        state = self.state
        if (state == 'STOPPED' or state == 'DEFAULT'):
            self.driving_speed = 0.0
            speed = self.driving_speed
        elif (state == 'FOLLOW_LANE'):
            self.driving_speed = 1.0
            speed = self.driving_speed
        else:
            self.get_logger().error('undefined behavior')
        self.get_logger().info(f'current speed: {speed}')

    def ObstacleDetectionCallback(self, msg):
        state = msg.data
        self.state = state
        self.LogicFunction()
        self.get_logger().info(
            f'state changed by obstacle_detection_node; new state: {state}')

    def destroy_node(self):
        self.get_logger().info('Destroying State Machine Node...')
        super().destroy_node()


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

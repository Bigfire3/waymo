import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time


class TrafficLightDetection(Node):
    def __init__(self):
        super().__init__('traffic_light_detection_node')
        self.state = True
        self.bridge = CvBridge()
        self.get_logger().info('Traffic Light Detection Node started!')

        qos_policy = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.publisher_ = self.create_publisher(
            Bool, 'traffic_light', qos_policy)

        self.img_subscriber = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            qos_profile=qos_policy
        )

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Could not convert image: {e}")
            return

        # Optional: downscale to reduce processing cost
        frame = cv2.resize(frame, (640, 480))

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        red_lower1 = np.array([0, 120, 70])
        red_upper1 = np.array([10, 255, 255])
        red_lower2 = np.array([170, 120, 70])
        red_upper2 = np.array([180, 255, 255])

        yellow_lower = np.array([15, 100, 100])
        yellow_upper = np.array([35, 255, 255])

        green_lower = np.array([36, 50, 70])
        green_upper = np.array([89, 255, 255])

        red_mask = cv2.bitwise_or(
            cv2.inRange(hsv, red_lower1, red_upper1),
            cv2.inRange(hsv, red_lower2, red_upper2)
        )
        yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)
        green_mask = cv2.inRange(hsv, green_lower, green_upper)

        red_count = cv2.countNonZero(red_mask)
        yellow_count = cv2.countNonZero(yellow_mask)
        green_count = cv2.countNonZero(green_mask)

        detection_threshold = 100
        curr_state = self.state  # fallback to previous

        if red_count > detection_threshold:
            curr_state = False
        elif yellow_count > detection_threshold:
            curr_state = True
        elif green_count > detection_threshold:
            curr_state = True

        if self.state != curr_state:
            self.get_logger().info(
                f'Traffic light changed: {curr_state}')
            msg = Bool()
            msg.data = curr_state
            self.publisher_.publish(msg)
            self.state = curr_state


def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightDetection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

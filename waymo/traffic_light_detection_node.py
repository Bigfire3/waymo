import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class TrafficLightDetection(Node):
    def __init__(self):
        super().__init__('traffic_light_detection_node')
        self.subscriber = self.create_subscription(
            Image, '/image_raw', self.image_callback, 1)
        self.bridge = CvBridge()
        self.get_logger().info('Traffic Light Detection Node started')

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Could not convert image: {e}")
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # HSV color ranges
        red_lower1 = np.array([0, 120, 70])
        red_upper1 = np.array([10, 255, 255])
        red_lower2 = np.array([170, 120, 70])
        red_upper2 = np.array([180, 255, 255])

        yellow_lower = np.array([15, 100, 100])
        yellow_upper = np.array([35, 255, 255])

        green_lower = np.array([36, 50, 70])
        green_upper = np.array([89, 255, 255])

        # Create color masks
        red_mask1 = cv2.inRange(hsv, red_lower1, red_upper1)
        red_mask2 = cv2.inRange(hsv, red_lower2, red_upper2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)

        yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)
        green_mask = cv2.inRange(hsv, green_lower, green_upper)

        # Count the number of detected pixels
        red_count = cv2.countNonZero(red_mask)
        yellow_count = cv2.countNonZero(yellow_mask)
        green_count = cv2.countNonZero(green_mask)

        # Detection threshold (higher = less sensitive)
        detection_threshold = 1000

        status = "None"
        if red_count > detection_threshold:
            status = "RED"
        elif yellow_count > detection_threshold:
            status = "YELLOW"
        elif green_count > detection_threshold:
            status = "GREEN"

        self.get_logger().info(f"Traffic light detected: {status}")

        # Display the image with the result
        try:
            display = frame.copy()
            cv2.imshow("Traffic Light Detection", display)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().warn(f"Cannot display image: {e}")


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

import rclpy
from std_msgs.msg import Bool
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class TrafficLightDetection(Node):
    def __init__(self):
        super().__init__('traffic_light_detection_node')
        self.state = True
        self.bridge = CvBridge()
        self.latest_msg = None  # store latest image message
        self.get_logger().info('Traffic Light Detection Node started!!!!')

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

        # Timer for throttled processing (e.g., 10 Hz)
        self.timer = self.create_timer(1, self.timer_callback)

    def image_callback(self, msg):
        # Just store the latest message
        self.latest_msg = msg

    def timer_callback(self):
        if self.latest_msg is None:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(
                self.latest_msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Could not convert image: {e}")
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        red_lower1 = np.array([0, 120, 70])
        red_upper1 = np.array([10, 255, 255])
        red_lower2 = np.array([170, 120, 70])
        red_upper2 = np.array([180, 255, 255])

        yellow_lower = np.array([15, 100, 100])
        yellow_upper = np.array([35, 255, 255])

        green_lower = np.array([36, 50, 70])
        green_upper = np.array([89, 255, 255])

        red_mask1 = cv2.inRange(hsv, red_lower1, red_upper1)
        red_mask2 = cv2.inRange(hsv, red_lower2, red_upper2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)

        yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)
        green_mask = cv2.inRange(hsv, green_lower, green_upper)

        red_count = cv2.countNonZero(red_mask)
        yellow_count = cv2.countNonZero(yellow_mask)
        green_count = cv2.countNonZero(green_mask)

        detection_threshold = 50
        status = "None"

        if red_count > detection_threshold:
            status = "STOP"
            curr_state = False
        elif yellow_count > detection_threshold:
            status = "GO"
            curr_state = True
        elif green_count > detection_threshold:
            status = "GO"
            curr_state = True

        if self.state != curr_state:
            self.get_logger().info(f'traffic light: {curr_state}')
            msg = Bool()
            msg.data = curr_state
            self.publisher_.publish(msg)
            self.state = curr_state

    def destroy_node(self):
        super().destroy_node()


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

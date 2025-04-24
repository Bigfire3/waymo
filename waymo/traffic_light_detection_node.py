import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np


class TrafficLightDetector(Node):

    def __init__(self):
        super().__init__('traffic_light_detector')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.image_callback,
            10)
        self.publisher_ = self.create_publisher(Bool, 'traffic_light', 10)
        self.previous_state = None  # Track previous Bool (True/False)
        self.spotted_once = False

    def image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        is_red_light, debug_frame = self.detect_red_light(frame)

        current_bool = not is_red_light  # Send True if NOT red
        if current_bool != self.previous_state:
            if (self.spotted_once is False and current_bool is False):
                self.spotted_once = True
            self.publisher_.publish(Bool(data=current_bool))
            self.get_logger().info(
                f"Traffic light is {'RED' if is_red_light else 'NOT RED'}")
            self.previous_state = current_bool
            # if (self.spotted_once is True and current_bool is True):
            # self.destroy_node()
        cv2.imshow('Traffic Light Debug View', debug_frame)
        cv2.waitKey(1)

    def detect_red_light(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Tighter HSV ranges for red (saturated & bright)
        lower_red1 = (0, 150, 150)
        upper_red1 = (10, 255, 255)
        lower_red2 = (160, 150, 150)
        upper_red2 = (179, 255, 255)

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = mask1 | mask2

        # Extract brightness info from V channel
        v_channel = hsv[:, :, 2]
        _, bright_mask = cv2.threshold(v_channel, 220, 255, cv2.THRESH_BINARY)

        # Combine red + brightness mask
        bright_red_mask = cv2.bitwise_and(red_mask, bright_mask)

        h, w, _ = frame.shape
        roi_x1, roi_y1 = int(w * 0.05), int(h * 0.3)
        roi_x2, roi_y2 = int(w * 0.95), int(h)

        debug_frame = frame.copy()
        cv2.rectangle(debug_frame, (roi_x1, roi_y1),
                      (roi_x2, roi_y2), (255, 255, 255), 2)

        roi_mask = bright_red_mask[roi_y1:roi_y2, roi_x1:roi_x2]
        contours, _ = cv2.findContours(
            roi_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 100:
                (x, y), radius = cv2.minEnclosingCircle(cnt)
                circle_area = np.pi * radius * radius
                shape_ratio = area / circle_area if circle_area > 0 else 0

                if 0.6 < shape_ratio < 1.4:
                    cnt_shifted = cnt + [roi_x1, roi_y1]
                    cv2.drawContours(
                        debug_frame, [cnt_shifted], -1, (0, 0, 255), 2)
                    cv2.putText(debug_frame, 'RED LIGHT',
                                (roi_x1, roi_y1 - 10), cv2.FONT_HERSHEY_SIMPLEX,
                                0.7, (0, 0, 255), 2)
                    return True, debug_frame

        return False, debug_frame


def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()

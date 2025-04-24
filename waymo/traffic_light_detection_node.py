import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
import time


class TrafficLightDetector(Node):
    def __init__(self):
        super().__init__('traffic_light_detector')

        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.image_callback,
            10)

        self.publisher_ = self.create_publisher(Bool, 'traffic_light', 10)

        self.previous_state = None
        self.spotted_once = False

        self.frame_buffer = []  # Store recent filtered masks
        self.last_update_time = time.time()

        # === CONFIGURABLE ===
        self.COMBINE_LAST_N = 3            # Number of frames to combine
        self.INTER_FRAME_DELAY = 0.005      # Seconds to wait between processing frames

    def image_callback(self, msg):
        now = time.time()
        if now - self.last_update_time < self.INTER_FRAME_DELAY:
            return  # Skip frame if too soon

        self.last_update_time = now

        # Decode the compressed image
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Crop to ignore bottom 1/4 of the image
        h, w, _ = frame.shape
        frame_cropped = frame[0:int(h * 0.8), :]

        # Detect red light (returns the filtered B&W mask)
        mask = self.detect_red_light(frame_cropped)

        # Ensure frame_buffer is a list
        if not isinstance(self.frame_buffer, list):
            self.frame_buffer = []

        self.frame_buffer.append(mask)

        # Keep only the last N frames
        if len(self.frame_buffer) > self.COMBINE_LAST_N:
            self.frame_buffer.pop(0)

        # Only process once enough frames are collected
        if len(self.frame_buffer) == self.COMBINE_LAST_N:
            combined_mask = self.soft_combine_masks(self.frame_buffer)
            is_red_light = np.any(combined_mask)

            current_bool = not is_red_light
            if current_bool != self.previous_state:
                if not self.spotted_once and not current_bool:
                    self.spotted_once = True

                self.publisher_.publish(Bool(data=current_bool))
                self.get_logger().info(
                    f"Traffic light is {'RED' if is_red_light else 'NOT RED'}")
                self.previous_state = current_bool

            cv2.imshow("Combined Filtered Structures", combined_mask)
            cv2.waitKey(1)

    def detect_red_light(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Red detection ranges
        lower_red1 = (0, 1, 150)
        upper_red1 = (30, 255, 255)
        lower_red2 = (150, 1, 150)
        upper_red2 = (180, 255, 255)

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = mask1 | mask2

        # Brightness mask
        v_channel = hsv[:, :, 2]
        _, brightness_mask = cv2.threshold(
            v_channel, 150, 255, cv2.THRESH_BINARY)

        # Combine masks
        combined_mask = cv2.bitwise_and(red_mask, brightness_mask)

        # Filter by shape
        filtered_mask = np.zeros_like(combined_mask)
        contours, _ = cv2.findContours(
            combined_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 20:
                (x, y), radius = cv2.minEnclosingCircle(cnt)
                circle_area = np.pi * radius * radius
                shape_ratio = area / circle_area if circle_area > 0 else 0
                if 0.6 < shape_ratio < 1.4:
                    cv2.drawContours(
                        filtered_mask, [cnt], -1, 255, thickness=cv2.FILLED)

        return filtered_mask

    def soft_combine_masks(self, masks):
        """
        Keep regions if they intersect even a bit with previous.
        """
        if not masks:
            return np.zeros_like(masks[0])

        combined = masks[0].copy()
        for i in range(1, len(masks)):
            overlap = cv2.bitwise_and(combined, masks[i])
            keep_mask = cv2.threshold(overlap, 1, 255, cv2.THRESH_BINARY)[1]
            # Only keep parts of current that overlap
            new_combined = np.zeros_like(combined)
            new_combined[keep_mask > 0] = masks[i][keep_mask > 0]
            combined = new_combined.copy()

        return combined


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

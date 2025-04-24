import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
from cv_bridge import CvBridge
from std_msgs.msg import String

class SignDetectionNode(Node):
    def __init__(self):
        super().__init__('sign_detection_node')

        # Subscriber für das komprimierte Bild vom Topic /image_raw/compressed
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',  # Das Topic für komprimierte Bilder
            self.listener_callback,
            10
        )

        # Lade das Template-Bild (Verkehrszeichen) – dieses kannst du anpassen
        self.template = cv2.imread('/home/fabian/ros2_ws/src/waymo/waymo/traffic_signs/park_sign.png', cv2.IMREAD_COLOR)

        # Hole die Dimensionen des Templates
        self.h, self.w = self.template.shape[:2]

        # Initialisiere CvBridge
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        # Entpacke das komprimierte Bild
        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Führe das Template Matching durch
        result = cv2.matchTemplate(image, self.template, cv2.TM_CCOEFF_NORMED)

        # Finde die Position des besten Matches
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)

        # Zeichne ein Rechteck um das gefundene Verkehrszeichen
        top_left = max_loc
        bottom_right = (top_left[0] + self.w, top_left[1] + self.h)
        cv2.rectangle(image, top_left, bottom_right, (0, 255, 0), 2)

        # Zeige das Ergebnis an
        cv2.imshow('Verkehrszeichenerkennung', image)
        cv2.waitKey(1)

        # Zeige den Wert der besten Übereinstimmung
        self.get_logger().info(f"Beste Übereinstimmung: {max_val}")

def main(args=None):
    rclpy.init(args=args)
    node = SignDetectionNode()
    rclpy.spin(node)

    # Zerstöre den Node nach Beenden
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

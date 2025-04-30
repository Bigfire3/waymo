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

        # Dictionary von Templates und ihren Namen (Templates werden hier als binäre Bilder geladen)
        self.templates = {
            'park_sign': cv2.imread('/home/fabian/ros2_ws/src/waymo/waymo/traffic_signs/park_sign.png', cv2.IMREAD_COLOR),
            'straight_sign': cv2.imread('/home/fabian/ros2_ws/src/waymo/waymo/traffic_signs/straight_sign.png', cv2.IMREAD_COLOR),
            'left_sign': cv2.imread('/home/fabian/ros2_ws/src/waymo/waymo/traffic_signs/left_sign.png', cv2.IMREAD_COLOR),
            'right_sign': cv2.imread('/home/fabian/ros2_ws/src/waymo/waymo/traffic_signs/right_sign.png', cv2.IMREAD_COLOR),        
        }

        # Überprüfen, ob alle Templates erfolgreich geladen wurden
        for name, template in self.templates.items():
            if template is None:
                self.get_logger().error(f"Template '{name}' konnte nicht geladen werden! Überprüfe den Pfad.")

        # Konvertiere die Templates in Graustufen und wende eine Schwellwertoperation an (binär)
        self.templates_bin = {name: self.convert_to_binary(template) for name, template in self.templates.items()}

        # Initialisiere CvBridge
        self.bridge = CvBridge()

    def convert_to_binary(self, image):
        """
        Diese Methode konvertiert das Bild in Graustufen und dann in ein Binärbild.
        """
        # Konvertiere das Bild in Graustufen
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Wende eine Schwellwertoperation an (Binärbild)
        _, binary_image = cv2.threshold(gray_image, 127, 255, cv2.THRESH_BINARY)

        return binary_image

    def listener_callback(self, msg):
        # Entpacke das komprimierte Bild
        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if image is None:
            self.get_logger().warn("Fehler beim Decodieren des Bildes.")
            return

        # Konvertiere das empfangene Bild in Graustufen
        image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Wende eine Schwellwertoperation an (Binärbild)
        _, image_bin = cv2.threshold(image_gray, 80, 255, cv2.THRESH_BINARY)

        # Zeige das binäre Bild an
        cv2.imshow('Binary Image', image_bin)  # Binärbild anzeigen
        cv2.waitKey(1)

        # Durchlaufe alle Templates und führe das Template Matching durch
        for template_name, template_bin in self.templates_bin.items():
            # Hole die Dimensionen des Templates
            h, w = template_bin.shape[:2]

            # Führe das Template Matching auf dem binären Bild durch
            result = cv2.matchTemplate(image_bin, template_bin, cv2.TM_CCOEFF_NORMED)

            # Finde die Position des besten Matches
            min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)

            # Wenn das Template gut erkannt wurde, zeichne ein Rechteck und markiere es
            if max_val > 0.1:  # Schwellenwert anpassen, je nach Anforderungen
                top_left = max_loc
                bottom_right = (top_left[0] + w, top_left[1] + h)

                # Unterschiedliche Farben für jedes Template
                color = (0, 255, 0)  # Standard: Grün
                if template_name == 'straight_sign':
                    color = (0, 0, 255)
                elif template_name == 'park_sign':  # z.B. Park Sign
                    color = (255, 0, 0)  # Blau für Parken

                # Rechteck zeichnen
                cv2.rectangle(image, top_left, bottom_right, color, 2)

                # Schreibe den Key des Dictionaries (Name des Templates) über das Rechteck
                font = cv2.FONT_HERSHEY_SIMPLEX
                cv2.putText(image, template_name, (top_left[0], top_left[1] - 10), font, 0.7, color, 2, cv2.LINE_AA)

        # Zeige das Ergebnis an
        cv2.imshow('Sign Detection', image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = SignDetectionNode()
    rclpy.spin(node)

    # Zerstöre den Node nach Beenden
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
from cv_bridge import CvBridge
import os
from std_msgs.msg import String  # Bool-Nachricht für das Veröffentlichen von True/False

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

        # Publisher für das Parkplatzzeichen
        self.parking_sign_publisher = self.create_publisher(String, '/sign', 10)

        # Lade Templates aus dem Ordner /traffic_sign
        self.templates = self.load_templates('/home/fabian/ros2_ws/src/waymo/waymo/traffic_signs')

        # Überprüfen, ob Templates erfolgreich geladen wurden
        for name, template in self.templates.items():
            if template is None:
                self.get_logger().error(f"Template '{name}' konnte nicht geladen werden! Überprüfe den Pfad.")

        # Konvertiere die Templates in Graustufen und wende eine Schwellwertoperation an (binär)
        self.templates_bin = {name: self.convert_to_binary(template) for name, template in self.templates.items()}

        # Initialisiere CvBridge
        self.bridge = CvBridge()

    def load_templates(self, folder_path):
        """
        Lädt alle Bilddateien aus dem Ordner und gibt ein Dictionary mit
        dem Dateinamen (ohne Erweiterung) und dem Bild zurück.
        """
        templates = {}
        for filename in os.listdir(folder_path):
            # Nur Bilddateien laden (PNG, JPG, etc.)
            if filename.endswith('.png') or filename.endswith('.jpg'):
                # Erstelle den vollen Pfad zum Template-Bild
                image_path = os.path.join(folder_path, filename)
                template = cv2.imread(image_path, cv2.IMREAD_COLOR)
                if template is not None:
                    # Verwende den Dateinamen ohne Erweiterung als Key
                    template_name = os.path.splitext(filename)[0]
                    templates[template_name] = template
                else:
                    self.get_logger().error(f"Fehler beim Laden von {filename}.")
        return templates

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

        # Durchlaufe alle Templates und führe das Template Matching durch
        for template_name, template_bin in self.templates_bin.items():
            # Hole die Dimensionen des Templates
            h, w = template_bin.shape[:2]

            # Führe das Template Matching auf dem binären Bild durch
            result = cv2.matchTemplate(image_bin, template_bin, cv2.TM_CCOEFF_NORMED)

            # Finde die Position des besten Matches
            min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)

            # Wenn das Template gut erkannt wurde, zeichne ein Rechteck und markiere es
            if max_val > 0.65:  # Schwellenwert anpassen, je nach Anforderungen
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

                # Wenn das Parkzeichen erkannt wurde, sende eine Nachricht
                if template_name == 'park_sign_0' or template_name == 'park_sign_1':
                    # Veröffentliche 'True' auf dem Topic '/sign/parking_sign'
                    parking_sign_msg = String()
                    parking_sign_msg.data = "parking_sign"
                    self.parking_sign_publisher.publish(parking_sign_msg)

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

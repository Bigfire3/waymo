import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import os
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from ament_index_python.packages import get_package_share_directory

class SignDetectionNode(Node):
    def __init__(self):
        super().__init__('sign_detection_node')

        def bool_desc(desc):
             return ParameterDescriptor(type=ParameterType.PARAMETER_BOOL, description=desc)

        # Debug Publisher Flags
        self.declare_parameter('publish_binary_sign', True, bool_desc("Publish binary frame with colored detection boxes as cv2 image"))

        # --- QoS Profile ---
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=1
        )
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1
        )

        # Subscriber für das komprimierte Bild vom Topic /image_raw/compressed
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.listener_callback,
            qos_best_effort
        )

        # Publisher für Erkennung von Straßenschildern
        self.parking_sign_publisher = self.create_publisher(String, '/sign', qos_reliable)

        # Publisher für Debug-Bilder (Binärbild mit farbigem Rahmen)
        self.pub_binary_sign_with_box = self.create_publisher(CompressedImage, '/debug/cam/binary_sign_boxed', qos_best_effort)
        # Hinweis: Ich habe das Topic leicht angepasst zu '/debug/cam/binary_sign_boxed'
        # um klarzustellen, dass es das Binärbild mit Box ist.
        # Wenn du exakt '/debug/cam/binary_sign' beibehalten willst, ändere dies zurück.

        # Ermittle den Pfad zum 'share'-Verzeichnis des eigenen Pakets ('waymo')
        package_share_directory = get_package_share_directory('waymo')
        # Konstruiere den vollständigen Pfad zum 'traffic_signs'-Ordner
        templates_folder_path = os.path.join(package_share_directory, 'traffic_signs')
        
        # self.get_logger().info(f"Lade Templates aus dem Verzeichnis: '{templates_folder_path}'")

        self.templates = self.load_templates(templates_folder_path)

        if not self.templates:
            self.get_logger().error("Keine Templates wurden geladen! Überprüfe den Pfad und die setup.py Konfiguration.")
        else:
            for name, template in self.templates.items():
                if template is None:
                    self.get_logger().error(f"Template '{name}' konnte nicht korrekt initialisiert werden (ist None).")

        self.templates_bin = {name: self.convert_to_binary(template) for name, template in self.templates.items() if template is not None}

        self.bridge = CvBridge()

    def _publish_image(self, publisher, image, timestamp):
        """Hilfsfunktion zum Komprimieren und Publishen eines Bildes."""
        if image is None:
            self.get_logger().warn(f"Versuch, ein None-Bild auf Topic '{publisher.topic}' zu publishen.", throttle_duration_sec=5)
            return
        try:
             processed_image = image.copy() 
             if processed_image.ndim == 2:
                  processed_image = cv2.cvtColor(processed_image, cv2.COLOR_GRAY2BGR)
             
             if processed_image.dtype != np.uint8:
                 if np.max(processed_image) <= 1.0 and (processed_image.dtype == np.float32 or processed_image.dtype == np.float64) :
                     processed_image = (processed_image * 255).astype(np.uint8)
                 else:
                     processed_image = processed_image.astype(np.uint8)

             ret, buffer = cv2.imencode('.jpg', processed_image, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
             if not ret:
                 self.get_logger().warn(f"Fehler beim JPEG-Encoding für Topic '{publisher.topic}'.")
                 return

             msg = CompressedImage(format="jpeg", data=buffer.tobytes())
             msg.header.stamp = timestamp
             publisher.publish(msg)
        except CvBridgeError as e:
              self.get_logger().error(f"CvBridge Fehler beim Publishen auf '{publisher.topic}': {e}", throttle_duration_sec=5)
        except Exception as e:
              self.get_logger().error(f"Allgemeiner Fehler beim Publishen auf '{publisher.topic}': {e}", throttle_duration_sec=5)

    def load_templates(self, folder_path):
        templates = {}
        if not os.path.isdir(folder_path):
            self.get_logger().error(f"Template-Ordner nicht gefunden oder ist kein Verzeichnis: {folder_path}")
            return templates 

        for filename in os.listdir(folder_path):
            if filename.lower().endswith(('.png', '.jpg', '.jpeg')):
                image_path = os.path.join(folder_path, filename)
                try:
                    template = cv2.imread(image_path, cv2.IMREAD_COLOR)
                    if template is not None:
                        template_name = os.path.splitext(filename)[0]
                        templates[template_name] = template
                        # self.get_logger().info(f"Template '{template_name}' erfolgreich geladen von '{image_path}'.")
                    else:
                        self.get_logger().error(f"Fehler beim Laden von Template '{filename}' (imread gab None zurück) von Pfad: {image_path}")
                except Exception as e:
                    self.get_logger().error(f"Ausnahme beim Laden von Template '{filename}' von Pfad '{image_path}': {e}")
        
        if not templates:
            self.get_logger().warn(f"Keine Templates im Ordner '{folder_path}' gefunden oder geladen.")
        return templates

    def convert_to_binary(self, image):
        if image is None:
            self.get_logger().error("Versuch, ein None-Bild zu konvertieren (convert_to_binary).")
            return None

        if len(image.shape) == 2 or image.shape[2] == 1:
            gray_image = image
        else:
            gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        _, binary_image = cv2.threshold(gray_image, 127, 255, cv2.THRESH_BINARY)
        return binary_image

    def listener_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            image_color_input = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # Original Farbbild
            timestamp = msg.header.stamp
        except Exception as e:
            self.get_logger().error(f"Fehler beim Dekodieren des komprimierten Bildes: {e}")
            return

        if image_color_input is None:
            self.get_logger().warn("Bild konnte nicht dekodiert werden (ist None).")
            return

        # 1. Binärbild für die Erkennungslogik erstellen
        image_gray_for_detection = cv2.cvtColor(image_color_input, cv2.COLOR_BGR2GRAY)
        _, image_bin_for_detection = cv2.threshold(image_gray_for_detection, 127, 255, cv2.THRESH_BINARY) 

        # 2. Vorbereitung des Debug-Bildes: Konvertiere das Binärbild der Erkennung in ein BGR-Format
        # Dieses Bild wird schwarz-weiß aussehen, aber Farbkanäle haben.
        if self.get_parameter('publish_binary_sign').value:
            # Nur erstellen, wenn es auch publiziert wird
            debug_image_base = cv2.cvtColor(image_bin_for_detection, cv2.COLOR_GRAY2BGR)
        else:
            debug_image_base = None # Wird nicht benötigt

        detected_signs_on_current_frame = False

        for template_name, template_bin in self.templates_bin.items():
            if template_bin is None:
                self.get_logger().warn(f"Template '{template_name}' ist None, wird übersprungen.")
                continue

            h, w = template_bin.shape[:2]

            if image_bin_for_detection.shape[0] < h or image_bin_for_detection.shape[1] < w:
                continue

            result = cv2.matchTemplate(image_bin_for_detection, template_bin, cv2.TM_CCOEFF_NORMED)
            min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)

            detection_threshold = 0.65 
            if max_val > detection_threshold:
                top_left = max_loc
                bottom_right = (top_left[0] + w, top_left[1] + h)
                
                # Farbe für den Rahmen und Text (BGR für Blau)
                frame_color = (255, 0, 0) 

                # 3. Zeichne den Rahmen auf dem BGR-konvertierten Binärbild, falls Publishing aktiv ist
                if debug_image_base is not None:
                    cv2.rectangle(debug_image_base, top_left, bottom_right, frame_color, 2)
                    cv2.putText(debug_image_base, template_name, (top_left[0], top_left[1] - 10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, frame_color, 2, cv2.LINE_AA)

                # Logging und Parking Sign Publishing bleiben wie zuvor
                # self.get_logger().info(f"'{template_name}' erkannt mit Konfidenz: {max_val:.2f}")

                if template_name in ('park_sign_0', 'park_sign_1') and not detected_signs_on_current_frame:
                    parking_sign_msg = String()
                    parking_sign_msg.data = "parking_sign_detected"
                    self.parking_sign_publisher.publish(parking_sign_msg)
                    # self.get_logger().info(f"Nachricht für '{template_name}' gesendet.")
                    detected_signs_on_current_frame = True
        
        # 4. Publishe das BGR-konvertierte Binärbild mit den (ggf. blauen) Rahmen
        if self.get_parameter('publish_binary_sign').value and debug_image_base is not None:
                self._publish_image(self.pub_binary_sign_with_box, debug_image_base, timestamp)

def main(args=None):
    rclpy.init(args=args)
    node = SignDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node and rclpy.ok():
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
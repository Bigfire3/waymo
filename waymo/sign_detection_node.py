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
from ament_index_python.packages import get_package_share_directory # WICHTIG: Dieser Import wird benötigt

class SignDetectionNode(Node):
    def __init__(self):
        super().__init__('sign_detection_node')

        def bool_desc(desc):
             return ParameterDescriptor(type=ParameterType.PARAMETER_BOOL, description=desc)

        # Debug Publisher Flags
        self.declare_parameter('publish_template_matching', True, bool_desc("Publish template matching results as cv2 image"))
        self.declare_parameter('publish_binary_sign', True, bool_desc("Publish binary frame results as cv2 image"))


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

        # Publisher für Debug-Bilder
        self.pub_template_matching = self.create_publisher(CompressedImage, '/debug/cam/template_matching', qos_best_effort)
        self.pub_binary_sign = self.create_publisher(CompressedImage, '/debug/cam/binary_sign', qos_best_effort)

        # --- ANPASSUNG FÜR TEMPLATE-PFAD ---
        # Ermittle den Pfad zum 'share'-Verzeichnis des eigenen Pakets ('waymo')
        package_share_directory = get_package_share_directory('waymo')
        # Konstruiere den vollständigen Pfad zum 'traffic_signs'-Ordner
        templates_folder_path = os.path.join(package_share_directory, 'traffic_signs')
        
        # self.get_logger().info(f"Lade Templates aus dem Verzeichnis: '{templates_folder_path}'")

        # Lade Templates aus dem korrekten Pfad
        self.templates = self.load_templates(templates_folder_path)
        # --- ENDE ANPASSUNG ---

        # Überprüfen, ob Templates erfolgreich geladen wurden
        # if not self.templates: # Prüft, ob das Dictionary leer ist
            # self.get_logger().error("Keine Templates wurden geladen! Überprüfe den Pfad und die setup.py Konfiguration.")
        # else:
        #     for name, template in self.templates.items():
        #         if template is None: # Sollte durch die Prüfung in load_templates schon abgefangen sein
                    # self.get_logger().error(f"Template '{name}' konnte nicht korrekt initialisiert werden (ist None).")

        # Konvertiere die Templates in Graustufen und wende eine Schwellwertoperation an (binär)
        self.templates_bin = {name: self.convert_to_binary(template) for name, template in self.templates.items() if template is not None}

        # Initialisiere CvBridge
        self.bridge = CvBridge()

    def _publish_image(self, publisher, image, timestamp):
        """Hilfsfunktion zum Komprimieren und Publishen eines Bildes."""
        if image is None: return
        try:
             if image.ndim == 2: # Wenn Graustufenbild, konvertiere zu BGR für JPEG-Encoding
                  image_bgr = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
             else:
                  image_bgr = image

             # JPEG-Kompression
             # Stelle sicher, dass das Bild im Format BGR oder GRAY (uint8) ist
             if image_bgr.dtype != np.uint8:
                 image_bgr = image_bgr.astype(np.uint8) # Konvertiere falls nötig, sei vorsichtig mit Skalierung

             ret, buffer = cv2.imencode('.jpg', image_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 85]) # 85 ist ein guter Kompromiss für Qualität/Größe
             if not ret:
                 self.get_logger().warn(f"Fehler beim JPEG-Encoding für Topic '{publisher.topic}'.")
                 return

             msg = CompressedImage(format="jpeg", data=buffer.tobytes())
             msg.header.stamp = timestamp
             publisher.publish(msg)
        except CvBridgeError as e: # Spezifischer Fehler
              self.get_logger().error(f"CvBridge Fehler beim Publishen auf '{publisher.topic}': {e}", throttle_duration_sec=5)
        except Exception as e: # Allgemeiner Fehler
              self.get_logger().error(f"Allgemeiner Fehler beim Publishen auf '{publisher.topic}': {e}", throttle_duration_sec=5)


    def load_templates(self, folder_path):
        """
        Lädt alle Bilddateien aus dem Ordner und gibt ein Dictionary mit
        dem Dateinamen (ohne Erweiterung) und dem Bild zurück.
        """
        templates = {}
        # Überprüfe zuerst, ob der angegebene Pfad ein Verzeichnis ist
        if not os.path.isdir(folder_path):
            self.get_logger().error(f"Template-Ordner nicht gefunden oder ist kein Verzeichnis: {folder_path}")
            return templates # Gib ein leeres Dictionary zurück, wenn der Ordner nicht existiert

        for filename in os.listdir(folder_path):
            # Nur Bilddateien laden (PNG, JPG, etc.)
            if filename.lower().endswith(('.png', '.jpg', '.jpeg')): # .lower() für Groß-/Kleinschreibung, Tuple für mehrere Endungen
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
        """
        Diese Methode konvertiert das Bild in Graustufen und dann in ein Binärbild.
        """
        if image is None:
            self.get_logger().error("Versuch, ein None-Bild zu konvertieren (convert_to_binary).")
            return None # Gebe None zurück, wenn das Eingangsbild None ist

        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        _, binary_image = cv2.threshold(gray_image, 127, 255, cv2.THRESH_BINARY)
        return binary_image

    def listener_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            timestamp = msg.header.stamp
        except Exception as e:
            self.get_logger().error(f"Fehler beim Dekodieren des komprimierten Bildes: {e}")
            return

        if image is None:
            self.get_logger().warn("Bild konnte nicht dekodiert werden (ist None).")
            return

        image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        _, image_bin = cv2.threshold(image_gray, 80, 255, cv2.THRESH_BINARY) # Schwellenwert für Szenenbild

        detected_signs_on_current_frame = False # Flag um Mehrfach-Publishen pro Frame zu vermeiden (optional)

        for template_name, template_bin in self.templates_bin.items():
            if template_bin is None: # Überspringe, falls ein Template nicht korrekt geladen wurde
                continue

            h, w = template_bin.shape[:2]

            if image_bin.shape[0] < h or image_bin.shape[1] < w:
                # self.get_logger().warn(f"Szenenbild ist kleiner als Template '{template_name}'. Überspringe.", throttle_duration_sec=10)
                continue # Template ist größer als das Bild, Matching nicht möglich

            result = cv2.matchTemplate(image_bin, template_bin, cv2.TM_CCOEFF_NORMED)
            min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)

            if max_val > 0.65: # Schwellenwert anpassen
                top_left = max_loc
                bottom_right = (top_left[0] + w, top_left[1] + h)

                color_map = {
                    'straight_sign': (0, 0, 255), # Rot
                    'park_sign_0': (255, 0, 0),   # Blau
                    'park_sign_1': (255, 0, 0),   # Blau
                    'left_sign': (0, 255, 255), # Gelb
                    'right_sign': (255, 0, 255) # Magenta
                }
                color = color_map.get(template_name, (0, 255, 0)) # Standard: Grün

                cv2.rectangle(image, top_left, bottom_right, color, 2)
                cv2.putText(image, template_name, (top_left[0], top_left[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv2.LINE_AA)

                if template_name in ('park_sign_0', 'park_sign_1') and not detected_signs_on_current_frame:
                    parking_sign_msg = String()
                    parking_sign_msg.data = "parking_sign_detected" # Klarere Nachricht
                    self.parking_sign_publisher.publish(parking_sign_msg)
                    # self.get_logger().info(f"'{template_name}' erkannt und Nachricht gesendet.")
                    detected_signs_on_current_frame = True # Verhindere mehrfaches Senden pro Frame

        if self.get_parameter('publish_template_matching').value:
                self._publish_image(self.pub_template_matching, image, timestamp)
        if self.get_parameter('publish_binary_sign').value:
                self._publish_image(self.pub_binary_sign, image_bin, timestamp)

def main(args=None):
    rclpy.init(args=args)
    node = SignDetectionNode()
    try:
        rclpy.spin(node)        
        rclpy.spin(SignDetectionNode)
    except KeyboardInterrupt:
        pass
    finally:
        if SignDetectionNode and isinstance(SignDetectionNode, Node) and rclpy.ok(): SignDetectionNode.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
        try: pass#cv2.destroyAllWindows()
        except Exception: pass

if __name__ == '__main__':
    main()
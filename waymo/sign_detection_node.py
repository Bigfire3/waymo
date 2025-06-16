import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import os
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, IntegerRange, FloatingPointRange
from ament_index_python.packages import get_package_share_directory

class SignDetectionNode(Node):
    def __init__(self):
        super().__init__('sign_detection_node')

        def bool_desc(desc):
             return ParameterDescriptor(type=ParameterType.PARAMETER_BOOL, description=desc)
        def int_desc(desc, min_val=0, max_val=255, step=1): return ParameterDescriptor(
            type=ParameterType.PARAMETER_INTEGER, description=desc,
            integer_range=[IntegerRange(from_value=min_val, to_value=max_val, step=step)])
        
        self.declare_parameter('publish_binary_sign', True, bool_desc("Publish binary frame with colored detection boxes as cv2 image"))
        self.declare_parameter('binary_threshold', 70, int_desc("Threshold for binary image"))
        # NEU: Parameter für die Erkennungsschwelle des Template Matchings
        self.declare_parameter('detection_confidence_threshold', 0.75, 
                               ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE, 
                                                 description="Confidence threshold for template matching (0.0 to 1.0)",
                                                 floating_point_range=[FloatingPointRange(from_value=0.0, to_value=1.0, step=0.01)]))


        qos_reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1)
        qos_best_effort = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)

        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.listener_callback,
            qos_best_effort
        )
        self.sign_publisher = self.create_publisher(String, '/sign', qos_reliable)
        self.pub_binary_sign_with_box = self.create_publisher(CompressedImage, '/debug/cam/binary_sign_boxed', qos_best_effort)

        package_share_directory = get_package_share_directory('waymo')
        templates_folder_path = os.path.join(package_share_directory, 'traffic_signs')
        
        self.templates = self.load_templates(templates_folder_path)
        if not self.templates:
            self.get_logger().error("Keine Templates wurden geladen! Überprüfe den Pfad und die setup.py Konfiguration.")
        else:
            for name, template in self.templates.items():
                if template is None:
                    self.get_logger().error(f"Template '{name}' konnte nicht korrekt initialisiert werden (ist None).")

        # Die templates_bin werden jetzt on-the-fly in der listener_callback Methode erstellt,
        # da der binary_threshold Parameter sich ändern kann.
        # Alternativ könnte man die Templates neu binarisieren, wenn sich der Parameter ändert.
        # Für Einfachheit hier: Konvertierung bei jeder Bildverarbeitung.
        # Wenn Performance kritisch ist, Parameter-Callback für binary_threshold implementieren und templates_bin neu erstellen.
        self.templates_bin = {} # Initial leer

        self.bridge = CvBridge()

    def _publish_image(self, publisher, image, timestamp):
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
                    else:
                        self.get_logger().error(f"Fehler beim Laden von Template '{filename}' (imread gab None zurück) von Pfad: {image_path}")
                except Exception as e:
                    self.get_logger().error(f"Ausnahme beim Laden von Template '{filename}' von Pfad '{image_path}': {e}")
        if not templates:
            self.get_logger().warn(f"Keine Templates im Ordner '{folder_path}' gefunden oder geladen.")
        return templates

    def convert_to_binary(self, image, binary_threshold_value): # Nimmt jetzt den Threshold-Wert entgegen
        if image is None:
            self.get_logger().error("Versuch, ein None-Bild zu konvertieren (convert_to_binary).")
            return None
        if len(image.shape) == 2 or image.shape[2] == 1: # Bereits Graustufen
            gray_image = image
        else:
            gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        _, binary_image = cv2.threshold(gray_image, binary_threshold_value, 255, cv2.THRESH_BINARY)
        # cv2.imshow("Binary Image", binary_image)  # Debugging-Zweck, kann entfernt werden
        # cv2.waitKey(0)
        return binary_image

    def listener_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            image_color_input = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            timestamp = msg.header.stamp
        except Exception as e:
            self.get_logger().error(f"Fehler beim Dekodieren des komprimierten Bildes: {e}")
            return

        if image_color_input is None:
            self.get_logger().warn("Bild konnte nicht dekodiert werden (ist None).")
            return

        # Bildauflösung (angenommen konstant, sonst aus image_color_input.shape holen)
        img_height, img_width = image_color_input.shape[:2] # Sollte 240, 320 sein

        # Definiere die ROI (oberes rechtes Viertel)
        roi_x_start = img_width // 2
        roi_y_start = 0
        roi_width = img_width // 2
        roi_height = img_height // 2
        
        # Extrahiere die ROI aus dem Farbbild
        image_roi_color = image_color_input[roi_y_start : roi_y_start + roi_height, 
                                            roi_x_start : roi_x_start + roi_width]

        if image_roi_color.size == 0:
            self.get_logger().warn("ROI ist leer, überspringe Frame.")
            return

        # Hole aktuellen Binärschwellenwert
        binary_threshold = self.get_parameter('binary_threshold').value
        detection_confidence = self.get_parameter('detection_confidence_threshold').value

        # Konvertiere die ROI in ein Binärbild für die Erkennung
        image_gray_roi_for_detection = cv2.cvtColor(image_roi_color, cv2.COLOR_BGR2GRAY)
        _, image_bin_roi_for_detection = cv2.threshold(image_gray_roi_for_detection, binary_threshold, 255, cv2.THRESH_BINARY)

        # Debug-Bild vorbereiten: Das *gesamte* Binärbild (für Kontext), aber Rahmen werden auf ROI-Koordinaten gezeichnet
        debug_image_base = None
        if self.get_parameter('publish_binary_sign').value:
            # Erstelle ein komplettes Binärbild für das Debugging, um die ROI-Box zu visualisieren
            image_gray_full = cv2.cvtColor(image_color_input, cv2.COLOR_BGR2GRAY)
            _, image_bin_full_for_debug = cv2.threshold(image_gray_full, binary_threshold, 255, cv2.THRESH_BINARY)
            debug_image_base = cv2.cvtColor(image_bin_full_for_debug, cv2.COLOR_GRAY2BGR)
            # Zeichne die ROI-Grenze in das Debug-Bild
            cv2.rectangle(debug_image_base, (roi_x_start, roi_y_start), 
                          (roi_x_start + roi_width -1 , roi_y_start + roi_height -1), (147, 20, 255), 2) # Lila ROI-Box

        detected_signs_on_current_frame = False

        # Binarisiere Templates on-the-fly mit dem aktuellen Threshold
        # oder implementiere einen Parameter-Callback, um templates_bin bei Änderung neu zu erstellen.
        # Hier für Einfachheit: Konvertierung bei Bedarf.
        current_templates_bin = {name: self.convert_to_binary(template, binary_threshold) 
                                 for name, template in self.templates.items() if template is not None}


        for template_name, template_bin in current_templates_bin.items():
            if template_bin is None:
                # self.get_logger().warn(f"Binarisiertes Template '{template_name}' ist None, wird übersprungen.")
                continue

            h, w = template_bin.shape[:2]

            # Stelle sicher, dass das Template in die ROI passt
            if image_bin_roi_for_detection.shape[0] < h or image_bin_roi_for_detection.shape[1] < w:
                # self.get_logger().debug(f"Template {template_name} ({w}x{h}) zu groß für ROI ({image_bin_roi_for_detection.shape[1]}x{image_bin_roi_for_detection.shape[0]}).")
                continue
            
            # Template Matching nur auf der binären ROI durchführen
            result = cv2.matchTemplate(image_bin_roi_for_detection, template_bin, cv2.TM_CCOEFF_NORMED)
            min_val, max_val, min_loc, max_loc_roi = cv2.minMaxLoc(result) # max_loc_roi ist relativ zur ROI

            if max_val > detection_confidence:
                # Konvertiere lokale ROI-Koordinaten zu globalen Bildkoordinaten
                top_left_global = (max_loc_roi[0] + roi_x_start, max_loc_roi[1] + roi_y_start)
                bottom_right_global = (top_left_global[0] + w, top_left_global[1] + h)
                
                frame_color = (255, 0, 0) # Blaue Box für Detektion

                if debug_image_base is not None:
                    cv2.rectangle(debug_image_base, top_left_global, bottom_right_global, frame_color, 2)
                    cv2.putText(debug_image_base, template_name, (top_left_global[0], top_left_global[1] - 10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, frame_color, 2, cv2.LINE_AA) # Kleinere Schrift

                # self.get_logger().info(f"'{template_name}' erkannt bei ({top_left_global}) mit Konfidenz: {max_val:.2f}")

                # Nachrichten senden (Logik unverändert, aber nur ein Schild pro Frame)
                if not detected_signs_on_current_frame:
                    sign_msg_data = None
                    if template_name in ('park_sign_0', 'park_sign_1'):
                        sign_msg_data = "parking_sign_detected"
                    elif template_name in ('straight_sign_0', 'straight_sign_1', ):
                        sign_msg_data = "straight_sign_detected"
                    elif template_name in ('left_sign_0', 'left_sign_1'):
                        sign_msg_data = "left_sign_detected"
                    elif template_name in ('right_sign_0', 'right_sign_1'):
                        sign_msg_data = "right_sign_detected"
                    
                    if sign_msg_data:
                        msg_to_send = String()
                        msg_to_send.data = sign_msg_data
                        self.sign_publisher.publish(msg_to_send)
                        # self.get_logger().info(f"Nachricht für '{template_name}' ({sign_msg_data}) gesendet.")
                        detected_signs_on_current_frame = True # Nur das erste erkannte Schild pro Frame senden
        
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
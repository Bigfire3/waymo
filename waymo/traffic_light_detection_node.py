# waymo/traffic_light_detection_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import cv2
import numpy as np
import sys
import traceback
import time
from cv_bridge import CvBridge, CvBridgeError # Hinzugefügt

# --- Imports für Parameter ---
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, IntegerRange, FloatingPointRange

class TrafficLightDetector(Node):
    def __init__(self):
        super().__init__('traffic_light_detection_node')
        self.bridge = CvBridge() # CvBridge Instanz

        # --- QoS Profile ---
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=1
        )
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1
        )

        # --- Parameter Descriptors (wie in deiner "alten" Datei) ---
        def int_desc(desc, min_val=0, max_val=255, step=1):
            return ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER, description=desc,
                integer_range=[IntegerRange(from_value=min_val, to_value=max_val, step=step)])
        def int_area_desc(desc, min_val=0, max_val=5000, step=1): # Max Area wie im Original
             return ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER, description=desc,
                integer_range=[IntegerRange(from_value=min_val, to_value=max_val, step=step)])
        # Float Desc hinzugefügt für roi_crop_factor_h, falls nicht vorhanden
        def float_desc(desc, min_val=0.0, max_val=1.0, step=0.01):
             return ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE, description=desc,
                floating_point_range=[FloatingPointRange(from_value=min_val, to_value=max_val, step=step)])
        def bool_desc(desc):
             return ParameterDescriptor(type=ParameterType.PARAMETER_BOOL, description=desc)


        # --- Parameter Deklarationen ---
        # ... (andere Parameter wie zuvor) ...
        # NEU: Parameter für Rotbereich 1 (niedrige Hue Werte)
        self.declare_parameter('hsv_lower_h1', 0, int_desc("Lower Hue 1 (0-180)", max_val=180))
        self.declare_parameter('hsv_upper_h1', 10, int_desc("Upper Hue 1 (0-180)", max_val=180))
        # NEU: Parameter für Rotbereich 2 (hohe Hue Werte)
        self.declare_parameter('hsv_lower_h2', 160, int_desc("Lower Hue 2 (0-180)", max_val=180))
        self.declare_parameter('hsv_upper_h2', 180, int_desc("Upper Hue 2 (0-180)", max_val=180))
        # Gemeinsame S und V Parameter
        self.declare_parameter('hsv_lower_s', 50, int_desc("Lower Saturation (0-255)")) # Startwert erhöht
        self.declare_parameter('hsv_lower_v', 10, int_desc("Lower Value (0-255)"))     # Startwert erhöht
        self.declare_parameter('hsv_upper_s', 255, int_desc("Upper Saturation (0-255)"))
        self.declare_parameter('hsv_upper_v', 255, int_desc("Upper Value (0-255)"))     # Oberen Wert evtl. anpassen
        # Blob Area
        self.declare_parameter('min_blob_area', 100, int_area_desc("Minimum Blob Area (pixels)")) # Angepasster Startwert
        self.declare_parameter('max_blob_area', 200, int_area_desc("Maximum Blob Area (pixels)"))# Deutlich erhöht
        # ROI
        self.declare_parameter('roi_crop_factor_h', 0.6, float_desc("ROI Crop Factor Height (0.1-1.0)", min_val=0.1, max_val=1.0, step = 0.01))

        # Debug Publisher Flags
        self.declare_parameter('publish_mask', True, bool_desc("Publish filtered color mask image"))
        self.declare_parameter('publish_overlay', True, bool_desc("Publish color detection overlay image"))

        # Subscriber und Publisher
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.image_callback,
            qos_best_effort)

        self.publisher_ = self.create_publisher(Bool, 'traffic_light', qos_reliable)

        # --- Debug Publisher (unverändert) ---
        self.pub_mask = self.create_publisher(CompressedImage, '/debug/cam/traffic_mask', qos_best_effort)
        self.pub_overlay = self.create_publisher(CompressedImage, '/debug/cam/traffic_overlay', qos_best_effort)


    def _publish_image(self, publisher, image, timestamp):
        """Hilfsfunktion zum Komprimieren und Publishen eines Bildes."""
        if image is None: return
        try:
             if image.ndim == 2:
                  image_bgr = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
             else:
                  image_bgr = image

             ret, buffer = cv2.imencode('.jpg', image_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
             if not ret: return

             msg = CompressedImage(format="jpeg", data=buffer.tobytes())
             msg.header.stamp = timestamp
             publisher.publish(msg)
        except CvBridgeError as e:
            pass
            #   self.get_logger().error(f"CvBridge Fehler beim Publishen auf '{publisher.topic}': {e}", throttle_duration_sec=5)
        except Exception as e:
            pass
            #   self.get_logger().error(f"Allgemeiner Fehler beim Publishen auf '{publisher.topic}': {e}", throttle_duration_sec=5)

    def image_callback(self, msg):
        try:
            # Parameter holen
            publish_mask_flag = self.get_parameter('publish_mask').value
            publish_overlay_flag = self.get_parameter('publish_overlay').value
            roi_crop_factor = self.get_parameter('roi_crop_factor_h').value

            # Bild dekodieren
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is None: return
            timestamp = msg.header.stamp

            # ROI anwenden
            h, w, _ = frame.shape
            frame_cropped = frame[0:int(h * roi_crop_factor), :]

            # Farbe erkennen (verwendet die oben deklarierten Parameter)
            detected, filtered_mask = self.detect_target_color(frame_cropped)

            # Debug-Bilder publishen (falls aktiviert)
            if publish_mask_flag:
                self._publish_image(self.pub_mask, filtered_mask, timestamp)

            if publish_overlay_flag:
                if frame_cropped.size > 0:
                    overlay = frame_cropped.copy()
                    overlay[filtered_mask > 0] = (0, 255, 0) # Grün für erkannte Bereiche
                    self._publish_image(self.pub_overlay, overlay, timestamp)

            # Status publizieren: False WENN Farbe erkannt wird (Stop)
            self.publisher_.publish(Bool(data=not detected))

        except Exception as e:
            pass
            #  self.get_logger().error(f"ERROR in image_callback: {e}", throttle_duration_sec=10)
            #  self.get_logger().error(traceback.format_exc())

    def detect_target_color(self, frame):
        """Erkennt Rot robust über zwei HSV-Bereiche und filtert nach Blob-Größe."""
        detected = False
        if frame is None or frame.shape[0] == 0 or frame.shape[1] == 0:
             return detected, np.array([[]], dtype=np.uint8)

        filtered_mask = np.zeros((frame.shape[0], frame.shape[1]), dtype=np.uint8)

        try:
            # Parameter abrufen
            # HIER: Passe die Parameter für Rot an! Siehe Vorschläge unten.
            # Bereich 1 (niedrige H-Werte, z.B. 0-10)
            h_l1 = self.get_parameter('hsv_lower_h1').value # NEUER PARAMETER
            s_l = self.get_parameter('hsv_lower_s').value
            v_l = self.get_parameter('hsv_lower_v').value
            h_u1 = self.get_parameter('hsv_upper_h1').value # NEUER PARAMETER
            s_u = self.get_parameter('hsv_upper_s').value
            v_u = self.get_parameter('hsv_upper_v').value
            # Bereich 2 (hohe H-Werte, z.B. 170-180)
            h_l2 = self.get_parameter('hsv_lower_h2').value # NEUER PARAMETER
            # s_l, v_l, s_u, v_u sind gleich für beide Bereiche
            h_u2 = self.get_parameter('hsv_upper_h2').value # NEUER PARAMETER

            min_blob_area = self.get_parameter('min_blob_area').value
            max_blob_area = self.get_parameter('max_blob_area').value

            # Optional: Preprocessing
            # frame_blurred = cv2.GaussianBlur(frame, (5, 5), 0)
            # hsv = cv2.cvtColor(frame_blurred, cv2.COLOR_BGR2HSV)
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # Erstelle Masken für beide Rot-Bereiche
            lower1 = np.array([h_l1, s_l, v_l])
            upper1 = np.array([h_u1, s_u, v_u])
            mask1 = cv2.inRange(hsv, lower1, upper1)

            lower2 = np.array([h_l2, s_l, v_l])
            upper2 = np.array([h_u2, s_u, v_u])
            mask2 = cv2.inRange(hsv, lower2, upper2)

            # Kombiniere beide Masken
            mask = cv2.bitwise_or(mask1, mask2)

            # Morphologische Operationen (Kernelgröße kann angepasst werden)
            kernel_size = 3 # oder 5
            kernel = np.ones((kernel_size, kernel_size), np.uint8)
            mask_opened = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
            mask_closed = cv2.morphologyEx(mask_opened, cv2.MORPH_CLOSE, kernel, iterations=1)

            # Finale Maske für Blob-Erkennung
            final_mask = mask_closed

            # Blobs extrahieren aus der gesäuberten Maske
            num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(final_mask, connectivity=8)

            if num_labels > 1:
                for label in range(1, num_labels):
                    area = stats[label, cv2.CC_STAT_AREA]
                    # Verwende min UND max area filter
                    if area >= min_blob_area and area <= max_blob_area:
                        filtered_mask[labels == label] = 255
                        detected = True
                        # break # Nicht breaken, um alle gültigen Blobs in filtered_mask zu haben

        except cv2.error as cv_err:
             pass
            #  self.get_logger().error(f"OpenCV error in detect_target_color: {cv_err}", throttle_duration_sec=10)
        except Exception as e:
            pass
            #  self.get_logger().error(f"Error in detect_target_color: {e}", throttle_duration_sec=10)
            #  self.get_logger().error(traceback.format_exc())

        # Gib die *gefilterte* Maske zurück (nur Blobs der richtigen Größe)
        # oder final_mask, wenn du alle erkannten roten Bereiche vor dem Größenfilter sehen willst.
        return detected, filtered_mask

    def destroy_node(self):
        # self.get_logger().info("Shutting down Traffic Light Detection Node.")
        super().destroy_node()

# main Funktion bleibt wie in deiner "alten" Datei
def main(args=None):
    rclpy.init(args=args)
    node = None
    node_name_for_log = 'TrafficLightDetector'
    try:
        node = TrafficLightDetector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        pass
        # Logge Fehler, bevor Node zerstört wird
        #  if node: node.get_logger().error(f"FATAL ERROR [{node_name_for_log}] in main: {e}\n{traceback.format_exc()}")
        #  else: print(f"FATAL ERROR [{node_name_for_log}] vor Node-Init: {e}\n{traceback.format_exc()}", file=sys.stderr)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        try: cv2.destroyAllWindows() # Sicherstellen, dass alle Fenster geschlossen werden
        except Exception: pass

if __name__ == '__main__':
    main()
# waymo/traffic_light_detection_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import cv2
# import cv_bridge # Nicht verwendet
import numpy as np
import sys
import traceback
import time

# --- NEU: Imports für Parameter ---
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult, ParameterType, IntegerRange, FloatingPointRange
from rclpy.parameter import Parameter
# --- Ende NEU ---

# --- Klasse und Node-Name konsistent gemacht ---
class TrafficLightDetector(Node):
    def __init__(self):
        super().__init__('traffic_light_detection_node') # Node-Name angepasst

        # --- QoS Profile ---
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=1
        )
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1
        )

        # --- NEU: Parameter Descriptors ---
        def int_desc(desc, min_val=0, max_val=255, step=1):
            return ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER, description=desc,
                integer_range=[IntegerRange(from_value=min_val, to_value=max_val, step=step)])
        def int_area_desc(desc, min_val=0, max_val=5000, step=1):
             return ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER, description=desc,
                integer_range=[IntegerRange(from_value=min_val, to_value=max_val, step=step)])
        # --- Ende NEU ---

        # --- NEU: Parameter Deklarationen HIER in __init__ ---
        # Standardwerte für dunkles Magenta (#610A1F), ca. H=173, S=229, V=97
        # Hinweis: Da H nahe 180 liegt, könnte ein zweiter Bereich um H=0 nötig sein,
        # falls die Farbe im Bild leicht variiert. Vorerst nur ein Bereich.
        self.declare_parameter('hsv_lower_h', 160, int_desc("Lower Hue (0-180)", max_val=180))
        self.declare_parameter('hsv_lower_s', 0, int_desc("Lower Saturation (0-255)"))
        self.declare_parameter('hsv_lower_v', 0, int_desc("Lower Value (0-255)"))
        self.declare_parameter('hsv_upper_h', 180, int_desc("Upper Hue (0-180)", max_val=180))
        self.declare_parameter('hsv_upper_s', 255, int_desc("Upper Saturation (0-255)"))
        self.declare_parameter('hsv_upper_v', 255, int_desc("Upper Value (0-255)"))

        self.declare_parameter('min_blob_area', 100, int_area_desc("Minimum Blob Area (pixels)"))
        self.declare_parameter('roi_crop_factor_h', 0.5, ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE, description="ROI Crop Factor Height (0.1-1.0)", floating_point_range=[FloatingPointRange(from_value=0.1, to_value=1.0, step=0.05)]))

        # --- Ende NEU ---

        # Subscriber und Publisher
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.image_callback,
            qos_sensor)

        self.publisher_ = self.create_publisher(Bool, 'traffic_light', qos_reliable)

        self.show_debug_windows = False # Auf True setzen für Debug-Fenster
        if self.show_debug_windows:
             try:
                 cv2.namedWindow('TrafficLight Mask', cv2.WINDOW_NORMAL)
                 cv2.namedWindow('TrafficLight Overlay', cv2.WINDOW_NORMAL)
             except Exception as e:
                  print(f"WARNUNG [{self.get_name()}]: Konnte Debug-Fenster nicht erstellen: {e}", file=sys.stderr)
                  self.show_debug_windows = False

    def image_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is None: return

            h, w, _ = frame.shape
            # ROI aus Parameter holen
            roi_crop_factor = self.get_parameter('roi_crop_factor_h').value
            frame_cropped = frame[0:int(h * roi_crop_factor), :]

            # Farbe erkennen (nutzt jetzt Parameter intern)
            detected, filtered_mask = self.detect_target_color(frame_cropped)

            if self.show_debug_windows:
                try:
                    cv2.imshow('TrafficLight Mask', filtered_mask)
                    if frame_cropped.size > 0:
                         overlay = frame_cropped.copy()
                         overlay[filtered_mask > 0] = (0, 255, 0) # Grün Overlay
                         cv2.imshow('TrafficLight Overlay', overlay)
                    cv2.waitKey(1)
                except Exception as e:
                     print(f"ERROR [{self.get_name()}] displaying debug windows: {e}", file=sys.stderr)

            # Status publizieren: False WENN Farbe erkannt wird (Stop)
            self.publisher_.publish(Bool(data=not detected))

        except Exception as e:
             print(f"ERROR [{self.get_name()}] in image_callback: {e}", file=sys.stderr)
             traceback.print_exc(file=sys.stderr)

    def detect_target_color(self, frame):
        """Erkennt die konfigurierte Zielfarbe und filtert nach Blob-Größe."""
        detected = False
        if frame is None or frame.shape[0] == 0 or frame.shape[1] == 0:
             return detected, np.array([[]], dtype=np.uint8)

        filtered_mask = np.zeros((frame.shape[0], frame.shape[1]), dtype=np.uint8)

        try:
            # --- Parameter HIER abrufen ---
            h_l = self.get_parameter('hsv_lower_h').value
            s_l = self.get_parameter('hsv_lower_s').value
            v_l = self.get_parameter('hsv_lower_v').value
            h_u = self.get_parameter('hsv_upper_h').value
            s_u = self.get_parameter('hsv_upper_s').value
            v_u = self.get_parameter('hsv_upper_v').value
            min_blob_area = self.get_parameter('min_blob_area').value
            # --- ---

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # --- HSV-Bereich aus Parametern ---
            lower = np.array([h_l, s_l, v_l])
            upper = np.array([h_u, s_u, v_u])
            # --- ---

            mask = cv2.inRange(hsv, lower, upper)

            # Morphologische Operationen (optional, könnten Parameter sein)
            # kernel = np.ones((3,3),np.uint8)
            # mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
            # mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)

            # Blobs extrahieren
            num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)

            if num_labels > 1:
                for label in range(1, num_labels):
                    area = stats[label, cv2.CC_STAT_AREA]
                    if area >= min_blob_area:
                        filtered_mask[labels == label] = 255
                        detected = True
                        # break # Frühzeitig beenden?

        except cv2.error as cv_err:
             print(f"ERROR [{self.get_name()}] OpenCV error in detect_target_color: {cv_err}", file=sys.stderr)
        except Exception as e:
             print(f"ERROR [{self.get_name()}] in detect_target_color: {e}", file=sys.stderr)
             traceback.print_exc(file=sys.stderr)

        return detected, filtered_mask

    def destroy_node(self):
        if self.show_debug_windows:
            try: cv2.destroyAllWindows()
            except Exception: pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = None
    # --- Node-Name für Log angepasst ---
    node_name_for_log = 'TrafficLightDetector'
    try:
        # --- Klassenname angepasst ---
        node = TrafficLightDetector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
         print(f"FATAL ERROR [{node_name_for_log}] in main: {e}", file=sys.stderr)
         traceback.print_exc(file=sys.stderr)
    finally:
        if node is not None:
            should_destroy_cv_windows = False
            if hasattr(node, 'show_debug_windows') and node.show_debug_windows:
                 should_destroy_cv_windows = True
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if should_destroy_cv_windows:
             try: cv2.destroyAllWindows()
             except Exception: pass

if __name__ == '__main__':
    main()
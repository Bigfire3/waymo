#!/usr/bin/env python3
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float64
import sys
import traceback
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult, ParameterType, IntegerRange, FloatingPointRange
from rclpy.parameter import Parameter

import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError # CvBridge hinzugefügt
from . import lane

class LaneDetectionNode(Node):
    def __init__(self):
        super().__init__('lane_detection_node')
        self.bridge = CvBridge() # CvBridge Instanz

        # --- Parameter Descriptors ---
        # Behalte die Helferfunktionen bei, die wir vorher definiert haben
        def int_desc(desc, min_val=0, max_val=255, step=1): return ParameterDescriptor(
            type=ParameterType.PARAMETER_INTEGER, description=desc,
            integer_range=[IntegerRange(from_value=min_val, to_value=max_val, step=step)])

        def float_desc(desc, min_val=0.0, max_val=1.0, step=0.001): return ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE, description=desc,
            floating_point_range=[FloatingPointRange(from_value=min_val, to_value=max_val, step=step)])

        def bool_desc(desc): return ParameterDescriptor(
             type=ParameterType.PARAMETER_BOOL, description=desc)

        # --- Parameter Deklarationen (basierend auf deiner "alten" Datei) ---
        self.declare_parameter('block_size', 11, int_desc("Auto-S: block_size"))
        self.declare_parameter('c_value', 20, int_desc("Auto-S: c_value"))
        self.declare_parameter('center_factor', 0.03, float_desc("Center_Calc: factor")) # Schrittweite 0.001
        # Beachte: min/max thickness hatten step=0.001, daher keine Rundung nötig für Standardwerte 2.5 und 5.0
        self.declare_parameter('min_thickness', 2.5, float_desc("Minimum thickness ratio", min_val=0.01, max_val=200.0, step=0.001))
        self.declare_parameter('max_thickness', 5.0, float_desc("Maximum thickness ratio", min_val=0.01, max_val=200.0, step=0.001))
        # ROI Parameter
        self.declare_parameter('roi_top_left_w', 0.1, float_desc("ROI TL Breite")) # Schrittweite 0.001
        self.declare_parameter('roi_top_left_h', 0.65, float_desc("ROI TL Höhe")) # Schrittweite 0.001
        self.declare_parameter('roi_top_right_w', 0.9, float_desc("ROI TR Breite")) # Schrittweite 0.001
        self.declare_parameter('roi_top_right_h', 0.65, float_desc("ROI TR Höhe")) # Schrittweite 0.001
        self.declare_parameter('roi_bottom_left_w', 0.0, float_desc("ROI BL Breite")) # Schrittweite 0.001
        self.declare_parameter('roi_bottom_left_h', 1.0, float_desc("ROI BL Höhe")) # Schrittweite 0.001
        self.declare_parameter('roi_bottom_right_w', 1.0, float_desc("ROI BR Breite")) # Schrittweite 0.001
        self.declare_parameter('roi_bottom_right_h', 1.0, float_desc("ROI BR Höhe")) # Schrittweite 0.001
        # desired_roi_padding_factor hat step=0.01, Standardwert 0.25 ist gültig
        self.declare_parameter('desired_roi_padding_factor', 0.25, float_desc("Ziel ROI Padding", max_val=0.4, step=0.01))
        self.declare_parameter('min_compactness', 0.8, float_desc("min_compactness"))

        # --- HINZUGEFÜGT: Parameter zum Steuern der Debug-Publisher ---
        self.declare_parameter('publish_lane_annotated', True, bool_desc("Publish final annotated lane image"))
        self.declare_parameter('publish_raw_markings', True, bool_desc("Publish raw detected line markings image"))
        self.declare_parameter('publish_warped_frame', True, bool_desc("Publish warped perspective image"))
        self.declare_parameter('publish_filtered_warped', True, bool_desc("Publish thickness-filtered warped image"))
        self.declare_parameter('publish_sliding_window', True, bool_desc("Publish sliding window search visualization"))
        self.declare_parameter('publish_roi_image', True, bool_desc("Publish ROI visualization image"))
        # --- Ende HINZUGEFÜGT ---


        # --- QoS und Subscriber/Publisher ---
        qos_sensor_data = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        qos_debug_images = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        qos_control_data = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1) # Offset ist Steuergröße


        self.subscription = self.create_subscription(CompressedImage, '/image_raw/compressed', self.image_callback, qos_sensor_data)

        # Publisher für Offset (Steuergröße)
        self.driving_publisher_ = self.create_publisher(Float64, '/lane/center_offset', qos_control_data)

        # --- Debug Publisher (unverändert) ---
        self.pub_lane_annotated = self.create_publisher(CompressedImage, '/debug/cam/lane_annotated', qos_debug_images)
        self.pub_raw_markings = self.create_publisher(CompressedImage, '/debug/cam/raw_markings', qos_debug_images)
        self.pub_warped_frame = self.create_publisher(CompressedImage, '/debug/cam/warped', qos_debug_images)
        self.pub_filtered_warped = self.create_publisher(CompressedImage, '/debug/cam/filtered_warped', qos_debug_images)
        self.pub_sliding_window = self.create_publisher(CompressedImage, '/debug/cam/sliding_window', qos_debug_images)
        self.pub_roi = self.create_publisher(CompressedImage, '/debug/cam/roi', qos_debug_images)
        # Erstelle einmal das Lane-Objekt
        self.lane_obj = lane.Lane()

    def _publish_image(self, publisher, image, timestamp):
        """Hilfsfunktion zum Komprimieren und Publishen eines Bildes."""
        if image is None:
            return
        try:
             if image.ndim == 2:
                  image_bgr = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
             else:
                  image_bgr = image

             ret, buffer = cv2.imencode('.jpg', image_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
             if not ret:
                  return

             msg = CompressedImage(format="jpeg", data=buffer.tobytes())
             msg.header.stamp = timestamp
             publisher.publish(msg)
        except CvBridgeError as e:
              self.get_logger().error(f"CvBridge Fehler beim Publishen auf '{publisher.topic}': {e}", throttle_duration_sec=5)
        except Exception as e:
              self.get_logger().error(f"Allgemeiner Fehler beim Publishen auf '{publisher.topic}': {e}", throttle_duration_sec=5)

    def image_callback(self, msg: CompressedImage):
        try:
            # --- Parameter sammeln ---
            # Hole *alle* deklarierten Parameter, inklusive der neuen publish_* Flags
            current_params = {param_name: self.get_parameter(param_name).value for param_name in self._parameters}
            # Korrigiere boolsche Parameter (werden manchmal als String gelesen?)
            for key in ['publish_lane_annotated', 'publish_raw_markings', 'publish_warped_frame', 'publish_filtered_warped', 'publish_sliding_window']:
                 if isinstance(current_params.get(key), str):
                      current_params[key] = current_params[key].lower() == 'true'

            # --- Bild dekodieren ---
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is None: return
            timestamp = msg.header.stamp

            # --- Bildverarbeitung mit dem Lane-Objekt ---
            self.lane_obj.update_frame(frame, **current_params)

            roi_img = self.lane_obj.plot_roi() # Ruft die geänderte Methode auf
            if current_params.get('publish_roi_image') and roi_img is not None:
                self._publish_image(self.pub_roi, roi_img, timestamp)

            # 1. Linienmarkierungen finden (Verwendet block_size, c_value etc.)
            raw_markings = self.lane_obj.get_line_markings(**current_params)
            if current_params.get('publish_raw_markings'):
                self._publish_image(self.pub_raw_markings, raw_markings, timestamp)

            # 2. Perspektivtransformation (Verwendet ROI-Parameter)
            self.lane_obj.perspective_transform(frame=raw_markings, plot=False)
            if current_params.get('publish_warped_frame'):
                self._publish_image(self.pub_warped_frame, self.lane_obj.warped_frame, timestamp)

            # 3. Nach Dicke filtern (Verwendet min/max_thickness)
            self.lane_obj.filter_lane_markings_by_thickness(plot=False, **current_params)
            if current_params.get('publish_filtered_warped'):
                 self._publish_image(self.pub_filtered_warped, self.lane_obj.filtered_warped_frame, timestamp)

            # 4. Histogramm berechnen
            self.histogram = self.lane_obj.calculate_histogram(plot=False)

            # 5. Sliding Windows / Fit finden (Verwendet Parameter aus lane.py, die indirekt von hier kommen könnten)
            left_fit, right_fit, sliding_window_img = self.lane_obj.get_lane_line_indices_sliding_windows(
                plot=True, **current_params) # plot=False, da Bild zurückkommt
            if current_params.get('publish_sliding_window') and sliding_window_img is not None:
                self._publish_image(self.pub_sliding_window, sliding_window_img, timestamp)

            # 6. Fits verfeinern / Fallback
            if left_fit is None or right_fit is None:
                  self.lane_obj.get_lane_line_previous_window(self.lane_obj.previous_left_fit, self.lane_obj.previous_right_fit, plot=False)

            # 7. Overlay, Krümmung und Offset berechnen
            final_annotated_frame = frame
            if self.lane_obj.left_fit is not None and self.lane_obj.right_fit is not None:
                overlay_frame = self.lane_obj.overlay_lane_lines(plot=False)
                # calculate_curvature/position verwenden jetzt ym/xm_per_pix aus lane.py oder params
                self.lane_obj.calculate_curvature(print_to_terminal=False, **current_params)
                # calculate_car_position verwendet center_factor
                self.lane_obj.calculate_car_position(print_to_terminal=False, **current_params)
                final_annotated_frame = self.lane_obj.display_curvature_offset(frame=overlay_frame, plot=False)
            else:
                 self.lane_obj.center_offset = 0.0 # Sicherstellen, dass Offset 0 ist, wenn keine Linie

            # 8. Finales annotiertes Bild publishen (falls aktiviert)
            if current_params.get('publish_lane_annotated'):
                 self._publish_image(self.pub_lane_annotated, final_annotated_frame, timestamp)

            # --- Offset publishen ---
            offset_msg = Float64()
            offset_msg.data = float(self.lane_obj.center_offset) if self.lane_obj.center_offset is not None else 0.0
            self.driving_publisher_.publish(offset_msg)

        except CvBridgeError as e:
             self.get_logger().error(f"CvBridge Error in image_callback: {e}", throttle_duration_sec=10)
        except Exception as e:
             self.get_logger().error(f"Error processing image: {e}", throttle_duration_sec=10)
             self.get_logger().error(traceback.format_exc())


    def destroy_node(self):
        self.get_logger().info("Shutting down Lane Detection Node.")
        try: cv2.destroyAllWindows()
        except Exception: pass
        super().destroy_node()

# main Funktion bleibt wie in deiner "neuen" Datei, da sie den Fehler-Fix enthielt
def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LaneDetectionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node: pass
        else: print("KeyboardInterrupt received before node init.", file=sys.stderr)
    except Exception as e:
        # Logge Fehler, bevor Node zerstört wird
        if node: node.get_logger().error(f"FATALER FEHLER in LaneDetectionNode: {e}\n{traceback.format_exc()}")
        else: print(f"FATALER FEHLER vor Node-Init: {e}\n{traceback.format_exc()}", file=sys.stderr)
    finally:
        if node and isinstance(node, Node):
            if rclpy.ok():
                 node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
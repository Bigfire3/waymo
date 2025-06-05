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
from . import edge_detection

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
        self.declare_parameter('block_size', 7, int_desc("Auto-S: block_size"))
        self.declare_parameter('c_value', 200, int_desc("Auto-S: c_value"))
        self.declare_parameter('center_factor', 0.015, float_desc("Center_Calc: factor")) # Schrittweite 0.001
        # Beachte: min/max thickness hatten step=0.001, daher keine Rundung nötig für Standardwerte 2.5 und 5.0
        self.declare_parameter('min_thickness', 1.5, float_desc("Minimum thickness ratio", min_val=0.01, max_val=200.0, step=0.001))
        self.declare_parameter('max_thickness', 5.0, float_desc("Maximum thickness ratio", min_val=0.01, max_val=200.0, step=0.001))
        # ROI Parameter
        self.declare_parameter('roi_top_left_w', 0.1, float_desc("ROI TL Breite")) # Schrittweite 0.001
        self.declare_parameter('roi_top_left_h', 0.7, float_desc("ROI TL Höhe")) # Schrittweite 0.001
        self.declare_parameter('roi_top_right_w', 0.9, float_desc("ROI TR Breite")) # Schrittweite 0.001
        self.declare_parameter('roi_top_right_h', 0.7, float_desc("ROI TR Höhe")) # Schrittweite 0.001
        self.declare_parameter('roi_bottom_left_w', 0.0, float_desc("ROI BL Breite")) # Schrittweite 0.001
        self.declare_parameter('roi_bottom_left_h', 1.0, float_desc("ROI BL Höhe")) # Schrittweite 0.001
        self.declare_parameter('roi_bottom_right_w', 1.0, float_desc("ROI BR Breite")) # Schrittweite 0.001
        self.declare_parameter('roi_bottom_right_h', 1.0, float_desc("ROI BR Höhe")) # Schrittweite 0.001
        # desired_roi_padding_factor hat step=0.01, Standardwert 0.25 ist gültig
        self.declare_parameter('desired_roi_padding_factor', 0.25, float_desc("Ziel ROI Padding", max_val=0.4, step=0.01))
        
        self.declare_parameter('binary_border', 150, int_desc("binary_border"))
        self.declare_parameter('area', 30, int_desc("area"))
        self.declare_parameter('aspect_ratio', 2.0, float_desc("aspect_ratio", min_val=0.1, max_val=10.0, step=0.1))
        self.declare_parameter('compactness', 0.015, float_desc("compactness", min_val=0.001, max_val=0.2, step=0.001))

        self.declare_parameter('sliding_window_margin_factor', 0.075, float_desc("sliding_window_margin_factor"))
        self.declare_parameter('sliding_window_minpix', 50, float_desc("sliding_window_minpix"))

        # Parameter für edge_detection
        self.declare_parameter('threshold_1', 100, int_desc("threshold_1"))
        self.declare_parameter('threshold_2', 200, int_desc("threshold_2"))

        self.declare_parameter('ksize', 3, int_desc("ksize"))
        self.declare_parameter('iterations', 1, int_desc("iterations"))

        self.declare_parameter('d_value', 5, int_desc("d_value"))
        self.declare_parameter('sigmaColor', 75, int_desc("sigmaColor"))
        self.declare_parameter('sigmaSpace', 75, int_desc("sigmaSpace"))

        self.declare_parameter('min_contour_area', 50, int_desc("min_contour_area", max_val=10000))
        self.declare_parameter('max_contour_area', 1500, int_desc("max_contour_area", max_val=100000))

        # --- HINZUGEFÜGT: Parameter zum Steuern der Debug-Publisher ---
        self.declare_parameter('publish_roi_image', True, bool_desc("Publish ROI visualization image"))
        self.declare_parameter('publish_lane_annotated', True, bool_desc("Publish final annotated lane image"))
        self.declare_parameter('publish_closed_edges', True, bool_desc("Publish closed edges image"))
        self.declare_parameter('publish_filled_areas', True, bool_desc("Publish filled areas image"))
        self.declare_parameter('publish_thickness_filtered', True, bool_desc("Publish thickness-filtered warped image"))
        self.declare_parameter('publish_sliding_window', True, bool_desc("Publish sliding window search visualization"))
        # --- Ende HINZUGEFÜGT ---


        # --- QoS und Subscriber/Publisher ---
        qos_sensor_data = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        qos_debug_images = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        qos_control_data = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1) # Offset ist Steuergröße


        self.subscription = self.create_subscription(CompressedImage, '/image_raw/compressed', self.image_callback, qos_sensor_data)

        # Publisher für Offset (Steuergröße)
        self.driving_publisher_ = self.create_publisher(Float64, '/lane/center_offset', qos_control_data)

        # --- Debug Publisher (unverändert) ---
        self.pub_roi = self.create_publisher(CompressedImage, '/debug/cam/roi', qos_debug_images)
        self.pub_lane_annotated = self.create_publisher(CompressedImage, '/debug/cam/lane_annotated', qos_debug_images)
        self.pub_closed_edges = self.create_publisher(CompressedImage, '/debug/cam/closed_edges', qos_debug_images)
        self.pub_filled_areas = self.create_publisher(CompressedImage, '/debug/cam/filled_areas', qos_debug_images)
        self.pub_thickness_filtered = self.create_publisher(CompressedImage, '/debug/cam/thickness_filtered', qos_debug_images)
        self.pub_sliding_window = self.create_publisher(CompressedImage, '/debug/cam/sliding_window', qos_debug_images)

        # Erstelle Instanzen von Lane und EdgeDetection
        self.lane_obj = lane.Lane()
        # EdgeDetection initialisieren, der Frame wird später im Callback übergeben
        self.edge_obj = None

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
            # --- Bild dekodieren ---
            # Die empfangene Nachricht enthält die komprimierten Daten im 'data'-Feld
            np_arr = np.frombuffer(msg.data, np.uint8)
            self.frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            self.gray_scale_frame = cv2.imdecode(np_arr, cv2.IMREAD_GRAYSCALE)

            if self.frame is None or self.gray_scale_frame is None:
                self.get_logger().warn("Dekodiertes Bild ist leer oder ungültig. Überspringe Bildverarbeitung.", throttle_duration_sec=1)
                return

            # Timestamp der empfangenen Nachricht
            timestamp = msg.header.stamp

            # --- Parameter sammeln ---
            # Hole *alle* deklarierten Parameter, inklusive der neuen publish_* Flags
            current_params = {param_name: self.get_parameter(param_name).value for param_name in self._parameters}
            # Korrigiere boolsche Parameter (werden manchmal als String gelesen?)
            for key in ['publish_lane_annotated', 'publish_filled_areas', 'publish_warped_frame', 'publish_thickness_filtered', 'publish_sliding_window']:
                 if isinstance(current_params.get(key), str):
                      current_params[key] = current_params[key].lower() == 'true'

            # --- Bildverarbeitung mit dem Lane-Objekt ---
            self.lane_obj.update_frame(self.frame, **current_params)

            roi_img = self.lane_obj.plot_roi()
            if current_params.get('publish_roi_image') and roi_img is not None:
                self._publish_image(self.pub_roi, roi_img, timestamp)

            # 1. Perspektivtransformation (Verwendet ROI-Parameter)
            warped_frame = self.lane_obj.perspective_transform(frame=self.gray_scale_frame, plot=False)

            # Aktualisiere den Frame für das EdgeDetection Objekt
            self.edge_obj = edge_detection.EdgeDetection(warped_frame)

            # 2. Closed Edges finden
            closed_edges_frame = self.edge_obj.get_closed_edges(**current_params)
            if current_params.get('publish_closed_edges'):
                self._publish_image(self.pub_closed_edges, closed_edges_frame, timestamp)

            # 3. Füllung der Flächen
            filled_areas_frame = self.edge_obj.get_filled_areas(**current_params)
            if current_params.get('publish_filled_areas'):
                self._publish_image(self.pub_filled_areas, filled_areas_frame, timestamp)

            # 3. Nach Dicke filtern (Verwendet min/max_thickness)
            self.lane_obj.filter_lane_markings_by_thickness(filled_areas_frame, plot=False, **current_params)
            if current_params.get('publish_thickness_filtered'):
                 self._publish_image(self.pub_thickness_filtered, self.lane_obj.filtered_warped_frame, timestamp)

            # 4. Histogramm berechnen
            self.histogram = self.lane_obj.calculate_histogram(plot=False)

            # 5. Sliding Windows / Fit finden (Verwendet Parameter aus lane.py, die indirekt von hier kommen könnten)
            left_fit, right_fit, sliding_window_img = self.lane_obj.get_lane_line_indices_sliding_windows(plot=True) # plot=False, da Bild zurückkommt
            if current_params.get('publish_sliding_window') and sliding_window_img is not None:
                self._publish_image(self.pub_sliding_window, sliding_window_img, timestamp)

            # 6. Fits verfeinern / Fallback
            if left_fit is None or right_fit is None:
                  self.lane_obj.get_lane_line_previous_window(self.lane_obj.previous_left_fit, self.lane_obj.previous_right_fit, plot=False)

            # 7. Overlay, Krümmung und Offset berechnen
            final_annotated_frame = self.frame
            if self.lane_obj.left_fit is not None and self.lane_obj.right_fit is not None:
                overlay_frame = self.lane_obj.overlay_lane_lines(plot=False)
                # calculate_curvature/position verwenden jetzt ym/xm_per_pix aus lane.py oder params
                self.lane_obj.calculate_curvature(print_to_terminal=False, **current_params)
                # calculate_car_position verwendet center_factor
                self.lane_obj.calculate_car_position(print_to_terminal=False, **current_params)
                final_annotated_frame = self.lane_obj.display_curvature_offset(frame=overlay_frame, plot=False)

            # 8. Finales annotiertes Bild publishen (falls aktiviert)
            if current_params.get('publish_lane_annotated'):
                 self._publish_image(self.pub_lane_annotated, final_annotated_frame, timestamp)

            # --- Offset publishen ---
            offset_msg = Float64()
            if self.lane_obj.center_offset is not None:
                offset_msg.data = float(self.lane_obj.center_offset)
                self.lane_obj.prev_center_offset = self.lane_obj.center_offset
            else:
                offset_msg.data = float(self.lane_obj.prev_center_offset)
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
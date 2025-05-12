#!/usr/bin/env python3
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float64
import sys
import traceback
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult, ParameterType, IntegerRange, FloatingPointRange, ParameterValue
from rclpy.parameter import Parameter

import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from . import lane # Dein Original-Import
from . import edge_detection # Importiere deine edge_detection.py

class LaneDetectionNode(Node):
    def __init__(self):
        super().__init__('lane_detection_node')
        self.bridge = CvBridge()

        def int_desc(desc, min_val=0, max_val=255, step=1): return ParameterDescriptor(
            type=ParameterType.PARAMETER_INTEGER, description=desc,
            integer_range=[IntegerRange(from_value=min_val, to_value=max_val, step=step)])

        def float_desc(desc, min_val=0.0, max_val=1.0, step=0.001): return ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE, description=desc,
            floating_point_range=[FloatingPointRange(from_value=min_val, to_value=max_val, step=step)])

        def bool_desc(desc): return ParameterDescriptor(
             type=ParameterType.PARAMETER_BOOL, description=desc)
        
        # *** NEU: String Descriptor ***
        def string_desc(desc): return ParameterDescriptor(
             type=ParameterType.PARAMETER_STRING, description=desc)

        # --- Deine bestehenden Parameter ---
        self.declare_parameter('block_size', 11, int_desc("Auto-S: block_size"))
        self.declare_parameter('c_value', 20, int_desc("Auto-S: c_value"))
        self.declare_parameter('center_factor', 0.03, float_desc("Center_Calc: factor"))
        self.declare_parameter('min_thickness', 2.5, ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE, description="Minimum thickness ratio", floating_point_range=[FloatingPointRange(from_value=0.01, to_value=200.0, step=0.001)]))
        self.declare_parameter('max_thickness', 5.0, ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE, description="Maximum thickness ratio", floating_point_range=[FloatingPointRange(from_value=0.01, to_value=200.0, step=0.001)]))
        self.declare_parameter('roi_top_left_w', 0.1, float_desc("ROI TL Breite"))
        self.declare_parameter('roi_top_left_h', 0.65, float_desc("ROI TL Höhe"))
        self.declare_parameter('roi_top_right_w', 0.9, float_desc("ROI TR Breite"))
        self.declare_parameter('roi_top_right_h', 0.65, float_desc("ROI TR Höhe"))
        self.declare_parameter('roi_bottom_left_w', 0.0, float_desc("ROI BL Breite"))
        self.declare_parameter('roi_bottom_left_h', 1.0, float_desc("ROI BL Höhe"))
        self.declare_parameter('roi_bottom_right_w', 1.0, float_desc("ROI BR Breite"))
        self.declare_parameter('roi_bottom_right_h', 1.0, float_desc("ROI BR Höhe"))
        self.declare_parameter('desired_roi_padding_factor', 0.25, float_desc("Ziel ROI Padding", max_val=0.4, step=0.01))
        self.declare_parameter('min_compactness', 0.8, float_desc("min_compactness"))


        # --- Parameter für Kantenerkennung (Canny) ---
        self.declare_parameter('edge_blur_ksize', 5, int_desc("Canny Edge: Gaussian blur kernel size (odd)", min_val=1, max_val=100))
        self.declare_parameter('edge_canny_low', 50, int_desc("Canny Edge: Canny lower threshold", min_val=0, max_val=255))
        self.declare_parameter('edge_canny_high', 150, int_desc("Canny Edge: Canny upper threshold", min_val=0, max_val=255))

        # --- Parameter für Morphologische Operationen (NEU) ---
        self.declare_parameter('edge_morph_op', "NONE", string_desc("Morph Op after Canny (NONE, OPEN, CLOSE)"))
        self.declare_parameter('edge_morph_ksize', 3, int_desc("Morph Op: Kernel size (odd)", min_val=1, max_val=100))
        self.declare_parameter('edge_morph_iterations', 1, int_desc("Morph Op: Iterations", min_val=1, max_val=100))

        # --- Parameter für Konturfilterung (NEU) ---
        self.declare_parameter('edge_min_contour_area', 10, int_desc("Contour Filter: Min Area (pixels, 0=off)", min_val=0, max_val=1000))


        # --- Parameter zum Steuern der Debug-Publisher ---
        self.declare_parameter('publish_roi_image', True, bool_desc("Publish ROI visualization image"))
        self.declare_parameter('publish_lane_annotated', True, bool_desc("Publish final annotated lane image"))
        self.declare_parameter('publish_raw_markings', True, bool_desc("Publish raw detected line markings image (after color filter)"))
        self.declare_parameter('publish_edge_filled', True, bool_desc("Publish image after edge detection and filling"))
        self.declare_parameter('publish_warped_frame', True, bool_desc("Publish warped perspective image"))
        self.declare_parameter('publish_filtered_warped', True, bool_desc("Publish thickness-filtered warped image"))
        self.declare_parameter('publish_sliding_window', True, bool_desc("Publish sliding window search visualization"))


        qos_sensor_data = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        qos_debug_images = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        qos_control_data = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1)

        self.subscription = self.create_subscription(CompressedImage, '/image_raw/compressed', self.image_callback, qos_sensor_data)
        self.driving_publisher_ = self.create_publisher(Float64, '/lane/center_offset', qos_control_data)

        self.pub_roi = self.create_publisher(CompressedImage, '/debug/cam/roi', qos_debug_images)
        self.pub_lane_annotated = self.create_publisher(CompressedImage, '/debug/cam/lane_annotated', qos_debug_images)
        self.pub_raw_markings = self.create_publisher(CompressedImage, '/debug/cam/raw_markings', qos_debug_images)
        self.pub_edge_filled = self.create_publisher(CompressedImage, '/debug/cam/edge_filled', qos_debug_images)
        self.pub_warped_frame = self.create_publisher(CompressedImage, '/debug/cam/warped', qos_debug_images)
        self.pub_filtered_warped = self.create_publisher(CompressedImage, '/debug/cam/filtered_warped', qos_debug_images)
        self.pub_sliding_window = self.create_publisher(CompressedImage, '/debug/cam/sliding_window', qos_debug_images)
        
        self.lane_obj = lane.Lane()
        self.edge_detector_obj = edge_detection.EdgeDetection()

        # self.get_logger().info('Lane Detection Node (mit erweiterter Kantenfilterung) gestartet.')

    def _publish_image(self, publisher, image, timestamp):
        # Deine unveränderte _publish_image Methode
        if image is None: return
        try:
            if image.ndim == 2: image_bgr = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
            else: image_bgr = image
            ret, buffer = cv2.imencode('.jpg', image_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
            if not ret: self.get_logger().warn(f"imencode failed for {publisher.topic}"); return
            msg = CompressedImage(format="jpeg", data=buffer.tobytes())
            msg.header.stamp = timestamp
            publisher.publish(msg)
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Fehler ('{publisher.topic}'): {e}", throttle_duration_sec=5)
        except Exception as e:
            self.get_logger().error(f"Pub Fehler ('{publisher.topic}'): {e}\n{traceback.format_exc()}", throttle_duration_sec=5)


    def image_callback(self, msg: CompressedImage):
        try:
            current_params = {param_name: self.get_parameter(param_name).value for param_name in self._parameters}
            for key in self._parameters:
                param_obj = self.get_parameter(key)
                # Sicherstellen, dass boolesche Parameter wirklich bool sind
                if param_obj.type_ == ParameterType.PARAMETER_BOOL and not isinstance(current_params[key], bool):
                    # Versuche Konvertierung von String oder Int
                    if isinstance(current_params[key], str):
                        current_params[key] = current_params[key].lower() in ['true', '1', 'yes']
                    elif isinstance(current_params[key], int):
                         current_params[key] = bool(current_params[key])
                    else: # Fallback oder Fehler loggen
                         self.get_logger().warn(f"Konnte booleschen Parameter '{key}' nicht von Typ {type(current_params[key])} konvertieren.")
                         current_params[key] = True # Sicherer Standard?

            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is None: self.get_logger().warn("Decoded frame is None."); return
            timestamp = msg.header.stamp

            self.lane_obj.update_frame(frame, **current_params)

            # --- ROI Bild ---
            if current_params.get('publish_roi_image', False): # Default False bei Fehler
                roi_img = self.lane_obj.plot_roi()
                self._publish_image(self.pub_roi, roi_img, timestamp)

            # --- 1. Linienmarkierungen (Farbfilter) ---
            raw_markings_img = self.lane_obj.get_line_markings(**current_params)
            if current_params.get('publish_raw_markings', False) and raw_markings_img is not None:
                self._publish_image(self.pub_raw_markings, raw_markings_img, timestamp)

            # --- 2. Kantenerkennung (mit neuen Filtern) ---
            edge_filled_image = None
            if raw_markings_img is not None:
                # Parameter für Kantenerkennung holen
                edge_blur = current_params.get('edge_blur_ksize', 5)
                edge_low = current_params.get('edge_canny_low', 50)
                edge_high = current_params.get('edge_canny_high', 150)
                # NEUE Parameter holen
                morph_op = current_params.get('edge_morph_op', "NONE")
                morph_ksize = current_params.get('edge_morph_ksize', 3)
                morph_iter = current_params.get('edge_morph_iterations', 1)
                min_area = current_params.get('edge_min_contour_area', 10)

                edge_filled_image = self.edge_detector_obj.detect_and_fill_lanes(
                    raw_markings_img,
                    blur_ksize=edge_blur,
                    canny_low_thresh=edge_low,
                    canny_high_thresh=edge_high,
                    morph_op=morph_op,             # Übergeben
                    morph_ksize=morph_ksize,       # Übergeben
                    morph_iterations=morph_iter, # Übergeben
                    min_contour_area=min_area      # Übergeben
                )
            if current_params.get('publish_edge_filled', False) and edge_filled_image is not None:
                self._publish_image(self.pub_edge_filled, edge_filled_image, timestamp)

            # --- 3. Perspektivtransformation ---
            input_for_warp = edge_filled_image if edge_filled_image is not None else raw_markings_img
            if input_for_warp is None: # Zusätzlicher Check: Wenn beides None ist
                 self.get_logger().warn("Kein gültiges Bild für Perspective Transform vorhanden.")
                 # Hier evtl. leeres warped_frame erzeugen oder Rest überspringen
                 self.lane_obj.warped_frame = None
                 self.lane_obj.filtered_warped_frame = None
            else:
                self.lane_obj.perspective_transform(frame=input_for_warp)

            if current_params.get('publish_warped_frame', False) and self.lane_obj.warped_frame is not None:
                self._publish_image(self.pub_warped_frame, self.lane_obj.warped_frame, timestamp)

            # --- 4. Nach Dicke filtern ---
            self.lane_obj.filter_lane_markings_by_thickness(plot=False, **current_params)
            if current_params.get('publish_filtered_warped', False) and self.lane_obj.filtered_warped_frame is not None:
                 self._publish_image(self.pub_filtered_warped, self.lane_obj.filtered_warped_frame, timestamp)

            # --- 5. Histogramm ---
            self.histogram = self.lane_obj.calculate_histogram(plot=False)

            # --- 6. Sliding Windows / Fit ---
            left_fit, right_fit, sliding_window_img = self.lane_obj.get_lane_line_indices_sliding_windows(
                plot=True, **current_params)
            if current_params.get('publish_sliding_window', False) and sliding_window_img is not None:
                self._publish_image(self.pub_sliding_window, sliding_window_img, timestamp)

            # --- 7. Fits verfeinern / Fallback ---
            if left_fit is None or right_fit is None:
                  self.lane_obj.get_lane_line_previous_window(self.lane_obj.previous_left_fit, self.lane_obj.previous_right_fit, plot=False)

            # --- 8. Overlay, Krümmung und Offset ---
            final_annotated_frame = frame
            if self.lane_obj.left_fit is not None and self.lane_obj.right_fit is not None:
                overlay_img_on_original = self.lane_obj.overlay_lane_lines(plot=False)
                if overlay_img_on_original is None: overlay_img_on_original = frame.copy()
                self.lane_obj.calculate_curvature(print_to_terminal=False, **current_params)
                self.lane_obj.calculate_car_position(print_to_terminal=False, **current_params)
                final_annotated_frame = self.lane_obj.display_curvature_offset(frame=overlay_img_on_original, plot=False)
                if final_annotated_frame is None: final_annotated_frame = overlay_img_on_original
            else:
                 self.lane_obj.center_offset = 0.0

            if current_params.get('publish_lane_annotated', False) and final_annotated_frame is not None:
                 self._publish_image(self.pub_lane_annotated, final_annotated_frame, timestamp)

            offset_msg = Float64()
            offset_msg.data = float(self.lane_obj.center_offset) if self.lane_obj.center_offset is not None else 0.0
            self.driving_publisher_.publish(offset_msg)

        except CvBridgeError as e:
             self.get_logger().error(f"CvBridge Error in image_callback: {e}", throttle_duration_sec=10)
        except Exception as e:
             self.get_logger().error(f"Error processing image: {e}", throttle_duration_sec=10)
             self.get_logger().error(traceback.format_exc())

    def destroy_node(self):
        # Deine unveränderte destroy_node Methode
        # self.get_logger().info("Shutting down Lane Detection Node.")
        try: cv2.destroyAllWindows()
        except Exception: pass
        super().destroy_node()

def main(args=None):
    # Deine unveränderte main Methode
    rclpy.init(args=args)
    node = None
    try:
        node = LaneDetectionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if node: node.get_logger().error(f"FATALER FEHLER in LaneDetectionNode: {e}\n{traceback.format_exc()}")
        else: print(f"FATALER FEHLER vor Node-Init: {e}\n{traceback.format_exc()}", file=sys.stderr)
    finally:
        if node and isinstance(node, Node) and rclpy.ok():
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
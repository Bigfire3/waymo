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
from . import lane

class LaneDetectionNode(Node):
    def __init__(self):
        super().__init__('lane_detection_node')

        # --- Parameter Descriptors (unverändert) ---
        def int_desc(desc, min_val=0, max_val=255, step=1): return ParameterDescriptor(
            type=ParameterType.PARAMETER_INTEGER, description=desc,
            integer_range=[IntegerRange(from_value=min_val, to_value=max_val, step=step)])

        def float_desc(desc, min_val=0.0, max_val=1.0, step=0.001): return ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE, description=desc,
            floating_point_range=[FloatingPointRange(from_value=min_val, to_value=max_val, step=step)])

        # --- Parameter Deklarationen (unverändert) ---
        self.declare_parameter('block_size', 11, int_desc("Auto-S: block_size"))
        self.declare_parameter('c_value', 20, int_desc("Auto-S: c_value"))
        self.declare_parameter('center_factor', 0.03, float_desc("Center_Calc: factor"))
        self.declare_parameter('min_thickness', 2.5, ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE, description="Minimum thickness ratio", floating_point_range=[FloatingPointRange(from_value=0.01, to_value=200.0, step=0.001)]))
        self.declare_parameter('max_thickness', 5.0, ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE, description="Maximum thickness ratio", floating_point_range=[FloatingPointRange(from_value=0.01, to_value=200.0, step=0.001)]))
        self.declare_parameter('roi_top_left_w', 0.2, float_desc("ROI TL Breite"))
        self.declare_parameter('roi_top_left_h', 0.65, float_desc("ROI TL Höhe"))
        self.declare_parameter('roi_top_right_w', 0.8, float_desc("ROI TR Breite"))
        self.declare_parameter('roi_top_right_h', 0.65, float_desc("ROI TR Höhe"))
        self.declare_parameter('roi_bottom_left_w', 0.0, float_desc("ROI BL Breite"))
        self.declare_parameter('roi_bottom_left_h', 1., float_desc("ROI BL Höhe"))
        self.declare_parameter('roi_bottom_right_w', 1.0, float_desc("ROI BR Breite"))
        self.declare_parameter('roi_bottom_right_h', 1.0, float_desc("ROI BR Höhe"))
        self.declare_parameter('desired_roi_padding_factor', 0.25, float_desc("Ziel ROI Padding", max_val=0.4, step=0.01))

        # --- QoS und Subscriber/Publisher (unverändert) ---
        qos_policy = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.subscription = self.create_subscription(CompressedImage, '/image_raw/compressed', self.image_callback, qos_policy)
        self.img_publisher_ = self.create_publisher(CompressedImage, '/lane/image_annotated', qos_policy)
        self.driving_publisher_ = self.create_publisher(Float64, '/lane/center_offset', qos_policy)

        # Erstelle einmal das Lane-Objekt (unverändert)
        self.lane_obj = lane.Lane()
        # self.get_logger().info('Lane Detection Node started.') # Init Log evtl. entfernen?

    def image_callback(self, msg: CompressedImage):
        # --- Komplette Logik der Bildverarbeitung bleibt unverändert ---
        current_params = {param_name: self.get_parameter(param_name).value for param_name in self._parameters}
        if msg is None: return
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None: return

        self.lane_obj.orig_frame = frame
        self.lane_obj.update_frame(frame, **current_params)

        self.lane_line_markings = self.lane_obj.get_line_markings(**current_params)
        # --- Plotting Aufrufe können Fehler verursachen, wenn keine GUI da ist ---
        # --- Sicherer: Nur plotten, wenn wirklich benötigt oder in try-except ---
        try:
            self.lane_obj.plot_roi(plot=False) # plot=True nur zum Debuggen
            self.lane_obj.perspective_transform(plot=False) # plot=True nur zum Debuggen
        except Exception as plot_e:
             # Vermeide Absturz, falls GUI nicht verfügbar ist (z.B. in Docker ohne X11)
             # self.get_logger().warn(f"Plotting failed (GUI available?): {plot_e}", throttle_duration_sec=10) # Log entfernt
             pass

        self.lane_obj.filter_lane_markings_by_thickness(plot=False, **current_params)
        self.histogram = self.lane_obj.calculate_histogram(plot=False)
        self.left_fit, right_fit = self.lane_obj.get_lane_line_indices_sliding_windows(plot=False) # plot=True nur zum Debuggen

        if self.left_fit is not None and right_fit is not None:
            self.lane_obj.get_lane_line_previous_window(self.left_fit, right_fit, plot=False)
            result = self.lane_obj.overlay_lane_lines(plot=False)
            self.lane_obj.calculate_curvature(print_to_terminal=False)
            self.lane_obj.calculate_car_position(print_to_terminal=False, **current_params)
            self.final_frame = self.lane_obj.display_curvature_offset(frame=result, plot=False)
        else:
            self.final_frame = frame

        ret, buffer = cv2.imencode('.jpg', self.final_frame)
        if not ret: return
        out_msg = CompressedImage(header=msg.header, format="jpeg", data=buffer.tobytes())
        # Publish nur wenn Kontext ok ist? QoS Best Effort macht es weniger kritisch
        try:
            if rclpy.ok() and self.context.ok(): self.img_publisher_.publish(out_msg)
        except Exception: pass # Fehler beim Publishen ignorieren

        offset_msg = Float64()
        if self.lane_obj.center_offset is not None:
            offset_msg.data = float(self.lane_obj.center_offset)
        else:
            offset_msg.data = 0.0
        try:
             if rclpy.ok() and self.context.ok(): self.driving_publisher_.publish(offset_msg)
        except Exception: pass
        # --- Ende Bildverarbeitungslogik ---

    # --- NEU: Explizite destroy_node Methode ---
    def destroy_node(self):
        """Aufräumarbeiten für LaneDetectionNode."""
        # Hier könnten spezifische Aufräumarbeiten stehen,
        # z.B. OpenCV Fenster schliessen, falls dieser Node sie öffnen würde.
        # Aktuell sind keine nötig, da die Fenster im gui_debug_node sind.
        # Wichtig ist der Aufruf der Superklasse.
        # self.get_logger().info("Shutting down Lane Detection Node.") # Log entfernt
        super().destroy_node()
    # --- Ende destroy_node ---


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LaneDetectionNode()
        rclpy.spin(node)
    # --- NEU: Explizites Abfangen von KeyboardInterrupt ---
    except KeyboardInterrupt:
        # Gib eine minimale Meldung aus oder einfach 'pass'
        if node: pass # node.get_logger().info("KeyboardInterrupt received, shutting down cleanly.") # Log entfernt
        else: print("KeyboardInterrupt received before node init.", file=sys.stderr)
    # --- Ende KeyboardInterrupt ---
    except Exception as e:
        # Logge andere Fehler weiterhin, aber mit print, falls Logger nicht geht
        print(f"FATALER FEHLER in LaneDetectionNode: {e}", file=sys.stderr)
        traceback.print_exc()
        # if node: node.get_logger().error(f"Unhandled exception: {e}", exc_info=True) # Log entfernt
    finally:
        # Node nur zerstören, wenn er erfolgreich erstellt wurde
        if node and isinstance(node, Node):
            # Prüfe ob rclpy noch ok ist, bevor destroy_node gerufen wird?
            # Normalerweise wird destroy_node vom Executor aufgerufen,
            # aber zur Sicherheit hier die Prüfung.
            if rclpy.ok():
                 node.destroy_node()
        # Shutdown nur wenn rclpy noch ok ist
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
#!/usr/bin/env python3
import time
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

        # --- Parameter Descriptors ---
        def int_desc(desc, min_val=0, max_val=255, step=1): return ParameterDescriptor(
            type=ParameterType.PARAMETER_INTEGER, description=desc,
            integer_range=[IntegerRange(from_value=min_val, to_value=max_val, step=step)])

        def float_desc(desc, min_val=0.0, max_val=1.0, step=0.001): return ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE, description=desc,
            floating_point_range=[FloatingPointRange(from_value=min_val, to_value=max_val, step=step)])

        # Thresholding
        # auto s_binary
        self.declare_parameter(
            'block_size', 11, int_desc("Auto-S: block_size"))
        self.declare_parameter('c_value', 20, int_desc("Auto-S: c_value"))

        self.declare_parameter('center_factor', 0.03,
                               float_desc("Center_Calc: factor"))

        # line thickness filter
        self.declare_parameter(
            'min_thickness', 2.5,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Minimum thickness ratio for filtering lane contours",
                floating_point_range=[FloatingPointRange(
                    from_value=0.01, to_value=200.0, step=0.001)]
            )
        )
        self.declare_parameter(
            'max_thickness', 5.0,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Maximum thickness ratio for filtering lane contours",
                floating_point_range=[FloatingPointRange(
                    from_value=0.01, to_value=200.0, step=0.001)]
            )
        )
        # self.declare_parameter('s_thresh_min', 80, int_desc("S-Kanal: Untere Schwelle"))
        # self.declare_parameter('s_thresh_max', 255, int_desc("S-Kanal: Obere Schwelle"))
        # self.declare_parameter('l_thresh_min', 120, int_desc("L-Kanal: Untere Schwelle"))
        # self.declare_parameter('l_thresh_max', 255, int_desc("L-Kanal: Obere Schwelle"))
        # self.declare_parameter('r_thresh_min', 150, int_desc("R-Kanal: Untere Schwelle"))
        # self.declare_parameter('r_thresh_max', 255, int_desc("R-Kanal: Obere Schwelle"))
        # self.declare_parameter('sobel_mag_thresh_min', 50, int_desc("Sobel Mag: Untere Schwelle"))
        # self.declare_parameter('sobel_mag_thresh_max', 255, int_desc("Sobel Mag: Obere Schwelle"))

        # Perspective Transform ROI
        self.declare_parameter('roi_top_left_w', 0.2,
                               float_desc("ROI TL Breite"))
        self.declare_parameter('roi_top_left_h', 0.65,
                               float_desc("ROI TL Höhe"))
        self.declare_parameter('roi_top_right_w', 0.8,
                               float_desc("ROI TR Breite"))
        self.declare_parameter('roi_top_right_h', 0.65,
                               float_desc("ROI TR Höhe"))
        self.declare_parameter('roi_bottom_left_w', 0.0,
                               float_desc("ROI BL Breite"))
        self.declare_parameter('roi_bottom_left_h', 1.,
                               float_desc("ROI BL Höhe"))
        self.declare_parameter('roi_bottom_right_w', 1.0,
                               float_desc("ROI BR Breite"))
        self.declare_parameter('roi_bottom_right_h', 1.0,
                               float_desc("ROI BR Höhe"))
        self.declare_parameter('desired_roi_padding_factor', 0.25, float_desc(
            "Ziel ROI Padding", max_val=0.4, step=0.01))

        # --- QoS und Subscriber/Publisher ---
        qos_policy = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.subscription = self.create_subscription(
            CompressedImage, '/image_raw/compressed', self.image_callback, qos_policy)
        self.img_publisher_ = self.create_publisher(
            CompressedImage, '/lane/image_annotated', qos_policy)
        self.driving_publisher_ = self.create_publisher(
            Float64, '/lane/center_offset', qos_policy)

        # Erstelle einmal das Lane-Objekt
        self.lane_obj = lane.Lane()

    def image_callback(self, msg: CompressedImage):
        current_params = {param_name: self.get_parameter(param_name).value
                          for param_name in self._parameters}

        if msg is None:
            return

        # Dekodiere das Bild
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            return

        # Verarbeite das aktuelle Frame:
        self.lane_obj.orig_frame = frame
        self.lane_obj.update_frame(frame, **current_params)

        # Führe alle Verarbeitungsschritte durch
        self.lane_line_markings = self.lane_obj.get_line_markings(
            **current_params)
        self.lane_obj.plot_roi(plot=True)
        self.lane_obj.perspective_transform(plot=True)
        self.lane_obj.filter_lane_markings_by_thickness(
            plot=False, **current_params)
        self.histogram = self.lane_obj.calculate_histogram(plot=False)
        self.left_fit, right_fit = self.lane_obj.get_lane_line_indices_sliding_windows(
            plot=True)

        if self.left_fit is not None and right_fit is not None:
            self.lane_obj.get_lane_line_previous_window(
                self.left_fit, right_fit, plot=False)
            result = self.lane_obj.overlay_lane_lines(plot=False)
            self.lane_obj.calculate_curvature(print_to_terminal=False)
            self.lane_obj.calculate_car_position(
                print_to_terminal=False, **current_params)
            self.final_frame = self.lane_obj.display_curvature_offset(
                frame=result, plot=False)
        else:
            self.final_frame = frame

        # Publish final image
        ret, buffer = cv2.imencode('.jpg', self.final_frame)
        if not ret:
            return
        out_msg = CompressedImage(
            header=msg.header, format="jpeg", data=buffer.tobytes())
        self.img_publisher_.publish(out_msg)

        offset_msg = Float64()
        if self.lane_obj.center_offset is not None:
            offset_msg.data = float(self.lane_obj.center_offset)
            # Publish center offset to topic /lane/center_offset
            self.driving_publisher_.publish(offset_msg)
        else:
            offset_msg.data = 0.0
            self.driving_publisher_.publish(offset_msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LaneDetectionNode()
        rclpy.spin(node)
    except Exception as e:
        print(f"FATALER FEHLER beim Starten/Ausführen: {e}", file=sys.stderr)
        traceback.print_exc()
    finally:
        # Node nur zerstören, wenn er erfolgreich erstellt wurde und rclpy noch ok ist
        if node and isinstance(node, Node) and rclpy.ok():
            node.destroy_node()
        # Shutdown nur wenn rclpy noch ok ist
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

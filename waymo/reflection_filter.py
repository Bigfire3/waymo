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

class ReflectionFilter(Node):
    def __init__(self):
        super().__init__('reflection_filter')
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
        self.declare_parameter('threshold_1', 50, int_desc("threshold_1"))
        self.declare_parameter('threshold_2', 200, int_desc("threshold_2"))

        self.declare_parameter('ksize', 3, int_desc("ksize"))
        self.declare_parameter('iterations', 1, int_desc("iterations"))

        self.declare_parameter('d_value', 5, int_desc("d_value"))
        self.declare_parameter('sigmaColor', 75, int_desc("sigmaColor"))
        self.declare_parameter('sigmaSpace', 75, int_desc("sigmaSpace"))

        self.declare_parameter('min_contour_area', 200, int_desc("min_contour_area", max_val=1000))
        self.declare_parameter('max_contour_area', 1000, int_desc("max_contour_area", max_val=100000))

        self.declare_parameter('roi_top_left_w', 0.1, float_desc("ROI TL Breite")) # Schrittweite 0.001
        self.declare_parameter('roi_top_left_h', 0.65, float_desc("ROI TL Höhe")) # Schrittweite 0.001
        self.declare_parameter('roi_top_right_w', 0.9, float_desc("ROI TR Breite")) # Schrittweite 0.001
        self.declare_parameter('roi_top_right_h', 0.65, float_desc("ROI TR Höhe")) # Schrittweite 0.001
        self.declare_parameter('roi_bottom_left_w', 0.0, float_desc("ROI BL Breite")) # Schrittweite 0.001
        self.declare_parameter('roi_bottom_left_h', 1.0, float_desc("ROI BL Höhe")) # Schrittweite 0.001
        self.declare_parameter('roi_bottom_right_w', 1.0, float_desc("ROI BR Breite")) # Schrittweite 0.001
        self.declare_parameter('roi_bottom_right_h', 1.0, float_desc("ROI BR Höhe")) # Schrittweite 0.001


        # Beachte: min/max thickness hatten step=0.001, daher keine Rundung nötig für Standardwerte 2.5 und 5.0
        self.declare_parameter('min_thickness', 2.0, float_desc("Minimum thickness ratio", min_val=0.01, max_val=200.0, step=0.001))


        # --- QoS und Subscriber/Publisher ---
        qos_sensor_data = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.subscription = self.create_subscription(CompressedImage, '/image_raw/compressed', self.image_callback, qos_sensor_data)

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
        # --- Bild dekodieren ---
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        gray_scale_frame = cv2.imdecode(np_arr, cv2.IMREAD_GRAYSCALE)

        height, width = frame.shape[:2]

        self.roi_points = np.float32([
            (width * self.get_parameter('roi_top_left_w').value, height * self.get_parameter('roi_top_left_h').value),   # top-left
            (width * self.get_parameter('roi_bottom_left_w').value, height * self.get_parameter('roi_bottom_left_h').value),     # bottom-left
            (width * self.get_parameter('roi_bottom_right_w').value, height * self.get_parameter('roi_bottom_right_h').value),     # bottom-right
            (width * self.get_parameter('roi_top_right_w').value, height * self.get_parameter('roi_top_right_h').value)     # top-right
        ])
        # Zielpunkte für die Perspektivtransformation
        padding = int(0.25 * width)
        orig_image_size = frame.shape[::-1][1:]
        self.desired_roi_points = np.float32([
            [padding, 0],
            [padding, orig_image_size[1]],
            [orig_image_size[0] - padding, orig_image_size[1]],
            [orig_image_size[0] - padding, 0]
        ])

        warped_frame = self.perspective_transform(gray_scale_frame)
        cropped = warped_frame[:, 160:width - 160]
        cv2.imshow('Warped', cropped)

        # Kontrast erhöhen
        # image_eq = cv2.equalizeHist(warped_frame)
        # cv2.imshow('Contrast', image_eq)

        d_value = self.get_parameter("d_value").value
        sigmaColor = self.get_parameter("sigmaColor").value
        sigmaSpace = self.get_parameter("sigmaSpace").value
        blur = cv2.bilateralFilter(cropped, d=d_value, sigmaColor=sigmaColor, sigmaSpace=sigmaSpace)
        cv2.imshow('Blur', blur)

        # Kanten finden
        threshold_1 = self.get_parameter("threshold_1").value
        threshold_2 = self.get_parameter("threshold_2").value
        edges = cv2.Canny(blur, threshold_1, threshold_2)

        # Extra Linien
        h, w = cropped.shape[:2]
        cv2.line(edges, (0, h - 1), (w, h - 1), color=255, thickness=1)
        cv2.line(edges, (w - 100, h - 450), (w - 10, h - 450), color=255, thickness=1)
        cv2.line(edges, (w - 100, h - 200), (w - 10, h - 200), color=255, thickness=1)

        # --- Lücken schließen ---
        ksize = self.get_parameter("ksize").value
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (ksize, ksize))
        closed = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
        iterations = self.get_parameter("iterations").value
        #closed = cv2.dilate(closed, kernel, iterations=iterations)
        cv2.imshow("Edges (Closed)", closed)

        # --- Konturen finden ---
        contours, _ = cv2.findContours(closed, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # --- Maske vorbereiten ---
        filled_mask = np.zeros_like(cropped, dtype=np.uint8)

        # --- Alle Flächen füllen ---
        for cnt in contours:
            cv2.drawContours(filled_mask, [cnt], -1, 255, cv2.FILLED)
            # if cv2.contourArea(cnt) > self.get_parameter("min_contour_area").value and cv2.contourArea(cnt) < self.get_parameter("max_contour_area").value:  # kleinere Objekte ignorieren
            #     cv2.drawContours(filled_mask, [cnt], -1, 255, cv2.FILLED)

        cv2.imshow("Gefüllte Flächen", filled_mask)


        # Leeres Binärbild (gleiche Größe)
        output_mask = np.zeros_like(frame, dtype=np.uint8)

        # for i, contour in enumerate(contours):
        #     # Konturvereinfachung
        #     arc_len = cv2.arcLength(contour, True)
        #     epsilon = self.get_parameter("epsilon_factor").value * arc_len
        #     approx = cv2.approxPolyDP(contour, epsilon, True)

        #     if len(approx) == 4:
        #         # Kontur füllen (weiß)
        #         cv2.drawContours(output_mask, [approx], -1, 255, thickness=cv2.FILLED)

        #     # Nur vereinfachte Eckpunkte anzeigen
        #     for point in approx:
        #         x, y = point[0]
        #         cv2.circle(frame,
        #                 (x, y),
        #                 2,                  # kleiner Radius
        #                 (0, 255, 0),        # grün
        #                 -1,                 # gefüllt
        #                 cv2.LINE_AA)
            
                
        # cv2.imshow('Points', frame)
        # cv2.imshow('Filled', output_mask)

        cv2.waitKey(1)


    def destroy_node(self):
        self.get_logger().info("Shutting down Lane Detection Node.")
        try: cv2.destroyAllWindows()
        except Exception: pass
        super().destroy_node()
    
    def perspective_transform(self, frame=None):
        try:
             self.transformation_matrix = cv2.getPerspectiveTransform(self.roi_points, self.desired_roi_points)
             self.inv_transformation_matrix = cv2.getPerspectiveTransform(self.desired_roi_points, self.roi_points)
        except cv2.error as e: print(f"Fehler bei getPerspectiveTransform: {e}"); self.warped_frame=None; self.transformation_matrix=None; self.inv_transformation_matrix=None; return
        try:
            h, w = frame.shape[:2]
            warped_frame = cv2.warpPerspective(frame, self.transformation_matrix, (w, h), flags=cv2.INTER_LINEAR)
        except cv2.error as e: print(f"Fehler bei warpPerspective: {e}"); self.warped_frame = None; return
        
        mask = np.zeros_like(warped_frame, dtype=np.uint8)
        cv2.fillPoly(mask, [np.int32(self.desired_roi_points)], 255)
        warped_frame = cv2.bitwise_and(warped_frame, mask)
        
        return warped_frame


# main Funktion bleibt wie in deiner "neuen" Datei, da sie den Fehler-Fix enthielt
def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ReflectionFilter()
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
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2
from cv_bridge import CvBridge
import sys
import traceback

from rcl_interfaces.msg import (
    ParameterDescriptor,
    ParameterType,
    FloatingPointRange,
    IntegerRange,
)

NODE_NAME = "image_analysis_debugger_node"
IMAGE_TOPIC = "/image_raw/compressed"


class ImageAnalysisDebuggerNode(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.bridge = CvBridge()
        self.get_logger().info(
            f"'{NODE_NAME}' gestartet. Suche nach dem 'Tal' im Histogramm."
        )
        self.get_logger().info(
            "Fenster werden angezeigt. Stelle sicher, dass du eine grafische Umgebung (z.B. Desktop) hast."
        )

        # --- Helfer für Parameter-Deskriptoren ---
        def int_desc(desc, min_val=0, max_val=255, step=1):
            return ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description=desc,
                integer_range=[
                    IntegerRange(from_value=min_val, to_value=max_val, step=step)
                ],
            )

        def percent_desc(desc):
            return ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description=desc,
                floating_point_range=[
                    FloatingPointRange(from_value=0.0, to_value=100.0, step=0.1)
                ],
            )

        # --- Parameter für die Bildanalyse ---
        self.declare_parameter(
            "img_analysis_crop_top_percent",
            60,
            percent_desc("Crop top part of image for analysis (%)"),
        )
        self.declare_parameter(
            "img_analysis_crop_bottom_percent",
            75.0,
            percent_desc("Crop bottom part of image for analysis (%)"),
        )
        self.declare_parameter(
            "img_analysis_binary_threshold",
            128,
            int_desc("Threshold for binary image conversion in analysis"),
        )

        # NEUER Parameter, um das Tal zu definieren
        self.declare_parameter(
            "histogram_valley_threshold",
            500,
            int_desc(
                "Max 'height' in histogram to be considered a 'valley' (road)",
                min_val=0,
                max_val=50000,
            ),
        )

        # --- QoS & Subscriber ---
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.image_subscriber = self.create_subscription(
            CompressedImage, IMAGE_TOPIC, self.image_callback, qos_sensor
        )

    def image_callback(self, msg: CompressedImage):
        """Wird bei jedem neuen Bild aufgerufen und startet die Analyse."""
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is None:
                self.get_logger().warn(
                    "Empfangener Frame konnte nicht dekodiert werden."
                )
                return
            self.analyze_and_display_image(frame)
        except Exception as e:
            self.get_logger().error(
                f"Fehler im Image-Callback: {e}\n{traceback.format_exc()}",
                throttle_duration_sec=5,
            )

    def analyze_and_display_image(self, frame: np.ndarray):
        """
        Analysiert das Bild, um die Fahrspurmitte ZWISCHEN den Linien zu finden,
        indem das breiteste "Tal" im Histogramm gesucht wird.
        """
        h, w, _ = frame.shape
        output_frame = frame.copy()

        # 1. Parameter holen
        crop_top_percent = self.get_parameter("img_analysis_crop_top_percent").value
        crop_bottom_percent = self.get_parameter(
            "img_analysis_crop_bottom_percent"
        ).value
        binary_threshold = self.get_parameter("img_analysis_binary_threshold").value
        valley_threshold = self.get_parameter("histogram_valley_threshold").value

        # 2. Zuschneiden und Binarisieren
        crop_top = int(h * crop_top_percent / 100.0)
        crop_bottom = int(h * crop_bottom_percent / 100.0)
        cropped_frame = frame[crop_top:crop_bottom, :]
        if cropped_frame.size == 0:
            return
        gray_frame = cv2.cvtColor(cropped_frame, cv2.COLOR_BGR2GRAY)
        _, binary_frame = cv2.threshold(
            gray_frame, binary_threshold, 255, cv2.THRESH_BINARY
        )

        # 3. Histogramm berechnen
        histogram = np.sum(binary_frame, axis=0)

        # 4. Das breiteste "Tal" (zusammenhängender Bereich unter dem Schwellenwert) finden
        is_valley = histogram < valley_threshold

        # Finde Start- und Endpunkte der Täler
        diff = np.diff(is_valley.astype(int))
        starts = np.where(diff == 1)[0] + 1
        ends = np.where(diff == -1)[0]

        # Randfälle behandeln (wenn Tal am Bildrand beginnt oder endet)
        if is_valley[0]:
            starts = np.insert(starts, 0, 0)
        if is_valley[-1]:
            ends = np.append(ends, len(is_valley) - 1)

        road_center_pixel = -1
        pixel_offset = "N/A"
        road_start, road_end = -1, -1

        if len(starts) > 0 and len(ends) > 0 and len(starts) == len(ends):
            lengths = ends - starts
            if np.max(lengths) > 0:
                longest_block_idx = np.argmax(lengths)
                road_start = starts[longest_block_idx]
                road_end = ends[longest_block_idx]

                # 5. Mitte des Tals berechnen
                road_center_pixel = (road_start + road_end) // 2
                image_center_pixel = w // 2
                pixel_offset = float(road_center_pixel - image_center_pixel)
                self.get_logger().info(
                    f"Analyse OK -> Mitte bei {road_center_pixel}px, Offset: {pixel_offset:.1f}px (Tal: {road_start}-{road_end})",
                    throttle_duration_sec=1,
                )
            else:
                self.get_logger().warn(
                    "Kein valides Tal im Histogramm gefunden.", throttle_duration_sec=1
                )
        else:
            self.get_logger().warn(
                "Kein Tal im Histogramm gefunden (starts/ends inkonsistent).",
                throttle_duration_sec=1,
            )

        # --- VISUALISIERUNG ---

        # Zeichne Linien auf das Originalbild
        cv2.line(output_frame, (0, crop_top), (w, crop_top), (0, 255, 255), 1)
        cv2.line(output_frame, (0, crop_bottom), (w, crop_bottom), (0, 255, 255), 1)
        cv2.line(output_frame, (w // 2, 0), (w // 2, h), (0, 255, 0), 1)

        if road_center_pixel != -1:
            cv2.line(
                output_frame,
                (road_center_pixel, crop_top),
                (road_center_pixel, crop_bottom),
                (255, 0, 0),
                2,
            )

        cv2.putText(
            output_frame,
            f"Offset: {pixel_offset}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 0, 0),
            3,
        )
        cv2.putText(
            output_frame,
            f"Offset: {pixel_offset}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (255, 255, 255),
            2,
        )

        cv2.imshow("1 - Original mit Overlays", output_frame)
        cv2.imshow("2 - Binarisiertes Bild (Cropped)", binary_frame)

        # Histogramm-Visualisierung
        hist_h = 200
        hist_img = np.zeros((hist_h, w, 3), dtype=np.uint8)
        hist_normalized = (
            histogram
            / (np.max(histogram) if np.max(histogram) > 0 else 1.0)
            * (hist_h - 10)
        ).astype(int)

        for x, h_val in enumerate(hist_normalized):
            cv2.line(hist_img, (x, hist_h), (x, hist_h - h_val), (255, 255, 255), 1)

        # Zeichne den Schwellenwert und das erkannte Tal ein
        cv2.line(
            hist_img,
            (0, hist_h - int(valley_threshold / np.max(histogram) * (hist_h - 10))),
            (w, hist_h - int(valley_threshold / np.max(histogram) * (hist_h - 10))),
            (0, 255, 255),
            1,
        )
        if road_start != -1:
            cv2.rectangle(
                hist_img, (road_start, 0), (road_end, hist_h), (0, 255, 0), 1
            )  # Grüner Kasten für das Tal
            cv2.line(
                hist_img,
                (road_center_pixel, 0),
                (road_center_pixel, hist_h),
                (255, 0, 0),
                1,
            )  # Blaue Linie für die Mitte

        cv2.imshow("3 - Histogramm (mit Tal-Erkennung)", hist_img)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ImageAnalysisDebuggerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if node:
            node.get_logger().error(f"FATAL ERROR: {e}\n{traceback.format_exc()}")
        else:
            print(
                f"FATAL ERROR (pre-init): {e}\n{traceback.format_exc()}",
                file=sys.stderr,
            )
    finally:
        cv2.destroyAllWindows()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

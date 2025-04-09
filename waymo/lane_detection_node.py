#!/usr/bin/env python3
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2
# Stelle sicher, dass edge_detection im gleichen Python-Paket liegt
from . import edge_detection as edge
import sys
import traceback
# IMPORTE für Parameter
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult, ParameterType, IntegerRange, FloatingPointRange
from rclpy.parameter import Parameter

# ==============================================================================
# Lane Class (Kombinierte Version mit Parametern und Robustheit)
# ==============================================================================
class Lane:
    """
    Repräsentiert eine Fahrspur (Lane) in einem Bild. Verarbeitet ein Bild
    basierend auf den übergebenen Parametern und mit erhöhter Robustheit.
    """
    def __init__(self, orig_frame, logger, **params):
        """
        Konstruktor mit Eingangsbild und einem Dictionary von Parametern.
        :param orig_frame: Ursprüngliches Bild (BGR)
        :param logger: Der Logger des aufrufenden Nodes
        :param params: Dictionary mit allen Parameterwerten
        """
        self.orig_frame = orig_frame
        self.logger = logger

        # --- Parameter aus dem Dictionary holen und speichern ---
        self.s_thresh_min = params.get('s_thresh_min', 80)
        self.s_thresh_max = params.get('s_thresh_max', 255)
        self.l_thresh_min = params.get('l_thresh_min', 120)
        self.l_thresh_max = params.get('l_thresh_max', 255)
        self.sobel_mag_thresh_min = params.get('sobel_mag_thresh_min', 50)
        self.sobel_mag_thresh_max = params.get('sobel_mag_thresh_max', 255)
        self.r_thresh_min = params.get('r_thresh_min', 150)
        self.r_thresh_max = params.get('r_thresh_max', 255)
        self.gaussian_ksize = params.get('gaussian_ksize', 3)
        if self.gaussian_ksize % 2 == 0: self.gaussian_ksize += 1

        self.sw_margin_factor = params.get('sw_margin_factor', 1.0/12.0)
        self.sw_minpix_factor = params.get('sw_minpix_factor', 1.0/24.0)
        self.sw_num_windows = params.get('sw_num_windows', 10)

        self.roi_tf = {
             'tl_w': params.get('roi_top_left_w', 0.45), 'tl_h': params.get('roi_top_left_h', 0.60),
             'bl_w': params.get('roi_bottom_left_w', 0.05), 'bl_h': params.get('roi_bottom_left_h', 0.95),
             'br_w': params.get('roi_bottom_right_w', 0.95), 'br_h': params.get('roi_bottom_right_h', 0.95),
             'tr_w': params.get('roi_top_right_w', 0.55), 'tr_h': params.get('roi_top_right_h', 0.60)
        }
        self.desired_roi_padding_factor = params.get('desired_roi_padding_factor', 0.25)

        self.YM_PER_PIX = params.get('ym_per_pix', 10.0/720.0)
        self.XM_PER_PIX = params.get('xm_per_pix', 3.7/700.0)
        if self.YM_PER_PIX <= 0: self.YM_PER_PIX = 1e-6; self.logger.warn("YM_PER_PIX war 0, setze auf kleinen Wert.")
        if self.XM_PER_PIX <= 0: self.XM_PER_PIX = 1e-6; self.logger.warn("XM_PER_PIX war 0, setze auf kleinen Wert.")

        self.overlay_weight = params.get('overlay_weight', 0.3)
        # --- Ende Parameter holen ---

        # --- Initialisierung von Attributen ---
        self.lane_line_markings = None; self.warped_frame = None
        self.transformation_matrix = None; self.inv_transformation_matrix = None
        self.orig_image_size = (0,0); self.width = 0; self.height = 0
        self.roi_points = np.float32([]); self.padding = 0; self.desired_roi_points = np.float32([])
        self.histogram = None; self.no_of_windows = max(1, self.sw_num_windows)
        self.margin = 1; self.minpix = 1

        # --- Berechnungen basierend auf Bild und Parametern ---
        if self.orig_frame is not None and self.orig_frame.shape[0] > 0 and self.orig_frame.shape[1] > 0:
            self.orig_image_size = self.orig_frame.shape[::-1][1:]
            self.width = self.orig_image_size[0]; self.height = self.orig_image_size[1]
            h, w = self.orig_frame.shape[:2]
            self.roi_points = np.float32([
                (w * self.roi_tf['tl_w'], h * self.roi_tf['tl_h']), (w * self.roi_tf['bl_w'], h * self.roi_tf['bl_h']),
                (w * self.roi_tf['br_w'], h * self.roi_tf['br_h']), (w * self.roi_tf['tr_w'], h * self.roi_tf['tr_h'])
            ])
            self.padding = int(self.desired_roi_padding_factor * self.width)
            self.desired_roi_points = np.float32([
                [self.padding, 0], [self.padding, self.height],
                [self.width-self.padding, self.height], [self.width-self.padding, 0]
            ])
            self.margin = max(1, int(self.sw_margin_factor * self.width))
            self.minpix = max(1, int(self.sw_minpix_factor * self.width))
        else: self.logger.error("__init__: Ungültiges Eingabebild!")

        self._reset_lane_line_attributes() # Setzt Fits, Punkte etc. auf None
        self.left_curvem = None; self.right_curvem = None; self.center_offset = None

    # --- Hilfsfunktion für Logger ---
    def get_logger(self): return self.logger

    # --- Bildverarbeitungsmethoden mit Robustheit ---

    def get_line_markings(self, frame=None):
        if frame is None: frame = self.orig_frame
        if frame is None: self.get_logger().error("get_line_markings: Kein Bild."); return None
        try:
            hls = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)
            s_channel = hls[:, :, 2]; l_channel = hls[:, :, 1]

            # Verwende die Parameterwerte aus self
            _, s_binary = edge.threshold(s_channel, thresh=(self.s_thresh_min, self.s_thresh_max))
            # Optional: Andere Kanäle/Methoden hinzufügen
            # _, l_binary = edge.threshold(l_channel, thresh=(self.l_thresh_min, self.l_thresh_max))
            # _, r_thresh = edge.threshold(frame[:, :, 2], thresh=(self.r_thresh_min, self.r_thresh_max))
            # mag_binary = edge.mag_thresh(l_channel, sobel_kernel=3, thresh=(self.sobel_mag_thresh_min, self.sobel_mag_thresh_max))

            # --- Kombination wählen (könnte auch Parameter sein) ---
            combined_binary = s_binary # Nur S-Kanal als Beispiel

            if self.gaussian_ksize >= 3:
                 combined_binary = edge.blur_gaussian(combined_binary, ksize=self.gaussian_ksize)

            self.lane_line_markings = combined_binary
            return self.lane_line_markings
        except cv2.error as e: self.get_logger().error(f"OpenCV Fehler get_line_markings: {e}")
        except Exception as e: self.get_logger().error(f"Fehler get_line_markings: {e}")
        self.lane_line_markings = None; return None

    def perspective_transform(self, frame=None):
        if frame is None: frame = self.lane_line_markings
        if frame is None or self.width == 0 or self.height == 0 or len(self.roi_points) == 0:
            # self.get_logger().warn("perspective_transform: Ungültige Eingabe.") # Weniger verbose
            self._reset_perspective_attributes(); return None
        try:
            self.transformation_matrix = cv2.getPerspectiveTransform(self.roi_points, self.desired_roi_points)
            self.inv_transformation_matrix = cv2.getPerspectiveTransform(self.desired_roi_points, self.roi_points)
            self.warped_frame = cv2.warpPerspective(frame, self.transformation_matrix, (self.width, self.height), flags=cv2.INTER_LINEAR)
            return self.warped_frame
        except Exception as e:
             self.get_logger().error(f"Fehler in perspective_transform: {e}")
             self._reset_perspective_attributes(); return None

    def calculate_histogram(self, frame=None):
        if frame is None: frame = self.warped_frame
        if frame is None or frame.shape[0] < 2:
             self.histogram = None; return None
        try:
            self.histogram = np.sum(frame[frame.shape[0]//2:, :], axis=0)
            return self.histogram
        except Exception as e:
             self.get_logger().error(f"Fehler in calculate_histogram: {e}")
             self.histogram = None; return None

    def histogram_peak(self):
        if self.histogram is None or len(self.histogram) == 0: raise ValueError("Histogramm ist leer.")
        midpoint = int(len(self.histogram)//2)
        if midpoint == 0: raise ValueError("Histogramm zu klein.")
        try:
            leftx_base = np.argmax(self.histogram[:midpoint])
            rightx_base = np.argmax(self.histogram[midpoint:]) + midpoint
            return leftx_base, rightx_base
        except ValueError as e: raise ValueError(f"Fehler bei Peak-Findung: {e}")

    def get_lane_line_indices_sliding_windows(self):
        self._reset_lane_line_attributes() # Start fresh
        if self.warped_frame is None or self.histogram is None or self.height == 0: return None, None
        margin, minpix, num_windows = self.margin, self.minpix, self.no_of_windows
        window_height = int(self.height // num_windows)
        if window_height == 0: return None, None

        try: nonzero = self.warped_frame.nonzero(); nonzeroy = np.array(nonzero[0]); nonzerox = np.array(nonzero[1])
        except Exception: return None, None # Fehler bei Nonzero
        if len(nonzeroy) == 0: return None, None

        left_lane_inds, right_lane_inds = [], []
        try: leftx_base, rightx_base = self.histogram_peak()
        except ValueError: return None, None
        leftx_current, rightx_current = leftx_base, rightx_base

        for window in range(num_windows):
            win_y_low = self.height - (window + 1) * window_height; win_y_high = self.height - window * window_height
            win_xleft_low = int(leftx_current - margin); win_xleft_high = int(leftx_current + margin)
            win_xright_low = int(rightx_current - margin); win_xright_high = int(rightx_current + margin)

            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]
            left_lane_inds.append(good_left_inds); right_lane_inds.append(good_right_inds)

            try:
                if len(good_left_inds) > minpix: leftx_current = np.int_(np.mean(nonzerox[good_left_inds]))
                if len(good_right_inds) > minpix: rightx_current = np.int_(np.mean(nonzerox[good_right_inds]))
            except Exception: pass # Ignoriere Fehler bei Mittelwertbildung, behalte alten Current

        try:
             left_lane_inds = np.concatenate(left_lane_inds); right_lane_inds = np.concatenate(right_lane_inds)
        except ValueError: return None, None

        leftx, lefty = nonzerox[left_lane_inds], nonzeroy[left_lane_inds]
        rightx, righty = nonzerox[right_lane_inds], nonzeroy[right_lane_inds]
        self.leftx, self.rightx, self.lefty, self.righty = leftx, rightx, lefty, righty

        if len(lefty) < 3 or len(righty) < 3: return None, None
        try:
            left_fit = np.polyfit(lefty, leftx, 2); right_fit = np.polyfit(righty, rightx, 2)
            self.left_fit, self.right_fit = left_fit, right_fit
            self._update_plot_attributes()
        except (np.linalg.LinAlgError, TypeError, ValueError): return None, None
        return self.left_fit, self.right_fit

    def get_lane_line_previous_window(self, left_fit, right_fit):
        if left_fit is None or right_fit is None or self.warped_frame is None:
             self._reset_lane_line_attributes(keep_fits=True, fits=(left_fit, right_fit)); return
        margin = self.margin
        try:
            nonzero = self.warped_frame.nonzero(); nonzeroy = np.array(nonzero[0]); nonzerox = np.array(nonzero[1])
            if len(nonzeroy) == 0: self._reset_lane_line_attributes(keep_fits=True, fits=(left_fit, right_fit)); return

            left_fit_y = left_fit[0]*(nonzeroy**2) + left_fit[1]*nonzeroy + left_fit[2]
            right_fit_y = right_fit[0]*(nonzeroy**2) + right_fit[1]*nonzeroy + right_fit[2]
            left_lane_inds = ((nonzerox > (left_fit_y - margin)) & (nonzerox < (left_fit_y + margin)))
            right_lane_inds = ((nonzerox > (right_fit_y - margin)) & (nonzerox < (right_fit_y + margin)))
            self.left_lane_inds, self.right_lane_inds = left_lane_inds, right_lane_inds

            leftx, lefty = nonzerox[left_lane_inds], nonzeroy[left_lane_inds]
            rightx, righty = nonzerox[right_lane_inds], nonzeroy[right_lane_inds]

            if len(lefty) < 3 or len(righty) < 3: # Nicht genug Punkte für Refit
                self._reset_lane_line_attributes(keep_fits=True, fits=(left_fit, right_fit))
                self._update_plot_attributes() # Update Plot basierend auf altem Fit
                return

            self.leftx, self.rightx, self.lefty, self.righty = leftx, rightx, lefty, righty
            new_left_fit = np.polyfit(lefty, leftx, 2); new_right_fit = np.polyfit(righty, rightx, 2)
            self.left_fit, self.right_fit = new_left_fit, new_right_fit
            self._update_plot_attributes()

        except (np.linalg.LinAlgError, TypeError, ValueError) as e:
             self._reset_lane_line_attributes(keep_fits=True, fits=(left_fit, right_fit))
             self._update_plot_attributes()
        except Exception as e:
            self.get_logger().error(f"Fehler in get_lane_line_previous_window: {e}")
            self._reset_lane_line_attributes()

    def overlay_lane_lines(self):
        if self.warped_frame is None or self.left_fitx is None or self.right_fitx is None or \
           self.ploty is None or self.inv_transformation_matrix is None or self.orig_frame is None:
            return self.orig_frame
        try:
            warp_zero = np.zeros_like(self.warped_frame).astype(np.uint8)
            color_warp = np.dstack((warp_zero, warp_zero, warp_zero))
            pts_left = np.array([np.transpose(np.vstack([self.left_fitx, self.ploty]))])
            pts_right = np.array([np.flipud(np.transpose(np.vstack([self.right_fitx, self.ploty])))])
            pts = np.hstack((pts_left, pts_right))
            cv2.fillPoly(color_warp, np.int_([pts]), (0,255, 0))
            newwarp = cv2.warpPerspective(color_warp, self.inv_transformation_matrix, (self.width, self.height))
            result = cv2.addWeighted(self.orig_frame, 1, newwarp, self.overlay_weight, 0) # Parameter nutzen
            return result
        except Exception as e:
            self.get_logger().error(f"Fehler in overlay_lane_lines: {e}"); return self.orig_frame

    def calculate_curvature(self):
        if self.ploty is None or self.lefty is None or self.leftx is None or self.righty is None or \
           self.rightx is None or len(self.lefty) < 3 or len(self.righty) < 3:
            self.left_curvem, self.right_curvem = None, None; return None, None
        try:
            y_eval_pix = np.max(self.ploty) # Y-Position (Pixel) am unteren Bildrand
            y_eval_m = y_eval_pix * self.YM_PER_PIX # Y-Position (Meter)

            left_fit_cr = np.polyfit(self.lefty * self.YM_PER_PIX, self.leftx * self.XM_PER_PIX, 2)
            right_fit_cr = np.polyfit(self.righty * self.YM_PER_PIX, self.rightx * self.XM_PER_PIX, 2)
            left_A, left_B = left_fit_cr[0], left_fit_cr[1]; right_A, right_B = right_fit_cr[0], right_fit_cr[1]

            # Vermeide Division durch Null
            left_curvem = ((1 + (2*left_A*y_eval_m + left_B)**2)**1.5) / np.absolute(2*left_A) if abs(left_A) > 1e-6 else float('inf')
            right_curvem = ((1 + (2*right_A*y_eval_m + right_B)**2)**1.5) / np.absolute(2*right_A) if abs(right_A) > 1e-6 else float('inf')

            self.left_curvem, self.right_curvem = left_curvem, right_curvem
            return left_curvem, right_curvem
        except (np.linalg.LinAlgError, TypeError, ValueError) as e:
             self.left_curvem, self.right_curvem = None, None; return None, None
        except Exception as e:
            self.get_logger().error(f"Fehler in calculate_curvature: {e}")
            self.left_curvem, self.right_curvem = None, None; return None, None

    def calculate_car_position(self):
        if self.left_fit is None or self.right_fit is None or self.height == 0 or self.width == 0:
            self.center_offset = None; return None
        try:
            y_eval = self.height # Auswertung am unteren Rand
            bottom_left = self.left_fit[0]*y_eval**2 + self.left_fit[1]*y_eval + self.left_fit[2]
            bottom_right = self.right_fit[0]*y_eval**2 + self.right_fit[1]*y_eval + self.right_fit[2]
            center_lane_pix = (bottom_left + bottom_right) / 2
            center_car_pix = self.width / 2
            self.center_offset = (center_car_pix - center_lane_pix) * self.XM_PER_PIX * 100 # cm
            return self.center_offset
        except Exception as e:
            self.get_logger().error(f"Fehler in calculate_car_position: {e}")
            self.center_offset = None; return None

    def display_curvature_offset(self, frame):
        if frame is None: return None
        image_copy = frame.copy()
        curv_text, offset_text = 'Curve Radius: N/A', 'Center Offset: N/A'
        # Anzeige nur, wenn Werte berechnet wurden und sinnvoll sind
        if self.left_curvem is not None and self.right_curvem is not None:
            # Mittelwert nur, wenn beide endlich sind
            if np.isfinite(self.left_curvem) and np.isfinite(self.right_curvem):
                 avg_curve = (self.left_curvem + self.right_curvem) / 2
                 if abs(avg_curve) < 5000: curv_text = f'Curve Radius: {avg_curve:.0f} m'
            elif np.isfinite(self.left_curvem) and abs(self.left_curvem) < 5000:
                 curv_text = f'Curve Radius: L={self.left_curvem:.0f} m'
            elif np.isfinite(self.right_curvem) and abs(self.right_curvem) < 5000:
                 curv_text = f'Curve Radius: R={self.right_curvem:.0f} m'

        if self.center_offset is not None:
            offset_text = f'Center Offset: {self.center_offset:.1f} cm'
        try:
            font_scale = max(0.4, float((0.5/600)*self.width)) if self.width > 0 else 0.5
            # Positionen für Text angepasst
            cv2.putText(image_copy, curv_text, (15, 35), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255,255,255), 2, cv2.LINE_AA)
            cv2.putText(image_copy, offset_text, (15, 65), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255,255,255), 2, cv2.LINE_AA)
        except Exception as e: self.get_logger().error(f"Fehler beim Zeichnen von Text: {e}")
        return image_copy

    # --- Private Hilfsfunktionen ---
    def _reset_lane_line_attributes(self, keep_fits=False, fits=(None, None)):
        if not keep_fits: self.left_fit, self.right_fit = None, None
        else: self.left_fit, self.right_fit = fits
        self.left_lane_inds, self.right_lane_inds = None, None
        self.ploty, self.left_fitx, self.right_fitx = None, None, None
        self.leftx, self.rightx, self.lefty, self.righty = None, None, None, None

    def _reset_perspective_attributes(self):
         self.warped_frame, self.transformation_matrix, self.inv_transformation_matrix = None, None, None

    def _update_plot_attributes(self):
         if self.left_fit is None or self.right_fit is None or self.height == 0:
              self.ploty, self.left_fitx, self.right_fitx = None, None, None; return
         try:
              self.ploty = np.linspace(0, self.height - 1, self.height)
              self.left_fitx = self.left_fit[0]*self.ploty**2 + self.left_fit[1]*self.ploty + self.left_fit[2]
              self.right_fitx = self.right_fit[0]*self.ploty**2 + self.right_fit[1]*self.ploty + self.right_fit[2]
         except Exception as e:
              self.get_logger().error(f"Fehler _update_plot_attributes: {e}")
              self.ploty, self.left_fitx, self.right_fitx = None, None, None

# ==============================================================================
# LaneDetectionNode Class
# ==============================================================================
class LaneDetectionNode(Node):
    def __init__(self):
        super().__init__('lane_detection_node')

        # --- Parameter Descriptors ---
        int_desc = lambda desc, min_val=0, max_val=255, step=1: ParameterDescriptor(
            type=ParameterType.PARAMETER_INTEGER, description=desc,
            integer_range=[IntegerRange(from_value=min_val, to_value=max_val, step=step)])
        float_desc = lambda desc, min_val=0.0, max_val=1.0, step=0.01: ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE, description=desc,
            floating_point_range=[FloatingPointRange(from_value=min_val, to_value=max_val, step=step)])

        # --- Parameter Deklaration (KORRIGIERTE VERSION) ---
        # Thresholding
        self.declare_parameter('s_thresh_min', 80, int_desc("S-Kanal: Untere Schwelle"))
        self.declare_parameter('s_thresh_max', 255, int_desc("S-Kanal: Obere Schwelle"))
        self.declare_parameter('l_thresh_min', 120, int_desc("L-Kanal: Untere Schwelle"))
        self.declare_parameter('l_thresh_max', 255, int_desc("L-Kanal: Obere Schwelle"))
        self.declare_parameter('r_thresh_min', 150, int_desc("R-Kanal: Untere Schwelle"))
        self.declare_parameter('r_thresh_max', 255, int_desc("R-Kanal: Obere Schwelle"))
        self.declare_parameter('sobel_mag_thresh_min', 50, int_desc("Sobel Mag: Untere Schwelle"))
        self.declare_parameter('sobel_mag_thresh_max', 255, int_desc("Sobel Mag: Obere Schwelle"))
        self.declare_parameter('gaussian_ksize', 3, int_desc("Gauss KSize (ungerade)", 1, 15, 2))

        # Sliding Windows (Korrigierte Standardwerte)
        self.declare_parameter('sw_margin_factor', 0.085, float_desc("SW Margin Faktor", max_val=0.5, step=0.005))
        self.declare_parameter('sw_minpix_factor', 0.042, float_desc("SW MinPix Faktor", max_val=0.2, step=0.002))
        self.declare_parameter('sw_num_windows', 10, int_desc("SW Anzahl Fenster", 5, 20))

        # Perspective Transform ROI
        self.declare_parameter('roi_top_left_w', 0.45, float_desc("ROI TL Breite"))
        self.declare_parameter('roi_top_left_h', 0.60, float_desc("ROI TL Höhe"))
        self.declare_parameter('roi_bottom_left_w', 0.05, float_desc("ROI BL Breite"))
        self.declare_parameter('roi_bottom_left_h', 0.95, float_desc("ROI BL Höhe"))
        self.declare_parameter('roi_bottom_right_w', 0.95, float_desc("ROI BR Breite"))
        self.declare_parameter('roi_bottom_right_h', 0.95, float_desc("ROI BR Höhe"))
        self.declare_parameter('roi_top_right_w', 0.55, float_desc("ROI TR Breite"))
        self.declare_parameter('roi_top_right_h', 0.60, float_desc("ROI TR Höhe"))
        self.declare_parameter('desired_roi_padding_factor', 0.25, float_desc("Ziel ROI Padding", max_val=0.4, step=0.01))

        # Calibration (Korrigierte Standardwerte)
        self.declare_parameter('ym_per_pix', 0.0139, float_desc("Y m/pix", max_val=0.1, step=0.0001)) # Nahe 10/720
        self.declare_parameter('xm_per_pix', 0.0053, float_desc("X m/pix", max_val=0.1, step=0.0001)) # Nahe 3.7/700

        # Overlay
        self.declare_parameter('overlay_weight', 0.3, float_desc("Overlay Gewicht"))
        # --- Ende Parameter Deklaration ---

        # --- QoS und Subscriber/Publisher ---
        qos_policy = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.subscription = self.create_subscription(CompressedImage, '/image_raw/compressed', self.image_callback, qos_policy)
        self.publisher_ = self.create_publisher(CompressedImage, '/lane/image_annotated', qos_policy)
        self.get_logger().info('Lane Detection Node gestartet mit umfassenden dynamischen Parametern.')


    def image_callback(self, msg: CompressedImage):
        try:
            # --- Hole ALLE aktuellen Parameterwerte ---
            current_params = {param_name: self.get_parameter(param_name).value
                              for param_name in self._parameters}

            # --- Bild dekodieren ---
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if cv_image is None: self.get_logger().warn("Bild Dekodieren fehlgeschlagen."); return

            # --- Lane Objekt erstellen & Pipeline ausführen ---
            lane_obj = Lane(orig_frame=cv_image, logger=self.get_logger(), **current_params)
            annotated_frame = cv_image # Fallback

            lane_markings = lane_obj.get_line_markings()
            if lane_markings is not None:
                warped_frame = lane_obj.perspective_transform(lane_markings)
                if warped_frame is not None:
                    histogram = lane_obj.calculate_histogram(warped_frame)
                    if histogram is not None:
                        left_fit, right_fit = lane_obj.get_lane_line_indices_sliding_windows()
                        if left_fit is not None and right_fit is not None:
                            # Optional: Previous Window Verfeinerung
                            lane_obj.get_lane_line_previous_window(left_fit, right_fit)
                            frame_with_lanes = lane_obj.overlay_lane_lines()
                            if frame_with_lanes is not None:
                                lane_obj.calculate_curvature()
                                lane_obj.calculate_car_position()
                                annotated_frame = lane_obj.display_curvature_offset(frame_with_lanes)
            # --- Ende Pipeline ---

            # --- Bild kodieren und publizieren ---
            ret, buffer = cv2.imencode('.jpg', annotated_frame)
            if not ret: self.get_logger().error("JPEG Kodieren fehlgeschlagen."); return
            out_msg = CompressedImage(header=msg.header, format="jpeg", data=buffer.tobytes())
            self.publisher_.publish(out_msg)

        except Exception as e:
             self.get_logger().fatal(f'FATALER Fehler im image_callback: {e}\n{traceback.format_exc()}')

# ==============================================================================
# Main Funktion
# ==============================================================================
def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LaneDetectionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node: node.get_logger().info("KeyboardInterrupt, beende Node.")
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
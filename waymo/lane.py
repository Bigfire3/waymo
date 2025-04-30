# waymo/lane.py
import cv2
import numpy as np
from . import edge_detection as edge
# Importiere matplotlib nur, wenn es wirklich gebraucht wird (z.B. für optionale Plots)
# import matplotlib.pyplot as plt


class Lane:
    def __init__(self):
        # Polynom-Anpassung (Fit) für linke und rechte Spur
        self.left_fit = None
        self.right_fit = None
        self.left_lane_inds = None
        self.right_lane_inds = None
        self.ploty = None
        self.left_fitx = None
        self.right_fitx = None
        self.leftx = None
        self.rightx = None
        self.lefty = None
        self.righty = None

        # Für die Glättung am unteren Rand:
        self.previous_bottom_left_x = None
        self.previous_bottom_right_x = None

        # Für die Glättung des Fahrspurmittelpunkts (über die Zeit)
        self.previous_lane_center = None
        self.smoothed_center = None

        # Für die Glättung der gesamten Fits (optional)
        self.previous_left_fit = None
        self.previous_right_fit = None

        # Umrechnungsfaktoren von Pixel in Meter (Standardwerte, können durch Params überschrieben werden)
        self.YM_PER_PIX = 30.0 / 720 # Default Meters per pixel in y dimension
        self.XM_PER_PIX = 3.7 / 700  # Default Meters per pixel in x dimension

        # Krümmungsradien und Offset (später berechnet)
        self.left_curvem = None
        self.right_curvem = None
        self.center_offset = None

        self.current_center = None

        # --- NEU: Variable für Sliding Window Debug Bild ---
        self.sliding_window_debug_img = None
        # --- Ende NEU ---

    def update_frame(self, orig_frame, **params):
        self.orig_frame = orig_frame

        # Bild mit den Fahrspurlinien (binär)
        self.lane_line_markings = None

        # Bild nach der Perspektivtransformation
        self.warped_frame = None
        self.filtered_warped_frame = None
        self.transformation_matrix = None
        self.inv_transformation_matrix = None

        # Originalbildgröße (Breite, Höhe)
        self.orig_image_size = self.orig_frame.shape[::-1][1:]
        width = self.orig_image_size[0]
        height = self.orig_image_size[1]
        self.width = width
        self.height = height

        # Vier Ecken des trapezförmigen ROI (Region of Interest)
        h, w = self.orig_frame.shape[:2]
        # Sicherstellen, dass Parameter existieren, bevor sie verwendet werden
        self.roi_points = np.float32([
            (w * params.get('roi_top_left_w', 0.1), h * params.get('roi_top_left_h', 0.65)),   # top-left
            (w * params.get('roi_bottom_left_w', 0.0), h * params.get('roi_bottom_left_h', 1.0)),     # bottom-left
            (w * params.get('roi_bottom_right_w', 1.0), h * params.get('roi_bottom_right_h', 1.0)),     # bottom-right
            (w * params.get('roi_top_right_w', 0.9), h * params.get('roi_top_right_h', 0.65))     # top-right
        ])

        # Zielpunkte für die Perspektivtransformation
        self.padding = int(params.get('desired_roi_padding_factor', 0.25) * width)
        self.desired_roi_points = np.float32([
            [self.padding, 0],
            [self.padding, self.orig_image_size[1]],
            [self.orig_image_size[0] - self.padding, self.orig_image_size[1]],
            [self.orig_image_size[0] - self.padding, 0]
        ])

        # Histogramm zur Bestimmung der Fahrspurbasis
        self.histogram = None

        # Parameter für die Sliding-Window-Methode
        self.no_of_windows = 10
        # Margin und Minpix aus Parametern holen oder Defaults verwenden
        self.margin = int(params.get('sliding_window_margin_factor', 1/8) * width)
        self.minpix = int(params.get('sliding_window_minpix', 50))

        # --- NEU: Reset Sliding Window Debug Bild ---
        self.sliding_window_debug_img = None
        # --- Ende NEU ---

    def calculate_car_position(self, print_to_terminal=False, **params):
        """
        Berechnet den Offset zwischen dem Fahrzeug (angenommen, Kamera ist im Bildzentrum)
        und der Fahrspurmitte.
        """
        if self.left_fit is None or self.right_fit is None or self.ploty is None:
            self.center_offset = None # Sicherstellen, dass Offset None ist
            return None

        # Y-Koordinate am unteren Rand für die Berechnung (etwas über dem absoluten Rand)
        self.y_bottom = self.orig_frame.shape[0] - 1 # Oder ein anderer Wert wie self.height -1

        # Berechne die X-Koordinaten der Linien am unteren Rand
        try:
             current_left_x = self.left_fit[0] * self.y_bottom**2 + \
                 self.left_fit[1] * self.y_bottom + self.left_fit[2]
             current_right_x = self.right_fit[0] * self.y_bottom**2 + \
                 self.right_fit[1] * self.y_bottom + self.right_fit[2]
             self.current_center = ((current_left_x + current_right_x) / 2)
        except (TypeError, IndexError):
             self.current_center = None
             self.center_offset = None
             return None

        # Kamera als Bildmitte annehmen
        car_location = self.orig_frame.shape[1] / 2
        pixel_offset = self.current_center - car_location
        center_factor = params.get('center_factor', 0.03) # Standardwert wie im Original-Node
        center_offset_calculated = pixel_offset * center_factor

        if print_to_terminal:
            print(f"Pixel Offset: {pixel_offset:.2f} px, Calculated Offset: {center_offset_calculated:.4f}")
        self.center_offset = center_offset_calculated
        return self.center_offset

    def calculate_curvature(self, print_to_terminal=False, **params):
        if (self.left_fit is None or self.right_fit is None or
            self.lefty is None or self.righty is None or self.ploty is None or
                len(self.lefty) == 0 or len(self.righty) == 0):
            if print_to_terminal: print("Keine Fahrspurlinie für Krümmungsberechnung erkannt")
            self.left_curvem = None; self.right_curvem = None
            return None, None

        y_eval = np.max(self.ploty)
        ym_per_pix = params.get('ym_per_pix', self.YM_PER_PIX)
        xm_per_pix = params.get('xm_per_pix', self.XM_PER_PIX)

        try:
            left_fit_cr = np.polyfit(self.lefty * ym_per_pix, self.leftx * xm_per_pix, 2)
            right_fit_cr = np.polyfit(self.righty * ym_per_pix, self.rightx * xm_per_pix, 2)
        except (np.linalg.LinAlgError, TypeError):
             print("Fehler beim Polyfit für Krümmung."); self.left_curvem = None; self.right_curvem = None
             return None, None

        left_a = left_fit_cr[0]; right_a = right_fit_cr[0]
        if abs(left_a) < 1e-6: self.left_curvem = float('inf')
        else: self.left_curvem = ((1 + (2*left_a*y_eval*ym_per_pix + left_fit_cr[1])**2)**1.5) / np.absolute(2*left_a)
        if abs(right_a) < 1e-6: self.right_curvem = float('inf')
        else: self.right_curvem = ((1 + (2*right_a*y_eval*ym_per_pix + right_fit_cr[1])**2)**1.5) / np.absolute(2*right_a)

        if print_to_terminal:
            left_str = f"{self.left_curvem:.1f} m" if self.left_curvem != float('inf') else "straight"
            right_str = f"{self.right_curvem:.1f} m" if self.right_curvem != float('inf') else "straight"
            print(f"Left Curvature: {left_str}, Right Curvature: {right_str}")
        return self.left_curvem, self.right_curvem

    def calculate_histogram(self, frame=None, plot=False):
        #... (Code wie zuvor)
        if frame is None: frame = self.filtered_warped_frame
        if frame is None or frame.ndim != 2: self.histogram = None; return None
        self.histogram = np.sum(frame[int(frame.shape[0] / 2):, :], axis=0)
        if plot:
            try:
                import matplotlib.pyplot as plt
                fig, (ax1, ax2) = plt.subplots(2, 1); fig.set_size_inches(10, 5)
                ax1.imshow(frame, cmap='gray'); ax1.set_title("Warped Binary Frame for Histogram")
                ax2.plot(self.histogram); ax2.set_title("Histogram Peaks"); plt.show()
            except Exception as e: print(f"Fehler beim Plotten des Histogramms: {e}")
        return self.histogram

    def display_curvature_offset(self, frame=None, plot=False):
        #... (Code wie zuvor)
        image_copy = frame.copy() if frame is not None else self.orig_frame.copy()
        h, w = image_copy.shape[:2]
        font_scale = float(0.6 * w / 640); thickness = max(1, int(2 * w / 640))
        pos_curve = (int(10 * w / 640), int(30 * h / 480)); pos_offset = (int(10 * w / 640), int(60 * h / 480))
        curve_radius_text = "Curve Radius: N/A"
        valid_curves = [c for c in [self.left_curvem, self.right_curvem] if c is not None and c != float('inf')]
        if valid_curves:
             avg_curve = np.mean(valid_curves)
             curve_radius_text = "Curve Radius: straight" if avg_curve > 5000 else f"Curve Radius: {avg_curve:.1f} m"
        elif self.left_curvem == float('inf') or self.right_curvem == float('inf'): curve_radius_text = "Curve Radius: straight"
        cv2.putText(image_copy, curve_radius_text, pos_curve, cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), thickness, cv2.LINE_AA)
        offset_text = "Center Offset: N/A"
        if self.center_offset is not None: offset_text = f"Center Offset: {self.center_offset:.3f}" # Einheit unklar
        cv2.putText(image_copy, offset_text, pos_offset, cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), thickness, cv2.LINE_AA)
        if plot:
            try: cv2.imshow("Image with Curvature and Offset", image_copy); cv2.waitKey(1)
            except Exception as e: print(f"Fehler beim Anzeigen von Curvature/Offset: {e}")
        return image_copy


    def get_lane_line_previous_window(self, left_fit, right_fit, plot=False):
        #... (Code wie zuvor, nutzt self.minpix)
        if left_fit is None or right_fit is None or self.filtered_warped_frame is None:
            self.left_fit = None; self.right_fit = None; self.leftx = None; self.lefty = None; self.rightx = None; self.righty = None; self.left_fitx = None; self.right_fitx = None; self.ploty = None
            return
        margin = self.margin; minpix = self.minpix
        nonzero = self.filtered_warped_frame.nonzero(); nonzeroy = np.array(nonzero[0]); nonzerox = np.array(nonzero[1])
        left_lane_inds = ((nonzerox > (left_fit[0]*(nonzeroy**2)+left_fit[1]*nonzeroy+left_fit[2]-margin)) & (nonzerox < (left_fit[0]*(nonzeroy**2)+left_fit[1]*nonzeroy+left_fit[2]+margin)))
        right_lane_inds = ((nonzerox > (right_fit[0]*(nonzeroy**2)+right_fit[1]*nonzeroy+right_fit[2]-margin)) & (nonzerox < (right_fit[0]*(nonzeroy**2)+right_fit[1]*nonzeroy+right_fit[2]+margin)))
        self.left_lane_inds = left_lane_inds; self.right_lane_inds = right_lane_inds
        leftx = nonzerox[left_lane_inds]; lefty = nonzeroy[left_lane_inds]; rightx = nonzerox[right_lane_inds]; righty = nonzeroy[right_lane_inds]
        self.leftx = leftx; self.lefty = lefty; self.rightx = rightx; self.righty = righty
        if leftx.size > minpix and lefty.size > minpix and rightx.size > minpix and righty.size > minpix:
            try:
                new_left_fit = np.polyfit(lefty, leftx, 2); new_right_fit = np.polyfit(righty, rightx, 2)
                use_smoothing = False # Glättung aus
                if use_smoothing:
                    alpha = 0.2
                    if self.previous_left_fit is not None: self.left_fit = alpha * new_left_fit + (1 - alpha) * self.previous_left_fit
                    else: self.left_fit = new_left_fit
                    if self.previous_right_fit is not None: self.right_fit = alpha * new_right_fit + (1 - alpha) * self.previous_right_fit
                    else: self.right_fit = new_right_fit
                else:
                     self.left_fit = new_left_fit; self.right_fit = new_right_fit
                self.previous_left_fit = self.left_fit; self.previous_right_fit = self.right_fit
                self.ploty = np.linspace(0, self.filtered_warped_frame.shape[0]-1, self.filtered_warped_frame.shape[0])
                self.left_fitx = self.left_fit[0]*self.ploty**2 + self.left_fit[1]*self.ploty + self.left_fit[2]
                self.right_fitx = self.right_fit[0]*self.ploty**2 + self.right_fit[1]*self.ploty + self.right_fit[2]
            except (np.linalg.LinAlgError, TypeError):
                self.left_fit = self.previous_left_fit; self.right_fit = self.previous_right_fit
                if self.left_fit is not None and self.right_fit is not None and self.ploty is not None:
                     self.left_fitx = self.left_fit[0]*self.ploty**2 + self.left_fit[1]*self.ploty + self.left_fit[2]
                     self.right_fitx = self.right_fit[0]*self.ploty**2 + self.right_fit[1]*self.ploty + self.right_fit[2]
                else: self.left_fitx = None; self.right_fitx = None; self.ploty = None
            if plot: # Plotting-Code unverändert
                try:
                    out_img = np.dstack((self.filtered_warped_frame, self.filtered_warped_frame, self.filtered_warped_frame)) * 255
                    window_img = np.zeros_like(out_img)
                    out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
                    out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 0, 255]
                    margin_mask_left = np.zeros_like(self.filtered_warped_frame); margin_mask_right = np.zeros_like(self.filtered_warped_frame)
                    pts_left = np.array([np.transpose(np.vstack([self.left_fitx - margin, self.ploty]))]); pts_left_outer = np.array([np.flipud(np.transpose(np.vstack([self.left_fitx + margin, self.ploty])))])
                    pts_left_poly = np.hstack((pts_left, pts_left_outer))
                    pts_right = np.array([np.transpose(np.vstack([self.right_fitx - margin, self.ploty]))]); pts_right_outer = np.array([np.flipud(np.transpose(np.vstack([self.right_fitx + margin, self.ploty])))])
                    pts_right_poly = np.hstack((pts_right, pts_right_outer))
                    cv2.fillPoly(window_img, np.int_([pts_left_poly]), (0,255, 0)); cv2.fillPoly(window_img, np.int_([pts_right_poly]), (0,255, 0))
                    result = cv2.addWeighted(out_img, 1, window_img, 0.3, 0)
                    pts_left_line = np.array([np.transpose(np.vstack([self.left_fitx, self.ploty]))]); pts_right_line = np.array([np.transpose(np.vstack([self.right_fitx, self.ploty]))])
                    cv2.polylines(result, np.int_([pts_left_line]), isClosed=False, color=(255,255,0), thickness=2); cv2.polylines(result, np.int_([pts_right_line]), isClosed=False, color=(255,255,0), thickness=2)
                    cv2.imshow("Previous Window Search", result); cv2.waitKey(1)
                except Exception as e: print(f"Fehler beim Plotten von Previous Window: {e}")
        else: # Nicht genug Pixel gefunden
            self.left_fit = self.previous_left_fit; self.right_fit = self.previous_right_fit
            if self.left_fit is None or self.right_fit is None:
                 self.leftx=None; self.lefty=None; self.rightx=None; self.righty=None; self.left_fitx=None; self.right_fitx=None; self.ploty=None

    def get_lane_line_indices_sliding_windows(self, plot=False, **params):
        # --- Code wie zuvor, stellt sicher, dass Debug-Bild zurückgegeben wird ---
        if self.filtered_warped_frame is None or self.histogram is None:
             self.left_fit = None; self.right_fit = None; self.sliding_window_debug_img = None
             return None, None, None # Fit links, Fit rechts, Debug Bild

        margin = self.margin; minpix = self.minpix
        frame_sliding_window = self.filtered_warped_frame.copy()
        out_img = None
        if plot: out_img = np.dstack((frame_sliding_window, frame_sliding_window, frame_sliding_window)) * 255

        window_height = int(self.filtered_warped_frame.shape[0] / self.no_of_windows)
        nonzero = frame_sliding_window.nonzero(); nonzeroy = np.array(nonzero[0]); nonzerox = np.array(nonzero[1])

        target_distance = params.get('histogram_peak_target_distance_factor', 0.4) * self.width
        min_peak_value = params.get('histogram_min_peak_value', 5000) # Wert aus Original
        distance_tolerance = params.get('histogram_distance_tolerance_factor', 0.1) * self.width

        peaks = self.select_lane_peaks(self.histogram, target_distance=target_distance,
                                      min_peak_value=min_peak_value, distance_tolerance=distance_tolerance)

        if peaks is None or peaks[0] is None or peaks[1] is None:
            if self.previous_left_fit is not None and self.previous_right_fit is not None:
                 self.left_fit = self.previous_left_fit; self.right_fit = self.previous_right_fit
                 if self.ploty is None: self.ploty = np.linspace(0, self.filtered_warped_frame.shape[0]-1, self.filtered_warped_frame.shape[0])
                 try:
                      self.left_fitx = self.left_fit[0]*self.ploty**2 + self.left_fit[1]*self.ploty + self.left_fit[2]
                      self.right_fitx = self.right_fit[0]*self.ploty**2 + self.right_fit[1]*self.ploty + self.right_fit[2]
                 except (TypeError, IndexError): self.left_fit=None; self.right_fit=None; self.left_fitx=None; self.right_fitx=None; self.ploty=None
                 return self.left_fit, self.right_fit, self.sliding_window_debug_img
            else:
                 self.left_fit = None; self.right_fit = None; self.sliding_window_debug_img = None
                 return None, None, None

        leftx_base, rightx_base = peaks
        leftx_current = leftx_base; rightx_current = rightx_base
        left_lane_inds = []; right_lane_inds = []

        for window in range(self.no_of_windows):
            win_y_low = self.filtered_warped_frame.shape[0] - (window+1)*window_height; win_y_high = self.filtered_warped_frame.shape[0] - window*window_height
            win_xleft_low = int(leftx_current - margin); win_xleft_high = int(leftx_current + margin)
            win_xright_low = int(rightx_current - margin); win_xright_high = int(rightx_current + margin)
            if plot and out_img is not None:
                cv2.rectangle(out_img, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high), (0, 255, 0), 2)
                cv2.rectangle(out_img, (win_xright_low, win_y_low), (win_xright_high, win_y_high), (0, 255, 0), 2)
            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]
            left_lane_inds.append(good_left_inds); right_lane_inds.append(good_right_inds)
            if len(good_left_inds) > minpix: leftx_current = np.int_(np.mean(nonzerox[good_left_inds]))
            if len(good_right_inds) > minpix: rightx_current = np.int_(np.mean(nonzerox[good_right_inds]))

        try: left_lane_inds = np.concatenate(left_lane_inds); right_lane_inds = np.concatenate(right_lane_inds)
        except ValueError: left_lane_inds = np.array([], dtype=int); right_lane_inds = np.array([], dtype=int)

        leftx = nonzerox[left_lane_inds]; lefty = nonzeroy[left_lane_inds]; rightx = nonzerox[right_lane_inds]; righty = nonzeroy[right_lane_inds]
        self.leftx=leftx; self.lefty=lefty; self.rightx=rightx; self.righty=righty

        new_left_fit, new_right_fit = None, None
        if leftx.size > minpix: # Use minpix consistent with previous_window check
            try: new_left_fit = np.polyfit(lefty, leftx, 2)
            except (np.linalg.LinAlgError, TypeError): new_left_fit = None
        if rightx.size > minpix:
            try: new_right_fit = np.polyfit(righty, rightx, 2)
            except (np.linalg.LinAlgError, TypeError): new_right_fit = None

        use_smoothing = False # Glättung aus
        if new_left_fit is not None:
             if use_smoothing and self.previous_left_fit is not None: self.left_fit = 0.2 * new_left_fit + 0.8 * self.previous_left_fit
             else: self.left_fit = new_left_fit
             self.previous_left_fit = self.left_fit
        elif self.previous_left_fit is not None: self.left_fit = self.previous_left_fit
        else: self.left_fit = None
        if new_right_fit is not None:
             if use_smoothing and self.previous_right_fit is not None: self.right_fit = 0.2 * new_right_fit + 0.8 * self.previous_right_fit
             else: self.right_fit = new_right_fit
             self.previous_right_fit = self.right_fit
        elif self.previous_right_fit is not None: self.right_fit = self.previous_right_fit
        else: self.right_fit = None

        if self.left_fit is not None and self.right_fit is not None:
            if self.ploty is None or len(self.ploty) != frame_sliding_window.shape[0]: self.ploty = np.linspace(0, frame_sliding_window.shape[0]-1, frame_sliding_window.shape[0])
            try:
                self.left_fitx = self.left_fit[0]*self.ploty**2 + self.left_fit[1]*self.ploty + self.left_fit[2]
                self.right_fitx = self.right_fit[0]*self.ploty**2 + self.right_fit[1]*self.ploty + self.right_fit[2]
            except (TypeError, IndexError): self.left_fitx=None; self.right_fitx=None; self.ploty=None; self.left_fit=None; self.right_fit=None
        else: self.left_fitx=None; self.right_fitx=None; self.ploty=None

        if plot and out_img is not None:
            try:
                out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]; out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 0, 255]
                if self.left_fitx is not None and self.ploty is not None:
                    pts_left = np.array([np.transpose(np.vstack([self.left_fitx, self.ploty]))]); cv2.polylines(out_img, np.int_([pts_left]), isClosed=False, color=(255,255,0), thickness=3)
                if self.right_fitx is not None and self.ploty is not None:
                    pts_right = np.array([np.transpose(np.vstack([self.right_fitx, self.ploty]))]); cv2.polylines(out_img, np.int_([pts_right]), isClosed=False, color=(255,255,0), thickness=3)
                self.sliding_window_debug_img = out_img
            except Exception as e: print(f"Fehler beim Erstellen des Sliding Window Debug-Bildes: {e}"); self.sliding_window_debug_img = None
        else: self.sliding_window_debug_img = None

        return self.left_fit, self.right_fit, self.sliding_window_debug_img

    def get_line_markings(self, frame=None, **params):
        # --- Code wie zuvor, verwendet block_size, c_value aus params ---
        if frame is None: frame = self.orig_frame
        if frame is None: return None
        hls = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)
        # Verwendung von Parametern aus **params (die vom Node kommen)
        # Diese Version verwendet adaptiveThreshold und gradient/color thresholds wie im Original
        _, sxbinary = edge.threshold(hls[:, :, 1], thresh=(120, 255)) # L channel threshold
        sxbinary = edge.blur_gaussian(sxbinary, ksize=3)
        sxbinary = edge.mag_thresh(sxbinary, sobel_kernel=3, thresh=(110, 255)) # Gradient magnitude
        s_channel = hls[:, :, 2] # S channel
        s_binary = cv2.adaptiveThreshold(s_channel, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                         cv2.THRESH_BINARY,
                                         params.get('block_size', 11), # aus Node params
                                         params.get('c_value', 200))      # aus Node params (war 200 im Ur-Original?)
        _, r_thresh = edge.threshold(frame[:, :, 2], thresh=(120, 255)) # R channel threshold
        rs_binary = cv2.bitwise_and(s_binary, r_thresh) # Combine S and R
        self.lane_line_markings = cv2.bitwise_or(rs_binary, sxbinary.astype(np.uint8)) # Combine color and gradient
        return self.lane_line_markings


    @staticmethod
    def select_lane_peaks(histogram, target_distance, min_peak_value, distance_tolerance):
        # --- Code wie zuvor ---
        if histogram is None: return None, None
        peaks_indices = []
        for i in range(1, len(histogram)-1):
            if histogram[i] > histogram[i-1] and histogram[i] >= histogram[i+1] and histogram[i] >= min_peak_value:
                peaks_indices.append(i)
        if len(peaks_indices) < 2: return None, None
        best_pair = None; min_diff_to_target = float('inf')
        for i in range(len(peaks_indices)):
            for j in range(i+1, len(peaks_indices)):
                peak1_idx = peaks_indices[i]; peak2_idx = peaks_indices[j]
                current_distance = abs(peak2_idx - peak1_idx)
                diff = abs(current_distance - target_distance)
                if diff <= distance_tolerance and diff < min_diff_to_target:
                        min_diff_to_target = diff; best_pair = (min(peak1_idx, peak2_idx), max(peak1_idx, peak2_idx))
        if best_pair is None and len(peaks_indices) >= 2: # Fallback
             sorted_peaks = sorted(peaks_indices, key=lambda idx: histogram[idx], reverse=True)
             strongest_pair = sorted(sorted_peaks[:2])
             if abs(strongest_pair[1] - strongest_pair[0]) > 0.1 * len(histogram): best_pair = (strongest_pair[0], strongest_pair[1])
        return best_pair[0] if best_pair else None, best_pair[1] if best_pair else None


    def overlay_lane_lines(self, plot=False):
        # --- Code wie zuvor ---
        if self.left_fit is None or self.right_fit is None or self.ploty is None or self.left_fitx is None or self.right_fitx is None or self.filtered_warped_frame is None or self.inv_transformation_matrix is None:
            return self.orig_frame
        warp_zero = np.zeros_like(self.filtered_warped_frame).astype(np.uint8); color_warp = np.dstack((warp_zero, warp_zero, warp_zero))
        pts_left = np.array([np.transpose(np.vstack([self.left_fitx, self.ploty]))]); pts_right = np.array([np.flipud(np.transpose(np.vstack([self.right_fitx, self.ploty])))])
        pts = np.hstack((pts_left, pts_right)); cv2.fillPoly(color_warp, np.int_([pts]), (0, 255, 0))
        try: newwarp = cv2.warpPerspective(color_warp, self.inv_transformation_matrix, (self.orig_frame.shape[1], self.orig_frame.shape[0]))
        except cv2.error as e: print(f"Fehler bei WarpPerspective im Overlay: {e}"); return self.orig_frame
        result = cv2.addWeighted(self.orig_frame, 1, newwarp, 0.3, 0)
        if self.current_center is not None and hasattr(self, 'y_bottom'):
            try: bottom_pixel_y = int(self.y_bottom); cv2.circle(result, (int(self.current_center), bottom_pixel_y), 5, (0, 0, 255), -1)
            except (ValueError, TypeError): pass
        if plot:
            try: cv2.imshow("Lane Overlay", result); cv2.waitKey(1)
            except Exception as e: print(f"Fehler beim Anzeigen des Overlays: {e}")
        return result


    def perspective_transform(self, frame=None, plot=False):
        # --- Code wie zuvor ---
        if frame is None: frame = self.lane_line_markings
        if frame is None or frame.ndim != 2: self.warped_frame=None; self.transformation_matrix=None; self.inv_transformation_matrix=None; return
        try:
             self.transformation_matrix = cv2.getPerspectiveTransform(self.roi_points, self.desired_roi_points)
             self.inv_transformation_matrix = cv2.getPerspectiveTransform(self.desired_roi_points, self.roi_points)
        except cv2.error as e: print(f"Fehler bei getPerspectiveTransform: {e}"); self.warped_frame=None; self.transformation_matrix=None; self.inv_transformation_matrix=None; return
        try: self.warped_frame = cv2.warpPerspective(frame, self.transformation_matrix, self.orig_image_size, flags=cv2.INTER_LINEAR)
        except cv2.error as e: print(f"Fehler bei warpPerspective: {e}"); self.warped_frame = None; return
        # Binarisierung nach Warp (wie im Original-Code)
        (thresh, binary_warped) = cv2.threshold(self.warped_frame, 127, 255, cv2.THRESH_BINARY)
        self.warped_frame = binary_warped
        # Maske anwenden (wie im Original-Code)
        mask = np.zeros_like(self.warped_frame, dtype=np.uint8); cv2.fillPoly(mask, [np.int32(self.desired_roi_points)], 255)
        self.warped_frame = cv2.bitwise_and(self.warped_frame, mask)
        if plot:
            try:
                warped_copy = self.warped_frame.copy(); warped_plot = cv2.polylines(warped_copy, [np.int32(self.desired_roi_points)], True, (147, 20, 255), 3)
                cv2.imshow('Warped Image', warped_plot); cv2.waitKey(1)
            except Exception as e: print(f"Fehler beim Anzeigen des Warped Image: {e}")

    def filter_lane_markings_by_thickness(self, plot=False, **params):
        # --- Code wie zuvor, verwendet min/max_thickness aus params ---
        if self.warped_frame is None: self.filtered_warped_frame = None; return
        min_thickness = params.get('min_thickness', 2.5) # Standardwert aus Original
        max_thickness = params.get('max_thickness', 5.0) # Standardwert aus Original
        contours, hierarchy = cv2.findContours(self.warped_frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        filtered_mask = np.zeros_like(self.warped_frame); contours_drawn = 0
        for contour in contours:
            area = cv2.contourArea(contour); perimeter = cv2.arcLength(contour, True)
            if perimeter == 0 or area < 5: continue
            thickness_ratio = area / perimeter # Einfache Heuristik
            if min_thickness <= thickness_ratio <= max_thickness:
                cv2.drawContours(filtered_mask, [contour], -1, 255, thickness=cv2.FILLED); contours_drawn += 1
        self.filtered_warped_frame = filtered_mask
        if plot:
            try: cv2.imshow("Filtered Lane Markings (Thickness)", filtered_mask); cv2.waitKey(1)
            except Exception as e: print(f"Fehler beim Anzeigen der gefilterten Markierungen: {e}")


    def plot_roi(self, frame=None): # Entferne plot=False Argument, gibt jetzt immer Bild zurück
        # Gibt das Bild mit eingezeichnetem ROI zurück, zeigt es nicht mehr selbst an.
        if frame is None:
            # Stelle sicher, dass self.orig_frame existiert
            if self.orig_frame is None:
                return None # Kann nichts zeichnen ohne Originalbild
            frame = self.orig_frame.copy()
        else:
            # Kopie erstellen, um das Original nicht zu verändern
            frame = frame.copy()

        # Stelle sicher, dass roi_points existieren (sollten in update_frame gesetzt werden)
        if not hasattr(self, 'roi_points') or self.roi_points is None:
             print("WARNUNG: roi_points nicht in lane.py gefunden zum Zeichnen.")
             return frame # Gib das Originalbild zurück

        # Zeichne das ROI-Polygon auf das Bild
        try:
             roi_drawable = [np.int32(self.roi_points)] # Muss eine Liste von Arrays sein
             # Zeichne auf die Kopie
             cv2.polylines(frame, roi_drawable, True, (147, 20, 255), 2) # Lila Farbe, Dicke 2
             return frame # Gib das modifizierte Bild zurück
        except Exception as e:
             print(f"Fehler beim Plotten des ROI in lane.py: {e}")
             # Gib das unmodifizierte Frame zurück bei Fehler
             return frame
import cv2
import numpy as np
from . import edge_detection as edge
import matplotlib.pyplot as plt


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

        # Umrechnungsfaktoren von Pixel in Meter
        self.YM_PER_PIX = 10.0 / 1000

        # Krümmungsradien und Offset (später berechnet)
        self.left_curvem = None
        self.right_curvem = None
        self.center_offset = None

        self.current_center = None

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
        self.roi_points = np.float32([
            (w * params.get('roi_top_left_w'), h * params.get('roi_top_left_h')),   # top-left
            (w * params.get('roi_bottom_left_w'), h * params.get('roi_bottom_left_h')),     # bottom-left
            (w * params.get('roi_bottom_right_w'), h * params.get('roi_bottom_right_h')),     # bottom-right
            (w * params.get('roi_top_right_w'), h * params.get('roi_top_right_h'))     # top-right
        ])

        # Zielpunkte für die Perspektivtransformation
        self.padding = int(params.get(
            'desired_roi_padding_factor', 0.25) * width)
        self.desired_roi_points = np.float32([
            [self.padding, 0],
            [self.padding, self.orig_image_size[1]],
            [self.orig_image_size[0] - self.padding, self.orig_image_size[1]],
            [self.orig_image_size[0] - self.padding, 0]
        ])

        # Histogramm zur Bestimmung der Fahrspurbasis
        self.histogram = None

        # Parameter für die Sliding-Window-Methode
        self.no_of_windows = 10  # 10
        self.margin = int((1/12) * width)  # 1/12
        self.minpix = int((1/24) * width)

    def calculate_car_position(self, print_to_terminal=False, **params):
        """
        Berechnet den Offset (in cm) zwischen dem Fahrzeug (angenommen, Kamera ist im Bildzentrum)
        und der Fahrspurmitte. Als Referenz wird der geglättete Mittelwert am unteren Bildrand verwendet.
        """
        if self.left_fit is None or self.right_fit is None:
            return None

        y_bottom = self.orig_frame.shape[0] - 20

        # Berechne den aktuellen Mittelwert der Fahrspur am unteren Rand
        current_left_x = self.left_fit[0] * y_bottom**2 + \
            self.left_fit[1] * y_bottom + self.left_fit[2]
        current_right_x = self.right_fit[0] * y_bottom**2 + \
            self.right_fit[1] * y_bottom + self.right_fit[2]
        self.current_center = ((current_left_x + current_right_x) / 2)

        # Glätte den Mittelwert
        # self.smoothed_center = self.smooth_lane_center(current_center, alpha=0.1)

        # Kamera als Bildmitte annehmen
        car_location = self.orig_frame.shape[1] / 2

        center_offset = (abs(car_location) - abs(self.current_center)
                         ) * params.get('center_factor', 0.01)

        if print_to_terminal:
            print(f"Center Offset (smoothed): {center_offset} cm")

        self.center_offset = center_offset
        return center_offset

    def calculate_curvature(self, print_to_terminal=False):
        if (self.left_fit is None or self.right_fit is None or
            self.lefty is None or self.righty is None or
                len(self.lefty) == 0 or len(self.righty) == 0):
            if print_to_terminal:
                print("Keine Fahrspurlinie erkannt")
            self.left_curvem = None
            self.right_curvem = None
            return None, None

        y_eval = np.max(self.ploty)
        left_fit_cr = np.polyfit(
            self.lefty * self.YM_PER_PIX, self.leftx * 0.00475, 2)
        right_fit_cr = np.polyfit(
            self.righty * self.YM_PER_PIX, self.rightx * 0.00475, 2)
        left_curvem = (
            (1 + (2 * left_fit_cr[0] * y_eval * self.YM_PER_PIX + left_fit_cr[1])**2)**1.5) / abs(2 * left_fit_cr[0])
        right_curvem = (
            (1 + (2 * right_fit_cr[0] * y_eval * self.YM_PER_PIX + right_fit_cr[1])**2)**1.5) / abs(2 * right_fit_cr[0])

        if print_to_terminal:
            print(
                f"Left Curvature: {left_curvem} m, Right Curvature: {right_curvem} m")
        self.left_curvem = left_curvem
        self.right_curvem = right_curvem
        return left_curvem, right_curvem

    def calculate_histogram(self, frame=None, plot=False):
        if frame is None:
            frame = self.filtered_warped_frame
        self.histogram = np.sum(frame[int(frame.shape[0] / 2):, :], axis=0)
        if plot:
            fig, (ax1, ax2) = plt.subplots(2, 1)
            fig.set_size_inches(10, 5)
            ax1.imshow(frame, cmap='gray')
            ax1.set_title("Warped Binary Frame")
            ax2.plot(self.histogram)
            ax2.set_title("Histogram Peaks")
            plt.show()
        return self.histogram

    def display_curvature_offset(self, frame=None, plot=False):
        image_copy = frame.copy() if frame is not None else self.orig_frame.copy()
        if (self.left_curvem is not None and self.right_curvem is not None and
                self.center_offset is not None):
            cv2.putText(image_copy, 'Curve Radius: ' + str((self.left_curvem + self.right_curvem)/2)[:7] + ' m',
                        (int(5 * self.width / 600), int(20 * self.height / 338)),
                        cv2.FONT_HERSHEY_SIMPLEX, float(0.5 * self.width / 600), (255, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(image_copy, 'Center Offset: ' + str(self.center_offset)[:7],
                        (int(5 * self.width / 600), int(40 * self.height / 338)),
                        cv2.FONT_HERSHEY_SIMPLEX, float(0.5 * self.width / 600), (255, 255, 255), 2, cv2.LINE_AA)
        if plot:
            cv2.imshow("Image with Curvature and Offset", image_copy)
        return image_copy

    def get_lane_line_previous_window(self, left_fit, right_fit, plot=False):
        if left_fit is None or right_fit is None:
            return
        margin = self.margin
        nonzero = self.filtered_warped_frame.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        left_lane_inds = ((nonzerox > (left_fit[0]*(nonzeroy**2)+left_fit[1]*nonzeroy+left_fit[2]-margin)) &
                          (nonzerox < (left_fit[0]*(nonzeroy**2)+left_fit[1]*nonzeroy+left_fit[2]+margin)))
        right_lane_inds = ((nonzerox > (right_fit[0]*(nonzeroy**2)+right_fit[1]*nonzeroy+right_fit[2]-margin)) &
                           (nonzerox < (right_fit[0]*(nonzeroy**2)+right_fit[1]*nonzeroy+right_fit[2]+margin)))
        self.left_lane_inds = left_lane_inds
        self.right_lane_inds = right_lane_inds
        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds]
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]
        self.leftx = leftx
        self.lefty = lefty
        self.rightx = rightx
        self.righty = righty

        if leftx.size != 0 and lefty.size != 0 and rightx.size != 0 and righty.size != 0:
            left_fit = np.polyfit(lefty, leftx, 2)
            right_fit = np.polyfit(righty, rightx, 2)
            self.left_fit = left_fit
            self.right_fit = right_fit
            self.ploty = np.linspace(
                0, self.filtered_warped_frame.shape[0]-1, self.filtered_warped_frame.shape[0])
            self.left_fitx = left_fit[0]*self.ploty**2 + \
                left_fit[1]*self.ploty + left_fit[2]
            self.right_fitx = right_fit[0]*self.ploty**2 + \
                right_fit[1]*self.ploty + right_fit[2]
            if plot:
                out_img = np.dstack(
                    (self.filtered_warped_frame, self.filtered_warped_frame, self.filtered_warped_frame))*255
                out_img[nonzeroy[left_lane_inds],
                        nonzerox[left_lane_inds]] = [255, 0, 0]
                out_img[nonzeroy[right_lane_inds],
                        nonzerox[right_lane_inds]] = [0, 0, 255]
                cv2.imshow("Sliding Window Search", out_img)
                cv2.waitKey(1)
        else:
            self.left_fit = None
            self.right_fit = None

    def get_lane_line_indices_sliding_windows(self, plot=False):
        if self.filtered_warped_frame is None:
            return
        
        margin = self.margin

        frame_sliding_window = self.filtered_warped_frame.copy()

        window_height = int(self.filtered_warped_frame.shape[0] / self.no_of_windows)
        nonzero = self.filtered_warped_frame.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        peaks = self.select_lane_peaks(
            self.histogram, target_distance=250, min_peak_value=5000)

        if peaks is None or peaks[0] is None or peaks[1] is None:
            self.left_fit = None
            self.right_fit = None
            return None, None

        leftx_base, rightx_base = peaks

        leftx_current = leftx_base
        rightx_current = rightx_base
        left_lane_inds = []
        right_lane_inds = []

        for window in range(self.no_of_windows):
            win_y_low = self.filtered_warped_frame.shape[0] - (window+1)*window_height
            win_y_high = self.filtered_warped_frame.shape[0] - window*window_height
            win_xleft_low = leftx_current - margin
            win_xleft_high = leftx_current + margin
            win_xright_low = rightx_current - margin
            win_xright_high = rightx_current + margin
            cv2.rectangle(frame_sliding_window, (win_xleft_low, win_y_low),
                          (win_xleft_high, win_y_high), (255, 255, 255), 2)
            cv2.rectangle(frame_sliding_window, (win_xright_low, win_y_low),
                          (win_xright_high, win_y_high), (255, 255, 255), 2)
            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                              (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                               (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]
            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)

            if len(good_left_inds) > self.minpix:
                leftx_current = int(np.mean(nonzerox[good_left_inds]))
            if len(good_right_inds) > self.minpix:
                rightx_current = int(np.mean(nonzerox[good_right_inds]))

            # # Nur in eine Richtung verschieben
            # # Links
            # if len(good_left_inds) > self.minpix:
            #     mean_x = int(np.mean(nonzerox[good_left_inds]))
            #     if mean_x < leftx_current:
            #         leftx_current = mean_x  # nur weiter nach links erlaubt
            #     # else: bleib auf Position

            # # Rechts
            # if len(good_right_inds) > self.minpix:
            #     mean_x = int(np.mean(nonzerox[good_right_inds]))
            #     if mean_x > rightx_current:
            #         rightx_current = mean_x  # nur weiter nach rechts erlaubt
            #     # else: bleib auf Position

        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)
        if left_lane_inds.size == 0 or right_lane_inds.size == 0:
            self.left_fit = None
            self.right_fit = None
            return None, None
        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds]
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]
        try:
            left_fit = np.polyfit(lefty, leftx, 2)
            right_fit = np.polyfit(righty, rightx, 2)
        except Exception as e:
            self.left_fit = None
            self.right_fit = None
            return None, None
        self.left_fit = left_fit
        self.right_fit = right_fit

        if plot:

            out_img = cv2.cvtColor(frame_sliding_window, cv2.COLOR_GRAY2BGR)
            out_img[nonzeroy[left_lane_inds],
                    nonzerox[left_lane_inds]] = [255, 0, 0]
            out_img[nonzeroy[right_lane_inds],
                    nonzerox[right_lane_inds]] = [0, 0, 255]
            cv2.imshow("Sliding Window Search", out_img)
            cv2.waitKey(1)

        return self.left_fit, self.right_fit

    def get_line_markings(self, frame=None, **params):
        if frame is None:
            frame = self.orig_frame
        hls = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)
        _, sxbinary = edge.threshold(hls[:, :, 1], thresh=(120, 255))
        sxbinary = edge.blur_gaussian(sxbinary, ksize=3)
        sxbinary = edge.mag_thresh(sxbinary, sobel_kernel=3, thresh=(110, 255))
        s_channel = hls[:, :, 2]
        # _, s_binary = edge.threshold(s_channel, (80, 255))
        s_binary = cv2.adaptiveThreshold(s_channel, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                         cv2.THRESH_BINARY,
                                         params.get('block_size', 11),
                                         params.get('c_value', 200))
        _, r_thresh = edge.threshold(frame[:, :, 2], thresh=(120, 255))
        rs_binary = cv2.bitwise_and(s_binary, r_thresh)
        self.lane_line_markings = cv2.bitwise_or(
            rs_binary, sxbinary.astype(np.uint8))
        return self.lane_line_markings

    @staticmethod
    def select_lane_peaks(histogram, target_distance=250, min_peak_value=5000):
        peaks = []
        for i in range(1, len(histogram)-1):
            if histogram[i] > histogram[i-1] and histogram[i] >= histogram[i+1] and histogram[i] >= min_peak_value:
                peaks.append(i)
        if len(peaks) < 2:
            return None, None
        best_pair = None
        best_diff = float('inf')
        for i in range(len(peaks)):
            for j in range(i+1, len(peaks)):
                current_distance = abs(peaks[j]-peaks[i])
                diff = abs(current_distance - target_distance)
                if diff < best_diff and diff < 20:
                    best_diff = diff
                    best_pair = (min(peaks[i], peaks[j]),
                                 max(peaks[i], peaks[j]))

        # print(best_diff, flush=True)
        return best_pair

    def overlay_lane_lines(self, plot=False):
        if self.left_fit is None or self.right_fit is None:
            return self.orig_frame
        warp_zero = np.zeros_like(self.filtered_warped_frame).astype(np.uint8)
        color_warp = np.dstack((warp_zero, warp_zero, warp_zero))
        pts_left = np.array(
            [np.transpose(np.vstack([self.left_fitx, self.ploty]))])
        pts_right = np.array(
            [np.flipud(np.transpose(np.vstack([self.right_fitx, self.ploty])))])
        pts = np.hstack((pts_left, pts_right))
        cv2.fillPoly(color_warp, np.int_([pts]), (0, 255, 0))
        newwarp = cv2.warpPerspective(color_warp, self.inv_transformation_matrix,
                                      (self.orig_frame.shape[1], self.orig_frame.shape[0]))
        result = cv2.addWeighted(self.orig_frame, 1, newwarp, 0.3, 0)
        # Berechne die Fahrspurmitte am unteren Rand und markiere diese mit einem roten Punkt
        y_bottom = self.orig_frame.shape[0] - 1
        if self.previous_bottom_left_x is not None and self.previous_bottom_right_x is not None:
            lane_center = int((self.previous_bottom_left_x +
                              self.previous_bottom_right_x) / 2)
        else:
            current_left_x = self.left_fit[0]*y_bottom**2 + \
                self.left_fit[1]*y_bottom + self.left_fit[2]
            current_right_x = self.right_fit[0]*y_bottom**2 + \
                self.right_fit[1]*y_bottom + self.right_fit[2]
            lane_center = int((current_left_x + current_right_x) / 2)
        if self.current_center is not None:
            cv2.circle(result, (int(self.current_center),
                       y_bottom-20), 5, (0, 0, 255), -1)
        if plot:
            fig, (ax1, ax2) = plt.subplots(2, 1)
            fig.set_size_inches(10, 10)
            fig.tight_layout(pad=3.0)
            ax1.imshow(cv2.cvtColor(self.orig_frame, cv2.COLOR_BGR2RGB))
            ax2.imshow(cv2.cvtColor(result, cv2.COLOR_BGR2RGB))
            ax1.set_title("Original Frame")
            ax2.set_title("Frame mit Lane Overlay + roter Mittelpunkt")
            plt.show()
        return result

    def perspective_transform(self, frame=None, plot=False):
        if frame is None:
            frame = self.lane_line_markings
        self.transformation_matrix = cv2.getPerspectiveTransform(
            self.roi_points, self.desired_roi_points)
        self.inv_transformation_matrix = cv2.getPerspectiveTransform(
            self.desired_roi_points, self.roi_points)
        self.warped_frame = cv2.warpPerspective(
            frame, self.transformation_matrix, self.orig_image_size, flags=cv2.INTER_LINEAR)
        (thresh, binary_warped) = cv2.threshold(
            self.warped_frame, 127, 255, cv2.THRESH_BINARY)
        self.warped_frame = binary_warped
        mask = np.zeros_like(self.warped_frame, dtype=np.uint8)
        cv2.fillPoly(mask, [np.int32(self.desired_roi_points)], 255)
        self.warped_frame = cv2.bitwise_and(self.warped_frame, mask)
        if plot:
            warped_copy = self.warped_frame.copy()
            warped_plot = cv2.polylines(warped_copy, [np.int32(
                self.desired_roi_points)], True, (147, 20, 255), 3)
            cv2.imshow('Warped Image', warped_plot)
    
    def filter_lane_markings_by_thickness(self, plot=False, **params):       
        min_thickness = params.get('min_thickness')
        max_thickness = params.get('max_thickness')

        contours, _ = cv2.findContours(self.warped_frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        filtered_mask = np.zeros_like(self.warped_frame)

        for contour in contours:
            area = cv2.contourArea(contour)
            perimeter = cv2.arcLength(contour, True)
            if perimeter == 0:
                continue
            thickness = area / perimeter

            if min_thickness <= thickness <= max_thickness:
                cv2.drawContours(
                    filtered_mask, [contour], -1, 255, thickness=cv2.FILLED)

        self.filtered_warped_frame = filtered_mask

        if plot:
            cv2.imshow("Filtered Lane Markings", filtered_mask)
            cv2.waitKey(1)

    def plot_roi(self, frame=None, plot=False):
        if not plot:
            return
        if frame is None:
            frame = self.orig_frame.copy()
        this_image = cv2.polylines(
            frame, [np.int32(self.roi_points)], True, (147, 20, 255), 2)
        cv2.imshow('ROI Image', this_image)
        cv2.waitKey(1)

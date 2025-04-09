#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2
import edge_detection as edge  # Dein Modul zur Kantenextraktion
#import matplotlib.pyplot as plt  # Für Debugging/Plotten (im Node auskommentiert)

# --- Lane Detection Class -----------------------------------------------------
class Lane:
    """
    Repräsentiert eine Fahrspur (Lane) in einem Bild.
    """
    def __init__(self, orig_frame):
        """
        Konstruktor mit Eingangsbild.
        :param orig_frame: Ursprüngliches Bild (BGR)
        """
        self.orig_frame = orig_frame

        # Hier wird das Bild mit Fahrbahnmarkierungen gespeichert.
        self.lane_line_markings = None

        # Nach der Perspektivtransformation.
        self.warped_frame = None
        self.transformation_matrix = None
        self.inv_transformation_matrix = None

        # Bildgröße (Breite, Höhe) des Eingangsbildes
        self.orig_image_size = self.orig_frame.shape[::-1][1:]
        width = self.orig_image_size[0]
        height = self.orig_image_size[1]
        self.width = width
        self.height = height

        # Vier Ecken des trapezförmigen Regions of Interest (ROI)
        self.roi_points = np.float32([
            (274, 184),  # Top-Left
            (0, 337),    # Bottom-Left
            (575, 337),  # Bottom-Right
            (371, 184)   # Top-Right
        ])

        # Zielpunkte (nach der Perspektivtransformation)
        self.padding = int(0.25 * width)
        self.desired_roi_points = np.float32([
            [self.padding, 0],
            [self.padding, self.orig_image_size[1]],
            [self.orig_image_size[0]-self.padding, self.orig_image_size[1]],
            [self.orig_image_size[0]-self.padding, 0]
        ])

        # Histogramm für die Erkennung von weißen Pixelspitzen
        self.histogram = None

        # Parameter für die Sliding-Window-Methode
        self.no_of_windows = 10
        self.margin = int((1/12) * width)
        self.minpix = int((1/24) * width)

        # Polynome für linke und rechte Fahrbahnmarkierung
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

        # Pixel zu realen Maßen
        self.YM_PER_PIX = 10.0 / 1000    # Meter pro Pixel in y-Richtung
        self.XM_PER_PIX = 3.7 / 781      # Meter pro Pixel in x-Richtung

        # Radius der Krümmung und Versatz
        self.left_curvem = None
        self.right_curvem = None
        self.center_offset = None

    def calculate_car_position(self, print_to_terminal=False):
        """
        Berechnet die Position des Fahrzeugs relativ zur Bildmitte.
        :param print_to_terminal: Falls True, wird das Ergebnis auch ausgegeben.
        :return: Offset in Zentimetern
        """
        car_location = self.orig_frame.shape[1] / 2
        height = self.orig_frame.shape[0]
        bottom_left = self.left_fit[0]*height**2 + self.left_fit[1]*height + self.left_fit[2]
        bottom_right = self.right_fit[0]*height**2 + self.right_fit[1]*height + self.right_fit[2]
        center_lane = (bottom_right - bottom_left) / 2 + bottom_left
        center_offset = (np.abs(car_location) - np.abs(center_lane)) * self.XM_PER_PIX * 100

        if print_to_terminal:
            print(str(center_offset) + 'cm')

        self.center_offset = center_offset
        return center_offset

    def calculate_curvature(self, print_to_terminal=False):
        """
        Berechnet die Krümmung der Straße in Metern.
        :param print_to_terminal: Falls True, wird der Wert ausgegeben.
        :return: (linker Krümmungsradius, rechter Krümmungsradius)
        """
        y_eval = np.max(self.ploty)
        left_fit_cr = np.polyfit(self.lefty * self.YM_PER_PIX, self.leftx * self.XM_PER_PIX, 2)
        right_fit_cr = np.polyfit(self.righty * self.YM_PER_PIX, self.rightx * self.XM_PER_PIX, 2)
        left_curvem = ((1 + (2*left_fit_cr[0]*y_eval*self.YM_PER_PIX + left_fit_cr[1])**2)**1.5) / np.absolute(2*left_fit_cr[0])
        right_curvem = ((1 + (2*right_fit_cr[0]*y_eval*self.YM_PER_PIX + right_fit_cr[1])**2)**1.5) / np.absolute(2*right_fit_cr[0])
        if print_to_terminal:
            print(left_curvem, 'm', right_curvem, 'm')
        self.left_curvem = left_curvem
        self.right_curvem = right_curvem
        return left_curvem, right_curvem

    def calculate_histogram(self, frame=None, plot=False):
        """
        Berechnet das Histogramm des unteren Bildteils (vom gefilterten Bild).
        :param frame: Eingabebild (nach Perspektivtransformation)
        :param plot: Falls True, wird ein Plot angezeigt (normalerweise False in ROS2)
        :return: Histogramm als 1D-Array
        """
        if frame is None:
            frame = self.warped_frame
        self.histogram = np.sum(frame[int(frame.shape[0]/2):, :], axis=0)
        if plot:
            # Debug-Plot (nicht in ROS2 verwendet)
            import matplotlib.pyplot as plt
            figure, (ax1, ax2) = plt.subplots(2, 1)
            figure.set_size_inches(10, 5)
            ax1.imshow(frame, cmap='gray')
            ax1.set_title("Warped Binary Frame")
            ax2.plot(self.histogram)
            ax2.set_title("Histogram Peaks")
            plt.show()
        return self.histogram

    def display_curvature_offset(self, frame=None, plot=False):
        """
        Überlagert Kurvenradius und Versatz auf das Originalbild.
        :param frame: Bild, auf dem gezeichnet wird (Standard: Originalbild)
        :param plot: Falls True, wird das Bild angezeigt (in ROS2 also False)
        :return: Annotiertes Bild
        """
        image_copy = frame.copy() if frame is not None else self.orig_frame.copy()
        cv2.putText(image_copy, 'Curve Radius: '+str(((self.left_curvem+self.right_curvem)/2))[:7]+' m',
                    (int((5/600)*self.width), int((20/338)*self.height)),
                    cv2.FONT_HERSHEY_SIMPLEX, float((0.5/600)*self.width), (255,255,255), 2, cv2.LINE_AA)
        cv2.putText(image_copy, 'Center Offset: '+str(self.center_offset)[:7]+' cm',
                    (int((5/600)*self.width), int((40/338)*self.height)),
                    cv2.FONT_HERSHEY_SIMPLEX, float((0.5/600)*self.width), (255,255,255), 2, cv2.LINE_AA)
        if plot:
            cv2.imshow("Image with Curvature and Offset", image_copy)
        return image_copy

    def get_lane_line_previous_window(self, left_fit, right_fit, plot=False):
        """
        Verfeinert die Linien anhand der vorherigen Fenster.
        :param left_fit: Polynomausgleich für die linke Linie
        :param right_fit: Polynomausgleich für die rechte Linie
        :param plot: Debug-Anzeige (standardmäßig False)
        """
        margin = self.margin
        nonzero = self.warped_frame.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        left_lane_inds = ((nonzerox > (left_fit[0]*(nonzeroy**2) +
                            left_fit[1]*nonzeroy + left_fit[2] - margin)) &
                          (nonzerox < (left_fit[0]*(nonzeroy**2) +
                            left_fit[1]*nonzeroy + left_fit[2] + margin)))
        right_lane_inds = ((nonzerox > (right_fit[0]*(nonzeroy**2) +
                             right_fit[1]*nonzeroy + right_fit[2] - margin)) &
                           (nonzerox < (right_fit[0]*(nonzeroy**2) +
                             right_fit[1]*nonzeroy + right_fit[2] + margin)))
        self.left_lane_inds = left_lane_inds
        self.right_lane_inds = right_lane_inds
        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds]
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]
        self.leftx = leftx
        self.rightx = rightx
        self.lefty = lefty
        self.righty = righty
        left_fit = np.polyfit(lefty, leftx, 2)
        right_fit = np.polyfit(righty, rightx, 2)
        self.left_fit = left_fit
        self.right_fit = right_fit
        ploty = np.linspace(0, self.warped_frame.shape[0]-1, self.warped_frame.shape[0])
        left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
        right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]
        self.ploty = ploty
        self.left_fitx = left_fitx
        self.right_fitx = right_fitx

        if plot:
            # Debug-Darstellung (nicht in ROS2 genutzt)
            out_img = np.dstack((self.warped_frame, self.warped_frame, self.warped_frame))*255
            window_img = np.zeros_like(out_img)
            out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
            out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 0, 255]
            import matplotlib.pyplot as plt
            figure, (ax1, ax2, ax3) = plt.subplots(3, 1)
            figure.set_size_inches(10, 10)
            figure.tight_layout(pad=3.0)
            ax1.imshow(cv2.cvtColor(self.orig_frame, cv2.COLOR_BGR2RGB))
            ax2.imshow(self.warped_frame, cmap='gray')
            ax3.imshow(out_img)
            ax3.plot(left_fitx, ploty, color='yellow')
            ax3.plot(right_fitx, ploty, color='yellow')
            ax1.set_title("Original Frame")
            ax2.set_title("Warped Frame")
            ax3.set_title("Warped Frame With Search Window")
            plt.show()

    def get_lane_line_indices_sliding_windows(self, plot=False):
        """
        Sucht die Pixel-Indizes der Linien mit der Sliding-Window-Methode.
        :param plot: Debug-Anzeige (standardmäßig False)
        :return: linkes und rechtes Polyfit
        """
        margin = self.margin
        frame_sliding_window = self.warped_frame.copy()
        window_height = np.int(self.warped_frame.shape[0]/self.no_of_windows)
        nonzero = self.warped_frame.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        left_lane_inds = []
        right_lane_inds = []
        leftx_base, rightx_base = self.histogram_peak()
        leftx_current = leftx_base
        rightx_current = rightx_base
        for window in range(self.no_of_windows):
            win_y_low = self.warped_frame.shape[0] - (window + 1) * window_height
            win_y_high = self.warped_frame.shape[0] - window * window_height
            win_xleft_low = leftx_current - margin
            win_xleft_high = leftx_current + margin
            win_xright_low = rightx_current - margin
            win_xright_high = rightx_current + margin
            cv2.rectangle(frame_sliding_window, (win_xleft_low, win_y_low),
                          (win_xleft_high, win_y_high), (255,255,255), 2)
            cv2.rectangle(frame_sliding_window, (win_xright_low, win_y_low),
                          (win_xright_high, win_y_high), (255,255,255), 2)
            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                              (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                               (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]
            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)
            if len(good_left_inds) > self.minpix:
                leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
            if len(good_right_inds) > self.minpix:
                rightx_current = np.int(np.mean(nonzerox[good_right_inds]))
        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)
        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds]
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]
        left_fit = np.polyfit(lefty, leftx, 2)
        right_fit = np.polyfit(righty, rightx, 2)
        self.left_fit = left_fit
        self.right_fit = right_fit

        if plot:
            ploty = np.linspace(0, frame_sliding_window.shape[0]-1, frame_sliding_window.shape[0])
            left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
            right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]
            out_img = np.dstack((frame_sliding_window, frame_sliding_window, frame_sliding_window))*255
            out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255,0,0]
            out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0,0,255]
            import matplotlib.pyplot as plt
            figure, (ax1, ax2, ax3) = plt.subplots(3, 1)
            figure.set_size_inches(10, 10)
            figure.tight_layout(pad=3.0)
            ax1.imshow(cv2.cvtColor(self.orig_frame, cv2.COLOR_BGR2RGB))
            ax2.imshow(frame_sliding_window, cmap='gray')
            ax3.imshow(out_img)
            ax3.plot(left_fitx, ploty, color='yellow')
            ax3.plot(right_fitx, ploty, color='yellow')
            ax1.set_title("Original Frame")
            ax2.set_title("Warped Frame with Sliding Windows")
            ax3.set_title("Detected Lane Lines with Sliding Windows")
            plt.show()
        return self.left_fit, self.right_fit

    def get_line_markings(self, frame=None):
        """
        Isoliert Fahrbahnmarkierungen.
        :param frame: Falls nicht angegeben, wird self.orig_frame genutzt.
        :return: Binäres Bild, in dem die markanten Kanten (Lane-Lines) hervorgehoben sind.
        """
        if frame is None:
            frame = self.orig_frame
        hls = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)
        # Wende Sobel-Operation und Schwellenwerten an
        _, sxbinary = edge.threshold(hls[:, :, 1], thresh=(120, 255))
        sxbinary = edge.blur_gaussian(sxbinary, ksize=3)
        sxbinary = edge.mag_thresh(sxbinary, sobel_kernel=3, thresh=(110, 255))
        s_channel = hls[:, :, 2]
        _, s_binary = edge.threshold(s_channel, (80, 255))
        _, r_thresh = edge.threshold(frame[:, :, 2], thresh=(120, 255))
        rs_binary = cv2.bitwise_and(s_binary, r_thresh)
        self.lane_line_markings = cv2.bitwise_or(rs_binary, sxbinary.astype(np.uint8))
        return self.lane_line_markings

    def histogram_peak(self):
        """
        Bestimmt den linken und rechten Peak im Histogramm.
        :return: (x-Koordinate des linken Peaks, x-Koordinate des rechten Peaks)
        """
        midpoint = np.int(self.histogram.shape[0]/2)
        leftx_base = np.argmax(self.histogram[:midpoint])
        rightx_base = np.argmax(self.histogram[midpoint:]) + midpoint
        return leftx_base, rightx_base

    def overlay_lane_lines(self, plot=False):
        """
        Legt die erkannten Fahrbahnmarkierungen auf das Originalbild.
        :param plot: Falls True, wird zur Debug-Anzeige geplottet.
        :return: Bild mit Überlagerung
        """
        warp_zero = np.zeros_like(self.warped_frame).astype(np.uint8)
        color_warp = np.dstack((warp_zero, warp_zero, warp_zero))
        pts_left = np.array([np.transpose(np.vstack([self.left_fitx, self.ploty]))])
        pts_right = np.array([np.flipud(np.transpose(np.vstack([self.right_fitx, self.ploty])))])
        pts = np.hstack((pts_left, pts_right))
        cv2.fillPoly(color_warp, np.int_([pts]), (0,255,0))
        newwarp = cv2.warpPerspective(color_warp, self.inv_transformation_matrix,
                                      (self.orig_frame.shape[1], self.orig_frame.shape[0]))
        result = cv2.addWeighted(self.orig_frame, 1, newwarp, 0.3, 0)
        if plot:
            import matplotlib.pyplot as plt
            figure, (ax1, ax2) = plt.subplots(2, 1)
            figure.set_size_inches(10, 10)
            figure.tight_layout(pad=3.0)
            ax1.imshow(cv2.cvtColor(self.orig_frame, cv2.COLOR_BGR2RGB))
            ax2.imshow(cv2.cvtColor(result, cv2.COLOR_BGR2RGB))
            ax1.set_title("Original Frame")
            ax2.set_title("Original Frame With Lane Overlay")
            plt.show()
        return result

    def perspective_transform(self, frame=None, plot=False):
        """
        Führt die Perspektivtransformation aus (Bird’s Eye View).
        :param frame: Wenn nicht angegeben, wird self.lane_line_markings genutzt.
        :param plot: Debug-Anzeige (normalerweise False in ROS2)
        :return: Warped (transformiertes) Bild
        """
        if frame is None:
            frame = self.lane_line_markings
        self.transformation_matrix = cv2.getPerspectiveTransform(self.roi_points, self.desired_roi_points)
        self.inv_transformation_matrix = cv2.getPerspectiveTransform(self.desired_roi_points, self.roi_points)
        self.warped_frame = cv2.warpPerspective(frame, self.transformation_matrix, self.orig_image_size, flags=cv2.INTER_LINEAR)
        (thresh, binary_warped) = cv2.threshold(self.warped_frame, 127, 255, cv2.THRESH_BINARY)
        self.warped_frame = binary_warped
        if plot:
            warped_copy = self.warped_frame.copy()
            warped_plot = cv2.polylines(warped_copy, np.int32([self.desired_roi_points]), True, (147,20,255), 3)
            cv2.imshow('Warped Image', warped_plot)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        return self.warped_frame

    def plot_roi(self, frame=None, plot=False):
        """
        Zeichnet das Region-of-Interest (ROI) im Bild.
        :param frame: Eingangsbild, falls nicht angegeben wird self.orig_frame genutzt.
        :param plot: Falls True, wird das Bild angezeigt.
        """
        if not plot:
            return
        if frame is None:
            frame = self.orig_frame.copy()
        this_image = cv2.polylines(frame, np.int32([self.roi_points]), True, (147,20,255), 3)
        cv2.imshow('ROI Image', this_image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

# --- ROS2 Node ---------------------------------------------------------------
class LaneDetectionNode(Node):
    def __init__(self):
        super().__init__('lane_detection_node')
        # Subscriber für das komprimierte Eingangssignal
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.image_callback,
            10)
        # Publisher für das annotierte Bild (als CompressedImage)
        self.publisher_ = self.create_publisher(CompressedImage, '/lane/image_annotated', 10)
        self.get_logger().info('Lane Detection Node gestartet.')

    def image_callback(self, msg: CompressedImage):
        # Dekodiere das komprimierte Bild in ein OpenCV-Bild (BGR)
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if cv_image is None:
            self.get_logger().warn('Fehler beim Dekodieren des Bildes.')
            return

        # Wende Lane-Detection-Pipeline an:
        # 1. Erzeuge ein Lane-Objekt
        lane_obj = Lane(orig_frame=cv_image)
        # 2. Isoliere Fahrbahnmarkierungen
        lane_obj.get_line_markings()
        # 3. Perspektivtransformation (Bird’s Eye View)
        lane_obj.perspective_transform(plot=False)
        # 4. Histogramm-Berechnung zum Auffinden der Pixelspitzen
        lane_obj.calculate_histogram(plot=False)
        # 5. Suche nach den Linien mittels Sliding Windows
        left_fit, right_fit = lane_obj.get_lane_line_indices_sliding_windows(plot=False)
        # 6. Feineinstellung über vorheriges Fenster
        lane_obj.get_lane_line_previous_window(left_fit, right_fit, plot=False)
        # 7. Overlay der erkannten Linien auf das Originalbild
        frame_with_lane_lines = lane_obj.overlay_lane_lines(plot=False)
        # 8. Berechnung von Krümmung und Fahrzeugversatz (optional: Ausgabe auf Konsole)
        lane_obj.calculate_curvature(print_to_terminal=False)
        lane_obj.calculate_car_position(print_to_terminal=False)
        annotated_frame = lane_obj.display_curvature_offset(frame=frame_with_lane_lines, plot=False)

        # Kodieren des annotierten Bildes als JPEG
        ret, buffer = cv2.imencode('.jpg', annotated_frame)
        if not ret:
            self.get_logger().error("Fehler beim Kodieren des annotierten Bildes.")
            return
        # Erstelle eine CompressedImage-Nachricht
        out_msg = CompressedImage()
        out_msg.header.stamp = self.get_clock().now().to_msg()
        out_msg.format = "jpeg"
        out_msg.data = buffer.tobytes()
        # Publiziere das annotierte Bild
        self.publisher_.publish(out_msg)
        self.get_logger().debug("Publizierte ein annotiertes Bild.")

def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

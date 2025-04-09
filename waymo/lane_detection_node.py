#!/usr/bin/env python3
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2
from . import edge_detection as edge # Dein Modul zur Kantenextraktion
#import matplotlib.pyplot as plt  # Für Debugging/Plotten (im Node auskommentiert)
import sys # Importiere sys für sys.exit

# --- Lane Detection Class -----------------------------------------------------
class Lane:
    """
    Repräsentiert eine Fahrspur (Lane) in einem Bild.
    """
    def __init__(self, orig_frame, logger): # Logger hinzugefügt
        """
        Konstruktor mit Eingangsbild.
        :param orig_frame: Ursprüngliches Bild (BGR)
        :param logger: Der Logger des aufrufenden Nodes
        """
        self.orig_frame = orig_frame
        self.logger = logger # Logger speichern

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
        h, w = self.orig_frame.shape[:2]
        # Diese Punkte müssen möglicherweise an dein spezifisches Kamerabild angepasst werden!
        self.roi_points = np.float32([
            (w * 0.45, h * 0.6),   # top-left
            (w * 0.05, h * 0.95),  # bottom-left
            (w * 0.95, h * 0.95),  # bottom-right
            (w * 0.55, h * 0.6)    # top-right
        ])

        # Zielpunkte (nach der Perspektivtransformation)
        # Diese Punkte definieren die "Draufsicht"
        self.padding = int(0.25 * width) # Padding anpassen, falls nötig
        self.desired_roi_points = np.float32([
            [self.padding, 0],                                   # top-left
            [self.padding, self.orig_image_size[1]],             # bottom-left
            [self.orig_image_size[0]-self.padding, self.orig_image_size[1]], # bottom-right
            [self.orig_image_size[0]-self.padding, 0]            # top-right
        ])

        # Histogramm für die Erkennung von weißen Pixelspitzen
        self.histogram = None

        # Parameter für die Sliding-Window-Methode
        self.no_of_windows = 10 # Anzahl der Fenster
        self.margin = int((1/12) * width) # Suchbreite um die Linie (Pixel)
        self.minpix = int((1/24) * width) # Mindestanzahl Pixel pro Fenster zum Zentrieren

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

        # Pixel zu realen Maßen (diese Werte sind oft fahrzeug-/kameraspezifisch!)
        self.YM_PER_PIX = 10.0 / 720    # Meter pro Pixel in y-Richtung (anpassen!)
        self.XM_PER_PIX = 3.7 / 700     # Meter pro Pixel in x-Richtung (anpassen!)

        # Radius der Krümmung und Versatz
        self.left_curvem = None
        self.right_curvem = None
        self.center_offset = None

    # --- Hilfsfunktion für Logger ---
    def get_logger(self):
        return self.logger

    def calculate_car_position(self, print_to_terminal=False):
        """
        Berechnet die Position des Fahrzeugs relativ zur Bildmitte.
        :param print_to_terminal: Falls True, wird das Ergebnis auch ausgegeben.
        :return: Offset in Zentimetern oder None bei Fehler
        """
        # Prüfen, ob Fits gültig sind
        if self.left_fit is None or self.right_fit is None:
            self.get_logger().warn('calculate_car_position: Ungültige Fits.')
            self.center_offset = None
            return None

        try:
            car_location = self.orig_frame.shape[1] / 2
            height = self.orig_frame.shape[0]
            bottom_left = self.left_fit[0]*height**2 + self.left_fit[1]*height + self.left_fit[2]
            bottom_right = self.right_fit[0]*height**2 + self.right_fit[1]*height + self.right_fit[2]
            center_lane = (bottom_right - bottom_left) / 2 + bottom_left
            center_offset = (np.abs(car_location) - np.abs(center_lane)) * self.XM_PER_PIX * 100

            if print_to_terminal:
                print(f'Offset: {center_offset:.2f} cm')

            self.center_offset = center_offset
            return center_offset
        except Exception as e:
            self.get_logger().error(f'Fehler in calculate_car_position: {e}')
            self.center_offset = None
            return None


    def calculate_curvature(self, print_to_terminal=False):
        """
        Berechnet die Krümmung der Straße in Metern.
        :param print_to_terminal: Falls True, wird der Wert ausgegeben.
        :return: (linker Krümmungsradius, rechter Krümmungsradius) oder (None, None) bei Fehler
        """
        # Prüfen, ob benötigte Daten vorhanden sind
        if self.ploty is None or self.lefty is None or self.leftx is None or \
           self.righty is None or self.rightx is None or \
           len(self.lefty) < 3 or len(self.righty) < 3: # Sicherstellen, dass genug Punkte da sind
            self.get_logger().warn('calculate_curvature: Ungültige oder fehlende Liniendaten.')
            self.left_curvem = None
            self.right_curvem = None
            return None, None

        try:
            y_eval = np.max(self.ploty) # Auswertung am unteren Bildrand

            # Fit polynomial in real-world space
            left_fit_cr = np.polyfit(self.lefty * self.YM_PER_PIX, self.leftx * self.XM_PER_PIX, 2)
            right_fit_cr = np.polyfit(self.righty * self.YM_PER_PIX, self.rightx * self.XM_PER_PIX, 2)

            # Calculate curvature radius
            left_curvem = ((1 + (2*left_fit_cr[0]*y_eval*self.YM_PER_PIX + left_fit_cr[1])**2)**1.5) / np.absolute(2*left_fit_cr[0])
            right_curvem = ((1 + (2*right_fit_cr[0]*y_eval*self.YM_PER_PIX + right_fit_cr[1])**2)**1.5) / np.absolute(2*right_fit_cr[0])

            if print_to_terminal:
                 print(f'Curvature: L={left_curvem:.0f}m, R={right_curvem:.0f}m')

            self.left_curvem = left_curvem
            self.right_curvem = right_curvem
            return left_curvem, right_curvem
        except np.linalg.LinAlgError:
             self.get_logger().warn('calculate_curvature: Polyfit für Krümmung fehlgeschlagen.')
             self.left_curvem = None
             self.right_curvem = None
             return None, None
        except Exception as e:
            self.get_logger().error(f'Fehler in calculate_curvature: {e}')
            self.left_curvem = None
            self.right_curvem = None
            return None, None


    def calculate_histogram(self, frame=None, plot=False):
        """
        Berechnet das Histogramm des unteren Bildteils (vom gefilterten Bild).
        :param frame: Eingabebild (nach Perspektivtransformation)
        :param plot: Falls True, wird ein Plot angezeigt (normalerweise False in ROS2)
        :return: Histogramm als 1D-Array
        """
        if frame is None:
            frame = self.warped_frame

        if frame is None:
             self.get_logger().warn("calculate_histogram: Kein warped_frame vorhanden.")
             return None

        # Histogramm nur vom unteren Teil berechnen
        self.histogram = np.sum(frame[int(frame.shape[0]/2):, :], axis=0)

        if plot:
            # Debug-Plot (nicht in ROS2 verwenden, blockiert den Node)
            import matplotlib.pyplot as plt
            figure, (ax1, ax2) = plt.subplots(2, 1)
            figure.set_size_inches(10, 5)
            ax1.imshow(frame, cmap='gray')
            ax1.set_title("Warped Binary Frame")
            ax2.plot(self.histogram)
            ax2.set_title("Histogram Peaks")
            plt.show() # Diese Zeile blockiert!
        return self.histogram

    def display_curvature_offset(self, frame=None, plot=False):
        """
        Überlagert Kurvenradius und Versatz auf das Originalbild.
        :param frame: Bild, auf dem gezeichnet wird (Standard: Originalbild)
        :param plot: Falls True, wird das Bild angezeigt (in ROS2 also False)
        :return: Annotiertes Bild
        """
        image_copy = frame.copy() if frame is not None else self.orig_frame.copy()

        # Nur anzeigen, wenn Werte vorhanden sind
        if self.left_curvem is not None and self.right_curvem is not None:
            avg_curve = (self.left_curvem + self.right_curvem) / 2
            cv2.putText(image_copy, f'Curve Radius: {avg_curve:.0f} m',
                        (int((5/600)*self.width), int((20/338)*self.height)),
                        cv2.FONT_HERSHEY_SIMPLEX, float((0.5/600)*self.width), (255,255,255), 2, cv2.LINE_AA)
        else:
             cv2.putText(image_copy, 'Curve Radius: N/A',
                        (int((5/600)*self.width), int((20/338)*self.height)),
                        cv2.FONT_HERSHEY_SIMPLEX, float((0.5/600)*self.width), (255,255,255), 2, cv2.LINE_AA)


        if self.center_offset is not None:
            cv2.putText(image_copy, f'Center Offset: {self.center_offset:.2f} cm',
                        (int((5/600)*self.width), int((40/338)*self.height)),
                        cv2.FONT_HERSHEY_SIMPLEX, float((0.5/600)*self.width), (255,255,255), 2, cv2.LINE_AA)
        else:
             cv2.putText(image_copy, 'Center Offset: N/A',
                        (int((5/600)*self.width), int((40/338)*self.height)),
                        cv2.FONT_HERSHEY_SIMPLEX, float((0.5/600)*self.width), (255,255,255), 2, cv2.LINE_AA)

        if plot: # Sollte in ROS2 False sein
            cv2.imshow("Image with Curvature and Offset", image_copy)
            cv2.waitKey(1) # Nötig für imshow, aber besser im GUI Node
        return image_copy

    def get_lane_line_previous_window(self, left_fit, right_fit, plot=False):
        """
        Verfeinert die Linien anhand der vorherigen Fenster (wenn Fits vorhanden sind).
        :param left_fit: Polynomausgleich für die linke Linie
        :param right_fit: Polynomausgleich für die rechte Linie
        :param plot: Debug-Anzeige (standardmäßig False)
        """
        # --- PRÜFUNG HINZUGEFÜGT ---
        if left_fit is None or right_fit is None or self.warped_frame is None:
            self.get_logger().warn("get_lane_line_previous_window: Ungültige Fits oder warped_frame.")
            # Wichtig: Setze Folgeattribute zurück oder behalte alte gültige Werte
            self.left_fitx = None
            self.right_fitx = None
            self.ploty = None
            self.leftx = None
            self.lefty = None
            self.rightx = None
            self.righty = None
            return # Beende die Funktion hier

        margin = self.margin
        nonzero = self.warped_frame.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        try:
            # Berechne die Grenzen basierend auf den Fits
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

            # Extrahiere Pixelpositionen
            leftx = nonzerox[left_lane_inds]
            lefty = nonzeroy[left_lane_inds]
            rightx = nonzerox[right_lane_inds]
            righty = nonzeroy[right_lane_inds]

            # --- PRÜFUNG VOR POLYFIT ---
            if len(lefty) < 3 or len(righty) < 3:
                self.get_logger().warn('get_lane_line_previous_window: Nicht genügend Punkte für Refit.')
                # Behalte die Fits aus dem Sliding Window, setze aber x/y zurück
                self.leftx = None
                self.lefty = None
                self.rightx = None
                self.righty = None
                # Berechne ploty und fitx trotzdem mit den alten Fits für die Visualisierung
                self.ploty = np.linspace(0, self.warped_frame.shape[0]-1, self.warped_frame.shape[0])
                self.left_fitx = left_fit[0]*self.ploty**2 + left_fit[1]*self.ploty + left_fit[2]
                self.right_fitx = right_fit[0]*self.ploty**2 + right_fit[1]*self.ploty + right_fit[2]
                return # Verlasse die Funktion hier

            self.leftx = leftx
            self.rightx = rightx
            self.lefty = lefty
            self.righty = righty

            # Refit mit den neuen Punkten
            new_left_fit = np.polyfit(lefty, leftx, 2)
            new_right_fit = np.polyfit(righty, rightx, 2)

            # Update der Klassenattribute nur bei erfolgreichem Refit
            self.left_fit = new_left_fit
            self.right_fit = new_right_fit

            # Berechne Punkte für die Visualisierung
            self.ploty = np.linspace(0, self.warped_frame.shape[0]-1, self.warped_frame.shape[0])
            self.left_fitx = self.left_fit[0]*self.ploty**2 + self.left_fit[1]*self.ploty + self.left_fit[2]
            self.right_fitx = self.right_fit[0]*self.ploty**2 + self.right_fit[1]*self.ploty + self.right_fit[2]

        except np.linalg.LinAlgError:
            self.get_logger().warn('get_lane_line_previous_window: Polyfit fehlgeschlagen.')
            # Behalte alte Fits, aber setze x/y/fitx/ploty zurück? Oder behalte alte Plot-Daten?
            # Hier behalten wir die alten Plot-Daten, falls vorhanden, sonst None
            if self.ploty is None: # Nur berechnen, wenn noch nicht vorhanden
                 self.ploty = np.linspace(0, self.warped_frame.shape[0]-1, self.warped_frame.shape[0])
            if self.left_fit is not None: # Verwende existierenden Fit
                 self.left_fitx = self.left_fit[0]*self.ploty**2 + self.left_fit[1]*self.ploty + self.left_fit[2]
            else: self.left_fitx = None
            if self.right_fit is not None:
                 self.right_fitx = self.right_fit[0]*self.ploty**2 + self.right_fit[1]*self.ploty + self.right_fit[2]
            else: self.right_fitx = None
            self.leftx = None # Keine gültigen neuen Punkte gefunden
            self.lefty = None
            self.rightx = None
            self.righty = None
            return # Verlasse die Funktion
        except Exception as e:
            self.get_logger().error(f"Fehler in get_lane_line_previous_window: {e}")
            # Setze relevante Attribute zurück
            self.left_fitx = None
            self.right_fitx = None
            self.ploty = None
            self.leftx = None
            self.lefty = None
            self.rightx = None
            self.righty = None
            return # Verlasse die Funktion


        if plot: # Sollte in ROS2 nicht verwendet werden
            # ... (Plotting Code, der jetzt self.left_fitx etc. verwendet) ...
            pass


    def get_lane_line_indices_sliding_windows(self, plot=False):
        """
        Sucht die Pixel-Indizes der Linien mit der Sliding-Window-Methode.
        :param plot: Debug-Anzeige (standardmäßig False)
        :return: (linkes Polyfit, rechtes Polyfit) oder (None, None) bei Fehler
        """
        if self.warped_frame is None or self.histogram is None:
             self.get_logger().warn("get_lane_line_indices_sliding_windows: Kein warped_frame oder Histogramm.")
             self.left_fit = None # Sicherstellen, dass Fits None sind
             self.right_fit = None
             return None, None

        margin = self.margin
        frame_sliding_window = self.warped_frame.copy()
        # Konvertiere zu int, da float nicht mehr unterstützt wird für window_height
        window_height = int(self.warped_frame.shape[0]/self.no_of_windows)
        nonzero = self.warped_frame.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        left_lane_inds = []
        right_lane_inds = []

        try:
            leftx_base, rightx_base = self.histogram_peak()
        except ValueError: # Kann auftreten, wenn Histogramm leer ist
            self.get_logger().warn("Histogramm-Peak konnte nicht bestimmt werden (evtl. leeres Histogramm).")
            self.left_fit = None
            self.right_fit = None
            return None, None

        leftx_current = leftx_base
        rightx_current = rightx_base

        for window in range(self.no_of_windows):
            # Fenstergrenzen berechnen
            win_y_low = self.warped_frame.shape[0] - (window + 1) * window_height
            win_y_high = self.warped_frame.shape[0] - window * window_height
            # Wichtig: x/y-Grenzen müssen Integer sein für cv2.rectangle und Indexing
            win_xleft_low = int(leftx_current - margin)
            win_xleft_high = int(leftx_current + margin)
            win_xright_low = int(rightx_current - margin)
            win_xright_high = int(rightx_current + margin)

            if plot: # Nur zeichnen wenn plot=True (normalerweise nicht in ROS2)
                cv2.rectangle(frame_sliding_window, (win_xleft_low, win_y_low),
                              (win_xleft_high, win_y_high), (255,255,255), 2)
                cv2.rectangle(frame_sliding_window, (win_xright_low, win_y_low),
                              (win_xright_high, win_y_high), (255,255,255), 2)

            # Finde Punkte innerhalb des Fensters
            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                              (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                               (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]
            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)

            # Zentriere nächstes Fenster neu, wenn genug Punkte gefunden wurden
            if len(good_left_inds) > self.minpix:
                leftx_current = np.int_(np.mean(nonzerox[good_left_inds])) # np.int_ verwenden
            if len(good_right_inds) > self.minpix:
                rightx_current = np.int_(np.mean(nonzerox[good_right_inds])) # np.int_ verwenden

        # Verbinde die Indizes aus allen Fenstern
        try:
            left_lane_inds = np.concatenate(left_lane_inds)
            right_lane_inds = np.concatenate(right_lane_inds)
        except ValueError: # Kann passieren, wenn alle Fenster leer waren
             self.get_logger().warn("Sliding windows haben keine Punkte gefunden.")
             self.left_fit = None
             self.right_fit = None
             return None, None

        # Extrahiere Linienpixel-Positionen
        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds]
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]

        # Speichere aktuelle Punkte (wichtig für calculate_curvature)
        self.leftx = leftx
        self.lefty = lefty
        self.rightx = rightx
        self.righty = righty

        # --- PRÜFUNG VOR POLYFIT ---
        if len(lefty) < 3 or len(righty) < 3:
             self.get_logger().warn('Nicht genügend Linienpunkte für Polyfit nach Sliding Windows gefunden.')
             self.left_fit = None
             self.right_fit = None
             return None, None # Signalisiert dem Callback, dass die Fits ungültig sind
        # --- ENDE PRÜFUNG ---

        # Passe Polynom an, nur wenn genügend Punkte vorhanden sind
        try:
            left_fit = np.polyfit(lefty, leftx, 2)
            right_fit = np.polyfit(righty, rightx, 2)
            # Speichere die aktuellen Fits
            self.left_fit = left_fit
            self.right_fit = right_fit
        except (np.linalg.LinAlgError, TypeError) as e: # TypeError fangen wir auch ab
             self.get_logger().warn(f'Polyfit fehlgeschlagen: {e}')
             self.left_fit = None # Setze Fits zurück
             self.right_fit = None
             return None, None # Signalisiert dem Callback, dass die Fits ungültig sind

        # Berechne Punkte für die Visualisierung (jetzt basierend auf den neuen Fits)
        try:
            self.ploty = np.linspace(0, self.warped_frame.shape[0]-1, self.warped_frame.shape[0])
            self.left_fitx = self.left_fit[0]*self.ploty**2 + self.left_fit[1]*self.ploty + self.left_fit[2]
            self.right_fitx = self.right_fit[0]*self.ploty**2 + self.right_fit[1]*self.ploty + self.right_fit[2]
        except Exception as e:
             self.get_logger().error(f"Fehler beim Berechnen von fitx/ploty: {e}")
             self.ploty = None
             self.left_fitx = None
             self.right_fitx = None
             # Fits sind zwar da, aber Visualisierungspunkte nicht -> trotzdem Fits zurückgeben? Ja.

        if plot:
            # ... (Plotting code, der jetzt self.left_fitx etc. verwendet) ...
            pass

        return self.left_fit, self.right_fit # Gibt die berechneten Fits zurück


    def get_line_markings(self, frame=None):
        """
        Isoliert Fahrbahnmarkierungen.
        :param frame: Falls nicht angegeben, wird self.orig_frame genutzt.
        :return: Binäres Bild oder None bei Fehler
        """
        if frame is None:
            frame = self.orig_frame

        if frame is None:
             self.get_logger().error("get_line_markings: Kein Eingabebild vorhanden.")
             return None

        try:
            # Konvertiere zu HLS
            hls = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)

            # --- Schwellwerte anpassen ---
            # Diese Werte sind oft sehr szenenabhängig (Licht, Kameratyp etc.)
            # Experimentiere hiermit!

            # S-Kanal (Sättigung) - gut für weiße/gelbe Linien bei unterschiedlichem Licht
            s_channel = hls[:, :, 2]
            s_thresh_min = 100 # Untere Schwelle für Sättigung
            s_thresh_max = 255 # Obere Schwelle für Sättigung
            _, s_binary = edge.threshold(s_channel, thresh=(s_thresh_min, s_thresh_max))

            # L-Kanal (Helligkeit) - kann helfen, Schatten zu reduzieren
            l_channel = hls[:, :, 1]
            l_thresh_min = 120 # Untere Schwelle Helligkeit (vorsichtig anpassen!)
            l_thresh_max = 255
            _, l_binary = edge.threshold(l_channel, thresh=(l_thresh_min, l_thresh_max))


            # R-Kanal (Rot) aus BGR - manchmal nützlich, aber anfällig für Lichtänderungen
            # r_thresh_min = 150
            # r_thresh_max = 255
            # _, r_thresh = edge.threshold(frame[:, :, 2], thresh=(r_thresh_min, r_thresh_max))

            # Kantenextraktion (Sobel auf L-Kanal)
            # _, sx_binary = edge.threshold(l_channel, thresh=(50, 255)) # Optional: Vorfilterung
            sx_binary = edge.mag_thresh(l_channel, sobel_kernel=3, thresh=(50, 255)) # Kantenstärke

            # Kombiniere die binären Bilder
            # Versuche verschiedene Kombinationen:
            # combined_binary = cv2.bitwise_or(s_binary, sx_binary.astype(np.uint8))
            combined_binary = s_binary # Oft ist S-Kanal alleine robust
            # combined_binary = cv2.bitwise_or(s_binary, l_binary)
            # combined_binary = cv2.bitwise_or(combined_binary, r_thresh) # Wenn R-Kanal genutzt wird

            # Optional: Gaußscher Weichzeichner nach der Kombination
            # combined_binary = edge.blur_gaussian(combined_binary, ksize=3)

            self.lane_line_markings = combined_binary
            return self.lane_line_markings

        except cv2.error as e:
            self.get_logger().error(f"OpenCV Fehler in get_line_markings: {e}")
            self.lane_line_markings = None
            return None
        except Exception as e:
            self.get_logger().error(f"Allgemeiner Fehler in get_line_markings: {e}")
            self.lane_line_markings = None
            return None


    def histogram_peak(self):
        """
        Bestimmt den linken und rechten Peak im Histogramm.
        Kann ValueError werfen, wenn Histogramm leer ist oder keine Peaks hat.
        :return: (x-Koordinate des linken Peaks, x-Koordinate des rechten Peaks)
        """
        if self.histogram is None or len(self.histogram) == 0:
             raise ValueError("Histogramm ist leer.")

        midpoint = int(self.histogram.shape[0]/2)
        leftx_base = np.argmax(self.histogram[:midpoint])
        rightx_base = np.argmax(self.histogram[midpoint:]) + midpoint

        # Optional: Prüfung, ob Peaks sinnvoll sind (z.B. nicht beide 0)
        if self.histogram[leftx_base] == 0 and self.histogram[rightx_base] == 0:
            # Keine Peaks gefunden
            raise ValueError("Keine Peaks im Histogramm gefunden.")

        return leftx_base, rightx_base

    def overlay_lane_lines(self, plot=False):
        """
        Legt die erkannten Fahrbahnmarkierungen auf das Originalbild.
        :param plot: Falls True, wird zur Debug-Anzeige geplottet.
        :return: Bild mit Überlagerung oder Originalbild bei Fehler
        """
        # Prüfen ob alle nötigen Daten vorhanden sind
        if self.warped_frame is None or self.left_fitx is None or \
           self.right_fitx is None or self.ploty is None or \
           self.inv_transformation_matrix is None:
            self.get_logger().warn("overlay_lane_lines: Fehlende Daten für Overlay.")
            return self.orig_frame # Gib Originalbild zurück

        try:
            warp_zero = np.zeros_like(self.warped_frame).astype(np.uint8)
            color_warp = np.dstack((warp_zero, warp_zero, warp_zero))

            # Punkte für Polygon definieren
            pts_left = np.array([np.transpose(np.vstack([self.left_fitx, self.ploty]))])
            pts_right = np.array([np.flipud(np.transpose(np.vstack([self.right_fitx, self.ploty])))])
            pts = np.hstack((pts_left, pts_right))

            # Polygon zeichnen und zurücktransformieren
            cv2.fillPoly(color_warp, np.int_([pts]), (0,255, 0)) # Grün für den Bereich
            newwarp = cv2.warpPerspective(color_warp, self.inv_transformation_matrix,
                                          (self.orig_frame.shape[1], self.orig_frame.shape[0]))

            # Bilder überlagern
            result = cv2.addWeighted(self.orig_frame, 1, newwarp, 0.3, 0)

            if plot: # Nicht in ROS2 verwenden
                # ... (Plotting code) ...
                pass

            return result
        except Exception as e:
            self.get_logger().error(f"Fehler in overlay_lane_lines: {e}")
            return self.orig_frame # Gib Originalbild bei Fehler zurück

    def perspective_transform(self, frame=None, plot=False):
        """
        Führt die Perspektivtransformation aus (Bird’s Eye View).
        :param frame: Wenn nicht angegeben, wird self.lane_line_markings genutzt.
        :param plot: Debug-Anzeige (normalerweise False in ROS2)
        :return: Warped (transformiertes) Bild oder None bei Fehler
        """
        if frame is None:
            frame = self.lane_line_markings

        if frame is None:
            self.get_logger().warn("perspective_transform: Kein Eingabebild für Transformation.")
            self.warped_frame = None
            return None

        try:
            # Transformationsmatrizen berechnen
            self.transformation_matrix = cv2.getPerspectiveTransform(self.roi_points, self.desired_roi_points)
            self.inv_transformation_matrix = cv2.getPerspectiveTransform(self.desired_roi_points, self.roi_points)

            # Transformation anwenden
            self.warped_frame = cv2.warpPerspective(frame, self.transformation_matrix, self.orig_image_size, flags=cv2.INTER_LINEAR)

            # Optional: Nochmaliges Thresholding nach der Transformation
            # (thresh, binary_warped) = cv2.threshold(self.warped_frame, 127, 255, cv2.THRESH_BINARY)
            # self.warped_frame = binary_warped

            if plot: # Nicht in ROS2 verwenden
                # ... (Plotting code) ...
                pass

            return self.warped_frame
        except Exception as e:
             self.get_logger().error(f"Fehler in perspective_transform: {e}")
             self.warped_frame = None
             self.transformation_matrix = None # Matrizen zurücksetzen
             self.inv_transformation_matrix = None
             return None


    def plot_roi(self, frame=None, plot=False):
        """
        Zeichnet das Region-of-Interest (ROI) im Bild. (Nur für Debugging außerhalb von ROS2)
        :param frame: Eingangsbild, falls nicht angegeben wird self.orig_frame genutzt.
        :param plot: Falls True, wird das Bild angezeigt.
        """
        if not plot:
            return
        if frame is None:
            frame = self.orig_frame.copy()

        if frame is None: return # Nichts zu zeichnen

        try:
            # Zeichne ROI
            this_image = cv2.polylines(frame, np.int32([self.roi_points]), True, (147,20,255), 3)
            cv2.imshow('ROI Image', this_image)
            cv2.waitKey(0) # Blockiert!
            cv2.destroyAllWindows()
        except Exception as e:
            self.get_logger().error(f"Fehler in plot_roi: {e}")


# --- ROS2 Node ---------------------------------------------------------------
class LaneDetectionNode(Node):
    def __init__(self):
        super().__init__('lane_detection_node')

        # Definition des QoS-Profils
        qos_policy = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriber für das komprimierte Eingangssignal
        # KORREKTUR: Das letzte Argument '10' entfernt
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed', # Stelle sicher, dass dieses Topic existiert!
            self.image_callback,
            qos_policy                 # Nur 4 Argumente hier
        )

        # Publisher für das annotierte Bild (als CompressedImage)
        self.publisher_ = self.create_publisher(
             CompressedImage,
             '/lane/image_annotated',
             qos_policy # QoS hier auch anwenden
        )
        self.get_logger().info('Lane Detection Node gestartet. Warte auf Bilder von /image_raw/compressed...')


    def image_callback(self, msg: CompressedImage):
        try:
            # Dekodiere das komprimierte Bild in ein OpenCV-Bild (BGR)
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if cv_image is None:
                self.get_logger().warn('Fehler beim Dekodieren des Bildes.')
                return

            # Wende Lane-Detection-Pipeline an:
            # Erzeuge ein Lane-Objekt (übergebe den Logger)
            lane_obj = Lane(orig_frame=cv_image, logger=self.get_logger())

            # 1. Isoliere Fahrbahnmarkierungen
            if lane_obj.get_line_markings() is None:
                 self.get_logger().warn("Keine Linienmarkierungen gefunden/erzeugt.")
                 annotated_frame = cv_image # Zeige Originalbild
            # 2. Perspektivtransformation
            elif lane_obj.perspective_transform(plot=False) is None:
                 self.get_logger().warn("Perspektivtransformation fehlgeschlagen.")
                 annotated_frame = cv_image
            # 3. Histogramm-Berechnung
            elif lane_obj.calculate_histogram(plot=False) is None:
                 self.get_logger().warn("Histogramm-Berechnung fehlgeschlagen.")
                 annotated_frame = cv_image
            else:
                 # 4. Suche nach den Linien mittels Sliding Windows
                 left_fit, right_fit = lane_obj.get_lane_line_indices_sliding_windows(plot=False)

                 # Wenn Fits nicht gefunden wurden (Rückgabe ist None), publiziere Originalbild
                 if left_fit is None or right_fit is None:
                     self.get_logger().debug('Keine validen Fits aus Sliding Windows gefunden.')
                     annotated_frame = cv_image # Zeige Originalbild bei Fehler
                 else:
                     # 5. Feineinstellung über vorheriges Fenster (optional, aber oft nützlich)
                     # Diese Funktion ist jetzt robuster und gibt nichts zurück bei Fehler
                     lane_obj.get_lane_line_previous_window(left_fit, right_fit, plot=False)

                     # 6. Overlay der erkannten Linien auf das Originalbild
                     # Diese Funktion gibt jetzt das Originalbild zurück, wenn etwas schiefgeht
                     frame_with_lane_lines = lane_obj.overlay_lane_lines(plot=False)

                     # 7. Berechnung von Krümmung und Fahrzeugversatz
                     # Diese Funktionen geben jetzt None zurück bei Fehlern
                     lane_obj.calculate_curvature(print_to_terminal=False)
                     lane_obj.calculate_car_position(print_to_terminal=False)

                     # 8. Overlay der Texte (Krümmung, Offset)
                     # Diese Funktion zeigt jetzt "N/A" an, wenn Werte None sind
                     annotated_frame = lane_obj.display_curvature_offset(frame=frame_with_lane_lines, plot=False)


            # --- Kein cv2.imshow hier! Das macht der gui_debug_node ---

            # Kodieren des (möglicherweise nur originalen) Bildes als JPEG
            ret, buffer = cv2.imencode('.jpg', annotated_frame)
            if not ret:
                self.get_logger().error("Fehler beim Kodieren des Bildes zu JPEG.")
                return

            # Erstelle eine CompressedImage-Nachricht
            out_msg = CompressedImage()
            # Verwende den Zeitstempel der Originalnachricht für bessere Synchronisation
            out_msg.header.stamp = msg.header.stamp
            out_msg.format = "jpeg"
            out_msg.data = buffer.tobytes()

            # Publiziere das annotierte Bild
            self.publisher_.publish(out_msg)
            # self.get_logger().debug("Publizierte ein annotiertes Bild.") # Optional: Weniger Logging

        except Exception as e:
             # Fange alle unerwarteten Fehler im Callback ab, damit der Node nicht stirbt
             self.get_logger().fatal(f'Unerwarteter Fehler im image_callback: {e}')
             import traceback
             self.get_logger().error(traceback.format_exc()) # Gibt den vollen Traceback aus


def main(args=None):
    rclpy.init(args=args)
    try:
        node = LaneDetectionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt erhalten, beende Node.")
    except Exception as e:
         # Fange Fehler bei der Node-Initialisierung oder beim Spinnen ab
         # (Ein Logger existiert hier möglicherweise noch nicht, daher print)
         print(f"FATALER FEHLER beim Starten/Ausführen des Nodes: {e}", file=sys.stderr)
         import traceback
         traceback.print_exc()
    finally:
        # Aufräumen, auch bei Fehlern
        if 'node' in locals() and isinstance(node, Node) and rclpy.ok():
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
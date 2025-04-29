# waymo/traffic_light_detection_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import cv2, cv_bridge
import numpy as np
import sys
import traceback
import time # Import time für sleep

# --- WICHTIG: Sicherstellen, dass der Klassenname korrekt ist, falls er SpecificColorDetector sein soll ---
# Falls der Node tatsächlich traffic_light_detection_node heissen soll,
# wäre es konsistenter, die Klasse auch so zu nennen (z.B. TrafficLightDetector)
# Aktuell basiert der Code auf der Klasse 'SpecificColorDetector', wie von dir gepostet.
class SpecificColorDetector(Node): # Oder TrafficLightDetector, wenn du umbenennst
    def __init__(self):
        # --- Den Node-Namen hier anpassen, falls die Klasse umbenannt wird ---
        super().__init__('specific_color_detector') # Oder 'traffic_light_detector'

        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=1
        )
        # QoS für Bilddaten
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1
        )

        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.image_callback,
            qos_sensor) # QoS für Sensor

        self.publisher_ = self.create_publisher(Bool, 'traffic_light', qos_reliable)

        # --- FEHLENDES ATTRIBUT HINZUGEFÜGT ---
        self.show_debug_windows = False # Auf True setzen, um Fenster anzuzeigen
        # --- ---

        # Optional: Debug-Fenster nur erstellen, wenn aktiviert
        if self.show_debug_windows:
             try: # Fehler abfangen, falls GUI nicht verfügbar
                 # Namen der Fenster können angepasst werden
                 cv2.namedWindow('TrafficLight Mask (Blobs > 100px)', cv2.WINDOW_NORMAL)
                 cv2.namedWindow('TrafficLight Overlay', cv2.WINDOW_NORMAL)
             except Exception as e:
                  # Minimales Logging bei Fehler
                  print(f"WARNUNG [{self.get_name()}]: Konnte Debug-Fenster nicht erstellen: {e}", file=sys.stderr)
                  self.show_debug_windows = False # Deaktiviere Fenster, wenn Erstellung fehlschlägt

    def image_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is None: return # Frame nicht dekodierbar

            h, w, _ = frame.shape
            # ROI anpassen? Beispiel: Obere Hälfte
            # Dies sollte evtl. ein Parameter sein, wie im früheren Code
            frame_cropped = frame[0:int(h * 0.5), :]

            # Farbe erkennen
            detected, filtered_mask = self.detect_specific_color(frame_cropped)

            # Debug-Fenster anzeigen (falls aktiviert)
            if self.show_debug_windows:
                try: # Fehler beim Anzeigen abfangen
                    cv2.imshow('TrafficLight Mask (Blobs > 100px)', filtered_mask)
                    # Vermeide Fehler, wenn frame_cropped leer ist
                    if frame_cropped.size > 0:
                         overlay = frame_cropped.copy()
                         # Farbe für Overlay anpassen (z.B. Grün für erkannte Farbe)
                         overlay[filtered_mask > 0] = (0, 255, 0)
                         cv2.imshow('TrafficLight Overlay', overlay)
                    cv2.waitKey(1)
                except Exception as e:
                     # Fehler beim Anzeigen loggen, aber weiterlaufen
                     print(f"ERROR [{self.get_name()}] displaying debug windows: {e}", file=sys.stderr)


            # Status publizieren
            # Wenn Farbe DETECTED -> Sende False (entspricht Rot/Stop)
            # Wenn Farbe NICHT DETECTED -> Sende True (entspricht Grün/Go)
            self.publisher_.publish(Bool(data=not detected))

        except Exception as e:
             # Allgemeiner Fehler bei der Bildverarbeitung
             print(f"ERROR [{self.get_name()}] in image_callback: {e}", file=sys.stderr)
             traceback.print_exc(file=sys.stderr) # Zeige Traceback für Debugging
             # Sende im Fehlerfall sicherheitshalber "Stop"?
             # self.publisher_.publish(Bool(data=False))


    def detect_specific_color(self, frame):
        """Erkennt eine spezifische Farbe und filtert nach Blob-Größe."""
        detected = False
        # Sicherstellen, dass filtered_mask die richtige Größe hat, auch wenn frame leer ist
        if frame is None or frame.shape[0] == 0 or frame.shape[1] == 0:
             # print(f"WARNUNG [{self.get_name()}]: detect_specific_color received empty frame.", file=sys.stderr) # Optional
             return detected, np.array([[]], dtype=np.uint8) # Leere Maske zurückgeben

        filtered_mask = np.zeros((frame.shape[0], frame.shape[1]), dtype=np.uint8)

        try:
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # Farbe: #9e475f (Dunkles Pink/Magenta)
            # Diese Werte sollten wahrscheinlich Parameter sein (wie im früheren Code)
            lower = np.array([160, 50, 50])
            upper = np.array([180, 255, 255])

            mask = cv2.inRange(hsv, lower, upper)

            # Morphologische Operationen können helfen
            # kernel = np.ones((3,3),np.uint8)
            # mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            # mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            # Blobs extrahieren
            num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)

            # Mindestfläche sollte Parameter sein
            min_blob_area = 100

            if num_labels > 1: # Wenn mehr als nur der Hintergrund gefunden wurde
                for label in range(1, num_labels):
                    area = stats[label, cv2.CC_STAT_AREA]
                    if area >= min_blob_area:
                        filtered_mask[labels == label] = 255
                        detected = True
                        # break # Frühzeitig beenden, wenn ein Blob reicht?

        except cv2.error as cv_err:
             # Spezifische OpenCV-Fehler abfangen
             print(f"ERROR [{self.get_name()}] OpenCV error in detect_specific_color: {cv_err}", file=sys.stderr)
        except Exception as e:
             print(f"ERROR [{self.get_name()}] in detect_specific_color: {e}", file=sys.stderr)
             traceback.print_exc(file=sys.stderr)

        return detected, filtered_mask

    def destroy_node(self):
        """Ressourcen freigeben."""
        # print(f"INFO [{self.get_name()}]: Destroying node...", file=sys.stderr) # Log entfernt
        if self.show_debug_windows:
            try:
                 cv2.destroyAllWindows()
            except Exception: pass # Fehler ignorieren
        # Publisher und Subscriber werden automatisch zerstört
        super().destroy_node()

# --- main Funktion mit korrigiertem Shutdown ---
def main(args=None):
    rclpy.init(args=args)
    node = None
    node_name_for_log = SpecificColorDetector.__name__ # Oder fester String
    try:
        # --- Hier den Klassennamen anpassen, falls geändert ---
        node = SpecificColorDetector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        # print(f"INFO [{node_name_for_log}]: KeyboardInterrupt received.", file=sys.stderr) # Log entfernt
        pass
    except Exception as e:
         # Kritische Fehler weiterhin anzeigen
         print(f"FATAL ERROR [{node_name_for_log}] in main: {e}", file=sys.stderr)
         traceback.print_exc(file=sys.stderr)
    finally:
        # Node zerstören, falls erstellt
        if node is not None:
            # Prüfen, ob Attribut existiert, bevor darauf zugegriffen wird
            should_destroy_cv_windows = False
            if hasattr(node, 'show_debug_windows') and node.show_debug_windows:
                 should_destroy_cv_windows = True
            # Zerstöre den Node (ruft node.destroy_node() auf)
            node.destroy_node()

        # rclpy nur herunterfahren, wenn es noch läuft
        if rclpy.ok(): # <-- Wichtige Prüfung
            rclpy.shutdown()

        # Sicherstellen, dass Fenster geschlossen sind
        if should_destroy_cv_windows:
             try:
                  # Kurze Pause kann helfen, damit Fenster sicher geschlossen werden
                  # time.sleep(0.1) # Optional
                  cv2.destroyAllWindows()
             except Exception: pass

if __name__ == '__main__':
    main()
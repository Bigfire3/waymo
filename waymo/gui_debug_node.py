#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# Benötigte ROS-Nachrichtentypen und Tools
from sensor_msgs.msg import CompressedImage # Für Debug-Bilder
from std_msgs.msg import String             # Für Statusnachrichten und Keyboard-Befehle
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import sys
import traceback
from collections import OrderedDict # Um die Reihenfolge der Bilder beizubehalten

class GuiDebugNode(Node):
    def __init__(self):
        super().__init__('gui_debug_node')

        # --- Konfiguration ---
        # Veraltetes Topic entfernen: self.image_topic = '/lane/image_annotated'
        self.state_topic = '/robot/state'
        self.keyboard_command_topic = '/keyboard_command' # Keyboard Befehle Topic
        # Liste der zu abonnierenden Debug-Topics
        self.debug_topics = OrderedDict([
            ('roi', '/debug/cam/roi'),
            ('lane_annotated', '/debug/cam/lane_annotated'),
            ('raw_markings', '/debug/cam/raw_markings'),
            ('warped', '/debug/cam/warped'),
            ('filtered_warped', '/debug/cam/filtered_warped'),
            ('sliding_window', '/debug/cam/sliding_window'),
            ('traffic_mask', '/debug/cam/traffic_mask'),
            ('traffic_overlay', '/debug/cam/traffic_overlay'),
            # Füge hier bei Bedarf weitere Topics hinzu
        ])
        self.image_msg_type = CompressedImage
        self.gui_window_name = 'Waymo Debug Canvas' # Angepasster Name
        self.canvas_cols = 2 # Anzahl der Spalten im Canvas
        self.placeholder_color = (40, 40, 40) # Dunkelgrauer Platzhalter
        self.placeholder_text_color = (200, 200, 200)

        # Initialisiere internen Status
        self.current_robot_state = "WAYMO_STARTED"
        self.canvas_visible = False # Canvas standardmäßig aus
        self.last_images = {name: None for name in self.debug_topics.keys()} # Speicher für letzte Bilder
        self.last_canvas_shape = None # Zum Vergleichen, ob sich das Layout ändert

        # --- Initialisierung ---
        self.bridge = CvBridge()

        # --- QoS Profile ---
        qos_debug_images = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, # Best Effort für Bilder ist OK
            history=HistoryPolicy.KEEP_LAST, depth=1
        )
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, # Reliable für Status/Befehle
            history=HistoryPolicy.KEEP_LAST, depth=1
        )

        # --- Subscriber ---
        # Status Subscriber (unverändert)
        self.state_subscriber = self.create_subscription(
            String, self.state_topic, self.state_callback, qos_reliable
        )
        # Keyboard Command Subscriber (NEU)
        self.keyboard_subscriber = self.create_subscription(
            String, self.keyboard_command_topic, self.keyboard_callback, qos_reliable
        )
        # Debug Image Subscribers (NEU)
        self.debug_image_subscribers = {}
        for name, topic in self.debug_topics.items():
            # Verwende lambda, um den Namen des Topics an den Callback zu binden
            callback = lambda msg, topic_name=name: self.debug_image_callback(msg, topic_name)
            self.debug_image_subscribers[name] = self.create_subscription(
                self.image_msg_type, topic, callback, qos_debug_images
            )
            # self.get_logger().info(f"Abonniere Debug-Topic: {topic}")

        # --- GUI Setup (Fenster wird erst bei Bedarf erstellt) ---
        self.gui_initialized = False # Flag, ob das Fenster schon mal offen war

        # --- Timer zum Aktualisieren des Canvas ---
        # Aktualisiert das Canvas nur, wenn es sichtbar ist
        canvas_update_period = 0.1 # Sekunden (10 Hz) - Anpassen bei Bedarf
        self.canvas_timer = self.create_timer(canvas_update_period, self.update_canvas)

        # Initiales Logging
        self.get_logger().info(f"GuiDebugNode gestartet. Drücke 'd' im Keyboard Handler, um das Debug-Canvas anzuzeigen.")
        self.get_logger().info(f"Robot Status: {self.current_robot_state}")


    def _create_placeholder_image(self, width=320, height=240, text="Warte..."):
        """ Erstellt ein einfaches Platzhalterbild. """
        img = np.full((height, width, 3), self.placeholder_color, dtype=np.uint8)
        # Text zentrieren
        text_size, _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        text_x = (width - text_size[0]) // 2
        text_y = (height + text_size[1]) // 2
        cv2.putText(img, text, (text_x, text_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.placeholder_text_color, 1, cv2.LINE_AA)
        return img

    def debug_image_callback(self, msg, topic_name):
        """ Speichert das zuletzt empfangene Bild für jedes Debug-Topic. """
        if not self.canvas_visible: return # Nur verarbeiten, wenn Canvas sichtbar ist
        try:
            # Dekomprimiere das Bild
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            # Speichere das Bild im Dictionary
            if topic_name in self.last_images:
                self.last_images[topic_name] = cv_image
        except CvBridgeError as e:
            # self.get_logger().error(f"CvBridge Error für Topic '{topic_name}': {e}", throttle_duration_sec=10)
            self.last_images[topic_name] = None # Bei Fehler zurücksetzen
        except Exception as e:
            # self.get_logger().error(f"Allgemeiner Fehler beim Verarbeiten von '{topic_name}': {e}", throttle_duration_sec=10)
            self.last_images[topic_name] = None # Bei Fehler zurücksetzen

    def state_callback(self, msg: String):
        """ Verarbeitet Statusnachrichten und loggt NUR Änderungen. """
        new_state = msg.data
        if new_state != self.current_robot_state:
            self.current_robot_state = new_state
            self.get_logger().info(f"Robot Status: {self.current_robot_state}")

    def keyboard_callback(self, msg: String):
        """ Verarbeitet Keyboard-Befehle. """
        command = msg.data
        if command == 'toggle_debug_canvas':
            self.canvas_visible = not self.canvas_visible
            if self.canvas_visible:
                self.get_logger().info("Debug-Canvas wird angezeigt.")
                # Erstelle Fenster, falls es noch nicht existiert
                if not self.gui_initialized:
                     try:
                          cv2.namedWindow(self.gui_window_name, cv2.WINDOW_AUTOSIZE) # Oder WINDOW_NORMAL für Größenänderung
                          cv2.waitKey(1) # Wichtig, damit Fenster gezeichnet wird
                          self.gui_initialized = True
                     except cv2.error as e:
                          self.get_logger().error(f"Konnte OpenCV Fenster nicht erstellen: {e}. Canvas deaktiviert.")
                          self.canvas_visible = False
            else:
                self.get_logger().info("Debug-Canvas wird geschlossen.")
                # Schließe das Fenster
                if self.gui_initialized:
                    try:
                        cv2.destroyWindow(self.gui_window_name)
                        cv2.waitKey(1) # Wichtig, damit das Schließen verarbeitet wird
                        self.gui_initialized = False # Beim nächsten Öffnen neu erstellen
                    except cv2.error: pass # Fenster existiert vielleicht nicht mehr
                # Setze letzte Bilder zurück, um Speicher freizugeben? Optional.
                # for key in self.last_images: self.last_images[key] = None


    def update_canvas(self):
        """ Erstellt und aktualisiert das Canvas-Fenster, wenn sichtbar. """
        if not self.canvas_visible or not self.gui_initialized:
            return

        # Sammle die anzuzeigenden Bilder (oder Platzhalter)
        images_to_show = []
        target_h, target_w = -1, -1 # Zielgröße für alle Kacheln

        # Finde die Größe des ersten verfügbaren Bildes oder nimm Standardgröße
        first_valid_img = next((img for img in self.last_images.values() if img is not None), None)
        if first_valid_img is not None:
             target_h, target_w = first_valid_img.shape[:2]
        else:
             target_h, target_w = 240, 320 # Standardgröße, falls noch keine Bilder da sind

        # Erstelle Liste der Bilder (oder Platzhalter) in der richtigen Größe
        for name, img in self.last_images.items():
            if img is not None:
                # Versuche, Bild auf Zielgröße zu skalieren
                try:
                     # Behalte Seitenverhältnis bei, fülle Rest mit Platzhalterfarbe
                     h, w = img.shape[:2]
                     scale = min(target_w/w, target_h/h)
                     new_w, new_h = int(w*scale), int(h*scale)
                     resized_img = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_AREA)

                     # Erstelle Zielbild mit Platzhalterfarbe
                     tile_img = np.full((target_h, target_w, 3), self.placeholder_color, dtype=np.uint8)
                     # Füge skaliertes Bild zentriert ein
                     x_offset = (target_w - new_w) // 2
                     y_offset = (target_h - new_h) // 2
                     tile_img[y_offset:y_offset+new_h, x_offset:x_offset+new_w] = resized_img

                     # Füge Titel hinzu
                     cv2.putText(tile_img, name, (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, self.placeholder_text_color, 1, cv2.LINE_AA)
                     images_to_show.append(tile_img)

                except Exception as e:
                     # self.get_logger().warn(f"Fehler beim Skalieren/Einbetten von Bild '{name}': {e}", throttle_duration_sec=10)
                     images_to_show.append(self._create_placeholder_image(target_w, target_h, f"{name} (Error)"))
            else:
                # Füge Platzhalter hinzu
                images_to_show.append(self._create_placeholder_image(target_w, target_h, name))

        # Bestimme Anzahl der Zeilen
        num_images = len(images_to_show)
        if num_images == 0:
             canvas = self._create_placeholder_image(target_w*self.canvas_cols, target_h, "Keine Bilder")
        else:
            num_rows = (num_images + self.canvas_cols - 1) // self.canvas_cols

            # Füge leere Platzhalter hinzu, um die letzte Zeile aufzufüllen
            needed_placeholders = num_rows * self.canvas_cols - num_images
            for _ in range(needed_placeholders):
                images_to_show.append(self._create_placeholder_image(target_w, target_h, ""))

            # Ordne Bilder in Zeilen an
            rows = []
            for i in range(num_rows):
                start_idx = i * self.canvas_cols
                end_idx = start_idx + self.canvas_cols
                row_images = images_to_show[start_idx:end_idx]
                # Kombiniere Bilder horizontal für eine Zeile
                rows.append(np.hstack(row_images))

            # Kombiniere Zeilen vertikal zum finalen Canvas
            canvas = np.vstack(rows)

        # Zeige das Canvas an
        try:
            # Nur anzeigen, wenn sich Form geändert hat oder Bild nicht leer ist
            # (Vermeidet Flackern, wenn alle Bilder weg sind)
            # if canvas.shape != self.last_canvas_shape or canvas.size > 0:
            cv2.imshow(self.gui_window_name, canvas)
            self.last_canvas_shape = canvas.shape
            cv2.waitKey(1) # Wichtig für die GUI-Verarbeitung
        except cv2.error:
             # Fenster wurde möglicherweise extern geschlossen
             # self.get_logger().warn("Fehler beim Anzeigen des Canvas (Fenster geschlossen?).", throttle_duration_sec=10)
             self.canvas_visible = False # Deaktiviere Canvas
             self.gui_initialized = False
        except Exception as e:
             # self.get_logger().error(f"Fehler im Canvas Update: {e}", throttle_duration_sec=10)
             pass

    def destroy_node(self):
        """ Ressourcen freigeben beim Beenden. """
        # self.get_logger().info("Shutting down GuiDebugNode.")
        # Schließe das Fenster, falls es noch offen ist
        if self.gui_initialized:
            try:
                cv2.destroyWindow(self.gui_window_name)
                cv2.waitKey(1)
            except Exception: pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    gui_debug_node = None
    try:
        gui_debug_node = GuiDebugNode()
        rclpy.spin(gui_debug_node)
    except KeyboardInterrupt:
        pass # Kein Logging
    except Exception as e:
         print(f"FATALER FEHLER in GuiDebugNode main: {e}", file=sys.stderr)
         traceback.print_exc(file=sys.stderr)
         pass
    finally:
        if gui_debug_node and isinstance(gui_debug_node, Node) and rclpy.ok():
             gui_debug_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        # Sicherstellen, dass alle OpenCV Fenster geschlossen sind
        try: cv2.destroyAllWindows()
        except Exception: pass

if __name__ == '__main__':
    main()
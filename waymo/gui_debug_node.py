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
# NEU für Parameter-Deskriptoren
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange
from rclpy.parameter import ParameterType

class GuiDebugNode(Node):
    def __init__(self):
        super().__init__('gui_debug_node')

        

        # --- Konfiguration ---
        self.state_topic = '/robot/state'
        self.keyboard_command_topic = '/keyboard_command'
        self.debug_topics = OrderedDict([
            ('roi', '/debug/cam/roi'),
            ('lane_annotated', '/debug/cam/lane_annotated'),
            ('raw_markings', '/debug/cam/raw_markings'),
            ('warped', '/debug/cam/warped'),
            ('filtered_warped', '/debug/cam/filtered_warped'),
            ('sliding_window', '/debug/cam/sliding_window'),
            ('traffic_light_mask', '/debug/cam/traffic_mask'),
            ('traffic_light_overlay', '/debug/cam/traffic_overlay'),
            ('sign_detection_binary_with_box', '/debug/cam/binary_sign_boxed'),
        ])
        self.image_msg_type = CompressedImage
        self.gui_window_name = 'Waymo Debug Canvas'
        self.canvas_cols = 5
        self.placeholder_color = (40, 40, 40)
        self.placeholder_text_color = (9, 106, 206)
        # NEU: Feste Kachelgröße (basierend auf deiner Angabe 320x240)
        self.tile_width = 320
        self.tile_height = 240

        # NEU: Parameter für die Skalierung des gesamten Canvas
        scale_factor_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description='Skalierungsfaktor für das gesamte Debug-Canvas (0.1 bis 2.0)',
            floating_point_range=[FloatingPointRange(from_value=0.0001, to_value=2.0, step=0.0001)]
        )
        self.declare_parameter('canvas_scale_factor', 1.0, scale_factor_descriptor)

        # Initialisiere internen Status
        self.current_robot_state = "WAYMO_STARTED"
        self.canvas_visible = False
        self.last_images = {name: None for name in self.debug_topics.keys()}
        # self.last_canvas_shape nicht mehr nötig für Vergleich

        self.bridge = CvBridge()
        qos_debug_images = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        qos_reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1)

        # --- Subscriber ---
        self.state_subscriber = self.create_subscription(String, self.state_topic, self.state_callback, qos_reliable)
        self.keyboard_subscriber = self.create_subscription(String, self.keyboard_command_topic, self.keyboard_callback, qos_reliable)
        self.debug_image_subscribers = {}
        for name, topic in self.debug_topics.items():
            callback = lambda msg, topic_name=name: self.debug_image_callback(msg, topic_name)
            self.debug_image_subscribers[name] = self.create_subscription(
                self.image_msg_type, topic, callback, qos_debug_images
            )

        self.gui_initialized = False
        canvas_update_period = 0.1
        self.canvas_timer = self.create_timer(canvas_update_period, self.update_canvas)

        self.get_logger().info(f"GuiDebugNode gestartet. Drücke 'd' im Keyboard Handler, um das Debug-Canvas anzuzeigen.")
        self.get_logger().info(f"Skalierungsfaktor: {self.get_parameter('canvas_scale_factor').value}")
        self.get_logger().info(f"Robot Status: {self.current_robot_state}")


    def _create_placeholder_image(self, width, height, text="Warte..."): # Nimmt jetzt w/h als Argument
        """ Erstellt ein einfaches Platzhalterbild in der Zielgröße. """
        img = np.full((height, width, 3), self.placeholder_color, dtype=np.uint8)
        try:
            text_size, _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            text_x = (width - text_size[0]) // 2; text_y = (height + text_size[1]) // 2
            cv2.putText(img, text, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.placeholder_text_color, 1, cv2.LINE_AA)
        except Exception: pass
        return img

    def debug_image_callback(self, msg, topic_name):
        """ Speichert das zuletzt empfangene Bild für jedes Debug-Topic. """
        # Verarbeite immer, auch wenn Canvas nicht sichtbar ist, damit aktuelle Bilder da sind, wenn es geöffnet wird
        # if not self.canvas_visible: return
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            if topic_name in self.last_images:
                self.last_images[topic_name] = cv_image
        except CvBridgeError as e:
            self.get_logger().warn(f"CvBridge Error für '{topic_name}': {e}", throttle_duration_sec=10)
            if topic_name in self.last_images: self.last_images[topic_name] = None
        except Exception as e:
            self.get_logger().warn(f"Fehler beim Verarbeiten von '{topic_name}': {e}", throttle_duration_sec=10)
            if topic_name in self.last_images: self.last_images[topic_name] = None

    def state_callback(self, msg: String):
        # ... (unverändert) ...
        new_state = msg.data
        if new_state != self.current_robot_state:
            self.current_robot_state = new_state; self.get_logger().info(f"Robot Status: {self.current_robot_state}")

    def keyboard_callback(self, msg: String):
        # ... (unverändert) ...
        command = msg.data
        if command == 'toggle_debug_canvas':
            self.canvas_visible = not self.canvas_visible
            if self.canvas_visible:
                self.get_logger().info("Debug-Canvas wird angezeigt.")
                if not self.gui_initialized:
                     try:
                          # Verwende weiterhin AUTOSIZE, da wir das Bild *vorher* skalieren
                          cv2.namedWindow(self.gui_window_name, cv2.WINDOW_AUTOSIZE)
                          cv2.waitKey(1); self.gui_initialized = True
                     except cv2.error as e: self.get_logger().error(f"Fensterfehler: {e}. Canvas deaktiviert."); self.canvas_visible = False
            else:
                self.get_logger().info("Debug-Canvas wird geschlossen.")
                if self.gui_initialized:
                    try: cv2.destroyWindow(self.gui_window_name); cv2.waitKey(1); self.gui_initialized = False
                    except cv2.error: pass


    def update_canvas(self):
        """ Erstellt und aktualisiert das Canvas-Fenster, wenn sichtbar. """
        if not self.canvas_visible or not self.gui_initialized:
            return

        # Feste Zielgröße für jede Kachel verwenden
        target_h, target_w = self.tile_height, self.tile_width

        images_to_show = []
        # Erstelle Liste der Bilder (oder Platzhalter) in der festen Kachelgröße
        for name, img in self.last_images.items():
            if img is not None:
                try:
                     h, w = img.shape[:2]
                     if w == 0 or h == 0: # Ungültiges Bild
                          tile_img = self._create_placeholder_image(target_w, target_h, f"{name} (Invalid)")
                     else:
                          # Skaliere mit Randerhalt in die feste Kachelgröße
                          scale = min(target_w/w, target_h/h)
                          new_w, new_h = int(w*scale), int(h*scale)
                          if new_w <= 0 or new_h <= 0: # Fehler bei Skalierung
                               tile_img = self._create_placeholder_image(target_w, target_h, f"{name} (Resize Err)")
                          else:
                               resized_img = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_AREA)
                               tile_img = np.full((target_h, target_w, 3), self.placeholder_color, dtype=np.uint8)
                               x_offset = (target_w - new_w) // 2; y_offset = (target_h - new_h) // 2
                               tile_img[y_offset:y_offset+new_h, x_offset:x_offset+new_w] = resized_img

                     # Füge Titel hinzu
                     cv2.putText(tile_img, name, (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, self.placeholder_text_color, 1, cv2.LINE_AA)
                     images_to_show.append(tile_img)
                except Exception as e:
                     self.get_logger().warn(f"Fehler beim Skalieren von Bild '{name}': {e}", throttle_duration_sec=10)
                     images_to_show.append(self._create_placeholder_image(target_w, target_h, f"{name} (Error)"))
            else:
                images_to_show.append(self._create_placeholder_image(target_w, target_h, name))

        # Bestimme Anzahl der Zeilen
        num_images = len(images_to_show)
        if num_images == 0:
             # Erstelle Platzhalter-Canvas, falls keine Bilder vorhanden
             num_rows_expected = (len(self.debug_topics) + self.canvas_cols - 1) // self.canvas_cols
             canvas = self._create_placeholder_image(target_w * self.canvas_cols, target_h * num_rows_expected, "Keine Bilder")
        else:
            num_rows = (num_images + self.canvas_cols - 1) // self.canvas_cols
            needed_placeholders = num_rows * self.canvas_cols - num_images
            for _ in range(needed_placeholders): images_to_show.append(self._create_placeholder_image(target_w, target_h, ""))

            rows = []
            for i in range(num_rows):
                start_idx = i * self.canvas_cols; end_idx = start_idx + self.canvas_cols
                row_images = images_to_show[start_idx:end_idx]
                try: rows.append(np.hstack(row_images))
                except ValueError as e: self.get_logger().error(f"HStack Fehler Zeile {i}: {e}"); rows.append(np.full((target_h, target_w * self.canvas_cols, 3), self.placeholder_color, dtype=np.uint8))

            if not rows: canvas = self._create_placeholder_image(target_w*self.canvas_cols, target_h, "Fehler bei Zeilen")
            else:
                 try: canvas = np.vstack(rows)
                 except ValueError as e: self.get_logger().error(f"VStack Fehler: {e}"); canvas = self._create_placeholder_image(target_w*self.canvas_cols, target_h * len(rows), "Fehler bei Canvas")

        # --- NEU: Skaliere das gesamte Canvas basierend auf dem Parameter ---
        try:
            scale_factor = self.get_parameter('canvas_scale_factor').value
            if scale_factor < 1.0 and canvas.size > 0:
                new_h = int(canvas.shape[0] * scale_factor)
                new_w = int(canvas.shape[1] * scale_factor)
                # Stelle sicher, dass die Größe nicht 0 wird
                if new_h > 0 and new_w > 0:
                     canvas_display = cv2.resize(canvas, (new_w, new_h), interpolation=cv2.INTER_AREA)
                else:
                     canvas_display = canvas # Fallback auf Originalgröße bei Skalierungsfehler
            else:
                 canvas_display = canvas # Keine Skalierung nötig oder möglich
        except Exception as e:
             self.get_logger().error(f"Fehler beim Skalieren des Canvas: {e}")
             canvas_display = canvas # Fallback bei Fehler


        # Zeige das (potenziell skalierte) Canvas an
        try:
            if canvas_display.size > 0:
                 cv2.imshow(self.gui_window_name, canvas_display)
            # Wartezeit ist wichtig für GUI-Events
            cv2.waitKey(1)
        except cv2.error:
             self.get_logger().warn("Anzeigefehler (Fenster geschlossen?). Canvas wird deaktiviert.", throttle_duration_sec=10)
             self.canvas_visible = False; self.gui_initialized = False
        except Exception as e:
             self.get_logger().error(f"Canvas Update Fehler: {e}", throttle_duration_sec=10)


    def destroy_node(self):
       # ... (unverändert) ...
        # self.get_logger().info("Shutting down GuiDebugNode.")
        if self.gui_initialized:
            try: cv2.destroyWindow(self.gui_window_name); cv2.waitKey(1)
            except Exception: pass
        super().destroy_node()

def main(args=None):
    # ... (unverändert) ...
    rclpy.init(args=args)
    gui_debug_node = None
    try:
        gui_debug_node = GuiDebugNode()
        rclpy.spin(gui_debug_node)
    except KeyboardInterrupt: pass
    except Exception as e:
         print(f"FATALER FEHLER in GuiDebugNode main: {e}", file=sys.stderr); traceback.print_exc(file=sys.stderr); pass
    finally:
        if gui_debug_node and isinstance(gui_debug_node, Node) and rclpy.ok(): gui_debug_node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
        try: cv2.destroyAllWindows()
        except Exception: pass

if __name__ == '__main__':
    main()
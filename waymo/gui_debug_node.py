#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# Benötigte ROS-Nachrichtentypen und Tools
from sensor_msgs.msg import CompressedImage # Für das Bild
from std_msgs.msg import String             # Für die Statusnachrichten
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import sys
import traceback

class GuiDebugNode(Node):
    def __init__(self):
        super().__init__('gui_debug_node')

        # --- Konfiguration ---
        self.image_topic = '/lane/image_annotated'
        self.state_topic = '/robot/state'
        self.image_msg_type = CompressedImage
        self.gui_window_name = 'Kamerabild Debug'
        # Initialisiere internen Status mit dem Startwert
        self.current_robot_state = "WAYMO_STARTED"

        # --- Initialisierung ---
        self.bridge = CvBridge()

        # --- QoS Profile ---
        qos_profile_img = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1
        )
        qos_profile_state = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=1
        )

        # --- Subscriber ---
        self.image_subscriber = self.create_subscription(
            self.image_msg_type, self.image_topic, self.image_callback, qos_profile_img
        )
        self.state_subscriber = self.create_subscription(
            String, self.state_topic, self.state_callback, qos_profile_state
        )

        # --- GUI Setup (ohne Logging bei Fehler) ---
        self.gui_enabled = False
        try:
            cv2.namedWindow(self.gui_window_name, cv2.WINDOW_AUTOSIZE)
            self.placeholder_img = self._create_placeholder_image()
            cv2.imshow(self.gui_window_name, self.placeholder_img)
            cv2.waitKey(1)
            self.gui_enabled = True
        except cv2.error:
            self.placeholder_img = None

        # --- EINZIGE INITIALE Log-Ausgabe dieser Node ---
        self.get_logger().info(f"Robot Status: {self.current_robot_state}")
        # --- Ende der initialen Log-Ausgaben ---


    def _create_placeholder_image(self, width=640, height=480):
        """ Erstellt ein einfaches schwarzes Bild mit Text. """
        img = np.zeros((height, width, 3), dtype=np.uint8)
        # Kein Logging hier, nur Bild erstellen
        cv2.putText(img, "Warte auf Bild...", (50, height // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
        return img

    # --- Callback Funktion für Bilder (ohne Logging) ---
    def image_callback(self, msg, plot=False):
        if (plot):
            if not self.gui_enabled: return
            try:
                if self.image_msg_type == CompressedImage:
                    cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
                else:
                    cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                cv2.imshow(self.gui_window_name, cv_image)
                cv2.waitKey(1)
            except CvBridgeError: pass # Fehler still ignorieren
            except Exception: pass    # Fehler still ignorieren

    # --- Callback Funktion für Status (loggt nur bei Änderung) ---
    def state_callback(self, msg: String):
        """ Verarbeitet Statusnachrichten und loggt NUR Änderungen. """
        new_state = msg.data
        # Nur loggen, wenn sich der Status geändert hat
        if new_state != self.current_robot_state:
            self.current_robot_state = new_state
            # Logge die Status-ÄNDERUNG
            self.get_logger().info(f"Robot Status: {self.current_robot_state}")

    def destroy_node(self):
        """ Ressourcen freigeben beim Beenden (ohne Logging). """
        if self.gui_enabled:
             try: cv2.destroyAllWindows()
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
         # print(f"FATALER FEHLER in GuiDebugNode main: {e}", file=sys.stderr) # Auskommentiert
         # traceback.print_exc() # Auskommentiert
         pass
    finally:
        if gui_debug_node and isinstance(gui_debug_node, Node) and rclpy.ok():
             gui_debug_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
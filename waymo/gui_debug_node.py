#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# Importiere nur die benötigten ROS-Nachrichtentypen und Tools
from sensor_msgs.msg import Image, CompressedImage
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np # Wird für Platzhalterbild benötigt

class GuiDebugNode(Node):
    def __init__(self):
        super().__init__('gui_debug_node')

        # --- Konfiguration ---
        self.image_topic = '/lane/image_annotated' # Topic für das Kamerabild
        # Wähle den richtigen Bildtyp basierend auf dem Publisher
        # self.image_msg_type = Image
        self.image_msg_type = CompressedImage # Passe dies ggf. an Image an
        self.gui_window_name = 'Kamerabild Debug'

        # --- Initialisierung ---
        self.bridge = CvBridge()

        # --- QoS Profile ---
        # Passendes QoS für das Bildtopic wählen
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, # Oft gut für Video Streams
            history=HistoryPolicy.KEEP_LAST,
            depth=1 # Nur das neueste Bild ist relevant
        )

        # --- Subscriber ---
        self.image_subscriber = self.create_subscription(
            self.image_msg_type,
            self.image_topic,
            self.image_callback,
            qos_profile
        )

        # --- GUI Setup ---
        cv2.namedWindow(self.gui_window_name, cv2.WINDOW_AUTOSIZE)
        self.placeholder_img = self._create_placeholder_image()

        self.get_logger().info(f"GUI Debug Node gestartet. Zeige Bild von {self.image_topic}")
        # Zeige initiales Platzhalterbild an
        cv2.imshow(self.gui_window_name, self.placeholder_img)
        cv2.waitKey(1)


    def _create_placeholder_image(self, width=640, height=480):
        """ Erstellt ein einfaches schwarzes Bild mit Text. """
        img = np.zeros((height, width, 3), dtype=np.uint8)
        cv2.putText(img, "Warte auf Bild...", (50, height // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
        return img

    # --- Callback Funktion ---
    def image_callback(self, msg):
        """ Verarbeitet eingehende Bildnachrichten und zeigt sie an. """
        try:
            # Bild konvertieren
            if self.image_msg_type == CompressedImage:
                cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            else:
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Bild direkt anzeigen
            cv2.imshow(self.gui_window_name, cv_image)
            # Wichtig: Warte kurz, damit das Fenster Events verarbeiten kann
            cv2.waitKey(1)

        except CvBridgeError as e:
            self.get_logger().error(f'Fehler bei CvBridge Konvertierung: {e}')
            # Optional: Zeige wieder das Platzhalterbild bei Fehler
            # cv2.imshow(self.gui_window_name, self.placeholder_img)
            # cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'Unerwarteter Fehler in image_callback: {e}')


    def destroy_node(self):
        """ Ressourcen freigeben beim Beenden. """
        self.get_logger().info("Schließe GUI Debug Node...")
        cv2.destroyAllWindows() # Schließe OpenCV Fenster
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    gui_debug_node = GuiDebugNode()
    try:
        # Da die Anzeige direkt im Callback passiert, reicht spin()
        rclpy.spin(gui_debug_node)
    except KeyboardInterrupt:
        pass # Ctrl+C fängt das Programm ab
    finally:
        # Aufräumen
        gui_debug_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
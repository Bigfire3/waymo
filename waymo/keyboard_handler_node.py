# waymo/keyboard_handler_node.py

import rclpy
import rclpy.node
from std_msgs.msg import String
import sys
import select
import tty
import termios
import threading
import time
import traceback

NODE_NAME = 'keyboard_handler_node'
COMMAND_TOPIC = '/keyboard_command'
TOGGLE_PAUSE_KEY = 's'
TOGGLE_DEBUG_KEY = 'd'
TOGGLE_LANE_KEY = 'l'
TOGGLE_PARKING_KEY = 'p'
TOGGLE_TRAFFIC_LIGHT_KEY = 't'
TOGGLE_OBSTACLE_KEY = 'o'

class KeyboardHandlerNode(rclpy.node.Node):

    def __init__(self):
        super().__init__(NODE_NAME)
        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1 # Tiefe 1 ist okay für Befehle
        )
        self.command_publisher = self.create_publisher(String, COMMAND_TOPIC, qos_profile)

        self.get_logger().info(f'{NODE_NAME} started.')
        # --- Info-Text angepasst ---
        self.get_logger().info(f'Press "{TOGGLE_PAUSE_KEY}" to toggle pause, "{TOGGLE_DEBUG_KEY}" to toggle debug canvas (Ctrl+C to exit).')
        # --- Ende Anpassung ---

        self.stdin_fd = sys.stdin.fileno()
        self.old_term_settings = None
        if sys.stdin.isatty():
            try:
                self.old_term_settings = termios.tcgetattr(self.stdin_fd)
            except termios.error as e:
                self.get_logger().error(f"Is a TTY, but cannot get terminal attributes: {e}")
                rclpy.try_shutdown(); sys.exit(1)
        else:
             self.get_logger().error("Not running in a TTY. Keyboard input disabled.")
             rclpy.try_shutdown(); sys.exit(1)

        self.listener_thread_active = True
        self.listener_thread = threading.Thread(target=self.listen_keyboard, daemon=True)
        self.listener_thread.start()

    def listen_keyboard(self):
        """Lauscht auf Tastatureingaben."""
        settings_changed = False
        try:
            tty.setraw(self.stdin_fd)
            settings_changed = True
            while self.listener_thread_active and rclpy.ok():
                # Warte auf Eingabe, aber nicht blockierend, damit rclpy.ok() geprüft werden kann
                if select.select([sys.stdin], [], [], 0.1) == ([sys.stdin], [], []):
                    key = sys.stdin.read(1)
                    cmd_to_send = None

                    # --- Logik für verschiedene Tasten ---
                    if key == TOGGLE_PAUSE_KEY:
                        cmd_to_send = 'toggle_pause'
                        self.get_logger().info(f'"{TOGGLE_PAUSE_KEY}" pressed, sending "{cmd_to_send}" command.')
                    elif key == TOGGLE_DEBUG_KEY:
                        cmd_to_send = 'toggle_debug_canvas'
                        self.get_logger().info(f'"{TOGGLE_DEBUG_KEY}" pressed, sending "{cmd_to_send}" command.')
                    elif key == TOGGLE_LANE_KEY:
                        cmd_to_send = 'toggle_lane'
                        self.get_logger().info(f'"{TOGGLE_LANE_KEY}" pressed, sending "{cmd_to_send}" command.')
                    elif key == TOGGLE_PARKING_KEY:
                        cmd_to_send = 'toggle_parking'
                        self.get_logger().info(f'"{TOGGLE_PARKING_KEY}" pressed, sending "{cmd_to_send}" command.')
                    elif key == TOGGLE_TRAFFIC_LIGHT_KEY:
                        cmd_to_send = 'toggle_traffic_light'
                        self.get_logger().info(f'"{TOGGLE_TRAFFIC_LIGHT_KEY}" pressed, sending "{cmd_to_send}" command.')
                    elif key == TOGGLE_OBSTACLE_KEY:
                        cmd_to_send = 'toggle_obstacle'
                        self.get_logger().info(f'"{TOGGLE_OBSTACLE_KEY}" pressed, sending "{cmd_to_send}" command.')
                    elif key == '\x03': # Ctrl+C
                        self.get_logger().info('Ctrl+C detected in thread, stopping listener...')
                        self.listener_thread_active = False
                        # Versuche rclpy sauber herunterzufahren
                        if rclpy.ok(): rclpy.try_shutdown()
                        break # Thread-Schleife verlassen
                    # --- Ende Logik ---

                    # Sende Befehl, falls einer erkannt wurde
                    if cmd_to_send:
                        cmd_msg = String()
                        cmd_msg.data = cmd_to_send
                        try:
                           # Prüfe, ob Publisher noch gültig ist und Kontext OK ist
                           if self.command_publisher.handle and rclpy.ok() and self.context.ok():
                               self.command_publisher.publish(cmd_msg)
                           elif not self.command_publisher.handle:
                                if rclpy.ok(): self.get_logger().warn("Command publisher ist ungültig.", throttle_duration_sec=5)
                           elif not rclpy.ok():
                                # self.get_logger().info("RCLPY nicht mehr ok, sende keinen Befehl.") # Zu viel Log
                                break # Schleife verlassen, wenn rclpy nicht ok ist
                        except Exception as pub_e:
                            if rclpy.ok(): self.get_logger().error(f"Fehler beim Senden des Befehls '{cmd_to_send}': {pub_e}", throttle_duration_sec=5)


                # Kurze Pause, um CPU nicht zu belasten
                time.sleep(0.01)

        except termios.error as e:
             # Fehler kann auftreten, wenn Terminal währenddessen geschlossen wird
             if rclpy.ok(): self.get_logger().warn(f"Termios error in listener thread: {e}")
        except Exception as e:
             # Andere unerwartete Fehler
             if rclpy.ok(): self.get_logger().error(f"Exception in keyboard listener thread: {e}", throttle_duration_sec=5)
             traceback.print_exc(file=sys.stderr) # Traceback für Debugging ausgeben
        finally:
            # Terminal Einstellungen sicher wiederherstellen
            if settings_changed and self.old_term_settings:
                 try:
                     termios.tcsetattr(self.stdin_fd, termios.TCSADRAIN, self.old_term_settings)
                 except Exception as term_restore_e:
                     # Kann fehlschlagen, wenn Terminal weg ist
                     print(f"WARNUNG: Fehler beim Wiederherstellen der Terminaleinstellungen: {term_restore_e}", file=sys.stderr)
            self.listener_thread_active = False # Flag sicher setzen

    def destroy_node(self):
        """Aufräumarbeiten."""
        # self.get_logger().info("Shutting down Keyboard Handler Node...")
        self.listener_thread_active = False
        # Warte kurz auf den Thread, aber blockiere nicht ewig
        if hasattr(self, 'listener_thread') and self.listener_thread.is_alive():
            self.listener_thread.join(timeout=0.2)
        # Terminal wird im finally des Threads wiederhergestellt
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = None
    original_settings = None
    is_tty = sys.stdin.isatty()

    if is_tty:
        try:
            # Speichere originale Einstellungen GLEICH HIER
            original_settings = termios.tcgetattr(sys.stdin.fileno())
        except termios.error as e:
             print(f"FEHLER: Konnte initiale Terminal-Einstellungen nicht lesen: {e}", file=sys.stderr)
             is_tty = False # Behandle als non-tty
    else:
         print("WARNUNG: Läuft nicht in einem TTY. Tastatureingabe wird nicht funktionieren.", file=sys.stderr)

    try:
        if is_tty:
            node = KeyboardHandlerNode()
            rclpy.spin(node)
        else:
            # Halte kurz an, damit die Meldung gelesen werden kann, dann beenden
            print("Beende Keyboard Handler Node, da kein TTY vorhanden ist.", file=sys.stderr)
            time.sleep(2)
            # Kein rclpy.spin() aufrufen

    except KeyboardInterrupt:
        if node: pass # Log im destroy_node
        else: print("KeyboardInterrupt erhalten, bevor Node initialisiert wurde.", file=sys.stderr)
    except Exception as e:
        print(f"Unerwarteter Fehler im Keyboard Handler main: {e}", file=sys.stderr)
        traceback.print_exc()
    finally:
        # Node sauber beenden
        if node is not None:
             node.destroy_node() # Ruft Thread-Join und super().destroy_node() auf

        # Terminal Einstellungen *immer* versuchen wiederherzustellen, wenn original vorhanden
        if original_settings:
             try:
                  termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, original_settings)
                  # print("Terminaleinstellungen wiederhergestellt.", file=sys.stderr) # Debug Log
             except Exception as term_e:
                  # Kann fehlschlagen, wenn TTY weg ist oder nie richtig gesetzt wurde
                  print(f"WARNUNG: Fehler beim finalen Wiederherstellen der Terminaleinstellungen: {term_e}", file=sys.stderr)

        # rclpy herunterfahren, falls noch aktiv
        if rclpy.ok():
            rclpy.shutdown()
        # print("Keyboard Handler Node beendet.", file=sys.stderr) # Bestätigung

if __name__ == '__main__':
    main()
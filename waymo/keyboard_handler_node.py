# waymo/waymo/keyboard_handler_node.py

import rclpy
import rclpy.node
from std_msgs.msg import String
import sys
import select
import tty
import termios
import threading
import time
import traceback # Für Fehler in Main

NODE_NAME = 'keyboard_handler_node'
COMMAND_TOPIC = '/keyboard_command' # Topic zum Senden von Befehlen
TOGGLE_KEY = 's'

class KeyboardHandlerNode(rclpy.node.Node):

    def __init__(self):
        super().__init__(NODE_NAME)
        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.command_publisher = self.create_publisher(String, COMMAND_TOPIC, qos_profile)

        self.get_logger().info(f'{NODE_NAME} started.')
        self.get_logger().info(f'Press "{TOGGLE_KEY}" in this terminal to send toggle pause command (Ctrl+C to exit).')

        self.stdin_fd = sys.stdin.fileno()
        self.old_term_settings = None # Initialisieren
        if sys.stdin.isatty():
            try:
                self.old_term_settings = termios.tcgetattr(self.stdin_fd)
            except termios.error as e:
                self.get_logger().error(f"Is a TTY, but cannot get terminal attributes: {e}")
                rclpy.try_shutdown(); sys.exit(1)
        else:
             self.get_logger().error("Not running in a TTY. Keyboard input disabled.")
             rclpy.try_shutdown(); sys.exit(1) # Beenden wenn kein TTY

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
                if select.select([sys.stdin], [], [], 0.1) == ([sys.stdin], [], []):
                    key = sys.stdin.read(1)
                    if key == TOGGLE_KEY:
                        # Loggen vor dem Senden
                        self.get_logger().info(f'"{TOGGLE_KEY}" pressed, sending toggle_pause command.')
                        cmd_msg = String(); cmd_msg.data = 'toggle_pause'
                        try:
                           if rclpy.ok() and self.context.ok():
                               self.command_publisher.publish(cmd_msg)
                        except Exception: pass # Fehler beim Senden ignorieren
                    elif key == '\x03': # Ctrl+C
                        self.get_logger().info('Ctrl+C detected in thread, stopping listener...')
                        self.listener_thread_active = False
                        rclpy.try_shutdown()
                        break # Thread-Schleife verlassen
                time.sleep(0.01)
        except termios.error as e:
             if rclpy.ok(): self.get_logger().warn(f"Termios error in listener: {e}")
        except Exception as e:
             if rclpy.ok(): self.get_logger().error(f"Exception in keyboard listener: {e}", throttle_duration_sec=5)
        finally:
            # Terminal Einstellungen nur zurücksetzen, wenn sie geändert wurden UND wir sie haben
            if settings_changed and self.old_term_settings:
                 termios.tcsetattr(self.stdin_fd, termios.TCSADRAIN, self.old_term_settings)
            self.listener_thread_active = False # Sicherstellen

    def destroy_node(self):
        """Aufräumarbeiten."""
        self.listener_thread_active = False
        if hasattr(self, 'listener_thread') and self.listener_thread.is_alive():
            self.listener_thread.join(timeout=0.5)
        # Das Zurücksetzen des Terminals erfolgt jetzt zuverlässiger im finally Block des Threads
        # und im finally Block von main.
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = None
    original_settings = None
    is_tty = False
    if sys.stdin.isatty():
         is_tty = True
         original_settings = termios.tcgetattr(sys.stdin)
    else:
         print("WARNING: Not running in a TTY. Keyboard input will not work.", file=sys.stderr)

    try:
        if is_tty:
            node = KeyboardHandlerNode()
            rclpy.spin(node)
        else:
            print("Exiting because not run in a TTY.", file=sys.stderr)
            # Halte kurz an, damit die Meldung gelesen werden kann
            time.sleep(2)

    except KeyboardInterrupt:
        if node: pass # Loggen hier ist oft zu spät
            # node.get_logger().info("KeyboardInterrupt received.") # Verursacht oft Fehler
    except Exception as e:
        print(f"Unhandled exception in keyboard_handler main: {e}", file=sys.stderr)
        traceback.print_exc()
        # if node: node.get_logger().error(f"Unhandled exception: {e}", exc_info=True)
        # else: print(f"Unhandled exception before node init: {e}", file=sys.stderr)
    finally:
        # WICHTIG: Terminal Einstellungen wiederherstellen
        if is_tty and original_settings:
             try:
                  termios.tcsetattr(sys.stdin, termios.TCSADRAIN, original_settings)
             except Exception as term_e:
                  print(f"Error restoring terminal settings: {term_e}", file=sys.stderr)

        # Node sauber beenden
        if node:
             node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("Keyboard handler finished.", file=sys.stderr) # Bestätigung

if __name__ == '__main__':
    main()
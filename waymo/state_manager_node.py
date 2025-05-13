# waymo/state_manager_node.py

import rclpy
import rclpy.node
import numpy as np
import time
import sys
import traceback

from std_msgs.msg import String, Float64, Bool
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# --- Zustand für manuelle Pause ---
MANUAL_PAUSE_STATE = 'MANUAL_PAUSE'
KEYBOARD_COMMAND_TOPIC = '/keyboard_command'

# --- Roboter Zustände ---
STATE_FOLLOW_LANE = 'FOLLOW_LANE'
STATE_STOPPED_AT_OBSTACLE = 'STOPPED_AT_OBSTACLE'
STATE_PASSING_OBSTACLE = 'PASSING_OBSTACLE'
STATE_STOPPED_AT_TRAFFIC_LIGHT = 'STOPPED_AT_TRAFFIC_LIGHT'
STATE_STOPPED_AT_PARKING_SIGN = 'STOPPED_AT_PARKING_SIGN'


class StateMachine(rclpy.node.Node):

    def __init__(self):
        super().__init__('state_manager_node')
        self.declare_parameter('drivingspeed', 0.15)

        # Interne Variablen
        self.center_offset = 0.0
        self.state = STATE_STOPPED_AT_TRAFFIC_LIGHT
        self.manual_pause_active = False
        self.obstacle_is_blocking = False
        self.traffic_light_is_red = True
        self.obstacle_just_passed = False
        self.initial_traffic_light_check_done = False
        self.parking_sign_detected = False

        # QoS Profile
        qos_sensor = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        qos_reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1)

        # Subscribers
        self.obstacle_subscription = self.create_subscription(Bool, 'obstacle/blocked', self.obstacle_detection_callback, qos_sensor)
        self.offset_subscription = self.create_subscription(Float64, 'lane/center_offset', self.lane_detection_callback, qos_sensor)
        self.passed_subscription = self.create_subscription(Bool, '/obstacle/passed', self.obstacle_passed_callback, qos_reliable)
        self.traffic_light_subscription = self.create_subscription(Bool, 'traffic_light', self.traffic_light_callback, qos_reliable)
        self.keyboard_cmd_subscription = self.create_subscription(String, KEYBOARD_COMMAND_TOPIC, self.keyboard_command_callback, qos_reliable)
        self.sign_subscription = self.create_subscription(String, 'sign', self.sign_detection_callback, qos_reliable)

        # Publishers
        self.state_publisher_ = self.create_publisher(String, 'robot/state', qos_reliable)
        self.twist_publisher_ = self.create_publisher(Twist, 'cmd_vel', qos_reliable)

        # Control Timer
        control_loop_period = 0.005 # 200 Hz
        self.control_timer = self.create_timer(control_loop_period, self.control_loop_callback)

        # Initiale Zustandsmeldung und Stopp
        self.publish_current_state()
        self.send_cmd_vel(0.0, 0.0)

    def keyboard_command_callback(self, msg: String):
        # Minimales Logging nur für empfangenen Befehl, falls gewünscht
        # print(f"DEBUG [SM]: Received keyboard command: {msg.data}", file=sys.stderr)
        if msg.data == 'toggle_pause':
            self.toggle_manual_pause()

    def toggle_manual_pause(self):
        if not self.manual_pause_active:
            self.manual_pause_active = True
            self.send_cmd_vel(0.0, 0.0)
            pause_state_msg = String(); pause_state_msg.data = MANUAL_PAUSE_STATE
            try:
                 if rclpy.ok() and self.context.ok(): self.state_publisher_.publish(pause_state_msg)
            except Exception: pass
        else:
            self.manual_pause_active = False
            self.change_state(STATE_FOLLOW_LANE)
            self.publish_current_state() # Letzten internen Zustand publizieren

    def obstacle_detection_callback(self, msg: Bool):
        if self.manual_pause_active: return

        self.obstacle_is_blocking = msg.data
        if self.obstacle_is_blocking and self.state == STATE_STOPPED_AT_OBSTACLE:
            self.change_state(STATE_PASSING_OBSTACLE)

    def lane_detection_callback(self, msg: Float64):
        self.center_offset = msg.data

    def obstacle_passed_callback(self, msg: Bool):
        if self.manual_pause_active: return
        if msg.data and self.state == STATE_PASSING_OBSTACLE:
             self.obstacle_just_passed = True

    def traffic_light_callback(self, msg: Bool):
        if self.manual_pause_active: return
        if self.initial_traffic_light_check_done: return

        is_red = not msg.data
        if not is_red:
            if self.traffic_light_is_red:
                 self.traffic_light_is_red = False
                 self.initial_traffic_light_check_done = True
                 if self.traffic_light_subscription:
                     self.destroy_subscription(self.traffic_light_subscription)
                     self.traffic_light_subscription = None
        else:
             if not self.traffic_light_is_red:
                  self.traffic_light_is_red = True

    def sign_detection_callback(self, msg: Bool):
        if msg.data == "parking_sign_detected":
            self.change_state(STATE_STOPPED_AT_PARKING_SIGN)


    def control_loop_callback(self):
        if self.manual_pause_active:
             self.send_cmd_vel(0.0, 0.0)
             # Zustand wird in publish_current_state behandelt
             self.publish_current_state() # Sicherstellen, dass Pause publiziert wird
             return

        current_state = self.state
        next_state = current_state

        if self.obstacle_just_passed and current_state == STATE_PASSING_OBSTACLE:
             next_state = STATE_FOLLOW_LANE
             self.obstacle_just_passed = False
        elif current_state == STATE_STOPPED_AT_TRAFFIC_LIGHT and not self.initial_traffic_light_check_done and self.traffic_light_is_red:
            pass
        elif current_state == STATE_STOPPED_AT_PARKING_SIGN:
            pass
        elif not (current_state == STATE_STOPPED_AT_TRAFFIC_LIGHT and not self.initial_traffic_light_check_done and self.traffic_light_is_red):
            if self.obstacle_is_blocking:
                if current_state == STATE_FOLLOW_LANE:
                    next_state = STATE_STOPPED_AT_OBSTACLE
                elif current_state == STATE_STOPPED_AT_TRAFFIC_LIGHT and self.initial_traffic_light_check_done:
                     next_state = STATE_STOPPED_AT_OBSTACLE
                elif current_state == STATE_STOPPED_AT_OBSTACLE:
                    pass
            elif not self.obstacle_is_blocking and current_state != STATE_PASSING_OBSTACLE:
                 if current_state != STATE_FOLLOW_LANE:
                      next_state = STATE_FOLLOW_LANE

        if next_state != current_state:
            self.change_state(next_state)

        if self.state == STATE_FOLLOW_LANE:
            driving_speed = self.get_parameter('drivingspeed').get_parameter_value().double_value
            angular_z = self.center_offset
            max_angular_z = 1.0
            angular_z = np.clip(angular_z, -max_angular_z, max_angular_z)
            self.send_cmd_vel(driving_speed, angular_z)
        elif self.state in [STATE_STOPPED_AT_OBSTACLE, STATE_STOPPED_AT_TRAFFIC_LIGHT]:
            self.send_cmd_vel(0.0, 0.0)
        elif self.state == STATE_PASSING_OBSTACLE:
            pass

        self.publish_current_state() # Zustand periodisch senden

    def change_state(self, new_state):
        if self.manual_pause_active: return
        if self.state != new_state:
            # print(f"DEBUG [SM]: State changed from {self.state} -> {new_state}", file=sys.stderr) # Debug Log
            self.state = new_state
            self.publish_current_state()
            if self.state in [STATE_STOPPED_AT_OBSTACLE, STATE_STOPPED_AT_TRAFFIC_LIGHT, STATE_STOPPED_AT_PARKING_SIGN]:
                 self.send_cmd_vel(0.0, 0.0)

    def publish_current_state(self):
        state_to_publish = self.state if not self.manual_pause_active else MANUAL_PAUSE_STATE
        state_msg = String(); state_msg.data = state_to_publish
        try:
             if rclpy.ok() and self.context.ok(): self.state_publisher_.publish(state_msg)
        except Exception: pass

    def send_cmd_vel(self, linear_x, angular_z):
       is_stop_command = abs(linear_x) < 0.001 and abs(angular_z) < 0.001
       if self.manual_pause_active and not is_stop_command:
            return
       twist = Twist(); twist.linear.x = linear_x; twist.angular.z = float(angular_z)
       try:
           if rclpy.ok() and self.context.ok(): self.twist_publisher_.publish(twist)
       except Exception: pass

    def destroy_node(self):
        try:
             if rclpy.ok() and self.context.ok():
                  self.manual_pause_active = False
                  self.send_cmd_vel(0.0, 0.0); time.sleep(0.1)
        except Exception as e: pass
        super().destroy_node()

# --- main Funktion mit Delay ---
def main(args=None):
    rclpy.init(args=args); node = None
    try:
        node = StateMachine()
        try: time.sleep(0.5)
        except KeyboardInterrupt: raise
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    except Exception as e:
        print(f"FATAL ERROR [SM]: {e}", file=sys.stderr); traceback.print_exc()
    finally:
        if node is not None: node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()
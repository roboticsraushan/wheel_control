#!/usr/bin/env python3
"""
ROS2 (Humble) serial bridge for Arduino Mega motor controller.

- Subscribes to:
  * /motors (std_msgs/msg/Int16MultiArray): data=[left, right] in range [-255, 255]
  * /cmd_vel (geometry_msgs/msg/Twist): optional; parameters map twist to wheel PWM

- Sends to Arduino over serial: ASCII lines `M <left> <right>\n`

Parameters:
  ~port (string): Serial port (default: /dev/ttyACM0)
  ~baud (int): Baud rate (default: 115200)
  ~watchdog_ms (int): Send periodic stop if no commands (default: 600)
  ~use_cmd_vel (bool): If true, enable /cmd_vel subscriber (default: True)
  ~wheel_separation (float): m, default 0.40
  ~max_speed_mps (float): m/s corresponding to |PWM|=255, default 1.0

Usage:
  - Ensure ROS2 Humble environment is sourced.
  - Install pyserial if needed: pip install pyserial
  - Run: ros2 run with a proper package or execute directly with Python.
"""

import sys
import time
import threading

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int16MultiArray
from geometry_msgs.msg import Twist

try:
    import serial  # pyserial
except Exception as e:
    serial = None


class SerialMotorBridge(Node):
    def __init__(self):
        super().__init__('serial_motor_bridge')
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('watchdog_ms', 600)
        self.declare_parameter('use_cmd_vel', True)
        self.declare_parameter('wheel_separation', 0.75)
        self.declare_parameter('max_speed_mps', 1.0)

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.watchdog_ms = self.get_parameter('watchdog_ms').get_parameter_value().integer_value
        self.use_cmd_vel = self.get_parameter('use_cmd_vel').get_parameter_value().bool_value
        self.wheel_sep = self.get_parameter('wheel_separation').get_parameter_value().double_value
        self.max_speed = self.get_parameter('max_speed_mps').get_parameter_value().double_value

        if serial is None:
            self.get_logger().error('pyserial not available. Install with: pip install pyserial')
            raise RuntimeError('pyserial missing')

        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.02)
        except Exception as e:
            self.get_logger().error(f'Failed to open serial {self.port} @ {self.baud}: {e}')
            raise

        self.get_logger().info(f'Opened serial {self.port} @ {self.baud}')
        self.last_cmd_time = self.get_clock().now()
        self.lock = threading.Lock()
        
        # Store last PWM values for printing
        self.last_left_pwm = 0
        self.last_right_pwm = 0

        self.sub_motors = self.create_subscription(Int16MultiArray, 'motors', self.motors_cb, 10)
        if self.use_cmd_vel:
            self.sub_cmd = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_cb, 10)
        else:
            self.sub_cmd = None

        # watchdog timer
        self.timer = self.create_timer(self.watchdog_ms / 1000.0, self.watchdog_tick)


    def write_cmd(self, left: int, right: int):
        # SWAP left and right to match correct motor association
        r = max(-255, min(255, int(left)))
        l = max(-255, min(255, int(right)))
        line = f"M {l} {r}\n".encode('ascii')
        with self.lock:
            try:
                self.ser.write(line)
                self.ser.flush()  # Force immediate send
                # Throttled logging (every 1 second) to avoid slowing down
                self.get_logger().info(
                    f'Arduino CMD: M {l} {r}',
                    throttle_duration_sec=1.0
                )
            except Exception as e:
                self.get_logger().error(f'serial write failed: {e}')
                return
        self.last_cmd_time = self.get_clock().now()

    def motors_cb(self, msg: Int16MultiArray):
        data = list(msg.data)
        if len(data) < 2:
            self.get_logger().warn('motors message requires 2 elements [left,right]')
            return
        self.write_cmd(data[0], data[1])

    def cmd_vel_cb(self, msg: Twist):
        # Differential drive mapping: v (m/s), w (rad/s)
        v = float(msg.linear.x)
        w = float(msg.angular.z)
        # Wheel linear speeds
        v_l = v - (self.wheel_sep / 2.0) * w
        v_r = v + (self.wheel_sep / 2.0) * w
        # Map to PWM [-255,255]
        def to_pwm(vx: float) -> int:
            if self.max_speed <= 0.0:
                return 0
            pwm = int(255.0 * (vx / self.max_speed))
            if pwm > 255: pwm = 255
            if pwm < -255: pwm = -255
            return pwm
        left_pwm = to_pwm(v_l)
        right_pwm = to_pwm(v_r)
        self.write_cmd(left_pwm, right_pwm)
        print(f"Arduino CMD: M {left_pwm} {right_pwm}")
        # Store for external access
        self.last_left_pwm = left_pwm
        self.last_right_pwm = right_pwm

    def watchdog_tick(self):
        # If no command recently, send stop
        dt = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e6
        if dt > self.watchdog_ms * 1.5:
            with self.lock:
                try:
                    self.ser.write(b'S\n')
                except Exception as e:
                    self.get_logger().error(f'serial write failed: {e}')
            self.last_cmd_time = self.get_clock().now()


def main(argv=None):
    rclpy.init(args=argv)
    node = None
    try:
        node = SerialMotorBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)

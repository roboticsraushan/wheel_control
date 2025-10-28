#!/usr/bin/env python3
"""
teleop_keyboard.py

ROS 2 Humble keyboard teleoperation for differential drive.
- Default mode: publish /cmd_vel (geometry_msgs/Twist)
- Optional mode: publish /motors (std_msgs/Int16MultiArray with [left,right] PWM)

Key bindings (similar to teleop_twist_keyboard):
  Movement (v: linear, w: angular)
    u    i    o
    j    k    l
    m    ,    .

  where:
    i:  forward
    ,:  backward
    j:  rotate left
    l:  rotate right
    u/o: forward + rotate left/right
    m/.: backward + rotate left/right
    k or space: stop

Speed controls:
  q/z : increase/decrease both linear and angular scales
  w/x : increase/decrease linear scale only
  e/c : increase/decrease angular scale only

Parameters (ROS 2):
  ~mode (string): 'twist' (default) or 'motors'
  ~topic (string): topic name for publishing (default depends on mode: '/cmd_vel' or '/motors')
  ~lin (double): base linear speed (m/s) when pressing 'i' (default 0.2)
  ~ang (double): base angular speed (rad/s) when pressing 'j' (default 0.8)
  ~lin_step (double): multiplier step for linear (default 1.1)
  ~ang_step (double): multiplier step for angular (default 1.1)
  ~pwm_step (int): increment step for motors mode (default 25)
  ~pwm_max (int): max abs PWM for motors mode (default 255)

Usage examples:
  # Twist mode (default)
  ros2 run wheel_control teleop_keyboard

  # Motors PWM mode
  ros2 run wheel_control teleop_keyboard --ros-args -p mode:=motors -p topic:=/motors
"""

import sys
import select
import termios
import tty
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Int16MultiArray

MOVE_BINDINGS = {
    'i': (1, 0),
    ',': (-1, 0),
    'j': (0, 1),
    'l': (0, -1),
    'u': (1, 1),
    'o': (1, -1),
    'm': (-1, 1),
    '.': (-1, -1),
}

STOP_KEYS = {'k', ' '}

HELP_TEXT = """
Reading keyboard. Press Ctrl-C to quit.

Move:        u    i    o
             j    k    l
             m    ,    .

k/space: stop

Speed:  q/z (+/- both)   w/x (+/- linear)   e/c (+/- angular)
"""


def get_key(timeout=0.1):
    dr, _, _ = select.select([sys.stdin], [], [], timeout)
    if dr:
        return sys.stdin.read(1)
    return ''


class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        # Declare parameters
        self.declare_parameter('mode', 'twist')
        self.declare_parameter('topic', '')
        self.declare_parameter('lin', 0.2)
        self.declare_parameter('ang', 0.8)
        self.declare_parameter('lin_step', 1.1)
        self.declare_parameter('ang_step', 1.1)
        self.declare_parameter('pwm_step', 25)
        self.declare_parameter('pwm_max', 255)

        self.mode = self.get_parameter('mode').get_parameter_value().string_value
        topic_param = self.get_parameter('topic').get_parameter_value().string_value
        self.lin = float(self.get_parameter('lin').value)
        self.ang = float(self.get_parameter('ang').value)
        self.lin_step = float(self.get_parameter('lin_step').value)
        self.ang_step = float(self.get_parameter('ang_step').value)
        self.pwm_step = int(self.get_parameter('pwm_step').value)
        self.pwm_max = int(self.get_parameter('pwm_max').value)

        if self.mode not in ('twist', 'motors'):
            self.get_logger().warn(f"Unknown mode '{self.mode}', defaulting to 'twist'")
            self.mode = 'twist'

        if self.mode == 'twist':
            topic = topic_param if topic_param else '/cmd_vel'
            self.pub_twist = self.create_publisher(Twist, topic, 10)
            self.pub_motors = None
            self.get_logger().info(f"Publishing Twist on {topic}")
        else:
            topic = topic_param if topic_param else '/motors'
            self.pub_motors = self.create_publisher(Int16MultiArray, topic, 10)
            self.pub_twist = None
            self.left_pwm = 0
            self.right_pwm = 0
            self.get_logger().info(f"Publishing PWM on {topic}")

        self.print_help()

    def print_help(self):
        self.get_logger().info(HELP_TEXT)
        self.get_logger().info(f"Mode: {self.mode} | lin: {self.lin:.2f} m/s | ang: {self.ang:.2f} rad/s | pwm_step: {self.pwm_step}")

    def adjust_speed(self, key):
        if key == 'q':
            self.lin *= self.lin_step
            self.ang *= self.ang_step
        elif key == 'z':
            self.lin /= self.lin_step
            self.ang /= self.ang_step
        elif key == 'w':
            self.lin *= self.lin_step
        elif key == 'x':
            self.lin /= self.lin_step
        elif key == 'e':
            self.ang *= self.ang_step
        elif key == 'c':
            self.ang /= self.ang_step
        self.get_logger().info(f"lin: {self.lin:.2f} m/s, ang: {self.ang:.2f} rad/s")

    def publish_twist(self, x, th):
        msg = Twist()
        msg.linear.x = self.lin * x
        msg.angular.z = self.ang * th
        self.pub_twist.publish(msg)

    def publish_motors(self, x, th):
        # x: -1..1 forward/back, th: -1..1 left/right
        # Basic differential mixing: forward/back plus yaw
        if x == 0 and th == 0:
            self.left_pwm = 0
            self.right_pwm = 0
        else:
            self.left_pwm += int(self.pwm_step * x) + int(self.pwm_step * th)
            self.right_pwm += int(self.pwm_step * x) - int(self.pwm_step * th)
            # Clamp
            self.left_pwm = max(-self.pwm_max, min(self.pwm_max, self.left_pwm))
            self.right_pwm = max(-self.pwm_max, min(self.pwm_max, self.right_pwm))
        msg = Int16MultiArray()
        msg.data = [int(self.left_pwm), int(self.right_pwm)]
        self.pub_motors.publish(msg)

    def send_stop(self):
        if self.pub_twist is not None:
            self.publish_twist(0, 0)
        else:
            self.left_pwm = 0
            self.right_pwm = 0
            msg = Int16MultiArray()
            msg.data = [0, 0]
            self.pub_motors.publish(msg)


class TerminalContext:
    def __init__(self):
        self.settings = termios.tcgetattr(sys.stdin)

    def __enter__(self):
        tty.setcbreak(sys.stdin.fileno())
        return self

    def __exit__(self, exc_type, exc, tb):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


def main(argv=None):
    rclpy.init(args=argv)
    node = TeleopNode()

    with TerminalContext():
        try:
            while rclpy.ok():
                key = get_key(timeout=0.1)
                if key:
                    if key == '\u0003':  # Ctrl-C
                        break
                    if key in STOP_KEYS:
                        node.send_stop()
                        continue
                    if key in ('q', 'z', 'w', 'x', 'e', 'c'):
                        node.adjust_speed(key)
                        continue
                    if key in MOVE_BINDINGS:
                        x, th = MOVE_BINDINGS[key]
                        if node.mode == 'twist':
                            node.publish_twist(x, th)
                        else:
                            node.publish_motors(x, th)
                        continue
                rclpy.spin_once(node, timeout_sec=0.0)
        except KeyboardInterrupt:
            pass
        finally:
            node.send_stop()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)

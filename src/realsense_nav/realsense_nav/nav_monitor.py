#!/usr/bin/env python3
"""
Navigation Monitor Node
Prints: Goal Distance | Left PWM | Right PWM
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Twist
import math


class NavMonitor(Node):
    def __init__(self):
        super().__init__('nav_monitor')
        
        # State
        self.goal_distance = 0.0
        self.left_pwm = 0
        self.right_pwm = 0
        
        # Parameters for PWM calculation
        self.declare_parameter('wheel_separation', 0.40)
        self.declare_parameter('max_speed_mps', 1.0)
        
        # Subscribers
        self.goal_sub = self.create_subscription(
            PointStamped,
            '/goal/position',
            self.goal_callback,
            10
        )
        
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Print timer (10 Hz)
        self.create_timer(0.1, self.print_status)
    
    def goal_callback(self, msg):
        """Calculate distance to goal"""
        x = msg.point.x
        y = msg.point.y
        z = msg.point.z
        self.goal_distance = math.sqrt(x**2 + y**2 + z**2)
    
    def cmd_vel_callback(self, msg):
        """Calculate PWM from cmd_vel (same logic as serial_motor_bridge)"""
        wheel_sep = self.get_parameter('wheel_separation').value
        max_speed = self.get_parameter('max_speed_mps').value
        
        v = float(msg.linear.x)
        w = float(msg.angular.z)
        
        # Differential drive
        v_l = v - (wheel_sep / 2.0) * w
        v_r = v + (wheel_sep / 2.0) * w
        
        # Convert to PWM
        def to_pwm(vx: float) -> int:
            if max_speed <= 0.0:
                return 0
            pwm = int(255.0 * (vx / max_speed))
            return max(-255, min(255, pwm))
        
        self.left_pwm = to_pwm(v_l)
        self.right_pwm = to_pwm(v_r)
    
    def print_status(self):
        """Print the monitoring data"""
        print(f"Distance: {self.goal_distance:5.2f}m | Left PWM: {self.left_pwm:4d} | Right PWM: {self.right_pwm:4d}", flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = NavMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

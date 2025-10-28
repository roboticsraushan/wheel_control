#!/usr/bin/env python3
"""
Motor PWM Monitor
Shows left and right motor PWM values calculated from /cmd_vel
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class MotorPWMMonitor(Node):
    def __init__(self):
        super().__init__('motor_pwm_monitor')
        
        # Robot parameters (same as serial_motor_bridge)
        self.wheel_separation = 0.40  # meters
        self.max_speed_mps = 1.0      # meters per second
        
        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_cb,
            10
        )
        
        self.get_logger().info('Motor PWM Monitor started')
        self.get_logger().info('Monitoring /cmd_vel and calculating motor PWM...')
        self.get_logger().info('Format: LEFT_PWM | RIGHT_PWM')
        self.get_logger().info('Range: -255 to +255 (negative = reverse)')
        print('\n' + '='*50)
    
    def cmd_vel_cb(self, msg):
        # Extract velocity commands
        linear = msg.linear.x
        angular = msg.angular.z
        
        # Calculate differential drive wheel velocities
        v_left = linear - (self.wheel_separation / 2.0) * angular
        v_right = linear + (self.wheel_separation / 2.0) * angular
        
        # Convert to PWM (-255 to 255)
        left_pwm = int((v_left / self.max_speed_mps) * 255)
        right_pwm = int((v_right / self.max_speed_mps) * 255)
        
        # Clamp to valid range
        left_pwm = max(-255, min(255, left_pwm))
        right_pwm = max(-255, min(255, right_pwm))
        
        # Create visual bar
        left_bar = self.create_bar(left_pwm)
        right_bar = self.create_bar(right_pwm)
        
        print(f"\rLEFT: {left_pwm:4d} {left_bar}  |  RIGHT: {right_pwm:4d} {right_bar}", end='', flush=True)
    
    def create_bar(self, value):
        """Create visual bar representation of PWM value"""
        max_width = 20
        if value == 0:
            return '|' + ' ' * max_width + '|'
        
        width = int(abs(value) / 255.0 * max_width)
        width = max(1, min(width, max_width))
        
        if value > 0:
            # Forward (green)
            bar = ' ' * (max_width - width) + '█' * width
        else:
            # Reverse (red)
            bar = '█' * width + ' ' * (max_width - width)
        
        return '|' + bar + '|'


def main():
    rclpy.init()
    node = MotorPWMMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

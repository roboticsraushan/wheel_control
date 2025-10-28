#!/usr/bin/env python3
"""
Quick monitor to see what the segmentation node is doing
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Bool


class SegmentationMonitor(Node):
    def __init__(self):
        super().__init__('segmentation_monitor')
        
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.create_subscription(Point, '/segmentation/centroid', self.centroid_cb, 10)
        self.create_subscription(Bool, '/segmentation/navigable', self.navigable_cb, 10)
        
        self.get_logger().info('Monitoring segmentation output...')
        self.get_logger().info('Point camera at floor and adjust lighting for best results')
    
    def cmd_vel_cb(self, msg):
        self.get_logger().info(
            f'💨 CMD_VEL: linear={msg.linear.x:.3f} m/s, angular={msg.angular.z:.3f} rad/s'
        )
    
    def centroid_cb(self, msg):
        self.get_logger().info(
            f'🎯 CENTROID: x={msg.x:.1f}, y={msg.y:.1f} px'
        )
    
    def navigable_cb(self, msg):
        status = '✅ PATH CLEAR' if msg.data else '❌ NO PATH'
        self.get_logger().info(f'{status}')


def main():
    rclpy.init()
    node = SegmentationMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

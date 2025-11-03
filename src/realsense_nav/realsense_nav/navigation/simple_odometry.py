#!/usr/bin/env python3
"""
Simple Odometry Publisher

Publishes robot pose based on wheel odometry or dead reckoning.
For now, uses cmd_vel integration for simple pose estimation.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
import math
from rclpy.time import Time


class SimpleOdometryNode(Node):
    def __init__(self):
        super().__init__('simple_odometry')
        
        self.declare_parameter('publish_rate', 10.0)
        
        publish_rate = self.get_parameter('publish_rate').value
        
        # Current pose estimate
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Last update time
        self.last_time = self.get_clock().now()
        
        # Current velocity
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        self.pose_pub = self.create_publisher(PoseStamped, '/robot/pose', 10)
        
        self.create_timer(1.0 / publish_rate, self.publish_pose)
        
        self.get_logger().info('Simple Odometry started')
    
    def cmd_vel_callback(self, msg):
        """Update velocity from cmd_vel"""
        self.linear_vel = msg.linear.x
        self.angular_vel = msg.angular.z
    
    def publish_pose(self):
        """Integrate velocity and publish pose"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        if dt > 0.0:
            # Simple Euler integration
            self.theta += self.angular_vel * dt
            self.x += self.linear_vel * math.cos(self.theta) * dt
            self.y += self.linear_vel * math.sin(self.theta) * dt
            
            # Normalize theta
            self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        self.last_time = current_time
        
        # Publish pose
        msg = PoseStamped()
        msg.header.stamp = current_time.to_msg()
        msg.header.frame_id = 'odom'
        
        msg.pose.position.x = self.x
        msg.pose.position.y = self.y
        msg.pose.position.z = 0.0
        
        # Convert theta to quaternion
        msg.pose.orientation.z = math.sin(self.theta / 2.0)
        msg.pose.orientation.w = math.cos(self.theta / 2.0)
        
        self.pose_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleOdometryNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

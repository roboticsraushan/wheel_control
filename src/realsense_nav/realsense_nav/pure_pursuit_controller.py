#!/usr/bin/env python3
"""
Pure Pursuit Controller Node
Combines path following (centroid) with goal seeking (yellow cone)
Uses pure pursuit algorithm for smooth trajectory generation
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped
from std_msgs.msg import Bool, Float32MultiArray
import numpy as np
import math


class PurePursuitController(Node):
    def __init__(self):
        super().__init__('pure_pursuit_controller')
        
        # Parameters
        self.declare_parameter('lookahead_distance', 0.5)  # meters
        self.declare_parameter('max_linear_vel', 0.5)     # m/s
        self.declare_parameter('max_angular_vel', 1.0)    # rad/s
        self.declare_parameter('goal_threshold', 0.3)     # meters, stop distance
        self.declare_parameter('path_follow_weight', 0.3) # Weight for path following
        self.declare_parameter('goal_seek_weight', 0.7)   # Weight for goal seeking
        
        # State variables
        self.goal_position = None
        self.goal_detected = False
        self.path_centroid = None  # From segmentation
        self.navigable = False
        
        # Subscribers
        self.goal_sub = self.create_subscription(
            PointStamped,
            '/goal/position',
            self.goal_callback,
            10
        )
        
        self.goal_detected_sub = self.create_subscription(
            Bool,
            '/goal/detected',
            self.goal_detected_callback,
            10
        )
        
        self.centroid_sub = self.create_subscription(
            Float32MultiArray,
            '/segmentation/centroid',
            self.centroid_callback,
            10
        )
        
        self.navigable_sub = self.create_subscription(
            Bool,
            '/segmentation/navigable',
            self.navigable_callback,
            10
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        self.trajectory_pub = self.create_publisher(
            Float32MultiArray,
            '/pure_pursuit/trajectory',
            10
        )
        
        # Control timer
        self.create_timer(0.05, self.control_loop)  # 20 Hz
        
        self.get_logger().info('Pure Pursuit Controller started')
    
    def goal_callback(self, msg):
        self.goal_position = msg.point
    
    def goal_detected_callback(self, msg):
        self.goal_detected = msg.data
    
    def centroid_callback(self, msg):
        if len(msg.data) >= 2:
            self.path_centroid = (msg.data[0], msg.data[1])
    
    def navigable_callback(self, msg):
        self.navigable = msg.data
    
    def control_loop(self):
        """Main control loop using pure pursuit algorithm"""
        cmd = Twist()
        
        # Get parameters
        lookahead = self.get_parameter('lookahead_distance').value
        max_linear = self.get_parameter('max_linear_vel').value
        max_angular = self.get_parameter('max_angular_vel').value
        goal_thresh = self.get_parameter('goal_threshold').value
        path_weight = self.get_parameter('path_follow_weight').value
        goal_weight = self.get_parameter('goal_seek_weight').value
        
        # Trajectory data for visualization [goal_x, goal_y, path_angle, goal_angle, combined_angle]
        trajectory_data = Float32MultiArray()
        
        if self.goal_detected and self.goal_position is not None:
            # Goal-seeking mode with pure pursuit
            goal_x = self.goal_position.x
            goal_y = self.goal_position.y
            
            # Calculate distance to goal
            distance = math.sqrt(goal_x**2 + goal_y**2)
            
            if distance < goal_thresh:
                # Reached goal, stop
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.get_logger().info('Goal reached!', throttle_duration_sec=2.0)
            else:
                # Pure pursuit algorithm
                # Calculate lookahead point on path to goal
                if distance > lookahead:
                    # Use lookahead distance
                    alpha = math.atan2(goal_y, goal_x)
                    target_x = lookahead * math.cos(alpha)
                    target_y = lookahead * math.sin(alpha)
                else:
                    # Use actual goal if closer than lookahead
                    target_x = goal_x
                    target_y = goal_y
                
                # Calculate curvature for pure pursuit
                # k = 2 * sin(alpha) / L, where alpha is angle to target, L is lookahead
                alpha = math.atan2(target_y, target_x)
                curvature = 2.0 * math.sin(alpha) / lookahead
                
                # Calculate goal-seeking angular velocity
                goal_angular = curvature * max_linear
                
                # If path is navigable, blend with path following
                if self.navigable and self.path_centroid is not None:
                    # Path following component (proportional control)
                    # Assuming image width ~640 or scaled centroid
                    cx = self.path_centroid[0]
                    # Normalize error (assume center is at ~320 for typical image)
                    image_center = 320.0
                    error = (cx - image_center) / image_center
                    path_angular = -1.0 * error  # Proportional control
                    
                    # Blend path following and goal seeking
                    combined_angular = (path_weight * path_angular + 
                                      goal_weight * goal_angular)
                    
                    trajectory_data.data = [
                        float(goal_x), float(goal_y),
                        float(path_angular), float(goal_angular), float(combined_angular)
                    ]
                else:
                    # Only goal seeking
                    combined_angular = goal_angular
                    trajectory_data.data = [
                        float(goal_x), float(goal_y),
                        0.0, float(goal_angular), float(combined_angular)
                    ]
                
                # Set velocities
                cmd.linear.x = max_linear * (1.0 - abs(alpha) / math.pi)  # Slow down for sharp turns
                cmd.angular.z = max(-max_angular, min(max_angular, combined_angular))
                
                self.get_logger().info(
                    f'Pure Pursuit: dist={distance:.2f}m, angle={math.degrees(alpha):.1f}°, '
                    f'lin={cmd.linear.x:.2f}, ang={cmd.angular.z:.2f}',
                    throttle_duration_sec=0.5
                )
        
        elif self.navigable and self.path_centroid is not None:
            # Path following mode only (no goal detected)
            cx = self.path_centroid[0]
            image_center = 320.0
            error = (cx - image_center) / image_center
            
            cmd.linear.x = max_linear * (1.0 - abs(error))
            cmd.angular.z = -1.0 * error
            cmd.angular.z = max(-max_angular, min(max_angular, cmd.angular.z))
            
            trajectory_data.data = [0.0, 0.0, float(cmd.angular.z), 0.0, float(cmd.angular.z)]
        
        else:
            # No goal and no path, stop
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            trajectory_data.data = [0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Publish commands
        self.cmd_vel_pub.publish(cmd)
        self.trajectory_pub.publish(trajectory_data)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

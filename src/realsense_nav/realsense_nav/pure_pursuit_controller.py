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
        self.declare_parameter('goal_threshold', 0.8)     # meters, stop distance (80 cm)
        self.declare_parameter('path_follow_weight', 0.3) # Weight for path following
        self.declare_parameter('goal_seek_weight', 0.7)   # Weight for goal seeking
        # State variables
        self.goal_position = None
        self.goal_detected = False
        self.path_centroid = None  # From segmentation
        self.navigable = False
        
        # Smoothing and timeout
        self.prev_linear_vel = 0.0
        self.prev_angular_vel = 0.0
        self.alpha_smooth = 0.3  # Smoothing factor (0=no change, 1=instant)
        self.last_goal_time = self.get_clock().now()
        self.goal_timeout = 0.5  # seconds
        
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
        
        # Publisher for BT state completion
        self.state_complete_pub = self.create_publisher(
            Bool,
            '/behavior_state_complete',
            10
        )
        # Control timer
        self.create_timer(0.02, self.control_loop)  # 50 Hz for smoother control
        self.get_logger().info('Pure Pursuit Controller started')
    
    def goal_callback(self, msg):
        self.goal_position = msg.point
        self.last_goal_time = self.get_clock().now()  # Update timestamp
    
    def goal_detected_callback(self, msg):
        self.goal_detected = msg.data
        if msg.data:
            self.last_goal_time = self.get_clock().now()  # Update timestamp
    
    def centroid_callback(self, msg):
        if len(msg.data) >= 2:
            self.path_centroid = (msg.data[0], msg.data[1])
    
    def navigable_callback(self, msg):
        self.navigable = msg.data
    
    def control_loop(self):
        """Main control loop using pure pursuit algorithm"""
        cmd = Twist()
        print('control_loop tick')
        # Get parameters
        lookahead = self.get_parameter('lookahead_distance').value
        max_linear = self.get_parameter('max_linear_vel').value
        max_angular = self.get_parameter('max_angular_vel').value
        goal_thresh = self.get_parameter('goal_threshold').value
        path_weight = self.get_parameter('path_follow_weight').value
        goal_weight = self.get_parameter('goal_seek_weight').value

        
        # Check if goal data is stale
        current_time = self.get_clock().now()
        time_since_goal = (current_time - self.last_goal_time).nanoseconds / 1e9
        goal_is_fresh = time_since_goal < self.goal_timeout
        
        # Trajectory data for visualization [goal_x, goal_y, path_angle, goal_angle, combined_angle]
        trajectory_data = Float32MultiArray()
        
        # Desired velocities (will be smoothed later)
        desired_linear = 0.0
        desired_angular = 0.0
        self.get_logger().debug(f'Goal detected: {self.goal_detected}, Fresh: {goal_is_fresh}, Navigable: {self.navigable}')
        if self.goal_detected and self.goal_position is not None and goal_is_fresh:
            # Goal-seeking mode with pure pursuit
            goal_x = self.goal_position.x
            goal_y = self.goal_position.y
            
            # Calculate distance to goal
            distance = math.sqrt(goal_x**2 + goal_y**2)
            self.get_logger().info(f"distance: {distance}, goal_thresh: {goal_thresh}")

            if distance < goal_thresh:
                # Reached goal, stop
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.get_logger().info(f"distance: {distance}, goal_thresh: {goal_thresh}")

                # Publish state complete to BT
                self.state_complete_pub.publish(Bool(data=True))
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
                    self.get_logger().info('control_loop tick')
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
                
                # Set desired velocities
                desired_linear = max_linear * (1.0 - abs(alpha) / math.pi)  # Slow down for sharp turns
                desired_angular = max(-max_angular, min(max_angular, combined_angular))
        
        elif self.navigable and self.path_centroid is not None:
            # Path following mode only (no goal detected or stale goal)
            cx = self.path_centroid[0]
            image_center = 320.0
            error = (cx - image_center) / image_center
            
            desired_linear = max_linear * (1.0 - abs(error))
            desired_angular = -1.0 * error
            desired_angular = max(-max_angular, min(max_angular, desired_angular))
            
            trajectory_data.data = [0.0, 0.0, float(desired_angular), 0.0, float(desired_angular)]
        
        else:
            # No goal and no path, gradually stop
            desired_linear = 0.0
            desired_angular = 0.0
            trajectory_data.data = [0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Apply exponential smoothing for smooth motion
        # new_vel = alpha * desired + (1 - alpha) * prev
        cmd.linear.x = (self.alpha_smooth * desired_linear + 
                       (1.0 - self.alpha_smooth) * self.prev_linear_vel)
        cmd.angular.z = (self.alpha_smooth * desired_angular + 
                        (1.0 - self.alpha_smooth) * self.prev_angular_vel)
        
        # Update previous velocities
        self.prev_linear_vel = cmd.linear.x
        self.prev_angular_vel = cmd.angular.z
        
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

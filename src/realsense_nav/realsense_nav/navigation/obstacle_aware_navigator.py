#!/usr/bin/env python3
"""
Obstacle-Aware Junction Navigator

Enhanced junction navigator with obstacle avoidance.
Uses floor segmentation to detect and avoid obstacles while navigating to junction.
"""
import rclpy
from rclpy.node import Node
from realsense_nav.msg import JunctionPath, JunctionDetection
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from cv_bridge import CvBridge
import cv2
import numpy as np
import json


class ObstacleAwareNavigatorNode(Node):
    def __init__(self):
        super().__init__('obstacle_aware_navigator')
        
        self.declare_parameter('arrival_threshold', 0.8)
        self.declare_parameter('linear_vel', 0.3)
        self.declare_parameter('angular_vel', 0.5)
        self.declare_parameter('obstacle_threshold', 0.3)  # 30% of image blocked = obstacle
        self.declare_parameter('min_clear_width', 0.4)  # 40% width must be clear
        
        self.arrival_threshold = self.get_parameter('arrival_threshold').value
        self.linear_vel = self.get_parameter('linear_vel').value
        self.angular_vel = self.get_parameter('angular_vel').value
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        self.min_clear_width = self.get_parameter('min_clear_width').value
        
        self.bridge = CvBridge()
        self.junction_path = []
        self.current_index = 0
        self.navigating = False
        self.current_confidence = 0.0
        self.junction_visible = True
        self.floor_mask = None
        
        self.create_subscription(JunctionPath, '/navigation/junction_path', self.path_callback, 10)
        self.create_subscription(JunctionDetection, '/junction/detected', self.detection_callback, 10)
        self.create_subscription(Bool, '/junction/visibility', self.visibility_callback, 10)
        self.create_subscription(Image, '/segmentation/image', self.segmentation_callback, 10)
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.expected_pub = self.create_publisher(String, '/navigation/expected_junction', 10)
        self.status_pub = self.create_publisher(String, '/navigation/status', 10)
        
        self.create_timer(0.1, self.navigation_loop)
        
        self.get_logger().info('Obstacle-Aware Navigator ready')
    
    def path_callback(self, msg):
        """Receive new junction path"""
        self.junction_path = msg.junction_ids
        self.current_index = 0
        self.navigating = True
        
        self.get_logger().info(f'New path: {" → ".join(self.junction_path)}')
        
        if self.junction_path:
            self._publish_expected_junction(self.junction_path[self.current_index])
    
    def detection_callback(self, msg):
        """Handle junction detection"""
        if not self.navigating or self.current_index >= len(self.junction_path):
            return
        
        expected = self.junction_path[self.current_index]
        
        if msg.junction_id == expected and msg.is_expected:
            self.current_confidence = msg.confidence
            
            if self.current_confidence >= self.arrival_threshold:
                self._junction_arrived()
    
    def visibility_callback(self, msg):
        """Update junction visibility status"""
        self.junction_visible = msg.data
        
        if not self.junction_visible and self.navigating:
            self.get_logger().warn('⚠️ Junction lost from view!')
            self._publish_status('warning', 'Junction not visible')
    
    def segmentation_callback(self, msg):
        """Store floor segmentation mask"""
        try:
            self.floor_mask = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        except Exception as e:
            self.get_logger().debug(f'Segmentation conversion failed: {e}')
    
    def _detect_obstacles(self):
        """
        Analyze floor mask to detect obstacles.
        Returns: (obstacle_detected, turn_direction)
        """
        if self.floor_mask is None:
            return False, 0.0
        
        h, w = self.floor_mask.shape
        
        # Analyze bottom half of image (closer to robot)
        roi = self.floor_mask[h//2:, :]
        
        # Calculate navigable area
        navigable_pixels = np.sum(roi > 128)
        total_pixels = roi.size
        navigable_ratio = navigable_pixels / total_pixels
        
        # Check if path is blocked
        if navigable_ratio < (1.0 - self.obstacle_threshold):
            # Obstacle detected, determine which direction to turn
            
            # Split into left and right halves
            left_half = roi[:, :w//2]
            right_half = roi[:, w//2:]
            
            left_navigable = np.sum(left_half > 128) / left_half.size
            right_navigable = np.sum(right_half > 128) / right_half.size
            
            # Turn toward more navigable side
            if left_navigable > right_navigable:
                return True, self.angular_vel  # Turn left
            else:
                return True, -self.angular_vel  # Turn right
        
        return False, 0.0
    
    def _junction_arrived(self):
        """Handle arrival at junction"""
        arrived_junction = self.junction_path[self.current_index]
        self.get_logger().info(f'✓ Arrived at {arrived_junction}')
        
        self._stop_robot()
        
        self.current_index += 1
        
        if self.current_index >= len(self.junction_path):
            self.navigating = False
            self._publish_status('arrived', 'Destination reached')
            self.get_logger().info('🎯 Navigation complete!')
        else:
            next_junction = self.junction_path[self.current_index]
            self._publish_expected_junction(next_junction)
            self._publish_status('navigating', f'Moving to {next_junction}')
    
    def navigation_loop(self):
        """Main navigation control loop with obstacle avoidance"""
        if not self.navigating or self.current_index >= len(self.junction_path):
            return
        
        cmd = Twist()
        
        # Check if junction is visible
        if not self.junction_visible:
            # Stop and wait for junction to be visible again
            self._stop_robot()
            return
        
        # Check for obstacles
        obstacle_detected, turn_direction = self._detect_obstacles()
        
        if obstacle_detected:
            # Obstacle avoidance: turn while moving slowly
            cmd.linear.x = self.linear_vel * 0.3
            cmd.angular.z = turn_direction
            self.get_logger().debug('Avoiding obstacle')
        else:
            # Clear path: move forward
            cmd.linear.x = self.linear_vel
            cmd.angular.z = 0.0
        
        self.cmd_pub.publish(cmd)
    
    def _stop_robot(self):
        """Stop robot movement"""
        cmd = Twist()
        self.cmd_pub.publish(cmd)
    
    def _publish_expected_junction(self, junction_id):
        """Publish expected junction ID"""
        msg = String()
        msg.data = junction_id
        self.expected_pub.publish(msg)
    
    def _publish_status(self, state, message):
        """Publish navigation status"""
        status = {
            'state': state,
            'message': message,
            'current_index': self.current_index,
            'total_junctions': len(self.junction_path),
            'junction_visible': self.junction_visible,
            'timestamp': self.get_clock().now().to_msg().sec
        }
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAwareNavigatorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

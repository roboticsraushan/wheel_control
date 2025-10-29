#!/usr/bin/env python3
"""
Goal Detection Node
Detects yellow traffic cone using HSV color segmentation
Publishes goal position for pure pursuit controller
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np


class GoalDetectionNode(Node):
    def __init__(self):
        super().__init__('goal_detection_node')
        
        # Parameters for yellow cone detection
        self.declare_parameter('yellow_h_min', 20)
        self.declare_parameter('yellow_h_max', 35)
        self.declare_parameter('yellow_s_min', 100)
        self.declare_parameter('yellow_s_max', 255)
        self.declare_parameter('yellow_v_min', 100)
        self.declare_parameter('yellow_v_max', 255)
        self.declare_parameter('min_cone_area', 500)
        self.declare_parameter('max_cone_area', 50000)
        
        self.bridge = CvBridge()
        
        # Camera parameters
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        
        # Subscribers
        self.color_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.color_callback,
            10
        )
        
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',
            self.depth_callback,
            10
        )
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',
            self.camera_info_callback,
            10
        )
        
        # Publishers
        self.goal_pub = self.create_publisher(
            PointStamped,
            '/goal/position',
            10
        )
        
        self.goal_detected_pub = self.create_publisher(
            Bool,
            '/goal/detected',
            10
        )
        
        self.goal_image_pub = self.create_publisher(
            Image,
            '/goal/image',
            10
        )
        
        self.depth_image = None
        
        self.get_logger().info('Goal Detection Node started')
        self.get_logger().info('Looking for yellow traffic cone...')
    
    def camera_info_callback(self, msg):
        if self.fx is None:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.get_logger().info(f'Camera intrinsics received: fx={self.fx:.1f}, fy={self.fy:.1f}')
    
    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'Error converting depth image: {e}')
    
    def color_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Detect yellow cone
            cone_detected, cone_center, cone_depth, vis_image = self.detect_yellow_cone(cv_image)
            
            # Publish visualization
            if vis_image is not None:
                vis_msg = self.bridge.cv2_to_imgmsg(vis_image, encoding='bgr8')
                vis_msg.header = msg.header
                self.goal_image_pub.publish(vis_msg)
            
            # Publish detection status
            detected_msg = Bool()
            detected_msg.data = cone_detected
            self.goal_detected_pub.publish(detected_msg)
            
            # If cone detected and we have depth and camera params, compute 3D position
            if cone_detected and cone_depth > 0 and self.fx is not None:
                goal_msg = PointStamped()
                goal_msg.header = msg.header
                goal_msg.header.frame_id = 'camera_color_optical_frame'
                
                # Convert pixel coordinates to 3D position
                x = (cone_center[0] - self.cx) * cone_depth / self.fx
                y = (cone_center[1] - self.cy) * cone_depth / self.fy
                z = cone_depth
                
                goal_msg.point.x = z  # Forward in camera frame
                goal_msg.point.y = -x  # Left/right (negated for ROS convention)
                goal_msg.point.z = -y  # Up/down (negated for ROS convention)
                
                self.goal_pub.publish(goal_msg)
        
        except Exception as e:
            self.get_logger().error(f'Error in color callback: {e}')
    
    def detect_yellow_cone(self, image):
        """Detect yellow cone using HSV color segmentation"""
        h_min = self.get_parameter('yellow_h_min').value
        h_max = self.get_parameter('yellow_h_max').value
        s_min = self.get_parameter('yellow_s_min').value
        s_max = self.get_parameter('yellow_s_max').value
        v_min = self.get_parameter('yellow_v_min').value
        v_max = self.get_parameter('yellow_v_max').value
        min_area = self.get_parameter('min_cone_area').value
        max_area = self.get_parameter('max_cone_area').value
        
        # Convert to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Create mask for yellow color
        lower_yellow = np.array([h_min, s_min, v_min])
        upper_yellow = np.array([h_max, s_max, v_max])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        
        # Morphological operations to clean up mask
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Create visualization image
        vis_image = image.copy()
        cv2.putText(vis_image, 'GOAL DETECTION', (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
        
        if not contours:
            cv2.putText(vis_image, 'NO CONE DETECTED', (10, 70),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            return False, None, 0, vis_image
        
        # Find largest contour within area range
        largest_contour = None
        largest_area = 0
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if min_area < area < max_area and area > largest_area:
                largest_area = area
                largest_contour = contour
        
        if largest_contour is None:
            cv2.putText(vis_image, 'NO VALID CONE', (10, 70),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            return False, None, 0, vis_image
        
        # Calculate centroid
        M = cv2.moments(largest_contour)
        if M['m00'] == 0:
            return False, None, 0, vis_image
        
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        
        # Get depth at cone center
        depth = 0
        if self.depth_image is not None:
            depth_h, depth_w = self.depth_image.shape
            # Scale coordinates if depth image has different resolution
            scale_x = depth_w / image.shape[1]
            scale_y = depth_h / image.shape[0]
            depth_x = int(cx * scale_x)
            depth_y = int(cy * scale_y)
            
            # Average depth in small region around center
            region = self.depth_image[
                max(0, depth_y-5):min(depth_h, depth_y+5),
                max(0, depth_x-5):min(depth_w, depth_x+5)
            ]
            valid_depths = region[region > 0]
            if len(valid_depths) > 0:
                depth = np.median(valid_depths) / 1000.0  # Convert mm to meters
        
        # Draw visualization
        cv2.drawContours(vis_image, [largest_contour], -1, (0, 255, 255), 3)
        cv2.circle(vis_image, (cx, cy), 10, (0, 0, 255), -1)
        cv2.circle(vis_image, (cx, cy), 15, (0, 255, 255), 2)
        
        # Draw bounding box
        x, y, w, h = cv2.boundingRect(largest_contour)
        cv2.rectangle(vis_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
        
        # Add text info
        cv2.putText(vis_image, f'CONE DETECTED', (10, 70),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(vis_image, f'Depth: {depth:.2f}m', (10, 100),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(vis_image, f'Area: {int(largest_area)}', (10, 130),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(vis_image, f'Center: ({cx}, {cy})', (10, 160),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        return True, (cx, cy), depth, vis_image


def main(args=None):
    rclpy.init(args=args)
    node = GoalDetectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

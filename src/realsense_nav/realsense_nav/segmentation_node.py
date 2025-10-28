#!/usr/bin/env python3
"""
Semantic Segmentation Node for RealSense D455
Performs real-time semantic segmentation to identify navigable areas.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np


class SegmentationNode(Node):
    def __init__(self):
        super().__init__('segmentation_node')
        
        # Declare parameters
        self.declare_parameter('segmentation_method', 'color_based')  # color_based, deeplab, etc.
        self.declare_parameter('min_area', 5000)  # Minimum navigable area in pixels
        self.declare_parameter('target_distance', 1.0)  # Target distance in meters
        self.declare_parameter('linear_gain', 0.5)  # Linear velocity gain
        self.declare_parameter('angular_gain', 1.0)  # Angular velocity gain
        self.declare_parameter('max_linear_vel', 0.5)  # Max linear velocity m/s
        self.declare_parameter('max_angular_vel', 1.0)  # Max angular velocity rad/s
        
        # Get parameters
        self.seg_method = self.get_parameter('segmentation_method').value
        self.min_area = self.get_parameter('min_area').value
        self.target_distance = self.get_parameter('target_distance').value
        self.linear_gain = self.get_parameter('linear_gain').value
        self.angular_gain = self.get_parameter('angular_gain').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Camera info
        self.camera_info = None
        self.image_width = 640
        self.image_height = 480
        
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
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.seg_image_pub = self.create_publisher(Image, '/segmentation/image', 10)
        self.centroid_pub = self.create_publisher(Point, '/segmentation/centroid', 10)
        self.navigable_pub = self.create_publisher(Bool, '/segmentation/navigable', 10)
        
        # State variables
        self.latest_depth = None
        self.centroid = None
        
        self.get_logger().info(f'Segmentation node initialized with method: {self.seg_method}')
    
    def camera_info_callback(self, msg):
        """Store camera info for coordinate transformations."""
        if self.camera_info is None:
            self.camera_info = msg
            self.image_width = msg.width
            self.image_height = msg.height
            self.get_logger().info(f'Camera info received: {self.image_width}x{self.image_height}')
    
    def depth_callback(self, msg):
        """Store latest depth image."""
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'Failed to convert depth image: {e}')
    
    def color_callback(self, msg):
        """Process color image and perform segmentation."""
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Perform segmentation based on method
            if self.seg_method == 'color_based':
                seg_mask = self.color_based_segmentation(cv_image)
            else:
                self.get_logger().warn(f'Unknown segmentation method: {self.seg_method}')
                return
            
            # Find navigable area centroid
            centroid, area = self.find_centroid(seg_mask)
            
            # Publish segmentation visualization
            self.publish_segmentation_image(cv_image, seg_mask, centroid)
            
            # Publish centroid
            if centroid is not None:
                centroid_msg = Point()
                centroid_msg.x = float(centroid[0])
                centroid_msg.y = float(centroid[1])
                centroid_msg.z = 0.0
                self.centroid_pub.publish(centroid_msg)
                self.centroid = centroid
            
            # Check if area is navigable
            is_navigable = area > self.min_area and centroid is not None
            navigable_msg = Bool()
            navigable_msg.data = is_navigable
            self.navigable_pub.publish(navigable_msg)
                
        except Exception as e:
            self.get_logger().error(f'Error in color callback: {e}')
    
    def color_based_segmentation(self, image):
        """
        Simple color-based segmentation to identify navigable floor area.
        This can be replaced with deep learning models for better accuracy.
        """
        # Convert to HSV for better color segmentation
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Define range for floor color (adjust these values for your environment)
        # Example: light gray to white floor
        lower_bound = np.array([0, 0, 100])
        upper_bound = np.array([180, 50, 255])
        
        # Create mask
        mask = cv2.inRange(hsv, lower_bound, upper_bound)
        
        # Apply morphological operations to clean up the mask
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        # Apply Gaussian blur to smooth edges
        mask = cv2.GaussianBlur(mask, (5, 5), 0)
        
        return mask
    
    def find_centroid(self, mask):
        """Find the centroid of the largest navigable region."""
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None, 0
        
        # Find largest contour
        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)
        
        if area < self.min_area:
            return None, area
        
        # Calculate centroid using moments
        M = cv2.moments(largest_contour)
        if M['m00'] == 0:
            return None, area
        
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        
        return (cx, cy), area
    
    def publish_segmentation_image(self, original, mask, centroid):
        """Publish visualization of segmentation result."""
        # Create colored overlay
        overlay = original.copy()
        overlay[mask > 0] = [0, 255, 0]  # Green for navigable area
        
        # Blend with original image
        vis_image = cv2.addWeighted(original, 0.7, overlay, 0.3, 0)
        
        # Draw centroid
        if centroid is not None:
            cv2.circle(vis_image, centroid, 10, (0, 0, 255), -1)
            cv2.circle(vis_image, centroid, 12, (255, 255, 255), 2)
            
            # Draw crosshair
            cv2.line(vis_image, (centroid[0] - 20, centroid[1]), 
                    (centroid[0] + 20, centroid[1]), (255, 255, 255), 2)
            cv2.line(vis_image, (centroid[0], centroid[1] - 20), 
                    (centroid[0], centroid[1] + 20), (255, 255, 255), 2)
            
            # Draw center line for reference
            center_x = self.image_width // 2
            cv2.line(vis_image, (center_x, 0), (center_x, self.image_height), 
                    (255, 0, 0), 2)
        
        # Convert back to ROS Image and publish
        try:
            ros_image = self.bridge.cv2_to_imgmsg(vis_image, encoding='bgr8')
            self.seg_image_pub.publish(ros_image)
        except Exception as e:
            self.get_logger().error(f'Failed to publish segmentation image: {e}')
    
    def publish_velocity_command(self, centroid):
        """
        Calculate and publish velocity command to navigate towards centroid.
        Uses proportional control based on centroid position.
        """
        if centroid is None:
            self.publish_stop_command()
            return
        
        # Calculate error from image center
        center_x = self.image_width / 2.0
        error_x = (centroid[0] - center_x) / center_x  # Normalized error [-1, 1]
        
        # Calculate angular velocity (negative because image x increases to the right)
        angular_vel = -self.angular_gain * error_x
        angular_vel = np.clip(angular_vel, -self.max_angular_vel, self.max_angular_vel)
        
        # Calculate linear velocity (reduce speed when turning)
        linear_vel = self.linear_gain * (1.0 - abs(error_x))
        linear_vel = np.clip(linear_vel, 0.0, self.max_linear_vel)
        
        # Get distance to target if depth is available
        if self.latest_depth is not None:
            try:
                cy = centroid[1]
                cx = centroid[0]
                if 0 <= cy < self.latest_depth.shape[0] and 0 <= cx < self.latest_depth.shape[1]:
                    depth_value = self.latest_depth[cy, cx]
                    # Convert depth to meters (depends on encoding)
                    distance = depth_value / 1000.0  # Assuming mm encoding
                    
                    # Adjust linear velocity based on distance
                    if distance > 0 and distance < 5.0:  # Valid depth reading
                        distance_error = distance - self.target_distance
                        linear_vel = self.linear_gain * distance_error
                        linear_vel = np.clip(linear_vel, 0.0, self.max_linear_vel)
            except Exception as e:
                self.get_logger().debug(f'Depth processing error: {e}')
        
        # Create and publish Twist message
        cmd = Twist()
        cmd.linear.x = linear_vel
        cmd.angular.z = angular_vel
        self.cmd_vel_pub.publish(cmd)
    
    def publish_stop_command(self):
        """Publish zero velocity command."""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = SegmentationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

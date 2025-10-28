#!/usr/bin/env python3
"""
Semantic Segmentation Node for Standard USB RGB Camera (1080p)
Performs real-time semantic segmentation to identify navigable areas.
Works with any v4l2 compatible USB camera.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np


class RGBSegmentationNode(Node):
    def __init__(self):
        super().__init__('rgb_segmentation_node')
        
        # Declare parameters
        self.declare_parameter('camera_device', 0)  # /dev/video0
        self.declare_parameter('camera_width', 1920)
        self.declare_parameter('camera_height', 1080)
        self.declare_parameter('camera_fps', 30)
        self.declare_parameter('segmentation_method', 'color_based')
        self.declare_parameter('min_area', 10000)  # Larger for 1080p
        self.declare_parameter('target_distance', 1.0)
        self.declare_parameter('linear_gain', 0.5)
        self.declare_parameter('angular_gain', 1.0)
        self.declare_parameter('max_linear_vel', 0.5)
        self.declare_parameter('max_angular_vel', 1.0)
        self.declare_parameter('processing_scale', 0.5)  # Scale down for faster processing
        
        # Get parameters
        self.camera_device = self.get_parameter('camera_device').value
        self.camera_width = self.get_parameter('camera_width').value
        self.camera_height = self.get_parameter('camera_height').value
        self.camera_fps = self.get_parameter('camera_fps').value
        self.seg_method = self.get_parameter('segmentation_method').value
        self.min_area = self.get_parameter('min_area').value
        self.target_distance = self.get_parameter('target_distance').value
        self.linear_gain = self.get_parameter('linear_gain').value
        self.angular_gain = self.get_parameter('angular_gain').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.processing_scale = self.get_parameter('processing_scale').value
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Initialize camera
        self.cap = cv2.VideoCapture(self.camera_device)
        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open camera device {self.camera_device}')
            raise RuntimeError('Camera initialization failed')
        
        # Set camera properties
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_height)
        self.cap.set(cv2.CAP_PROP_FPS, self.camera_fps)
        
        # Get actual camera properties
        actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = int(self.cap.get(cv2.CAP_PROP_FPS))
        
        self.get_logger().info(f'Camera initialized: {actual_width}x{actual_height} @ {actual_fps}fps')
        self.get_logger().info(f'Processing scale: {self.processing_scale}x')
        
        self.image_width = actual_width
        self.image_height = actual_height
        self.proc_width = int(actual_width * self.processing_scale)
        self.proc_height = int(actual_height * self.processing_scale)
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.raw_image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.seg_image_pub = self.create_publisher(Image, '/segmentation/image', 10)
        self.centroid_pub = self.create_publisher(Point, '/segmentation/centroid', 10)
        self.navigable_pub = self.create_publisher(Bool, '/segmentation/navigable', 10)
        
        # State variables
        self.centroid = None
        
        # Create timer for camera capture and processing
        timer_period = 1.0 / self.camera_fps
        self.timer = self.create_timer(timer_period, self.camera_callback)
        
        self.get_logger().info(f'RGB Segmentation node initialized with method: {self.seg_method}')
    
    def camera_callback(self):
        """Capture and process camera frame."""
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to read camera frame')
            return
        
        try:
            # Publish raw image
            raw_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.raw_image_pub.publish(raw_msg)
            
            # Scale down for processing
            if self.processing_scale != 1.0:
                proc_frame = cv2.resize(frame, (self.proc_width, self.proc_height))
            else:
                proc_frame = frame
            
            # Perform segmentation
            if self.seg_method == 'color_based':
                seg_mask = self.color_based_segmentation(proc_frame)
            elif self.seg_method == 'edge_based':
                seg_mask = self.edge_based_segmentation(proc_frame)
            elif self.seg_method == 'combined':
                seg_mask = self.combined_segmentation(proc_frame)
            else:
                self.get_logger().warn(f'Unknown segmentation method: {self.seg_method}')
                return
            
            # Find navigable area centroid
            centroid, area = self.find_centroid(seg_mask)
            
            # Scale centroid back to original resolution
            if centroid is not None and self.processing_scale != 1.0:
                centroid = (
                    int(centroid[0] / self.processing_scale),
                    int(centroid[1] / self.processing_scale)
                )
            
            # Publish segmentation visualization
            self.publish_segmentation_image(frame, seg_mask, centroid)
            
            # Publish centroid
            if centroid is not None:
                centroid_msg = Point()
                centroid_msg.x = float(centroid[0])
                centroid_msg.y = float(centroid[1])
                centroid_msg.z = 0.0
                self.centroid_pub.publish(centroid_msg)
                self.centroid = centroid
            
            # Check if area is navigable (scale area by processing scale squared)
            scaled_min_area = self.min_area * (self.processing_scale ** 2)
            is_navigable = area > scaled_min_area and centroid is not None
            navigable_msg = Bool()
            navigable_msg.data = is_navigable
            self.navigable_pub.publish(navigable_msg)
            
            # Calculate and publish velocity command
            if is_navigable:
                self.publish_velocity_command(centroid)
            else:
                self.publish_stop_command()
                
        except Exception as e:
            self.get_logger().error(f'Error in camera callback: {e}')
    
    def color_based_segmentation(self, image):
        """Color-based segmentation for floor detection."""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Define range for floor color (adjust for your environment)
        lower_bound = np.array([0, 0, 100])
        upper_bound = np.array([180, 50, 255])
        
        mask = cv2.inRange(hsv, lower_bound, upper_bound)
        
        # Morphological operations
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.GaussianBlur(mask, (5, 5), 0)
        
        return mask
    
    def edge_based_segmentation(self, image):
        """Edge-based segmentation for road/path detection."""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Apply Gaussian blur
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Canny edge detection
        edges = cv2.Canny(blurred, 50, 150)
        
        # Invert (we want regions, not edges)
        mask = cv2.bitwise_not(edges)
        
        # Fill small gaps
        kernel = np.ones((7, 7), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        return mask
    
    def combined_segmentation(self, image):
        """Combined color and edge-based segmentation."""
        color_mask = self.color_based_segmentation(image)
        edge_mask = self.edge_based_segmentation(image)
        
        # Combine masks
        combined = cv2.bitwise_and(color_mask, edge_mask)
        
        return combined
    
    def find_centroid(self, mask):
        """Find the centroid of the largest navigable region."""
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None, 0
        
        # Find largest contour
        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)
        
        # Scale area check by processing scale
        scaled_min_area = self.min_area * (self.processing_scale ** 2)
        if area < scaled_min_area:
            return None, area
        
        # Calculate centroid
        M = cv2.moments(largest_contour)
        if M['m00'] == 0:
            return None, area
        
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        
        return (cx, cy), area
    
    def publish_segmentation_image(self, original, mask, centroid):
        """Publish visualization of segmentation result."""
        # Resize mask to match original if needed
        if mask.shape[:2] != original.shape[:2]:
            mask = cv2.resize(mask, (original.shape[1], original.shape[0]))
        
        # Create colored overlay
        overlay = original.copy()
        overlay[mask > 128] = [0, 255, 0]  # Green for navigable area
        
        # Blend with original image
        vis_image = cv2.addWeighted(original, 0.7, overlay, 0.3, 0)
        
        # Draw centroid
        if centroid is not None:
            cv2.circle(vis_image, centroid, 15, (0, 0, 255), -1)
            cv2.circle(vis_image, centroid, 18, (255, 255, 255), 3)
            
            # Draw crosshair
            cv2.line(vis_image, (centroid[0] - 30, centroid[1]), 
                    (centroid[0] + 30, centroid[1]), (255, 255, 255), 3)
            cv2.line(vis_image, (centroid[0], centroid[1] - 30), 
                    (centroid[0], centroid[1] + 30), (255, 255, 255), 3)
            
            # Draw center line for reference
            center_x = self.image_width // 2
            cv2.line(vis_image, (center_x, 0), (center_x, self.image_height), 
                    (255, 0, 0), 3)
            
            # Add text info
            error = (centroid[0] - center_x) / center_x
            text = f"Error: {error:.2f}"
            cv2.putText(vis_image, text, (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 
                       1.0, (0, 255, 255), 2)
        
        # Convert back to ROS Image and publish
        try:
            ros_image = self.bridge.cv2_to_imgmsg(vis_image, encoding='bgr8')
            self.seg_image_pub.publish(ros_image)
        except Exception as e:
            self.get_logger().error(f'Failed to publish segmentation image: {e}')
    
    def publish_velocity_command(self, centroid):
        """Calculate and publish velocity command."""
        if centroid is None:
            self.publish_stop_command()
            return
        
        # Calculate error from image center
        center_x = self.image_width / 2.0
        error_x = (centroid[0] - center_x) / center_x  # Normalized [-1, 1]
        
        # Calculate angular velocity
        angular_vel = -self.angular_gain * error_x
        angular_vel = np.clip(angular_vel, -self.max_angular_vel, self.max_angular_vel)
        
        # Calculate linear velocity (reduce when turning sharply)
        linear_vel = self.linear_gain * (1.0 - abs(error_x) * 0.5)
        linear_vel = np.clip(linear_vel, 0.0, self.max_linear_vel)
        
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
    
    def destroy_node(self):
        """Cleanup camera on shutdown."""
        if hasattr(self, 'cap'):
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RGBSegmentationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

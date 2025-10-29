#!/usr/bin/env python3
"""
Interactive HSV color tuner for segmentation
Helps tune color thresholds for floor detection
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class ColorTuner(Node):
    def __init__(self):
        super().__init__('color_tuner')
        self.bridge = CvBridge()
        
        self.sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )
        
        # Create window with trackbars
        cv2.namedWindow('Color Tuner')
        cv2.createTrackbar('H Min', 'Color Tuner', 0, 179, lambda x: None)
        cv2.createTrackbar('H Max', 'Color Tuner', 179, 179, lambda x: None)
        cv2.createTrackbar('S Min', 'Color Tuner', 0, 255, lambda x: None)
        cv2.createTrackbar('S Max', 'Color Tuner', 255, 255, lambda x: None)
        cv2.createTrackbar('V Min', 'Color Tuner', 100, 255, lambda x: None)
        cv2.createTrackbar('V Max', 'Color Tuner', 255, 255, lambda x: None)
        
        self.get_logger().info('Color Tuner started. Adjust trackbars to tune thresholds.')
        self.get_logger().info('Press "q" in the window to quit.')
    
    def image_callback(self, msg):
        try:
            # Convert to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # Get trackbar values
            h_min = cv2.getTrackbarPos('H Min', 'Color Tuner')
            h_max = cv2.getTrackbarPos('H Max', 'Color Tuner')
            s_min = cv2.getTrackbarPos('S Min', 'Color Tuner')
            s_max = cv2.getTrackbarPos('S Max', 'Color Tuner')
            v_min = cv2.getTrackbarPos('V Min', 'Color Tuner')
            v_max = cv2.getTrackbarPos('V Max', 'Color Tuner')
            
            # Create mask
            lower = np.array([h_min, s_min, v_min])
            upper = np.array([h_max, s_max, v_max])
            mask = cv2.inRange(hsv, lower, upper)
            
            # Apply morphology
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            
            # Create result
            result = cv2.bitwise_and(cv_image, cv_image, mask=mask)
            
            # Stack images for display
            mask_3ch = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            combined = np.hstack([cv_image, mask_3ch, result])
            
            # Add text
            text = f"H:[{h_min},{h_max}] S:[{s_min},{s_max}] V:[{v_min},{v_max}]"
            cv2.putText(combined, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                       0.7, (0, 255, 0), 2)
            
            # Show
            cv2.imshow('Color Tuner', combined)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.get_logger().info(f'Final values: lower_bound = np.array([{h_min}, {s_min}, {v_min}])')
                self.get_logger().info(f'Final values: upper_bound = np.array([{h_max}, {s_max}, {v_max}])')
                rclpy.shutdown()
                
        except Exception as e:
            self.get_logger().error(f'Error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ColorTuner()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

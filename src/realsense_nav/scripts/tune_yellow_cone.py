#!/usr/bin/env python3
"""
Yellow Cone HSV Tuner
Interactive tool to tune HSV values for yellow cone detection
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class YellowConeTuner(Node):
    def __init__(self):
        super().__init__('yellow_cone_tuner')
        
        self.bridge = CvBridge()
        
        # Current HSV values
        self.h_min = 20
        self.h_max = 35
        self.s_min = 100
        self.s_max = 255
        self.v_min = 100
        self.v_max = 255
        self.min_area = 500
        self.max_area = 50000
        
        self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10
        )
        
        self.latest_image = None
        
        # Create window and trackbars
        cv2.namedWindow('Yellow Cone Tuner')
        cv2.createTrackbar('H Min', 'Yellow Cone Tuner', self.h_min, 180, self.on_h_min_change)
        cv2.createTrackbar('H Max', 'Yellow Cone Tuner', self.h_max, 180, self.on_h_max_change)
        cv2.createTrackbar('S Min', 'Yellow Cone Tuner', self.s_min, 255, self.on_s_min_change)
        cv2.createTrackbar('S Max', 'Yellow Cone Tuner', self.s_max, 255, self.on_s_max_change)
        cv2.createTrackbar('V Min', 'Yellow Cone Tuner', self.v_min, 255, self.on_v_min_change)
        cv2.createTrackbar('V Max', 'Yellow Cone Tuner', self.v_max, 255, self.on_v_max_change)
        cv2.createTrackbar('Min Area/10', 'Yellow Cone Tuner', self.min_area//10, 1000, self.on_min_area_change)
        cv2.createTrackbar('Max Area/100', 'Yellow Cone Tuner', self.max_area//100, 1000, self.on_max_area_change)
        
        self.get_logger().info('Yellow Cone Tuner started')
        self.get_logger().info('Adjust sliders to tune detection')
        self.get_logger().info('Press "q" to quit, "p" to print current values')
        
        # Timer to process images
        self.create_timer(0.03, self.process_image)
    
    def on_h_min_change(self, val):
        self.h_min = val
    
    def on_h_max_change(self, val):
        self.h_max = val
    
    def on_s_min_change(self, val):
        self.s_min = val
    
    def on_s_max_change(self, val):
        self.s_max = val
    
    def on_v_min_change(self, val):
        self.v_min = val
    
    def on_v_max_change(self, val):
        self.v_max = val
    
    def on_min_area_change(self, val):
        self.min_area = val * 10
    
    def on_max_area_change(self, val):
        self.max_area = val * 100
    
    def image_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')
    
    def process_image(self):
        if self.latest_image is None:
            return
        
        image = self.latest_image.copy()
        
        # Convert to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Create mask
        lower = np.array([self.h_min, self.s_min, self.v_min])
        upper = np.array([self.h_max, self.s_max, self.v_max])
        mask = cv2.inRange(hsv, lower, upper)
        
        # Morphological operations
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Draw all contours that meet area criteria
        result = image.copy()
        cone_count = 0
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if self.min_area < area < self.max_area:
                # Draw contour
                cv2.drawContours(result, [contour], -1, (0, 255, 255), 2)
                
                # Calculate and draw centroid
                M = cv2.moments(contour)
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    cv2.circle(result, (cx, cy), 5, (0, 0, 255), -1)
                    
                    # Draw bounding box
                    x, y, w, h = cv2.boundingRect(contour)
                    cv2.rectangle(result, (x, y), (x+w, y+h), (0, 255, 0), 2)
                    
                    # Display area
                    cv2.putText(result, f'Area: {int(area)}', (x, y-10),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    
                    cone_count += 1
        
        # Display info
        info_text = [
            f'H: [{self.h_min}, {self.h_max}]',
            f'S: [{self.s_min}, {self.s_max}]',
            f'V: [{self.v_min}, {self.v_max}]',
            f'Area: [{self.min_area}, {self.max_area}]',
            f'Cones detected: {cone_count}'
        ]
        
        y_offset = 30
        for text in info_text:
            cv2.putText(result, text, (10, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            y_offset += 30
        
        # Show mask and result side by side
        mask_colored = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        combined = np.hstack([result, mask_colored])
        
        cv2.imshow('Yellow Cone Tuner', combined)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.print_values()
            rclpy.shutdown()
        elif key == ord('p'):
            self.print_values()
    
    def print_values(self):
        self.get_logger().info('\n' + '='*50)
        self.get_logger().info('Current HSV Values:')
        self.get_logger().info(f"  'yellow_h_min': {self.h_min},")
        self.get_logger().info(f"  'yellow_h_max': {self.h_max},")
        self.get_logger().info(f"  'yellow_s_min': {self.s_min},")
        self.get_logger().info(f"  'yellow_s_max': {self.s_max},")
        self.get_logger().info(f"  'yellow_v_min': {self.v_min},")
        self.get_logger().info(f"  'yellow_v_max': {self.v_max},")
        self.get_logger().info(f"  'min_cone_area': {self.min_area},")
        self.get_logger().info(f"  'max_cone_area': {self.max_area},")
        self.get_logger().info('='*50)


def main():
    rclpy.init()
    node = YellowConeTuner()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()


if __name__ == '__main__':
    main()

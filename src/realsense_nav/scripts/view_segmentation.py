#!/usr/bin/env python3
"""
Simple OpenCV-based viewer for segmentation output
Shows pure pursuit trajectory visualization
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import json


class SegmentationViewer(Node):
    def __init__(self):
        super().__init__('segmentation_viewer')
        
        self.bridge = CvBridge()
        
        # Subscribe to multiple image topics
        self.create_subscription(
            Image,
            '/segmentation/image',
            self.seg_image_cb,
            10
        )
        
        self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.raw_image_cb,
            10
        )
        
        self.create_subscription(
            Image,
            '/goal/image',
            self.goal_image_cb,
            10
        )

        # Subscribe to scene graph (contains YOLO labels and bboxes)
        self.create_subscription(
            String,
            '/scene_graph',
            self.scene_graph_cb,
            10
        )
        
        self.create_subscription(
            Float32MultiArray,
            '/pure_pursuit/trajectory',
            self.trajectory_cb,
            10
        )
        
        self.create_subscription(
            PointStamped,
            '/goal/position',
            self.goal_position_cb,
            10
        )
        
        self.create_subscription(
            Point,
            '/segmentation/centroid',
            self.centroid_cb,
            10
        )
        
        self.create_subscription(
            Bool,
            '/goal/detected',
            self.goal_detected_cb,
            10
        )
        
        self.seg_image = None
        self.raw_image = None
        self.goal_image = None
        self.trajectory_data = None  # [goal_x, goal_y, path_ang, goal_ang, combined_ang]
        self.goal_position = None
        self.centroid = None
        self.goal_detected = False
        self.scene_graph = None
        
        self.get_logger().info('Segmentation Viewer started')
        self.get_logger().info('Press "q" to quit, "s" to save screenshot')
        
        # Create window
        cv2.namedWindow('RealSense Navigation', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('RealSense Navigation', 1920, 480)
    
    def trajectory_cb(self, msg):
        if len(msg.data) >= 5:
            self.trajectory_data = msg.data
    
    def goal_position_cb(self, msg):
        self.goal_position = msg.point
    
    def centroid_cb(self, msg):
        self.centroid = (msg.x, msg.y)
    
    def goal_detected_cb(self, msg):
        self.goal_detected = msg.data
    
    def seg_image_cb(self, msg):
        try:
            self.seg_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.display()
        except Exception as e:
            self.get_logger().error(f'Error converting segmentation image: {e}')
    
    def raw_image_cb(self, msg):
        try:
            self.raw_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error converting raw image: {e}')
    
    def goal_image_cb(self, msg):
        try:
            self.goal_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error converting goal image: {e}')

    def scene_graph_cb(self, msg):
        try:
            self.scene_graph = json.loads(msg.data)
        except Exception:
            self.scene_graph = None
    
    def display(self):
        if self.seg_image is None:
            return
        
        images_to_show = []
        
        # Raw camera image with trajectory overlay
        if self.raw_image is not None:
            trajectory_vis = self.draw_trajectory_overlay(self.raw_image.copy())
            images_to_show.append(trajectory_vis)
        
        # Segmentation image
        images_to_show.append(self.seg_image)
        
        # Goal detection image
        if self.goal_image is not None:
            images_to_show.append(self.goal_image)

        # YOLO / scene_graph overlay (drawn on raw image)
        if self.raw_image is not None and self.scene_graph is not None:
            yolo_vis = self.draw_yolo_overlay(self.raw_image.copy())
            images_to_show.append(yolo_vis)
        
        # Resize all to same height
        if len(images_to_show) > 1:
            target_h = images_to_show[0].shape[0]
            resized_images = []
            for img in images_to_show:
                if img.shape[0] != target_h:
                    scale = target_h / img.shape[0]
                    new_w = int(img.shape[1] * scale)
                    img = cv2.resize(img, (new_w, target_h))
                resized_images.append(img)
            
            # Stack horizontally
            display = np.hstack(resized_images)
            
            # Add labels at bottom for each panel
            label_y = display.shape[0] - 10
            panel_labels = ['TRAJECTORY', 'SEGMENTATION', 'GOAL DETECTION', 'YOLO']
            offset = 0
            for i, img in enumerate(resized_images):
                lab = panel_labels[i] if i < len(panel_labels) else f'PANEL {i}'
                cv2.putText(display, lab, (offset + 10, label_y),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                offset += img.shape[1]
        else:
            display = images_to_show[0]
        
        # Show
        cv2.imshow('RealSense Navigation', display)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.get_logger().info('Quitting...')
            rclpy.shutdown()
        elif key == ord('s'):
            filename = f'navigation_screenshot_{cv2.getTickCount()}.png'
            cv2.imwrite(filename, display)
            self.get_logger().info(f'Saved screenshot: {filename}')
    
    def draw_trajectory_overlay(self, image):
        """Draw pure pursuit trajectory on image"""
        h, w = image.shape[:2]
        
        # Draw title
        cv2.putText(image, 'PURE PURSUIT TRAJECTORY', (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        
        # Draw path centroid if available
        if self.centroid is not None:
            cx, cy = int(self.centroid[0]), int(self.centroid[1])
            cv2.circle(image, (cx, cy), 8, (0, 255, 0), -1)
            cv2.putText(image, 'PATH', (cx + 15, cy - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # Draw goal position if detected
        if self.goal_detected and self.goal_position is not None:
            # Project 3D goal to image plane
            # Simple projection: use lateral offset (y) to estimate pixel position
            # Assuming camera FOV ~69 degrees horizontal
            goal_x = self.goal_position.x  # Forward distance
            goal_y = self.goal_position.y  # Lateral offset (negative is right in camera frame)
            
            if goal_x > 0.1:  # If goal is in front
                # Simple projection: lateral offset / distance * width/2 + center
                fov_scale = w / 2.0  # Approximate scale
                pixel_x = int(w/2 - goal_y * fov_scale / goal_x)
                pixel_y = int(h * 0.7)  # Place at lower part of image
                
                pixel_x = max(0, min(w-1, pixel_x))
                
                # Draw goal marker
                cv2.circle(image, (pixel_x, pixel_y), 12, (0, 255, 255), 3)
                cv2.circle(image, (pixel_x, pixel_y), 4, (0, 255, 255), -1)
                cv2.putText(image, f'GOAL ({goal_x:.1f}m)', (pixel_x + 20, pixel_y),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                
                # Draw trajectory arc from bottom center to goal
                center_x = w // 2
                center_y = h
                
                # Draw curved trajectory
                num_points = 20
                trajectory_points = []
                for i in range(num_points + 1):
                    t = i / num_points
                    # Quadratic bezier curve
                    px = int((1-t)**2 * center_x + 2*(1-t)*t*center_x + t**2 * pixel_x)
                    py = int((1-t)**2 * center_y + 2*(1-t)*t*(center_y - h*0.3) + t**2 * pixel_y)
                    trajectory_points.append((px, py))
                
                # Draw trajectory line
                for i in range(len(trajectory_points) - 1):
                    cv2.line(image, trajectory_points[i], trajectory_points[i+1],
                            (255, 0, 255), 3)
                
                # Draw robot position at bottom center
                cv2.circle(image, (center_x, center_y - 20), 15, (255, 255, 0), -1)
                cv2.circle(image, (center_x, center_y - 20), 18, (255, 255, 255), 2)
        
        # Draw trajectory data if available
        if self.trajectory_data is not None and len(self.trajectory_data) >= 5:
            goal_x, goal_y, path_ang, goal_ang, combined_ang = self.trajectory_data
            
            y_offset = 60
            cv2.putText(image, f'Goal: ({goal_x:.2f}, {goal_y:.2f})m', (10, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            y_offset += 25
            cv2.putText(image, f'Path Ang: {path_ang:.2f} rad/s', (10, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            y_offset += 25
            cv2.putText(image, f'Goal Ang: {goal_ang:.2f} rad/s', (10, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            y_offset += 25
            cv2.putText(image, f'Combined: {combined_ang:.2f} rad/s', (10, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)
        
        return image

    def draw_yolo_overlay(self, image):
        """Draw YOLO boxes and labels on the provided image using the last scene_graph."""
        if self.scene_graph is None:
            return image

        objs = self.scene_graph.get('objects', [])
        for o in objs:
            bbox = o.get('bbox', None)
            label = o.get('label', None)
            conf = o.get('conf', None)
            centroid = o.get('centroid_px', None)

            if bbox:
                try:
                    x, y, w, h = [int(v) for v in bbox]
                except Exception:
                    continue
                # draw bbox
                cv2.rectangle(image, (x, y), (x + w, y + h), (0, 165, 255), 2)
                # prepare label text
                lab = ''
                if label:
                    lab += str(label)
                if conf is not None:
                    try:
                        lab += f' {float(conf):.2f}'
                    except Exception:
                        pass
                if lab:
                    # draw text background
                    (tw, th), _ = cv2.getTextSize(lab, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
                    cv2.rectangle(image, (x, y - th - 6), (x + tw + 4, y), (0, 165, 255), -1)
                    cv2.putText(image, lab, (x + 2, y - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)

            if centroid:
                try:
                    cx, cy = int(centroid[0]), int(centroid[1])
                    cv2.circle(image, (cx, cy), 4, (0, 0, 255), -1)
                except Exception:
                    pass

        return image


def main():
    rclpy.init()
    node = SegmentationViewer()
    
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

#!/usr/bin/env python3
"""
Junction Recognizer

Visual recognition of junctions using feature matching.
Compares current camera view with expected junction images.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from realsense_nav.msg import JunctionDetection
from cv_bridge import CvBridge
import cv2
import json
import os
import numpy as np


class JunctionRecognizerNode(Node):
    def __init__(self):
        super().__init__('junction_recognizer')
        
        self.declare_parameter('data_dir', '/home/raushan/control_one/wheel_control/data/junctions')
        self.declare_parameter('feature_type', 'ORB')
        self.declare_parameter('match_threshold', 0.7)
        self.declare_parameter('min_matches', 15)
        
        self.data_dir = self.get_parameter('data_dir').value
        self.feature_type = self.get_parameter('feature_type').value
        self.match_threshold = self.get_parameter('match_threshold').value
        self.min_matches = self.get_parameter('min_matches').value
        
        self.bridge = CvBridge()
        self.latest_image = None
        self.expected_junction_id = None
        self.junction_images = {}
        
        # Load junction images
        self._load_junction_images()
        
        # Create feature detector
        if self.feature_type == 'ORB':
            self.detector = cv2.ORB_create(nfeatures=1000)
        elif self.feature_type == 'SIFT':
            self.detector = cv2.SIFT_create()
        else:
            self.detector = cv2.ORB_create(nfeatures=1000)
        
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING if self.feature_type == 'ORB' else cv2.NORM_L2, crossCheck=True)
        
        self.create_subscription(Image, '/camera/camera/color/image_raw', self.image_callback, 10)
        self.create_subscription(String, '/navigation/expected_junction', self.expected_callback, 10)
        
        self.detection_pub = self.create_publisher(JunctionDetection, '/junction/detected', 10)
        
        self.create_timer(0.5, self.recognize_junction)
        
        self.get_logger().info(f'Junction Recognizer initialized with {self.feature_type}')
    
    def _load_junction_images(self):
        """Load all junction images and extract features"""
        junction_file = os.path.join(self.data_dir, 'junctions.json')
        if not os.path.exists(junction_file):
            return
        
        with open(junction_file, 'r') as f:
            junctions = json.load(f)
        
        for junction in junctions:
            img_path = junction['image_path']
            if os.path.exists(img_path):
                img = cv2.imread(img_path)
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                kp, desc = self.detector.detectAndCompute(gray, None)
                
                self.junction_images[junction['id']] = {
                    'image': img,
                    'keypoints': kp,
                    'descriptors': desc
                }
        
        self.get_logger().info(f'Loaded {len(self.junction_images)} junction images')
    
    def image_callback(self, msg):
        """Store latest image"""
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'Image conversion failed: {e}')
    
    def expected_callback(self, msg):
        """Set expected junction"""
        self.expected_junction_id = msg.data
        self.get_logger().info(f'Expecting junction: {self.expected_junction_id}')
    
    def recognize_junction(self):
        """Recognize junction from current image"""
        if self.latest_image is None or not self.junction_images:
            return
        
        # Extract features from current image
        gray = cv2.cvtColor(self.latest_image, cv2.COLOR_BGR2GRAY)
        kp_cur, desc_cur = self.detector.detectAndCompute(gray, None)
        
        if desc_cur is None or len(kp_cur) < self.min_matches:
            return
        
        # Match against all junctions (prioritize expected)
        best_match = None
        best_score = 0.0
        best_matches_count = 0
        
        junctions_to_check = [self.expected_junction_id] if self.expected_junction_id else []
        junctions_to_check += [jid for jid in self.junction_images.keys() if jid != self.expected_junction_id]
        
        for jid in junctions_to_check:
            if jid not in self.junction_images:
                continue
            
            junction_data = self.junction_images[jid]
            desc_junc = junction_data['descriptors']
            
            if desc_junc is None:
                continue
            
            try:
                matches = self.matcher.match(desc_cur, desc_junc)
                if len(matches) < self.min_matches:
                    continue
                
                # Sort by distance
                matches = sorted(matches, key=lambda x: x.distance)
                
                # Compute confidence (ratio of good matches)
                good_matches = [m for m in matches if m.distance < 50]
                confidence = len(good_matches) / max(len(kp_cur), len(junction_data['keypoints']))
                
                if confidence > best_score:
                    best_score = confidence
                    best_match = jid
                    best_matches_count = len(good_matches)
                    
            except Exception as e:
                self.get_logger().debug(f'Matching failed for {jid}: {e}')
        
        # Publish detection if confidence is high enough
        if best_match and best_score >= self.match_threshold:
            msg = JunctionDetection()
            msg.junction_id = best_match
            msg.confidence = float(best_score)
            msg.num_matches = best_matches_count
            msg.timestamp = self.get_clock().now().to_msg().sec
            msg.is_expected = (best_match == self.expected_junction_id)
            
            self.detection_pub.publish(msg)
            
            if msg.is_expected:
                self.get_logger().info(f'✓ Detected expected junction {best_match} (conf: {best_score:.2f})')


def main(args=None):
    rclpy.init(args=args)
    node = JunctionRecognizerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

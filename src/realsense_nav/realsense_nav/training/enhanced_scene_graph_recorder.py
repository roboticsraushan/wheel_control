#!/usr/bin/env python3
"""
Enhanced Scene Graph Recorder with Node Image Storage

Records node images during junction recording for better destination verification.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import json
import os
import cv2
from datetime import datetime


class EnhancedSceneGraphRecorderNode(Node):
    def __init__(self):
        super().__init__('enhanced_scene_graph_recorder')
        
        self.declare_parameter('junction_dir', '/home/raushan/control_one/wheel_control/data/junctions')
        self.declare_parameter('node_dir', '/home/raushan/control_one/wheel_control/data/nodes')
        
        self.junction_dir = self.get_parameter('junction_dir').value
        self.node_dir = self.get_parameter('node_dir').value
        
        os.makedirs(self.node_dir, exist_ok=True)
        
        self.bridge = CvBridge()
        self.latest_scene_graph = None
        self.latest_image = None
        self.recorded_associations = []
        
        self.create_subscription(String, '/scene_graph', self.scene_graph_callback, 10)
        self.create_subscription(Image, '/camera/camera/color/image_raw', self.image_callback, 10)
        self.create_subscription(String, '/junction/recorded', self.junction_recorded_callback, 10)
        
        self.get_logger().info('Enhanced Scene Graph Recorder initialized')
    
    def scene_graph_callback(self, msg):
        """Store latest scene graph"""
        try:
            self.latest_scene_graph = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f'Failed to parse scene graph: {e}')
    
    def image_callback(self, msg):
        """Store latest image"""
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().debug(f'Image conversion failed: {e}')
    
    def junction_recorded_callback(self, msg):
        """Associate scene graph and extract node images"""
        try:
            event_data = json.loads(msg.data)
            junction_id = event_data.get('junction_id')
            
            if not junction_id:
                return
            
            if self.latest_scene_graph is None:
                self.get_logger().warn(f'No scene graph for {junction_id}')
                return
            
            objects = self.latest_scene_graph.get('objects', [])
            
            # Save cropped images for each detected object (node)
            saved_nodes = []
            if self.latest_image is not None:
                for i, obj in enumerate(objects):
                    node_image_path = self._save_node_image(junction_id, obj, i)
                    if node_image_path:
                        obj['image_path'] = node_image_path
                        saved_nodes.append(obj['label'])
            
            # Create association record
            association = {
                'junction_id': junction_id,
                'timestamp': datetime.now().isoformat(),
                'num_objects': len(objects),
                'objects': objects,
                'saved_node_images': saved_nodes
            }
            
            self.recorded_associations.append(association)
            
            # Save association
            self._save_association(junction_id, association)
            
            self.get_logger().info(
                f'✓ {len(objects)} objects, {len(saved_nodes)} node images saved for {junction_id}'
            )
            
        except Exception as e:
            self.get_logger().error(f'Failed to process junction: {e}')
    
    def _save_node_image(self, junction_id, obj, index):
        """Save cropped image of detected node"""
        if self.latest_image is None:
            return None
        
        bbox = obj.get('bbox')
        if not bbox or len(bbox) < 4:
            return None
        
        try:
            x, y, w, h = bbox
            
            # Add padding
            padding = 20
            x1 = max(0, x - padding)
            y1 = max(0, y - padding)
            x2 = min(self.latest_image.shape[1], x + w + padding)
            y2 = min(self.latest_image.shape[0], y + h + padding)
            
            # Crop
            cropped = self.latest_image[y1:y2, x1:x2]
            
            if cropped.size == 0:
                return None
            
            # Save
            label = obj.get('label', 'unknown')
            filename = f"{junction_id}_{label}_{index}.jpg"
            filepath = os.path.join(self.node_dir, filename)
            
            cv2.imwrite(filepath, cropped)
            
            return filepath
            
        except Exception as e:
            self.get_logger().debug(f'Failed to save node image: {e}')
            return None
    
    def _save_association(self, junction_id, association):
        """Save scene graph association to file"""
        filename = f'{junction_id}_scene_graph.json'
        filepath = os.path.join(self.junction_dir, filename)
        
        try:
            with open(filepath, 'w') as f:
                json.dump(association, f, indent=2)
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to save association: {e}')
            return False


def main(args=None):
    rclpy.init(args=args)
    node = EnhancedSceneGraphRecorderNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
Junction Manager Node

Records junctions during training phase. Each junction stores:
- Unique ID
- RGB image
- Pose (x, y, theta)
- Timestamp
- Associated scene graph

Listens for voice commands or service calls to record junctions.
Validates line-of-sight to previous junction.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from realsense_nav.srv import RecordJunction, SaveMap
from cv_bridge import CvBridge
import cv2
import json
import os
from datetime import datetime
import numpy as np


class JunctionManagerNode(Node):
    def __init__(self):
        super().__init__('junction_manager')
        
        # Parameters
        self.declare_parameter('data_dir', '/home/raushan/control_one/wheel_control/data/junctions')
        self.declare_parameter('image_format', 'jpg')
        self.declare_parameter('line_of_sight_distance', 5.0)  # meters
        self.declare_parameter('auto_save', True)
        
        self.data_dir = self.get_parameter('data_dir').value
        self.image_format = self.get_parameter('image_format').value
        self.los_distance = self.get_parameter('line_of_sight_distance').value
        self.auto_save = self.get_parameter('auto_save').value
        
        # Ensure data directory exists
        os.makedirs(self.data_dir, exist_ok=True)
        
        # Internal state
        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_pose = None
        self.latest_scene_graph = None
        self.junctions = []
        self.junction_counter = 0
        
        # Load existing junctions if any
        self._load_junctions()
        
        # Subscribers
        self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10
        )
        
        self.create_subscription(
            PoseStamped,
            '/robot/pose',
            self.pose_callback,
            10
        )
        
        self.create_subscription(
            String,
            '/scene_graph',
            self.scene_graph_callback,
            10
        )
        
        self.create_subscription(
            String,
            '/voice/command',
            self.voice_command_callback,
            10
        )
        
        # Publishers
        self.status_pub = self.create_publisher(String, '/junction/status', 10)
        self.recorded_pub = self.create_publisher(String, '/junction/recorded', 10)
        
        # Services
        self.record_srv = self.create_service(
            RecordJunction,
            '~/record_junction',
            self.record_junction_service
        )
        
        self.save_srv = self.create_service(
            SaveMap,
            '~/save_map',
            self.save_map_service
        )
        
        self.get_logger().info(f'Junction Manager initialized. Data dir: {self.data_dir}')
        self.get_logger().info(f'Loaded {len(self.junctions)} existing junctions')
        self._publish_status('ready', f'{len(self.junctions)} junctions loaded')
    
    def image_callback(self, msg):
        """Store latest camera image"""
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'Failed to convert image: {e}')
    
    def pose_callback(self, msg):
        """Store latest robot pose"""
        self.latest_pose = msg
    
    def scene_graph_callback(self, msg):
        """Store latest scene graph"""
        try:
            self.latest_scene_graph = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f'Failed to parse scene graph: {e}')
    
    def voice_command_callback(self, msg):
        """Listen for voice commands to record junctions"""
        command = msg.data.lower()
        
        # Trigger junction recording
        if any(phrase in command for phrase in ['record junction', 'save junction', 'mark junction']):
            self.get_logger().info('Voice command detected: Recording junction')
            success, junction_id, message = self._record_junction()
            
            if success:
                self.get_logger().info(f'✓ {message}')
            else:
                self.get_logger().error(f'✗ {message}')
        
        # Trigger map save
        elif any(phrase in command for phrase in ['save map', 'save the map']):
            self.get_logger().info('Voice command detected: Saving map')
            success, filepath, message = self._save_map()
            
            if success:
                self.get_logger().info(f'✓ Map saved: {filepath}')
            else:
                self.get_logger().error(f'✗ {message}')
    
    def record_junction_service(self, request, response):
        """Service to record a junction"""
        success, junction_id, message = self._record_junction(request.junction_name)
        
        response.success = success
        response.junction_id = junction_id
        response.message = message
        
        return response
    
    def save_map_service(self, request, response):
        """Service to save the map"""
        filename = request.filename if request.filename else None
        success, filepath, message = self._save_map(filename)
        
        response.success = success
        response.filepath = filepath
        response.num_junctions = len(self.junctions)
        response.num_nodes = 0  # Will be populated by scene graph recorder
        response.message = message
        
        return response
    
    def _record_junction(self, name=''):
        """
        Internal method to record a junction
        Returns: (success, junction_id, message)
        """
        # Validate we have required data
        if self.latest_image is None:
            return False, '', 'No camera image available'
        
        if self.latest_pose is None:
            self.get_logger().warn('No pose available, using default (0, 0, 0)')
            # Create a default pose
            self.latest_pose = PoseStamped()
        
        # Generate junction ID
        self.junction_counter += 1
        junction_id = f"J{self.junction_counter:03d}"
        
        # Check line of sight to previous junction
        los_valid = True
        if len(self.junctions) > 0:
            los_valid = self._check_line_of_sight(self.latest_pose)
            if not los_valid:
                self.get_logger().warn(
                    f'Warning: Junction may not have line-of-sight to previous junction '
                    f'(distance > {self.los_distance}m)'
                )
        
        # Save image
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        image_filename = f"{junction_id}_{timestamp}.{self.image_format}"
        image_path = os.path.join(self.data_dir, image_filename)
        
        try:
            cv2.imwrite(image_path, self.latest_image)
        except Exception as e:
            return False, '', f'Failed to save image: {e}'
        
        # Create junction data
        junction_data = {
            'id': junction_id,
            'name': name if name else junction_id,
            'pose': {
                'x': float(self.latest_pose.pose.position.x),
                'y': float(self.latest_pose.pose.position.y),
                'z': float(self.latest_pose.pose.position.z),
                'orientation': {
                    'x': float(self.latest_pose.pose.orientation.x),
                    'y': float(self.latest_pose.pose.orientation.y),
                    'z': float(self.latest_pose.pose.orientation.z),
                    'w': float(self.latest_pose.pose.orientation.w),
                }
            },
            'image_path': image_path,
            'timestamp': timestamp,
            'scene_graph': self.latest_scene_graph,
            'visible_junctions': [],  # Will be computed later
            'associated_nodes': []     # Will be populated by scene graph recorder
        }
        
        self.junctions.append(junction_data)
        
        # Publish junction recorded event
        event_msg = String()
        event_msg.data = json.dumps({'junction_id': junction_id, 'timestamp': timestamp})
        self.recorded_pub.publish(event_msg)
        
        # Auto-save if enabled
        if self.auto_save:
            self._save_junctions()
        
        message = f'Junction {junction_id} recorded at ({junction_data["pose"]["x"]:.2f}, {junction_data["pose"]["y"]:.2f})'
        self._publish_status('junction_recorded', message, junction_id)
        
        return True, junction_id, message
    
    def _check_line_of_sight(self, current_pose):
        """
        Check if current pose is within line-of-sight distance of last junction
        Returns: True if within range, False otherwise
        """
        if len(self.junctions) == 0:
            return True
        
        last_junction = self.junctions[-1]
        last_pose = last_junction['pose']
        
        # Calculate Euclidean distance
        dx = current_pose.pose.position.x - last_pose['x']
        dy = current_pose.pose.position.y - last_pose['y']
        distance = np.sqrt(dx**2 + dy**2)
        
        return distance <= self.los_distance
    
    def _save_junctions(self):
        """Save junctions to JSON file"""
        junction_file = os.path.join(self.data_dir, 'junctions.json')
        try:
            with open(junction_file, 'w') as f:
                json.dump(self.junctions, f, indent=2)
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to save junctions: {e}')
            return False
    
    def _load_junctions(self):
        """Load existing junctions from JSON file"""
        junction_file = os.path.join(self.data_dir, 'junctions.json')
        if os.path.exists(junction_file):
            try:
                with open(junction_file, 'r') as f:
                    self.junctions = json.load(f)
                
                # Update counter
                if self.junctions:
                    last_id = self.junctions[-1]['id']
                    self.junction_counter = int(last_id[1:])  # Extract number from "J001"
                
                return True
            except Exception as e:
                self.get_logger().error(f'Failed to load junctions: {e}')
                return False
        return False
    
    def _save_map(self, filename=None):
        """
        Save complete map data
        Returns: (success, filepath, message)
        """
        if not filename:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = f'map_{timestamp}.json'
        
        map_dir = '/home/raushan/control_one/wheel_control/data/maps'
        os.makedirs(map_dir, exist_ok=True)
        filepath = os.path.join(map_dir, filename)
        
        # Build edges between consecutive junctions
        edges = []
        for i in range(len(self.junctions) - 1):
            j1 = self.junctions[i]
            j2 = self.junctions[i + 1]
            
            # Calculate distance
            dx = j2['pose']['x'] - j1['pose']['x']
            dy = j2['pose']['y'] - j1['pose']['y']
            distance = float(np.sqrt(dx**2 + dy**2))
            
            edges.append({
                'from': j1['id'],
                'to': j2['id'],
                'distance': distance
            })
        
        map_data = {
            'metadata': {
                'created': datetime.now().isoformat(),
                'num_junctions': len(self.junctions),
                'num_edges': len(edges)
            },
            'junctions': self.junctions,
            'edges': edges,
            'nodes': []  # Will be populated by other nodes
        }
        
        try:
            with open(filepath, 'w') as f:
                json.dump(map_data, f, indent=2)
            
            message = f'Map saved: {len(self.junctions)} junctions, {len(edges)} edges'
            self._publish_status('map_saved', message)
            return True, filepath, message
        except Exception as e:
            return False, '', f'Failed to save map: {e}'
    
    def _publish_status(self, event_type, message, junction_id=''):
        """Publish status message"""
        status_data = {
            'event': event_type,
            'message': message,
            'junction_id': junction_id,
            'total_junctions': len(self.junctions),
            'timestamp': datetime.now().isoformat()
        }
        
        msg = String()
        msg.data = json.dumps(status_data)
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JunctionManagerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
Localization Manager

Tracks robot position in junction graph.
Persists last known position and handles recovery.
"""
import rclpy
from rclpy.node import Node
from realsense_nav.msg import JunctionDetection
from realsense_nav.srv import GetLastPosition
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import json
import os


class LocalizationManagerNode(Node):
    def __init__(self):
        super().__init__('localization_manager')
        
        self.declare_parameter('state_file', '/home/raushan/control_one/wheel_control/data/maps/localization_state.json')
        
        self.state_file = self.get_parameter('state_file').value
        self.current_junction = None
        self.current_pose = None
        self.last_update_time = 0.0
        
        # Load saved state
        self._load_state()
        
        self.create_subscription(JunctionDetection, '/junction/detected', self.detection_callback, 10)
        
        self.pub = self.create_publisher(String, '/robot/current_junction', 10)
        
        self.srv = self.create_service(GetLastPosition, '~/get_last_position', self.get_last_position_service)
        
        # Publish current position periodically
        self.create_timer(1.0, self.publish_position)
        
        self.get_logger().info(f'Localization Manager initialized. Current junction: {self.current_junction}')
    
    def detection_callback(self, msg):
        """Update position based on junction detection"""
        if msg.confidence >= 0.8:
            self.current_junction = msg.junction_id
            self.last_update_time = msg.timestamp
            
            self._save_state()
            
            self.get_logger().info(f'Position updated: {self.current_junction}')
    
    def publish_position(self):
        """Publish current position"""
        if self.current_junction:
            msg = String()
            msg.data = self.current_junction
            self.pub.publish(msg)
    
    def get_last_position_service(self, request, response):
        """Service to get last known position"""
        response.success = self.current_junction is not None
        response.junction_id = self.current_junction if self.current_junction else ''
        response.timestamp = self.last_update_time
        response.message = f'Last known junction: {self.current_junction}' if self.current_junction else 'No position known'
        
        if self.current_pose:
            response.pose = self.current_pose
        
        return response
    
    def _save_state(self):
        """Save localization state to file"""
        state = {
            'current_junction': self.current_junction,
            'timestamp': self.last_update_time
        }
        
        try:
            os.makedirs(os.path.dirname(self.state_file), exist_ok=True)
            with open(self.state_file, 'w') as f:
                json.dump(state, f, indent=2)
        except Exception as e:
            self.get_logger().error(f'Failed to save state: {e}')
    
    def _load_state(self):
        """Load localization state from file"""
        if os.path.exists(self.state_file):
            try:
                with open(self.state_file, 'r') as f:
                    state = json.load(f)
                
                self.current_junction = state.get('current_junction')
                self.last_update_time = state.get('timestamp', 0.0)
                
            except Exception as e:
                self.get_logger().error(f'Failed to load state: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationManagerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

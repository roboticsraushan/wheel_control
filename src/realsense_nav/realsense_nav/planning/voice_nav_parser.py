#!/usr/bin/env python3
"""
Voice Navigation Parser

Parses voice commands for navigation tasks.
Extracts source and destination nodes from natural language.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import re


class VoiceNavParserNode(Node):
    def __init__(self):
        super().__init__('voice_nav_parser')
        
        self.create_subscription(String, '/voice/command', self.voice_callback, 10)
        self.task_pub = self.create_publisher(String, '/navigation/task', 10)
        
        self.get_logger().info('Voice Navigation Parser ready')
    
    def voice_callback(self, msg):
        """Parse voice command for navigation intent"""
        text = msg.data.lower()
        
        # Check for navigation keywords
        if not any(kw in text for kw in ['go to', 'navigate to', 'take me to', 'move to']):
            return
        
        # Extract destination
        destination = self._extract_destination(text)
        if not destination:
            self.get_logger().warn(f'Could not extract destination from: {text}')
            return
        
        # Create navigation task
        task = {
            'task_id': f'nav_{int(self.get_clock().now().nanoseconds / 1e6)}',
            'source_node': '',  # Use current location
            'destination_node': destination,
            'command': msg.data,
            'timestamp': self.get_clock().now().to_msg().sec
        }
        
        task_msg = String()
        task_msg.data = json.dumps(task)
        self.task_pub.publish(task_msg)
        
        self.get_logger().info(f'Navigation task: Go to {destination}')
    
    def _extract_destination(self, text):
        """Extract destination from text"""
        # Remove navigation keywords
        for kw in ['go to', 'navigate to', 'take me to', 'move to']:
            text = text.replace(kw, '')
        
        text = text.strip()
        
        # Simple extraction - take everything after keywords
        if text:
            return text.replace(' ', '_')
        
        return None


def main(args=None):
    rclpy.init(args=args)
    node = VoiceNavParserNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

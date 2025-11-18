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
        
        self.create_subscription(
            String,
            '/voice/command',
            self.voice_command_callback,
            10
        )

        
        # Publisher to trigger the scene_graph snapshot (used by wake-word)
        self.sg_trigger_pub = self.create_publisher(String, '/scene_graph/trigger', 10)

        self.pending_scene_graph_record = False
        
    def voice_command_callback(self, msg):
        """Listen for voice commands to record junctions"""
        command = msg.data.lower()
        self.get_logger().info(f'Voice command received: "{command}"')
        
        if 'record' in command :
            self.get_logger().info('Voice command detected: Recording junction')
            trigger_msg = String()
            trigger_msg.data = 'snapshot'
            self.get_logger().info('Publishing /scene_graph/trigger snapshot')
            self.sg_trigger_pub.publish(trigger_msg)
            # Set pending flag so scene_graph_callback records the junction
            self.pending_scene_graph_record = True
            
    

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

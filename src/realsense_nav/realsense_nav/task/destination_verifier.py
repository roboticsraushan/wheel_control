#!/usr/bin/env python3
"""
Destination Verifier

Verifies arrival at destination node by matching scene graph.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import json


class DestinationVerifierNode(Node):
    def __init__(self):
        super().__init__('destination_verifier')
        
        self.declare_parameter('match_threshold', 0.7)
        
        self.match_threshold = self.get_parameter('match_threshold').value
        
        self.current_scene_graph = None
        self.current_junction = None
        self.navigation_task = None
        
        self.create_subscription(String, '/scene_graph', self.scene_graph_callback, 10)
        self.create_subscription(String, '/robot/current_junction', self.junction_callback, 10)
        self.create_subscription(String, '/navigation/task', self.task_callback, 10)
        
        self.arrived_pub = self.create_publisher(Bool, '/navigation/arrived', 10)
        self.task_pub = self.create_publisher(String, '/task/start_inspection', 10)
        
        self.create_timer(1.0, self.check_arrival)
        
        self.get_logger().info('Destination Verifier ready')
    
    def scene_graph_callback(self, msg):
        """Store current scene graph"""
        try:
            self.current_scene_graph = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f'Failed to parse scene graph: {e}')
    
    def junction_callback(self, msg):
        """Store current junction"""
        self.current_junction = msg.data
    
    def task_callback(self, msg):
        """Store navigation task"""
        try:
            self.navigation_task = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f'Failed to parse task: {e}')
    
    def check_arrival(self):
        """Check if arrived at destination"""
        if not self.navigation_task or not self.current_scene_graph:
            return
        
        destination_node = self.navigation_task.get('destination_node')
        if not destination_node:
            return
        
        # Check if destination node is in current scene graph
        objects = self.current_scene_graph.get('objects', [])
        
        for obj in objects:
            label = obj.get('label', '').lower()
            
            # Simple string matching
            if destination_node.lower() in label or label in destination_node.lower():
                self.get_logger().info(f'✓ Destination verified: {destination_node} found!')
                
                # Publish arrival
                arrived_msg = Bool()
                arrived_msg.data = True
                self.arrived_pub.publish(arrived_msg)
                
                # Trigger task
                task_msg = String()
                task_msg.data = json.dumps({
                    'destination': destination_node,
                    'junction': self.current_junction,
                    'timestamp': self.get_clock().now().to_msg().sec
                })
                self.task_pub.publish(task_msg)
                
                # Clear task to avoid repeated triggers
                self.navigation_task = None
                
                break


def main(args=None):
    rclpy.init(args=args)
    node = DestinationVerifierNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

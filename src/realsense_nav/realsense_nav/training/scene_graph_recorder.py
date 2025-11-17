#!/usr/bin/env python3
"""
Scene Graph Recorder Node
=================
Raushan: this node is not useful for now
=================

Associates scene graphs with junctions during training.
Listens for junction recording events and stores the scene graph
snapshot for that junction. Helps build node-junction associations.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import os
from datetime import datetime


class SceneGraphRecorderNode(Node):
    def __init__(self):
        super().__init__('scene_graph_recorder')
        
        # Parameters
        self.declare_parameter('data_dir', '/home/raushan/control_one/wheel_control/data/junctions')
        self.data_dir = self.get_parameter('data_dir').value
        
        # Internal state
        self.latest_scene_graph = None
        self.recorded_associations = []
        
        # Subscribers
        self.create_subscription(
            String,
            '/scene_graph',
            self.scene_graph_callback,
            10
        )
        
        self.create_subscription(
            String,
            '/junction/recorded',
            self.junction_recorded_callback,
            10
        )
        
        self.get_logger().info('Scene Graph Recorder initialized')
    
    def scene_graph_callback(self, msg):
        """Store latest scene graph"""
        try:
            self.latest_scene_graph = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f'Failed to parse scene graph: {e}')
    
    def junction_recorded_callback(self, msg):
        """
        Called when a junction is recorded.
        Associate the current scene graph with this junction.
        """
        try:
            event_data = json.loads(msg.data)
            junction_id = event_data.get('junction_id')
            
            if not junction_id:
                self.get_logger().warn('No junction_id in event')
                return
            
            if self.latest_scene_graph is None:
                self.get_logger().warn(f'No scene graph available for junction {junction_id}')
                return
            
            # Extract nodes (objects) from scene graph
            objects = self.latest_scene_graph.get('objects', [])
            
            # Create association record
            association = {
                'junction_id': junction_id,
                'timestamp': datetime.now().isoformat(),
                'num_objects': len(objects),
                'objects': objects
            }
            
            self.recorded_associations.append(association)
            
            # Save association to file
            self._save_association(junction_id, association)
            
            self.get_logger().info(
                f'✓ Associated {len(objects)} objects with junction {junction_id}'
            )
            
        except Exception as e:
            self.get_logger().error(f'Failed to process junction recorded event: {e}')
    
    def _save_association(self, junction_id, association):
        """Save scene graph association to file"""
        filename = f'{junction_id}_scene_graph.json'
        filepath = os.path.join(self.data_dir, filename)
        
        try:
            with open(filepath, 'w') as f:
                json.dump(association, f, indent=2)
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to save association: {e}')
            return False


def main(args=None):
    rclpy.init(args=args)
    node = SceneGraphRecorderNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

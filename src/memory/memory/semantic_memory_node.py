#!/usr/bin/env python3
"""
Semantic Memory Node

Manages semantic information about the environment:
- Object classifications and relationships
- Location semantic labels (e.g., "kitchen", "hallway")
- Persistent object identities and categories
- Spatial relationships between objects

This node builds on top of the floorplan and scene graph data.

Topics:
  - /memory/semantic_map (std_msgs/String): JSON of semantic entities
  - /scene_graph (subscription): Detections from YOLO/segmentation
  - /memory/object_classification (std_msgs/String): Object labels and relationships

Services:
  - /memory/query_semantic_map: Query semantic information
  - /memory/update_semantic_entity: Add/update a semantic entity
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from datetime import datetime
from typing import Dict, List, Any, Optional


class SemanticMemory(Node):
    """Manages semantic understanding of the environment."""

    def __init__(self):
        super().__init__('semantic_memory')

        # ROS parameters
        self.declare_parameter('semantic_map_file', 'data/semantic_map.json')
        self.declare_parameter('max_entities', 1000)
        self.declare_parameter('publish_rate', 1.0)

        self.semantic_map_file = self.get_parameter('semantic_map_file').value
        self.max_entities = self.get_parameter('max_entities').value
        self.publish_rate = self.get_parameter('publish_rate').value

        # State: semantic entities indexed by ID
        self.semantic_entities: Dict[str, Dict[str, Any]] = {}
        self.location_labels: Dict[str, str] = {}  # region_id -> semantic_label
        self.object_relationships: List[Dict[str, Any]] = []

        # Publishers
        self.semantic_map_pub = self.create_publisher(String, '/memory/semantic_map', 10)
        self.object_classification_pub = self.create_publisher(
            String, '/memory/object_classification', 10
        )

        # Subscribers
        self.create_subscription(
            String, '/scene_graph/detections', self.scene_graph_cb, 10
        )

        # Timer for publishing semantic map
        self.create_timer(1.0 / self.publish_rate, self.publish_semantic_map)

        self.get_logger().info('Semantic Memory Node initialized')
        self._load_semantic_map()

    def _load_semantic_map(self):
        """Load semantic map from file if available."""
        try:
            from pathlib import Path
            semantic_file = Path(self.semantic_map_file)
            if not semantic_file.is_absolute():
                semantic_file = Path.cwd() / self.semantic_map_file

            if semantic_file.exists():
                with open(semantic_file, 'r') as f:
                    data = json.load(f)
                    self.semantic_entities = data.get('entities', {})
                    self.location_labels = data.get('locations', {})
                    self.object_relationships = data.get('relationships', [])
                self.get_logger().info(
                    f'Semantic map loaded: {len(self.semantic_entities)} entities'
                )
            else:
                self.get_logger().info('No semantic map file found. Starting fresh.')
        except Exception as e:
            self.get_logger().error(f'Failed to load semantic map: {e}')

    def scene_graph_cb(self, msg):
        """Process detections from scene graph and update semantic memory."""
        try:
            data = json.loads(msg.data)
            objects = data.get('objects', [])

            # Update semantic entities based on detections
            for obj in objects:
                obj_id = str(obj.get('id', 'unknown'))
                label = obj.get('label', 'unknown')
                conf = obj.get('conf', 0.0)

                if obj_id not in self.semantic_entities:
                    # New entity
                    self.semantic_entities[obj_id] = {
                        'id': obj_id,
                        'label': label,
                        'confidence': float(conf),
                        'first_seen': datetime.now().isoformat(),
                        'last_seen': datetime.now().isoformat(),
                        'observations': 1,
                        'attributes': {},
                    }
                else:
                    # Update existing entity
                    entity = self.semantic_entities[obj_id]
                    entity['last_seen'] = datetime.now().isoformat()
                    entity['observations'] += 1
                    # Update label if confidence is higher
                    if conf > entity.get('confidence', 0.0):
                        entity['label'] = label
                        entity['confidence'] = float(conf)

            # Prune old entities if exceeding max
            if len(self.semantic_entities) > self.max_entities:
                self._prune_old_entities()

        except Exception as e:
            self.get_logger().error(f'Error processing scene graph: {e}')

    def _prune_old_entities(self):
        """Remove oldest entities when exceeding max count."""
        # Sort by last_seen time and remove oldest
        sorted_entities = sorted(
            self.semantic_entities.items(),
            key=lambda x: x[1].get('last_seen', ''),
        )
        excess = len(sorted_entities) - self.max_entities
        for i in range(excess):
            del self.semantic_entities[sorted_entities[i][0]]

    def publish_semantic_map(self):
        """Publish the current semantic map."""
        try:
            semantic_data = {
                'timestamp': datetime.now().isoformat(),
                'entities': self.semantic_entities,
                'locations': self.location_labels,
                'relationships': self.object_relationships,
            }
            msg = String()
            msg.data = json.dumps(semantic_data)
            self.semantic_map_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish semantic map: {e}')

    def add_semantic_entity(
        self,
        entity_id: str,
        label: str,
        attributes: Optional[Dict[str, Any]] = None,
    ) -> bool:
        """Add or update a semantic entity."""
        try:
            self.semantic_entities[entity_id] = {
                'id': entity_id,
                'label': label,
                'confidence': 1.0,
                'first_seen': datetime.now().isoformat(),
                'last_seen': datetime.now().isoformat(),
                'observations': 1,
                'attributes': attributes or {},
            }
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to add semantic entity: {e}')
            return False

    def add_location_label(self, region_id: str, semantic_label: str) -> bool:
        """Label a region with semantic meaning (e.g., 'kitchen')."""
        try:
            self.location_labels[region_id] = semantic_label
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to add location label: {e}')
            return False


def main(args=None):
    rclpy.init(args=args)
    node = SemanticMemory()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

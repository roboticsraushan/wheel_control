#!/usr/bin/env python3
"""
Floorplan Manager Node

Loads and manages floorplan data. Provides access to the spatial layout
of the environment. This node serves as the foundation for both semantic
and episodic memory systems.

Topics:
  - /memory/floorplan (sensor_msgs/Image): The current floorplan as an image
  - /memory/floorplan_info (std_msgs/String): JSON metadata about the floorplan
  - /memory/robot_position (geometry_msgs/PointStamped): Robot's current position

Services:
  - /memory/load_floorplan: Load a new floorplan from file
  - /memory/get_floorplan: Get the current floorplan
  - /memory/get_location_from_coordinates: Look up location name from coordinates
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
import os
from pathlib import Path
import yaml


class FloorplanManager(Node):
    """Manages floorplan data and provides spatial context for memory systems."""

    def __init__(self):
        super().__init__('floorplan_manager')

        # ROS parameters
        self.declare_parameter('floorplan_path', 'data/floorplan.png')
        self.declare_parameter('metadata_path', 'data/floorplan_metadata.yaml')
        self.declare_parameter('publish_rate', 1.0)

        self.floorplan_path = self.get_parameter('floorplan_path').value
        self.metadata_path = self.get_parameter('metadata_path').value
        self.publish_rate = self.get_parameter('publish_rate').value

        # State
        self.floorplan_image = None
        self.floorplan_metadata = {}
        self.bridge = CvBridge()
        self.robot_position = Point(x=0.0, y=0.0, z=0.0)

        # Publishers
        self.floorplan_pub = self.create_publisher(Image, '/memory/floorplan', 10)
        self.floorplan_info_pub = self.create_publisher(String, '/memory/floorplan_info', 10)
        self.robot_position_pub = self.create_publisher(
            PointStamped, '/memory/robot_position', 10
        )
        # Also publish an OccupancyGrid so RViz can load the map as a Map display
        self.occupancy_pub = self.create_publisher(OccupancyGrid, '/map', 10)

        # Subscribers
        self.create_subscription(
            PointStamped, '/robot/pose', self.robot_pose_cb, 10
        )

        # Timer for periodic publication
        self.create_timer(1.0 / self.publish_rate, self.publish_floorplan)

        self.get_logger().info('Floorplan Manager Node initialized')

        # Load initial floorplan
        self._load_floorplan()

    def _load_floorplan(self):
        """Load floorplan from disk."""
        try:
            # Resolve path
            floorplan_file = Path(self.floorplan_path)
            if not floorplan_file.is_absolute():
                # Try relative to ROS package
                floorplan_file = Path.cwd() / self.floorplan_path

            if floorplan_file.exists():
                self.floorplan_image = cv2.imread(str(floorplan_file))
                self.get_logger().info(
                    f'Floorplan loaded from {floorplan_file}'
                )
            else:
                # Create a default placeholder if file doesn't exist
                self.get_logger().warn(
                    f'Floorplan file not found at {floorplan_file}. Creating placeholder.'
                )
                self.floorplan_image = np.ones((480, 640, 3), dtype=np.uint8) * 128
                cv2.putText(
                    self.floorplan_image,
                    'No Floorplan Loaded',
                    (100, 240),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.0,
                    (0, 0, 255),
                    2,
                )

            # Load metadata if available
            metadata_file = Path(self.metadata_path)
            if not metadata_file.is_absolute():
                metadata_file = Path.cwd() / self.metadata_path

            if metadata_file.exists():
                with open(metadata_file, 'r') as f:
                    self.floorplan_metadata = yaml.safe_load(f) or {}
                self.get_logger().info(
                    f'Floorplan metadata loaded from {metadata_file}'
                )
            else:
                self.get_logger().info('No metadata file found. Using defaults.')
                self.floorplan_metadata = {
                    'name': 'default_floorplan',
                    'resolution': 0.05,  # meters per pixel
                    'origin': {'x': 0.0, 'y': 0.0},
                    'locations': [],
                }

        except Exception as e:
            self.get_logger().error(f'Failed to load floorplan: {e}')
            self.floorplan_image = np.zeros((480, 640, 3), dtype=np.uint8)

    def publish_floorplan(self):
        """Publish the floorplan and metadata."""
        if self.floorplan_image is None:
            return

        try:
            # Publish floorplan image
            floorplan_msg = self.bridge.cv2_to_imgmsg(self.floorplan_image, 'bgr8')
            floorplan_msg.header.stamp = self.get_clock().now().to_msg()
            floorplan_msg.header.frame_id = 'map'
            self.floorplan_pub.publish(floorplan_msg)

            # Publish metadata as JSON
            metadata_msg = String()
            metadata_msg.data = json.dumps(self.floorplan_metadata)
            self.floorplan_info_pub.publish(metadata_msg)

            # Also publish an OccupancyGrid built from the floorplan image and metadata
            try:
                # Convert to grayscale
                gray = cv2.cvtColor(self.floorplan_image, cv2.COLOR_BGR2GRAY) if len(self.floorplan_image.shape) == 3 else self.floorplan_image.copy()

                # Flip vertically because image origin is top-left, OccupancyGrid expects origin at bottom-left
                gray_flipped = cv2.flip(gray, 0)

                # Threshold to occupancy: pixels darker than threshold -> occupied
                thresh = self.floorplan_metadata.get('occupied_threshold', 128) if isinstance(self.floorplan_metadata, dict) else 128
                _, occ = cv2.threshold(gray_flipped, int(thresh), 100, cv2.THRESH_BINARY_INV)

                # Map to 0 (free) / 100 (occupied)
                # After thresholding, occ pixels are 0 or 100
                data = occ.flatten().tolist()

                # Compose OccupancyGrid
                grid = OccupancyGrid()
                grid.header.stamp = floorplan_msg.header.stamp
                grid.header.frame_id = 'map'

                info = MapMetaData()
                resolution = float(self.floorplan_metadata.get('resolution', 0.05)) if isinstance(self.floorplan_metadata, dict) else 0.05
                info.resolution = resolution
                info.width = int(gray_flipped.shape[1])
                info.height = int(gray_flipped.shape[0])

                origin = self.floorplan_metadata.get('origin', {}) if isinstance(self.floorplan_metadata, dict) else {}
                ox = float(origin.get('x', 0.0)) if isinstance(origin, dict) else 0.0
                oy = float(origin.get('y', 0.0)) if isinstance(origin, dict) else 0.0
                info.origin.position.x = ox
                info.origin.position.y = oy
                info.origin.position.z = 0.0
                info.origin.orientation.x = 0.0
                info.origin.orientation.y = 0.0
                info.origin.orientation.z = 0.0
                info.origin.orientation.w = 1.0

                grid.info = info
                # Ensure values are in [-1,100]
                grid.data = [int(v) if v >= 0 else -1 for v in data]

                self.occupancy_pub.publish(grid)
            except Exception as e:
                self.get_logger().error(f'Failed to publish occupancy grid: {e}')

        except Exception as e:
            self.get_logger().error(f'Failed to publish floorplan: {e}')

    def robot_pose_cb(self, msg):
        """Update robot position from pose topic."""
        self.robot_position = msg.point

        # Publish robot position
        pos_msg = PointStamped()
        pos_msg.header.stamp = self.get_clock().now().to_msg()
        pos_msg.header.frame_id = 'map'
        pos_msg.point = self.robot_position
        self.robot_position_pub.publish(pos_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FloorplanManager()
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

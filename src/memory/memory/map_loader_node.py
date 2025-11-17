#!/usr/bin/env python3
"""
Map Loader Node

Reads a ROS map YAML (as produced by map_server / conversion tools) and the
associated PGM image, converts it to a nav_msgs/OccupancyGrid and publishes it
on the `/map` topic with frame_id `map`.

Parameters:
  - map_yaml: path to the YAML file describing the map (default: data/maps/my_floorplan.yaml)
  - map_topic: topic to publish the OccupancyGrid (default: /map)
  - publish_rate: publish frequency in Hz (default: 1.0)

"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header
import yaml
import os
from pathlib import Path
import numpy as np
import cv2
import math


def yaw_to_quaternion(z):
    # convert yaw angle (radians) to quaternion (x,y,z,w)
    half = z * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


class MapLoader(Node):
    def __init__(self):
        super().__init__('map_loader')

        self.declare_parameter('map_yaml', 'data/maps/my_floorplan.yaml')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('publish_rate', 1.0)

        self.map_yaml = self.get_parameter('map_yaml').value
        self.map_topic = self.get_parameter('map_topic').value
        self.publish_rate = float(self.get_parameter('publish_rate').value)

        self.pub = self.create_publisher(OccupancyGrid, self.map_topic, 10)

        self.map_msg = None

        self.get_logger().info(f'Loading map from: {self.map_yaml}')
        self._load_map()

        # periodic publish so tools like RViz see it
        self.create_timer(1.0 / max(self.publish_rate, 1e-6), self._publish)

    def _load_map(self):
        try:
            yaml_path = Path(self.map_yaml)
            if not yaml_path.is_absolute():
                yaml_path = Path.cwd() / self.map_yaml

            if not yaml_path.exists():
                self.get_logger().error(f'Map YAML not found: {yaml_path}')
                return

            with open(yaml_path, 'r') as f:
                data = yaml.safe_load(f)

            image_file = data.get('image')
            if image_file is None:
                self.get_logger().error('No image field in map yaml')
                return

            image_path = Path(image_file)
            if not image_path.is_absolute():
                image_path = yaml_path.parent / image_file

            if not image_path.exists():
                self.get_logger().error(f'Map image not found: {image_path}')
                return

            # read as grayscale
            img = cv2.imread(str(image_path), cv2.IMREAD_UNCHANGED)
            if img is None:
                self.get_logger().error(f'Failed to read image: {image_path}')
                return

            # If image has multiple channels, convert to grayscale
            if len(img.shape) == 3:
                img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                img = cv2.flip(img, 0)


            height, width = img.shape

            resolution = float(data.get('resolution', 1.0))

            origin_list = data.get('origin', [0.0, 0.0, 0.0])
            ox = float(origin_list[0]) if len(origin_list) > 0 else 0.0
            oy = float(origin_list[1]) if len(origin_list) > 1 else 0.0
            oyaw = float(origin_list[2]) if len(origin_list) > 2 else 0.0

            occupied_thresh = float(data.get('occupied_thresh', data.get('occupied_thresh', 0.65)))
            free_thresh = float(data.get('free_thresh', data.get('free_thresh', 0.196)))
            negate = int(data.get('negate', 0))

            # normalize pixels to [0..1]
            arr = img.astype(np.float32) / 255.0
            if negate:
                arr = 1.0 - arr

            # Build occupancy data following map_server conventions
            flat = []
            for v in arr.flatten():
                if v >= occupied_thresh:
                    flat.append(100)
                elif v <= free_thresh:
                    flat.append(0)
                else:
                    flat.append(-1)

            grid = OccupancyGrid()
            grid.header = Header()
            grid.header.stamp = self.get_clock().now().to_msg()
            grid.header.frame_id = 'map'

            info = MapMetaData()
            info.resolution = resolution
            info.width = int(width)
            info.height = int(height)
            info.origin.position.x = ox
            info.origin.position.y = oy
            info.origin.position.z = 0.0
            qx, qy, qz, qw = yaw_to_quaternion(oyaw)
            info.origin.orientation.x = qx
            info.origin.orientation.y = qy
            info.origin.orientation.z = qz
            info.origin.orientation.w = qw

            grid.info = info
            grid.data = list(map(int, flat))

            self.map_msg = grid
            self.get_logger().info(f'Map loaded: {image_path} ({width}x{height}, res={resolution})')

        except Exception as e:
            self.get_logger().error(f'Failed to load map: {e}')

    def _publish(self):
        if self.map_msg is None:
            return
        # update stamp
        self.map_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.map_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MapLoader()
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

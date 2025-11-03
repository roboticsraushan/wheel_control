#!/usr/bin/env python3
"""
Floorplan Converter Node - ROS 2 Service for converting images to map format.

Provides ROS 2 services to convert floorplan images to standard ROS map format
(PGM + YAML). Supports real-time conversion and upload of floorplan images.

Services:
  - /memory/convert_floorplan (std_srvs/Empty): Trigger conversion of uploaded image
  - /memory/list_maps: List available maps in data/maps/ directory
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from std_msgs.msg import String
import json
from pathlib import Path
import logging

# Import the converter
from memory.floorplan_converter import FloorplanConverter

# Custom service message types (if defined in package)
try:
    from memory.srv import ConvertFloorplan, ListMaps
except ImportError:
    # Fallback: use String-based communication
    ConvertFloorplan = None
    ListMaps = None


logger = logging.getLogger('FloorplanConverterNode')


class FloorplanConverterNode(Node):
    """ROS 2 node for floorplan conversion."""

    def __init__(self):
        super().__init__('floorplan_converter_node')

        # Parameters
        self.declare_parameter('output_dir', 'data/maps')
        self.declare_parameter('auto_threshold', True)
        self.declare_parameter('default_resolution', 0.05)

        self.output_dir = self.get_parameter('output_dir').value
        self.auto_threshold = self.get_parameter('auto_threshold').value
        self.default_resolution = self.get_parameter('default_resolution').value

        # Initialize converter
        self.converter = FloorplanConverter(output_dir=self.output_dir)

        # Create service (using String for compatibility)
        self.create_service(String, '/memory/convert_floorplan', self.convert_floorplan_cb)

        # Publisher for conversion status
        self.status_pub = self.create_publisher(String, '/memory/conversion_status', 10)

        # Subscriber for incoming file paths (for automated conversion)
        self.create_subscription(String, '/memory/floorplan_path', self.floorplan_path_cb, 10)

        self.get_logger().info('Floorplan Converter Node initialized')
        self.get_logger().info(f'Output directory: {self.output_dir}')

    def convert_floorplan_cb(self, request, response):
        """
        Service callback for floorplan conversion.

        Request format (JSON in String):
        {
            "image_path": "/path/to/image.png",
            "output_name": "my_floorplan",
            "resolution": 0.05,
            "origin_x": 0.0,
            "origin_y": 0.0,
            "auto_threshold": true,
            "invert": false
        }

        Response format (JSON in String):
        {
            "success": true,
            "message": "...",
            "pgm_path": "...",
            "yaml_path": "...",
            "map_metadata": {...}
        }
        """
        try:
            # Parse request
            try:
                params = json.loads(request.data)
            except json.JSONDecodeError:
                response.data = json.dumps({
                    'success': False,
                    'message': 'Invalid JSON in request',
                })
                return response

            # Extract parameters
            image_path = params.get('image_path')
            if not image_path:
                response.data = json.dumps({
                    'success': False,
                    'message': 'Missing required parameter: image_path',
                })
                return response

            output_name = params.get('output_name')
            resolution = params.get('resolution', self.default_resolution)
            origin_x = params.get('origin_x', 0.0)
            origin_y = params.get('origin_y', 0.0)
            auto_threshold = params.get('auto_threshold', self.auto_threshold)
            invert = params.get('invert', False)

            self.get_logger().info(f'Converting floorplan: {image_path}')

            # Perform conversion
            result = self.converter.convert(
                image_path=image_path,
                output_name=output_name,
                resolution=resolution,
                origin_x=origin_x,
                origin_y=origin_y,
                auto_threshold=auto_threshold,
                invert=invert,
            )

            # Format response
            response.data = json.dumps(result)

            # Publish status
            status_msg = String()
            status_msg.data = json.dumps({
                'success': result['success'],
                'message': result['message'],
                'pgm_path': result['pgm_path'],
                'yaml_path': result['yaml_path'],
            })
            self.status_pub.publish(status_msg)

            if result['success']:
                self.get_logger().info(f"✅ Conversion successful: {result['message']}")
            else:
                self.get_logger().error(f"❌ Conversion failed: {result['message']}")

            return response

        except Exception as e:
            self.get_logger().error(f'Service error: {e}', exc_info=True)
            response.data = json.dumps({
                'success': False,
                'message': f'Service error: {str(e)}',
            })
            return response

    def floorplan_path_cb(self, msg):
        """Callback for incoming floorplan image paths."""
        self.get_logger().info(f'Received floorplan path: {msg.data}')

        # Convert automatically
        result = self.converter.convert(
            image_path=msg.data,
            resolution=self.default_resolution,
            auto_threshold=self.auto_threshold,
        )

        # Publish result
        status_msg = String()
        status_msg.data = json.dumps(result)
        self.status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FloorplanConverterNode()
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

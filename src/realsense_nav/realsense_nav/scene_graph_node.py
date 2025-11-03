#!/usr/bin/env python3
"""
Scene Graph Builder (prototype)

Subscribes to color, depth and segmentation mask and publishes a compact
scene graph on /scene_graph (std_msgs/String with JSON). Objects are
extracted from connected components in the mask (simple, fast) and a
per-object image centroid + depth is provided.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import json


class SceneGraphNode(Node):
    def __init__(self):
        super().__init__('scene_graph_node')
        # parameters
        self.declare_parameter('use_yolo', False)
        self.declare_parameter('yolo_model', 'yolov8n.pt')
        self.declare_parameter('yolo_conf', 0.3)

        self.use_yolo = self.get_parameter('use_yolo').value
        self.yolo_model_name = self.get_parameter('yolo_model').value
        self.yolo_conf = float(self.get_parameter('yolo_conf').value)

        # This node does NOT run YOLO continuously. Instead it listens to
        # `/scene_graph/detections` published by a separate detector node and
        # will only publish a scene graph snapshot when a trigger is received.
        self.yolo = None
        self.latest_detections = None
        # subscribe to continuous detections (published by yolo_detector)
        self.create_subscription(String, '/scene_graph/detections', self.detections_cb, 10)
        self.bridge = CvBridge()
        self.latest_color = None
        self.latest_depth = None
        self.latest_mask = None

        self.create_subscription(Image, '/camera/camera/color/image_raw', self.color_cb, 2)
        self.create_subscription(Image, '/camera/camera/depth/image_rect_raw', self.depth_cb, 2)
        self.create_subscription(Image, '/segmentation/image', self.mask_cb, 2)

        self.sg_pub = self.create_publisher(String, '/scene_graph', 10)

        self.timer = self.create_timer(0.5, self.publish_scene_graph)
        self.get_logger().info('SceneGraphNode initialized')

        self.trigger_received = False
        self.create_subscription(String, '/scene_graph/trigger', self.trigger_cb, 10)

    def detections_cb(self, msg):
        try:
            self.latest_detections = json.loads(msg.data).get('objects', [])
        except Exception:
            self.latest_detections = None

    def color_cb(self, msg):
        try:
            self.latest_color = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().debug(f'color convert failed: {e}')

    def depth_cb(self, msg):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().debug(f'depth convert failed: {e}')

    def mask_cb(self, msg):
        try:
            self.latest_mask = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        except Exception as e:
            self.get_logger().debug(f'mask convert failed: {e}')

    def trigger_cb(self, msg):
        self.trigger_received = True
        self.get_logger().info('Trigger received for scene graph creation.')

    def publish_scene_graph(self):
        # Only publish on explicit trigger
        if not self.trigger_received:
            return

        self.trigger_received = False

        # Prefer detections published by the continuous detector
        objects = []
        if self.latest_detections:
            try:
                # assume detections already contain id, bbox, centroid_px, depth_m
                for o in self.latest_detections:
                    # canonicalize minimal fields
                    obj = {
                        'id': int(o.get('id', 0)),
                        'label': o.get('label'),
                        'conf': float(o.get('conf')) if o.get('conf') is not None else None,
                        'bbox': [int(v) for v in o.get('bbox', [])],
                        'centroid_px': [int(v) for v in o.get('centroid_px', [])] if o.get('centroid_px') else None,
                        'depth_m': o.get('depth_m'),
                    }
                    objects.append(obj)
            except Exception as e:
                self.get_logger().warn(f'Failed to decode latest_detections: {e}')

        # Fallback / additional: use segmentation connected components when no detections
        if not objects and self.latest_mask is not None:
            mask = self.latest_mask.copy()
            num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
            for i in range(1, num_labels):
                x, y, w, h, area = stats[i]
                cx, cy = int(centroids[i][0]), int(centroids[i][1])

                depth_m = None
                if self.latest_depth is not None:
                    h_d, w_d = self.latest_depth.shape[:2]
                    if 0 <= cy < h_d and 0 <= cx < w_d:
                        z = float(self.latest_depth[cy, cx])
                        if z > 10:
                            depth_m = z / 1000.0
                        else:
                            depth_m = z

                obj = {
                    'id': int(i),
                    'bbox': [int(x), int(y), int(w), int(h)],
                    'centroid_px': [cx, cy],
                    'depth_m': depth_m,
                    'area': int(area),
                }
                objects.append(obj)

        sg = {'timestamp': self.get_clock().now().to_msg().sec, 'objects': objects}
        msg = String()
        msg.data = json.dumps(sg)
        self.sg_pub.publish(msg)
        self.get_logger().info('Scene graph published.')


def main(args=None):
    rclpy.init(args=args)
    node = SceneGraphNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

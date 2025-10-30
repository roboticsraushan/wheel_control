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

        # Try to load YOLO if requested
        self.yolo = None
        if self.use_yolo:
            try:
                from ultralytics import YOLO
                self.get_logger().info(f'Loading YOLO model: {self.yolo_model_name}')
                self.yolo = YOLO(self.yolo_model_name)
                # try to move to GPU if available
                try:
                    self.yolo.to('cuda')
                    self.get_logger().info('YOLO model moved to CUDA')
                except Exception:
                    self.get_logger().info('YOLO model running on CPU')
            except Exception as e:
                self.get_logger().warn(f'Failed to load ultralytics YOLO: {e}. Falling back to mask-based detection.')
                self.yolo = None
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

    def publish_scene_graph(self):
        # Prefer YOLO detections when available
        objects = []
        if self.yolo is not None and self.latest_color is not None:
            try:
                # ultralytics expects RGB images
                img_rgb = cv2.cvtColor(self.latest_color, cv2.COLOR_BGR2RGB)
                results = self.yolo(img_rgb, conf=self.yolo_conf)
                # results may be a list; take first
                res = results[0]
                boxes = getattr(res, 'boxes', None)
                names = getattr(self.yolo, 'names', None) or {}
                if boxes is not None:
                    for i, b in enumerate(boxes.data.tolist()):
                        # b = [x1, y1, x2, y2, conf, cls]
                        x1, y1, x2, y2, conf, cls = b
                        x, y, w, h = int(x1), int(y1), int(x2 - x1), int(y2 - y1)
                        cx = int(x1 + w / 2)
                        cy = int(y1 + h / 2)
                        label = names.get(int(cls), str(int(cls)))

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
                            'id': i + 1,
                            'label': label,
                            'conf': float(conf),
                            'bbox': [x, y, w, h],
                            'centroid_px': [cx, cy],
                            'depth_m': depth_m,
                        }
                        objects.append(obj)
            except Exception as e:
                self.get_logger().warn(f'YOLO detection failed: {e} — falling back to mask CC')

        # Fallback / additional: use segmentation connected components when no YOLO
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

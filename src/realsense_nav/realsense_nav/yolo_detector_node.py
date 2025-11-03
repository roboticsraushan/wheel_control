#!/usr/bin/env python3
"""
YOLO Detector Node

Runs ultralytics YOLO continuously on the color stream, performs a
simple centroid-based tracker to assign persistent IDs, and publishes
the detection list (JSON in a std_msgs/String) on
`/scene_graph/detections`.

This keeps detection running continuously while the scene-graph builder
can still operate on a trigger to snapshot the current detections.
"""
import time
import json
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2


class YOLODetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')

        # params
        self.declare_parameter('yolo_model', 'yolov8n.pt')
        self.declare_parameter('yolo_conf', 0.3)
        self.declare_parameter('min_interval_s', 0.15)
        self.declare_parameter('track_dist_px', 80.0)
        self.declare_parameter('max_lost_s', 1.0)

        self.model_name = self.get_parameter('yolo_model').value
        self.yolo_conf = float(self.get_parameter('yolo_conf').value)
        self.min_interval_s = float(self.get_parameter('min_interval_s').value)
        self.track_dist_px = float(self.get_parameter('track_dist_px').value)
        self.max_lost_s = float(self.get_parameter('max_lost_s').value)

        self.bridge = CvBridge()
        self.latest_color = None
        self.latest_depth = None
        self.camera_info = None

        self.last_run = 0.0

        # Simple tracker state: id -> {'centroid':(x,y), 'label':str, 'last_seen':ts}
        self.next_id = 1
        self.tracked = {}

        # publisher: continuous detection stream
        self.det_pub = self.create_publisher(String, '/scene_graph/detections', 10)

        # subs
        self.create_subscription(Image, '/camera/camera/color/image_raw', self.color_cb, 2)
        self.create_subscription(Image, '/camera/camera/depth/image_rect_raw', self.depth_cb, 2)
        self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.camera_info_cb, 10)

        # Attempt to load YOLO model
        self.yolo = None
        try:
            from ultralytics import YOLO
            self.get_logger().info(f'Loading YOLO model: {self.model_name}')
            self.yolo = YOLO(self.model_name)
            try:
                self.yolo.to('cuda')
                self.get_logger().info('YOLO model moved to CUDA')
            except Exception:
                self.get_logger().info('YOLO running on CPU')
        except Exception as e:
            self.get_logger().warn(f'Failed to load ultralytics YOLO: {e}. Detector will be disabled.')
            self.yolo = None

        self.get_logger().info('YOLODetector initialized')

    def camera_info_cb(self, msg: CameraInfo):
        # store intrinsics (K row-major 3x3)
        try:
            k = msg.k
            if len(k) >= 9:
                self.camera_info = {
                    'fx': float(k[0]),
                    'fy': float(k[4]),
                    'cx': float(k[2]),
                    'cy': float(k[5]),
                }
        except Exception:
            pass

    def color_cb(self, msg: Image):
        try:
            self.latest_color = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            return

        now = time.time()
        if now - self.last_run < self.min_interval_s:
            return
        self.last_run = now

        # Run detection
        if self.yolo is None or self.latest_color is None:
            return

        try:
            img_rgb = cv2.cvtColor(self.latest_color, cv2.COLOR_BGR2RGB)
            results = self.yolo(img_rgb, conf=self.yolo_conf)
            res = results[0]
            boxes = getattr(res, 'boxes', None)
            names = getattr(self.yolo, 'names', None) or {}

            detections = []
            if boxes is not None:
                for b in boxes.data.tolist():
                    x1, y1, x2, y2, conf, cls = b
                    x, y, w, h = int(x1), int(y1), int(x2 - x1), int(y2 - y1)
                    cx = int(x1 + w / 2)
                    cy = int(y1 + h / 2)

                    depth_m = None
                    if self.latest_depth is not None:
                        h_d, w_d = self.latest_depth.shape[:2]
                        if 0 <= cy < h_d and 0 <= cx < w_d:
                            z = float(self.latest_depth[cy, cx])
                            if z > 10:
                                depth_m = z / 1000.0
                            else:
                                depth_m = z

                    world_xyz = None
                    if depth_m is not None and self.camera_info is not None:
                        fx = self.camera_info['fx']
                        fy = self.camera_info['fy']
                        cx_k = self.camera_info['cx']
                        cy_k = self.camera_info['cy']
                        x_cam = (cx - cx_k) * depth_m / fx
                        y_cam = (cy - cy_k) * depth_m / fy
                        z_cam = depth_m
                        world_xyz = [x_cam, y_cam, z_cam]

                    detections.append({
                        'label': names.get(int(cls), str(int(cls))),
                        'conf': float(conf),
                        'bbox': [x, y, w, h],
                        'centroid_px': [cx, cy],
                        'depth_m': depth_m,
                        'camera_xyz': world_xyz,
                    })

            # perform simple centroid-based tracking to assign persistent ids
            assigned = []
            new_tracked = {}
            for det in detections:
                cx, cy = det['centroid_px']
                label = det.get('label')
                best_id = None
                best_dist = None
                for tid, t in self.tracked.items():
                    if t.get('label') != label:
                        continue
                    tx, ty = t.get('centroid')
                    d = math.hypot(tx - cx, ty - cy)
                    if best_dist is None or d < best_dist:
                        best_dist = d
                        best_id = tid

                if best_dist is not None and best_dist < self.track_dist_px and best_id is not None:
                    # reuse id
                    det_id = best_id
                else:
                    det_id = self.next_id
                    self.next_id += 1

                det_out = det.copy()
                det_out['id'] = int(det_id)
                det_out['timestamp'] = int(time.time())
                new_tracked[det_id] = {'centroid': (cx, cy), 'label': label, 'last_seen': time.time()}
                assigned.append(det_out)

            # age out lost trackers
            now_ts = time.time()
            for tid, t in list(self.tracked.items()):
                if tid not in new_tracked:
                    if now_ts - t.get('last_seen', 0) < self.max_lost_s:
                        # keep old until max_lost_s
                        new_tracked[tid] = t

            self.tracked = new_tracked

            # publish detections as JSON
            sg = {'timestamp': int(time.time()), 'objects': assigned}
            msg = String()
            msg.data = json.dumps(sg)
            self.det_pub.publish(msg)

        except Exception as e:
            self.get_logger().warn(f'Detection failed: {e}')

    def depth_cb(self, msg: Image):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = YOLODetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

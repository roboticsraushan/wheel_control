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
import numpy as np


class YOLODetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')

        # params
        self.declare_parameter('yolo_model', 'yolov8n.pt')
        self.declare_parameter('yolo_conf', 0.3)
        self.declare_parameter('min_interval_s', 0.15)
        self.declare_parameter('track_dist_px', 80.0)
        self.declare_parameter('max_lost_s', 1.0)
        # Depth/ROI configuration
        self.declare_parameter('use_aligned_depth', True)
        self.declare_parameter('depth_topic', '/camera/camera/depth/image_rect_raw')
        self.declare_parameter('depth_roi_radius', 2)

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
        # Prefer aligned depth if requested
        self.use_aligned_depth = bool(self.get_parameter('use_aligned_depth').value)
        self.depth_topic = self.get_parameter('depth_topic').value
        self.depth_roi_radius = int(self.get_parameter('depth_roi_radius').value)

        if self.use_aligned_depth:
            # typical aligned depth topic created by the RealSense driver
            aligned = '/camera/camera/aligned_depth_to_color/image_raw'
            # allow override via parameter
            self.depth_topic = aligned if self.depth_topic == '/camera/camera/depth/image_rect_raw' else self.depth_topic

        self.get_logger().info(f'yolo_detector subscribing to depth topic: {self.depth_topic} (aligned={self.use_aligned_depth})')
        self.create_subscription(Image, self.depth_topic, self.depth_cb, 2)
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
                        depth_m = self._get_depth_meters(cx, cy)

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

    def _get_depth_meters(self, cx, cy):
        """
        Extract a depth estimate in meters at pixel (cx, cy) by taking a small
        ROI (radius `depth_roi_radius`) around the centroid, filtering out
        zero/NaN values, and returning the median. Handles images returned
        either in millimeters (uint16) or meters (float32).
        """
        try:
            depth = self.latest_depth
            if depth is None:
                return None
            h, w = depth.shape[:2]
            if not (0 <= cy < h and 0 <= cx < w):
                return None

            r = max(0, int(self.depth_roi_radius))
            x1 = max(0, cx - r)
            x2 = min(w, cx + r + 1)
            y1 = max(0, cy - r)
            y2 = min(h, cy + r + 1)

            patch = np.array(depth[y1:y2, x1:x2]).flatten().astype(np.float64)
            if patch.size == 0:
                return None

            # Filter invalid values
            if np.issubdtype(patch.dtype, np.floating):
                valid = patch[np.isfinite(patch) & (patch > 0.0)]
            else:
                # integers: 0 indicates no depth, regular values in mm
                valid = patch[patch > 0]

            if valid.size == 0:
                return None

            # Use the median to reduce spurious pixels
            med = float(np.median(valid))
            # Many depth sensors provide mm as uint16. If the median is > 10,
            # assume mm and scale to meters. Otherwise it is likely meters.
            if med > 10.0:
                med = med / 1000.0

            if med <= 0 or not np.isfinite(med):
                return None

            return med
        except Exception:
            return None

    @staticmethod
    def median_depth_from_array(depth_arr, cx, cy, radius=2):
        """Static helper identical to `_get_depth_meters` but works
        with a raw numpy array (useful for unit tests).
        """
        if depth_arr is None:
            return None
        try:
            h, w = depth_arr.shape[:2]
            if not (0 <= cy < h and 0 <= cx < w):
                return None
            r = max(0, int(radius))
            x1 = max(0, cx - r)
            x2 = min(w, cx + r + 1)
            y1 = max(0, cy - r)
            y2 = min(h, cy + r + 1)

            patch = np.array(depth_arr[y1:y2, x1:x2]).flatten().astype(np.float64)
            if patch.size == 0:
                return None

            if np.issubdtype(patch.dtype, np.floating):
                valid = patch[np.isfinite(patch) & (patch > 0.0)]
            else:
                valid = patch[patch > 0]

            if valid.size == 0:
                return None

            med = float(np.median(valid))
            if med > 10.0:
                med = med / 1000.0

            if med <= 0 or not np.isfinite(med):
                return None
            return med
        except Exception:
            return None


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

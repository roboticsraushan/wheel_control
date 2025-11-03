#!/usr/bin/env python3
"""
World viewer: shows navigation + segmentation + live detections +
snapshot scene-graph + graph visualization for the current scene.

This is based on the previous `view_segmentation.py` but adds two extra
panels:
  - SCENE_GRAPH_SNAPSHOT: the latest `/scene_graph` snapshot (drawn on
    the color image)
  - SCENE_GRAPH_GRAPH: a graph-style view (nodes arranged in a circle,
    edges between objects that are spatially close). This approximates a
    Graphviz-like visualization without extra dependencies.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import json


class WorldViewer(Node):
    def __init__(self):
        super().__init__('world_viewer')

        self.bridge = CvBridge()

        # Subscriptions
        self.create_subscription(Image, '/segmentation/image', self.seg_image_cb, 10)
        self.create_subscription(Image, '/camera/camera/color/image_raw', self.raw_image_cb, 10)
        self.create_subscription(Image, '/goal/image', self.goal_image_cb, 10)
        self.create_subscription(String, '/scene_graph', self.scene_graph_cb, 10)
        self.create_subscription(String, '/scene_graph/detections', self.detections_cb, 10)
        self.create_subscription(Float32MultiArray, '/pure_pursuit/trajectory', self.trajectory_cb, 10)
        self.create_subscription(PointStamped, '/goal/position', self.goal_position_cb, 10)
        self.create_subscription(Point, '/segmentation/centroid', self.centroid_cb, 10)
        self.create_subscription(Bool, '/goal/detected', self.goal_detected_cb, 10)

        # State
        self.seg_image = None
        self.raw_image = None
        self.goal_image = None
        self.trajectory_data = None
        self.goal_position = None
        self.centroid = None
        self.goal_detected = False
        self.scene_graph = None          # snapshot (triggered) scene graph
        self.latest_detections = None   # continuous detections

        self.get_logger().info('World Viewer started')
        self.get_logger().info('Press "q" to quit, "s" to save screenshot')

        # Create main window sized for multiple panels
        cv2.namedWindow('RealSense World', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('RealSense World', 1920, 720)

    # --- callbacks ---
    def trajectory_cb(self, msg):
        if len(msg.data) >= 5:
            self.trajectory_data = msg.data

    def goal_position_cb(self, msg):
        self.goal_position = msg.point

    def centroid_cb(self, msg):
        self.centroid = (msg.x, msg.y)

    def goal_detected_cb(self, msg):
        self.goal_detected = msg.data

    def seg_image_cb(self, msg):
        try:
            self.seg_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.display()
        except Exception as e:
            self.get_logger().error(f'Error converting segmentation image: {e}')

    def raw_image_cb(self, msg):
        try:
            self.raw_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error converting raw image: {e}')

    def goal_image_cb(self, msg):
        try:
            self.goal_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error converting goal image: {e}')

    def scene_graph_cb(self, msg):
        try:
            # snapshot-style scene graph (published by scene_graph_node on trigger)
            self.scene_graph = json.loads(msg.data)
        except Exception:
            self.scene_graph = None

    def detections_cb(self, msg):
        try:
            self.latest_detections = json.loads(msg.data)
        except Exception:
            self.latest_detections = None

    # --- drawing helpers ---
    def draw_trajectory_overlay(self, image):
        h, w = image.shape[:2]
        cv2.putText(image, 'TRAJECTORY', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        if self.centroid is not None:
            cx, cy = int(self.centroid[0]), int(self.centroid[1])
            cv2.circle(image, (cx, cy), 8, (0, 255, 0), -1)
        if self.goal_detected and self.goal_position is not None:
            goal_x = self.goal_position.x
            goal_y = self.goal_position.y
            if goal_x > 0.1:
                pixel_x = int(w/2 - goal_y * (w/2) / max(goal_x, 0.001))
                pixel_y = int(h * 0.7)
                pixel_x = max(0, min(w-1, pixel_x))
                cv2.circle(image, (pixel_x, pixel_y), 12, (0, 255, 255), 3)
                cv2.putText(image, f'GOAL ({goal_x:.1f}m)', (pixel_x + 20, pixel_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)
        return image

    def draw_yolo_overlay(self, image):
        objs = []
        if self.latest_detections is not None:
            objs = self.latest_detections.get('objects', [])
        elif self.scene_graph is not None:
            objs = self.scene_graph.get('objects', [])

        for o in objs:
            bbox = o.get('bbox', None)
            label = o.get('label', None)
            conf = o.get('conf', None)
            centroid = o.get('centroid_px', None)

            if bbox:
                try:
                    x, y, w, h = [int(v) for v in bbox]
                except Exception:
                    continue
                cv2.rectangle(image, (x, y), (x + w, y + h), (0, 165, 255), 2)
                lab = ''
                if label:
                    lab += str(label)
                if conf is not None:
                    try:
                        lab += f' {float(conf):.2f}'
                    except Exception:
                        pass
                if lab:
                    (tw, th), _ = cv2.getTextSize(lab, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
                    cv2.rectangle(image, (x, y - th - 6), (x + tw + 4, y), (0, 165, 255), -1)
                    cv2.putText(image, lab, (x + 2, y - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 1)
            if centroid:
                try:
                    cx, cy = int(centroid[0]), int(centroid[1])
                    cv2.circle(image, (cx, cy), 4, (0, 0, 255), -1)
                except Exception:
                    pass

        return image

    def draw_scene_graph_snapshot(self):
        # Render the snapshot scene_graph on top of the raw image (if available)
        if self.raw_image is None:
            # return a gray placeholder
            return np.full((self.seg_image.shape[0], self.seg_image.shape[1], 3), 80, dtype=np.uint8) if self.seg_image is not None else np.zeros((480,640,3), dtype=np.uint8)

        img = self.raw_image.copy()
        if self.scene_graph is None:
            cv2.putText(img, 'NO SCENE_GRAPH SNAPSHOT', (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200,200,200), 2)
            return img

        objs = self.scene_graph.get('objects', [])
        for o in objs:
            bbox = o.get('bbox', None)
            label = o.get('label', '')
            conf = o.get('conf', None)
            if bbox:
                try:
                    x, y, w, h = [int(v) for v in bbox]
                except Exception:
                    continue
                cv2.rectangle(img, (x,y), (x+w, y+h), (0,255,0), 2)
                lab = f'{label} {conf:.2f}' if conf is not None else label
                (tw, th), _ = cv2.getTextSize(lab, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
                cv2.rectangle(img, (x, y - th - 6), (x + tw + 4, y), (0,255,0), -1)
                cv2.putText(img, lab, (x + 2, y - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 1)

        return img

    def draw_graph_view(self):
        # Create a circular layout of detected objects and connect close ones.
        size = 480
        canvas = np.full((size, size, 3), 30, dtype=np.uint8)
        objs = []
        if self.latest_detections is not None:
            objs = self.latest_detections.get('objects', [])
        elif self.scene_graph is not None:
            objs = self.scene_graph.get('objects', [])

        n = len(objs)
        if n == 0:
            cv2.putText(canvas, 'NO OBJECTS', (20, size//2), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (200,200,200), 2)
            return canvas

        # positions on circle
        cx = size // 2
        cy = size // 2
        r = int(size * 0.35)
        positions = []
        for i, o in enumerate(objs):
            ang = 2 * math.pi * i / n
            x = int(cx + r * math.cos(ang))
            y = int(cy + r * math.sin(ang))
            positions.append((x, y))

        # Draw edges between objects whose centroids are close in image space
        centroids = []
        for o in objs:
            cp = o.get('centroid_px')
            if cp:
                centroids.append((int(cp[0]), int(cp[1])))
            else:
                centroids.append(None)

        # connect if distance < threshold (in pixels)
        thr = 200
        for i in range(n):
            for j in range(i+1, n):
                c1 = centroids[i]
                c2 = centroids[j]
                if c1 is None or c2 is None:
                    continue
                d = math.hypot(c1[0]-c2[0], c1[1]-c2[1])
                if d < thr:
                    cv2.line(canvas, positions[i], positions[j], (100,100,255), 2)

        # draw nodes
        for i, o in enumerate(objs):
            x, y = positions[i]
            cv2.circle(canvas, (x, y), 28, (50,200,50), -1)
            cv2.circle(canvas, (x, y), 30, (255,255,255), 2)
            lab = f"{o.get('id', i+1)}:{o.get('label','?')}"
            (tw, th), _ = cv2.getTextSize(lab, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.putText(canvas, lab, (x - tw//2, y + th//2), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 1)

        cv2.putText(canvas, 'SCENE GRAPH (graph view)', (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200,200,200), 2)
        return canvas

    def display(self):
        if self.seg_image is None:
            return

        images_to_show = []

        # Trajectory (on raw image)
        if self.raw_image is not None:
            traj = self.draw_trajectory_overlay(self.raw_image.copy())
            images_to_show.append(traj)

        # Segmentation
        images_to_show.append(self.seg_image)

        # Goal image
        if self.goal_image is not None:
            images_to_show.append(self.goal_image)

        # Live YOLO
        if self.raw_image is not None:
            yv = self.draw_yolo_overlay(self.raw_image.copy())
            images_to_show.append(yv)

        # Scene graph snapshot (drawn on raw image)
        sg_img = self.draw_scene_graph_snapshot()
        images_to_show.append(sg_img)

        # Graph view
        gv = self.draw_graph_view()
        images_to_show.append(gv)

        # Resize all to same height and stack
        target_h = images_to_show[0].shape[0]
        resized = []
        for img in images_to_show:
            if img.shape[0] != target_h:
                scale = target_h / img.shape[0]
                new_w = int(img.shape[1] * scale)
                img = cv2.resize(img, (new_w, target_h))
            resized.append(img)

        display = np.hstack(resized)

        # panel labels
        label_y = display.shape[0] - 10
        panel_labels = ['TRAJECTORY', 'SEGMENTATION', 'GOAL DETECTION', 'YOLO LIVE', 'SCENE_GRAPH_SNAPSHOT', 'SCENE_GRAPH_GRAPH']
        offset = 0
        for i, img in enumerate(resized):
            lab = panel_labels[i] if i < len(panel_labels) else f'PANEL {i}'
            cv2.putText(display, lab, (offset + 10, label_y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
            offset += img.shape[1]

        cv2.imshow('RealSense World', display)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.get_logger().info('Quitting...')
            rclpy.shutdown()
        elif key == ord('s'):
            filename = f'world_screenshot_{cv2.getTickCount()}.png'
            cv2.imwrite(filename, display)
            self.get_logger().info(f'Saved screenshot: {filename}')


def main():
    rclpy.init()
    node = WorldViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

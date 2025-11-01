#!/usr/bin/env python3
"""
LLM Goal Detection Node

Listens to `/scene_graph` (JSON) produced by SceneGraphNode (YOLO or masks),
prints and exposes the list of detected items, accepts an LLM / human command
on `/llm_goal` (std_msgs/String) and resolves it to a concrete target which
is published both as `/topo_goal` (JSON string like NLInterpreter) and as
`/goal/position` (PointStamped) when depth/camera info are available.

This allows a language model or teleop UI to say e.g. "go to the chair"
and have the robot set a visual centroid goal on the detected object.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import json
import re
import numpy as np
import cv2


class LLMGoalDetection(Node):
    def __init__(self):
        super().__init__('llm_goal_detection')

        self.bridge = CvBridge()
        self.scene_graph = None
        self.raw_image = None
        self.depth_image = None
        self.cam_info = None

        # Subscribers
        self.create_subscription(String, '/scene_graph', self.sg_cb, 10)
        self.create_subscription(String, '/llm_goal', self.llm_cb, 10)
        self.create_subscription(Image, '/camera/camera/color/image_raw', self.img_cb, 5)
        self.create_subscription(Image, '/camera/camera/depth/image_rect_raw', self.depth_cb, 5)
        self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.caminfo_cb, 5)

        # Publishers
        self.topo_pub = self.create_publisher(String, '/topo_goal', 10)
        self.goal_pub = self.create_publisher(PointStamped, '/goal/position', 10)
        self.goal_detected_pub = self.create_publisher(Bool, '/goal/detected', 10)
        self.vis_pub = self.create_publisher(Image, '/goal/image', 5)

        self.get_logger().info('LLM Goal Detection node started')
        self.get_logger().info('Publish a string to /llm_goal to resolve a target (e.g. "go to person")')

    def sg_cb(self, msg: String):
        try:
            self.scene_graph = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warning(f'Invalid scene_graph JSON: {e}')
            self.scene_graph = None

    def llm_cb(self, msg: String):
        text = msg.data.lower()
        self.get_logger().info(f'Received LLM goal: "{text}"')
        if self.scene_graph is None:
            self.get_logger().info('No scene graph available to resolve goal')
            return

        target = self.resolve_text_to_object(text)
        if target is None:
            self.get_logger().info('No matching object found for LLM goal')
            # publish failure-like topo message
            res = {'success': False, 'reason': 'no match'}
            self.topo_pub.publish(String(data=json.dumps(res)))
            return

        # publish topo goal JSON (same shape as NLInterpreter)
        res = {'success': True, 'target': target}
        self.topo_pub.publish(String(data=json.dumps(res)))
        self.get_logger().info(f'Published /topo_goal for id={target.get("id")}')

        # also publish goal/position if we can compute 3D point
        centroid = target.get('centroid_px', None)
        depth = target.get('depth_m', None)
        if centroid and depth and self.cam_info is not None:
            try:
                fx = self.cam_info.k[0]
                fy = self.cam_info.k[4]
                cx = self.cam_info.k[2]
                cy = self.cam_info.k[5]

                u, v = float(centroid[0]), float(centroid[1])
                z = float(depth)
                if z <= 0:
                    raise ValueError('non-positive depth')

                x = (u - cx) * z / fx
                y = (v - cy) * z / fy

                p = PointStamped()
                p.header.frame_id = self.cam_info.header.frame_id if self.cam_info.header.frame_id else 'camera_color_optical_frame'
                p.header.stamp = self.get_clock().now().to_msg()
                # follow same convention as existing node (forward in x)
                p.point.x = z
                p.point.y = -x
                p.point.z = -y
                self.goal_pub.publish(p)
                self.goal_detected_pub.publish(Bool(data=True))
                self.get_logger().info(f'Published /goal/position for id={target.get("id")}')
            except Exception as e:
                self.get_logger().warning(f'Failed to compute 3D goal: {e}')
                self.goal_detected_pub.publish(Bool(data=False))
        else:
            # no depth/camera info -> still notify not detected
            self.goal_detected_pub.publish(Bool(data=False))

        # publish visualization image with overlay if we have a raw image
        if self.raw_image is not None:
            vis = self.draw_scene_overlay(self.raw_image.copy(), highlight_id=target.get('id'))
            try:
                out = self.bridge.cv2_to_imgmsg(vis, encoding='bgr8')
                self.vis_pub.publish(out)
            except Exception as e:
                self.get_logger().warning(f'Failed to publish vis image: {e}')

    def img_cb(self, msg: Image):
        try:
            self.raw_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            self.raw_image = None

    def depth_cb(self, msg: Image):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception:
            self.depth_image = None

    def caminfo_cb(self, msg: CameraInfo):
        self.cam_info = msg

    def resolve_text_to_object(self, text: str):
        # simple heuristics: 'object N', exact label match, contains label, 'big', 'nearest'
        objs = self.scene_graph.get('objects', [])
        if not objs:
            return None

        # object N
        m = re.search(r'object\s*(\d+)', text)
        if m:
            oid = int(m.group(1))
            for o in objs:
                if o.get('id') == oid or int(o.get('id', -1)) == oid:
                    return o

        # largest / biggest
        if 'big' in text or 'largest' in text:
            return max(objs, key=lambda o: o.get('area', 0))

        # nearest: use smallest depth
        if 'near' in text or 'closest' in text:
            candidates = [o for o in objs if o.get('depth_m')]
            if candidates:
                return min(candidates, key=lambda o: o.get('depth_m', float('inf')))

        # match by label (exact first, then contains)
        for o in objs:
            lab = str(o.get('label', '')).lower()
            if lab and lab == text:
                return o

        for o in objs:
            lab = str(o.get('label', '')).lower()
            if lab and lab in text:
                return o

        # fallback: first object
        return objs[0]

    def draw_scene_overlay(self, image, highlight_id=None):
        # draw bboxes and labels from scene_graph
        if self.scene_graph is None:
            return image

        for o in self.scene_graph.get('objects', []):
            bbox = o.get('bbox', None)
            label = o.get('label', None)
            conf = o.get('conf', None)
            oid = o.get('id', None)

            color = (0, 165, 255) if oid == highlight_id else (0, 255, 0)

            if bbox:
                try:
                    x, y, w, h = [int(v) for v in bbox]
                except Exception:
                    continue
                cv2.rectangle(image, (x, y), (x + w, y + h), color, 2)
                lab = f'id:{oid}'
                if label:
                    lab += f' {label}'
                if conf is not None:
                    try:
                        lab += f' {float(conf):.2f}'
                    except Exception:
                        pass
                (tw, th), _ = cv2.getTextSize(lab, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
                cv2.rectangle(image, (x, y - th - 6), (x + tw + 4, y), color, -1)
                cv2.putText(image, lab, (x + 2, y - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)

        return image


def main(args=None):
    rclpy.init(args=args)
    node = LLMGoalDetection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

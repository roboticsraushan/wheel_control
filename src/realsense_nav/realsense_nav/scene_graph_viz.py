#!/usr/bin/env python3
"""
Scene Graph Visualizer

Subscribes to /scene_graph (JSON) and /camera/camera/color/image_raw and
publishes an overlaid image on /scene_graph/image and a MarkerArray on
/scene_graph/markers for RViz.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
import cv2
import json


class SceneGraphViz(Node):
    def __init__(self):
        super().__init__('scene_graph_viz')
        self.bridge = CvBridge()
        self.sg = None
        self.cam_info = None

        self.create_subscription(String, '/scene_graph', self.sg_cb, 10)
        self.create_subscription(Image, '/camera/camera/color/image_raw', self.img_cb, 5)
        self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.caminfo_cb, 5)

        self.img_pub = self.create_publisher(Image, '/scene_graph/image', 5)
        self.mark_pub = self.create_publisher(MarkerArray, '/scene_graph/markers', 5)

        self.get_logger().info('SceneGraphViz initialized')

    def sg_cb(self, msg: String):
        try:
            self.sg = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warning(f'bad scene_graph json: {e}')
            self.sg = None

    def caminfo_cb(self, msg: CameraInfo):
        self.cam_info = msg

    def img_cb(self, msg: Image):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().debug(f'img convert failed: {e}')
            return

        vis = img.copy()
        if self.sg is not None:
            for o in self.sg.get('objects', []):
                bbox = o.get('bbox', None)
                cid = o.get('id', '')
                label = o.get('label', None)
                conf = o.get('conf', None)
                area = o.get('area', 0)
                centroid = o.get('centroid_px', None)

                if bbox:
                    x, y, w, h = bbox
                    cv2.rectangle(vis, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    lab = f'id:{cid}'
                    if label:
                        lab += f' {label}'
                    if conf is not None:
                        lab += f' {conf:.2f}'
                    cv2.putText(vis, lab, (x, y - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

                if centroid:
                    cx, cy = int(centroid[0]), int(centroid[1])
                    cv2.circle(vis, (cx, cy), 5, (0, 0, 255), -1)

        # publish overlay image
        try:
            out = self.bridge.cv2_to_imgmsg(vis, encoding='bgr8')
            out.header = msg.header
            self.img_pub.publish(out)
        except Exception as e:
            self.get_logger().debug(f'failed to publish vis image: {e}')

        # publish markers (3D) if we have camera info and depths
        if self.cam_info is not None and self.sg is not None:
            markers = MarkerArray()
            frame = self.cam_info.header.frame_id if self.cam_info.header.frame_id else 'camera_link'
            fx = self.cam_info.k[0]
            fy = self.cam_info.k[4]
            cx = self.cam_info.k[2]
            cy = self.cam_info.k[5]

            for o in self.sg.get('objects', []):
                depth = o.get('depth_m', None)
                centroid = o.get('centroid_px', None)
                if depth is None or centroid is None:
                    continue
                u, v = float(centroid[0]), float(centroid[1])
                z = float(depth)
                if z <= 0:
                    continue
                x = (u - cx) * z / fx
                y = (v - cy) * z / fy

                m = Marker()
                m.header.frame_id = frame
                m.header.stamp = self.get_clock().now().to_msg()
                m.ns = 'scene_graph'
                m.id = int(o.get('id', 0))
                m.type = Marker.SPHERE
                m.action = Marker.ADD
                m.pose.position.x = float(x)
                m.pose.position.y = float(y)
                m.pose.position.z = float(z)
                m.pose.orientation.w = 1.0
                m.scale.x = 0.05
                m.scale.y = 0.05
                m.scale.z = 0.05
                m.color.r = 1.0
                m.color.g = 0.2
                m.color.b = 0.2
                m.color.a = 0.9
                markers.markers.append(m)

            if markers.markers:
                self.mark_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = SceneGraphViz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

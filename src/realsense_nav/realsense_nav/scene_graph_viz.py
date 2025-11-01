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
import numpy as np
import json
from geometry_msgs.msg import Point as GeoPoint
from geometry_msgs.msg import PointStamped
import tf2_ros
import tf2_geometry_msgs


class SceneGraphViz(Node):
    def __init__(self):
        super().__init__('scene_graph_viz')
        self.bridge = CvBridge()
        self.sg = None
        self.cam_info = None
        # persistent topo nodes: id -> {'count':int, 'pos':(x,y), 'label':str}
        self.node_presence = {}
        self.topo_nodes = {}  # persisted nodes

        # parameters for persistence, mini-map and topo frame
        self.declare_parameter('topo_persist_frames', 5)
        self.declare_parameter('minimap_size', 300)
        self.declare_parameter('minimap_scale', 100)  # pixels per meter
        self.declare_parameter('topo_frame', 'base_link')  # frame to publish topo markers in (robot-centric)

        # TF buffer/listener for transforms from camera frame -> topo_frame
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(String, '/scene_graph', self.sg_cb, 10)
        self.create_subscription(Image, '/camera/camera/color/image_raw', self.img_cb, 5)
        self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.caminfo_cb, 5)

        self.img_pub = self.create_publisher(Image, '/scene_graph/image', 5)
        self.mark_pub = self.create_publisher(MarkerArray, '/scene_graph/markers', 5)
        self.topo_mark_pub = self.create_publisher(MarkerArray, '/topo/nodes_markers', 5)
        self.minimap_pub = self.create_publisher(Image, '/topo/minimap_image', 5)

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

                # Also publish a 3D bounding-box outline projected at centroid depth
                bbox = o.get('bbox', None)
                if bbox:
                    bx, by, bw, bh = bbox
                    corners_px = [
                        (bx, by),
                        (bx + bw, by),
                        (bx + bw, by + bh),
                        (bx, by + bh),
                    ]

                    box_marker = Marker()
                    box_marker.header.frame_id = frame
                    box_marker.header.stamp = self.get_clock().now().to_msg()
                    box_marker.ns = 'scene_graph_bbox'
                    # make id distinct from sphere id
                    box_marker.id = int(o.get('id', 0)) * 10 + 1
                    box_marker.type = Marker.LINE_STRIP
                    box_marker.action = Marker.ADD
                    box_marker.scale.x = 0.01  # line width in meters
                    box_marker.color.r = 0.0
                    box_marker.color.g = 1.0
                    box_marker.color.b = 0.0
                    box_marker.color.a = 0.9

                    pts = []
                    for (u_px, v_px) in corners_px:
                        zx = float(z)
                        x3 = (float(u_px) - cx) * zx / fx
                        y3 = (float(v_px) - cy) * zx / fy
                        p = GeoPoint()
                        p.x = float(x3)
                        p.y = float(y3)
                        p.z = float(zx)
                        pts.append(p)

                    # close loop
                    pts.append(pts[0])
                    box_marker.points = pts
                    markers.markers.append(box_marker)

            if markers.markers:
                self.mark_pub.publish(markers)

            # update persistent topo nodes based on detections
            try:
                self._update_topo_nodes(self.sg, fx, fy, cx, cy)
            except Exception as e:
                self.get_logger().warning(f'failed to update topo nodes: {e}')

            # publish persistent topo node markers and mini-map
            try:
                self._publish_topo_markers()
                self._publish_minimap()
            except Exception as e:
                self.get_logger().warning(f'failed to publish topo visuals: {e}')

    def _update_topo_nodes(self, sg, fx, fy, cx, cy):
        """Update persistent topo nodes when objects persist across frames."""
        persist_frames = int(self.get_parameter('topo_persist_frames').value)
        seen_ids = set()
        for o in sg.get('objects', []):
            oid = int(o.get('id', -1))
            depth = o.get('depth_m', None)
            centroid = o.get('centroid_px', None)
            if oid < 0 or depth is None or centroid is None:
                continue
            try:
                u, v = float(centroid[0]), float(centroid[1])
                z = float(depth)
                if z <= 0:
                    continue
                x_cam = (u - cx) * z / fx
                y_cam = (v - cy) * z / fy
            except Exception:
                continue

            seen_ids.add(oid)
            self.node_presence[oid] = self.node_presence.get(oid, 0) + 1

            if self.node_presence[oid] >= persist_frames and oid not in self.topo_nodes:
                # create persistent node entry
                self.topo_nodes[oid] = {
                    'pos_cam': (float(x_cam), float(y_cam), float(z)),
                    'label': o.get('label', f'obj{oid}'),
                    'conf': float(o.get('conf', 0.0)),
                }

        # decay presence counters for ids not seen
        for oid in list(self.node_presence.keys()):
            if oid not in seen_ids:
                self.node_presence[oid] = max(0, self.node_presence[oid] - 1)

    def _publish_topo_markers(self):
        """Publish persistent topo nodes as MarkerArray for RViz."""
        if not self.topo_nodes:
            return
        markers = MarkerArray()
        source_frame = self.cam_info.header.frame_id if self.cam_info and self.cam_info.header.frame_id else 'camera_link'
        target_frame = str(self.get_parameter('topo_frame').value)
        now = self.get_clock().now().to_msg()

        for oid, info in self.topo_nodes.items():
            x_cam, y_cam, z = info['pos_cam']
            label = info.get('label', str(oid))

            # create a PointStamped in camera frame then try to transform to target_frame
            p_cam = PointStamped()
            p_cam.header.frame_id = source_frame
            p_cam.header.stamp = now
            p_cam.point.x = float(x_cam)
            p_cam.point.y = float(y_cam)
            p_cam.point.z = float(z)

            frame_to_use = source_frame
            x_r, y_r, z_r = float(x_cam), float(y_cam), float(z)
            try:
                tpt = self.tf_buffer.transform(p_cam, target_frame, timeout=rclpy.duration.Duration(seconds=0.5))
                x_r = tpt.point.x
                y_r = tpt.point.y
                z_r = tpt.point.z
                frame_to_use = target_frame
            except Exception as e:
                # TF may not be available; fall back to camera frame coords
                self.get_logger().debug(f'could not transform topo node {oid} to {target_frame}: {e}')

            m = Marker()
            m.header.frame_id = frame_to_use
            m.header.stamp = now
            m.ns = 'topo_nodes'
            m.id = int(oid)
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(x_r)
            m.pose.position.y = float(y_r)
            m.pose.position.z = float(z_r)
            m.pose.orientation.w = 1.0
            m.scale.x = 0.08
            m.scale.y = 0.08
            m.scale.z = 0.08
            m.color.r = 0.0
            m.color.g = 0.5
            m.color.b = 1.0
            m.color.a = 0.9
            markers.markers.append(m)

            # text label
            mt = Marker()
            mt.header.frame_id = frame_to_use
            mt.header.stamp = now
            mt.ns = 'topo_nodes_text'
            mt.id = int(oid) * 100
            mt.type = Marker.TEXT_VIEW_FACING
            mt.action = Marker.ADD
            mt.pose.position.x = float(x_r)
            mt.pose.position.y = float(y_r)
            mt.pose.position.z = float(z_r) + 0.12
            mt.pose.orientation.w = 1.0
            mt.scale.z = 0.08  # text height
            mt.color.r = 1.0
            mt.color.g = 1.0
            mt.color.b = 1.0
            mt.color.a = 1.0
            mt.text = f"{label}:{oid}"
            markers.markers.append(mt)

        self.topo_mark_pub.publish(markers)

    def _publish_minimap(self):
        """Render and publish a simple robot-centric top-down mini-map as an image."""
        if not self.topo_nodes or self.cam_info is None:
            return
        size = int(self.get_parameter('minimap_size').value)
        scale = float(self.get_parameter('minimap_scale').value)
        canvas = np.zeros((size, size, 3), dtype=np.uint8)
        canvas[:] = (40, 40, 40)
        cx = size // 2
        cy = size // 2

        # draw robot at center (triangle)
        pts = np.array([[cx, cy - 10], [cx - 8, cy + 12], [cx + 8, cy + 12]], np.int32)
        cv2.fillConvexPoly(canvas, pts, (0, 200, 0))

        for oid, info in self.topo_nodes.items():
            x_cam, y_cam, z = info['pos_cam']
            label = info.get('label', str(oid))
            # map coords: forward = z, lateral = -x_cam (so left is positive)
            map_x = float(z)
            map_y = -float(x_cam)
            px = int(cx + map_y * scale)
            py = int(cy - map_x * scale)
            if 0 <= px < size and 0 <= py < size:
                cv2.circle(canvas, (px, py), 6, (255, 150, 0), -1)
                cv2.putText(canvas, f'{label}:{oid}', (px + 8, py - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (230, 230, 230), 1)

        # publish image
        try:
            img_msg = self.bridge.cv2_to_imgmsg(canvas, encoding='bgr8')
            img_msg.header.frame_id = self.cam_info.header.frame_id if self.cam_info and self.cam_info.header.frame_id else 'camera_link'
            img_msg.header.stamp = self.get_clock().now().to_msg()
            self.minimap_pub.publish(img_msg)
        except Exception as e:
            self.get_logger().warning(f'failed to publish minimap image: {e}')


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

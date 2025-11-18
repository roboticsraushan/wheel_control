#!/usr/bin/env python3
"""
scene_graph_mapper.py

Subscribe to /scene_graph (JSON string), convert detections from camera frame
to the global `map` frame using TF2 and CameraInfo, and publish
visualization markers on /scene_graph/map_markers.

This node is intentionally conservative: it prefers an existing `camera_xyz`
field in the detection JSON, otherwise it will compute camera-space XYZ from
centroid_px + depth_m using CameraInfo intrinsics.

Make it executable and place in the realsense_nav package folder so the
training launch can run it directly from workspace.
"""
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped

from tf2_ros import Buffer, TransformListener


class SceneGraphMapper(Node):
    def __init__(self):
        super().__init__('scene_graph_mapper')

        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('scene_graph_topic', '/scene_graph')
        self.declare_parameter('marker_topic', '/scene_graph/map_markers')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('camera_frame_fallback', 'camera_color_optical_frame')
        # prefer to express detections in a canonical camera frame; this is
        # usually 'camera_link' in our system (mapped to base_link via a static transform)
        # We expose it as a parameter so different hardware setups can choose
        # 'camera_color_optical_frame' or 'camera_link' as the canonical frame.
        self.declare_parameter('camera_reference_frame', 'map')
        # Optional default intrinsics used when CameraInfo is not available
        # Set these via launch or param to something reasonable for your camera
        self.declare_parameter('default_fx', 0.0)
        self.declare_parameter('default_fy', 0.0)
        self.declare_parameter('default_cx', 0.0)
        self.declare_parameter('default_cy', 0.0)

        cam_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        scene_graph_topic = self.get_parameter('scene_graph_topic').get_parameter_value().string_value
        self.marker_topic = self.get_parameter('marker_topic').get_parameter_value().string_value
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
        self.camera_frame_fallback = self.get_parameter('camera_frame_fallback').get_parameter_value().string_value

        # default intrinsics (may be zero; if set, used when CameraInfo missing)
        self.default_fx = float(self.get_parameter('default_fx').get_parameter_value().double_value)
        self.default_fy = float(self.get_parameter('default_fy').get_parameter_value().double_value)
        self.default_cx = float(self.get_parameter('default_cx').get_parameter_value().double_value)
        self.default_cy = float(self.get_parameter('default_cy').get_parameter_value().double_value)

        self.cam_info = None
        self.camera_frame = None
        self.camera_reference_frame = self.get_parameter('camera_reference_frame').get_parameter_value().string_value

        self.create_subscription(CameraInfo, cam_info_topic, self.camera_info_cb, 10)
        self.create_subscription(String, scene_graph_topic, self.scene_graph_cb, 10)
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info(f'scene_graph_mapper started; listening to {scene_graph_topic}')

        self._marker_id_seq = 0

    def camera_info_cb(self, msg: CameraInfo):
        # store camera intrinsics and frame id
        self.cam_info = msg
        if not self.camera_frame:
            self.camera_frame = msg.header.frame_id or self.camera_frame_fallback
            self.get_logger().info(f'Using camera frame: {self.camera_frame}')

    def _do_transform_pose_robust(self, pose_stamped: PoseStamped, transform):
        """Call tf2_geometry_msgs.do_transform_pose robustly.

        Some tf2_geometry_msgs versions accept a PoseStamped while others
        accept Pose only. Try both to maximize compatibility.
        """
        try:
            from tf2_geometry_msgs import do_transform_pose
        except Exception as e:
            raise

        try:
            return do_transform_pose(pose_stamped, transform)
        except Exception as e:
            # fallback: some older/newer versions accept a Pose rather than a
            # PoseStamped; try calling with the inner pose and rewrap the result
            try:
                result_pose = do_transform_pose(pose_stamped.pose, transform)
                # result_pose could be a Pose or a PoseStamped; normalize
                if hasattr(result_pose, 'position') or hasattr(result_pose, 'orientation'):
                    # geometry_msgs/Pose returned; build PoseStamped
                    out = PoseStamped()
                    out.header = pose_stamped.header
                    out.pose = result_pose
                    return out
                return result_pose
            except Exception:
                # re-raise original exception for logging upstream
                raise e

    def _parse_objects(self, payload):
        # payload may be a dict with 'objects' or 'detections', or a list directly
        if isinstance(payload, dict):
            for key in ('objects', 'detections', 'items'):
                if key in payload:
                    return payload[key]
            # fallback: treat dict as single object
            return [payload]
        return payload

    def scene_graph_cb(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f'Failed to parse /scene_graph JSON: {e}')
            return

        objects = self._parse_objects(payload)
        if not isinstance(objects, list):
            self.get_logger().warning('Unexpected /scene_graph structure; expected list or dict')
            return

        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        self.get_logger().debug(f'received /scene_graph with {len(objects)} objects; have_cam_info={self.cam_info is not None}')

        for obj in objects:
            try:
                label = obj.get('label', obj.get('class', 'object'))
                # centroid: support multiple possible keys
                centroid = obj.get('centroid_px') or obj.get('centroid')
                if centroid is None:
                    # sometimes bbox is provided as [x1,y1,x2,y2]
                    bb = obj.get('bbox') or obj.get('box') or obj.get('bbox_xyxy') or obj.get('bbox_xywh')
                    if isinstance(bb, (list, tuple)) and len(bb) >= 4:
                        try:
                            # handle xyxy
                            x1, y1, x2, y2 = float(bb[0]), float(bb[1]), float(bb[2]), float(bb[3])
                            centroid = [(x1 + x2) / 2.0, (y1 + y2) / 2.0]
                        except Exception:
                            centroid = None

                # depth: support several names
                depth_m = obj.get('depth_m') or obj.get('z') or obj.get('distance_m') or obj.get('depth') or None
                # camera-space coords: support several aliases
                camera_xyz = obj.get('camera_xyz') or obj.get('camera_pos') or obj.get('camera_point') or obj.get('camera_coordinates')
                frame_id = obj.get('frame_id') or self.camera_frame or self.camera_frame_fallback

                # compute camera-space xyz
                if camera_xyz and isinstance(camera_xyz, (list, tuple)) and len(camera_xyz) >= 3:
                    x_cam, y_cam, z_cam = float(camera_xyz[0]), float(camera_xyz[1]), float(camera_xyz[2])
                elif centroid and depth_m and self.cam_info:
                    u, v = float(centroid[0]), float(centroid[1])
                    z_cam = float(depth_m)
                    fx = self.cam_info.k[0]
                    fy = self.cam_info.k[4]
                    cx = self.cam_info.k[2]
                    cy = self.cam_info.k[5]
                    x_cam = (u - cx) * z_cam / fx
                    y_cam = (v - cy) * z_cam / fy
                elif centroid and depth_m and (self.default_fx > 0.0 and self.default_fy > 0.0):
                    # fallback: use user-provided default intrinsics if CameraInfo not available
                    u, v = float(centroid[0]), float(centroid[1])
                    z_cam = float(depth_m)
                    fx = self.default_fx
                    fy = self.default_fy
                    cx = self.default_cx
                    cy = self.default_cy
                    x_cam = (u - cx) * z_cam / fx
                    y_cam = (v - cy) * z_cam / fy
                else:
                    # provide a clearer log message with reason and dump object keys for debugging
                    reason = 'no camera_xyz'
                    if not centroid or depth_m is None:
                        reason = 'missing centroid or depth'
                    elif not self.cam_info and not (self.default_fx > 0.0 and self.default_fy > 0.0):
                        reason = 'missing CameraInfo and no default intrinsics'
                    self.get_logger().warning(f"Skipping object '{label}': {reason}")
                    # debug dump of object keys/values (shortened)
                    try:
                        short = {k: (v if isinstance(v, (int, float, str)) else type(v).__name__) for k, v in obj.items()}
                        self.get_logger().debug(f"Skipped object content: {short}")
                    except Exception:
                        pass
                    continue

                pose_cam = PoseStamped()
                pose_cam.header.stamp = stamp
                # Pose is initially computed in the source frame (object/frame_id or camera optical frame)
                pose_cam.header.frame_id = frame_id
                pose_cam.pose.position.x = float(x_cam)
                pose_cam.pose.position.y = float(y_cam)
                pose_cam.pose.position.z = float(z_cam)
                pose_cam.pose.orientation.w = 1.0

                # transform to map frame
                # Before mapping to the target frame, prefer to express the pose in
                # the configured camera reference frame. For example, mapping from
                # 'camera_color_optical_frame' to 'camera_link' ensures the same
                # canonical camera frame is used for all detections.
                if self.camera_reference_frame and pose_cam.header.frame_id != self.camera_reference_frame:
                        t_camref = self.tf_buffer.lookup_transform(self.camera_reference_frame, pose_cam.header.frame_id, rclpy.time.Time())
                        try:
                            pose_cam = self._do_transform_pose_robust(pose_cam, t_camref)
                            pose_cam.header.frame_id = self.camera_reference_frame
                            self.get_logger().debug(f'Transformed pose from {frame_id} to camera reference {self.camera_reference_frame}')
                        except Exception as e:
                            # fallback to translation-only (rotation ignored) if tf2_geometry_msgs not installed
                            self.get_logger().warning(f'Could not import tf2_geometry_msgs.do_transform_pose while converting to camera reference: {e}')
                            pose_cam.header.frame_id = self.camera_reference_frame
                            pose_cam.pose.position.x += t_camref.transform.translation.x
                            pose_cam.pose.position.y += t_camref.transform.translation.y
                            pose_cam.pose.position.z += t_camref.transform.translation.z

                try:
                    source_frame_for_map = pose_cam.header.frame_id if pose_cam.header.frame_id else frame_id
                    t = self.tf_buffer.lookup_transform(self.target_frame, source_frame_for_map, rclpy.time.Time())
                    # try to use tf2_geometry_msgs.do_transform_pose if available
                    try:
                        pose_map = self._do_transform_pose_robust(pose_cam, t)
                        self.get_logger().debug(f'Transformed {label} from {source_frame_for_map} to {self.target_frame}')
                    except Exception as e:
                        # If do_transform_pose is not available, fall back to applying translation only
                        # NOTE: This fallback does NOT apply rotation! You need tf2_geometry_msgs installed.
                        self.get_logger().warning(f'tf2_geometry_msgs not available, using translation-only transform (rotation ignored!): {e}')
                        pose_map = pose_cam
                        pose_map.header.frame_id = self.target_frame
                        pose_map.pose.position.x += t.transform.translation.x
                        pose_map.pose.position.y += t.transform.translation.y
                        pose_map.pose.position.z += t.transform.translation.z
                except Exception as ex:
                    self.get_logger().warning(f'Could not transform from {frame_id} to {self.target_frame}: {ex}')
                    continue

                # make marker
                m = Marker()
                m.header.frame_id = self.target_frame
                m.header.stamp = stamp
                m.ns = 'scene_graph_objects'
                m.id = int(obj.get('id', self._marker_id_seq))
                m.type = Marker.SPHERE
                m.action = Marker.ADD
                m.pose = pose_map.pose
                m.scale.x = 0.15
                m.scale.y = 0.15
                m.scale.z = 0.15
                # color by label (simple heuristic)
                if isinstance(label, str) and label.lower() in ('person', 'human'):
                    m.color.r = 1.0
                    m.color.g = 0.2
                    m.color.b = 0.2
                else:
                    m.color.r = 0.0
                    m.color.g = 0.7
                    m.color.b = 0.2
                m.color.a = 0.9

                text = Marker()
                text.header = m.header
                text.ns = 'scene_graph_labels'
                text.id = 100000 + m.id
                text.type = Marker.TEXT_VIEW_FACING
                text.action = Marker.ADD
                text.pose = m.pose
                text.pose.position.z += 0.2
                text.scale.z = 0.12
                text.color.r = 1.0
                text.color.g = 1.0
                text.color.b = 1.0
                text.color.a = 0.9
                text.text = str(label)

                markers.markers.append(m)
                markers.markers.append(text)

                self._marker_id_seq += 1
            except Exception as e:
                self.get_logger().error(f'Error processing object in scene_graph: {e}')

        if markers.markers:
            self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = SceneGraphMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

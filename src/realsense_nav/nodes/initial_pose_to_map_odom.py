#!/usr/bin/env python3
"""
Listen for RViz /initialpose messages and publish a map->odom transform accordingly.

Behavior:
- Subscribe to /initialpose (geometry_msgs/PoseWithCovarianceStamped).
- When received, lookup current transform odom->base_link (via TF).
- Compute T_map_odom = T_map_base_desired * inv(T_odom_base_current).
- Broadcast T_map_odom continuously so the robot's pose in the map frame
  reflects the visual odometry (odom->base_link) but anchored to the map
  pose chosen in RViz.

This allows using RViz's 2D Pose Estimate tool to set the initial location
of the robot in the map frame while keeping visual odometry as the source of
motion (published as odom->base_link).
"""
import math
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
import tf2_ros
import rclpy.time


def quat_mul(q1, q2):
    # q = [x,y,z,w]
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    return (x, y, z, w)


def quat_conjugate(q):
    x, y, z, w = q
    return (-x, -y, -z, w)


def rotate_vector(q, v):
    # rotate vector v by quaternion q
    qv = (v[0], v[1], v[2], 0.0)
    q_conj = quat_conjugate(q)
    t = quat_mul(q, qv)
    t = quat_mul(t, q_conj)
    return (t[0], t[1], t[2])


def invert_transform(translation, rotation):
    # rotation is quaternion (x,y,z,w)
    q_inv = quat_conjugate(rotation)
    t_inv = rotate_vector(q_inv, (-translation[0], -translation[1], -translation[2]))
    return t_inv, q_inv


def multiply_transforms(t1, q1, t2, q2):
    # T = T1 * T2
    # rotate t2 by q1, then add t1
    rt2 = rotate_vector(q1, t2)
    t = (t1[0] + rt2[0], t1[1] + rt2[1], t1[2] + rt2[2])
    q = quat_mul(q1, q2)
    return t, q


class InitialPoseToMapOdom(Node):
    def __init__(self):
        super().__init__('initial_pose_to_map_odom')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.broadcaster = tf2_ros.TransformBroadcaster(self)

        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.initial_pose_cb,
            10,
        )

        self.last_map_to_odom = None
        # Re-broadcast at 10 Hz so transform remains available
        self.timer = self.create_timer(0.1, self.timer_cb)

        self.get_logger().info('initial_pose_to_map_odom ready, listening for /initialpose')

    def initial_pose_cb(self, msg: PoseWithCovarianceStamped):
        # Desired base_link pose in map frame (RViz publishes this with header.frame_id 'map')
        desired = msg.pose.pose
        map_frame = msg.header.frame_id if msg.header.frame_id else 'map'
        # Try to detect which frame the odometry is publishing the robot pose under.
        # Some setups publish odom->camera_link (VO directly on camera), others publish odom->base_link.
        # Prefer 'camera_link' if available, otherwise fall back to 'base_link'.
        base_frame = None
        odom_frame = 'odom'

        # attempt to see if odom->camera_link exists
        try:
            _ = self.tf_buffer.lookup_transform(
                odom_frame,
                'camera_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5),
            )
            base_frame = 'camera_link'
        except Exception:
            # fall back to base_link
            try:
                _ = self.tf_buffer.lookup_transform(
                    odom_frame,
                    'base_link',
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5),
                )
                base_frame = 'base_link'
            except Exception as e:
                self.get_logger().error(f'Could not determine base frame (tried camera_link and base_link): {e}')
                return

        # Build desired transform T_map_base
        t_map_base = (desired.position.x, desired.position.y, desired.position.z)
        q_map_base = (desired.orientation.x, desired.orientation.y, desired.orientation.z, desired.orientation.w)

        try:
            # lookup latest odom -> base_link
            # target_frame, source_frame
            trans = self.tf_buffer.lookup_transform(
                odom_frame,
                base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
        except Exception as e:
            self.get_logger().error(f'Failed to lookup transform {odom_frame} -> {base_frame}: {e}')
            return

        t_odom_base = (
            trans.transform.translation.x,
            trans.transform.translation.y,
            trans.transform.translation.z,
        )
        q_odom_base = (
            trans.transform.rotation.x,
            trans.transform.rotation.y,
            trans.transform.rotation.z,
            trans.transform.rotation.w,
        )

        # compute inv(T_odom_base)
        t_inv, q_inv = invert_transform(t_odom_base, q_odom_base)

        # T_map_odom = T_map_base * inv(T_odom_base)
        t_map_odom, q_map_odom = multiply_transforms(t_map_base, q_map_base, t_inv, q_inv)

        ts = TransformStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id = map_frame
        ts.child_frame_id = odom_frame
        ts.transform.translation.x = float(t_map_odom[0])
        ts.transform.translation.y = float(t_map_odom[1])
        ts.transform.translation.z = float(t_map_odom[2])
        ts.transform.rotation.x = float(q_map_odom[0])
        ts.transform.rotation.y = float(q_map_odom[1])
        ts.transform.rotation.z = float(q_map_odom[2])
        ts.transform.rotation.w = float(q_map_odom[3])

        self.last_map_to_odom = ts
        self.get_logger().info(f'Set map->odom transform from /initialpose (using odom->{base_frame} to compute)')

    def timer_cb(self):
        if self.last_map_to_odom is not None:
            # update stamp and broadcast
            self.last_map_to_odom.header.stamp = self.get_clock().now().to_msg()
            self.broadcaster.sendTransform(self.last_map_to_odom)


def main(args=None):
    rclpy.init(args=args)
    node = InitialPoseToMapOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
Topological Planner (prototype)

Subscribes to /topo_goal (JSON) and converts successful targets into an
image-space centroid goal published on /navigation/goal_centroid (geometry_msgs/Point)
which can be consumed by a visual servo controller.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point
import json


class TopoPlanner(Node):
    def __init__(self):
        super().__init__('topo_planner')
        self.sub = self.create_subscription(String, '/topo_goal', self.cb, 10)
        self.goal_pub = self.create_publisher(Point, '/navigation/goal_centroid', 10)
        self.get_logger().info('TopoPlanner ready')

    def cb(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            self.get_logger().warn('Invalid topo_goal payload')
            return

        if not data.get('success'):
            self.get_logger().info(f"TopoPlanner: no goal resolved: {data.get('reason')}")
            return

        target = data.get('target')
        if target is None:
            return

        cx, cy = target.get('centroid_px', [None, None])
        if cx is None:
            return

        p = Point()
        p.x = float(cx)
        p.y = float(cy)
        p.z = float(target.get('depth_m')) if target.get('depth_m') is not None else 0.0
        self.goal_pub.publish(p)
        self.get_logger().info(f'Published visual goal centroid {cx},{cy}')


def main(args=None):
    rclpy.init(args=args)
    node = TopoPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

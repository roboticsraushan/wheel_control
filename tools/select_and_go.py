#!/usr/bin/env python3
"""
Interactive helper: list objects from /scene_graph and command the robot to go there.

Usage: run while ROS2 is running and the realsense_nav nodes are publishing.
 - Lists detected objects (id, label, centroid, depth)
 - Prompts to pick an object index or id
 - Publishes a /topo_goal JSON (success + target) so TopoPlanner turns it into a visual goal
 - Optionally publishes a natural language command on /nl_goal

This is a small convenience tool for testing and teleoperation.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import threading
import time
import sys


class SelectAndGo(Node):
    def __init__(self, timeout=3.0):
        super().__init__('select_and_go_cli')
        self.sg = None
        self._ready = threading.Event()
        self.sub = self.create_subscription(String, '/scene_graph', self.sg_cb, 10)
        self.topo_pub = self.create_publisher(String, '/topo_goal', 10)
        self.nl_pub = self.create_publisher(String, '/nl_goal', 10)
        self.timeout = timeout

    def sg_cb(self, msg: String):
        try:
            self.sg = json.loads(msg.data)
            self._ready.set()
        except Exception:
            self.get_logger().warning('received invalid scene_graph JSON')

    def wait_for_scene(self):
        # wait until we have a scene graph or until timeout
        self.get_logger().info(f'waiting up to {self.timeout}s for /scene_graph...')
        if not self._ready.wait(timeout=self.timeout):
            self.get_logger().warning('timed out waiting for scene graph')
        return self.sg

    def list_objects(self):
        sg = self.sg
        if not sg or not sg.get('objects'):
            print('No objects visible in the scene graph.')
            return []

        objs = sg.get('objects')
        print('\nDetected objects:')
        for idx, o in enumerate(objs):
            oid = o.get('id')
            label = o.get('label', '<none>')
            area = o.get('area', 0)
            depth = o.get('depth_m', None)
            centroid = o.get('centroid_px', None)
            bbox = o.get('bbox', None)
            print(f'[{idx}] id={oid} label={label} depth={depth} centroid={centroid} bbox={bbox} area={area}')
        return objs

    def publish_topo_goal(self, target_obj):
        data = {'success': True, 'target': target_obj}
        msg = String()
        msg.data = json.dumps(data)
        self.topo_pub.publish(msg)
        self.get_logger().info(f'Published /topo_goal for id={target_obj.get("id")}')

    def publish_nl_goal(self, text):
        msg = String()
        msg.data = text
        self.nl_pub.publish(msg)
        self.get_logger().info(f'Published /nl_goal: "{text}"')


def main(argv=None):
    rclpy.init(args=argv)
    node = SelectAndGo(timeout=3.0)
    try:
        node.wait_for_scene()
        objs = node.list_objects()
        if not objs:
            return 0

        # prompt user
        try:
            choice = input('\nEnter object index (or id) to go to, or label name, or q to quit: ').strip()
        except EOFError:
            return 0

        if not choice or choice.lower() == 'q':
            return 0

        # try index first
        target = None
        if choice.isdigit():
            idx = int(choice)
            # first try index into list
            if 0 <= idx < len(objs):
                target = objs[idx]
            else:
                # try matching id field
                for o in objs:
                    if o.get('id') == idx:
                        target = o
                        break

        if target is None:
            # match by label (first match)
            for o in objs:
                if str(o.get('label', '')).lower() == choice.lower():
                    target = o
                    break

        if target is None:
            print('No matching object found for selection.')
            return 1

        # Ask how to send command
        print(f"Selected target: id={target.get('id')} label={target.get('label')}")
        method = input('Send as (1) /topo_goal JSON or (2) natural-language /nl_goal? [1/2]: ').strip()
        if method == '2':
            # simple NL phrase: 'object <id>' or 'go to <label>'
            if target.get('label'):
                text = f'go to {target.get("label")}'
            else:
                text = f'object {target.get("id")} '
            node.publish_nl_goal(text)
        else:
            node.publish_topo_goal(target)

        # give ROS a moment to publish
        time.sleep(0.2)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())

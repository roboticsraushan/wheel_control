#!/usr/bin/env python3
"""
NL Interpreter (prototype)

Listens for natural language goals on /nl_goal (std_msgs/String). It
performs a lightweight keyword match against the latest scene graph and
publishes a resolved target (centroid) on /topo_goal as JSON string.
This is intentionally simple: exact matching on attributes is used.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import re


class NLInterpreter(Node):
    def __init__(self):
        super().__init__('nl_interpreter')
        self.scene_graph = None
        self.sg_sub = self.create_subscription(String, '/scene_graph', self.sg_cb, 10)
        self.nl_sub = self.create_subscription(String, '/nl_goal', self.nl_cb, 10)
        self.topo_pub = self.create_publisher(String, '/topo_goal', 10)
        self.get_logger().info('NL Interpreter ready')

    def sg_cb(self, msg):
        try:
            self.scene_graph = json.loads(msg.data)
        except Exception:
            self.scene_graph = None

    def nl_cb(self, msg):
        text = msg.data.lower()
        res = {'success': False, 'reason': 'no match'}

        if self.scene_graph is None:
            res['reason'] = 'no scene graph'
            self.topo_pub.publish(String(data=json.dumps(res)))
            return

        # Very simple: if user says a number like 'object 3' we pick that id
        m = re.search(r'object\s*(\d+)', text)
        if m:
            oid = int(m.group(1))
            for o in self.scene_graph.get('objects', []):
                if o.get('id') == oid:
                    res = {'success': True, 'target': o}
                    self.topo_pub.publish(String(data=json.dumps(res)))
                    return

        # Otherwise try attributes like 'nearest' or 'biggest'
        if 'big' in text or 'largest' in text:
            objs = self.scene_graph.get('objects', [])
            if objs:
                best = max(objs, key=lambda o: o.get('area', 0))
                res = {'success': True, 'target': best}
                self.topo_pub.publish(String(data=json.dumps(res)))
                return

        # Fallback: return the first object
        objs = self.scene_graph.get('objects', [])
        if objs:
            res = {'success': True, 'target': objs[0]}
        else:
            res['reason'] = 'no objects'

        self.topo_pub.publish(String(data=json.dumps(res)))


def main(args=None):
    rclpy.init(args=args)
    node = NLInterpreter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

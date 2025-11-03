#!/usr/bin/env python3
"""
Graph Path Planner

Plans paths through junction graph using A* algorithm.
Converts node-to-node navigation into junction sequence.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from realsense_nav.msg import JunctionPath
from geometry_msgs.msg import Pose
import json
import heapq


class GraphPathPlannerNode(Node):
    def __init__(self):
        super().__init__('graph_path_planner')
        
        self.declare_parameter('map_file', '/home/raushan/control_one/wheel_control/data/maps/latest_map.json')
        
        self.junctions = []
        self.edges = []
        self.nodes = []
        self.junction_graph = {}
        
        self._load_map()
        
        self.create_subscription(String, '/navigation/task', self.task_callback, 10)
        self.path_pub = self.create_publisher(JunctionPath, '/navigation/junction_path', 10)
        
        self.get_logger().info('Graph Path Planner ready')
    
    def _load_map(self):
        """Load map from file"""
        map_file = self.get_parameter('map_file').value
        try:
            with open(map_file, 'r') as f:
                map_data = json.load(f)
            
            self.junctions = map_data.get('junctions', [])
            self.edges = map_data.get('edges', [])
            self.nodes = map_data.get('nodes', [])
            
            # Build adjacency list
            for edge in self.edges:
                if edge['from'] not in self.junction_graph:
                    self.junction_graph[edge['from']] = []
                if edge['to'] not in self.junction_graph:
                    self.junction_graph[edge['to']] = []
                
                self.junction_graph[edge['from']].append((edge['to'], edge['distance']))
                self.junction_graph[edge['to']].append((edge['from'], edge['distance']))
            
            self.get_logger().info(f'Map loaded: {len(self.junctions)} junctions')
        except Exception as e:
            self.get_logger().error(f'Failed to load map: {e}')
    
    def task_callback(self, msg):
        """Process navigation task"""
        try:
            task = json.loads(msg.data)
            dest_node = task.get('destination_node')
            
            # Find junction for destination node
            dest_junction = self._find_node_junction(dest_node)
            if not dest_junction:
                self.get_logger().error(f'Destination node {dest_node} not found')
                return
            
            # For now, assume current position is first junction
            # In real system, get from localization manager
            if not self.junctions:
                self.get_logger().error('No junctions available')
                return
            
            start_junction = self.junctions[0]['id']
            
            # Plan path
            path = self._plan_path(start_junction, dest_junction)
            if not path:
                self.get_logger().error(f'No path found from {start_junction} to {dest_junction}')
                return
            
            # Publish path
            self._publish_path(task['task_id'], path)
            
        except Exception as e:
            self.get_logger().error(f'Task processing failed: {e}')
    
    def _find_node_junction(self, node_name):
        """Find which junction contains a node"""
        for node in self.nodes:
            if node_name.lower() in node['id'].lower():
                return node['junction_id']
        return None
    
    def _plan_path(self, start, goal):
        """A* path planning"""
        if start not in self.junction_graph or goal not in self.junction_graph:
            return None
        
        frontier = [(0, start, [start])]
        visited = set()
        
        while frontier:
            cost, current, path = heapq.heappop(frontier)
            
            if current == goal:
                return path
            
            if current in visited:
                continue
            
            visited.add(current)
            
            for neighbor, distance in self.junction_graph.get(current, []):
                if neighbor not in visited:
                    new_path = path + [neighbor]
                    new_cost = cost + distance
                    heapq.heappush(frontier, (new_cost, neighbor, new_path))
        
        return None
    
    def _publish_path(self, task_id, path):
        """Publish junction path"""
        msg = JunctionPath()
        msg.task_id = task_id
        msg.junction_ids = path
        msg.num_junctions = len(path)
        
        # Get poses
        junction_dict = {j['id']: j for j in self.junctions}
        for jid in path:
            if jid in junction_dict:
                pose = Pose()
                pose.position.x = junction_dict[jid]['pose']['x']
                pose.position.y = junction_dict[jid]['pose']['y']
                pose.position.z = junction_dict[jid]['pose']['z']
                msg.junction_poses.append(pose)
        
        # Compute total distance
        total_dist = 0.0
        for i in range(len(path) - 1):
            for edge in self.edges:
                if (edge['from'] == path[i] and edge['to'] == path[i+1]) or \
                   (edge['to'] == path[i] and edge['from'] == path[i+1]):
                    total_dist += edge['distance']
        
        msg.total_distance = total_dist
        
        self.path_pub.publish(msg)
        self.get_logger().info(f'Path planned: {" → ".join(path)} ({total_dist:.2f}m)')


def main(args=None):
    rclpy.init(args=args)
    node = GraphPathPlannerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

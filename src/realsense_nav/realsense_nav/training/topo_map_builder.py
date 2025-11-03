#!/usr/bin/env python3
"""
Topological Map Builder Node

Builds navigation graph from recorded junctions.
Loads junction data, computes edges, validates connectivity.
"""
import rclpy
from rclpy.node import Node
from realsense_nav.srv import LoadMap
from std_msgs.msg import String
import json
import os
import numpy as np


class TopoMapBuilderNode(Node):
    def __init__(self):
        super().__init__('topo_map_builder')
        
        self.declare_parameter('data_dir', '/home/raushan/control_one/wheel_control/data/junctions')
        self.declare_parameter('map_dir', '/home/raushan/control_one/wheel_control/data/maps')
        
        self.data_dir = self.get_parameter('data_dir').value
        self.map_dir = self.get_parameter('map_dir').value
        
        self.junctions = []
        self.edges = []
        self.nodes = []
        
        # Service
        self.load_srv = self.create_service(LoadMap, '~/load_map', self.load_map_service)
        
        # Auto-load on startup
        self._load_and_build()
        
        self.get_logger().info('Topological Map Builder initialized')
    
    def _load_and_build(self):
        """Load junctions and build graph"""
        junction_file = os.path.join(self.data_dir, 'junctions.json')
        if not os.path.exists(junction_file):
            self.get_logger().warn('No junctions file found')
            return
        
        with open(junction_file, 'r') as f:
            self.junctions = json.load(f)
        
        self._build_edges()
        self._extract_nodes()
        
        self.get_logger().info(f'Loaded {len(self.junctions)} junctions, {len(self.edges)} edges, {len(self.nodes)} nodes')
    
    def _build_edges(self):
        """Build edges between consecutive junctions"""
        self.edges = []
        for i in range(len(self.junctions) - 1):
            j1 = self.junctions[i]
            j2 = self.junctions[i + 1]
            
            dx = j2['pose']['x'] - j1['pose']['x']
            dy = j2['pose']['y'] - j1['pose']['y']
            distance = float(np.sqrt(dx**2 + dy**2))
            
            self.edges.append({
                'from': j1['id'],
                'to': j2['id'],
                'distance': distance
            })
    
    def _extract_nodes(self):
        """Extract unique nodes from all junctions"""
        node_dict = {}
        for junction in self.junctions:
            sg = junction.get('scene_graph')
            if not sg:
                continue
            
            objects = sg.get('objects', [])
            for obj in objects:
                label = obj.get('label', 'unknown')
                node_id = f"{label}_{junction['id']}"
                
                if node_id not in node_dict:
                    node_dict[node_id] = {
                        'id': node_id,
                        'type': label,
                        'junction_id': junction['id'],
                        'bbox': obj.get('bbox'),
                        'centroid': obj.get('centroid_px'),
                        'depth': obj.get('depth_m')
                    }
        
        self.nodes = list(node_dict.values())
    
    def load_map_service(self, request, response):
        """Load a saved map"""
        filename = request.filename if request.filename else 'latest_map.json'
        filepath = os.path.join(self.map_dir, filename)
        
        if not os.path.exists(filepath):
            response.success = False
            response.message = f'Map file not found: {filepath}'
            return response
        
        try:
            with open(filepath, 'r') as f:
                map_data = json.load(f)
            
            self.junctions = map_data.get('junctions', [])
            self.edges = map_data.get('edges', [])
            self.nodes = map_data.get('nodes', [])
            
            response.success = True
            response.filepath = filepath
            response.num_junctions = len(self.junctions)
            response.num_nodes = len(self.nodes)
            response.message = 'Map loaded successfully'
            
            self.get_logger().info(f'Map loaded from {filepath}')
            
        except Exception as e:
            response.success = False
            response.message = f'Failed to load map: {e}'
        
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TopoMapBuilderNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

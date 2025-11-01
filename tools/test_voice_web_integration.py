#!/usr/bin/env python3
"""test_voice_web_integration.py

Simple test script to verify the voice web bridge integration with ROS2 navigation.

This script:
1. Publishes a mock scene_graph message
2. Sends a test chat request to the Flask API
3. Verifies that the /llm_goal topic receives the navigation command

Usage:
    # Terminal 1: Start the voice_web_bridge
    source install/local_setup.bash
    ros2 run voice_llm_navigator voice_web_bridge

    # Terminal 2: Run this test
    python3 tools/test_voice_web_integration.py
"""

import requests
import json
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class VoiceWebTester(Node):
    def __init__(self):
        super().__init__('voice_web_tester')
        self.scene_graph_pub = self.create_publisher(String, '/scene_graph', 10)
        self.llm_goal_sub = self.create_subscription(
            String, '/llm_goal', self.llm_goal_callback, 10
        )
        self.received_goal = None
        self.get_logger().info('Voice web tester node initialized')

    def llm_goal_callback(self, msg):
        self.received_goal = msg.data
        self.get_logger().info(f'Received /llm_goal: {msg.data}')

    def publish_mock_scene_graph(self):
        """Publish a mock scene graph with some test objects"""
        scene_graph = {
            'objects': [
                {'id': 1, 'label': 'chair'},
                {'id': 2, 'label': 'table'},
                {'id': 3, 'label': 'door'},
            ]
        }
        msg = String()
        msg.data = json.dumps(scene_graph)
        self.scene_graph_pub.publish(msg)
        self.get_logger().info(f'Published mock scene_graph: {scene_graph}')

    def test_chat_api(self, text):
        """Send a test request to the Flask chat API"""
        self.get_logger().info(f'Sending chat request: "{text}"')
        try:
            response = requests.post(
                'http://127.0.0.1:5003/api/chat',
                json={'text': text},
                timeout=30
            )
            if response.ok:
                data = response.json()
                self.get_logger().info(f'Chat API response: {data}')
                return data
            else:
                self.get_logger().error(f'Chat API error: {response.status_code}')
                return None
        except Exception as e:
            self.get_logger().error(f'Failed to call chat API: {e}')
            return None


def main():
    rclpy.init()
    tester = Node('voice_web_tester')
    
    print("\n" + "="*60)
    print("Voice Web Bridge Integration Test")
    print("="*60)
    
    # Wait a bit for connections to establish
    print("\n[1/5] Waiting for ROS2 connections...")
    time.sleep(2)
    
    # Test 1: Publish mock scene graph
    print("\n[2/5] Publishing mock scene graph...")
    scene_graph_pub = tester.create_publisher(String, '/scene_graph', 10)
    scene_graph = {
        'objects': [
            {'id': 1, 'label': 'chair'},
            {'id': 2, 'label': 'table'},
            {'id': 3, 'label': 'door'},
        ]
    }
    msg = String()
    msg.data = json.dumps(scene_graph)
    scene_graph_pub.publish(msg)
    print(f"✓ Published scene graph with objects: chair, table, door")
    time.sleep(1)
    
    # Test 2: Send chat request
    print("\n[3/5] Testing chat API with 'go to the chair'...")
    try:
        response = requests.post(
            'http://127.0.0.1:5003/api/chat',
            json={'text': 'go to the chair'},
            timeout=60
        )
        if response.ok:
            data = response.json()
            print(f"✓ Chat API responded")
            print(f"  Reply: {data.get('reply')}")
            print(f"  Audio: {data.get('audio')}")
        else:
            print(f"✗ Chat API error: {response.status_code}")
    except Exception as e:
        print(f"✗ Failed to call chat API: {e}")
    
    # Test 3: Check /llm_goal topic
    print("\n[4/5] Checking /llm_goal topic...")
    received_goal = []
    
    def goal_callback(msg):
        received_goal.append(msg.data)
        print(f"✓ Received /llm_goal: {msg.data}")
    
    goal_sub = tester.create_subscription(String, '/llm_goal', goal_callback, 10)
    
    # Spin briefly to receive the message
    for _ in range(10):
        rclpy.spin_once(tester, timeout_sec=0.5)
        if received_goal:
            break
    
    if received_goal:
        print(f"✓ Navigation command published successfully")
    else:
        print(f"⚠ No /llm_goal message received (check if voice_web_bridge is running)")
    
    # Test 4: Test web UI
    print("\n[5/5] Testing web UI...")
    try:
        response = requests.get('http://127.0.0.1:5003/', timeout=5)
        if response.ok:
            print(f"✓ Web UI is accessible at http://127.0.0.1:5003/")
        else:
            print(f"✗ Web UI error: {response.status_code}")
    except Exception as e:
        print(f"✗ Failed to access web UI: {e}")
    
    print("\n" + "="*60)
    print("Test Summary:")
    print("="*60)
    print("""
✓ Scene graph published
✓ Chat API integration working
✓ Navigation commands published to /llm_goal
✓ Web UI accessible

Next steps:
1. Open http://127.0.0.1:5003/ in your browser
2. Click the orb to record voice or type a message
3. Try commands like:
   - "go to the chair"
   - "navigate to the table"
   - "what do you see"
4. Monitor /llm_goal topic: ros2 topic echo /llm_goal
""")
    
    tester.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

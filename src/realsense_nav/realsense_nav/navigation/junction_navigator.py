#!/usr/bin/env python3
"""
Junction Navigator

Navigates through sequence of junctions.
Monitors junction recognition and triggers movement to next junction.
"""
import rclpy
from rclpy.node import Node
from realsense_nav.msg import JunctionPath, JunctionDetection
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import json


class JunctionNavigatorNode(Node):
    def __init__(self):
        super().__init__('junction_navigator')
        
        self.declare_parameter('arrival_threshold', 0.8)
        self.declare_parameter('linear_vel', 0.3)
        self.declare_parameter('angular_vel', 0.5)
        
        self.arrival_threshold = self.get_parameter('arrival_threshold').value
        self.linear_vel = self.get_parameter('linear_vel').value
        self.angular_vel = self.get_parameter('angular_vel').value
        
        self.junction_path = []
        self.current_index = 0
        self.navigating = False
        self.current_confidence = 0.0
        
        self.create_subscription(JunctionPath, '/navigation/junction_path', self.path_callback, 10)
        self.create_subscription(JunctionDetection, '/junction/detected', self.detection_callback, 10)
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.expected_pub = self.create_publisher(String, '/navigation/expected_junction', 10)
        self.status_pub = self.create_publisher(String, '/navigation/status', 10)
        
        self.create_timer(0.1, self.navigation_loop)
        
        self.get_logger().info('Junction Navigator ready')
    
    def path_callback(self, msg):
        """Receive new junction path"""
        self.junction_path = msg.junction_ids
        self.current_index = 0
        self.navigating = True
        
        self.get_logger().info(f'New path received: {" → ".join(self.junction_path)}')
        
        # Publish expected junction
        if self.junction_path:
            self._publish_expected_junction(self.junction_path[self.current_index])
    
    def detection_callback(self, msg):
        """Handle junction detection"""
        if not self.navigating or self.current_index >= len(self.junction_path):
            return
        
        expected = self.junction_path[self.current_index]
        
        if msg.junction_id == expected and msg.is_expected:
            self.current_confidence = msg.confidence
            
            # Check if arrived
            if self.current_confidence >= self.arrival_threshold:
                self._junction_arrived()
    
    def _junction_arrived(self):
        """Handle arrival at junction"""
        arrived_junction = self.junction_path[self.current_index]
        self.get_logger().info(f'✓ Arrived at junction {arrived_junction}')
        
        # Stop robot
        self._stop_robot()
        
        # Move to next junction
        self.current_index += 1
        
        if self.current_index >= len(self.junction_path):
            # Reached final junction
            self.navigating = False
            self._publish_status('arrived', 'Reached destination')
            self.get_logger().info('🎯 Navigation complete!')
        else:
            # Continue to next junction
            next_junction = self.junction_path[self.current_index]
            self._publish_expected_junction(next_junction)
            self._publish_status('navigating', f'Moving to {next_junction}')
    
    def navigation_loop(self):
        """Main navigation control loop"""
        if not self.navigating or self.current_index >= len(self.junction_path):
            return
        
        # Simple forward motion toward junction
        # In real system, integrate with obstacle avoidance
        cmd = Twist()
        cmd.linear.x = self.linear_vel
        cmd.angular.z = 0.0
        
        self.cmd_pub.publish(cmd)
    
    def _stop_robot(self):
        """Stop robot movement"""
        cmd = Twist()
        self.cmd_pub.publish(cmd)
    
    def _publish_expected_junction(self, junction_id):
        """Publish expected junction ID"""
        msg = String()
        msg.data = junction_id
        self.expected_pub.publish(msg)
    
    def _publish_status(self, state, message):
        """Publish navigation status"""
        status = {
            'state': state,
            'message': message,
            'current_index': self.current_index,
            'total_junctions': len(self.junction_path),
            'timestamp': self.get_clock().now().to_msg().sec
        }
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JunctionNavigatorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
Junction Visibility Monitor

Monitors if target junction remains visible during navigation.
Alerts if junction is lost or occluded.
"""
import rclpy
from rclpy.node import Node
from realsense_nav.msg import JunctionDetection
from std_msgs.msg import Bool, String
import json


class JunctionVisibilityMonitorNode(Node):
    def __init__(self):
        super().__init__('junction_visibility_monitor')
        
        self.declare_parameter('visibility_threshold', 0.6)
        self.declare_parameter('lost_timeout', 3.0)
        
        self.visibility_threshold = self.get_parameter('visibility_threshold').value
        self.lost_timeout = self.get_parameter('lost_timeout').value
        
        self.current_confidence = 0.0
        self.expected_junction = None
        self.last_detection_time = None
        self.is_visible = False
        
        self.create_subscription(JunctionDetection, '/junction/detected', self.detection_callback, 10)
        self.create_subscription(String, '/navigation/expected_junction', self.expected_callback, 10)
        
        self.visibility_pub = self.create_publisher(Bool, '/junction/visibility', 10)
        self.alert_pub = self.create_publisher(String, '/navigation/alerts', 10)
        
        self.create_timer(0.5, self.check_visibility)
        
        self.get_logger().info('Junction Visibility Monitor started')
    
    def detection_callback(self, msg):
        """Update from junction detection"""
        if self.expected_junction and msg.junction_id == self.expected_junction:
            self.current_confidence = msg.confidence
            self.last_detection_time = self.get_clock().now()
    
    def expected_callback(self, msg):
        """Set expected junction"""
        self.expected_junction = msg.data
        self.current_confidence = 0.0
        self.last_detection_time = None
        self.get_logger().info(f'Monitoring visibility of {self.expected_junction}')
    
    def check_visibility(self):
        """Check if junction is still visible"""
        if not self.expected_junction:
            return
        
        was_visible = self.is_visible
        
        # Check confidence threshold
        if self.current_confidence >= self.visibility_threshold:
            self.is_visible = True
        else:
            # Check timeout
            if self.last_detection_time:
                elapsed = (self.get_clock().now() - self.last_detection_time).nanoseconds / 1e9
                if elapsed > self.lost_timeout:
                    self.is_visible = False
            else:
                self.is_visible = False
        
        # Publish visibility status
        msg = Bool()
        msg.data = self.is_visible
        self.visibility_pub.publish(msg)
        
        # Alert on state change
        if was_visible and not self.is_visible:
            self._publish_alert('warning', f'Junction {self.expected_junction} lost from view!')
            self.get_logger().warn(f'⚠️ Junction {self.expected_junction} lost!')
        elif not was_visible and self.is_visible:
            self._publish_alert('info', f'Junction {self.expected_junction} is visible')
            self.get_logger().info(f'✓ Junction {self.expected_junction} visible')
    
    def _publish_alert(self, level, message):
        """Publish navigation alert"""
        alert = {
            'level': level,
            'message': message,
            'junction': self.expected_junction,
            'confidence': float(self.current_confidence),
            'timestamp': self.get_clock().now().to_msg().sec
        }
        
        msg = String()
        msg.data = json.dumps(alert)
        self.alert_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JunctionVisibilityMonitorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

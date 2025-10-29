#!/usr/bin/env python3
"""
Behavior Tree Controller Node
Executes predefined behavior tree for autonomous navigation
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped
from std_msgs.msg import Bool
from .behavior_tree import (
    BehaviorStatus,
    SequenceNode,
    GoToYellowCone,
    TurnDegrees,
    Wait,
    RepeatSequence
)


class BehaviorTreeController(Node):
    def __init__(self):
        super().__init__('behavior_tree_controller')
        
        # State from subscriptions
        self.goal_position = None
        self.goal_detected = False
        
        # Subscribers
        self.goal_sub = self.create_subscription(
            PointStamped,
            '/goal/position',
            self.goal_callback,
            10
        )
        
        self.goal_detected_sub = self.create_subscription(
            Bool,
            '/goal/detected',
            self.goal_detected_callback,
            10
        )
        
        # Publisher
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # Build the behavior tree
        self.behavior_tree = self.build_behavior_tree()
        
        # Control timer (20 Hz)
        self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info('Behavior Tree Controller started')
        self.log_behavior_tree()
    
    def goal_callback(self, msg):
        self.goal_position = msg.point
    
    def goal_detected_callback(self, msg):
        self.goal_detected = msg.data
    
    def build_behavior_tree(self):
        """
        Define your behavior tree workflow here.
        
        Example workflow:
        1. Go to first yellow cone (stop at 80cm)
        2. Wait 2 seconds
        3. Turn 90 degrees right
        4. Wait 1 second
        5. Go to next yellow cone (stop at 80cm)
        6. Repeat
        """
        
        # Create individual behaviors
        go_to_cone_1 = GoToYellowCone(stop_distance=0.8)
        wait_1 = Wait(duration_seconds=2.0)
        turn_right_90 = TurnDegrees(degrees=-90, angular_speed=0.5)  # Negative = right
        wait_2 = Wait(duration_seconds=1.0)
        go_to_cone_2 = GoToYellowCone(stop_distance=1.0)
        wait_3 = Wait(duration_seconds=2.0)
        turn_right_90_again = TurnDegrees(degrees=-90, angular_speed=0.5)
        
        # Create sequence
        sequence = SequenceNode(
            name="ConeNavigationSequence",
            children=[
                go_to_cone_1,
                wait_1,
                turn_right_90,
                wait_2,
                go_to_cone_2,
                wait_3,
                turn_right_90_again
            ]
        )
        
        # Optionally wrap in RepeatSequence to loop
        # return RepeatSequence("InfiniteConeLoop", sequence, repeat_count=None)
        
        # Or run once
        return sequence
    
    def log_behavior_tree(self):
        """Log the behavior tree structure"""
        self.get_logger().info("=" * 60)
        self.get_logger().info("Behavior Tree Workflow:")
        self.get_logger().info("=" * 60)
        
        def log_node(node, indent=0):
            prefix = "  " * indent
            self.get_logger().info(f"{prefix}→ {node.name}")
            if isinstance(node, (SequenceNode, RepeatSequence)):
                if hasattr(node, 'children'):
                    for child in node.children:
                        log_node(child, indent + 1)
                elif hasattr(node, 'child_sequence'):
                    log_node(node.child_sequence, indent + 1)
        
        log_node(self.behavior_tree)
        self.get_logger().info("=" * 60)
    
    def control_loop(self):
        """Execute behavior tree"""
        status = self.behavior_tree.tick(self)
        
        if status == BehaviorStatus.SUCCESS:
            self.get_logger().info("Behavior tree completed successfully!")
            # Stop robot
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)
            
            # Optionally restart or shutdown
            # self.behavior_tree.reset()
            # rclpy.shutdown()
        
        elif status == BehaviorStatus.FAILURE:
            self.get_logger().error("Behavior tree failed!")
            # Stop robot
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = BehaviorTreeController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop robot on shutdown
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        node.cmd_vel_pub.publish(cmd)
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
Behavior Tree Node (Python)

Pure Python-based behavior tree implementation using py_trees library.
Replaces the C++ behavior_tree_cpp_node with a lightweight, maintainable Python version.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Twist
import py_trees
from py_trees import behaviour, composites
import json
import logging

logger = logging.getLogger(__name__)


class NavigationBehavior(behaviour.Behaviour):
    """Behavior to navigate to a goal based on scene graph and planning."""

    def __init__(self, name, node):
        super().__init__(name)
        self.node = node
        self.current_task = None

    def setup(self, unused_observers):
        self.node.get_logger().info(f'Setting up {self.name}')
        return True

    def initialise(self):
        self.node.get_logger().debug(f'{self.name} initialised')

    def update(self):
        """Check if navigation task is available and proceed."""
        if self.current_task is None:
            self.node.get_logger().debug(f'{self.name}: no task received')
            return py_trees.common.Status.FAILURE

        # Simulate navigation logic
        self.node.get_logger().info(f'{self.name}: executing task {self.current_task}')
        # In a real scenario, you'd check /robot/pose, /scene_graph, plan a path, etc.
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        self.node.get_logger().debug(f'{self.name}: terminate, new_status={new_status}')


class ObstacleCheckBehavior(behaviour.Behaviour):
    """Behavior to check for obstacles."""

    def __init__(self, name, node):
        super().__init__(name)
        self.node = node
        self.has_obstacle = False

    def setup(self, unused_observers):
        self.node.get_logger().info(f'Setting up {self.name}')
        return True

    def initialise(self):
        self.node.get_logger().debug(f'{self.name} initialised')

    def update(self):
        """Check if obstacle is present."""
        if self.has_obstacle:
            self.node.get_logger().warn(f'{self.name}: obstacle detected!')
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        self.node.get_logger().debug(f'{self.name}: terminate, new_status={new_status}')


class StopMotorsBehavior(behaviour.Behaviour):
    """Behavior to stop motors."""

    def __init__(self, name, node, cmd_vel_pub):
        super().__init__(name)
        self.node = node
        self.cmd_vel_pub = cmd_vel_pub

    def setup(self, unused_observers):
        self.node.get_logger().info(f'Setting up {self.name}')
        return True

    def initialise(self):
        self.node.get_logger().debug(f'{self.name} initialised')

    def update(self):
        """Publish zero velocity to stop motors."""
        msg = Twist()
        self.cmd_vel_pub.publish(msg)
        self.node.get_logger().info(f'{self.name}: motors stopped')
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        self.node.get_logger().debug(f'{self.name}: terminate, new_status={new_status}')


class BehaviorTreeNode(Node):
    """Main Behavior Tree ROS2 Node."""

    def __init__(self):
        super().__init__('behavior_tree_node')
        self.get_logger().info('Initializing Behavior Tree Node')

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/behavior_tree/state', 10)

        # Subscribers
        self.task_sub = self.create_subscription(String, '/navigation/task', self.task_callback, 10)
        self.obstacle_sub = self.create_subscription(String, '/obstacle/detection', self.obstacle_callback, 10)

        # State
        self.current_task = None
        self.has_obstacle = False

        # Build the behavior tree
        self.root = self._build_tree()
        self.tree = py_trees.trees.BehaviourTree(root=self.root)

        # Timer for tree execution
        self.timer = self.create_timer(0.1, self.tick_tree)
        self.get_logger().info('Behavior Tree Node initialized and ready')

    def _build_tree(self):
        """
        Build a simple behavior tree structure:

        Root (Sequence)
        ├── CheckObstacle
        ├── GetNavigationTask
        └── ExecuteNavigation
            └── StopMotors (on failure)
        """
        # Create behaviors
        check_obstacle = ObstacleCheckBehavior('CheckObstacle', self)
        navigate = NavigationBehavior('Navigate', self)
        stop_motors = StopMotorsBehavior('StopMotors', self, self.cmd_vel_pub)

        # Build tree: if obstacle check fails, stop motors; otherwise navigate
        sequence = composites.Sequence(name='MainSequence', memory=False, children=[check_obstacle, navigate])
        selector = composites.Selector(name='MainSelector', memory=False, children=[sequence, stop_motors])

        return selector

    def task_callback(self, msg):
        """Handle incoming navigation task."""
        try:
            task_data = json.loads(msg.data)
            self.current_task = task_data
            self.get_logger().info(f'Received navigation task: {task_data}')
        except Exception as e:
            self.get_logger().warn(f'Failed to parse task: {e}')

    def obstacle_callback(self, msg):
        """Handle obstacle detection."""
        try:
            obstacle_data = json.loads(msg.data)
            self.has_obstacle = obstacle_data.get('detected', False)
            if self.has_obstacle:
                self.get_logger().warn('Obstacle detected - stopping navigation')
            else:
                self.get_logger().debug('No obstacle')
        except Exception as e:
            self.get_logger().warn(f'Failed to parse obstacle msg: {e}')

    def tick_tree(self):
        """Tick the behavior tree periodically."""
        try:
            self.tree.tick()
            # Publish current tree status
            status_msg = String()
            status_msg.data = f'BT Status: {self.root.status}'
            self.state_pub.publish(status_msg)
        except Exception as e:
            self.get_logger().error(f'Error ticking tree: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = BehaviorTreeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

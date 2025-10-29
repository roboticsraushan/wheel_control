#!/usr/bin/env python3
"""
Behavior Tree Framework for Robot Navigation
Supports sequential and parallel execution of behaviors
"""

from enum import Enum
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped
from std_msgs.msg import Bool
import math
import time


class BehaviorStatus(Enum):
    """Status returned by behaviors"""
    SUCCESS = 1
    FAILURE = 2
    RUNNING = 3


class Behavior:
    """Base class for all behaviors"""
    def __init__(self, name):
        self.name = name
        self.status = BehaviorStatus.RUNNING
    
    def tick(self, node):
        """Execute one step of the behavior. Override in subclasses."""
        raise NotImplementedError("Subclass must implement tick()")
    
    def reset(self):
        """Reset behavior state"""
        self.status = BehaviorStatus.RUNNING


class SequenceNode(Behavior):
    """Executes children in sequence. Fails if any child fails."""
    def __init__(self, name, children):
        super().__init__(name)
        self.children = children
        self.current_index = 0
    
    def tick(self, node):
        while self.current_index < len(self.children):
            child = self.children[self.current_index]
            status = child.tick(node)
            
            if status == BehaviorStatus.RUNNING:
                self.status = BehaviorStatus.RUNNING
                return self.status
            elif status == BehaviorStatus.FAILURE:
                self.status = BehaviorStatus.FAILURE
                node.get_logger().info(f"Sequence '{self.name}' failed at child '{child.name}'")
                return self.status
            else:  # SUCCESS
                self.current_index += 1
        
        # All children succeeded
        self.status = BehaviorStatus.SUCCESS
        node.get_logger().info(f"Sequence '{self.name}' completed successfully")
        return self.status
    
    def reset(self):
        super().reset()
        self.current_index = 0
        for child in self.children:
            child.reset()


class GoToYellowCone(Behavior):
    """Navigate to detected yellow cone"""
    def __init__(self, stop_distance=0.8):
        super().__init__("GoToYellowCone")
        self.stop_distance = stop_distance
        self.goal_position = None
        self.goal_detected = False
        self.start_time = None
        self.timeout = 30.0  # 30 seconds timeout
    
    def tick(self, node):
        if self.start_time is None:
            self.start_time = time.time()
            node.get_logger().info(f"Starting behavior: {self.name}")
        
        # Check timeout
        if time.time() - self.start_time > self.timeout:
            node.get_logger().warn(f"{self.name} timed out")
            self.status = BehaviorStatus.FAILURE
            return self.status
        
        # Update goal from node's subscribed data
        self.goal_position = node.goal_position
        self.goal_detected = node.goal_detected
        
        if not self.goal_detected or self.goal_position is None:
            # No goal detected, keep searching
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.3  # Slow rotation to search
            node.cmd_vel_pub.publish(cmd)
            self.status = BehaviorStatus.RUNNING
            return self.status
        
        # Calculate distance to goal
        distance = math.sqrt(
            self.goal_position.x**2 + 
            self.goal_position.y**2 + 
            self.goal_position.z**2
        )
        
        if distance < self.stop_distance:
            # Reached goal, stop
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            node.cmd_vel_pub.publish(cmd)
            node.get_logger().info(f"{self.name} reached goal at {distance:.2f}m")
            self.status = BehaviorStatus.SUCCESS
            return self.status
        
        # Move towards goal using simple proportional control
        angle_to_goal = math.atan2(self.goal_position.y, self.goal_position.x)
        
        cmd = Twist()
        cmd.linear.x = min(0.5, distance * 0.5)  # Proportional speed
        cmd.angular.z = 2.0 * angle_to_goal  # Proportional steering
        
        # Limit velocities
        cmd.linear.x = max(0.0, min(0.5, cmd.linear.x))
        cmd.angular.z = max(-1.0, min(1.0, cmd.angular.z))
        
        node.cmd_vel_pub.publish(cmd)
        
        self.status = BehaviorStatus.RUNNING
        return self.status
    
    def reset(self):
        super().reset()
        self.start_time = None
        self.goal_position = None


class TurnDegrees(Behavior):
    """Turn the robot by a specified number of degrees"""
    def __init__(self, degrees, angular_speed=0.5):
        super().__init__(f"TurnDegrees({degrees}°)")
        self.target_degrees = degrees
        self.target_radians = math.radians(degrees)
        self.angular_speed = angular_speed
        self.start_time = None
        self.timeout = 10.0  # 10 seconds timeout
    
    def tick(self, node):
        if self.start_time is None:
            self.start_time = time.time()
            self.expected_duration = abs(self.target_radians) / self.angular_speed
            node.get_logger().info(
                f"Starting {self.name} - Expected duration: {self.expected_duration:.2f}s"
            )
        
        elapsed = time.time() - self.start_time
        
        # Check if turn is complete (with small buffer)
        if elapsed >= self.expected_duration:
            # Stop rotation
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            node.cmd_vel_pub.publish(cmd)
            
            node.get_logger().info(f"{self.name} completed in {elapsed:.2f}s")
            self.status = BehaviorStatus.SUCCESS
            return self.status
        
        # Check timeout
        if elapsed > self.timeout:
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            node.cmd_vel_pub.publish(cmd)
            
            node.get_logger().warn(f"{self.name} timed out")
            self.status = BehaviorStatus.FAILURE
            return self.status
        
        # Continue turning
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = self.angular_speed if self.target_radians > 0 else -self.angular_speed
        node.cmd_vel_pub.publish(cmd)
        
        self.status = BehaviorStatus.RUNNING
        return self.status
    
    def reset(self):
        super().reset()
        self.start_time = None


class Wait(Behavior):
    """Wait for a specified duration"""
    def __init__(self, duration_seconds):
        super().__init__(f"Wait({duration_seconds}s)")
        self.duration = duration_seconds
        self.start_time = None
    
    def tick(self, node):
        if self.start_time is None:
            self.start_time = time.time()
            node.get_logger().info(f"Starting {self.name}")
            
            # Stop robot
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            node.cmd_vel_pub.publish(cmd)
        
        elapsed = time.time() - self.start_time
        
        if elapsed >= self.duration:
            node.get_logger().info(f"{self.name} completed")
            self.status = BehaviorStatus.SUCCESS
            return self.status
        
        self.status = BehaviorStatus.RUNNING
        return self.status
    
    def reset(self):
        super().reset()
        self.start_time = None


class RepeatSequence(Behavior):
    """Repeat a sequence of behaviors N times or indefinitely"""
    def __init__(self, name, child_sequence, repeat_count=None):
        super().__init__(name)
        self.child_sequence = child_sequence
        self.repeat_count = repeat_count  # None for infinite
        self.current_iteration = 0
    
    def tick(self, node):
        status = self.child_sequence.tick(node)
        
        if status == BehaviorStatus.SUCCESS:
            self.current_iteration += 1
            
            if self.repeat_count is not None and self.current_iteration >= self.repeat_count:
                node.get_logger().info(f"{self.name} completed {self.current_iteration} iterations")
                self.status = BehaviorStatus.SUCCESS
                return self.status
            
            # Reset for next iteration
            node.get_logger().info(
                f"{self.name} iteration {self.current_iteration} complete, starting next"
            )
            self.child_sequence.reset()
            self.status = BehaviorStatus.RUNNING
            return self.status
        
        elif status == BehaviorStatus.FAILURE:
            self.status = BehaviorStatus.FAILURE
            return self.status
        
        self.status = BehaviorStatus.RUNNING
        return self.status
    
    def reset(self):
        super().reset()
        self.current_iteration = 0
        self.child_sequence.reset()

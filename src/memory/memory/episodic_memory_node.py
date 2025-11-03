#!/usr/bin/env python3
"""
Episodic Memory Node

Manages episodic memories - event-based recordings of robot experiences:
- Navigation episodes (where the robot went and what it did)
- Interaction events (encounters with objects/people)
- Temporal context (when events occurred)
- Environmental snapshots at key moments

This node records experiences as episodes for later retrieval and learning.

Topics:
  - /memory/episodes (std_msgs/String): JSON of recorded episodes
  - /memory/current_episode (std_msgs/String): Active episode data
  - /scene_graph (subscription): Scene snapshots
  - /cmd_vel (subscription): Robot motion commands

Services:
  - /memory/start_episode: Begin a new episode
  - /memory/end_episode: End current episode
  - /memory/query_episodes: Retrieve episodes matching criteria
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import json
from datetime import datetime
from typing import Dict, List, Any, Optional


class EpisodicMemory(Node):
    """Manages episodic memory - event-based robot experiences."""

    def __init__(self):
        super().__init__('episodic_memory')

        # ROS parameters
        self.declare_parameter('episode_storage_path', 'data/episodes')
        self.declare_parameter('max_episodes', 1000)
        self.declare_parameter('episode_timeout', 600.0)  # 10 minutes
        self.declare_parameter('publish_rate', 1.0)

        self.episode_storage_path = self.get_parameter('episode_storage_path').value
        self.max_episodes = self.get_parameter('max_episodes').value
        self.episode_timeout = self.get_parameter('episode_timeout').value
        self.publish_rate = self.get_parameter('publish_rate').value

        # State: episodes indexed by ID
        self.episodes: Dict[str, Dict[str, Any]] = {}
        self.current_episode: Optional[Dict[str, Any]] = None
        self.episode_counter = 0

        # Publishers
        self.episodes_pub = self.create_publisher(String, '/memory/episodes', 10)
        self.current_episode_pub = self.create_publisher(String, '/memory/current_episode', 10)

        # Subscribers
        self.create_subscription(
            String, '/scene_graph/detections', self.scene_graph_cb, 10
        )
        self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_cb, 10
        )

        # Timer for publishing episodes
        self.create_timer(1.0 / self.publish_rate, self.publish_episodes)

        self.get_logger().info('Episodic Memory Node initialized')
        self._load_episodes()
        self.start_episode('default_session')

    def _load_episodes(self):
        """Load episodes from storage if available."""
        try:
            from pathlib import Path
            storage_dir = Path(self.episode_storage_path)
            if not storage_dir.is_absolute():
                storage_dir = Path.cwd() / self.episode_storage_path

            if storage_dir.exists():
                # Load all episode files
                for episode_file in storage_dir.glob('episode_*.json'):
                    try:
                        with open(episode_file, 'r') as f:
                            episode = json.load(f)
                            episode_id = episode.get('id')
                            if episode_id:
                                self.episodes[episode_id] = episode
                    except Exception as e:
                        self.get_logger().warn(f'Failed to load episode {episode_file}: {e}')

                self.get_logger().info(
                    f'Loaded {len(self.episodes)} episodes from storage'
                )
            else:
                self.get_logger().info('Episode storage directory not found. Starting fresh.')
                storage_dir.mkdir(parents=True, exist_ok=True)
        except Exception as e:
            self.get_logger().error(f'Failed to load episodes: {e}')

    def start_episode(self, label: str = 'unnamed') -> str:
        """Start a new episode."""
        try:
            self.episode_counter += 1
            episode_id = f'ep_{datetime.now().strftime("%Y%m%d_%H%M%S")}_{self.episode_counter}'

            self.current_episode = {
                'id': episode_id,
                'label': label,
                'start_time': datetime.now().isoformat(),
                'end_time': None,
                'duration_seconds': 0.0,
                'events': [],
                'snapshots': [],
                'motion_data': [],
            }

            self.episodes[episode_id] = self.current_episode
            self.get_logger().info(f'Started episode: {episode_id} ({label})')
            return episode_id
        except Exception as e:
            self.get_logger().error(f'Failed to start episode: {e}')
            return ''

    def end_episode(self) -> bool:
        """End the current episode."""
        try:
            if self.current_episode is None:
                self.get_logger().warn('No active episode to end')
                return False

            end_time = datetime.now()
            self.current_episode['end_time'] = end_time.isoformat()

            # Calculate duration
            start_time = datetime.fromisoformat(self.current_episode['start_time'])
            duration = (end_time - start_time).total_seconds()
            self.current_episode['duration_seconds'] = duration

            self.get_logger().info(
                f'Ended episode: {self.current_episode["id"]} '
                f'(duration: {duration:.1f}s, {len(self.current_episode["events"])} events)'
            )

            # Save episode to disk
            self._save_episode(self.current_episode)

            self.current_episode = None
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to end episode: {e}')
            return False

    def _save_episode(self, episode: Dict[str, Any]) -> bool:
        """Save an episode to disk."""
        try:
            from pathlib import Path
            storage_dir = Path(self.episode_storage_path)
            if not storage_dir.is_absolute():
                storage_dir = Path.cwd() / self.episode_storage_path

            storage_dir.mkdir(parents=True, exist_ok=True)
            episode_file = storage_dir / f"{episode['id']}.json"

            with open(episode_file, 'w') as f:
                json.dump(episode, f, indent=2)

            self.get_logger().info(f'Saved episode to {episode_file}')
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to save episode: {e}')
            return False

    def add_event_to_episode(
        self,
        event_type: str,
        description: str,
        data: Optional[Dict[str, Any]] = None,
    ) -> bool:
        """Add an event to the current episode."""
        try:
            if self.current_episode is None:
                return False

            event = {
                'timestamp': datetime.now().isoformat(),
                'type': event_type,
                'description': description,
                'data': data or {},
            }
            self.current_episode['events'].append(event)
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to add event to episode: {e}')
            return False

    def scene_graph_cb(self, msg):
        """Process scene graph updates and record in episode."""
        try:
            if self.current_episode is None:
                return

            data = json.loads(msg.data)
            snapshot = {
                'timestamp': datetime.now().isoformat(),
                'objects': data.get('objects', []),
            }
            self.current_episode['snapshots'].append(snapshot)

            # Only keep last 100 snapshots
            if len(self.current_episode['snapshots']) > 100:
                self.current_episode['snapshots'] = self.current_episode['snapshots'][-100:]

        except Exception as e:
            self.get_logger().error(f'Error processing scene graph: {e}')

    def cmd_vel_cb(self, msg):
        """Record motion commands in episode."""
        try:
            if self.current_episode is None:
                return

            motion = {
                'timestamp': datetime.now().isoformat(),
                'linear_x': float(msg.linear.x),
                'linear_y': float(msg.linear.y),
                'angular_z': float(msg.angular.z),
            }
            self.current_episode['motion_data'].append(motion)

            # Only keep last 100 motion records
            if len(self.current_episode['motion_data']) > 100:
                self.current_episode['motion_data'] = self.current_episode['motion_data'][-100:]

        except Exception as e:
            self.get_logger().error(f'Error processing motion command: {e}')

    def publish_episodes(self):
        """Publish current episodes state."""
        try:
            episodes_data = {
                'timestamp': datetime.now().isoformat(),
                'total_episodes': len(self.episodes),
                'current_episode_id': self.current_episode['id'] if self.current_episode else None,
                'episode_summaries': [
                    {
                        'id': ep['id'],
                        'label': ep['label'],
                        'start_time': ep['start_time'],
                        'duration_seconds': ep['duration_seconds'],
                        'num_events': len(ep['events']),
                    }
                    for ep in self.episodes.values()
                ],
            }
            msg = String()
            msg.data = json.dumps(episodes_data)
            self.episodes_pub.publish(msg)

            # Publish current episode details
            if self.current_episode:
                current_msg = String()
                current_msg.data = json.dumps(self.current_episode)
                self.current_episode_pub.publish(current_msg)

        except Exception as e:
            self.get_logger().error(f'Failed to publish episodes: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = EpisodicMemory()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.current_episode:
            node.end_episode()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

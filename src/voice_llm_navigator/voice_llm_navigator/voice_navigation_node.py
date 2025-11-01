"""voice_navigation_node.py

ROS2 node that coordinates voice processing and LLM parsing. MVP behavior:
- Subscribes to `/scene_graph` (String JSON produced by existing pipeline) to maintain a list of detected objects
- Runs a background loop that calls `VoiceProcessor.listen_once()` to get text
- Calls `LocalLLMClient.parse_command()` to map text -> intent
- Publishes navigation intent to `/llm_goal` as a simple string (for existing llm_goal_detection to pick up)

This MVP uses the rule-based LocalLLMClient; later we'll swap in Ollama/GPT4All client.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import threading
import time
from typing import List, Dict
import os
import yaml
from ament_index_python.packages import get_package_share_directory

from .voice_processor import VoiceProcessor
from .local_llm_client import LocalLLMClient


class VoiceNavigationNode(Node):
    def __init__(self):
        super().__init__('voice_navigation')
        self.get_logger().info('Starting voice_navigation node (MVP)')

        # publishers
        self.llm_goal_pub = self.create_publisher(String, '/llm_goal', 10)

        # subscribers
        self.sg_sub = self.create_subscription(String, '/scene_graph', self.sg_cb, 10)

        # internal state
        self.detected_objects: List[Dict] = []

        # components
        # Read a node parameter or environment variable to optionally force
        # stdin typing mode (useful for testing without a microphone).
        try:
            self.declare_parameter('force_stdin', False)
        except Exception:
            # Some older rclpy may behave differently; ignore declaration errors
            pass
        param_force = False
        try:
            param_force = bool(self.get_parameter('force_stdin').value)
        except Exception:
            param_force = False

        env_force = os.getenv('VOICE_TYPING_MODE', '')
        env_force_bool = str(env_force).lower() in ('1', 'true', 'yes')

        force_stdin = param_force or env_force_bool
        if force_stdin:
            self.get_logger().info('Typing mode enabled (force_stdin=True) — you can type commands in the terminal.')

        self.voice = VoiceProcessor(force_stdin=force_stdin)

        # load llm config from package config/llm_config.yaml if present
        model = None
        system_prompt = None
        try:
            pkg_share = get_package_share_directory('voice_llm_navigator')
            cfg_path = os.path.join(pkg_share, 'config', 'llm_config.yaml')
            if os.path.exists(cfg_path):
                with open(cfg_path, 'r') as f:
                    cfg = yaml.safe_load(f)
                    model = cfg.get('model')
                    system_prompt = cfg.get('system_prompt')
        except Exception as e:
            self.get_logger().warning(f'Failed to read llm_config.yaml: {e}')

        # create LocalLLMClient with configured model/prompt (will raise if Ollama missing)
        self.llm = LocalLLMClient(system_prompt=system_prompt, model=model)
        # Log which model/prompt are in use
        try:
            model_name = getattr(self.llm, 'model', None)
            prompt_snip = (getattr(self.llm, 'system_prompt', '') or '').splitlines()[0]
            self.get_logger().info(f'LocalLLMClient initialized with model="{model_name}" prompt="{prompt_snip}"')
        except Exception:
            pass

        # background thread to listen
        self._stop = False
        self._thread = threading.Thread(target=self._voice_loop, daemon=True)
        self._thread.start()

    def sg_cb(self, msg: String):
        try:
            sg = json.loads(msg.data)
            objs = sg.get('objects', [])
            # simplify store: list of dicts with 'label' and 'id'
            self.detected_objects = [{'id': o.get('id'), 'label': o.get('label')} for o in objs]
        except Exception as e:
            self.get_logger().warning(f'bad scene_graph json: {e}')

    def _voice_loop(self):
        # small delay for startup
        time.sleep(1.0)
        while rclpy.ok() and not self._stop:
            try:
                text = self.voice.listen_once(timeout=5.0)
                if not text:
                    # no speech detected; loop
                    time.sleep(0.5)
                    continue

                self.get_logger().info(f'Heard: "{text}"')
                result = self.llm.parse_command(text, self.detected_objects)
                self.get_logger().info(f'Parsed intent: {result}')

                if result.get('intent') == 'go_to' and result.get('target_label'):
                    # publish normalized llm_goal string
                    goal_str = f"go to {result.get('target_label')}"
                    self.llm_goal_pub.publish(String(data=goal_str))
                    self.get_logger().info(f'Published /llm_goal: {goal_str}')
                elif result.get('intent') == 'list_objects':
                    labels = result.get('objects', [])
                    self.get_logger().info(f'Objects seen: {labels}')
                elif result.get('intent') == 'stop':
                    self.llm_goal_pub.publish(String(data='stop'))
                else:
                    # unknown: publish raw to /llm_goal so existing nodes can try
                    self.llm_goal_pub.publish(String(data=result.get('raw', text)))

            except Exception as e:
                self.get_logger().warning(f'voice loop error: {e}')
            # small sleep to avoid tight loop
            time.sleep(0.2)

    def destroy_node(self):
        self._stop = True
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VoiceNavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

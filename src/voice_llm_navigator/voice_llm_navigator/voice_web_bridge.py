#!/usr/bin/env python3
"""voice_web_bridge.py

ROS2 node that bridges the Flask voice web UI with the voice_llm_navigator system.

This node:
- Runs the Flask server from tools/voice_web_ui/app.py
- Subscribes to /scene_graph to maintain detected objects list
- Provides a modified /api/chat endpoint that:
  1. Takes user voice input (transcribed text)
  2. Uses LocalLLMClient to parse the command with scene graph context
  3. Publishes navigation commands to /llm_goal topic
  4. Returns synthesized TTS response
- Integrates Piper TTS and Faster-Whisper STT like the standalone web UI
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import threading
import os
import sys
import subprocess
import tempfile
import uuid
import shlex
from typing import List, Dict, Optional

# Import Flask components
from flask import Flask, render_template, request, jsonify, send_file
from werkzeug.utils import safe_join

# Import voice_llm_navigator components
from .local_llm_client import LocalLLMClient


class VoiceWebBridge(Node):
    def __init__(self):
        super().__init__('voice_web_bridge')
        self.get_logger().info('Starting voice_web_bridge node')

        # ROS2 publishers
        self.llm_goal_pub = self.create_publisher(String, '/llm_goal', 10)

        # ROS2 subscribers
        self.sg_sub = self.create_subscription(String, '/scene_graph', self.sg_cb, 10)

        # Internal state
        self.detected_objects: List[Dict] = []
        
        # Initialize LLM client (same as voice_navigation_node)
        self.llm = LocalLLMClient(
            system_prompt='Parse navigation commands and return intent and target as JSON.',
            model='llama3.1:8b'
        )

        # Flask app setup
        self.flask_app = self._create_flask_app()
        
        # Get Flask server config from parameters
        self.declare_parameter('flask_host', '0.0.0.0')
        self.declare_parameter('flask_port', 5003)
        self.flask_host = self.get_parameter('flask_host').value
        self.flask_port = self.get_parameter('flask_port').value

        # Piper config
        self.piper_model = os.path.expanduser('~/.local/share/piper/voices/en_US-lessac-medium.onnx')
        self.piper_config = os.path.expanduser('~/.local/share/piper/voices/en_US-lessac-medium.onnx.json')
        
        # Temp directory for audio files
        self.tmpdir = os.path.join(tempfile.gettempdir(), 'voice_web_ui')
        os.makedirs(self.tmpdir, exist_ok=True)

        # Lazy-loaded faster-whisper model
        self._fw_model = None

        # Start Flask in background thread
        self._flask_thread = threading.Thread(target=self._run_flask, daemon=True)
        self._flask_thread.start()
        
        self.get_logger().info(f'Flask server starting on {self.flask_host}:{self.flask_port}')

    def sg_cb(self, msg: String):
        """Scene graph callback - updates detected objects list"""
        try:
            sg = json.loads(msg.data)
            objs = sg.get('objects', [])
            self.detected_objects = [{'id': o.get('id'), 'label': o.get('label')} for o in objs]
            self.get_logger().debug(f'Updated scene graph: {len(self.detected_objects)} objects')
        except Exception as e:
            self.get_logger().warning(f'Bad scene_graph json: {e}')

    def _load_faster_whisper(self, model_name='tiny'):
        """Lazy load faster-whisper model"""
        if self._fw_model is not None:
            return self._fw_model
        try:
            from faster_whisper import WhisperModel
            self._fw_model = WhisperModel(model_name, device='cpu')
            self.get_logger().info(f'Loaded faster-whisper model: {model_name}')
            return self._fw_model
        except Exception as e:
            self.get_logger().warning(f'faster_whisper not available: {e}')
            return None

    def _transcribe_file(self, wav_path, model_name='tiny'):
        """Transcribe audio file using faster-whisper"""
        m = self._load_faster_whisper(model_name)
        if m is None:
            return None, 'faster_whisper unavailable'
        try:
            segments, info = m.transcribe(wav_path)
            text = ' '.join(s.text.strip() for s in segments if hasattr(s, 'text'))
            return text, None
        except Exception as e:
            return None, str(e)

    def _convert_to_wav(self, in_path, out_path):
        """Convert audio to WAV using ffmpeg"""
        try:
            subprocess.run(['ffmpeg', '-y', '-i', in_path, out_path], 
                         check=True, capture_output=True, timeout=30)
            return True
        except Exception as e:
            self.get_logger().warning(f'ffmpeg convert failed: {e}')
            return False

    def _synthesize_with_piper_cli(self, text, model_path=None, config_path=None):
        """Synthesize speech using Piper CLI"""
        model_path = model_path or self.piper_model
        config_path = config_path or self.piper_config
        out_name = f'tts-{uuid.uuid4().hex}.wav'
        out_path = os.path.join(self.tmpdir, out_name)
        cmd = ['piper', '-m', model_path, '-c', config_path, '-f', out_path, '--text', text]
        try:
            self.get_logger().debug(f'Running Piper CLI: {" ".join(shlex.quote(x) for x in cmd)}')
            subprocess.run(cmd, check=True, capture_output=True, timeout=60)
            self.get_logger().debug(f'Piper produced: {out_path}')
            return out_path
        except Exception as e:
            self.get_logger().error(f'Piper CLI failed: {e}')
            return None

    def _create_flask_app(self):
        """Create and configure Flask app with ROS2-integrated endpoints"""
        
        # Find templates and static folders from tools/voice_web_ui
        # Navigate from the installed package location back to the workspace root
        pkg_dir = os.path.dirname(os.path.abspath(__file__))
        # Go up from voice_llm_navigator/voice_web_bridge.py to find workspace root
        # Installed structure: install/voice_llm_navigator/lib/python3.10/site-packages/voice_llm_navigator/
        # We need to get to the workspace root which should have tools/voice_web_ui
        workspace_root = os.path.abspath(os.path.join(pkg_dir, '../../../../../..'))
        template_folder = os.path.join(workspace_root, 'tools/voice_web_ui/templates')
        static_folder = os.path.join(workspace_root, 'tools/voice_web_ui/static')
        
        self.get_logger().info(f'Looking for templates at: {template_folder}')
        self.get_logger().info(f'Looking for static files at: {static_folder}')
        
        if not os.path.exists(template_folder):
            self.get_logger().warn(f'Template folder not found at {template_folder}')
        if not os.path.exists(static_folder):
            self.get_logger().warn(f'Static folder not found at {static_folder}')
        
        app = Flask(__name__, template_folder=template_folder, static_folder=static_folder)

        @app.route('/')
        def index():
            return render_template('index.html')

        @app.route('/api/transcribe', methods=['POST'])
        def api_transcribe():
            """Transcribe uploaded audio file"""
            if 'file' not in request.files:
                return jsonify({'error': 'no file uploaded'}), 400
            
            f = request.files['file']
            fname = f.filename or f'upload-{uuid.uuid4().hex}'
            raw_path = os.path.join(self.tmpdir, f'raw-{uuid.uuid4().hex}-{fname}')
            f.save(raw_path)

            # Convert to WAV
            wav_path = os.path.join(self.tmpdir, f'conv-{uuid.uuid4().hex}.wav')
            converted = self._convert_to_wav(raw_path, wav_path)
            if not converted:
                if raw_path.lower().endswith('.wav'):
                    wav_path = raw_path
                else:
                    return jsonify({'error': 'ffmpeg conversion failed'}), 500

            # Transcribe
            text, err = self._transcribe_file(wav_path)
            if err:
                return jsonify({'error': err}), 500
            return jsonify({'text': text})

        @app.route('/api/chat', methods=['POST'])
        def api_chat():
            """
            ROS2-integrated chat endpoint:
            - Takes user text (from voice or typing)
            - Parses intent using LocalLLMClient with scene graph context
            - Publishes navigation commands to /llm_goal
            - Returns LLM reply + synthesized audio
            """
            data = request.get_json(force=True)
            if not data or 'text' not in data:
                return jsonify({'error': 'text required'}), 400
            
            user_text = data['text']
            self.get_logger().info(f'[Web UI] Received: "{user_text}"')

            # Parse command using LLM with scene graph context
            result = self.llm.parse_command(user_text, self.detected_objects)
            self.get_logger().info(f'[Web UI] Parsed intent: {result}')

            # Generate response based on intent
            reply = self._generate_reply(result, user_text)

            # Publish navigation command if applicable
            if result.get('intent') == 'go_to' and result.get('target_label'):
                goal_str = f"go to {result.get('target_label')}"
                self.llm_goal_pub.publish(String(data=goal_str))
                self.get_logger().info(f'[Web UI] Published /llm_goal: {goal_str}')
            elif result.get('intent') == 'stop':
                self.llm_goal_pub.publish(String(data='stop'))
                self.get_logger().info('[Web UI] Published stop command')

            # Synthesize reply
            self.get_logger().info(f'[Web UI] Synthesizing reply: {reply[:100]}...')
            out = self._synthesize_with_piper_cli(reply)
            
            if not out:
                return jsonify({'reply': reply, 'audio': None})

            # Return audio URL
            fname = os.path.basename(out)
            base = request.url_root.rstrip('/')
            audio_url = f"{base}/tts/{fname}"
            self.get_logger().info(f'[Web UI] Reply ready, audio at: {audio_url}')
            return jsonify({'reply': reply, 'audio': audio_url})

        @app.route('/tts/<path:filename>')
        def tts_file(filename):
            """Serve synthesized audio files"""
            safe_path = safe_join(self.tmpdir, filename)
            if not safe_path or not os.path.exists(safe_path):
                return 'Not found', 404
            return send_file(safe_path, mimetype='audio/wav')

        return app

    def _generate_reply(self, result: Dict, user_text: str) -> str:
        """Generate natural language reply based on parsed intent"""
        intent = result.get('intent')
        
        if intent == 'go_to':
            target = result.get('target_label', 'unknown')
            return f"Navigating to the {target}."
        
        elif intent == 'list_objects':
            objs = result.get('objects', [])
            if objs:
                obj_list = ', '.join(objs)
                return f"I can see: {obj_list}."
            else:
                return "I don't see any objects right now."
        
        elif intent == 'stop':
            return "Stopping navigation."
        
        elif intent == 'llm_text':
            # LLM returned text but we couldn't parse JSON
            llm_text = result.get('text', '')
            return llm_text if llm_text else "I heard you, but I'm not sure what to do."
        
        elif intent == 'unknown':
            # Check if we have detected objects to help guide the user
            if self.detected_objects:
                labels = [o.get('label') for o in self.detected_objects if o.get('label')]
                obj_list = ', '.join(labels[:5])  # Limit to 5 objects
                return f"I'm not sure what you meant. I can see: {obj_list}. Try saying 'go to' followed by an object name."
            else:
                return "I'm not sure what you meant. Try saying 'go to' followed by an object name."
        
        else:
            return f"I heard: {user_text}"

    def _run_flask(self):
        """Run Flask server (called in background thread)"""
        try:
            self.flask_app.run(
                host=self.flask_host,
                port=self.flask_port,
                debug=False,
                use_reloader=False,  # Important: disable reloader in thread
                threaded=True
            )
        except Exception as e:
            self.get_logger().error(f'Flask server error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = VoiceWebBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

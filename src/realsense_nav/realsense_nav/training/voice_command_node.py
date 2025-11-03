#!/usr/bin/env python3
"""
Simple Voice Command Node

Listens to microphone and publishes recognized commands to /voice/command.
Uses speech_recognition library for simple voice input.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading

try:
    import speech_recognition as sr
    SPEECH_RECOGNITION_AVAILABLE = True
except ImportError:
    SPEECH_RECOGNITION_AVAILABLE = False


class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__('voice_command_node')
        
        self.declare_parameter('language', 'en-US')
        self.declare_parameter('energy_threshold', 4000)
        
        self.language = self.get_parameter('language').value
        self.energy_threshold = self.get_parameter('energy_threshold').value
        
        self.publisher = self.create_publisher(String, '/voice/command', 10)
        
        if not SPEECH_RECOGNITION_AVAILABLE:
            self.get_logger().error('speech_recognition not installed! Install: pip install SpeechRecognition pyaudio')
            return
        
        self.recognizer = sr.Recognizer()
        self.recognizer.energy_threshold = self.energy_threshold
        self.microphone = sr.Microphone()
        
        # Start listening in separate thread
        self.listening = True
        self.listen_thread = threading.Thread(target=self._listen_loop, daemon=True)
        self.listen_thread.start()
        
        self.get_logger().info('Voice Command Node started - speak commands')
        self.get_logger().info('Commands: "record junction", "save map", "go to [location]"')
    
    def _listen_loop(self):
        """Continuous listening loop"""
        with self.microphone as source:
            self.get_logger().info('Adjusting for ambient noise... please wait')
            self.recognizer.adjust_for_ambient_noise(source, duration=1)
            self.get_logger().info('✓ Ready! Start speaking...')
        
        while self.listening and rclpy.ok():
            try:
                with self.microphone as source:
                    audio = self.recognizer.listen(source, timeout=1, phrase_time_limit=5)
                
                try:
                    text = self.recognizer.recognize_google(audio, language=self.language)
                    
                    if text:
                        self.get_logger().info(f'Recognized: "{text}"')
                        
                        msg = String()
                        msg.data = text
                        self.publisher.publish(msg)
                        
                except sr.UnknownValueError:
                    pass  # Could not understand audio
                except sr.RequestError as e:
                    self.get_logger().error(f'Speech recognition error: {e}')
                    
            except sr.WaitTimeoutError:
                pass  # No speech detected
            except Exception as e:
                self.get_logger().error(f'Listening error: {e}')
    
    def destroy_node(self):
        """Clean shutdown"""
        self.listening = False
        if hasattr(self, 'listen_thread'):
            self.listen_thread.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommandNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

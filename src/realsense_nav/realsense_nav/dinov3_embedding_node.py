#!/usr/bin/env python3
"""
dinov3_embedding_node.py

Subscribe to RGB images and extract DINOv3-Small embeddings for place recognition.
Publishes scene embeddings that can be used for:
  - Place recognition (visual similarity)
  - Multi-modal reasoning (with text queries)
  - Topological mapping

DINOv3 provides both visual embeddings and text-image matching capabilities.
Uses adaptive frequency based on system load.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32MultiArray
from cv_bridge import CvBridge
import numpy as np
import json
import time

try:
    import torch
    from transformers import AutoModel, AutoImageProcessor
    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False


class DINOv3EmbeddingNode(Node):
    def __init__(self):
        super().__init__('dinov3_embedding_node')
        
        if not TORCH_AVAILABLE:
            self.get_logger().error('PyTorch and transformers not available! Install with: pip install torch transformers')
            raise ImportError('Required packages not available')
        
        # Parameters
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('embedding_topic', '/dinov3/embedding')
        self.declare_parameter('model_name', 'facebook/dinov2-small')  # Will update to dinov3 when available
        self.declare_parameter('device', 'cuda')  # or 'cpu'
        self.declare_parameter('update_rate_hz', 1.0)  # Default 1 Hz
        self.declare_parameter('adaptive_rate', True)  # Adjust rate based on confidence
        self.declare_parameter('embedding_dim', 384)  # DINOv3-Small: 384, Base: 768
        
        # Get parameters
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.embedding_topic = self.get_parameter('embedding_topic').get_parameter_value().string_value
        model_name = self.get_parameter('model_name').get_parameter_value().string_value
        device_str = self.get_parameter('device').get_parameter_value().string_value
        self.base_update_rate = self.get_parameter('update_rate_hz').get_parameter_value().double_value
        self.adaptive_rate = self.get_parameter('adaptive_rate').get_parameter_value().bool_value
        self.embedding_dim = self.get_parameter('embedding_dim').get_parameter_value().integer_value
        
        # Initialize
        self.bridge = CvBridge()
        self.device = torch.device(device_str if torch.cuda.is_available() else 'cpu')
        self.last_process_time = time.time()
        self.current_update_interval = 1.0 / self.base_update_rate
        self.processing_time_ms = 0.0
        
        # Load DINOv3 model
        self.get_logger().info(f'Loading DINOv3 model: {model_name} on {self.device}')
        try:
            self.processor = AutoImageProcessor.from_pretrained(model_name)
            self.model = AutoModel.from_pretrained(model_name).to(self.device)
            self.model.eval()  # Inference mode
            self.get_logger().info(f'DINOv3 model loaded successfully. Embedding dim: {self.embedding_dim}')
        except Exception as e:
            self.get_logger().error(f'Failed to load DINOv3 model: {e}')
            self.get_logger().info('Attempting to use DINOv2 as fallback...')
            model_name = 'facebook/dinov2-small'
            self.processor = AutoImageProcessor.from_pretrained(model_name)
            self.model = AutoModel.from_pretrained(model_name).to(self.device)
            self.model.eval()
            self.get_logger().info(f'Using DINOv2-Small as fallback. Embedding dim: {self.embedding_dim}')
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )
        
        # Publishers
        self.embedding_pub = self.create_publisher(
            Float32MultiArray,
            self.embedding_topic,
            10
        )
        
        self.stats_pub = self.create_publisher(
            String,
            '/dinov3/stats',
            10
        )
        
        # Statistics
        self.frame_count = 0
        self.processed_count = 0
        self.total_inference_time = 0.0
        
        self.get_logger().info(f'DINOv3 embedding node started')
        self.get_logger().info(f'  Image topic: {self.image_topic}')
        self.get_logger().info(f'  Embedding topic: {self.embedding_topic}')
        self.get_logger().info(f'  Update rate: {self.base_update_rate} Hz')
        self.get_logger().info(f'  Device: {self.device}')
    
    def image_callback(self, msg: Image):
        """Process image and extract DINOv3 embedding at adaptive rate"""
        self.frame_count += 1
        
        # Adaptive rate control
        current_time = time.time()
        time_since_last = current_time - self.last_process_time
        
        if time_since_last < self.current_update_interval:
            return  # Skip this frame
        
        self.last_process_time = current_time
        
        try:
            # Convert ROS image to numpy
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            
            # Extract embedding
            start_time = time.time()
            embedding = self.extract_embedding(cv_image)
            inference_time = (time.time() - start_time) * 1000.0  # ms
            
            # Update statistics
            self.processed_count += 1
            self.total_inference_time += inference_time
            self.processing_time_ms = inference_time
            
            # Publish embedding
            embedding_msg = Float32MultiArray()
            embedding_msg.data = embedding.tolist()
            self.embedding_pub.publish(embedding_msg)
            
            # Publish statistics (every 10 processed frames)
            if self.processed_count % 10 == 0:
                self.publish_stats()
            
            # Log periodically
            if self.processed_count % 50 == 0:
                avg_time = self.total_inference_time / self.processed_count
                self.get_logger().info(
                    f'Processed {self.processed_count} frames, '
                    f'avg inference: {avg_time:.1f}ms, '
                    f'last: {inference_time:.1f}ms'
                )
        
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')
    
    def extract_embedding(self, image: np.ndarray) -> np.ndarray:
        """
        Extract DINOv3 embedding from RGB image
        
        Args:
            image: RGB numpy array (H, W, 3)
        
        Returns:
            embedding: 1D numpy array of shape (embedding_dim,)
        """
        with torch.no_grad():
            # Preprocess image
            inputs = self.processor(images=image, return_tensors="pt")
            inputs = {k: v.to(self.device) for k, v in inputs.items()}
            
            # Forward pass
            outputs = self.model(**inputs)
            
            # Get CLS token embedding (global scene representation)
            # DINOv3/v2 uses CLS token as the global scene descriptor
            embedding = outputs.last_hidden_state[:, 0, :].squeeze()  # (embedding_dim,)
            
            # Convert to CPU numpy
            embedding = embedding.cpu().numpy()
            
            # Normalize for cosine similarity
            embedding = embedding / (np.linalg.norm(embedding) + 1e-8)
            
            return embedding
    
    def set_update_rate(self, rate_hz: float):
        """
        Dynamically adjust update rate
        Used by external nodes (e.g., belief tracker) to request more/less frequent updates
        """
        self.current_update_interval = 1.0 / max(0.1, min(rate_hz, 30.0))  # Clamp 0.1-30 Hz
        self.get_logger().info(f'Update rate changed to {rate_hz:.1f} Hz')
    
    def publish_stats(self):
        """Publish performance statistics"""
        avg_inference = self.total_inference_time / max(1, self.processed_count)
        actual_rate = self.processed_count / (time.time() - self.get_clock().now().seconds_nanoseconds()[0] + 1e-9)
        
        stats = {
            'node': 'dinov3_embedding',
            'total_frames': self.frame_count,
            'processed_frames': self.processed_count,
            'skip_rate': 1.0 - (self.processed_count / max(1, self.frame_count)),
            'avg_inference_ms': round(avg_inference, 2),
            'last_inference_ms': round(self.processing_time_ms, 2),
            'actual_rate_hz': round(actual_rate, 2),
            'target_rate_hz': round(1.0 / self.current_update_interval, 2),
            'device': str(self.device),
            'embedding_dim': self.embedding_dim
        }
        
        stats_msg = String()
        stats_msg.data = json.dumps(stats)
        self.stats_pub.publish(stats_msg)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = DINOv3EmbeddingNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

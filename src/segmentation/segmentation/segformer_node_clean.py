#!/usr/bin/env python3

from pathlib import Path
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, Float32
from cv_bridge import CvBridge
import cv2
import numpy as np
import torch
from transformers import AutoImageProcessor, SegformerForSemanticSegmentation


class SegFormerNode(Node):
    def __init__(self):
        super().__init__('segformer_node_clean')

        # parameters
        self.declare_parameter('model_name', 'nvidia/segformer-b1-finetuned-ade-512-512')
        self.declare_parameter('local_model_dir', 'models/segformer-b1-finetuned-ade-512-512')
        self.declare_parameter('allow_download', False)
        self.declare_parameter('prob_threshold', 0.5)
        self.declare_parameter('min_area', 5000)
        self.declare_parameter('morph_kernel', 5)

        self.model_name = self.get_parameter('model_name').value
        self.local_model_dir = self.get_parameter('local_model_dir').value
        self.allow_download = bool(self.get_parameter('allow_download').value)
        self.prob_threshold = float(self.get_parameter('prob_threshold').value)
        self.min_area = int(self.get_parameter('min_area').value)
        self.morph_kernel = int(self.get_parameter('morph_kernel').value)

        self.bridge = CvBridge()

        # pubs/subs
        self.image_sub = self.create_subscription(Image, '/camera/camera/color/image_raw', self.image_cb, 2)
        self.mask_pub = self.create_publisher(Image, '/segmentation/floor_mask', 10)
        self.overlay_pub = self.create_publisher(Image, '/segmentation/overlay', 10)
        self.seg_image_pub = self.create_publisher(Image, '/segmentation/image', 10)
        self.centroid_pub = self.create_publisher(Point, '/segmentation/centroid', 10)
        self.navigable_pub = self.create_publisher(Bool, '/segmentation/navigable', 10)
        self.conf_pub = self.create_publisher(Float32, '/segmentation/floor_confidence', 10)

        # load model (local-first)
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.get_logger().info(f'Loading {self.model_name} on {self.device}')

        loaded = False
        local_path = Path(self.local_model_dir)
        if not local_path.is_absolute():
            local_path = (Path(__file__).resolve().parent / local_path).resolve()

        if local_path.exists():
            try:
                self.processor = AutoImageProcessor.from_pretrained(str(local_path), local_files_only=True)
                self.model = SegformerForSemanticSegmentation.from_pretrained(str(local_path), local_files_only=True).to(self.device)
                loaded = True
                self.get_logger().info(f'Loaded model from {local_path}')
            except Exception as e:
                self.get_logger().warn(f'Local load failed: {e}')

        if not loaded:
            if self.allow_download:
                self.get_logger().info('Downloading model from Hugging Face...')
                self.processor = AutoImageProcessor.from_pretrained(self.model_name)
                self.model = SegformerForSemanticSegmentation.from_pretrained(self.model_name).to(self.device)
            else:
                raise RuntimeError('Local model not present and allow_download=False')

        # find floor-like classes
        self.keywords = ['floor', 'ground', 'road', 'pavement', 'sidewalk', 'carpet']
        self.floor_ids = []
        id2label = getattr(self.model.config, 'id2label', None)
        if id2label:
            for k, v in id2label.items():
                name = str(v).lower()
                if any(kw in name for kw in self.keywords):
                    try:
                        self.floor_ids.append(int(k))
                    except Exception:
                        pass

        self.get_logger().info(f'Floor class ids: {self.floor_ids}')

    def image_cb(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            h, w = frame.shape[:2]
            inputs = self.processor(images=frame, return_tensors='pt')
            pix = inputs['pixel_values'].to(self.device)
            with torch.no_grad():
                out = self.model(pixel_values=pix)
            probs = torch.softmax(out.logits, dim=1)
            probs_up = torch.nn.functional.interpolate(probs, size=(h, w), mode='bilinear', align_corners=False)[0].cpu().numpy()

            if self.floor_ids:
                mask = np.zeros((h, w), dtype=np.uint8)
                for fid in self.floor_ids:
                    mask = np.logical_or(mask, probs_up[fid] > self.prob_threshold)
                mask = mask.astype(np.uint8) * 255
                # Morphological filtering
                kernel = np.ones((self.morph_kernel, self.morph_kernel), np.uint8)
                mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
                # Area filter
                num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
                for i in range(1, num_labels):
                    if stats[i, cv2.CC_STAT_AREA] < self.min_area:
                        mask[labels == i] = 0
                # Publish mask
                mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
                mask_msg.header = msg.header
                self.mask_pub.publish(mask_msg)
                # Overlay
                overlay = frame.copy()
                overlay[mask > 0] = (0, 255, 0)
                overlay_msg = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
                overlay_msg.header = msg.header
                self.overlay_pub.publish(overlay_msg)
                self.seg_image_pub.publish(overlay_msg)
                # Centroid
                M = cv2.moments(mask)
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    centroid = Point()
                    centroid.x = float(cx)
                    centroid.y = float(cy)
                    centroid.z = 0.0
                    self.centroid_pub.publish(centroid)
                    self.navigable_pub.publish(Bool(data=True))
                else:
                    self.navigable_pub.publish(Bool(data=False))
                # Confidence
                conf = float(np.max(probs_up[self.floor_ids])) if self.floor_ids else 0.0
                self.conf_pub.publish(Float32(data=conf))
        except Exception as e:
            self.get_logger().error(f'Image callback error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = SegFormerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

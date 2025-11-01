#!/usr/bin/env python3
"""Save the first image published on a ROS2 image topic to disk.

Usage:
  python3 tools/save_topic_image.py /topic/name out.png

This is a small helper to capture a preview image when GUI tools like
`rqt_image_view` fail to load due to system/plugin issues.
"""
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class ImageSaver(Node):
    def __init__(self, topic: str, out_path: str):
        super().__init__('save_topic_image')
        self.topic = topic
        self.out_path = out_path
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, topic, self.cb_image, 10)
        self.get_logger().info(f'Subscribing to {topic} and will save first frame to {out_path}')

    def cb_image(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            rclpy.shutdown()
            return
        ok = cv2.imwrite(self.out_path, cv_img)
        if ok:
            self.get_logger().info(f'Wrote image to: {self.out_path}')
        else:
            self.get_logger().error(f'Failed to write image to: {self.out_path}')
        # shutdown after first frame
        rclpy.shutdown()


def main(argv=None):
    argv = argv or sys.argv[1:]
    if len(argv) < 2:
        print('Usage: save_topic_image.py /topic/name out.png')
        return 2
    topic = argv[0]
    out_path = argv[1]
    rclpy.init()
    node = ImageSaver(topic, out_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())

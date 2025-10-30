#!/usr/bin/env python3
"""
Vision Safety Layer

Converts segmentation mask + depth into a simple safety signal and a
prototype OccupancyGrid on /vision_occupancy. Also publishes /safety/stop
as std_msgs/Bool when obstacles (non-floor) appear closer than threshold.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from nav_msgs.msg import OccupancyGrid, MapMetaData
from cv_bridge import CvBridge
import numpy as np
 


class VisionSafetyNode(Node):
    def __init__(self):
        super().__init__('vision_safety_node')
        self.bridge = CvBridge()
        self.latest_depth = None
        self.latest_mask = None

        self.stop_threshold = 0.45  # meters

        self.create_subscription(Image, '/camera/camera/depth/image_rect_raw', self.depth_cb, 2)
        self.create_subscription(Image, '/segmentation/image', self.mask_cb, 2)

        self.stop_pub = self.create_publisher(Bool, '/safety/stop', 10)
        self.occ_pub = self.create_publisher(OccupancyGrid, '/vision_occupancy', 5)

        self.timer = self.create_timer(0.2, self.update_safety)
        self.get_logger().info('VisionSafetyNode initialized')

    def depth_cb(self, msg):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().debug(f'depth convert failed: {e}')

    def mask_cb(self, msg):
        try:
            self.latest_mask = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        except Exception as e:
            self.get_logger().debug(f'mask convert failed: {e}')

    def update_safety(self):
        if self.latest_mask is None or self.latest_depth is None:
            return

        mask = self.latest_mask
        depth = self.latest_depth

        # Check for any non-floor pixel (mask == 0) with depth < threshold
        # (Assumes mask: floor=255, non-floor=0)
        non_floor = (mask == 0)
        if non_floor.any():
            ys, xs = np.where(non_floor)
            distances = []
            h_d, w_d = depth.shape[:2]
            for y, x in zip(ys, xs):
                if 0 <= y < h_d and 0 <= x < w_d:
                    z = float(depth[y, x])
                    if z > 10:
                        z_m = z / 1000.0
                    else:
                        z_m = z
                    distances.append(z_m)
            stop = False
            if distances:
                min_d = float(np.nanmin(distances))
                stop = (min_d > 0 and min_d < self.stop_threshold)
            else:
                stop = False
        else:
            stop = False

        stop_msg = Bool()
        stop_msg.data = bool(stop)
        self.stop_pub.publish(stop_msg)

        # Publish a simple occupancy grid where non-floor=100, floor=0
        ag = OccupancyGrid()
        ag.header.stamp = self.get_clock().now().to_msg()
        ag.header.frame_id = 'camera_link'  # prototype frame
        h, w = mask.shape[:2]
        ag.info = MapMetaData()
        ag.info.resolution = 0.02  # assume 2cm per cell (prototype)
        ag.info.width = int(w)
        ag.info.height = int(h)
        ag.info.origin.position.x = 0.0
        ag.info.origin.position.y = 0.0
        ag.info.origin.orientation.w = 1.0

        # Flatten row-major; set unknown to -1 if depth missing
        grid = np.full((h, w), -1, dtype=np.int8)
        grid[mask > 0] = 0
        grid[mask == 0] = 100

        # nav_msgs/OccupancyGrid expects a Python list of ints (each in [-128,127]).
        # Converting numpy types to native ints avoids assertion errors in setter.
        flat = grid.flatten()
        ag.data = [int(x) for x in flat]
        self.occ_pub.publish(ag)


def main(args=None):
    rclpy.init(args=args)
    node = VisionSafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

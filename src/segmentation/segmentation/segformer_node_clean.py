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
        self.declare_parameter('robot_footprint_size', 0.4)  # 40cm in meters
        self.declare_parameter('pixels_per_meter', 200.0)  # Approximate scaling factor
        self.declare_parameter('obstacle_inflation_radius', 0.3)  # 30cm safety margin in meters
        self.declare_parameter('cost_scaling_factor', 2.0)  # How much to penalize near-obstacle paths

        self.model_name = self.get_parameter('model_name').value
        self.local_model_dir = self.get_parameter('local_model_dir').value
        self.allow_download = bool(self.get_parameter('allow_download').value)
        self.prob_threshold = float(self.get_parameter('prob_threshold').value)
        self.min_area = int(self.get_parameter('min_area').value)
        self.morph_kernel = int(self.get_parameter('morph_kernel').value)
        self.robot_footprint_size = float(self.get_parameter('robot_footprint_size').value)
        self.pixels_per_meter = float(self.get_parameter('pixels_per_meter').value)
        self.obstacle_inflation_radius = float(self.get_parameter('obstacle_inflation_radius').value)
        self.cost_scaling_factor = float(self.get_parameter('cost_scaling_factor').value)

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

    def mark_obstacles_with_inflation(self, floor_mask):
        """
        Mark obstacles (non-floor) with red color and inflation zone.
        Only processes within 1 meter range for performance.
        Returns: (overlay_mask, elapsed_time_ms)
        """
        import time
        start_time = time.time()
        
        h, w = floor_mask.shape
        
        # Convert 1 meter to pixels
        one_meter_pixels = int(1.0 * self.pixels_per_meter)
        
        # Only process bottom portion (within 1 meter from camera)
        process_height = min(one_meter_pixels, h)
        
        # Create colored overlay
        obstacle_overlay = np.zeros((h, w, 3), dtype=np.uint8)
        
        # Process only the bottom region
        process_region = floor_mask[h - process_height:h, :]
        
        # Calculate distance transform for inflation
        dist_transform = cv2.distanceTransform(process_region, cv2.DIST_L2, 5)
        
        # Convert inflation radius to pixels
        inflation_radius_pixels = int(self.obstacle_inflation_radius * self.pixels_per_meter)
        
        # Create masks
        obstacle_mask = process_region == 0
        inflation_mask = (dist_transform > 0) & (dist_transform <= inflation_radius_pixels)
        
        # Mark obstacles in red
        region_overlay = np.zeros((process_height, w, 3), dtype=np.uint8)
        region_overlay[obstacle_mask] = (0, 0, 255)  # Red for obstacles
        
        # Mark inflation zone with orange/yellow gradient
        if np.any(inflation_mask):
            # Gradient from red to yellow based on distance
            distances = dist_transform[inflation_mask]
            ratios = distances / inflation_radius_pixels
            # Orange to yellow gradient
            region_overlay[inflation_mask, 0] = (255 * ratios).astype(np.uint8)  # Blue channel
            region_overlay[inflation_mask, 1] = (165 + 90 * ratios).astype(np.uint8)  # Green channel
            region_overlay[inflation_mask, 2] = 255  # Red channel (full)
        
        # Copy to full image
        obstacle_overlay[h - process_height:h, :] = region_overlay
        
        elapsed_time = (time.time() - start_time) * 1000  # Convert to ms
        
        return obstacle_overlay, elapsed_time

    def find_floor_path(self, mask, start, end):
        """
        Find a path from start to end that stays on the floor (mask > 0).
        Uses a simplified A* pathfinding algorithm.
        """
        import heapq
        
        h, w = mask.shape
        start_x, start_y = start
        end_x, end_y = end
        
        # If start or end is not on floor, return direct line
        if mask[start_y, start_x] == 0 or mask[end_y, end_x] == 0:
            return [start, end]
        
        # A* pathfinding
        def heuristic(pos):
            return abs(pos[0] - end_x) + abs(pos[1] - end_y)
        
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: heuristic(start)}
        
        # 8-directional movement
        directions = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
        
        max_iterations = 5000
        iterations = 0
        
        while open_set and iterations < max_iterations:
            iterations += 1
            _, current = heapq.heappop(open_set)
            
            if current == end:
                # Reconstruct path
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path
            
            cx, cy = current
            for dx, dy in directions:
                nx, ny = cx + dx, cy + dy
                
                # Check bounds
                if not (0 <= nx < w and 0 <= ny < h):
                    continue
                
                # Only move on floor pixels
                if mask[ny, nx] == 0:
                    continue
                
                neighbor = (nx, ny)
                tentative_g_score = g_score[current] + np.sqrt(dx**2 + dy**2)
                
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
        
        # If no path found, return direct line
        return [start, end]
    
    def create_smooth_path(self, path, mask):
        """
        Create a smooth curvy path using cubic spline interpolation.
        Ensures the smoothed path still stays on floor pixels.
        """
        from scipy.interpolate import splprep, splev
        
        if len(path) < 3:
            return path
        
        # Convert path to numpy arrays
        path_array = np.array(path)
        x = path_array[:, 0].astype(float)
        y = path_array[:, 1].astype(float)
        
        # Downsample path for smoother curves (take every nth point)
        step = max(1, len(path) // 8)
        x_sampled = x[::step]
        y_sampled = y[::step]
        
        # Ensure start and end points are included
        if len(x_sampled) > 0 and x_sampled[-1] != x[-1]:
            x_sampled = np.append(x_sampled, x[-1])
            y_sampled = np.append(y_sampled, y[-1])
        
        if len(x_sampled) < 3:
            return path
        
        try:
            # Create cubic B-spline
            tck, u = splprep([x_sampled, y_sampled], s=len(x_sampled) * 10, k=min(3, len(x_sampled) - 1))
            
            # Generate smooth curve with many points
            u_fine = np.linspace(0, 1, len(path) * 2)
            x_smooth, y_smooth = splev(u_fine, tck)
            
            # Filter out points that are not on floor
            smooth_path = []
            h, w = mask.shape
            for i in range(len(x_smooth)):
                px, py = int(round(x_smooth[i])), int(round(y_smooth[i]))
                if 0 <= px < w and 0 <= py < h and mask[py, px] > 0:
                    smooth_path.append((px, py))
            
            # Remove consecutive duplicates
            if len(smooth_path) > 1:
                filtered_path = [smooth_path[0]]
                for point in smooth_path[1:]:
                    if point != filtered_path[-1]:
                        filtered_path.append(point)
                return filtered_path if len(filtered_path) > 1 else path
            
            return path if len(smooth_path) < 2 else smooth_path
            
        except Exception as e:
            # If spline fails, return original path
            return path
    
    def draw_robot_footprint(self, overlay, path, mask):
        """
        Draw robot footprint (40cm x 40cm) along the path.
        The footprint size scales with distance from camera (y-coordinate).
        """
        if len(path) < 2:
            return
        
        h, w = mask.shape
        
        # Draw footprints at regular intervals along the path
        num_footprints = min(8, len(path) // 20 + 1)
        indices = np.linspace(0, len(path) - 1, num_footprints, dtype=int)
        
        for idx in indices:
            x, y = path[idx]
            
            # Scale footprint based on y-coordinate (perspective)
            # Objects further away (smaller y) appear smaller
            scale_factor = 1.0 + (h - y) / h * 0.5  # Scale from 1.0 to 1.5
            
            # Calculate footprint size in pixels
            footprint_pixels = int(self.robot_footprint_size * self.pixels_per_meter / scale_factor)
            half_size = footprint_pixels // 2
            
            # Draw square footprint
            pt1 = (x - half_size, y - half_size)
            pt2 = (x + half_size, y + half_size)
            
            # Draw filled semi-transparent rectangle
            overlay_copy = overlay.copy()
            cv2.rectangle(overlay_copy, pt1, pt2, (0, 255, 255), -1)  # Cyan filled
            cv2.addWeighted(overlay_copy, 0.3, overlay, 0.7, 0, overlay)
            
            # Draw border
            cv2.rectangle(overlay, pt1, pt2, (0, 200, 200), 2)  # Darker cyan border
            
            # Draw direction indicator (small arrow/line showing orientation)
            if idx < len(path) - 1:
                next_x, next_y = path[min(idx + 5, len(path) - 1)]
                dx = next_x - x
                dy = next_y - y
                length = np.sqrt(dx**2 + dy**2)
                if length > 0:
                    dx, dy = dx / length, dy / length
                    arrow_end = (int(x + dx * half_size * 0.7), int(y + dy * half_size * 0.7))
                    cv2.arrowedLine(overlay, (x, y), arrow_end, (255, 255, 255), 2, tipLength=0.4)

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
                
                # Mark obstacles with inflation (optimized, within 1m)
                import time
                total_start = time.time()
                obstacle_overlay, obstacle_time = self.mark_obstacles_with_inflation(mask)
                
                # Blend obstacle overlay with main overlay
                obstacle_mask = np.any(obstacle_overlay > 0, axis=2)
                overlay[obstacle_mask] = cv2.addWeighted(
                    overlay[obstacle_mask], 0.3, 
                    obstacle_overlay[obstacle_mask], 0.7, 0
                )
                
                # Centroid
                M = cv2.moments(mask)
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    
                    # Draw path from bottom center to furthest floor point (top/far end)
                    start_x = w // 2
                    start_y = h - 1
                    
                    # Find the bottom-most floor pixel near the center (start point)
                    search_width = 50
                    for y in range(h - 1, -1, -1):
                        for dx in range(-search_width, search_width + 1):
                            x = start_x + dx
                            if 0 <= x < w and mask[y, x] > 0:
                                start_y = y
                                start_x = x
                                break
                        if start_y < h - 1:
                            break
                    
                    # Find the furthest floor point (smallest y, furthest from camera)
                    # Search around the centroid x-coordinate
                    goal_x = cx
                    goal_y = h - 1  # default to bottom if not found
                    search_width_goal = 100
                    
                    for y in range(0, h):
                        for dx in range(-search_width_goal, search_width_goal + 1):
                            x = cx + dx
                            if 0 <= x < w and mask[y, x] > 0:
                                goal_y = y
                                goal_x = x
                                break
                        if goal_y < h - 1:
                            break
                    
                    # Use A* or simple pathfinding to draw path on floor only
                    path = self.find_floor_path(mask, (start_x, start_y), (goal_x, goal_y))
                    
                    # Create smooth curvy path using spline interpolation
                    if len(path) > 2:
                        smooth_path = self.create_smooth_path(path, mask)
                    else:
                        smooth_path = path
                    
                    # Draw the curvy path on overlay
                    if len(smooth_path) > 1:
                        for i in range(len(smooth_path) - 1):
                            cv2.line(overlay, smooth_path[i], smooth_path[i + 1], (255, 0, 0), 3)
                    
                    # Draw goal point (furthest floor point) - Red
                    cv2.circle(overlay, (goal_x, goal_y), 10, (0, 0, 255), -1)
                    
                    # Draw centroid as reference - Magenta
                    cv2.circle(overlay, (cx, cy), 8, (255, 0, 255), -1)
                    
                    # Draw start point - Yellow
                    cv2.circle(overlay, (start_x, start_y), 8, (255, 255, 0), -1)
                    
                    # Calculate total processing time
                    total_time = (time.time() - total_start) * 1000
                    
                    # Display timing information on overlay
                    timing_text = f"Obstacle: {obstacle_time:.1f}ms | Total: {total_time:.1f}ms"
                    cv2.putText(overlay, timing_text, (10, 30), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                    cv2.putText(overlay, timing_text, (10, 30), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 1)
                    
                    centroid = Point()
                    centroid.x = float(cx)
                    centroid.y = float(cy)
                    centroid.z = 0.0
                    self.centroid_pub.publish(centroid)
                    self.navigable_pub.publish(Bool(data=True))
                else:
                    self.navigable_pub.publish(Bool(data=False))
                
                overlay_msg = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
                overlay_msg.header = msg.header
                self.overlay_pub.publish(overlay_msg)
                self.seg_image_pub.publish(overlay_msg)
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

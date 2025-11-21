#!/usr/bin/env python3

from pathlib import Path
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Bool, Float32
from cv_bridge import CvBridge
import time
import math
try:
    import cv2  # type: ignore
except Exception:
    cv2 = None
try:
    import numpy as np  # type: ignore
except Exception:
    np = None
try:
    import torch  # type: ignore
except Exception:
    torch = None
try:
    from transformers import AutoImageProcessor, SegformerForSemanticSegmentation  # type: ignore
except Exception:
    AutoImageProcessor = None
    SegformerForSemanticSegmentation = None
try:
    from scipy.interpolate import splprep, splev  # type: ignore
except Exception:
    splprep = None
    splev = None


class SegFormerNode(Node):
    def __init__(self):
        super().__init__('segformer_node_clean')

        # parameters
        self.declare_parameter('model_name', 'nvidia/segformer-b1-finetuned-ade-512-512')
        self.declare_parameter('local_model_dir', '/home/raushan/control_one/wheel_control/src/segmentation/models/segformer-b1-finetuned-ade-512-512')
        self.declare_parameter('allow_download', False)
        self.declare_parameter('prob_threshold', 0.7)
        self.declare_parameter('min_area', 500)
        self.declare_parameter('morph_kernel', 55)
        self.declare_parameter('robot_footprint_size', 0.4)  # 40cm in meters
        self.declare_parameter('pixels_per_meter', 50000.0)  # Approximate scaling factor
        self.declare_parameter('obstacle_inflation_radius', 0.3)  # 30cm safety margin in meters
        self.declare_parameter('cost_scaling_factor', 5.0)  # How much to penalize near-obstacle paths (higher = more curve)
        self.declare_parameter('enable_obstacle_detection', False)
        self.declare_parameter('enable_pathfinding', False)
        self.declare_parameter('enable_smoothing', False)
        self.declare_parameter('enable_centroid_search', False)
        self.declare_parameter('enable_drawing', False)
        self.declare_parameter('frame_skip', 1)  # process every Nth frame
        self.declare_parameter('inference_size', 512)  # resize short edge to this (square) for faster inference
        self.declare_parameter('use_amp', False)  # enable mixed precision autocast when on CUDA
        self.declare_parameter('cudnn_benchmark', True)  # enable torch.backends.cudnn.benchmark
        # Toggle frame skipping on/off
        self.declare_parameter('enable_frame_skip', False)
        # Show framerate in logs
        self.declare_parameter('show_fps', True)
        # Enable detailed timing for debugging bottlenecks
        self.declare_parameter('profile_timing', True)

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
        self.enable_obstacle_detection = bool(self.get_parameter('enable_obstacle_detection').value)
        self.enable_pathfinding = bool(self.get_parameter('enable_pathfinding').value)
        self.enable_smoothing = bool(self.get_parameter('enable_smoothing').value)
        self.enable_centroid_search = bool(self.get_parameter('enable_centroid_search').value)
        self.enable_drawing = bool(self.get_parameter('enable_drawing').value)
        if not self.enable_obstacle_detection:
            self.get_logger().info('Obstacle detection is disabled; obstacle inflation will be skipped')
        if not self.enable_pathfinding:
            self.get_logger().info('Pathfinding (A*) is disabled; skipping find_floor_path')
        if not self.enable_smoothing:
            self.get_logger().info('Smoothing (SciPy) is disabled; create_smooth_path will return raw path')
        if not self.enable_centroid_search:
            self.get_logger().info('Centroid search is disabled; nested ring search around centroid will be skipped')
        if not self.enable_drawing:
            self.get_logger().info('Drawing is disabled; overlay and path drawings are skipped')

        self.bridge = CvBridge()
        # throughput tuning state
        self.frame_skip = int(self.get_parameter('frame_skip').value)
        self.frame_counter = 0
        self.rgb_overlay_counter = 0  # Counter for skipping expensive RGB overlay
        self.inference_size = int(self.get_parameter('inference_size').value) if self.get_parameter('inference_size').value else None
        self.use_amp = bool(self.get_parameter('use_amp').value)
        self.use_cudnn_benchmark = bool(self.get_parameter('cudnn_benchmark').value)
        self.enable_frame_skip = bool(self.get_parameter('enable_frame_skip').value)
        self.show_fps = bool(self.get_parameter('show_fps').value)
        if torch.cuda.is_available() and self.use_cudnn_benchmark:
            torch.backends.cudnn.benchmark = True

        self.get_logger().info(
            f'Frame skip={self.frame_skip}, enable_frame_skip={self.enable_frame_skip}, inference_size={self.inference_size}, use_amp={self.use_amp}, cudnn_benchmark={self.use_cudnn_benchmark}, show_fps={self.show_fps}'
        )
        self.last_frame_time = None
        self.frame_processed_count = 0
        self.fps_ema = 0.0
        self.profile_timing = bool(self.get_parameter('profile_timing').value)
        self._profile_count = 0
        self._last_ipm_warn_time = 0.0

        self.declare_parameter('enable_bev', True)
        self.declare_parameter('bev_size_meters', 3.0)
        self.declare_parameter('bev_resolution', 0.01)
        self.declare_parameter('camera_height', 0.5)  # 50cm above ground
        self.declare_parameter('camera_tilt_deg', 10)  # 15 degrees downward tilt
        self.declare_parameter('use_ipm', True)
        self.declare_parameter('camera_fx', 643.089)
        self.declare_parameter('camera_fy', 641.544)
        self.declare_parameter('camera_cx', 649.395)
        self.declare_parameter('camera_cy', 367.171)
        self.declare_parameter('camera_fov_deg', 70.0)
        
        self.enable_bev = bool(self.get_parameter('enable_bev').value)
        self.bev_size_meters = float(self.get_parameter('bev_size_meters').value)
        self.bev_resolution = float(self.get_parameter('bev_resolution').value)
        self.bev_pixels = int(self.bev_size_meters / self.bev_resolution)  # 60×60 pixels
        self.camera_height = float(self.get_parameter('camera_height').value)
        self.camera_tilt_deg = float(self.get_parameter('camera_tilt_deg').value)
        self.use_ipm = bool(self.get_parameter('use_ipm').value)
        self.camera_fx = float(self.get_parameter('camera_fx').value)
        self.camera_fy = float(self.get_parameter('camera_fy').value)
        self.camera_cx = float(self.get_parameter('camera_cx').value)
        self.camera_cy = float(self.get_parameter('camera_cy').value)
        self.camera_fov_deg = float(self.get_parameter('camera_fov_deg').value)
        # Log intrinsics on startup for easier debugging
        try:
            self.get_logger().info(f'Camera intrinsics: fx={self.camera_fx:.3f}, fy={self.camera_fy:.3f}, cx={self.camera_cx:.3f}, cy={self.camera_cy:.3f}, fov_deg={self.camera_fov_deg:.2f}')
        except Exception:
            pass
        
        # Robot footprint for raycasting (radius = half of footprint_size)
        self.robot_radius = self.robot_footprint_size / 2.0  # 0.2m (20cm radius)
        
        # Goal caching for stability (prevent flickering)
        self.cached_best_idx = None
        self.cached_best_angle = None
        self.cached_best_distance = None
        self.goal_update_counter = 0
        self.goal_change_threshold_angle = 30.0  # degrees - only switch if angle differs by >30°
        self.goal_change_threshold_distance = 0.6  # meters - only switch if distance differs by >60cm
        self.goal_change_threshold_score = 0.3  # only switch if new score is 0.3 better
        self.goal_force_update_interval = 40  # force goal update every N frames
        
        self.get_logger().info(
            f'BEV: enabled={self.enable_bev}, size={self.bev_size_meters}m, resolution={self.bev_resolution}m/px ({self.bev_pixels}×{self.bev_pixels} pixels), robot_radius={self.robot_radius}m'
        )

        # pubs/subs
        self.image_sub = self.create_subscription(Image, '/camera/camera/color/image_raw', self.image_cb, 2)
        self.mask_pub = self.create_publisher(Image, '/segmentation/floor_mask', 10)
        self.wall_mask_pub = self.create_publisher(Image, '/segmentation/wall_mask', 10)
        self.overlay_pub = self.create_publisher(Image, '/segmentation/overlay', 10)
        self.seg_image_pub = self.create_publisher(Image, '/segmentation/image', 10)
        self.centroid_pub = self.create_publisher(Point, '/segmentation/centroid', 10)
        self.navigable_pub = self.create_publisher(Bool, '/segmentation/navigable', 10)
        self.conf_pub = self.create_publisher(Float32, '/segmentation/floor_confidence', 10)
        # Camera intrinsics are constants; no CameraInfo subscription
        
        # BEV publishers
        if self.enable_bev:
            self.bev_map_pub = self.create_publisher(Image, '/segmentation/bev_map', 10)
            self.local_goal_pub = self.create_publisher(PointStamped, '/segmentation/local_goal', 10)
            self.rgb_path_overlay_pub = self.create_publisher(Image, '/segmentation/rgb_path_overlay', 20)
            self.get_logger().info('BEV map publisher created on /segmentation/bev_map')
            self.get_logger().info('Local goal publisher created on /segmentation/local_goal')
            self.get_logger().info('RGB path overlay publisher created on /segmentation/rgb_path_overlay')

        # load model (local-first)
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.get_logger().info(f'Loading {self.model_name} on {self.device}')

        loaded = False
        local_path = Path(self.local_model_dir)
        if not local_path.is_absolute():
            local_path = (Path(__file__).resolve().parent / local_path).resolve()

        if local_path.exists():
            # Check if it's a Hugging Face repo structure with snapshots
            repo_dirs = [d for d in local_path.iterdir() if d.is_dir() and 'models--' in d.name]
            if repo_dirs:
                repo_dir = repo_dirs[0]
                snapshots_dir = repo_dir / 'snapshots'
                if snapshots_dir.exists():
                    snapshot_dirs = list(snapshots_dir.iterdir())
                    if snapshot_dirs:
                        model_path = snapshot_dirs[0]  # Use the first (latest) snapshot
                    else:
                        model_path = local_path
                else:
                    model_path = local_path
            else:
                model_path = local_path
            
            try:
                self.processor = AutoImageProcessor.from_pretrained(str(model_path), local_files_only=True)
                self.model = SegformerForSemanticSegmentation.from_pretrained(str(model_path), local_files_only=True).to(self.device)
                loaded = True
                self.get_logger().info(f'Loaded model from {model_path}')
                # Warm up GPU / allocator and optionally FP16
                try:
                    self.model.eval()
                    if self.inference_size:
                        dummy = torch.randn(1, 3, self.inference_size, self.inference_size).to(self.device)
                    else:
                        dummy = torch.randn(1, 3, 512, 512).to(self.device)
                    with torch.no_grad():
                        if self.use_amp and self.device.type == 'cuda':
                            with torch.cuda.amp.autocast():
                                _ = self.model(pixel_values=dummy)
                        else:
                            _ = self.model(pixel_values=dummy)
                except Exception:
                    # Warmup is best-effort; continue even if it fails
                    pass
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
        self.floor_keywords = ['floor', 'ground', 'road', 'pavement', 'sidewalk', 'carpet']
        self.floor_ids = []
        
        # find wall-like classes
        self.wall_keywords = ['wall']
        self.wall_ids = []
        
        id2label = getattr(self.model.config, 'id2label', None)
        if id2label:
            for k, v in id2label.items():
                name = str(v).lower()
                # Check for floor classes
                if any(kw in name for kw in self.floor_keywords):
                    try:
                        self.floor_ids.append(int(k))
                    except Exception:
                        pass
                # Check for wall classes
                if any(kw in name for kw in self.wall_keywords):
                    try:
                        self.wall_ids.append(int(k))
                    except Exception:
                        pass

        self.get_logger().info(f'Floor class ids: {self.floor_ids}')
        self.get_logger().info(f'Wall class ids: {self.wall_ids}')

    def mark_obstacles_with_inflation(self, floor_mask):
        """
        Mark obstacles (non-floor) with red color and inflation zone.
        Only processes within 1 meter range for performance.
        Returns: (overlay_mask, costmap, elapsed_time_ms)
        """
        start_time = time.time()
        
        h, w = floor_mask.shape
        
        # Convert 1 meter to pixels
        one_meter_pixels = int(1.0 * self.pixels_per_meter)
        
        # Only process bottom portion (within 1 meter from camera)
        process_height = min(one_meter_pixels, h)
        
        # Create colored overlay and costmap
        obstacle_overlay = np.zeros((h, w, 3), dtype=np.uint8)
        costmap = np.zeros((h, w), dtype=np.float32)
        
        # Process only the bottom region
        process_region = floor_mask[h - process_height:h, :]
        
        # Calculate distance transform for inflation
        dist_transform = cv2.distanceTransform(process_region, cv2.DIST_L2, 5)
        
        # Convert radii to pixels
        robot_radius_pixels = int((self.robot_footprint_size / 2) * self.pixels_per_meter)
        inflation_radius_pixels = int(self.obstacle_inflation_radius * self.pixels_per_meter)
        total_radius = robot_radius_pixels + inflation_radius_pixels
        
        # Create masks for different zones
        obstacle_mask = process_region == 0
        lethal_mask = (dist_transform > 0) & (dist_transform <= robot_radius_pixels)
        inflation_mask = (dist_transform > robot_radius_pixels) & (dist_transform <= total_radius)
        
        # Create region costmap
        region_costmap = np.zeros((process_height, w), dtype=np.float32)
        
        # Assign costs
        region_costmap[obstacle_mask] = 255.0  # Obstacles - impassable
        region_costmap[lethal_mask] = 254.0    # Lethal zone (robot can't fit)
        
        # Inflation zone - gradient cost (stronger penalty)
        if np.any(inflation_mask):
            distances = dist_transform[inflation_mask]
            # Cost decreases from 250 to 50 as distance increases
            ratios = (total_radius - distances) / (total_radius - robot_radius_pixels + 1e-6)
            region_costmap[inflation_mask] = (250.0 * ratios * self.cost_scaling_factor).clip(50, 250)
        
        # Copy to full costmap
        costmap[h - process_height:h, :] = region_costmap
        
        # Create visualization overlay
        region_overlay = np.zeros((process_height, w, 3), dtype=np.uint8)
        region_overlay[obstacle_mask] = (0, 0, 255)  # Red for obstacles
        
        # Mark inflation zone with orange/yellow gradient
        if np.any(inflation_mask):
            # Gradient from red to yellow based on distance
            distances = dist_transform[inflation_mask]
            ratios = distances / total_radius
            # Orange to yellow gradient
            region_overlay[inflation_mask, 0] = (255 * ratios).astype(np.uint8)  # Blue channel
            region_overlay[inflation_mask, 1] = (165 + 90 * ratios).astype(np.uint8)  # Green channel
            region_overlay[inflation_mask, 2] = 255  # Red channel (full)
        
        # Copy to full image
        obstacle_overlay[h - process_height:h, :] = region_overlay
        
        elapsed_time = (time.time() - start_time) * 1000  # Convert to ms
        
        return obstacle_overlay, costmap, elapsed_time

    def find_floor_path(self, mask, start, end, costmap=None):
        """
        Find a path from start to end that stays on the floor (mask > 0).
        Uses A* pathfinding algorithm with costmap penalties for obstacle avoidance.
        """
        import heapq
        
        h, w = mask.shape
        start_x, start_y = start
        end_x, end_y = end
        
        # If start or end is not on floor, return direct line
        if mask[start_y, start_x] == 0 or mask[end_y, end_x] == 0:
            return [start, end]
        
        use_costmap = costmap is not None
        
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
        
        max_iterations = 2000  # Reduced from 10000 to prevent slow searches
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
                
                # Skip if in lethal zone (cost >= 254)
                if use_costmap and costmap[ny, nx] >= 254:
                    continue
                
                neighbor = (nx, ny)
                
                # Calculate movement cost
                movement_cost = np.sqrt(dx**2 + dy**2)
                
                # Add strong costmap penalty to make paths curve around obstacles
                if use_costmap:
                    cost_value = costmap[ny, nx]
                    # Exponential penalty for high-cost areas
                    if cost_value > 50:
                        # Scale: cost 50-250 becomes penalty 1-20
                        cost_penalty = ((cost_value - 50) / 200.0) ** 2 * 20
                    else:
                        cost_penalty = 0
                    movement_cost += cost_penalty
                
                tentative_g_score = g_score[current] + movement_cost
                
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
        
        # If no path found, return empty list to indicate failure
        return []
    
    def find_floor_path(self, mask, start, end, costmap=None):
        """
        Fast potential fields pathfinding - follows gradient from start to goal.
        Much faster than A* (~1-3ms vs 5-15ms).
        
        Args:
            mask: Binary floor mask (floor=255, obstacle=0)
            start: (x, y) start position
            end: (x, y) goal position
            costmap: Optional costmap (not used in basic potential fields)
        
        Returns:
            path: List of (x, y) waypoints from start to end
        """
        if not self.enable_pathfinding:
            return [start, end]
        
        h, w = mask.shape
        start_x, start_y = start
        end_x, end_y = end
        
        # If start or end is not on floor, return direct line
        if mask[start_y, start_x] == 0 or mask[end_y, end_x] == 0:
            return [start, end]
        
        # Check if direct path is clear
        num_samples = max(abs(end_x - start_x), abs(end_y - start_y))
        if num_samples > 0:
            xs = np.linspace(start_x, end_x, num_samples, dtype=int)
            ys = np.linspace(start_y, end_y, num_samples, dtype=int)
            path_clear = True
            for x, y in zip(xs, ys):
                if 0 <= x < w and 0 <= y < h:
                    if mask[y, x] == 0:
                        path_clear = False
                        break
            if path_clear:
                # Direct path is clear, return straight line with intermediate points
                step = max(1, num_samples // 10)  # ~10 waypoints
                return [(int(x), int(y)) for x, y in zip(xs[::step], ys[::step])] + [end]
        
        # Use potential fields: follow gradient toward goal while avoiding obstacles
        path = [start]
        current = start
        max_steps = 200
        step_size = 5  # pixels per step
        
        for _ in range(max_steps):
            cx, cy = current
            
            # Check if reached goal (within step_size distance)
            dist_to_goal = np.sqrt((cx - end_x)**2 + (cy - end_y)**2)
            if dist_to_goal < step_size:
                path.append(end)
                break
            
            # Attractive force toward goal (normalized)
            dx_attract = end_x - cx
            dy_attract = end_y - cy
            attract_mag = np.sqrt(dx_attract**2 + dy_attract**2)
            if attract_mag > 0:
                dx_attract /= attract_mag
                dy_attract /= attract_mag
            
            # Repulsive force from obstacles (sample nearby pixels)
            dx_repel = 0.0
            dy_repel = 0.0
            repel_radius = 15  # Check obstacles within 15 pixels
            
            for dy in range(-repel_radius, repel_radius + 1, 3):
                for dx in range(-repel_radius, repel_radius + 1, 3):
                    nx, ny = cx + dx, cy + dy
                    if 0 <= nx < w and 0 <= ny < h and mask[ny, nx] == 0:
                        # Obstacle detected, add repulsive force
                        dist = np.sqrt(dx**2 + dy**2) + 1e-6
                        if dist < repel_radius:
                            # Stronger repulsion for closer obstacles
                            strength = (repel_radius - dist) / repel_radius
                            dx_repel -= dx / dist * strength * 2.0
                            dy_repel -= dy / dist * strength * 2.0
            
            # Combine forces (attractive + repulsive)
            dx_total = dx_attract + dx_repel
            dy_total = dy_attract + dy_repel
            
            # Normalize and scale by step size
            total_mag = np.sqrt(dx_total**2 + dy_total**2)
            if total_mag > 0:
                dx_total = dx_total / total_mag * step_size
                dy_total = dy_total / total_mag * step_size
            
            # Calculate next position
            next_x = int(cx + dx_total)
            next_y = int(cy + dy_total)
            
            # Clamp to bounds
            next_x = max(0, min(w - 1, next_x))
            next_y = max(0, min(h - 1, next_y))
            
            # Check if next position is on floor
            if mask[next_y, next_x] == 0:
                # Hit obstacle, try to slide along it
                # Try perpendicular directions
                for angle_offset in [90, -90, 45, -45]:
                    angle = np.arctan2(dy_total, dx_total) + np.deg2rad(angle_offset)
                    test_x = int(cx + np.cos(angle) * step_size)
                    test_y = int(cy + np.sin(angle) * step_size)
                    test_x = max(0, min(w - 1, test_x))
                    test_y = max(0, min(h - 1, test_y))
                    if mask[test_y, test_x] > 0:
                        next_x, next_y = test_x, test_y
                        break
                else:
                    # Still stuck, try smaller step
                    next_x = int(cx + dx_total * 0.5)
                    next_y = int(cy + dy_total * 0.5)
                    if mask[next_y, next_x] == 0:
                        # Give up, return what we have
                        break
            
            # Add to path if it's a new position
            if (next_x, next_y) != current:
                path.append((next_x, next_y))
                current = (next_x, next_y)
            else:
                # Stuck in same position, give up
                break
        
        # If we didn't reach goal, add direct line to goal
        if len(path) < 2 or path[-1] != end:
            path.append(end)
        
        return path
    
    def validate_path(self, path, costmap):
        """
        Validate that the path doesn't go through high-cost obstacles.
        Returns True if path is safe, False if it crosses lethal zones.
        """
        if len(path) < 2:
            return False
        
        if costmap is None:
            return True
        
        h, w = costmap.shape
        
        # Check each point in the path
        for x, y in path:
            if 0 <= x < w and 0 <= y < h:
                # Reject path if it goes through lethal zone (cost >= 254)
                if costmap[y, x] >= 254:
                    return False
        
        return True
    
    def create_smooth_path(self, path, mask):
        """
        Create a smooth curvy path using cubic spline interpolation.
        Ensures the smoothed path still stays on floor pixels.
        """
        # Allow runtime disabling of smoothing
        if not getattr(self, 'enable_smoothing', True):
            return path

        # SciPy interpolation may not be available in all environments
        if splprep is None or splev is None:
            return path
        
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
            tck, u = splprep([x_sampled, y_sampled], s=0, k=min(3, len(x_sampled) - 1))
            
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

    def _preprocess_frame(self, frame):
        """Preprocess frame for model inference."""
        h, w = frame.shape[:2]
        proc_img = frame
        if self.inference_size and (self.inference_size != min(h, w)):
            proc_img = cv2.resize(frame, (self.inference_size, self.inference_size), interpolation=cv2.INTER_AREA)
        
        if self.profile_timing:
            t0 = time.perf_counter()
        inputs = self.processor(images=proc_img, return_tensors='pt')
        pix = inputs['pixel_values'].to(self.device)
        if self.profile_timing:
            t_preproc = time.perf_counter() - t0
            return pix, t_preproc
        return pix, 0.0
    
    def _run_inference(self, pix):
        """Run model inference and return probabilities."""
        if self.profile_timing:
            t1 = time.perf_counter()
        with torch.no_grad():
            if self.use_amp and self.device.type == 'cuda':
                with torch.cuda.amp.autocast():
                    out = self.model(pixel_values=pix)
            else:
                out = self.model(pixel_values=pix)
        probs = torch.softmax(out.logits, dim=1)
        if self.profile_timing:
            t_model = time.perf_counter() - t1
            return probs, t_model
        return probs, 0.0
    
    def _upsample_and_transfer(self, probs, h, w):
        """Transfer floor and wall classes at model resolution, then upsample on CPU."""
        if self.profile_timing:
            t_upsample_start = time.perf_counter()
        
        # OPTIMIZATION: Combine floor/wall probs at low-res, then upsample once (much faster!)
        probs_np = probs[0].cpu().numpy()  # Shape: (150, 512, 512) or smaller
        
        # Combine floor class probabilities at model resolution (max pooling)
        if self.floor_ids:
            floor_prob_combined = np.max(probs_np[self.floor_ids], axis=0)  # (512, 512)
        else:
            floor_prob_combined = probs_np[0]  # Fallback
        
        # Combine wall class probabilities at model resolution
        if self.wall_ids:
            wall_prob_combined = np.max(probs_np[self.wall_ids], axis=0)  # (512, 512)
        else:
            wall_prob_combined = np.zeros_like(floor_prob_combined)
        
        # Upsample combined probabilities using INTER_NEAREST (4x faster than INTER_LINEAR for masks)
        floor_prob_up = cv2.resize(floor_prob_combined, (w, h), interpolation=cv2.INTER_NEAREST)
        wall_prob_up = cv2.resize(wall_prob_combined, (w, h), interpolation=cv2.INTER_NEAREST)
        
        # Create sparse probs_up array (only store combined floor/wall in first slots)
        probs_up = np.zeros((probs_np.shape[0], h, w), dtype=np.float32)
        if self.floor_ids:
            probs_up[self.floor_ids[0]] = floor_prob_up  # Store in first floor ID
        if self.wall_ids:
            probs_up[self.wall_ids[0]] = wall_prob_up  # Store in first wall ID
        
        if self.profile_timing:
            t_upsample = (time.perf_counter() - t_upsample_start) * 1000
            return probs_up, t_upsample
        return probs_up, 0.0
    
    def _create_floor_mask(self, probs_up, h, w):
        """Create binary floor mask from probabilities."""
        if self.profile_timing:
            t_mask_start = time.perf_counter()
        # Combined floor probability is stored in first floor ID
        if self.floor_ids:
            mask = (probs_up[self.floor_ids[0]] > self.prob_threshold).astype(np.uint8) * 255
        else:
            mask = np.zeros((h, w), dtype=np.uint8)
        if self.profile_timing:
            t_mask_create = (time.perf_counter() - t_mask_start) * 1000
            return mask, t_mask_create
        return mask, 0.0
    
    def _create_wall_mask(self, probs_up, h, w):
        """Create binary wall mask from probabilities."""
        if self.profile_timing:
            t_mask_start = time.perf_counter()
        # Combined wall probability is stored in first wall ID
        if self.wall_ids:
            mask = (probs_up[self.wall_ids[0]] > self.prob_threshold).astype(np.uint8) * 255
        else:
            mask = np.zeros((h, w), dtype=np.uint8)
        if self.profile_timing:
            t_mask_create = (time.perf_counter() - t_mask_start) * 1000
            return mask, t_mask_create
        return mask, 0.0
    
    def _apply_morphology(self, mask):
        """Apply morphological operations to clean up the mask."""
        if self.profile_timing:
            t_morph_start = time.perf_counter()
        kernel = np.ones((self.morph_kernel, self.morph_kernel), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        if self.profile_timing:
            t_morph = (time.perf_counter() - t_morph_start) * 1000
            return mask, t_morph
        return mask, 0.0
    
    def _filter_by_area(self, mask):
        """Filter out small connected components."""
        if self.profile_timing:
            t_cc_start = time.perf_counter()
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
        for i in range(1, num_labels):
            if stats[i, cv2.CC_STAT_AREA] < self.min_area:
                mask[labels == i] = 0
        if self.profile_timing:
            t_cc = (time.perf_counter() - t_cc_start) * 1000
            return mask, t_cc
        return mask, 0.0
    
    def _create_overlay_image(self, frame, floor_mask, wall_mask):
        """Create visualization overlay with floor highlighted in green and walls in blue."""
        if self.profile_timing:
            t_overlay_start = time.perf_counter()
        
        # OPTIMIZED: Use cv2.addWeighted for fast blending instead of per-pixel float operations
        h, w = frame.shape[:2]
        
        # Create colored masks
        wall_color_mask = np.zeros((h, w, 3), dtype=np.uint8)
        wall_color_mask[wall_mask > 0] = (255, 0, 0)  # Blue walls (BGR)
        
        floor_color_mask = np.zeros((h, w, 3), dtype=np.uint8)
        floor_color_mask[floor_mask > 0] = (0, 255, 0)  # Green floor (BGR)
        
        # Fast blending using cv2.addWeighted (hardware-accelerated)
        overlay = cv2.addWeighted(frame, 0.6, wall_color_mask, 0.4, 0)
        overlay = cv2.addWeighted(overlay, 1.0, floor_color_mask, 0.4, 0)
        
        if self.profile_timing:
            t_overlay_create = (time.perf_counter() - t_overlay_start) * 1000
            return overlay, t_overlay_create
        return overlay, 0.0
    
    def _detect_obstacles(self, mask, h, w):
        """Detect and mark obstacles with inflation zones."""
        total_start = time.time()
        if self.enable_obstacle_detection:
            obstacle_overlay, costmap, obstacle_time = self.mark_obstacles_with_inflation(mask)
        else:
            obstacle_overlay = np.zeros((h, w, 3), dtype=np.uint8)
            costmap = None
            obstacle_time = 0.0
        return obstacle_overlay, costmap, obstacle_time
    
    def _find_navigation_goal(self, mask, cx, cy, h, w):
        """Find the navigation goal point on the floor."""
        goal_x = cx
        goal_y = cy
        search_radius = 100
        
        if not (0 <= goal_x < w and 0 <= goal_y < h and mask[goal_y, goal_x] > 0):
            found = False
            # Expand search in rings around the centroid (skip if disabled)
            if self.enable_centroid_search:
                for r in range(1, search_radius + 1):
                    # Top/bottom edges of the ring
                    for dx in range(-r, r + 1):
                        for dy in (-r, r):
                            x = cx + dx
                            y = cy + dy
                            if 0 <= x < w and 0 <= y < h and mask[y, x] > 0:
                                goal_x = x
                                goal_y = y
                                found = True
                                break
                        if found:
                            break
                    if found:
                        break
                
                if not found:
                    # Left/right edges of the ring
                    for r in range(1, search_radius + 1):
                        for dy in range(-r + 1, r):
                            for dx in (-r, r):
                                x = cx + dx
                                y = cy + dy
                                if 0 <= x < w and 0 <= y < h and mask[y, x] > 0:
                                    goal_x = x
                                    goal_y = y
                                    found = True
                                    break
                            if found:
                                break
                        if found:
                            break
            
            # Fallback: search for furthest floor point
            if not found:
                goal_x = cx
                goal_y = h - 1
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
        
        return goal_x, goal_y
    
    def _compute_navigation_path(self, mask, start_x, start_y, goal_x, goal_y):
        """Compute navigation path from start to goal."""
        if self.enable_pathfinding:
            path_start_time = time.time()
            path = self.find_floor_path(mask, (start_x, start_y), (goal_x, goal_y), None)
            path_time = (time.time() - path_start_time) * 1000
        else:
            path = [(start_x, start_y), (goal_x, goal_y)]
            path_time = 0.0
        return path, path_time
    
    def _draw_navigation_visualization(self, overlay, mask, path, start_x, start_y, cx, cy, goal_x, goal_y, obstacle_time, path_time, total_time):
        """Draw all navigation visualization elements."""
        path_is_safe = self.validate_path(path, None)
        
        if path_is_safe and len(path) > 2:
            # Create smooth path
            if self.enable_smoothing:
                smooth_path = self.create_smooth_path(path, mask)
            else:
                smooth_path = path
            
            # Draw path
            if self.enable_drawing and len(smooth_path) > 1:
                for i in range(len(smooth_path) - 1):
                    cv2.line(overlay, smooth_path[i], smooth_path[i + 1], (255, 0, 0), 8)
                    cv2.line(overlay, smooth_path[i], smooth_path[i + 1], (255, 0, 0), 30)
            
            # Draw markers
            if self.enable_drawing:
                cv2.circle(overlay, (goal_x, goal_y), 10, (0, 0, 255), -1)  # Goal - Red
                cv2.circle(overlay, (cx, cy), 8, (255, 0, 255), -1)  # Centroid - Magenta
                cv2.circle(overlay, (start_x, start_y), 8, (255, 255, 0), -1)  # Start - Yellow
        else:
            # Path blocked warning
            # warning_text = "PATH BLOCKED - No Safe Route"
            pass
        
        # Display timing information
        timing_text = f"Obstacle: {obstacle_time:.1f}ms | Path: {path_time:.1f}ms | Total: {total_time:.1f}ms"
        cv2.putText(overlay, timing_text, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(overlay, timing_text, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)
    
    def _update_fps_stats(self):
        """Update and log FPS statistics."""
        if not self.show_fps:
            return
        
        now = time.perf_counter()
        if self.last_frame_time is not None:
            dt = now - self.last_frame_time
            if dt > 0:
                fps = 1.0 / dt
                if self.fps_ema == 0.0:
                    self.fps_ema = fps
                else:
                    self.fps_ema = 0.9 * self.fps_ema + 0.1 * fps
        self.last_frame_time = now
        
        self.frame_processed_count += 1
        if self.frame_processed_count % 10 == 0 and self.fps_ema > 0.0:
            self.get_logger().info(f'SegFormer FPS: {self.fps_ema:.1f}')
    
    def _log_profiling_info(self, t_preproc, t_model, t_upsample, t_mask_create, t_morph, t_cc, t_overlay_create, t_bev, t_raycast, t_centroid, t_publish):
        """Log profiling information in single line."""
        if not self.profile_timing:
            return
        
        self._profile_count += 1
        if self._profile_count % 20 == 0:
            self.get_logger().info(
                f'Timing(ms): pre={t_preproc*1000:.1f} model={t_model*1000:.1f} upsample={t_upsample:.1f} mask={t_mask_create:.1f} '
                f'morph={t_morph:.1f} cc={t_cc:.1f} overlay={t_overlay_create:.1f} bev={t_bev:.1f} raycast={t_raycast:.1f} '
                f'centroid={t_centroid:.1f} publish={t_publish:.1f}'
            )

    def project_masks_to_bev_ipm(self, floor_mask, wall_mask):
        """Project camera-space floor and wall masks into a metric BEV grid using an IPM-like projection.

        Returns a tuple of (bev_floor, bev_wall) binary masks in robot-centered BEV coordinates.
        This function performs axis alignment (world->camera axes), applies pitch (tilt) rotation,
        and uses camera intrinsics to project points into image coordinates for remapping.
        If projection is invalid or empty, it logs diagnostics and falls back to a simple resize.
        """
        # Local timing could be helpful to profile separately from build_bev_map
        ih, iw = floor_mask.shape

        # Intrinsics: use provided fx/fy or derive from FOV
        fx = self.camera_fx if getattr(self, 'camera_fx', 0) and self.camera_fx > 0 else None
        fy = self.camera_fy if getattr(self, 'camera_fy', 0) and self.camera_fy > 0 else None
        cx = self.camera_cx if self.camera_cx and self.camera_cx > 0 else (iw / 2.0)
        cy = self.camera_cy if self.camera_cy and self.camera_cy > 0 else (ih / 2.0)
        if fx is None or fy is None:
            fov_rad = math.radians(self.camera_fov_deg)
            fx = fy = iw / (2.0 * math.tan(fov_rad / 2.0))

        # Prepare BEV grid positions (world coords in robot base_link frame)
        cols = np.arange(self.bev_pixels)
        rows = np.arange(self.bev_pixels)
        grid_c, grid_r = np.meshgrid(cols, rows)  # c: lateral, r: forward index

        # Map grid indices to world coordinates (meters) - X forward, Y left
        Xw = (self.bev_pixels - 1 - grid_r).astype(np.float32) * self.bev_resolution
        Yw = (grid_c - (self.bev_pixels // 2)).astype(np.float32) * self.bev_resolution
        Zw = np.zeros_like(Xw)

        # Camera pose in robot frame: located at origin offset in z
        Cw = np.array([0.0, 0.0, float(self.camera_height)], dtype=np.float32)

        # Translate points to camera-relative coordinates (Pw)
        Pw_x = Xw - Cw[0]
        Pw_y = Yw - Cw[1]
        Pw_z = Zw - Cw[2]

        # First, align world axes to camera axes (no rotation):
        # camera x (right)  = -world Y
        # camera y (down)   = -world Z
        # camera z (forward)=  world X
        Pc0_x = Pw_y
        Pc0_y = -Pw_z
        Pc0_z = Pw_x

        # Apply tilt rotation about camera x-axis to pitch the camera down
        tilt_rad = math.radians(self.camera_tilt_deg)
        cos_t = math.cos(tilt_rad)
        sin_t = math.sin(tilt_rad)
        # Rotation about X: [1 0 0; 0 cos -sin; 0 sin cos]
        Pc_x = Pc0_x
        Pc_y = cos_t * Pc0_y - sin_t * Pc0_z
        Pc_z = sin_t * Pc0_y + cos_t * Pc0_z

        # Project to image pixel coordinates (account for very small depth to avoid divide-by-zero)
        denom = Pc_z + 1e-9
        u = (fx * (Pc_x / denom) + cx).astype(np.float32)
        v = (fy * (Pc_y / denom) + cy).astype(np.float32)

        # Valid points: in front of the camera and inside source image bounds
        valid = (Pc_z > 1e-6) & (u >= 0) & (u < iw - 1) & (v >= 0) & (v < ih - 1)

        # Remap to BEV grid
        map_x = u
        map_y = v
        bev_floor = cv2.remap(floor_mask, map_x, map_y, interpolation=cv2.INTER_NEAREST,
                              borderMode=cv2.BORDER_CONSTANT, borderValue=0)
        bev_wall = cv2.remap(wall_mask, map_x, map_y, interpolation=cv2.INTER_NEAREST,
                             borderMode=cv2.BORDER_CONSTANT, borderValue=0)

        bev_floor[~valid] = 0
        bev_wall[~valid] = 0

        # Diagnostic info
        valid_count = int(np.count_nonzero(valid))
        num_floor = int(np.count_nonzero(bev_floor))
        num_wall = int(np.count_nonzero(bev_wall))
        min_pcz, max_pcz = float(np.min(Pc_z)) if Pc_z.size > 0 else 0.0, float(np.max(Pc_z)) if Pc_z.size > 0 else 0.0
        min_u, max_u = float(np.min(u)) if u.size > 0 else 0.0, float(np.max(u)) if u.size > 0 else 0.0
        min_v, max_v = float(np.min(v)) if v.size > 0 else 0.0, float(np.max(v)) if v.size > 0 else 0.0
        input_floor_count = int(np.count_nonzero(floor_mask))
        input_wall_count = int(np.count_nonzero(wall_mask))

        if valid_count == 0 or (num_floor == 0 and num_wall == 0):
            now = time.time()
            if now - getattr(self, '_last_ipm_warn_time', 0.0) > 2.0:
                self._last_ipm_warn_time = now
                self.get_logger().warn(
                    'IPM mapping produced empty BEV (no valid samples); falling back to simple resize; '
                    f'fx={fx:.3f}, fy={fy:.3f}, cx={cx:.3f}, cy={cy:.3f}, iw={iw}, ih={ih}, tilt_deg={self.camera_tilt_deg}, height={self.camera_height}, valid_count={valid_count}, '
                    f'num_floor={num_floor}, num_wall={num_wall}, Pc_z=[{min_pcz:.3f},{max_pcz:.3f}], u=[{min_u:.3f},{max_u:.3f}], v=[{min_v:.3f},{max_v:.3f}]'
                )
                self.get_logger().warn(f'Input mask counts: floor={input_floor_count}, wall={input_wall_count}')
                try:
                    cols_mid = self.bev_pixels // 2
                    rows = [self.bev_pixels - 1, self.bev_pixels // 2, 0]
                    samples = []
                    for r in rows:
                        c = cols_mid
                        Xs = (self.bev_pixels - 1 - r) * self.bev_resolution
                        Ys = (c - (self.bev_pixels // 2)) * self.bev_resolution
                        Zs = 0.0
                        Pw_xs = Xs - Cw[0]
                        Pw_ys = Ys - Cw[1]
                        Pw_zs = Zs - Cw[2]
                        Pc0_xs = -Pw_ys
                        Pc0_ys = -Pw_zs
                        Pc0_zs = Pw_xs
                        Pc_xs = Pc0_xs
                        Pc_ys = cos_t * Pc0_ys - sin_t * Pc0_zs
                        Pc_zs = sin_t * Pc0_ys + cos_t * Pc0_zs
                        samples.append((r, c, Xs, Ys, float(Pc_xs), float(Pc_ys), float(Pc_zs)))
                    self.get_logger().warn(f'Sample points (row,col,Xm,Ym,Pc_x,Pc_y,Pc_z): {samples}')
                except Exception:
                    pass
            # Fall back to naive resize if we detected an invalid mapping
            bev_floor = cv2.resize(floor_mask, (self.bev_pixels, self.bev_pixels), interpolation=cv2.INTER_LINEAR)
            bev_wall = cv2.resize(wall_mask, (self.bev_pixels, self.bev_pixels), interpolation=cv2.INTER_LINEAR)
            bev_floor = (bev_floor > 127).astype(np.uint8) * 255
            bev_wall = (bev_wall > 127).astype(np.uint8) * 255

        # Ensure binary masks returned
        bev_floor = (bev_floor > 127).astype(np.uint8) * 255
        bev_wall = (bev_wall > 127).astype(np.uint8) * 255
        return bev_floor, bev_wall

    def bev_cell_to_image_coords(self, r, c, ih=None, iw=None):
        """
        Convert a BEV cell index (r,c) to image pixel coordinates (u,v) by applying
        the same projection used in project_masks_to_bev_ipm.

        Returns (u, v, valid) where u,v are ints for image pixel coordinate and
        valid is a boolean indicating the point projects inside the image bounds
        and in front of the camera.
        """
        # Read image size from parameters if not provided
        if ih is None or iw is None:
            # Try to determine from last seen frame size; fallback to camera default
            ih = getattr(self, 'camera_image_h', None)
            iw = getattr(self, 'camera_image_w', None)
            if ih is None or iw is None:
                # Can't reliably compute; return invalid
                return None, None, False

        # Intrinsics
        fx = self.camera_fx if getattr(self, 'camera_fx', 0) and self.camera_fx > 0 else None
        fy = self.camera_fy if getattr(self, 'camera_fy', 0) and self.camera_fy > 0 else None
        cx = self.camera_cx if self.camera_cx and self.camera_cx > 0 else (iw / 2.0)
        cy = self.camera_cy if self.camera_cy and self.camera_cy > 0 else (ih / 2.0)
        if fx is None or fy is None:
            fov_rad = math.radians(self.camera_fov_deg)
            fx = fy = iw / (2.0 * math.tan(fov_rad / 2.0))

        # Convert BEV cell index to world meters (X forward, Y left)
        Xw = (self.bev_pixels - 1 - r) * self.bev_resolution
        Yw = (c - (self.bev_pixels // 2)) * self.bev_resolution
        Zw = 0.0

        # Camera pose in robot frame
        Cw = np.array([0.0, 0.0, float(self.camera_height)], dtype=np.float32)

        # Translate to camera-relative coordinates
        Pw_x = Xw - Cw[0]
        Pw_y = Yw - Cw[1]
        Pw_z = Zw - Cw[2]

        # Axis alignment (consistent with project_masks_to_bev_ipm)
        Pc0_x = Pw_y
        Pc0_y = -Pw_z
        Pc0_z = Pw_x

        # Tilt rotation about camera x-axis
        tilt_rad = math.radians(self.camera_tilt_deg)
        cos_t = math.cos(tilt_rad)
        sin_t = math.sin(tilt_rad)
        Pc_x = Pc0_x
        Pc_y = cos_t * Pc0_y - sin_t * Pc0_z
        Pc_z = sin_t * Pc0_y + cos_t * Pc0_z

        # Projection
        if Pc_z <= 1e-6:
            return None, None, False

        u = (fx * (Pc_x / Pc_z) + cx)
        v = (fy * (Pc_y / Pc_z) + cy)

        valid = (u >= 0 and u < iw and v >= 0 and v < ih)
        if not valid:
            return None, None, False

        return int(round(u)), int(round(v)), True

    def build_bev_map(self, floor_mask, wall_mask):
        """
        Build Bird's Eye View map by simply rotating the camera view masks.
        No complex transformations - just flip to create top-down perspective.
        
        Args:
            floor_mask: Binary floor mask (H×W) from camera view
            wall_mask: Binary wall mask (H×W) from camera view
            
        Returns:
            bev_floor: BEV floor map (bev_pixels × bev_pixels)
            bev_wall: BEV wall map (bev_pixels × bev_pixels)
            elapsed_ms: Time taken in milliseconds
        """
        start_time = time.perf_counter()
        
        h, w = floor_mask.shape
        
        # Simple approach: Take bottom portion of the image (ground area)
        # and resize to BEV grid size
        # Bottom of camera view = close to robot, top = far from robot
        
        # If IPM/homography is disabled, fall back to resize
        if not getattr(self, 'use_ipm', False):
            bev_floor = cv2.resize(floor_mask, (self.bev_pixels, self.bev_pixels), interpolation=cv2.INTER_LINEAR)
            bev_wall = cv2.resize(wall_mask, (self.bev_pixels, self.bev_pixels), interpolation=cv2.INTER_LINEAR)
            bev_floor = (bev_floor > 127).astype(np.uint8) * 255
            bev_wall = (bev_wall > 127).astype(np.uint8) * 255
            elapsed_ms = (time.perf_counter() - start_time) * 1000
            return bev_floor, bev_wall, elapsed_ms

        # For IPM we call dedicated helper function which returns properly projected BEV masks or raises
        try:
            bev_floor, bev_wall = self.project_masks_to_bev_ipm(floor_mask, wall_mask)
        except Exception:
            # Fallback to simple resizing when IPM fails
            bev_floor = cv2.resize(floor_mask, (self.bev_pixels, self.bev_pixels), interpolation=cv2.INTER_LINEAR)
            bev_wall = cv2.resize(wall_mask, (self.bev_pixels, self.bev_pixels), interpolation=cv2.INTER_LINEAR)
            bev_floor = (bev_floor > 127).astype(np.uint8) * 255
            bev_wall = (bev_wall > 127).astype(np.uint8) * 255
        
        # Threshold to ensure binary masks
        bev_floor = (bev_floor > 127).astype(np.uint8) * 255
        bev_wall = (bev_wall > 127).astype(np.uint8) * 255
        
        elapsed_ms = (time.perf_counter() - start_time) * 1000
        
        return bev_floor, bev_wall, elapsed_ms

    def raycast_direction(self, bev_floor, bev_wall, angle_deg):
        """
        Cast a ray from robot position along specified direction to find free travel distance.
        Starts from the first visible floor pixel instead of robot center to avoid blind spot.
        
        Args:
            bev_floor: BEV floor mask (bev_pixels × bev_pixels)
            bev_wall: BEV wall mask (bev_pixels × bev_pixels)
            angle_deg: Direction angle in degrees (-30 to +30, 0=forward)
            
        Returns:
            tuple: (total_distance, raycast_distance, start_x, start_y)
                - total_distance: Gap + raycast distance in meters (for path planning)
                - raycast_distance: Actual raycasting distance in meters (for visualization)
                - start_x, start_y: First floor pixel position (for visualization)
        """
        # Robot is at bottom-center of BEV
        robot_x = self.bev_pixels // 2  # Center horizontally
        robot_y = self.bev_pixels - 1   # Bottom row
        
        # Convert angle to radians and create direction vector
        angle_rad = np.deg2rad(angle_deg)
        dx = np.sin(angle_rad)  # Horizontal component
        dy = -np.cos(angle_rad)  # Vertical component (negative because y increases downward)
        
        # Find first floor pixel along the ray direction to skip blind spot gap
        start_x = robot_x
        start_y = robot_y
        max_search = int(1.0 / self.bev_resolution)  # Search up to 1 meter ahead
        
        for search_step in range(1, max_search + 1):
            check_x = int(robot_x + dx * search_step)
            check_y = int(robot_y + dy * search_step)
            
            # Check bounds
            if not (0 <= check_x < self.bev_pixels and 0 <= check_y < self.bev_pixels):
                break
            
            # Found first floor pixel!
            if bev_floor[check_y, check_x] > 0 and bev_wall[check_y, check_x] == 0:
                start_x = check_x
                start_y = check_y
                break
        
        # March along ray from first floor position
        distance = 0.0
        step_size = 1  # pixels
        max_steps = int(self.bev_size_meters / self.bev_resolution)
        
        for step in range(1, max_steps + 1):  # Start from 1 to move away from start point
            # Current position along ray
            curr_x = int(start_x + dx * step * step_size)
            curr_y = int(start_y + dy * step * step_size)
            
            # Check bounds
            if not (0 <= curr_x < self.bev_pixels and 0 <= curr_y < self.bev_pixels):
                break
            
            # Check for obstacles (wall or non-floor)
            if bev_wall[curr_y, curr_x] > 0 or bev_floor[curr_y, curr_x] == 0:
                break
            
            # Update distance (successful step)
            distance = step * step_size * self.bev_resolution
        
        # Add the distance from robot to start point
        gap_distance = np.sqrt((start_x - robot_x)**2 + (start_y - robot_y)**2) * self.bev_resolution
        total_distance = gap_distance + distance
        
        return total_distance, distance, start_x, start_y
    
    def sample_directions_and_raycast(self, bev_floor, bev_wall):
        """
    Sample N directions (default 15) and raycast to find free distance for each.
        
        Args:
            bev_floor: BEV floor mask
            bev_wall: BEV wall mask
            
        Returns:
            angles: List of angles in degrees (samples across -30 to +30)
            free_distances: List of free distances in meters for each angle
            raycast_data: List of (raycast_distance, start_x, start_y) for visualization
            elapsed_ms: Time taken in milliseconds
        """
        start_time = time.perf_counter()
        
        # Sample directions evenly across [-30deg, +30deg]
        num_rays = 8
        angles = list(np.linspace(-40.0, 40.0, num_rays))
        free_distances = []
        raycast_data = []  # Store (raycast_distance, start_x, start_y) for each ray
        
        for angle in angles:
            total_dist, raycast_dist, start_x, start_y = self.raycast_direction(bev_floor, bev_wall, angle)
            free_distances.append(total_dist)  # Use total distance for path planning
            raycast_data.append((raycast_dist, start_x, start_y))  # Store for visualization
        
        elapsed_ms = (time.perf_counter() - start_time) * 1000
        
        return angles, free_distances, raycast_data, elapsed_ms
    
    def compute_path_scores(self, angles, free_distances, bev_floor, bev_wall):
        """
        Compute scores for each candidate path based on distance, smoothness, and frontier.
        
        Args:
            angles: List of sampled angles
            free_distances: List of free distances for each angle
            bev_floor: BEV floor mask
            bev_wall: BEV wall mask
            
        Returns:
            scores: List of normalized scores [0-1] for each path
            best_idx: Index of the best path
        """
        scores = []
        robot_x = self.bev_pixels // 2
        robot_y = self.bev_pixels - 1
        
        for i, (angle, distance) in enumerate(zip(angles, free_distances)):
            # 1. Distance score: normalized by max range
            distance_score = distance / self.bev_size_meters
            
            # 2. Smoothness score: prefer forward (0°), penalize turns
            # Score decreases as angle deviates from 0°
            smoothness_score = 1.0 - (abs(angle) / 90.0)  # 90° is max penalty
            
            # 3. Frontier score: check if path ends at unknown (not wall, not floor)
            angle_rad = np.deg2rad(angle)
            dx = np.sin(angle_rad)
            dy = -np.cos(angle_rad)
            
            # Get endpoint
            distance_px = int(distance / self.bev_resolution)
            end_x = int(robot_x + dx * distance_px)
            end_y = int(robot_y + dy * distance_px)
            end_x = max(0, min(self.bev_pixels - 1, end_x))
            end_y = max(0, min(self.bev_pixels - 1, end_y))
            
            # Check if endpoint hits wall or unknown
            if distance > 0:
                # If we have some distance, check what stopped us
                # Look slightly ahead of endpoint
                check_x = int(robot_x + dx * (distance_px + 2))
                check_y = int(robot_y + dy * (distance_px + 2))
                
                if 0 <= check_x < self.bev_pixels and 0 <= check_y < self.bev_pixels:
                    if bev_wall[check_y, check_x] > 0:
                        frontier_score = 0.1  # Hit wall - less interesting
                    elif bev_floor[check_y, check_x] == 0:
                        frontier_score = 1.0  # Hit unknown - explore!
                    else:
                        frontier_score = 0.1  # Still on floor
                else:
                    frontier_score = 0.8  # Out of bounds - potential exploration
            else:
                frontier_score = 0.0  # No distance - blocked immediately
            
            # 4. Combined score with weights
            # Distance: 50%, Smoothness: 30%, Frontier: 20%
            combined_score = (0.2 * distance_score + 
                            0.1 * smoothness_score + 
                            0.7 * frontier_score)
            
            scores.append(combined_score)
        
        # Find best path
        best_idx = int(np.argmax(scores)) if scores else 0
        
        return scores, best_idx

    def _create_rgb_path_overlay(self, frame, floor_mask, angles, free_distances, scores, best_idx, 
                                   goal_x, goal_y, best_distance, bev_floor=None, bev_wall=None):
        """
        Create RGB camera overlay showing A* planned path to goal.
        Raycasting is used only for decision making (selecting goal), not for display.
        
        Args:
            frame: Original RGB camera image
            floor_mask: Floor segmentation mask for A* path planning
            angles: List of path angles in degrees (for goal selection only)
            free_distances: List of free distances for each path (in meters)
            scores: List of path scores
            best_idx: Index of best path
            goal_x: Goal X coordinate in base_link frame (meters)
            goal_y: Goal Y coordinate in base_link frame (meters)
            best_distance: Free distance of best path (meters)
            
        Returns:
            overlay: RGB image with A* path and goal visualization
        """
        overlay = frame.copy()
        h, w = overlay.shape[:2]
        
        # OPTIMIZATION: Downsample floor mask for faster A* (4x smaller = 16x fewer cells)
        scale = 4
        floor_small = cv2.resize(floor_mask, (w // scale, h // scale), interpolation=cv2.INTER_NEAREST)
        h_small, w_small = floor_small.shape
        
        # Robot position (bottom center of image)
        robot_img_x = w // 2
        robot_img_y = h - 1
        robot_small_x = w_small // 2
        robot_small_y = h_small - 1
        
        # Project goal from base_link (meters) to image coordinates
        # Using simple perspective projection:
        # - Y (lateral) maps to horizontal displacement
        # - X (forward) maps to vertical position (larger X = higher in image)
        
        # Scale factors (approximate, tune as needed)
        pixels_per_meter_x = h / self.bev_size_meters  # Forward direction
        pixels_per_meter_y = w / (self.bev_size_meters * 1.5)  # Lateral direction (wider FOV)
        
        # Calculate goal position in image
        # X (forward) moves up in image, Y (lateral) moves horizontally
        goal_img_x = int(robot_img_x + goal_y * pixels_per_meter_y)
        goal_img_y = int(robot_img_y - goal_x * pixels_per_meter_x)
        
        # Clamp to image bounds
        goal_img_x = max(0, min(w - 1, goal_img_x))
        goal_img_y = max(0, min(h - 1, goal_img_y))
        
        # If BEV goal is available, project it to RGB and run A* on floor_mask
        path_img = None
        if bev_floor is not None:
            try:
                # Convert goal from base_link (meters) to BEV grid indices
                goal_bev_r = int(self.bev_pixels - 1 - (goal_x / self.bev_resolution))
                goal_bev_c = int((goal_y / self.bev_resolution) + (self.bev_pixels // 2))
                # Clamp
                goal_bev_r = max(0, min(self.bev_pixels - 1, goal_bev_r))
                goal_bev_c = max(0, min(self.bev_pixels - 1, goal_bev_c))

                # Project BEV goal to RGB image coordinates
                ih, iw = frame.shape[:2]
                gu, gv, gvalid = self.bev_cell_to_image_coords(goal_bev_r, goal_bev_c, ih=ih, iw=iw)
                
                if gvalid:
                    # Use projected goal coordinates
                    goal_img_x = gu
                    goal_img_y = gv
                    goal_small_x = goal_img_x // scale
                    goal_small_y = goal_img_y // scale
                    
                    # If projected goal is not on floor, search nearby for floor pixel (on small mask)
                    if 0 <= goal_small_y < h_small and 0 <= goal_small_x < w_small and floor_small[goal_small_y, goal_small_x] == 0:
                        found = False
                        search_radius = 12  # Reduced for downsampled image
                        for r in range(1, search_radius + 1):
                            for dy in range(-r, r + 1):
                                for dx in range(-r, r + 1):
                                    gy = goal_small_y + dy
                                    gx = goal_small_x + dx
                                    if 0 <= gx < w_small and 0 <= gy < h_small and floor_small[gy, gx] > 0:
                                        goal_small_x = gx
                                        goal_small_y = gy
                                        goal_img_x = gx * scale
                                        goal_img_y = gy * scale
                                        found = True
                                        break
                                if found:
                                    break
                            if found:
                                break
                    
                    # Run A* on downsampled floor mask (much faster!)
                    start_small = (robot_small_x, robot_small_y)
                    end_small = (goal_small_x, goal_small_y)
                    path_small = self.find_floor_path(floor_small, start_small, end_small)
                    
                    # Scale path back to full resolution
                    path_img = [(x * scale, y * scale) for x, y in path_small]
                    
                    # Log only occasionally to reduce spam
                    if self.rgb_overlay_counter % 40 == 0:
                        try:
                            self.get_logger().info(f'RGB overlay: BEV goal at ({goal_img_x}, {goal_img_y}), path={len(path_img)} pts')
                        except Exception:
                            pass
            except Exception as e:
                try:
                    self.get_logger().warn(f'BEV goal projection failed: {e}')
                except Exception:
                    pass
                path_img = None

        # Fallback: use simple perspective projection for goal
        if path_img is None:
            goal_small_x = goal_img_x // scale
            goal_small_y = goal_img_y // scale
            start_small = (robot_small_x, robot_small_y)
            end_small = (goal_small_x, goal_small_y)
            path_small = self.find_floor_path(floor_small, start_small, end_small)
            # Scale path back to full resolution
            path_img = [(x * scale, y * scale) for x, y in path_small]
        
        # Draw A* path (projected from BEV if available, else image path)
        if len(path_img) > 1:
            # Draw path as thick green line
            for i in range(len(path_img) - 1):
                pt1 = path_img[i]
                pt2 = path_img[i + 1]
                
                # Gradient color from yellow (start) to green (end)
                progress = i / (len(path_img) - 1)
                color_b = int(0)
                color_g = int(255)
                color_r = int((1 - progress) * 255)  # Yellow to green
                color = (color_b, color_g, color_r)
                
                # Draw with transparency
                temp_overlay = overlay.copy()
                cv2.line(temp_overlay, pt1, pt2, color, 4)
                cv2.addWeighted(temp_overlay, 0.7, overlay, 0.3, 0, overlay)
            
            # Draw waypoint markers along path
            step = max(1, len(path_img) // 10)  # Show ~10 waypoints
            for i in range(0, len(path_img), step):
                pt = path_img[i]
                cv2.circle(overlay, pt, 3, (0, 255, 0), -1)
                cv2.circle(overlay, pt, 4, (255, 255, 255), 1)
        
        # Draw goal marker: large green circle with cross
        cv2.circle(overlay, (goal_img_x, goal_img_y), 20, (0, 255, 0), 3)
        cv2.drawMarker(overlay, (goal_img_x, goal_img_y), (0, 255, 0), 
                      cv2.MARKER_CROSS, 30, 4)
        
        # Add goal label
        goal_label = f"GOAL"
        goal_coords = f"({goal_x:.2f}, {goal_y:.2f})m"
        cv2.putText(overlay, goal_label, (goal_img_x + 25, goal_img_y - 10), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(overlay, goal_coords, (goal_img_x + 25, goal_img_y + 10), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1, cv2.LINE_AA)
        
        # Draw robot indicator at bottom
        cv2.circle(overlay, (robot_img_x, robot_img_y), 10, (0, 255, 255), -1)
        cv2.circle(overlay, (robot_img_x, robot_img_y), 11, (255, 255, 255), 2)
        cv2.putText(overlay, "ROBOT", (robot_img_x - 30, robot_img_y - 15), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2, cv2.LINE_AA)
        
        # Add info panel at top
        panel_height = 90
        panel = overlay[0:panel_height, :].copy()
        panel = cv2.addWeighted(panel, 0.3, np.zeros_like(panel), 0.7, 0)
        overlay[0:panel_height, :] = panel
        
        # Add text info
        cv2.putText(overlay, "A* Path Planning to Raycasting Goal", (10, 25), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(overlay, f"Best Direction: P{best_idx+1} | Score: {scores[best_idx]:.2f} | Distance: {best_distance:.2f}m", 
                   (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1, cv2.LINE_AA)
        cv2.putText(overlay, f"Goal: ({goal_x:.2f}, {goal_y:.2f})m | Path: {len(path_img)} waypoints", 
                   (10, 72), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1, cv2.LINE_AA)
        
        return overlay

    def image_cb(self, msg: Image):
        """Main callback for processing incoming camera images."""
        try:
            # Frame skip for throughput
            self.frame_counter += 1
            if self.enable_frame_skip and self.frame_skip > 1 and (self.frame_counter % self.frame_skip) != 0:
                return

            # Convert ROS image to OpenCV format
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            h, w = frame.shape[:2]

            # Step 1: Preprocess and run inference
            pix, t_preproc = self._preprocess_frame(frame)
            probs, t_model = self._run_inference(pix)
            probs_up, t_upsample = self._upsample_and_transfer(probs, h, w)

            if self.floor_ids:
                # Step 2: Create and refine floor mask
                floor_mask, t_mask_create = self._create_floor_mask(probs_up, h, w)
                floor_mask, t_morph = self._apply_morphology(floor_mask)
                floor_mask, t_cc = self._filter_by_area(floor_mask)
                
                wall_mask, t_wall_mask_create = self._create_wall_mask(probs_up, h, w)
                wall_mask, t_wall_morph = self._apply_morphology(wall_mask)
                
                mask_msg = self.bridge.cv2_to_imgmsg(floor_mask, encoding='mono8')
                mask_msg.header = msg.header
                self.mask_pub.publish(mask_msg)
                
                wall_mask_msg = self.bridge.cv2_to_imgmsg(wall_mask, encoding='mono8')
                wall_mask_msg.header = msg.header
                self.wall_mask_pub.publish(wall_mask_msg)
                
                if self.enable_bev:
                    bev_floor, bev_wall, bev_time = self.build_bev_map(floor_mask, wall_mask)
                    angles, free_distances, raycast_data, raycast_time = self.sample_directions_and_raycast(bev_floor, bev_wall)
                    scores, best_idx = self.compute_path_scores(angles, free_distances, bev_floor, bev_wall)
                    
                    # Apply goal change hysteresis to prevent flickering
                    self.goal_update_counter += 1
                    
                    # Get candidate best path
                    candidate_angle = angles[best_idx]
                    candidate_distance = free_distances[best_idx]
                    candidate_score = scores[best_idx]
                    
                    # Decide whether to update goal (prevent frequent switching)
                    should_update_goal = False
                    
                    if self.cached_best_idx is None:
                        # First frame - always accept
                        should_update_goal = True
                    elif self.goal_update_counter >= self.goal_force_update_interval:
                        # Force update every N frames to respond to environment changes
                        should_update_goal = True
                        self.goal_update_counter = 0
                    else:
                        # Check if new goal is significantly different or better
                        angle_diff = abs(candidate_angle - self.cached_best_angle)
                        distance_diff = abs(candidate_distance - self.cached_best_distance)
                        cached_score = scores[self.cached_best_idx] if self.cached_best_idx < len(scores) else 0.0
                        score_improvement = candidate_score - cached_score
                        
                        # Update ONLY if there's a substantial change in direction AND
                        # either distance changed significantly OR score improved dramatically
                        # This prevents small oscillations while allowing real direction changes
                        if (angle_diff > self.goal_change_threshold_angle and 
                            (distance_diff > self.goal_change_threshold_distance or 
                             score_improvement > self.goal_change_threshold_score)):
                            should_update_goal = True
                    
                    # Update cached goal if needed
                    if should_update_goal:
                        self.cached_best_idx = best_idx
                        self.cached_best_angle = candidate_angle
                        self.cached_best_distance = candidate_distance
                    
                    # Use cached goal for stability
                    best_idx = self.cached_best_idx
                    best_angle = self.cached_best_angle
                    best_distance = self.cached_best_distance
                    
                    # Place goal at 80% of free distance
                    goal_distance = 0.8 * best_distance
                    
                    # Convert to Cartesian coordinates (robot-centric, base_link frame)
                    # In base_link: x=forward, y=left, z=up
                    angle_rad = np.deg2rad(best_angle)
                    goal_x = goal_distance * np.cos(angle_rad)  # Forward component
                    goal_y = goal_distance * np.sin(angle_rad)  # Lateral component
                    
                    # Publish goal point
                    goal_msg = PointStamped()
                    goal_msg.header = msg.header
                    goal_msg.header.frame_id = "base_link"
                    goal_msg.point.x = goal_x
                    goal_msg.point.y = goal_y
                    goal_msg.point.z = 0.0
                    self.local_goal_pub.publish(goal_msg)
                    
                    # Create colored BEV visualization
                    # Floor = Green, Wall = Blue, Unknown = Black
                    bev_viz = np.zeros((self.bev_pixels, self.bev_pixels, 3), dtype=np.uint8)
                    bev_viz[bev_floor > 0] = (0, 255, 0)
                    bev_viz[bev_wall > 0] = (255, 0, 0)
                    
                    robot_x = self.bev_pixels // 2
                    robot_y = self.bev_pixels - 1
                    
                    for i, (angle, distance, (raycast_dist, start_x, start_y), score) in enumerate(zip(angles, free_distances, raycast_data, scores)):
                        angle_rad = np.deg2rad(angle)
                        dx = np.sin(angle_rad)
                        dy = -np.cos(angle_rad)
                        distance_px = int(raycast_dist / self.bev_resolution)
                        distance_px_draw = max(1, distance_px)
                        end_x = int(start_x + dx * distance_px_draw)
                        end_y = int(start_y + dy * distance_px_draw)
                        
                        end_x = max(0, min(self.bev_pixels - 1, end_x))
                        end_y = max(0, min(self.bev_pixels - 1, end_y))
                        
                        if i == best_idx:
                            color = (0, 0, 255)
                            thickness = 4
                            radius = 5
                        elif score > 0.6:
                            color = (0, 255, 0)
                            thickness = 2
                            radius = 3
                        elif score > 0.3:
                            color = (0, 200, 255)
                            thickness = 2
                            radius = 3
                        else:
                            color = (0, 0, 255)
                            thickness = 2
                            radius = 3
                        
                        border_thickness = max(1, thickness + 2)
                        cv2.line(bev_viz, (start_x, start_y), (end_x, end_y), (0, 0, 0), border_thickness)
                        cv2.line(bev_viz, (start_x, start_y), (end_x, end_y), color, thickness)
                        
                        if start_x != robot_x or start_y != robot_y:
                            cv2.line(bev_viz, (robot_x, robot_y), (start_x, start_y), (128, 128, 128), 1)

                        cv2.circle(bev_viz, (end_x, end_y), radius + 2, (0, 0, 0), -1)
                        cv2.circle(bev_viz, (end_x, end_y), radius, color, -1)
                        
                        label_x = end_x + 5
                        label_y = end_y - 5
                        label_text = f"P{i+1}:{score:.2f}"
                        if i == best_idx:
                            label_text = f"*{label_text}"
                        (txt_w, txt_h), baseline = cv2.getTextSize(label_text, cv2.FONT_HERSHEY_SIMPLEX, 0.25, 1)
                        rect_tl = (max(0, label_x - 2), max(0, label_y - txt_h - 2))
                        rect_br = (min(self.bev_pixels - 1, label_x + txt_w + 2), min(self.bev_pixels - 1, label_y + 4))
                        cv2.rectangle(bev_viz, rect_tl, rect_br, (0, 0, 0), -1)
                        text_color = (0, 255, 255) if i == best_idx else (255, 255, 255)
                        cv2.putText(bev_viz, label_text, (label_x, label_y), cv2.FONT_HERSHEY_SIMPLEX, 0.25, text_color, 1)
                    
                    # Draw robot as triangle pointing forward (up in BEV)
                    triangle_size = 4
                    pts = np.array([
                        [robot_x, robot_y - triangle_size],  # Front point (top)
                        [robot_x - triangle_size//2, robot_y],  # Left back
                        [robot_x + triangle_size//2, robot_y]   # Right back
                    ], np.int32)
                    cv2.fillPoly(bev_viz, [pts], (0, 255, 255))  # Yellow robot
                    cv2.polylines(bev_viz, [pts], True, (0, 200, 200), 1)  # Border
                    
                    # Draw goal point on BEV
                    if best_distance > 0:
                        goal_angle_rad = np.deg2rad(best_angle)
                        goal_dist_px = int(goal_distance / self.bev_resolution)
                        goal_bev_x = int(robot_x + np.sin(goal_angle_rad) * goal_dist_px)
                        goal_bev_y = int(robot_y - np.cos(goal_angle_rad) * goal_dist_px)
                        
                        goal_bev_x = max(0, min(self.bev_pixels - 1, goal_bev_x))
                        goal_bev_y = max(0, min(self.bev_pixels - 1, goal_bev_y))
                        
                        cv2.circle(bev_viz, (goal_bev_x, goal_bev_y), 6, (0, 255, 0), 2)
                        cv2.drawMarker(bev_viz, (goal_bev_x, goal_bev_y), (0, 255, 0), 
                                     cv2.MARKER_CROSS, 8, 2)
                    
                    bev_viz_large = cv2.resize(bev_viz, (300, 300), interpolation=cv2.INTER_NEAREST)
                    cv2.putText(bev_viz_large, f"BEV ({self.bev_size_meters}m x {self.bev_size_meters}m)", 
                               (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
                    cv2.putText(bev_viz_large, "Robot", 
                               (130, 290), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
                    
                    bev_msg = self.bridge.cv2_to_imgmsg(bev_viz_large, encoding='bgr8')
                    bev_msg.header = msg.header
                    bev_msg.header.frame_id = "base_link"
                    self.bev_map_pub.publish(bev_msg)
                else:
                    bev_time = 0.0
                    raycast_time = 0.0
                
                overlay, t_overlay_create = self._create_overlay_image(frame, floor_mask, wall_mask)
                obstacle_overlay, costmap, obstacle_time = self._detect_obstacles(floor_mask, h, w)
                if self.enable_obstacle_detection and self.enable_drawing and np.any(obstacle_overlay > 0):
                    overlay = cv2.addWeighted(overlay, 0.8, obstacle_overlay, 0.2, 0)
                
                if self.enable_bev:
                    self.rgb_overlay_counter += 1
                    # Always create RGB path overlay (removed frame skipping to prevent flickering)
                    rgb_overlay = self._create_rgb_path_overlay(
                        overlay, floor_mask, angles, free_distances, scores, best_idx, 
                        goal_x, goal_y, best_distance, bev_floor=bev_floor, bev_wall=bev_wall
                    )
                    rgb_overlay_msg = self.bridge.cv2_to_imgmsg(rgb_overlay, encoding='bgr8')
                    rgb_overlay_msg.header = msg.header
                    rgb_overlay_msg.header.frame_id = "camera_color_optical_frame"
                    self.rgb_path_overlay_pub.publish(rgb_overlay_msg)
                
                self._update_fps_stats()
                
                if self.profile_timing:
                    t_centroid_start = time.perf_counter()
                
                M = cv2.moments(floor_mask)
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    
                    start_x = w // 2
                    start_y = h - 1
                    search_width = 50
                    for y in range(h - 1, -1, -1):
                        for dx in range(-search_width, search_width + 1):
                            x = start_x + dx
                            if 0 <= x < w and floor_mask[y, x] > 0:
                                start_y = y
                                start_x = x
                                break
                        if start_y < h - 1:
                            break
                    
                    goal_x, goal_y = self._find_navigation_goal(floor_mask, cx, cy, h, w)
                    path, path_time = self._compute_navigation_path(floor_mask, start_x, start_y, goal_x, goal_y)
                    total_time = obstacle_time + path_time
                    
                    self._draw_navigation_visualization(overlay, floor_mask, path, start_x, start_y, 
                                                       cx, cy, goal_x, goal_y, 
                                                       obstacle_time, path_time, total_time)
                    
                    # Publish centroid
                    centroid = Point()
                    centroid.x = float(cx + 1.0)
                    centroid.y = float(cy + 1.0)
                    centroid.z = 0.0
                    self.centroid_pub.publish(centroid)
                    self.navigable_pub.publish(Bool(data=True))
                else:
                    self.navigable_pub.publish(Bool(data=False))
                
                if self.profile_timing:
                    t_centroid = (time.perf_counter() - t_centroid_start) * 1000
                
                if self.profile_timing:
                    t_publish_start = time.perf_counter()
                overlay_msg = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
                overlay_msg.header = msg.header
                self.overlay_pub.publish(overlay_msg)
                self.seg_image_pub.publish(overlay_msg)
                
                conf = float(np.max(probs_up[self.floor_ids])) if self.floor_ids else 0.0
                self.conf_pub.publish(Float32(data=conf))
                if self.profile_timing:
                    t_publish = (time.perf_counter() - t_publish_start) * 1000
                
                if self.profile_timing:
                    self._log_profiling_info(t_preproc, t_model, t_upsample, t_mask_create, t_morph, t_cc, 
                                           t_overlay_create, bev_time, raycast_time, t_centroid, t_publish)
        except Exception as e:
            self.get_logger().error(f'Image callback error: {e}')

    # CameraInfo parsing handler removed; constants used for intrinsics

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

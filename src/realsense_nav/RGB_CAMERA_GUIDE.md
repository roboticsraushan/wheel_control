# USB RGB Camera Navigation Guide

Alternative navigation system using standard USB RGB cameras (1080p) instead of RealSense.

## Advantages

- ✅ Works with any USB webcam (Logitech, Microsoft, generic)
- ✅ Lower cost than RealSense D455
- ✅ No special drivers needed (uses v4l2)
- ✅ 1080p high resolution for better segmentation
- ✅ Faster setup - plug and play

## Disadvantages

- ❌ No depth information (distance estimation)
- ❌ 2D navigation only
- ❌ More sensitive to lighting conditions

## Compatible Cameras

Any USB camera that supports:
- 1920x1080 resolution
- v4l2 (Video4Linux2) - standard on Linux
- 30fps capture rate

Examples:
- Logitech C920/C922/C930e
- Microsoft LifeCam
- Generic USB webcams
- Laptop built-in cameras

## Setup

### 1. Check Camera Detection

```bash
# List video devices
ls /dev/video*

# Check camera info
v4l2-ctl --list-devices

# Test camera (optional)
ffplay /dev/video0
```

### 2. Install v4l-utils (if needed)

```bash
sudo apt install v4l-utils
```

### 3. Build Package

```bash
cd ~/control_one/wheel_control
colcon build --packages-select realsense_nav
source install/setup.bash
```

## Usage

### Tune Color Thresholds (Do this first!)

```bash
# Run the standalone tuner
python3 src/realsense_nav/realsense_nav/rgb_color_tuner.py
```

- Adjust trackbars until floor appears white in the middle panel
- Press 's' to save values
- Press 'q' to quit and get final values
- Copy the printed values to `rgb_segmentation_node.py` lines ~118-119

### Launch Navigation Only

```bash
source install/setup.bash
ros2 launch realsense_nav rgb_nav.launch.py
```

### Launch Full System (Camera + Motors)

```bash
# Fix serial permissions
sudo chmod 666 /dev/ttyACM0

# Launch
source install/setup.bash
ros2 launch realsense_nav rgb_full_navigation.launch.py
```

### Custom Parameters

```bash
ros2 launch realsense_nav rgb_full_navigation.launch.py \
    camera_device:=0 \
    processing_scale:=0.5 \
    linear_gain:=0.7 \
    angular_gain:=1.5
```

## Parameters Explained

### Camera Parameters

- `camera_device` (int, default: 0)
  - 0 = /dev/video0, 1 = /dev/video1, etc.
  
- `camera_width` (int, default: 1920)
  - Capture resolution width
  
- `camera_height` (int, default: 1080)
  - Capture resolution height
  
- `camera_fps` (int, default: 30)
  - Frames per second

- `processing_scale` (float, default: 0.5)
  - Scale down factor for processing
  - 0.5 = process at 960x540 for speed
  - 1.0 = process at full 1080p (slower but more accurate)

### Segmentation Parameters

- `segmentation_method` (string, default: 'color_based')
  - `color_based`: HSV color thresholding (recommended)
  - `edge_based`: Canny edge detection
  - `combined`: Both color and edge

- `min_area` (int, default: 10000)
  - Minimum navigable area in pixels
  - Larger value = more strict

### Control Parameters

- `linear_gain` (float, default: 0.5)
  - Forward speed control
  
- `angular_gain` (float, default: 1.0)
  - Turning speed control
  
- `max_linear_vel` (float, default: 0.5 m/s)
  - Maximum forward speed
  
- `max_angular_vel` (float, default: 1.0 rad/s)
  - Maximum turning speed

## Topics

### Published

- `/camera/image_raw` (sensor_msgs/Image) - Raw camera feed
- `/segmentation/image` (sensor_msgs/Image) - Visualization
- `/segmentation/centroid` (geometry_msgs/Point) - Target point
- `/segmentation/navigable` (std_msgs/Bool) - Path clear flag
- `/cmd_vel` (geometry_msgs/Twist) - Velocity commands

## Visualization

```bash
# View raw camera
ros2 run rqt_image_view rqt_image_view /camera/image_raw

# View segmentation overlay
ros2 run rqt_image_view rqt_image_view /segmentation/image

# Monitor centroid
ros2 topic echo /segmentation/centroid
```

## Performance Tuning

### If processing is slow:

1. **Reduce processing scale:**
   ```bash
   processing_scale:=0.3  # 30% = 576x324 pixels
   ```

2. **Lower camera resolution:**
   ```bash
   camera_width:=1280 camera_height:=720
   ```

3. **Reduce FPS:**
   ```bash
   camera_fps:=15
   ```

### If navigation is inaccurate:

1. **Increase processing scale:**
   ```bash
   processing_scale:=0.8  # 80% of 1080p
   ```

2. **Tune color thresholds** using the color tuner

3. **Adjust control gains:**
   ```bash
   linear_gain:=0.3 angular_gain:=1.5
   ```

## Color Threshold Tuning Tips

### For Different Floor Types:

**Light floors (white, gray):**
```python
lower_bound = np.array([0, 0, 100])
upper_bound = np.array([180, 50, 255])
```

**Dark floors (black, dark gray):**
```python
lower_bound = np.array([0, 0, 0])
upper_bound = np.array([180, 50, 100])
```

**Wooden floors (brown):**
```python
lower_bound = np.array([10, 50, 50])
upper_bound = np.array([25, 255, 200])
```

**Colored paths (e.g., blue tape):**
```python
# For blue
lower_bound = np.array([100, 100, 50])
upper_bound = np.array([130, 255, 255])
```

## Comparison: RealSense vs USB RGB

| Feature | RealSense D455 | USB RGB Camera |
|---------|---------------|----------------|
| Depth sensing | ✅ Yes | ❌ No |
| Cost | $$$ High | $ Low |
| Setup complexity | Medium | Easy |
| Distance control | ✅ Accurate | ❌ Visual only |
| Resolution | 1280x720 | 1920x1080 |
| Lighting sensitivity | Low | High |
| Outdoor use | Good | Fair |

## When to Use Each

### Use RealSense D455 when:
- Need accurate distance measurement
- Operating in varying lighting
- Obstacle detection required
- Budget allows

### Use USB RGB Camera when:
- Budget constrained
- Indoor controlled environment
- Following colored lines/paths
- 2D navigation sufficient

## Troubleshooting

### Camera not found:
```bash
# Check permissions
sudo chmod 666 /dev/video0

# Verify device
v4l2-ctl --device=/dev/video0 --all
```

### Poor segmentation:
- Run color tuner and adjust for your floor
- Improve lighting conditions
- Clean camera lens
- Try different segmentation methods

### Robot not moving:
- Check `/segmentation/navigable` topic
- Reduce `min_area` parameter
- Verify motor connections and power
- Test with manual `/cmd_vel` commands

## Example Workflows

### Quick Test:
```bash
# Terminal 1: Start navigation
ros2 launch realsense_nav rgb_nav.launch.py

# Terminal 2: View results
ros2 run rqt_image_view rqt_image_view /segmentation/image
```

### Full Autonomous System:
```bash
sudo chmod 666 /dev/ttyACM0
ros2 launch realsense_nav rgb_full_navigation.launch.py
```

### Development/Tuning:
```bash
# Step 1: Tune colors
python3 src/realsense_nav/realsense_nav/rgb_color_tuner.py

# Step 2: Update values in code

# Step 3: Rebuild and test
colcon build --packages-select realsense_nav
source install/setup.bash
ros2 launch realsense_nav rgb_nav.launch.py
```

Happy navigating! 🎥🤖

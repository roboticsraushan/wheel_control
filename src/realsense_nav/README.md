# RealSense Navigation Package

ROS2 package for autonomous robot navigation using Intel RealSense D455 camera with real-time semantic segmentation.

## Features

- Real-time semantic segmentation of navigable areas
- Centroid-based navigation control
- Depth-aware obstacle avoidance
- Configurable control parameters
- Visualization of segmentation results

## Dependencies

- ROS2 Humble
- realsense2_camera
- cv_bridge
- OpenCV (python3-opencv)
- NumPy (python3-numpy)

## Installation

1. Install RealSense ROS2 wrapper:
```bash
sudo apt install ros-humble-realsense2-camera
```

2. Install Python dependencies:
```bash
sudo apt install python3-opencv python3-numpy
```

3. Build the package:
```bash
cd ~/control_one/wheel_control
colcon build --packages-select realsense_nav
source install/setup.bash
```

## Usage

### Launch with default parameters:
```bash
ros2 launch realsense_nav realsense_nav.launch.py
```

### Launch with custom parameters:
```bash
ros2 launch realsense_nav realsense_nav.launch.py \
    linear_gain:=0.7 \
    angular_gain:=1.5 \
    max_linear_vel:=0.8
```

### Run segmentation node standalone:
```bash
ros2 run realsense_nav segmentation_node
```

## Topics

### Subscribed Topics:
- `/camera/color/image_raw` (sensor_msgs/Image) - RGB camera stream
- `/camera/depth/image_rect_raw` (sensor_msgs/Image) - Depth image
- `/camera/color/camera_info` (sensor_msgs/CameraInfo) - Camera calibration

### Published Topics:
- `/cmd_vel` (geometry_msgs/Twist) - Velocity commands for robot
- `/segmentation/image` (sensor_msgs/Image) - Visualization of segmentation
- `/segmentation/centroid` (geometry_msgs/Point) - Centroid of navigable area
- `/segmentation/navigable` (std_msgs/Bool) - Whether path is clear

## Parameters

- `segmentation_method` (string, default: "color_based") - Segmentation algorithm
- `min_area` (int, default: 5000) - Minimum navigable area in pixels
- `target_distance` (float, default: 1.0) - Target distance to maintain (meters)
- `linear_gain` (float, default: 0.5) - Proportional gain for linear velocity
- `angular_gain` (float, default: 1.0) - Proportional gain for angular velocity
- `max_linear_vel` (float, default: 0.5) - Maximum linear velocity (m/s)
- `max_angular_vel` (float, default: 1.0) - Maximum angular velocity (rad/s)

## Segmentation Methods

### Color-Based Segmentation (default)
Simple HSV color thresholding for floor detection. Fast but requires tuning for different environments.

To adjust color thresholds, edit the values in `segmentation_node.py`:
```python
lower_bound = np.array([0, 0, 100])    # [H, S, V] minimum
upper_bound = np.array([180, 50, 255])  # [H, S, V] maximum
```

## Visualization

To view the segmentation results:
```bash
ros2 run rqt_image_view rqt_image_view /segmentation/image
```

## Integration with Motor Control

Connect to your wheel_control package:
```bash
# Terminal 1: Start motor bridge
ros2 launch wheel_control bringup.launch.py

# Terminal 2: Start RealSense navigation
ros2 launch realsense_nav realsense_nav.launch.py
```

The segmentation node will publish `/cmd_vel` commands that your motor bridge will execute.

## Troubleshooting

### Camera not detected:
```bash
# Check if camera is connected
rs-enumerate-devices

# Install udev rules
sudo apt install ros-humble-realsense2-camera
sudo cp /usr/local/lib/udev/rules.d/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### Segmentation not working:
- Adjust HSV color thresholds for your floor
- Ensure adequate lighting
- Check `/segmentation/image` topic for visualization
- Reduce `min_area` parameter if needed

## Future Enhancements

- [ ] Deep learning-based segmentation (DeepLabv3+, SegFormer)
- [ ] Multi-class segmentation (floor, obstacles, walls)
- [ ] Dynamic obstacle detection and avoidance
- [ ] Path planning integration
- [ ] SLAM integration for mapping

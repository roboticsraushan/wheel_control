# RealSense Navigation Quick Start Guide

## Package Created Successfully! 

Your new `realsense_nav` package is ready to use with Intel RealSense D455.

## What's Included

1. **segmentation_node** - Main navigation node with semantic segmentation
2. **color_tuner** - Interactive tool to tune color thresholds
3. **Launch files** - Easy system startup
4. **Configuration** - Tunable parameters

## Setup Steps

### 1. Install RealSense ROS2 Package
```bash
sudo apt install ros-humble-realsense2-camera
```

### 2. Install RealSense SDK (if not already installed)
```bash
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main"
sudo apt update
sudo apt install librealsense2-dkms librealsense2-utils
```

### 3. Verify Camera Connection
```bash
# Check if camera is detected
rs-enumerate-devices

# Test camera stream
realsense-viewer
```

## Usage

### Option 1: Full Autonomous Navigation (Camera + Motors)
```bash
# Make sure Arduino is connected and has permission
sudo chmod 666 /dev/ttyACM0

# Launch everything together
source install/setup.bash
ros2 launch realsense_nav full_navigation.launch.py
```

### Option 2: Camera + Segmentation Only (No Motors)
```bash
source install/setup.bash
ros2 launch realsense_nav realsense_nav.launch.py
```

### Option 3: Tune Color Thresholds First
```bash
# Step 1: Launch camera
ros2 launch realsense2_camera rs_launch.py

# Step 2: In another terminal, run color tuner
source install/setup.bash
ros2 run realsense_nav color_tuner
```

Use the trackbars to adjust HSV values until your floor is white in the mask.
The tool will print the final values to use in `segmentation_node.py`.

## Visualization

### View segmentation output:
```bash
ros2 run rqt_image_view rqt_image_view /segmentation/image
```

### Monitor topics:
```bash
# Check if navigable path detected
ros2 topic echo /segmentation/navigable

# Check centroid position
ros2 topic echo /segmentation/centroid

# Check velocity commands
ros2 topic echo /cmd_vel
```

## Adjusting Parameters

Edit the launch file or use command-line arguments:

```bash
ros2 launch realsense_nav full_navigation.launch.py \
    linear_gain:=0.7 \
    angular_gain:=1.5
```

Or edit `config/segmentation_params.yaml` for persistent changes.

## Color Threshold Tuning

The default values work for light-colored floors (gray/white). For different floors:

1. Run the color tuner tool
2. Adjust H, S, V ranges until floor is properly segmented
3. Copy the printed values into `segmentation_node.py` line ~131-132:

```python
lower_bound = np.array([your_h_min, your_s_min, your_v_min])
upper_bound = np.array([your_h_max, your_s_max, your_v_max])
```

4. Rebuild: `colcon build --packages-select realsense_nav`

## Troubleshooting

### Camera not found
```bash
# Add udev rules
sudo cp /usr/local/lib/udev/rules.d/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### Robot not moving
- Check motor driver is powered
- Verify serial connection: `pio device monitor -p /dev/ttyACM0 -b 115200`
- Increase `linear_gain` and `max_linear_vel` parameters
- Check if navigable area is detected: `ros2 topic echo /segmentation/navigable`

### Poor segmentation
- Use color tuner to adjust thresholds
- Ensure good lighting
- Lower `min_area` if floor patches are small
- Clean camera lens

## System Architecture

```
RealSense D455 Camera
    ↓ (RGB + Depth)
Segmentation Node
    ↓ (Centroid + Distance)
Velocity Controller
    ↓ (/cmd_vel)
Serial Motor Bridge
    ↓ (Serial commands)
Arduino Due
    ↓ (PWM signals)
Motors
```

## Next Steps

- Test with your robot in a controlled environment
- Tune color thresholds for your floor
- Adjust control gains for smooth movement
- Add obstacle avoidance logic
- Integrate with path planning

Enjoy your autonomous navigation system! 🚀

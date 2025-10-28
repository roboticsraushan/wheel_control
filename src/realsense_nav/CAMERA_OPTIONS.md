# Navigation Package - Camera Options

Your `realsense_nav` package now supports **TWO camera types**!

## Option 1: Intel RealSense D455 (Depth Camera)

### Features:
- ✅ RGB + Depth sensing
- ✅ Accurate distance measurement
- ✅ 3D obstacle detection
- ✅ Better for varying lighting
- 💰 Higher cost (~$300)

### Setup:
```bash
sudo apt install ros-humble-realsense2-camera
```

### Launch:
```bash
# Navigation only
ros2 launch realsense_nav realsense_nav.launch.py

# Full system (camera + motors)
ros2 launch realsense_nav full_navigation.launch.py
```

### Color tuner:
```bash
ros2 run realsense_nav color_tuner
```

---

## Option 2: USB RGB Camera (1080p Webcam)

### Features:
- ✅ Works with any USB camera
- ✅ 1080p high resolution
- ✅ No special drivers
- ✅ Plug and play
- 💰 Low cost (~$30-100)
- ⚠️ No depth sensing

### Setup:
```bash
# Just plug in the camera!
ls /dev/video*  # Check it's detected
```

### Launch:
```bash
# Navigation only
ros2 launch realsense_nav rgb_nav.launch.py

# Full system (camera + motors)
ros2 launch realsense_nav rgb_full_navigation.launch.py
```

### Color tuner:
```bash
# Standalone (no ROS needed)
python3 src/realsense_nav/realsense_nav/rgb_color_tuner.py

# Or with ROS
ros2 run realsense_nav rgb_color_tuner
```

---

## Quick Start - USB RGB Camera

Since you asked for RGB camera support, here's the fastest way to get started:

### 1. Test your USB camera:
```bash
ls /dev/video*
# You should see /dev/video0 (or similar)
```

### 2. Tune color thresholds for your floor:
```bash
cd ~/control_one/wheel_control
python3 src/realsense_nav/realsense_nav/rgb_color_tuner.py
```
- Adjust sliders until floor is white in mask
- Press 's' to save, 'q' to quit
- Copy printed values to `rgb_segmentation_node.py`

### 3. Test navigation only:
```bash
source install/setup.bash
ros2 launch realsense_nav rgb_nav.launch.py
```

### 4. View results:
```bash
# In another terminal
ros2 run rqt_image_view rqt_image_view /segmentation/image
```

### 5. Connect motors and run full system:
```bash
sudo chmod 666 /dev/ttyACM0
ros2 launch realsense_nav rgb_full_navigation.launch.py
```

---

## Which Camera Should You Use?

### Choose **RealSense D455** if:
- You need accurate distance measurement
- You want 3D obstacle detection
- You have the budget ($300+)
- You operate in varying lighting conditions
- You need depth for SLAM/mapping

### Choose **USB RGB Camera** if:
- You want low cost ($30-100)
- You have a controlled indoor environment
- You're following colored paths/lines
- You want 1080p resolution
- You want faster setup (plug and play)

---

## All Available Launch Files

1. **realsense_nav.launch.py** - RealSense camera only
2. **full_navigation.launch.py** - RealSense + motors
3. **rgb_nav.launch.py** - USB RGB camera only  ⭐ NEW
4. **rgb_full_navigation.launch.py** - USB RGB + motors  ⭐ NEW

## All Available Nodes

1. **segmentation_node** - RealSense navigation
2. **color_tuner** - RealSense color tuning (ROS node)
3. **rgb_segmentation_node** - USB camera navigation  ⭐ NEW
4. **rgb_color_tuner** - USB camera tuning (standalone)  ⭐ NEW

---

## Documentation

- **README.md** - Main package documentation
- **QUICKSTART.md** - RealSense setup guide
- **RGB_CAMERA_GUIDE.md** - USB camera detailed guide  ⭐ NEW
- **CAMERA_OPTIONS.md** - This file

---

## Example Commands

### Test USB camera detection:
```bash
v4l2-ctl --list-devices
```

### Run RGB navigation with custom settings:
```bash
ros2 launch realsense_nav rgb_full_navigation.launch.py \
    camera_device:=0 \
    processing_scale:=0.5 \
    linear_gain:=0.7 \
    angular_gain:=1.5
```

### Monitor navigation:
```bash
# Check if path is clear
ros2 topic echo /segmentation/navigable

# Check steering
ros2 topic echo /cmd_vel

# Check centroid position
ros2 topic echo /segmentation/centroid
```

---

Ready to navigate! Choose your camera and get started! 🚀

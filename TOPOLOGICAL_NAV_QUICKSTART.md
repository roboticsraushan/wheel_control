# Quick Start: Topological Navigation Setup

## 1. Install Python Dependencies

```bash
cd /home/raushan/control_one/wheel_control

# Install PyTorch (CUDA 11.8 or 12.1)
pip3 install torch torchvision --index-url https://download.pytorch.org/whl/cu118

# Install transformers and other dependencies
pip3 install transformers accelerate safetensors
pip3 install opencv-python Pillow numpy

# Or install all at once from requirements.txt
pip3 install -r requirements.txt
```

**Note**: First run will download DINOv2-Small model (~90MB) from HuggingFace.

## 2. Build ROS2 Package

```bash
cd /home/raushan/control_one/wheel_control
colcon build --packages-select realsense_nav
source install/setup.bash
```

## 3. Verify Installation

```bash
# Check nodes are available
ros2 pkg executables realsense_nav | grep dinov3

# Should show:
# realsense_nav dinov3_embedding_node
# realsense_nav topological_mapper_node
# realsense_nav joystick_node_recorder
# realsense_nav place_recognition_node
```

## 4. Test DINOv3 Embedding (Standalone)

```bash
# Terminal 1: Launch camera
ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true \
  enable_depth:=true \
  color_width:=1280 \
  color_height:=720

# Terminal 2: Test embedding extraction
ros2 run realsense_nav dinov3_embedding_node \
  --ros-args \
  -p device:=cuda \
  -p update_rate_hz:=1.0

# Terminal 3: Monitor output
ros2 topic echo /dinov3/stats
```

Expected output:
```json
{
  "node": "dinov3_embedding",
  "avg_inference_ms": 45.2,
  "actual_rate_hz": 1.0,
  "device": "cuda:0",
  "embedding_dim": 384
}
```

## 5. Create Your First Map

```bash
# Terminal 1: Camera + Visual Odometry
ros2 launch realsense_nav realsense_visual_odometry.launch.py

# Terminal 2: Topological Mapping
ros2 launch realsense_nav topological_mapping.launch.py

# Drive around with joystick, press Button 0 to create nodes
# Map saved to: data/maps/topological_map.json
```

## 6. Test Place Recognition

```bash
# Terminal 1: Camera (already running)

# Terminal 2: Navigation mode
ros2 launch realsense_nav topological_navigation.launch.py

# Terminal 3: Watch recognition
ros2 topic echo /place_recognition/best_match
```

## Troubleshooting

### CUDA Out of Memory
```bash
# Use CPU instead (slower but works)
ros2 launch realsense_nav topological_mapping.launch.py device:=cpu
```

### Model Download Fails
```bash
# Pre-download manually
python3 -c "from transformers import AutoModel; AutoModel.from_pretrained('facebook/dinov2-small')"
```

### Camera Topic Wrong
```bash
# Check your camera topic
ros2 topic list | grep image_raw

# If different, update launch file or remap:
ros2 run realsense_nav dinov3_embedding_node \
  --ros-args -r /camera/camera/color/image_raw:=/your/camera/topic
```

## Next Steps

1. **Map your environment**: Drive around, create 10-20 nodes
2. **Test recognition**: Drive to mapped locations, check confidence
3. **Add belief tracker**: Bayesian localization (coming next)
4. **Add planner**: A* topological path planning
5. **Add controller**: Autonomous navigation

## Files Created

- `dinov3_embedding_node.py` - Scene embedding extraction
- `topological_mapper_node.py` - Node recording and management
- `joystick_node_recorder.py` - Manual node creation trigger
- `place_recognition_node.py` - Visual place recognition
- `topological_mapping.launch.py` - Mapping phase launcher
- `topological_navigation.launch.py` - Navigation phase launcher
- `TOPOLOGICAL_NAV_README.md` - Full documentation
- `requirements.txt` - Python dependencies

## Performance Expectations

| Component | Time | Rate |
|-----------|------|------|
| DINOv3 Embedding | 40-50ms | 20-25 Hz max |
| Place Recognition | 1-2ms | 500+ Hz |
| Total Pipeline | ~50ms | ~20 Hz |

Good for human-speed navigation! 🚀

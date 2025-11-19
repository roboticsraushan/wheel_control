# Topological Navigation with DINOv3

Human-inspired visual place recognition for robot navigation.

## Overview

This system enables robots to navigate using a **topological map** (graph of places) rather than metric SLAM. It uses **DINOv3 embeddings** to recognize locations based on visual appearance, similar to how humans remember and recognize places.

### Key Features
- 🎯 **Place Recognition**: DINOv3-Small embeddings (384-dim) for visual similarity
- 🗺️ **Topological Mapping**: Record nodes (places) and edges (connections)
- 🕹️ **Joystick Interface**: Manual node creation during mapping
- 🧠 **Adaptive Frequency**: Adjusts update rate based on confidence
- 📊 **Bayesian Belief**: Multi-hypothesis localization (coming soon)

## Architecture

```
Camera → DINOv3 Embedding → Place Recognition → Belief Tracker → Planner → Controller
           (40-50ms)         (cosine similarity)    (Bayesian)     (A*)     (reactive)
```

See full architecture: [docs/TOPOLOGICAL_NAVIGATION_ARCHITECTURE.md](../../docs/TOPOLOGICAL_NAVIGATION_ARCHITECTURE.md)

## Installation

### 1. Install Python Dependencies

```bash
cd /home/raushan/control_one/wheel_control
pip install -r requirements.txt
```

This installs:
- `torch` - PyTorch for DINOv3
- `transformers` - HuggingFace model loading
- `numpy`, `opencv-python` - Vision processing

### 2. Build ROS2 Package

```bash
cd /home/raushan/control_one/wheel_control
colcon build --packages-select realsense_nav
source install/setup.bash
```

## Usage

### Phase 1: Mapping

**Goal**: Drive around and create nodes at important locations.

```bash
# Terminal 1: Launch RealSense camera + visual odometry
ros2 launch realsense_nav realsense_visual_odometry.launch.py

# Terminal 2: Launch topological mapping
ros2 launch realsense_nav topological_mapping.launch.py

# Terminal 3: (Optional) Monitor
ros2 topic echo /topological_map/status
```

**Controls**:
- Drive with joystick
- Press **Button 0 (A/X)** to create node at current location
- Press **Button 1 (B/Circle)** to save map

**Output**: `data/maps/topological_map.json` with nodes and edges

### Phase 2: Navigation

**Goal**: Recognize current location and navigate to goal.

```bash
# Terminal 1: RealSense camera (already running)

# Terminal 2: Launch topological navigation
ros2 launch realsense_nav topological_navigation.launch.py

# Terminal 3: Monitor place recognition
ros2 topic echo /place_recognition/best_match
```

**Output**:
- `/place_recognition/best_match` - Most likely current node
- `/place_recognition/confidence` - Confidence (0-1)
- `/place_recognition/belief` - Full probability distribution

## Map Format

`data/maps/topological_map.json`:

```json
{
  "nodes": {
    "node_0001": {
      "node_id": "node_0001",
      "embedding": [0.123, -0.456, ...],  // 384-dim DINOv3 embedding
      "pose": {
        "x": 1.23, "y": -0.45, "z": 0.0,
        "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0
      },
      "landmarks": ["chair", "door", "table"],  // YOLO detections
      "floor_quality": {"navigable_ratio": 0.85},
      "timestamp": "2025-01-17T10:30:00",
      "edges": [
        {"to_node": "node_0002", "distance": 1.5, "traversable": true}
      ]
    }
  },
  "metadata": {
    "num_nodes": 42,
    "next_node_id": 43
  }
}
```

## Parameters

### dinov3_embedding_node

| Parameter | Default | Description |
|-----------|---------|-------------|
| `device` | `cuda` | `cuda` or `cpu` |
| `update_rate_hz` | `1.0` | Embedding extraction rate (Hz) |
| `model_name` | `facebook/dinov2-small` | HuggingFace model ID |
| `embedding_dim` | `384` | DINOv2-Small: 384, Base: 768 |
| `adaptive_rate` | `true` | Adjust rate based on load |

### topological_mapper_node

| Parameter | Default | Description |
|-----------|---------|-------------|
| `map_file` | `data/maps/topological_map.json` | Save location |
| `auto_edge_threshold` | `2.0` | Auto-connect nodes within N meters |
| `min_node_distance` | `0.5` | Minimum distance between nodes (m) |

### place_recognition_node

| Parameter | Default | Description |
|-----------|---------|-------------|
| `similarity_threshold` | `0.7` | Min cosine similarity for match |
| `temperature` | `0.1` | Softmax temperature (lower = sharper) |
| `enable_temporal_smoothing` | `true` | Smooth belief over time |
| `smoothing_alpha` | `0.3` | EMA factor (0=full smoothing, 1=no smoothing) |

## Topics

### Published

| Topic | Type | Description |
|-------|------|-------------|
| `/dinov3/embedding` | `Float32MultiArray` | Scene embedding (384-dim) |
| `/dinov3/stats` | `String` | Performance statistics (JSON) |
| `/topological_map/status` | `String` | Map status (JSON) |
| `/place_recognition/belief` | `String` | Probability distribution (JSON) |
| `/place_recognition/best_match` | `String` | Best node match (JSON) |
| `/place_recognition/confidence` | `Float32MultiArray` | Confidence scalar |

### Subscribed

| Topic | Type | Description |
|-------|------|-------------|
| `/camera/camera/color/image_raw` | `Image` | RGB camera feed |
| `/visual_odometry/pose` | `PoseStamped` | Robot pose (cuVSLAM) |
| `/scene_graph/nodes` | `String` | YOLO landmarks (JSON) |
| `/sam/floor_quality` | `String` | Floor segmentation (JSON) |
| `/joy` | `Joy` | Joystick input |

## Performance

### DINOv2-Small (Current)

- **Inference Time**: 40-50ms on RTX 3060
- **Embedding Dim**: 384
- **Parameters**: 22M
- **Visual-only** (no text capability)

### DINOv3-Small (Future)

When HuggingFace releases DINOv3:
- **Inference Time**: ~50ms (similar to v2)
- **Embedding Dim**: 1536 (richer representation)
- **Multi-modal**: Text + vision queries
- Auto-detects and uses if available

## Troubleshooting

### No embeddings published

```bash
# Check if DINOv3 node is running
ros2 node list | grep dinov3

# Check camera feed
ros2 topic hz /camera/camera/color/image_raw

# Check for errors
ros2 topic echo /dinov3/stats
```

### "No embedding available" error

DINOv3 embedding node must run before mapper:
```bash
ros2 node info /dinov3_embedding_node
```

### Low place recognition confidence

- **Add more nodes**: Sparse map = poor recognition
- **Reduce similarity_threshold**: Try 0.6 instead of 0.7
- **Check lighting**: DINOv3 robust but extreme changes affect it
- **Enable temporal_smoothing**: Averages over time

### CUDA out of memory

```bash
# Use CPU instead (slower but works)
ros2 launch realsense_nav topological_mapping.launch.py device:=cpu
```

## Next Steps

Remaining components (see architecture doc):

1. **Belief Tracker**: Bayesian multi-hypothesis localization
2. **Topological Planner**: A* path planning on node graph
3. **Edge Executor**: Follow edges between nodes
4. **Reactive Controller**: Obstacle avoidance + floor following

## References

- [DINOv2 Paper](https://arxiv.org/abs/2304.07193) - Self-supervised vision transformers
- [Topological Navigation](../../docs/TOPOLOGICAL_NAVIGATION_ARCHITECTURE.md) - Full architecture
- [Implementation Checklist](../../docs/IMPLEMENTATION_CHECKLIST.md) - Task tracking

## License

Apache-2.0

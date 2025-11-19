# Human-Inspired Topological Navigation Architecture

> **Implementation Status**: ✅ Core nodes implemented (DINOv2-Small embedding extraction, topological mapper, place recognition, joystick recorder). See [Quick Start Guide](../TOPOLOGICAL_NAV_QUICKSTART.md) for setup.

> **Note on DINOv2 vs DINOv3**: Currently using **DINOv2-Small** (384-dim, visual-only) as DINOv3 isn't yet available via HuggingFace Transformers. Code is prepared to auto-upgrade to DINOv3 when released (1536-dim, multi-modal text+vision). Performance: 40-50ms inference on RTX 3060.

## Overview

This document describes a **human-inspired topological navigation system** that prioritizes cognitive navigation over metric precision. Like humans who navigate by recognizing places and landmarks rather than calculating exact coordinates, our system uses:

- **Topological mapping**: Discrete nodes (places) connected by edges (paths)
- **Place recognition**: Visual embeddings and semantic markers to identify "where am I?"
- **Reactive control**: Floor-based navigation between nodes without cm-level precision
- **Multi-modal sensing**: Vision (YOLO + DINOv2), SAM floor segmentation, IMU, and visual odometry

### Navigation Philosophy

Humans don't navigate by computing (x=2.34m, y=5.67m). Instead:
1. **Recognize places**: "I'm in the kitchen"
2. **Plan topologically**: "Kitchen → Hallway → Bedroom"  
3. **React locally**: "Walk straight, avoid that chair"
4. **Use landmarks**: "Turn left at the door"

Our system emulates this approach for robust indoor navigation with uncertainty awareness and semantic reasoning.

---

## System Architecture Overview

### Three-Layer Navigation Hierarchy

```
┌─────────────────────────────────────────────────────────────┐
│                   HIGH-LEVEL PLANNER                         │
│  Topological Path Planning: A* / Dijkstra on Node Graph     │
│  Input: Current node belief, Goal node                       │
│  Output: Sequence of nodes to visit                          │
└─────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────┐
│                  MID-LEVEL CONTROLLER                        │
│  Place Recognition + Edge Execution                          │
│  Input: Visual embeddings, marker detections                 │
│  Output: Confidence belief over nodes, navigation actions    │
└─────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────┐
│                  LOW-LEVEL CONTROLLER                        │
│  Reactive Floor-Following Navigation                         │
│  Input: SAM floor mask, visual odometry, IMU                 │
│  Output: Velocity commands (/cmd_vel)                        │
└─────────────────────────────────────────────────────────────┘
```

---

## Component Architecture

### Perception Layer

The perception layer extracts semantic and geometric information from sensors:

```
┌──────────────────────────────────────────────────────────────────┐
│                        PERCEPTION LAYER                           │
├──────────────────────────────────────────────────────────────────┤
│                                                                   │
│  ┌─────────────────┐  ┌──────────────────┐  ┌────────────────┐ │
│  │  RealSense D455 │  │   Visual Odom    │  │      IMU       │ │
│  │  RGB + Depth    │  │  (cuVSLAM/ORB)   │  │  Orientation   │ │
│  └────────┬────────┘  └────────┬─────────┘  └───────┬────────┘ │
│           │                    │                     │           │
│           ↓                    ↓                     ↓           │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │              MULTI-MODAL FEATURE EXTRACTION              │   │
│  ├──────────────────────────────────────────────────────────┤   │
│  │                                                           │   │
│  │  • YOLO Detection → Semantic markers (door, chair, etc)  │   │
│  │  • SAM Segmentation → Floor mask for navigation          │   │
│  │  • DINOv2 Embeddings → Scene fingerprint (1536-dim)      │   │
│  │  • Depth Processing → 3D positions in camera frame       │   │
│  │  • IMU Integration → Heading and angular velocity        │   │
│  │                                                           │   │
│  └──────────────────────────────────────────────────────────┘   │
│                                                                   │
└──────────────────────────────────────────────────────────────────┘
```

**Key Nodes:**
- `scene_graph_node.py` - YOLO detection, publishes `/scene_graph`
- `scene_graph_mapper.py` - Transforms detections to map frame using camera_link
- `sam_floor_segmentation_node.py` - Floor segmentation for reactive navigation
- `visual_odometry_node` - Relative motion estimation
- `imu_odometry_node.py` - Orientation tracking

---

### Mapping Layer (Training Phase)

During exploration with joystick, the system builds a topological map:

```
┌──────────────────────────────────────────────────────────────────┐
│                       MAPPING PHASE                               │
│                  (Joystick Exploration)                           │
├──────────────────────────────────────────────────────────────────┤
│                                                                   │
│  Human drives robot → System records nodes at key locations      │
│                                                                   │
│  ┌────────────────────────────────────────────────────────────┐  │
│  │              NODE CREATION TRIGGER                         │  │
│  │  • Manual: Joystick button press                          │  │
│  │  • Auto: Every N meters OR scene change detected         │  │
│  │  • Semantic: At doorways, junctions, landmarks           │  │
│  └────────────────────────────────────────────────────────────┘  │
│                                                                   │
│  At Each Node:                                                    │
│  ┌────────────────────────────────────────────────────────────┐  │
│  │  1. Capture RGB Image                                     │  │
│  │  2. Extract DINOv2 Embedding (1536-dim scene descriptor)  │  │
│  │  3. Record YOLO Markers (labels + relative positions)     │  │
│  │  4. Compute Floor Traversability (SAM segmentation score) │  │
│  │  5. Store Visual Odometry Pose (for rough positioning)    │  │
│  │  6. Save Image Thumbnail (for human inspection)           │  │
│  └────────────────────────────────────────────────────────────┘  │
│                                                                   │
│  Between Nodes:                                                   │
│  ┌────────────────────────────────────────────────────────────┐  │
│  │  • Visual odometry tracks motion                          │  │
│  │  • Record edge properties: distance, direction, duration  │  │
│  │  • Note markers visible along path                        │  │
│  │  • Store floor quality (average SAM navigability)         │  │
│  └────────────────────────────────────────────────────────────┘  │
│                                                                   │
│  Output: Topological Graph Saved to Disk                         │
│  ├─ nodes.json (node metadata, embeddings, markers)              │
│  ├─ edges.json (connections, traversal properties)               │
│  └─ images/ (node thumbnails)                                    │
│                                                                   │
└──────────────────────────────────────────────────────────────────┘
```

**Node Data Structure:**
```json
{
  "id": "kitchen_001",
  "label": "Kitchen",
  "embedding": [0.23, 0.56, ...],  // DINOv2 1536-dim
  "markers": [
    {
      "label": "refrigerator",
      "direction_deg": 45,
      "distance_m": 2.5,
      "confidence": 0.92
    },
    {
      "label": "table",
      "direction_deg": 90,
      "distance_m": 1.8,
      "confidence": 0.87
    }
  ],
  "floor_traversability": 0.95,
  "image_path": "images/kitchen_001.jpg",
  "timestamp": 1700000000.123,
  "connections": ["hallway_002", "pantry_005"]
}
```

**Edge Data Structure:**
```json
{
  "from_node": "kitchen_001",
  "to_node": "hallway_002",
  "action": "move_forward",
  "distance_m": 3.5,
  "duration_s": 5.2,
  "floor_quality": 0.88,
  "markers_along_path": ["door"],
  "traversal_count": 0
}
```

**Key Nodes:**
- `topological_mapper_node.py` - Creates nodes, extracts embeddings
- `scene_graph_mapper.py` - Provides marker positions in map frame
- `joystick_node_recorder.py` - Manual node creation trigger

---

### Localization Layer (Where Am I?)

During navigation, the system maintains a **belief distribution** over nodes:

```
┌──────────────────────────────────────────────────────────────────┐
│                    PLACE RECOGNITION                              │
│               (Multi-Hypothesis Localization)                     │
├──────────────────────────────────────────────────────────────────┤
│                                                                   │
│  Current Belief: {Kitchen: 0.7, Hallway: 0.2, Bedroom: 0.1}     │
│                                                                   │
│  ┌────────────────────────────────────────────────────────────┐  │
│  │           VISUAL SIMILARITY (DINOv2)                       │  │
│  │                                                            │  │
│  │  1. Extract embedding from current camera view            │  │
│  │  2. Compute cosine similarity with all stored nodes       │  │
│  │  3. Generate probability: P(node | visual_features)       │  │
│  │                                                            │  │
│  │  Similarity > 0.9  → High confidence                      │  │
│  │  Similarity 0.7-0.9 → Medium confidence                   │  │
│  │  Similarity < 0.7  → Low confidence / lost               │  │
│  └────────────────────────────────────────────────────────────┘  │
│                            +                                      │
│  ┌────────────────────────────────────────────────────────────┐  │
│  │          MARKER VALIDATION (YOLO)                          │  │
│  │                                                            │  │
│  │  1. Detect current visible markers                        │  │
│  │  2. Compare with expected markers at each node            │  │
│  │  3. Compute Jaccard similarity of marker sets             │  │
│  │  4. Generate probability: P(node | markers)               │  │
│  │                                                            │  │
│  │  Example:                                                  │  │
│  │  • Expected at Kitchen: {fridge, table}                   │  │
│  │  • Observed: {fridge, table, chair}                       │  │
│  │  • Jaccard: 2/3 = 0.67 → Likely Kitchen                  │  │
│  └────────────────────────────────────────────────────────────┘  │
│                            +                                      │
│  ┌────────────────────────────────────────────────────────────┐  │
│  │        MOTION CONSISTENCY (Visual Odometry)                │  │
│  │                                                            │  │
│  │  1. Track relative motion since last update               │  │
│  │  2. Update belief based on graph connectivity             │  │
│  │  3. P(next_node | current_node, motion)                   │  │
│  │                                                            │  │
│  │  Example:                                                  │  │
│  │  • Was at Kitchen, moved 3m forward                       │  │
│  │  • Kitchen connects to Hallway                            │  │
│  │  • Increase P(Hallway)                                    │  │
│  └────────────────────────────────────────────────────────────┘  │
│                                                                   │
│  ═══════════════════════════════════════════════════════════    │
│                   BAYESIAN FUSION                                │
│  ═══════════════════════════════════════════════════════════    │
│                                                                   │
│  Combined: P(node | all_observations) ∝                          │
│            P(node | visual) × P(node | markers) × P(node | motion)│
│                                                                   │
│  Normalize → Updated belief distribution                         │
│                                                                   │
│  Decision:                                                        │
│  • If max P(node) > 0.8 → "Localized at [node]"                 │
│  • If max P(node) 0.5-0.8 → "Uncertain, keep checking"          │
│  • If max P(node) < 0.5 → "Lost! Initiate recovery"             │
│                                                                   │
└──────────────────────────────────────────────────────────────────┘
```

**Recovery Strategies When Lost:**
1. **Rotate in place** - Gather more visual information (360° scan)
2. **Move to vantage point** - Navigate to location with distinctive markers
3. **Global re-localization** - Compare with all nodes (computationally expensive)
4. **Human assistance** - Request voice/UI guidance

**Key Nodes:**
- `place_recognition_node.py` - DINOv2 embedding comparison
- `marker_validator_node.py` - Marker-based validation
- `belief_tracker_node.py` - Multi-hypothesis tracking, Bayesian fusion

---

### Navigation Layer (Get There)

Once localized, the system plans and executes paths through the topological graph:

```
┌──────────────────────────────────────────────────────────────────┐
│                    NAVIGATION EXECUTION                           │
├──────────────────────────────────────────────────────────────────┤
│                                                                   │
│  Goal: "Go to Bedroom"                                           │
│                                                                   │
│  ╔══════════════════════════════════════════════════════════╗    │
│  ║         STEP 1: TOPOLOGICAL PLANNING                     ║    │
│  ╚══════════════════════════════════════════════════════════╝    │
│                                                                   │
│  ┌────────────────────────────────────────────────────────────┐  │
│  │  Graph Search (A* or Dijkstra)                            │  │
│  │  Input: Current node belief, Goal node                    │  │
│  │  Output: Node sequence [Kitchen → Hallway → Bedroom]     │  │
│  │                                                            │  │
│  │  Cost function considers:                                 │  │
│  │  • Edge distance                                          │  │
│  │  • Floor quality (prefer higher traversability)           │  │
│  │  • Historical success (edges traversed successfully)      │  │
│  └────────────────────────────────────────────────────────────┘  │
│                                                                   │
│  ╔══════════════════════════════════════════════════════════╗    │
│  ║         STEP 2: EDGE EXECUTION (Per Edge)                ║    │
│  ╚══════════════════════════════════════════════════════════╝    │
│                                                                   │
│  For edge: Kitchen → Hallway                                     │
│                                                                   │
│  ┌────────────────────────────────────────────────────────────┐  │
│  │  a) Verify at Start Node                                  │  │
│  │     - Place recognition confirms "At Kitchen"             │  │
│  │     - Confidence > 0.8 to proceed                         │  │
│  └────────────────────────────────────────────────────────────┘  │
│           ↓                                                       │
│  ┌────────────────────────────────────────────────────────────┐  │
│  │  b) Execute Edge Action                                   │  │
│  │     - Action: "move_forward"                              │  │
│  │     - Target: ~3.5 meters based on edge metadata          │  │
│  └────────────────────────────────────────────────────────────┘  │
│           ↓                                                       │
│  ┌────────────────────────────────────────────────────────────┐  │
│  │  c) REACTIVE CONTROL (Low-Level Loop)                     │  │
│  │                                                            │  │
│  │  While traversing edge:                                   │  │
│  │    ┌─────────────────────────────────────────────────┐   │  │
│  │    │  • SAM floor mask → Identify navigable surface  │   │  │
│  │    │  • Follow floor centerline (like lane keeping)  │   │  │
│  │    │  • Visual odometry → Smooth velocity control    │   │  │
│  │    │  • IMU → Maintain heading stability             │   │  │
│  │    │  • Obstacle avoidance: veer around non-floor    │   │  │
│  │    └─────────────────────────────────────────────────┘   │  │
│  │                                                            │  │
│  │  Publish: /cmd_vel (Twist messages)                       │  │
│  └────────────────────────────────────────────────────────────┘  │
│           ↓                                                       │
│  ┌────────────────────────────────────────────────────────────┐  │
│  │  d) Monitor Progress                                      │  │
│  │     - Expect to see edge markers (e.g., "door")           │  │
│  │     - If unexpected markers → Re-localize                 │  │
│  │     - Track distance traveled (visual odometry)           │  │
│  └────────────────────────────────────────────────────────────┘  │
│           ↓                                                       │
│  ┌────────────────────────────────────────────────────────────┐  │
│  │  e) Confirm Arrival at Next Node                          │  │
│  │     - Place recognition: "Now at Hallway" (conf > 0.8)    │  │
│  │     - Update belief state                                 │  │
│  │     - Proceed to next edge OR Goal reached                │  │
│  └────────────────────────────────────────────────────────────┘  │
│                                                                   │
│  ╔══════════════════════════════════════════════════════════╗    │
│  ║         STEP 3: COMPLETION                               ║    │
│  ╚══════════════════════════════════════════════════════════╝    │
│                                                                   │
│  Place recognition confirms: "At Bedroom" → Task complete!       │
│  Publish: /navigation/status "SUCCESS"                           │
│                                                                   │
└──────────────────────────────────────────────────────────────────┘
```

**Key Nodes:**
- `topological_planner_node.py` - High-level graph path planning
- `edge_executor_node.py` - Manages edge traversal, coordinates low-level control
- `reactive_controller_node.py` - Floor-following, publishes /cmd_vel
- `navigation_monitor_node.py` - Tracks progress, handles failures

---

## Data Flow Architecture

### Complete System Data Flow

```
┌─────────────────────────────────────────────────────────────────────┐
│                        SENSOR INPUTS                                 │
└────────┬────────────────────────────────────────────────────────────┘
         │
         ├─── RealSense RGB ──→ /camera/color/image_raw
         ├─── RealSense Depth ─→ /camera/depth/image_rect_raw
         ├─── Visual Odometry ─→ /camera/odom/sample
         └─── IMU ─────────────→ /imu/data
                                       │
         ┌─────────────────────────────┴──────────────────────────┐
         │                                                         │
         ↓                                                         ↓
┌─────────────────────┐                              ┌──────────────────────┐
│  scene_graph_node   │                              │  sam_floor_seg_node  │
│  (YOLO Detection)   │                              │  (Floor Masking)     │
└──────┬──────────────┘                              └──────┬───────────────┘
       │                                                     │
       ├─→ /scene_graph (JSON)                              ├─→ /floor/mask
       └─→ /yolo/overlay_image                              └─→ /floor/navigability
              │                                                      │
              ↓                                                      │
   ┌────────────────────────┐                                       │
   │ scene_graph_mapper     │                                       │
   │ (Transform to map)     │                                       │
   └──────┬─────────────────┘                                       │
          │                                                          │
          ├─→ /scene_graph/map_markers (MarkerArray)                │
          └─→ /scene_graph/map_poses                                │
                    │                                                │
                    ↓                                                │
        ┌───────────────────────────────────┐                       │
        │                                   │                       │
        │  MAPPING PHASE           or    NAVIGATION PHASE          │
        │                                   │                       │
        ↓                                   ↓                       │
┌──────────────────────┐         ┌──────────────────────┐          │
│ topological_mapper   │         │ place_recognition    │          │
│ (Record nodes)       │         │ (DINOv2 + Markers)   │          │
└──────┬───────────────┘         └──────┬───────────────┘          │
       │                                 │                          │
       ├─→ nodes.json                    ├─→ /localization/belief  │
       └─→ edges.json                    └─→ /localization/node_id │
                                                   │                 │
                                                   ↓                 │
                                         ┌─────────────────────────┐│
                                         │ topological_planner     ││
                                         │ (A* path planning)      ││
                                         └──────┬──────────────────┘│
                                                │                    │
                                                ├─→ /navigation/plan │
                                                │                    │
                                                ↓                    │
                                         ┌─────────────────────────┐│
                                         │ edge_executor           ││
                                         │ (Traverse edges)        ││
                                         └──────┬──────────────────┘│
                                                │                    │
                                                ↓                    │
                                         ┌─────────────────────────┐│
                                         │ reactive_controller     ││
                                         │ (Floor-following)       ││←┘
                                         └──────┬──────────────────┘
                                                │
                                                ├─→ /cmd_vel
                                                ↓
                                         ┌─────────────────────────┐
                                         │ serial_motor_bridge     │
                                         │ (Arduino interface)     │
                                         └──────┬──────────────────┘
                                                │
                                                ↓
                                         [ MOTOR CONTROL ]
```

---

## Technology Stack

### Core Technologies

| Component | Technology | Purpose |
|-----------|-----------|---------|
| **Place Recognition** | DINOv2 (Meta AI) | Self-supervised vision transformer for scene embeddings |
| **Object Detection** | YOLOv8 | Real-time semantic marker detection |
| **Floor Segmentation** | SAM (Segment Anything) | Navigable surface identification |
| **Visual Odometry** | cuVSLAM / ORB-SLAM | Relative motion estimation |
| **Graph Planning** | NetworkX (A*) | Topological path planning |
| **Localization** | Custom Bayesian Filter | Multi-hypothesis belief tracking |
| **Control** | Custom Reactive | Floor-following with obstacle avoidance |

### Key Advantages Over Traditional SLAM

| Traditional Metric SLAM | Our Topological Approach |
|------------------------|--------------------------|
| Continuous (x,y,θ) estimation | Discrete place recognition |
| Requires precise map alignment | Tolerates map changes / furniture moved |
| Computationally expensive (particle filters, EKF) | Lightweight (embedding comparison, ~10ms) |
| Brittle to dynamic environments | Robust (semantic, not geometric) |
| Hard to interpret/debug | Explainable: "I see fridge → Kitchen" |
| Needs cm-level accuracy everywhere | Only precise when needed (grasping, docking) |
| Loop closure is critical | Naturally handles revisiting (place recognition) |

---

## Message Schemas and Topics

### Key ROS2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/scene_graph` | `std_msgs/String` | JSON of YOLO detections with 3D positions |
| `/scene_graph/map_markers` | `visualization_msgs/MarkerArray` | Markers in map frame for RViz |
| `/floor/mask` | `sensor_msgs/Image` | Binary mask of navigable floor |
| `/floor/navigability` | `std_msgs/Float32` | Traversability score [0-1] |
| `/localization/belief` | `custom_msgs/BeliefState` | Probability distribution over nodes |
| `/localization/node_id` | `std_msgs/String` | Most likely current node |
| `/navigation/plan` | `custom_msgs/NodePath` | Sequence of nodes to goal |
| `/navigation/status` | `std_msgs/String` | Current navigation state |
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands to motors |
| `/camera/odom/sample` | `nav_msgs/Odometry` | Visual odometry estimates |

### Scene Graph Message Format

`/scene_graph` publishes JSON strings with this structure:

```json
{
  "frame_id": "camera_color_optical_frame",
  "timestamp": 1700000000.123,
  "objects": [
    {
      "id": 1,
      "label": "refrigerator",
      "confidence": 0.92,
      "bbox": [120, 80, 200, 300],
      "centroid_px": [220, 190],
      "depth_m": 2.5,
      "camera_xyz": [0.34, -0.12, 2.5],
      "map_xyz": [1.5, 2.3, 0.0],
      "area": 60000
    }
  ]
}
```

### Node Database Schema

Stored in `data/nodes/nodes.json`:

```json
{
  "nodes": [
    {
      "id": "kitchen_001",
      "label": "Kitchen",
      "embedding": [0.23, 0.56, -0.12, ...],
      "markers": [
        {
          "label": "refrigerator",
          "direction_deg": 45,
          "distance_m": 2.5,
          "confidence": 0.92
        }
      ],
      "floor_traversability": 0.95,
      "image_path": "images/kitchen_001.jpg",
      "created_timestamp": 1700000000.123,
      "connections": ["hallway_002"]
    }
  ],
  "edges": [
    {
      "from": "kitchen_001",
      "to": "hallway_002",
      "action": "move_forward",
      "distance_m": 3.5,
      "duration_s": 5.2,
      "floor_quality": 0.88,
      "markers_along_path": ["door"],
      "traversals": 0,
      "last_traversed": null
    }
  ]
}
```

---

## Usage Workflow

### Phase 1: Mapping (Training)

```bash
# 1. Launch mapping mode
ros2 launch realsense_nav topological_mapping.launch.py

# 2. Drive robot with joystick, press button to create nodes
# System automatically:
#   - Extracts DINOv2 embeddings
#   - Records YOLO markers
#   - Computes floor quality
#   - Stores node data

# 3. Save map when done
ros2 service call /mapper/save_map std_srvs/Trigger
# Saves to: data/nodes/nodes.json
```

### Phase 2: Navigation (Autonomous)

```bash
# 1. Launch navigation mode
ros2 launch realsense_nav topological_navigation.launch.py

# 2. Set goal via voice or topic
ros2 topic pub /navigation/goal std_msgs/String "data: 'bedroom'" --once

# Or via voice:
# Say: "Go to bedroom"

# 3. Monitor status
ros2 topic echo /localization/belief
ros2 topic echo /navigation/status

# 4. Visualize in RViz
rviz2 -d config/topological_nav.rviz
```

### Debugging Commands

```bash
# Check current place recognition
ros2 topic echo /localization/belief

# Verify marker detections
ros2 topic echo /scene_graph

# View floor segmentation
ros2 run rqt_image_view rqt_image_view /floor/mask

# Test place recognition manually
ros2 service call /place_recognition/query custom_msgs/srv/QueryPlace "{}"

# Force re-localization
ros2 service call /localization/reset std_srvs/Empty
```

---

## Implementation Checklist

### Phase 1: Core Perception (DONE ✅)
- [x] YOLO detection (`scene_graph_node.py`)
- [x] Scene graph mapper with camera_link transform (`scene_graph_mapper.py`)
- [x] SAM floor segmentation
- [x] Visual odometry integration
- [x] IMU integration

### Phase 2: Mapping (DONE ✅)
- [x] DINOv2 embedding extraction node (`dinov3_embedding_node.py`)
- [x] Node creation and storage system (`topological_mapper_node.py`)
- [x] Edge recording with metadata (auto-edge creation within threshold)
- [x] Joystick trigger for manual node creation (`joystick_node_recorder.py`)
- [x] Map save/load services (JSON persistence)
- [x] Launch file for mapping phase (`topological_mapping.launch.py`)
- [ ] Auto-detection of node creation points (future: scene change detection)

### Phase 3: Localization (IN PROGRESS �)
- [x] DINOv2-based place recognition (`place_recognition_node.py`)
- [x] Cosine similarity matching with softmax probability
- [x] Temporal smoothing for belief stability
- [x] Launch file for navigation phase (`topological_navigation.launch.py`)
- [ ] Marker-based validation (future: combine with visual similarity)
- [ ] Motion consistency tracker (future: odometry + graph connectivity)
- [ ] Bayesian belief fusion (future: combine all observations)
- [ ] Confidence-based decision making
- [ ] Recovery strategies (rotation, re-localization)

### Phase 4: Navigation (TODO 📋)
- [ ] Topological path planner (A*)
- [ ] Edge executor controller
- [ ] Reactive floor-following controller
- [ ] Navigation monitor and error handling
- [ ] Goal specification interface

### Phase 5: Integration & Testing (TODO 📋)
- [ ] Voice command integration
- [ ] RViz visualization plugins
- [ ] Performance benchmarking
- [ ] Real-world testing in multiple environments
- [ ] Documentation and tutorials

---

## Performance Characteristics

### Expected Performance

| Metric | Target | Notes |
|--------|--------|-------|
| Place Recognition Latency | <50ms | DINOv2 forward pass on GPU |
| Localization Update Rate | 5-10 Hz | Belief update frequency |
| Reactive Control Rate | 30 Hz | Floor-following velocity commands |
| Mapping Speed | Real-time | No post-processing required |
| Memory per Node | ~2 KB | Embedding + metadata (without images) |
| Map Size | 100-500 nodes | Typical indoor building floor |

### Computational Requirements

- **GPU**: Recommended for DINOv2 (RTX 3060 or better)
- **CPU**: 4+ cores for parallel processing
- **RAM**: 4GB+ for large maps
- **Storage**: ~1MB per node (with images)

---

## Failure Modes and Recovery

### Common Failure Scenarios

| Failure | Detection | Recovery |
|---------|-----------|----------|
| **Lost (low confidence)** | All P(node) < 0.5 | Rotate 360°, move to vantage point |
| **Stuck (obstacle)** | No forward progress | Rotate and find alternate path |
| **Wrong node** | Marker mismatch | Re-localize, update belief |
| **Dynamic changes** | Expected markers missing | Use visual similarity only |
| **Poor lighting** | Low detection confidence | Slow down, increase sampling |
| **Kidnapped robot** | Abrupt location change | Global re-localization |

### Uncertainty-Aware Behavior

The system adapts based on confidence:

- **High confidence (>0.8)**: Normal navigation speed
- **Medium confidence (0.5-0.8)**: Slow down, gather more observations
- **Low confidence (<0.5)**: Stop and re-localize
- **Ambiguous (multiple high-prob nodes)**: Move to disambiguating location

---

## Future Enhancements

### Planned Features

1. **Multi-floor support**: Hierarchical topological maps
2. **Semantic scene understanding**: Room type classification
3. **Dynamic object tracking**: Moving people, furniture
4. **Active exploration**: Automatic map building
5. **Social navigation**: Human-aware path planning
6. **Long-term adaptation**: Update node embeddings over time
7. **Collaborative mapping**: Multi-robot map merging

### Research Directions

- **Uncertainty quantification**: Better confidence estimates
- **Transfer learning**: Pre-trained embeddings for new environments
- **Affordance-based planning**: Action-centric navigation
- **Attention mechanisms**: Focus on salient landmarks

---

## References and Related Work

### Key Papers

1. **DINOv2**: "DINOv2: Learning Robust Visual Features without Supervision" (Meta AI, 2023)
2. **NetVLAD**: "NetVLAD: CNN architecture for weakly supervised place recognition" (Arandjelovic et al., 2016)
3. **SAM**: "Segment Anything" (Meta AI, 2023)
4. **RatSLAM**: "RatSLAM: A Hippocampal Model for Simultaneous Localization and Mapping" (Milford et al., 2004)
5. **Topological SLAM**: "SemanticFusion: Dense 3D semantic mapping with convolutional neural networks" (McCormac et al., 2017)

### Inspirations

- **Cognitive mapping** (Tolman, 1948): Rats navigate using cognitive maps, not metric coordinates
- **Place cells** (O'Keefe & Nadel, 1978): Hippocampal neurons encode discrete locations
- **Affordance theory** (Gibson, 1979): Navigation is about action possibilities, not geometry

---

## Contact and Support

For questions or contributions:
- Repository: `roboticsraushan/wheel_control`
- Documentation: See `docs/` folder
- Issues: Use GitHub issue tracker

---

**Last Updated**: November 18, 2025
**Version**: 3.0 (Topological Navigation)

# Memory Package Integration Guide

## Overview

The **memory** package has been created as a new ROS 2 package to provide semantic and episodic memory capabilities for your robot navigation system. This package evolves from initial floorplan loading to comprehensive semantic and episodic memory management.

## Package Structure

```
src/memory/
├── CMakeLists.txt                    # Build configuration
├── package.xml                       # Package metadata
├── setup.py                          # Python setup
├── README.md                         # Package documentation
├── config/
│   └── memory.yaml                   # Configuration for all nodes
├── launch/
│   └── memory.launch.py              # Launch file for all memory nodes
├── memory/                           # Python package
│   ├── __init__.py
│   ├── floorplan_manager_node.py     # Floorplan loading and management
│   ├── semantic_memory_node.py       # Semantic entity tracking
│   └── episodic_memory_node.py       # Episode recording
└── resource/
    └── memory                        # Package resource marker
```

## Components

### 1. Floorplan Manager (`floorplan_manager_node.py`)

**Purpose**: Initial entry point - loads floorplan images and metadata

**Features**:
- Loads floorplan PNG images
- Parses floorplan metadata (resolution, origin, locations)
- Publishes floorplan to `/memory/floorplan` topic
- Tracks robot position in map frame
- Auto-creates placeholder if floorplan not found

**Data Sources**:
- `data/floorplan.png` - the floorplan image
- `data/floorplan_metadata.yaml` - spatial metadata

**Example Metadata Structure**:
```yaml
name: floor_1
resolution: 0.05  # meters per pixel
origin:
  x: 0.0
  y: 0.0
locations:
  - id: L1
    name: kitchen
    bounds: [0, 0, 100, 150]
  - id: L2
    name: hallway
    bounds: [100, 0, 250, 150]
```

### 2. Semantic Memory (`semantic_memory_node.py`)

**Purpose**: Track semantic understanding of environment

**Features**:
- Maintains persistent object entity database
- Associates detections with semantic labels
- Tracks object confidence and observation count
- Supports location semantic labeling (e.g., "kitchen")
- Publishes semantic map for downstream consumers

**How It Works**:
1. Subscribes to `/scene_graph/detections` from YOLO detector
2. Creates/updates semantic entities based on detection ID and label
3. Tracks confidence, first seen, last seen, observation count
4. Publishes semantic map as JSON to `/memory/semantic_map`

**Example Output**:
```json
{
  "timestamp": "2025-11-03T15:37:00.123456",
  "entities": {
    "1": {
      "id": "1",
      "label": "chair",
      "confidence": 0.92,
      "first_seen": "2025-11-03T15:30:00",
      "last_seen": "2025-11-03T15:37:00",
      "observations": 47,
      "attributes": {}
    }
  },
  "locations": {
    "L1": "kitchen"
  },
  "relationships": []
}
```

### 3. Episodic Memory (`episodic_memory_node.py`)

**Purpose**: Record robot experiences as episodes

**Features**:
- Creates episodes with start/end times
- Records events, motion data, and scene snapshots
- Persists episodes to JSON files
- Auto-loads previous episodes on startup
- Automatically ends episodes on node shutdown

**How It Works**:
1. On startup, creates a "default_session" episode
2. Subscribes to `/scene_graph/detections` and stores snapshots
3. Subscribes to `/cmd_vel` and records motion history
4. Episodes saved to `data/episodes/ep_*.json` files
5. On shutdown, properly ends current episode

**Example Episode File** (`data/episodes/ep_20251103_153700_1.json`):
```json
{
  "id": "ep_20251103_153700_1",
  "label": "exploring_kitchen",
  "start_time": "2025-11-03T15:37:00.123456",
  "end_time": "2025-11-03T15:37:30.654321",
  "duration_seconds": 30.5,
  "events": [
    {
      "timestamp": "2025-11-03T15:37:05",
      "type": "object_detected",
      "description": "Found chair",
      "data": {"confidence": 0.95}
    }
  ],
  "snapshots": [...],
  "motion_data": [...]
}
```

## Installation & Building

### Step 1: Build the package

```bash
cd /home/raushan/control_one/wheel_control
colcon build --packages-select memory
```

### Step 2: Source the workspace

```bash
source install/setup.bash
```

### Step 3: Verify installation

```bash
# Check that console scripts are available
ros2 run memory floorplan_manager --help
ros2 run memory semantic_memory --help
ros2 run memory episodic_memory --help
```

## Running the Memory System

### Option 1: Run all memory nodes with launch file

```bash
ros2 launch memory memory.launch.py
```

### Option 2: Run individual nodes

```bash
# Terminal 1: Floorplan Manager
ros2 run memory floorplan_manager

# Terminal 2: Semantic Memory
ros2 run memory semantic_memory

# Terminal 3: Episodic Memory
ros2 run memory episodic_memory
```

### Option 3: Configure with custom parameters

```bash
ros2 launch memory memory.launch.py \
  params_file:=/path/to/custom_memory.yaml
```

## Data Files Setup

Create data directory structure:

```bash
mkdir -p data/episodes

# Create a sample floorplan metadata file
cat > data/floorplan_metadata.yaml << 'EOF'
name: default_environment
resolution: 0.05
origin:
  x: 0.0
  y: 0.0
locations:
  - id: L1
    name: main_area
    bounds: [0, 0, 640, 480]
EOF

# Create or copy floorplan image
# cp /path/to/your/floorplan.png data/floorplan.png
```

## Integration with Existing System

### With Training Mode Launch

The memory system can be integrated into `training_mode.launch.py`:

```python
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

# In generate_launch_description():
memory_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        os.path.join(
            get_package_share_directory('memory'),
            'launch',
            'memory.launch.py'
        )
    ])
)

# Add to launch description:
ld.add_action(memory_launch)
```

### Data Flow Integration

```
Camera → YOLO Detector → /scene_graph/detections
            ↓
    Semantic Memory (entity tracking)
            ↓
    Episodic Memory (records experiences)
            ↓
    /memory/semantic_map (for downstream use)
    /memory/episodes (persistent storage)
```

### Topic Connections

```yaml
Subscriptions:
  - scene_graph_node → scene_graph/detections → semantic_memory
  - cmd_vel (motion controller) → episodic_memory
  - robot/pose → floorplan_manager

Publications:
  - floorplan_manager → /memory/floorplan (Image)
  - floorplan_manager → /memory/floorplan_info (JSON)
  - semantic_memory → /memory/semantic_map (JSON)
  - episodic_memory → /memory/episodes (JSON summary)
  - episodic_memory → /memory/current_episode (JSON details)
```

## Configuration

Edit `src/memory/config/memory.yaml`:

```yaml
floorplan_manager:
  ros__parameters:
    floorplan_path: "data/floorplan.png"
    metadata_path: "data/floorplan_metadata.yaml"
    publish_rate: 1.0  # Hz

semantic_memory:
  ros__parameters:
    semantic_map_file: "data/semantic_map.json"
    max_entities: 1000
    publish_rate: 1.0

episodic_memory:
  ros__parameters:
    episode_storage_path: "data/episodes"
    max_episodes: 1000
    episode_timeout: 600.0  # seconds
    publish_rate: 1.0
```

## Future Enhancement Roadmap

### Phase 1: Floorplan (Current)
- ✅ Load and serve floorplan images
- ✅ Parse metadata
- ✅ Publish spatial context

### Phase 2: Semantic Memory (Current)
- ✅ Track semantic entities
- ✅ Maintain object persistence
- ⏳ Query service for semantic lookups
- ⏳ Infer object relationships

### Phase 3: Episodic Memory (Current)
- ✅ Record experiences as episodes
- ✅ Store events and snapshots
- ⏳ Episode retrieval and replay
- ⏳ Temporal queries

### Phase 4: Advanced Memory (Future)
- Memory consolidation (compress old memories)
- Associative memory (link related episodes)
- Experience replay for learning
- Integration with LLM for natural language queries
- Forgetting/pruning old memories based on relevance
- Active memory management and summarization

## Example Usage Patterns

### Query semantic map at runtime

```python
# Create a client to query semantic memory
from std_msgs.msg import String
import json

# Publish query
query = String()
query.data = json.dumps({
    "query": "get_entities_with_label",
    "label": "chair"
})
semantic_query_pub.publish(query)

# Subscribe to responses
def semantic_response_cb(msg):
    response = json.loads(msg.data)
    chairs = response.get("entities", [])
    for chair in chairs:
        print(f"Chair ID {chair['id']}: confidence {chair['confidence']}")

semantic_response_sub = node.create_subscription(
    String,
    '/memory/semantic_map_response',
    semantic_response_cb,
    10
)
```

### Record custom events in episode

```python
# Add event to current episode
event_data = {
    "custom_field": "custom_value"
}
node.add_event_to_episode(
    event_type="custom_event",
    description="Something interesting happened",
    data=event_data
)
```

### Access episode data programmatically

```python
import json

# Load a specific episode
with open("data/episodes/ep_20251103_153700_1.json") as f:
    episode = json.load(f)
    
print(f"Episode: {episode['label']}")
print(f"Duration: {episode['duration_seconds']}s")
print(f"Events: {len(episode['events'])}")

for event in episode['events']:
    print(f"  - {event['type']}: {event['description']}")
```

## Troubleshooting

### Floorplan not found
- Check that `data/floorplan.png` exists
- Verify the path in `config/memory.yaml`
- Node will create placeholder if file missing

### Semantic entities not appearing
- Verify scene_graph_detector is running
- Check `/scene_graph/detections` topic: `ros2 topic echo /scene_graph/detections`
- Ensure YOLO model is loaded: check detector logs

### Episodes not saving
- Verify `data/episodes/` directory exists and is writable
- Check disk space
- Look for permissions issues on data directory

### High memory usage
- Reduce `max_entities` or `max_episodes` parameters
- Episodes keep last 100 snapshots/motion records (configurable)
- Prune old episodes manually from `data/episodes/`

## Next Steps

1. **Build the package**:
   ```bash
   colcon build --packages-select memory
   source install/setup.bash
   ```

2. **Create floorplan data**:
   - Add your floorplan image to `data/floorplan.png`
   - Create metadata in `data/floorplan_metadata.yaml`

3. **Integrate with training launch**:
   - Add memory launch to `training_mode.launch.py`

4. **Test the system**:
   ```bash
   ros2 launch memory memory.launch.py
   ros2 topic list | grep /memory
   ```

5. **Add to your navigation pipeline** for experience recording and semantic context

---

For more details, see `src/memory/README.md`

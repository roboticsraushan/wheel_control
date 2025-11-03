# Memory Package

## Overview

The **memory** package provides semantic and episodic memory capabilities for the robot navigation system. It manages:

1. **Floorplan Management**: Loads and serves spatial layout information
2. **Semantic Memory**: Stores semantic understanding of the environment (object types, location labels, relationships)
3. **Episodic Memory**: Records experiences as episodes (events, motion history, scene snapshots)

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    Memory System                        │
├─────────────────────────────────────────────────────────┤
│                                                         │
│  ┌──────────────────────────────────────────────────┐  │
│  │        Floorplan Manager Node                   │  │
│  │  - Loads floorplan images                       │  │
│  │  - Publishes spatial context                    │  │
│  │  - Tracks robot position                        │  │
│  └──────────────────────────────────────────────────┘  │
│                        ↓                                 │
│  ┌──────────────────────────────────────────────────┐  │
│  │        Semantic Memory Node                      │  │
│  │  - Object classification & persistence          │  │
│  │  - Location semantic labels                     │  │
│  │  - Spatial relationships                        │  │
│  └──────────────────────────────────────────────────┘  │
│                        ↓                                 │
│  ┌──────────────────────────────────────────────────┐  │
│  │        Episodic Memory Node                      │  │
│  │  - Records experiences as episodes              │  │
│  │  - Stores event sequences                       │  │
│  │  - Maintains motion history                     │  │
│  └──────────────────────────────────────────────────┘  │
│                                                         │
└─────────────────────────────────────────────────────────┘
```

## Nodes

### 1. Floorplan Manager (`floorplan_manager_node.py`)

**Purpose**: Loads and manages floorplan data as the spatial foundation for memory.

**Topics**:
- **Publish**:
  - `/memory/floorplan` (sensor_msgs/Image): Floorplan image
  - `/memory/floorplan_info` (std_msgs/String): JSON metadata
  - `/memory/robot_position` (geometry_msgs/PointStamped): Current robot position
- **Subscribe**:
  - `/robot/pose` (geometry_msgs/PointStamped): Robot pose updates

**Parameters**:
- `floorplan_path`: Path to floorplan image (default: `data/floorplan.png`)
- `metadata_path`: Path to floorplan metadata YAML (default: `data/floorplan_metadata.yaml`)
- `publish_rate`: Publishing frequency in Hz (default: 1.0)

### 2. Semantic Memory (`semantic_memory_node.py`)

**Purpose**: Maintains semantic understanding of objects and locations.

**Topics**:
- **Publish**:
  - `/memory/semantic_map` (std_msgs/String): JSON of all semantic entities
  - `/memory/object_classification` (std_msgs/String): Object labels and relationships
- **Subscribe**:
  - `/scene_graph/detections` (std_msgs/String): YOLO/segmentation detections

**Parameters**:
- `semantic_map_file`: Path to semantic map storage (default: `data/semantic_map.json`)
- `max_entities`: Maximum semantic entities to track (default: 1000)
- `publish_rate`: Publishing frequency in Hz (default: 1.0)

**Semantic Entity Schema**:
```json
{
  "id": "1",
  "label": "chair",
  "confidence": 0.87,
  "first_seen": "2025-11-03T15:37:00.123456",
  "last_seen": "2025-11-03T15:37:10.654321",
  "observations": 5,
  "attributes": {
    "color": "red",
    "material": "fabric"
  }
}
```

### 3. Episodic Memory (`episodic_memory_node.py`)

**Purpose**: Records robot experiences as episodes for later retrieval.

**Topics**:
- **Publish**:
  - `/memory/episodes` (std_msgs/String): Summary of all episodes
  - `/memory/current_episode` (std_msgs/String): Active episode details
- **Subscribe**:
  - `/scene_graph/detections` (std_msgs/String): Scene snapshots
  - `/cmd_vel` (geometry_msgs/Twist): Robot motion commands

**Parameters**:
- `episode_storage_path`: Directory for episode files (default: `data/episodes`)
- `max_episodes`: Maximum episodes in memory (default: 1000)
- `episode_timeout`: Episode duration before auto-end (default: 600s)
- `publish_rate`: Publishing frequency in Hz (default: 1.0)

**Episode Schema**:
```json
{
  "id": "ep_20251103_153700_1",
  "label": "exploring_kitchen",
  "start_time": "2025-11-03T15:37:00.123456",
  "end_time": "2025-11-03T15:37:30.654321",
  "duration_seconds": 30.5,
  "events": [
    {
      "timestamp": "2025-11-03T15:37:05.000000",
      "type": "object_detected",
      "description": "Found chair",
      "data": {"confidence": 0.95}
    }
  ],
  "snapshots": [...],
  "motion_data": [...]
}
```

## Data Storage

```
data/
├── floorplan.png                    # Floorplan image
├── floorplan_metadata.yaml          # Floorplan metadata
├── semantic_map.json                # Persistent semantic entities
└── episodes/                        # Episode storage
    ├── ep_20251103_153700_1.json
    ├── ep_20251103_154200_2.json
    └── ...
```

## Launch

### Start all memory nodes:
```bash
ros2 launch memory memory.launch.py
```

### Start individual nodes:
```bash
ros2 run memory floorplan_manager
ros2 run memory semantic_memory
ros2 run memory episodic_memory
```

## Integration

The memory system integrates with:
- **Scene Graph**: Processes `/scene_graph/detections` from YOLO detector
- **Navigation**: Provides spatial context for path planning
- **Voice LLM Navigator**: Can query semantic/episodic memory for context
- **Training System**: Records experiences during training phases

## Future Enhancements

1. **Persistent Storage**: Save/load semantic maps and episodes
2. **Memory Queries**: Services for querying by type, location, time range
3. **Experience Replay**: Re-run episodes for learning
4. **Semantic Reasoning**: Infer relationships between objects
5. **Temporal Context**: Query "what happened when"
6. **Integration with LLM**: Use memory for context in natural language understanding

## Example: Adding Semantic Context to Navigation

```python
# In your navigation node:
from memory import SemanticMemory

# Query semantic memory
semantic_query = String()
semantic_query.data = json.dumps({
    "query": "get_locations_with_label",
    "label": "kitchen"
})
semantic_client.publish(semantic_query)

# Receive semantic entities
response = wait_for_response(timeout=5.0)
locations = json.loads(response.data)
```

## Example: Recording an Episode

```python
# Start episode
memory_client.call_start_episode("exploring_kitchen")

# Do navigation...

# Log an event
memory_client.call_add_event(
    event_type="object_detected",
    description="Found table",
    data={"confidence": 0.92, "bbox": [100, 200, 300, 400]}
)

# End episode
memory_client.call_end_episode()
```

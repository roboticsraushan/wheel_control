# 🎨 Semantic Twin Visualization System

An interactive, multi-view visualization system for the robot's topological navigation stack. Provides real-time insights into the robot's understanding of its environment through synchronized 2D map, 3D scene graph, and status displays.

## 🌟 Features

### 1. **Fused Map View (2D)**
- **Static Floorplan Background**: Shows the metric floor plan image
- **Topological Graph Overlay**: 
  - Junction nodes (circular markers) with IDs
  - Edges connecting navigable junctions
  - Active navigation path highlighted in green
- **Live Robot Position**: Red triangle showing current pose and heading
- **Interactive Junction Inspector**: Click any junction to see:
  - RGB image captured at that location
  - Junction pose (x, y, theta)
  - Connected junctions

### 2. **Semantic Scene Graph (3D/Abstract)**
- **Relationship-Focused View**: Not metric-accurate, emphasizes connections
- **Semantic Nodes**: Furniture and objects with cropped images (rectangles)
- **Junction Nodes**: Waypoints in the topological map (circles)
- **Visual Edges**: 
  - Thick lines: Object ↔ Junction associations
  - Thin lines: Junction ↔ Junction navigation paths
- **Dynamic Highlighting**: Active navigation path pulses

### 3. **Live Status Panel**
- **Current State**: "Navigating to bedroom_sofa" or "Idle"
- **Current Junction**: Which junction the robot is at/near
- **Navigation Goal**: Target destination node
- **Robot Pose**: Current (x, y) position
- **Junction Visibility**: Green/red indicator showing if target is visible
- **Recognition Confidence**: Progress bar (0-100%) showing junction match confidence
- **Recent Alerts**: Scrolling list of navigation events and warnings

### 4. **Foxglove Studio Integration**
Pre-configured layout with:
- 3D visualization with transforms
- Camera feed display
- Junction recognition confidence plot
- State transition tracking
- Topic graph
- Navigation alerts table

## 🚀 Quick Start

### Launch with Training Mode
```bash
ros2 launch realsense_nav training_mode.launch.py
```

### Launch with Navigation Mode
```bash
ros2 launch realsense_nav junction_navigation.launch.py
```

### Access Web Interface
Open browser to: **http://localhost:8080**

### Use Foxglove Studio
1. Install Foxglove Studio: https://foxglove.dev/download
2. Connect to: `ws://localhost:9090` (rosbridge)
3. Import layout: `src/realsense_nav/realsense_nav/visualization/foxglove_layout.json`

## 📡 ROS Topics Subscribed

| Topic | Message Type | Purpose |
|-------|--------------|---------|
| `/map` | `nav_msgs/OccupancyGrid` | Static floor map |
| `/robot/pose` | `geometry_msgs/PoseStamped` | Robot localization |
| `/cmd_vel` | `geometry_msgs/Twist` | Robot velocity |
| `/robot/current_junction` | `std_msgs/String` | Current junction ID |
| `/navigation/task` | `std_msgs/String` | Navigation task (JSON) |
| `/navigation/junction_path` | `std_msgs/String` | Planned junction sequence |
| `/junction/detected` | `std_msgs/String` | Detected junction info |
| `/junction/recognition` | `std_msgs/Float32` | Recognition confidence |
| `/junction/visibility` | `std_msgs/Bool` | Is target visible? |
| `/navigation/alerts` | `std_msgs/String` | Navigation warnings |
| `/scene_graph` | `std_msgs/String` | Current scene detections |
| `/navigation/arrived` | `std_msgs/Bool` | Arrival notification |
| `/navigation/status` | `std_msgs/String` | General status updates |

## 🎮 Interactive Features

### Map View Interactions
- **Click Junction**: Pop up showing junction image and details
- **Pan/Zoom**: (Future) Mouse wheel to zoom, drag to pan
- **Robot Following**: (Future) Toggle to keep robot centered

### Scene Graph Interactions
- **Node Selection**: (Future) Click to highlight related junctions
- **Path Highlighting**: Active navigation path is auto-highlighted
- **Layout Adjustment**: (Future) Drag nodes to rearrange

## 🔧 Configuration

### Web Server Ports
Edit launch parameters:
```python
parameters=[{
    'web_port': 8080,           # HTTP server
    'websocket_port': 8081,     # WebSocket updates
}]
```

### Data Paths
```python
parameters=[{
    'map_yaml_path': 'data/maps/my_floorplan.yaml',
    'junction_db_path': 'data/junctions/junction_db.json',
    'topo_map_path': 'data/maps/topological_map.json',
}]
```

## 📊 Performance

- **Update Rate**: 10 Hz (configurable in node)
- **WebSocket Latency**: < 50ms typical
- **Browser Requirements**: Modern browser with Canvas & WebSocket support
- **Recommended**: Chrome/Edge 90+, Firefox 88+

## 🎨 Visualization Color Scheme

| Element | Color | Meaning |
|---------|-------|---------|
| 🔵 Cyan | `#00d9ff` | Junctions |
| 🟢 Green | `#00ff88` | Active path / Success |
| 🔴 Red | `#ff4b2b` | Robot / Semantic nodes |
| 🟠 Orange | `#ffaa00` | Current junction |
| 🟡 Yellow | `#ffaa00` | Warnings |

## 🐛 Debugging

### WebSocket Not Connecting
Check that the visualizer node is running:
```bash
ros2 node list | grep semantic_visualizer
```

### No Map Displayed
Verify map file exists:
```bash
ls -la data/maps/my_floorplan.yaml
ls -la data/maps/my_floorplan.pgm
```

Check topic:
```bash
ros2 topic echo /map --once
```

### Junction Images Not Showing
Check junction database:
```bash
cat data/junctions/junction_db.json | jq '.junctions[0]'
```

Verify image paths are absolute or relative to workspace root.

### Browser Console
Press F12 in browser to see WebSocket connection status and errors.

## 🔮 Future Enhancements

- [ ] 3D rendering with Three.js for better scene graph visualization
- [ ] Time-travel debugging (replay recorded navigation sessions)
- [ ] Multi-robot support
- [ ] Heatmap overlay showing visited areas
- [ ] Real-time editing of junction graph
- [ ] Voice command integration in web UI
- [ ] Mobile-responsive design
- [ ] AR view using device camera

## 📚 Architecture

```
┌─────────────────────────────────────────┐
│   semantic_visualizer_node.py          │
│   ┌─────────────────────────────────┐  │
│   │  State Aggregator               │  │
│   │  - Subscribes to 13 topics      │  │
│   │  - Loads static data            │  │
│   │  - Compiles unified state       │  │
│   └──────────┬──────────────────────┘  │
│              │                           │
│   ┌──────────▼──────────────────────┐  │
│   │  WebSocket Broadcaster          │  │
│   │  - 10Hz state updates           │  │
│   │  - JSON serialization           │  │
│   └──────────┬──────────────────────┘  │
│              │                           │
└──────────────┼───────────────────────────┘
               │ ws://localhost:8081
               ▼
┌─────────────────────────────────────────┐
│   Web Frontend (index.html)             │
│   ┌─────────────┐  ┌─────────────────┐ │
│   │  Map Canvas │  │ Scene Graph     │ │
│   │  (2D)       │  │ Canvas (3D)     │ │
│   └─────────────┘  └─────────────────┘ │
│   ┌─────────────────────────────────┐  │
│   │  Status Panel                   │  │
│   │  - Real-time metrics            │  │
│   │  - Confidence bars              │  │
│   │  - Alert feed                   │  │
│   └─────────────────────────────────┘  │
└─────────────────────────────────────────┘
```

## 🤝 Integration with Existing Components

The visualizer is **read-only** and does not interfere with navigation:
- ✅ No control commands published
- ✅ Passive observation of all topics
- ✅ Can be started/stopped independently
- ✅ Zero impact on navigation performance

## 📄 License

Part of the wheel_control navigation stack.

---

**Created**: November 4, 2025  
**Author**: Navigation Team  
**Status**: ✅ Integrated with training and navigation modes

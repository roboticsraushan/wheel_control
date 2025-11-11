# 🚀 Semantic Twin Visualization - Quick Start Guide

## Prerequisites

```bash
# Source your workspace
cd /home/raushan/control_one/wheel_control
source install/setup.bash

# Ensure you have the required Python packages
pip3 install opencv-python numpy pyyaml websocket-server
```

## 🎬 Quick Demo (Without Robot)

Test the visualization with sample data:

```bash
# Terminal 1: Start ROS core (if not already running)
# (ROS 2 doesn't need this, skip)

# Terminal 2: Publish sample map
ros2 run realsense_nav semantic_visualizer_node

# Terminal 3: Open browser
firefox http://localhost:8080
# or
google-chrome http://localhost:8080
```

The visualizer will load static data from:
- `data/maps/my_floorplan.yaml` - Floor plan
- `data/junctions/junction_db.json` - Junction database  
- `data/maps/topological_map.json` - Topological graph

## 🤖 Live Demo (With Robot)

### Option 1: Training Mode

```bash
# Terminal 1: Launch training mode (includes visualizer)
source install/setup.bash
ros2 launch realsense_nav training_mode.launch.py

# Terminal 2: Open web interface
firefox http://localhost:8080

# Terminal 3: Record junctions via voice
# Say "record junction" at each waypoint
# The visualizer will update in real-time

# When done, say "save map"
```

### Option 2: Navigation Mode

```bash
# Terminal 1: Launch navigation stack (includes visualizer)
source install/setup.bash
ros2 launch realsense_nav junction_navigation.launch.py

# Terminal 2: Open web interface
firefox http://localhost:8080

# Terminal 3: Send navigation command
ros2 topic pub /voice/command std_msgs/String "data: 'go to sofa'" --once

# Watch the robot navigate in real-time on the visualizer!
```

## 🔍 What You'll See

### 2D Fused Map View (Left Panel)
- **Gray overlay**: Your floor plan image
- **Blue circles**: Junction waypoints with IDs
- **Gray lines**: Navigable edges between junctions
- **Green dashed line**: Active navigation path
- **Red triangle**: Live robot position and heading
- **Click any junction**: Pop-up with RGB image from that viewpoint

### 3D Scene Graph (Right Panel)
- **Blue circles**: Junction nodes
- **Red rectangles**: Semantic object nodes (table, chair, sofa, etc.)
- **Cyan dashed lines**: Object ↔ Junction associations
- **Gray lines**: Junction ↔ Junction connections
- **Green highlights**: Active navigation path

### Status Panel (Bottom)
- **Current State**: What the robot is doing
- **Current Junction**: Where the robot is
- **Navigation Goal**: Target destination
- **Robot Pose**: Live (x, y) position
- **Junction Visibility**: 🟢 Green = visible, 🔴 Red = occluded
- **Recognition Confidence**: 0-100% bar showing junction match strength
- **Recent Alerts**: Scrolling event log

## 🐛 Troubleshooting

### "WebSocket disconnected" in browser

**Check if visualizer node is running:**
```bash
ros2 node list | grep semantic_visualizer
```

**Check logs:**
```bash
ros2 node info /semantic_visualizer
```

**Restart visualizer:**
```bash
ros2 run realsense_nav semantic_visualizer_node
```

### Map not displaying

**Verify map file exists:**
```bash
ls -la data/maps/my_floorplan.yaml
ls -la data/maps/my_floorplan.pgm
```

**Check /map topic:**
```bash
ros2 topic echo /map --once
```

**Check visualizer parameters:**
```bash
ros2 param list /semantic_visualizer
ros2 param get /semantic_visualizer map_yaml_path
```

### Junctions not showing

**Verify junction database:**
```bash
cat data/junctions/junction_db.json | jq '.junctions | length'
```

**Check if junctions array is populated:**
```bash
cat data/junctions/junction_db.json | jq '.junctions[0]'
```

### Robot not visible

**Check /robot/pose topic:**
```bash
ros2 topic echo /robot/pose
```

**Check odometry node:**
```bash
ros2 node list | grep odometry
```

### Port already in use

**Change ports in launch file:**
```python
parameters=[{
    'web_port': 8888,        # Change this
    'websocket_port': 8889,  # Change this
}]
```

**Or kill existing process:**
```bash
lsof -ti:8080 | xargs kill -9
lsof -ti:8081 | xargs kill -9
```

## 🎨 Customization

### Change Update Rate

Edit `semantic_visualizer_node.py`:
```python
# Change from 10Hz to 5Hz for lower CPU usage
self.create_timer(0.2, self.broadcast_state)  # 5Hz = 0.2s interval
```

### Change Color Scheme

Edit `index.html` and modify CSS/Canvas drawing colors:
```javascript
// Junction color
ctx.fillStyle = '#00d9ff';  // Cyan

// Robot color  
ctx.fillStyle = '#ff4b2b';  // Red

// Active path color
ctx.strokeStyle = '#00ff88'; // Green
```

### Add Custom Metrics

Edit `semantic_visualizer_node.py` `get_current_state()`:
```python
def get_current_state(self) -> dict:
    return {
        # ...existing fields...
        'custom': {
            'battery_level': self.battery_level,  # Add your own
            'cpu_usage': self.cpu_usage,
        }
    }
```

Then update `index.html` to display it in the status panel.

## 🦊 Foxglove Studio Setup

### Install Foxglove
```bash
# Download from https://foxglove.dev/download
# Or use snap:
sudo snap install foxglove-studio
```

### Launch Foxglove Bridge
```bash
# Terminal 1: Launch your robot stack
ros2 launch realsense_nav junction_navigation.launch.py

# Terminal 2: Launch rosbridge for Foxglove
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

### Import Layout
1. Open Foxglove Studio
2. Connect to `ws://localhost:9090`
3. Click **Layout** → **Import from file**
4. Select: `src/realsense_nav/realsense_nav/visualization/foxglove_layout.json`

### View Panels
- **3D View**: Shows map, robot pose, and transforms
- **Image View**: Camera feed tabs (robot view + junction recognition)
- **Plot**: Junction recognition confidence over time
- **State Transitions**: Junction changes
- **Indicator**: Junction visibility status (green/red)
- **Table**: Navigation alerts
- **Topic Graph**: Visualize ROS topic connections

## 📊 Performance Tips

### Reduce CPU Usage
- Lower visualizer update rate (from 10Hz to 5Hz)
- Use lower resolution floorplan image
- Reduce camera framerate in RealSense config

### Reduce Network Usage
- Run browser on same machine as robot
- Use localhost instead of remote IP
- Compress floorplan images (PNG with higher compression)

### Improve Responsiveness
- Use Chrome/Edge (better Canvas performance than Firefox)
- Close unused browser tabs
- Disable browser extensions
- Use hardware acceleration in browser settings

## 📸 Screenshots / Recording

### Capture Screenshot
Press F12 in browser → Console:
```javascript
// Capture map canvas
let canvas = document.getElementById('map-canvas');
let link = document.createElement('a');
link.download = 'map_view.png';
link.href = canvas.toDataURL();
link.click();
```

### Record Video
Use browser built-in screen recording or:
```bash
# Install simplescreenrecorder
sudo apt install simplescreenrecorder

# Or use OBS Studio
sudo apt install obs-studio
```

## 🎓 Next Steps

1. **Record Training Data**: Drive robot around, record junctions
2. **Build Topological Map**: Let topo_map_builder create the graph
3. **Test Navigation**: Give voice commands and watch visualization
4. **Analyze Behavior**: Use Foxglove for detailed analysis
5. **Customize**: Add your own visualizations and metrics

## 📚 Additional Resources

- **Full Documentation**: `src/realsense_nav/realsense_nav/visualization/README.md`
- **Navigation Architecture**: `NAVIGATION_ARCHITECTURE.md`
- **Implementation Checklist**: `IMPLEMENTATION_CHECKLIST.md`

---

**Questions?** Check the visualization README or navigation architecture docs.

**Found a bug?** The visualization is read-only and won't affect navigation. Report issues in your project tracker.

**Happy navigating! 🤖🗺️**

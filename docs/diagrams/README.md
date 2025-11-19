# System Architecture Diagrams

This directory contains D2 diagrams documenting the human-inspired topological navigation system.

## Viewing the Diagrams

### Option 1: Using D2 CLI (Recommended)

Install D2:
```bash
curl -fsSL https://d2lang.com/install.sh | sh -s --
```

Render diagrams to SVG:
```bash
# Render all diagrams
for file in docs/diagrams/*.d2; do
    d2 "$file" "${file%.d2}.svg"
done

# Or render individual diagram
d2 docs/diagrams/system_overview.d2 docs/diagrams/system_overview.svg
```

### Option 2: Online Viewer

Visit [D2 Playground](https://play.d2lang.com/) and paste the content of any `.d2` file.

### Option 3: VS Code Extension

Install the [D2 extension](https://marketplace.visualstudio.com/items?itemName=terrastruct.d2) for VS Code.

## Diagram Descriptions

### 1. `system_overview.d2`
**High-level system architecture** showing the three-layer hierarchy:
- Sensor Layer (Hardware)
- Perception Layer (Feature extraction)
- Cognitive Layer (Decision making)
- Control Layer (Actuation)

**Best for**: Understanding overall system structure, component relationships

### 2. `place_recognition.d2`
**Localization algorithm** detailing multi-modal place recognition:
- Visual similarity (DINOv2 embeddings)
- Marker validation (YOLO detections)
- Motion consistency (visual odometry)
- Bayesian fusion process
- Confidence-based decision making

**Best for**: Understanding "Where am I?" algorithm, sensor fusion

### 3. `navigation_flow.d2`
**Navigation execution** from goal to completion:
- Step 1: Topological planning (A* search)
- Step 2: Edge execution loop
- Step 3: Task completion
- Failure handling and recovery

**Best for**: Understanding autonomous navigation workflow, error handling

### 4. `mapping_phase.d2`
**Map creation during training** with joystick:
- Node creation triggers (manual, auto, semantic)
- Multi-modal data capture at each node
- Edge recording between nodes
- Storage structure (JSON + images)
- Resulting topological graph

**Best for**: Understanding how maps are built, what data is stored

### 5. `ros2_architecture.d2`
**ROS2 node and topic graph** showing:
- Hardware layer
- Camera nodes
- Perception nodes
- Mapping nodes (training mode)
- Navigation nodes (autonomous mode)
- Control nodes
- Topic connections

**Best for**: Understanding ROS2 implementation, message flow, node responsibilities

### 6. `data_structures.d2`
**Data schemas and formats**:
- Node structure (JSON schema)
- Edge structure
- Marker structure
- Belief state structure
- Complete graph structure
- Example JSON payloads

**Best for**: Understanding data models, implementing new nodes, debugging

## Architecture Philosophy

These diagrams illustrate a **human-inspired navigation system** that:

1. **Recognizes places** instead of computing precise coordinates
2. **Plans topologically** through discrete nodes, not continuous space
3. **Reacts locally** to floor and obstacles without global precision
4. **Handles uncertainty** with confidence-aware decision making
5. **Reasons semantically** using landmarks and scene understanding

### Key Differences from Traditional SLAM

| Traditional | Our Approach |
|-------------|--------------|
| Metric map (occupancy grid) | Topological graph (nodes + edges) |
| Continuous localization (x,y,θ) | Discrete place recognition |
| Geometric features (ORB, SIFT) | Semantic features (YOLO, DINOv2) |
| Single-hypothesis tracking | Multi-hypothesis belief |
| Requires cm-level precision | Tolerates uncertainty |

## Extending the Diagrams

To add new diagrams:

1. Create a new `.d2` file in this directory
2. Follow the existing naming convention: `{component}_{aspect}.d2`
3. Use consistent styling:
   - Blue tones (`#e1f5fe`) for input/sensors
   - Orange tones (`#fff3e0`) for processing
   - Purple tones (`#f3e5f5`) for cognition
   - Green tones (`#e8f5e9`) for control/output
   - Yellow tones (`#fff9c4`) for storage/data
4. Add icons from [Terrastruct Icons](https://icons.terrastruct.com/) where appropriate
5. Include annotations explaining key concepts
6. Update this README with a description

## Rendering Tips

### SVG Output (Best Quality)
```bash
d2 --theme=200 system_overview.d2 system_overview.svg
```

### PNG Output (For Documentation)
```bash
d2 --theme=200 system_overview.d2 system_overview.png
```

### PDF Output (For Presentations)
```bash
d2 --theme=200 system_overview.d2 system_overview.pdf
```

### Available Themes
- `0`: Neutral (default)
- `1`: Neutral Grey
- `3`: Cool Classics
- `4`: Mixed Berry Blue
- `200`: Terminal

## Related Documentation

- **Architecture**: See `../TOPOLOGICAL_NAVIGATION_ARCHITECTURE.md` for detailed system description
- **Implementation**: See `../IMPLEMENTATION_CHECKLIST.md` for development progress
- **Quick Start**: See `../QUICK_START_GUIDE.md` for running the system

## Questions?

For questions about the architecture or diagrams:
- Create an issue in the GitHub repository
- Refer to the main architecture document
- Check the inline comments in each diagram

---

**Last Updated**: November 18, 2025

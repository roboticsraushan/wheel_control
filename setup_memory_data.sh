#!/bin/bash
# Setup script for memory package data directories and default files

set -e

WORKSPACE_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
DATA_DIR="$WORKSPACE_DIR/data"

echo "Setting up memory package data structure..."

# Create data directories
mkdir -p "$DATA_DIR/episodes"
mkdir -p "$DATA_DIR"

# Create default floorplan metadata if it doesn't exist
if [ ! -f "$DATA_DIR/floorplan_metadata.yaml" ]; then
    cat > "$DATA_DIR/floorplan_metadata.yaml" << 'EOF'
# Floorplan Metadata
# Define the spatial layout and regions of your environment

name: default_environment
description: Default environment floorplan

# Resolution in meters per pixel
resolution: 0.05

# Origin point (top-left corner of floorplan in world coordinates)
origin:
  x: 0.0
  y: 0.0

# Define semantic locations/regions in the floorplan
locations:
  - id: L1
    name: main_area
    description: Main navigation area
    bounds: [0, 0, 640, 480]  # [x, y, width, height] in pixels

# Add more locations as needed:
# locations:
#   - id: L2
#     name: kitchen
#     bounds: [0, 0, 200, 200]
#   - id: L3
#     name: hallway
#     bounds: [200, 0, 440, 200]
#   - id: L4
#     name: living_room
#     bounds: [0, 200, 320, 480]
EOF
    echo "✓ Created default floorplan_metadata.yaml"
else
    echo "ℹ floorplan_metadata.yaml already exists"
fi

# Create default semantic map if it doesn't exist
if [ ! -f "$DATA_DIR/semantic_map.json" ]; then
    cat > "$DATA_DIR/semantic_map.json" << 'EOF'
{
  "entities": {},
  "locations": {},
  "relationships": []
}
EOF
    echo "✓ Created default semantic_map.json"
else
    echo "ℹ semantic_map.json already exists"
fi

# Create README for data directory
if [ ! -f "$DATA_DIR/README.md" ]; then
    cat > "$DATA_DIR/README.md" << 'EOF'
# Memory Package Data Directory

This directory stores data for the memory package:

## Files

- **floorplan.png**: The floorplan image file (PNG format)
  - Place your environment's floorplan image here
  - Update resolution in floorplan_metadata.yaml to match

- **floorplan_metadata.yaml**: Metadata about the floorplan
  - Resolution (meters per pixel)
  - Origin point (world coordinates)
  - Location definitions (semantic regions)

- **semantic_map.json**: Persistent semantic entities database
  - Object classifications
  - Location labels
  - Relationships between objects

## Directories

- **episodes/**: Episode storage
  - Individual episode JSON files (ep_*.json)
  - Created automatically by episodic_memory node
  - Can be manually archived or pruned

## Setup Instructions

1. Add your floorplan image:
   ```bash
   cp /path/to/your/floorplan.png data/floorplan.png
   ```

2. Update floorplan_metadata.yaml:
   - Set correct resolution (meters per pixel)
   - Define location regions
   - Add semantic labels

3. The memory nodes will:
   - Load floorplan on startup
   - Parse and serve metadata
   - Create episodes automatically
   - Maintain semantic entities

## Example Floorplan Metadata

See floorplan_metadata.yaml for format and examples.
EOF
    echo "✓ Created data/README.md"
fi

echo ""
echo "✅ Memory package data setup complete!"
echo ""
echo "Next steps:"
echo "1. Add your floorplan image: cp <your_floorplan.png> data/floorplan.png"
echo "2. Update data/floorplan_metadata.yaml with your environment"
echo "3. Build: colcon build --packages-select memory"
echo "4. Launch: ros2 launch memory memory.launch.py"
echo ""

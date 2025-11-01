#!/bin/bash
# start_voice_web_bridge.sh - Quick start script for voice web bridge

set -e

cd "$(dirname "$0")"

echo "======================================================"
echo "  Starting Voice Web Bridge for ROS2 Navigation"
echo "======================================================"

# Source ROS2 and workspace
echo "[1/3] Sourcing ROS2 environment..."
source install/local_setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash

echo "[2/3] Starting voice_web_bridge node..."
echo "  - Flask server will run on http://127.0.0.1:5003"
echo "  - Subscribing to /scene_graph"
echo "  - Publishing to /llm_goal"
echo ""

# Run the node
install/voice_llm_navigator/lib/voice_llm_navigator/voice_web_bridge \
    --ros-args -p flask_host:=127.0.0.1 -p flask_port:=5003

echo ""
echo "[3/3] Voice web bridge stopped."

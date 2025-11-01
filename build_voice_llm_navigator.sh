#!/bin/bash
# Wrapper script to build voice_llm_navigator and fix ament hooks
# Usage: ./build_voice_llm_navigator.sh

cd "$(dirname "$0")"

echo "Building voice_llm_navigator..."
colcon build --packages-select voice_llm_navigator

if [ $? -eq 0 ]; then
    echo "Build successful. Installing ament hooks..."
    ./src/voice_llm_navigator/install_ament_hooks.sh ./install/voice_llm_navigator
    
    echo ""
    echo "✓ Build complete! You can now run:"
    echo "  . install/setup.bash"
    echo "  ros2 run voice_llm_navigator voice_navigation"
    echo ""
else
    echo "✗ Build failed!"
    exit 1
fi

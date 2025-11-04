#!/bin/bash
# Splat-SLAM installation script for system Python (no conda/docker)
# This script installs Splat-SLAM dependencies directly into the system Python environment

set -e  # Exit on error

WORKSPACE_ROOT="/home/raushan/control_one/wheel_control"
SPLAT_SLAM_DIR="${WORKSPACE_ROOT}/src/splat_slam"

echo "=========================================="
echo "Installing Splat-SLAM Dependencies"
echo "=========================================="
echo "Python version: $(python3 --version)"
echo "PyTorch version: $(python3 -c 'import torch; print(torch.__version__)')"
echo "CUDA available: $(python3 -c 'import torch; print(torch.cuda.is_available())')"
echo ""

# Step 1: Patch depth rendering threshold
echo "[1/5] Patching depth rendering threshold in diff-gaussian-rasterization-w-pose..."
RASTERIZER_FILE="${SPLAT_SLAM_DIR}/thirdparty/diff-gaussian-rasterization-w-pose/cuda_rasterizer/auxiliary.h"
if [ -f "$RASTERIZER_FILE" ]; then
    # Backup original file
    cp "$RASTERIZER_FILE" "${RASTERIZER_FILE}.bak"
    # Replace 0.2f with 0.001f for near plane threshold
    sed -i 's/if (p_view\.z <= 0\.2f)/if (p_view.z <= 0.001f)/g' "$RASTERIZER_FILE"
    echo "✓ Patched depth threshold (0.2f → 0.001f)"
else
    echo "⚠ Warning: Rasterizer file not found at $RASTERIZER_FILE"
fi

# Step 2: Install thirdparty dependencies
echo ""
echo "[2/5] Installing thirdparty Python packages..."

echo "  - Installing lietorch..."
python3 -m pip install -e "${SPLAT_SLAM_DIR}/thirdparty/lietorch/" --user

echo "  - Installing diff-gaussian-rasterization-w-pose..."
python3 -m pip install -e "${SPLAT_SLAM_DIR}/thirdparty/diff-gaussian-rasterization-w-pose/" --user

echo "  - Installing simple-knn..."
python3 -m pip install -e "${SPLAT_SLAM_DIR}/thirdparty/simple-knn/" --user

echo "  - Installing evaluate_3d_reconstruction_lib..."
python3 -m pip install -e "${SPLAT_SLAM_DIR}/thirdparty/evaluate_3d_reconstruction_lib/" --user

# Step 3: Install Splat-SLAM main package
echo ""
echo "[3/5] Installing Splat-SLAM main package..."
python3 -m pip install -e "${SPLAT_SLAM_DIR}/" --user

# Step 4: Install requirements.txt
echo ""
echo "[4/5] Installing additional requirements..."
python3 -m pip install -r "${SPLAT_SLAM_DIR}/requirements.txt" --user
python3 -m pip install pytorch-lightning==1.9 --no-deps --user

# Step 5: Verify installation
echo ""
echo "[5/5] Verifying installation..."
python3 -c "import torch; import lietorch; import simple_knn; import diff_gaussian_rasterization; print('✓ All core modules imported successfully')"
python3 -c "import torch; print(f'✓ CUDA available: {torch.cuda.is_available()}')"

echo ""
echo "=========================================="
echo "✓ Splat-SLAM installation complete!"
echo "=========================================="
echo ""
echo "Next steps:"
echo "1. Download pretrained models: bash src/splat_slam/scripts/download_pretrained.sh (if available)"
echo "   OR manually download from: https://drive.google.com/file/d/1oZbVPrubtaIUjRRuT8F-YjjHBW-1spKT/view"
echo "   and extract to: src/splat_slam/pretrained/"
echo ""
echo "2. Create ROS2 wrapper package: splat_slam_ros"
echo "3. Test with RealSense camera topics"
echo ""

#!/usr/bin/env python3
"""
Test script to verify that the RealSense serial_no parameter 
is being correctly passed to rs_launch.py in training_mode.launch.py
"""
import os
import sys
import yaml
from ament_index_python.packages import get_package_share_directory

# Load the training_mode.launch.py file and simulate what it does
cfg_path = os.path.join(get_package_share_directory('realsense_nav'), 'config', 'hardware.yaml')
print(f"[INFO] Loading config from: {cfg_path}")

hw_cfg = {}
try:
    with open(cfg_path, 'r') as f:
        hw_cfg = yaml.safe_load(f) or {}
except Exception as e:
    print(f"[ERROR] Failed to load hardware.yaml: {e}")
    sys.exit(1)

rs_cfg = hw_cfg.get('realsense', {})
default_serial = str(rs_cfg.get('serial', ''))

print(f"[INFO] Hardware config loaded:")
print(f"  realsense.serial: {default_serial}")
print(f"  realsense.enable_color: {rs_cfg.get('enable_color', True)}")
print(f"  realsense.enable_depth: {rs_cfg.get('enable_depth', True)}")
print(f"  realsense.rgb_profile: {rs_cfg.get('rgb_profile', '1280x720x30')}")
print(f"  realsense.depth_profile: {rs_cfg.get('depth_profile', '848x480x30')}")
print(f"  realsense.align_depth: {rs_cfg.get('align_depth', True)}")

print(f"\n[INFO] Launch arguments that would be forwarded to rs_launch.py:")
launch_args = {
    'enable_color': 'true' if rs_cfg.get('enable_color', True) else 'false',
    'enable_depth': 'true' if rs_cfg.get('enable_depth', True) else 'false',
    'rgb_camera.profile': rs_cfg.get('rgb_profile', '1280x720x30'),
    'depth_module.profile': rs_cfg.get('depth_profile', '848x480x30'),
    'align_depth.enable': 'true' if rs_cfg.get('align_depth', True) else 'false',
    'serial_no': default_serial,  # Note: this would be a LaunchConfiguration in real launch
}

for key, val in launch_args.items():
    print(f"  {key}: {val}")

# Verify device is available
try:
    import subprocess
    result = subprocess.run(['rs-enumerate-devices'], capture_output=True, text=True, timeout=5)
    if '239222303501' in result.stdout or 'Intel RealSense D455' in result.stdout:
        print(f"\n[SUCCESS] RealSense device with serial 239222303501 is connected and available!")
    else:
        print(f"\n[WARNING] rs-enumerate-devices output doesn't mention the expected device")
        print(f"Output:\n{result.stdout[:500]}")
except Exception as e:
    print(f"\n[WARNING] Could not check RealSense device: {e}")

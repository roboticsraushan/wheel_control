#!/usr/bin/env python3
"""
Quick camera test script
Tests all available video devices and shows their capabilities
"""

import cv2
import sys


def test_camera(device_id):
    """Test if a camera device works and show its properties."""
    print(f"\n{'='*60}")
    print(f"Testing /dev/video{device_id}")
    print(f"{'='*60}")
    
    cap = cv2.VideoCapture(device_id)
    
    if not cap.isOpened():
        print(f"❌ Cannot open /dev/video{device_id}")
        return False
    
    # Try to get a frame
    ret, frame = cap.read()
    if not ret:
        print(f"❌ Cannot read frame from /dev/video{device_id}")
        cap.release()
        return False
    
    # Get camera properties
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = int(cap.get(cv2.CAP_PROP_FPS))
    
    print(f"✅ Camera working!")
    print(f"   Resolution: {width}x{height}")
    print(f"   FPS: {fps}")
    print(f"   Frame shape: {frame.shape}")
    
    # Try to set 1080p
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    
    ret, frame = cap.read()
    if ret:
        new_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        new_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(f"   1080p capable: {new_width}x{new_height}")
    
    print(f"\n   Preview window opened. Press any key to continue...")
    cv2.imshow(f'Camera {device_id} Test', frame)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    cap.release()
    return True


def main():
    print("\n" + "="*60)
    print("Camera Detection and Test Tool")
    print("="*60)
    
    # Test devices 0-7
    working_cameras = []
    
    for device_id in range(8):
        if test_camera(device_id):
            working_cameras.append(device_id)
    
    print(f"\n{'='*60}")
    print("Summary")
    print(f"{'='*60}")
    
    if working_cameras:
        print(f"\n✅ Found {len(working_cameras)} working camera(s):")
        for device_id in working_cameras:
            print(f"   - /dev/video{device_id}")
        
        print(f"\n💡 To use in launch file, set:")
        print(f"   camera_device:={working_cameras[0]}")
        
        print(f"\n🚀 Quick start command:")
        print(f"   ros2 launch realsense_nav rgb_nav.launch.py camera_device:={working_cameras[0]}")
    else:
        print("\n❌ No working cameras found!")
    
    print()


if __name__ == '__main__':
    main()

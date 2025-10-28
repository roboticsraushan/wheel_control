#!/usr/bin/env python3
"""
Interactive HSV color tuner for USB RGB camera
Helps tune color thresholds for floor detection
"""

import cv2
import numpy as np


def main():
    # Open camera
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("Error: Could not open camera")
        return
    
    # Set resolution
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    
    # Get actual resolution
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Camera resolution: {width}x{height}")
    
    # Create window with trackbars
    cv2.namedWindow('RGB Camera Color Tuner')
    cv2.createTrackbar('H Min', 'RGB Camera Color Tuner', 0, 179, lambda x: None)
    cv2.createTrackbar('H Max', 'RGB Camera Color Tuner', 179, 179, lambda x: None)
    cv2.createTrackbar('S Min', 'RGB Camera Color Tuner', 0, 255, lambda x: None)
    cv2.createTrackbar('S Max', 'RGB Camera Color Tuner', 50, 255, lambda x: None)
    cv2.createTrackbar('V Min', 'RGB Camera Color Tuner', 100, 255, lambda x: None)
    cv2.createTrackbar('V Max', 'RGB Camera Color Tuner', 255, 255, lambda x: None)
    cv2.createTrackbar('Scale', 'RGB Camera Color Tuner', 50, 100, lambda x: None)
    
    print("\n=== RGB Camera Color Tuner ===")
    print("Adjust trackbars to tune HSV thresholds for your floor")
    print("Scale: Reduces image size for display (50 = 50%)")
    print("Press 'q' to quit and print final values")
    print("Press 's' to save current values\n")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to read frame")
            break
        
        # Get trackbar values
        h_min = cv2.getTrackbarPos('H Min', 'RGB Camera Color Tuner')
        h_max = cv2.getTrackbarPos('H Max', 'RGB Camera Color Tuner')
        s_min = cv2.getTrackbarPos('S Min', 'RGB Camera Color Tuner')
        s_max = cv2.getTrackbarPos('S Max', 'RGB Camera Color Tuner')
        v_min = cv2.getTrackbarPos('V Min', 'RGB Camera Color Tuner')
        v_max = cv2.getTrackbarPos('V Max', 'RGB Camera Color Tuner')
        scale = max(1, cv2.getTrackbarPos('Scale', 'RGB Camera Color Tuner')) / 100.0
        
        # Scale frame for display
        display_frame = cv2.resize(frame, (int(width * scale), int(height * scale)))
        
        # Convert to HSV
        hsv = cv2.cvtColor(display_frame, cv2.COLOR_BGR2HSV)
        
        # Create mask
        lower = np.array([h_min, s_min, v_min])
        upper = np.array([h_max, s_max, v_max])
        mask = cv2.inRange(hsv, lower, upper)
        
        # Apply morphology
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        # Find largest contour and area
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        area = 0
        if contours:
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)
        
        # Create result
        result = cv2.bitwise_and(display_frame, display_frame, mask=mask)
        
        # Stack images for display
        mask_3ch = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        
        # Resize to fit screen better
        display_h = 360
        display_w = int(display_h * width / height)
        
        frame_small = cv2.resize(display_frame, (display_w, display_h))
        mask_small = cv2.resize(mask_3ch, (display_w, display_h))
        result_small = cv2.resize(result, (display_w, display_h))
        
        combined = np.hstack([frame_small, mask_small, result_small])
        
        # Add text
        text1 = f"H:[{h_min},{h_max}] S:[{s_min},{s_max}] V:[{v_min},{v_max}]"
        text2 = f"Area: {area:.0f} pixels | Scale: {scale:.0%}"
        cv2.putText(combined, text1, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                   0.6, (0, 255, 0), 2)
        cv2.putText(combined, text2, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 
                   0.6, (0, 255, 0), 2)
        cv2.putText(combined, "Press 'q' to quit, 's' to save", (10, 90), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        
        # Show
        cv2.imshow('RGB Camera Color Tuner', combined)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            print("\n=== Final HSV Values ===")
            print(f"lower_bound = np.array([{h_min}, {s_min}, {v_min}])")
            print(f"upper_bound = np.array([{h_max}, {s_max}, {v_max}])")
            print("\nCopy these values to rgb_segmentation_node.py line ~118-119")
            break
        elif key == ord('s'):
            print(f"\n[SAVED] lower=[{h_min},{s_min},{v_min}] upper=[{h_max},{s_max},{v_max}] area={area:.0f}")
    
    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()

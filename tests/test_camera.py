#!/usr/bin/env python3

import cv2
import sys
import time

def test_camera():
    # Try to open the first USB camera (usually index 0)
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("Error: Could not open camera")
        return False
        
    # Wait a moment for camera to initialize
    time.sleep(2)
    
    # Try to grab a frame
    ret, frame = cap.read()
    
    # Release the camera
    cap.release()
    
    if not ret:
        print("Error: Could not grab frame from camera")
        return False
        
    print("Successfully grabbed frame from camera")
    print(f"Frame dimensions: {frame.shape}")
    
    # Save the frame to a PNG file
    cv2.imwrite("test_camera.png", frame)
    print("Saved frame to test_camera.png")
    
    return True

if __name__ == "__main__":
    success = test_camera()
    sys.exit(0 if success else 1)

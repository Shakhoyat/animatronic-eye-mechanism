#!/usr/bin/env python3
"""
Camera Test Script
Quick test to verify your webcam is working before full system setup.

Run this first to make sure your camera is accessible.
"""

import cv2
import sys

def test_camera():
    print("🔍 Testing camera access...")
    
    # Try different camera indices
    for camera_index in range(3):
        print(f"Trying camera index {camera_index}...")
        
        cap = cv2.VideoCapture(camera_index)
        
        if cap.isOpened():
            print(f"✅ Camera {camera_index} opened successfully!")
            
            # Test reading a frame
            ret, frame = cap.read()
            if ret:
                print(f"✅ Camera {camera_index} can read frames")
                print(f"Frame size: {frame.shape[1]}x{frame.shape[0]}")
                
                # Show camera feed for 5 seconds
                print("📷 Showing camera feed for 5 seconds...")
                print("Press 'q' to exit early")
                
                for i in range(150):  # ~5 seconds at 30fps
                    ret, frame = cap.read()
                    if ret:
                        cv2.imshow(f'Camera Test - Index {camera_index}', frame)
                        key = cv2.waitKey(1) & 0xFF
                        if key == ord('q'):
                            break
                    else:
                        print("❌ Failed to read frame")
                        break
                
                cv2.destroyAllWindows()
                cap.release()
                
                print(f"✅ Camera {camera_index} test completed successfully!")
                return camera_index
            
            else:
                print(f"❌ Camera {camera_index} opened but can't read frames")
                cap.release()
        
        else:
            print(f"❌ Could not open camera {camera_index}")
    
    print("❌ No working camera found!")
    return None

def main():
    print("📹 Webcam Test for Animatronic Eye Tracker")
    print("=" * 50)
    
    working_camera = test_camera()
    
    if working_camera is not None:
        print(f"\n✅ SUCCESS: Use camera index {working_camera} in your eye tracker")
        print("You can now proceed with the ESP32 setup!")
    else:
        print("\n❌ CAMERA TEST FAILED")
        print("Troubleshooting steps:")
        print("1. Check if camera is connected properly")
        print("2. Close any other applications using the camera")
        print("3. Check Windows camera privacy settings")
        print("4. Try a different USB port")
        print("5. Restart your computer")
    
    print("\nPress Enter to exit...")
    input()

if __name__ == "__main__":
    main()
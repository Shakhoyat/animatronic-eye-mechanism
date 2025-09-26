#!/usr/bin/env python3
"""
Animatronic Eye Tracking System
Python Client for PC Webcam Integration

SAFETY FEATURES:
- Connection monitoring with automatic reconnection
- Smooth movement filtering to prevent servo jerking
- Emergency stop capability
- Adjustable sensitivity and dead zones

Hardware Requirements:
- PC with webcam
- ESP32 with servo controller (see Arduino code)
- Stable WiFi network connection

Author: Eye Tracking System
Version: 1.0 - Safety First Edition
"""

import cv2
import numpy as np
import socket
import threading
import json
import time
import queue
from collections import deque
import sys

class EyeTracker:
    def __init__(self, esp32_ip="192.168.1.100", esp32_port=8888):
        # Network Configuration - UPDATE THIS IP!
        self.esp32_ip = esp32_ip
        self.esp32_port = esp32_port
        self.socket = None
        self.connected = False
        
        # Camera Configuration
        self.camera_index = 0
        self.cap = None
        
        # Eye Detection Configuration
        self.face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
        self.eye_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_eye.xml')
        
        # Calibration and Tracking
        self.center_x = None
        self.center_y = None
        self.calibrated = False
        
        # Movement Parameters (ADJUST FOR YOUR PREFERENCE)
        self.dead_zone = 30          # Pixels - larger = less sensitive
        self.smoothing_window = 5    # Frames - larger = smoother movement
        self.max_gaze_range = 100    # Pixels - maximum gaze detection range
        self.sensitivity = 0.8       # 0.1-1.0 - movement sensitivity multiplier
        
        # Smoothing buffers
        self.gaze_x_buffer = deque(maxlen=self.smoothing_window)
        self.gaze_y_buffer = deque(maxlen=self.smoothing_window)
        
        # Blink Detection
        self.blink_threshold = 0.25  # Ratio threshold for blink detection
        self.blink_frames = 0
        self.blink_cooldown = 0
        
        # Communication
        self.command_queue = queue.Queue()
        self.command_interval = 0.1  # Seconds between commands
        self.last_command_time = 0
        
        # Status tracking
        self.frame_count = 0
        self.fps_counter = 0
        self.last_fps_time = time.time()
        self.current_fps = 0
        
        print("🎭 Animatronic Eye Tracker Initialized")
        print(f"ESP32 Target: {self.esp32_ip}:{self.esp32_port}")
        
    def initialize_camera(self):
        """Initialize camera with error handling"""
        print("📷 Initializing camera...")
        
        # Try different camera indices
        for cam_index in range(3):
            self.cap = cv2.VideoCapture(cam_index)
            if self.cap.isOpened():
                self.camera_index = cam_index
                print(f"✅ Camera {cam_index} initialized successfully")
                
                # Set camera properties for better performance
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                self.cap.set(cv2.CAP_PROP_FPS, 30)
                return True
            else:
                self.cap.release()
        
        print("❌ No camera found! Check camera connection and permissions.")
        return False
    
    def connect_to_esp32(self):
        """Connect to ESP32 with retry logic"""
        if self.connected:
            return True
            
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(5.0)
            self.socket.connect((self.esp32_ip, self.esp32_port))
            self.connected = True
            print(f"✅ Connected to ESP32 at {self.esp32_ip}:{self.esp32_port}")
            return True
            
        except Exception as e:
            print(f"❌ ESP32 connection failed: {e}")
            if self.socket:
                self.socket.close()
                self.socket = None
            self.connected = False
            return False
    
    def send_command(self, command_data):
        """Send command to ESP32 with error handling"""
        if not self.connected:
            if not self.connect_to_esp32():
                return False
        
        try:
            command_json = json.dumps(command_data) + '\n'
            self.socket.send(command_json.encode())
            return True
            
        except Exception as e:
            print(f"❌ Send command failed: {e}")
            self.connected = False
            if self.socket:
                self.socket.close()
                self.socket = None
            return False
    
    def calibrate_center(self, eye_center_x, eye_center_y):
        """Calibrate center position for gaze tracking"""
        self.center_x = eye_center_x
        self.center_y = eye_center_y
        self.calibrated = True
        
        # Send reset command to ESP32
        reset_command = {
            "type": "reset_center"
        }
        self.send_command(reset_command)
        
        print(f"🎯 Center calibrated at ({self.center_x}, {self.center_y})")
        print("✅ Calibration complete! Eye tracking is now active.")
    
    def detect_blink(self, eye_aspect_ratio):
        """Detect blink based on eye aspect ratio"""
        if eye_aspect_ratio < self.blink_threshold:
            self.blink_frames += 1
        else:
            if self.blink_frames >= 2 and self.blink_cooldown <= 0:
                self.blink_cooldown = 10  # Prevent rapid blink detection
                self.blink_frames = 0
                return True
            self.blink_frames = 0
            
        if self.blink_cooldown > 0:
            self.blink_cooldown -= 1
            
        return False
    
    def calculate_eye_aspect_ratio(self, eye_points):
        """Calculate eye aspect ratio for blink detection"""
        if len(eye_points) < 6:
            return 1.0
            
        # Convert to numpy array if needed
        if not isinstance(eye_points, np.ndarray):
            eye_points = np.array(eye_points)
        
        # Calculate distances
        A = np.linalg.norm(eye_points[1] - eye_points[5])
        B = np.linalg.norm(eye_points[2] - eye_points[4])
        C = np.linalg.norm(eye_points[0] - eye_points[3])
        
        # Eye aspect ratio
        ear = (A + B) / (2.0 * C)
        return ear
    
    def smooth_gaze_data(self, raw_x, raw_y):
        """Apply smoothing to gaze data"""
        self.gaze_x_buffer.append(raw_x)
        self.gaze_y_buffer.append(raw_y)
        
        # Calculate moving average
        smooth_x = sum(self.gaze_x_buffer) / len(self.gaze_x_buffer)
        smooth_y = sum(self.gaze_y_buffer) / len(self.gaze_y_buffer)
        
        return smooth_x, smooth_y
    
    def process_eye_tracking(self, frame):
        """Main eye tracking processing function"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(gray, 1.3, 5)
        
        blink_detected = False
        gaze_x, gaze_y = 0.0, 0.0
        
        for (x, y, w, h) in faces:
            # Draw face rectangle
            cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)
            
            # Region of interest for eyes
            roi_gray = gray[y:y+h, x:x+w]
            roi_color = frame[y:y+h, x:x+w]
            
            # Detect eyes
            eyes = self.eye_cascade.detectMultiScale(roi_gray)
            
            if len(eyes) >= 2:
                # Sort eyes by x-coordinate (left to right)
                eyes = sorted(eyes, key=lambda e: e[0])
                
                eye_centers = []
                for (ex, ey, ew, eh) in eyes[:2]:  # Process only first 2 eyes
                    # Draw eye rectangle
                    cv2.rectangle(roi_color, (ex, ey), (ex+ew, ey+eh), (0, 255, 0), 2)
                    
                    # Find eye center (simplified pupil detection)
                    eye_center_x = x + ex + ew // 2
                    eye_center_y = y + ey + eh // 2
                    eye_centers.append((eye_center_x, eye_center_y))
                    
                    # Draw eye center
                    cv2.circle(frame, (eye_center_x, eye_center_y), 3, (0, 0, 255), -1)
                    
                    # Basic blink detection using eye height ratio
                    eye_ratio = eh / ew if ew > 0 else 1.0
                    if eye_ratio < self.blink_threshold:
                        blink_detected = True
                
                # Calculate average eye position for gaze tracking
                if len(eye_centers) >= 2:
                    avg_eye_x = sum(center[0] for center in eye_centers) / len(eye_centers)
                    avg_eye_y = sum(center[1] for center in eye_centers) / len(eye_centers)
                    
                    # Calibration mode
                    if not self.calibrated:
                        cv2.putText(frame, "Look at camera and press 'C' to calibrate", 
                                  (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                        self.current_eye_pos = (avg_eye_x, avg_eye_y)
                    else:
                        # Calculate gaze relative to center
                        delta_x = avg_eye_x - self.center_x
                        delta_y = avg_eye_y - self.center_y
                        
                        # Apply dead zone
                        if abs(delta_x) > self.dead_zone:
                            gaze_x = delta_x / self.max_gaze_range
                        if abs(delta_y) > self.dead_zone:
                            gaze_y = delta_y / self.max_gaze_range
                        
                        # Apply sensitivity and constraints
                        gaze_x = np.clip(gaze_x * self.sensitivity, -1.0, 1.0)
                        gaze_y = np.clip(gaze_y * self.sensitivity, -1.0, 1.0)
                        
                        # Apply smoothing
                        gaze_x, gaze_y = self.smooth_gaze_data(gaze_x, gaze_y)
                        
                        # Draw center point and dead zone
                        cv2.circle(frame, (int(self.center_x), int(self.center_y)), 5, (255, 255, 255), 2)
                        cv2.rectangle(frame, 
                                    (int(self.center_x - self.dead_zone), int(self.center_y - self.dead_zone)),
                                    (int(self.center_x + self.dead_zone), int(self.center_y + self.dead_zone)),
                                    (128, 128, 128), 1)
        
        return gaze_x, gaze_y, blink_detected
    
    def send_eye_movement(self, gaze_x, gaze_y, blink):
        """Send eye movement command to ESP32"""
        current_time = time.time()
        
        # Throttle command sending
        if current_time - self.last_command_time < self.command_interval:
            return
        
        self.last_command_time = current_time
        
        command = {
            "type": "eye_movement",
            "x": float(gaze_x),
            "y": float(gaze_y),
            "blink": bool(blink)
        }
        
        self.send_command(command)
    
    def draw_ui(self, frame, gaze_x, gaze_y, blink):
        """Draw user interface elements"""
        # Status information
        status_y = frame.shape[0] - 120
        
        # Connection status
        connection_status = "🟢 Connected" if self.connected else "🔴 Disconnected"
        cv2.putText(frame, f"ESP32: {connection_status}", (10, status_y), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0) if self.connected else (0, 0, 255), 2)
        
        # Gaze information
        cv2.putText(frame, f"Gaze X: {gaze_x:.2f}", (10, status_y + 25), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(frame, f"Gaze Y: {gaze_y:.2f}", (10, status_y + 50), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Blink indicator
        if blink:
            cv2.putText(frame, "BLINK!", (10, status_y + 75), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        
        # Calibration status
        if not self.calibrated:
            cv2.putText(frame, "⚠️  NOT CALIBRATED", (200, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        
        # FPS counter
        cv2.putText(frame, f"FPS: {self.current_fps:.1f}", (frame.shape[1] - 100, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Instructions
        cv2.putText(frame, "Controls: 'C' = Calibrate | 'Q' = Quit", (10, frame.shape[0] - 10), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
    
    def update_fps(self):
        """Update FPS counter"""
        self.fps_counter += 1
        current_time = time.time()
        
        if current_time - self.last_fps_time >= 1.0:
            self.current_fps = self.fps_counter / (current_time - self.last_fps_time)
            self.fps_counter = 0
            self.last_fps_time = current_time
    
    def run(self):
        """Main application loop"""
        print("🚀 Starting Eye Tracker...")
        
        # Initialize camera
        if not self.initialize_camera():
            return False
        
        # Try initial ESP32 connection
        self.connect_to_esp32()
        
        print("\n📋 INSTRUCTIONS:")
        print("1. Position yourself 2-3 feet from the camera")
        print("2. Look directly at the camera")
        print("3. Press 'C' to calibrate center position")
        print("4. Your eye movements will now control the servos")
        print("5. Press 'Q' to quit")
        print("\n🎬 Starting camera feed...")
        
        try:
            while True:
                ret, frame = self.cap.read()
                if not ret:
                    print("❌ Failed to read from camera")
                    break
                
                # Flip frame horizontally for mirror effect
                frame = cv2.flip(frame, 1)
                
                # Process eye tracking
                gaze_x, gaze_y, blink = self.process_eye_tracking(frame)
                
                # Send commands to ESP32
                if self.calibrated and (abs(gaze_x) > 0.01 or abs(gaze_y) > 0.01 or blink):
                    self.send_eye_movement(gaze_x, gaze_y, blink)
                
                # Draw UI
                self.draw_ui(frame, gaze_x, gaze_y, blink)
                
                # Update FPS
                self.update_fps()
                
                # Display frame
                cv2.imshow('Animatronic Eye Tracker', frame)
                
                # Handle keyboard input
                key = cv2.waitKey(1) & 0xFF
                
                if key == ord('q'):
                    print("👋 Quitting...")
                    break
                elif key == ord('c'):
                    if hasattr(self, 'current_eye_pos'):
                        self.calibrate_center(self.current_eye_pos[0], self.current_eye_pos[1])
                    else:
                        print("⚠️  No eye detected for calibration")
                elif key == ord(' '):  # Spacebar for emergency stop
                    print("🚨 Emergency stop sent!")
                    self.send_command({"type": "emergency_stop"})
        
        except KeyboardInterrupt:
            print("\n🛑 Interrupted by user")
        
        except Exception as e:
            print(f"❌ Error in main loop: {e}")
        
        finally:
            # Cleanup
            if self.cap:
                self.cap.release()
            if self.socket:
                self.socket.close()
            cv2.destroyAllWindows()
            print("✅ Cleanup complete")
        
        return True

def main():
    print("🎭 Animatronic Eye Tracking System")
    print("=" * 50)
    
    # Configuration
    ESP32_IP = "192.168.1.100"  # ⚠️  UPDATE THIS TO YOUR ESP32's IP ADDRESS!
    ESP32_PORT = 8888
    
    print(f"Target ESP32: {ESP32_IP}:{ESP32_PORT}")
    print("⚠️  Make sure to update ESP32_IP to match your ESP32's actual IP address!")
    print()
    
    # Create and run eye tracker
    tracker = EyeTracker(ESP32_IP, ESP32_PORT)
    
    try:
        success = tracker.run()
        if success:
            print("✅ Eye tracking completed successfully")
        else:
            print("❌ Eye tracking failed to start")
            return 1
            
    except Exception as e:
        print(f"❌ Fatal error: {e}")
        return 1
    
    return 0

if __name__ == "__main__":
    sys.exit(main())
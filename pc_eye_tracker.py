"""
PC Eye Tracker for Animatronic Eye Mechanism
Uses OpenCV and MediaPipe to track eye movements via webcam
Sends eye position data to ESP32 via serial communication

Requirements:
pip install opencv-python mediapipe pyserial numpy
"""

import cv2
import mediapipe as mp
import serial
import serial.tools.list_ports
import time
import numpy as np
import sys

class EyeTracker:
    def __init__(self, serial_port='COM3', baud_rate=115200):
        """
        Initialize the eye tracker with webcam and serial connection
        
        Args:
            serial_port: COM port for ESP32 (e.g., 'COM3' on Windows, '/dev/ttyUSB0' on Linux)
            baud_rate: Serial communication speed (default 115200)
        """
        # Initialize MediaPipe Face Mesh
        self.mp_face_mesh = mp.solutions.face_mesh
        self.face_mesh = self.mp_face_mesh.FaceMesh(
            max_num_faces=1,
            refine_landmarks=True,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        self.mp_drawing = mp.solutions.drawing_utils
        self.drawing_spec = self.mp_drawing.DrawingSpec(thickness=1, circle_radius=1)
        
        # Initialize webcam
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            print("Error: Could not open webcam")
            sys.exit(1)
            
        # Set webcam resolution
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        # Initialize serial connection
        try:
            self.serial = serial.Serial(serial_port, baud_rate, timeout=1)
            time.sleep(2)  # Wait for serial connection to establish
            print(f"Connected to ESP32 on {serial_port}")
        except serial.SerialException as e:
            print(f"Error: Could not open serial port {serial_port}")
            print(f"Details: {e}")
            print("\nAvailable ports:")
            for port in serial.tools.list_ports.comports():
                print(f"  - {port.device}: {port.description}")
            sys.exit(1)
        
        # Calibration values (will be adjusted during calibration)
        self.calibration = {
            'center_x': 0.5,
            'center_y': 0.5,
            'range_x': 0.15,  # Horizontal range
            'range_y': 0.12,  # Vertical range
            'smoothing': 0.3   # Smoothing factor (lower = more smoothing)
        }
        
        # Previous positions for smoothing
        self.prev_lr = 90
        self.prev_ud = 90
        
        # Blink detection
        self.blink_threshold = 0.2  # Adjust based on your eye
        self.prev_blink = False
        
        # FPS calculation
        self.prev_time = time.time()
        
        # Calibration mode
        self.calibration_mode = False
        self.calibration_points = []
        
    def get_eye_aspect_ratio(self, landmarks, eye_indices):
        """
        Calculate Eye Aspect Ratio (EAR) for blink detection
        """
        # Vertical eye landmarks
        v1 = np.linalg.norm(landmarks[eye_indices[1]] - landmarks[eye_indices[5]])
        v2 = np.linalg.norm(landmarks[eye_indices[2]] - landmarks[eye_indices[4]])
        # Horizontal eye landmark
        h = np.linalg.norm(landmarks[eye_indices[0]] - landmarks[eye_indices[3]])
        
        ear = (v1 + v2) / (2.0 * h)
        return ear
    
    def get_gaze_position(self, face_landmarks, frame_width, frame_height):
        """
        Calculate gaze position from facial landmarks
        Returns normalized x, y position (0-1 range)
        """
        # Get left and right iris center landmarks (468-473 indices)
        left_iris = [469, 470, 471, 472]  # Left iris landmarks
        right_iris = [474, 475, 476, 477]  # Right iris landmarks
        
        # Extract iris positions
        left_iris_coords = []
        right_iris_coords = []
        
        for idx in left_iris:
            landmark = face_landmarks.landmark[idx]
            left_iris_coords.append([landmark.x, landmark.y])
            
        for idx in right_iris:
            landmark = face_landmarks.landmark[idx]
            right_iris_coords.append([landmark.x, landmark.y])
        
        # Calculate center of each iris
        left_center = np.mean(left_iris_coords, axis=0)
        right_center = np.mean(right_iris_coords, axis=0)
        
        # Average both eyes for gaze direction
        gaze_x = (left_center[0] + right_center[0]) / 2
        gaze_y = (left_center[1] + right_center[1]) / 2
        
        return gaze_x, gaze_y
    
    def detect_blink(self, face_landmarks):
        """
        Detect if eyes are blinking using Eye Aspect Ratio
        """
        # Left eye landmarks
        left_eye = [362, 385, 387, 263, 373, 380]
        # Right eye landmarks
        right_eye = [33, 160, 158, 133, 153, 144]
        
        # Extract landmark coordinates
        landmarks = []
        for lm in face_landmarks.landmark:
            landmarks.append(np.array([lm.x, lm.y]))
        landmarks = np.array(landmarks)
        
        # Calculate EAR for both eyes
        left_ear = self.get_eye_aspect_ratio(landmarks, left_eye)
        right_ear = self.get_eye_aspect_ratio(landmarks, right_eye)
        
        # Average EAR
        ear = (left_ear + right_ear) / 2.0
        
        # Check if blinking
        is_blinking = ear < self.blink_threshold
        
        return is_blinking, ear
    
    def map_to_servo_angle(self, gaze_x, gaze_y):
        """
        Map gaze position to servo angles (0-180 degrees)
        With calibration adjustments
        """
        # Normalize around calibrated center
        norm_x = (gaze_x - self.calibration['center_x']) / self.calibration['range_x']
        norm_y = (gaze_y - self.calibration['center_y']) / self.calibration['range_y']
        
        # Clamp to -1 to 1 range
        norm_x = max(-1, min(1, norm_x))
        norm_y = max(-1, min(1, norm_y))
        
        # Map to servo angles (center at 90 degrees)
        # LR: left gaze = higher angle, right gaze = lower angle
        lr_angle = 90 - (norm_x * 30)  # ±30 degrees from center
        
        # UD: up gaze = higher angle, down gaze = lower angle
        ud_angle = 90 + (norm_y * 30)  # ±30 degrees from center
        
        # Apply smoothing
        lr_angle = self.prev_lr + (lr_angle - self.prev_lr) * self.calibration['smoothing']
        ud_angle = self.prev_ud + (ud_angle - self.prev_ud) * self.calibration['smoothing']
        
        # Clamp to valid servo range
        lr_angle = max(60, min(120, lr_angle))
        ud_angle = max(60, min(120, ud_angle))
        
        # Update previous values
        self.prev_lr = lr_angle
        self.prev_ud = ud_angle
        
        return int(lr_angle), int(ud_angle)
    
    def send_to_esp32(self, lr_angle, ud_angle, blink):
        """
        Send eye position and blink data to ESP32 via serial
        Format: "ELR:90,EUD:90,TLR:90,TUD:90,HR:90,BL:0\n"
        
        For 7 servos:
        - ELR/EUD: Eyeball tracking (fast response)
        - TLR/TUD: Face tilt follows eyeball with offset (natural head movement)
        - HR: Head rotation follows horizontal gaze
        """
        blink_val = 1 if blink else 0
        
        # Calculate head tilt based on eyeball position (follows with less range)
        # Maps eyeball 60-120° to tilt 70-110° (less extreme)
        tilt_lr = int(90 + (lr_angle - 90) * 0.3)  # 30% of eyeball movement
        tilt_ud = int(90 + (ud_angle - 90) * 0.3)  # 30% of eyeball movement
        
        # Calculate head rotation based on horizontal gaze
        # Large left/right gaze triggers head turn
        head_offset = (lr_angle - 90) * 0.5  # 50% of eyeball movement
        head_rotate = int(90 + head_offset)
        
        # Clamp values
        tilt_lr = max(70, min(110, tilt_lr))
        tilt_ud = max(70, min(110, tilt_ud))
        head_rotate = max(45, min(135, head_rotate))
        
        command = f"ELR:{lr_angle},EUD:{ud_angle},TLR:{tilt_lr},TUD:{tilt_ud},HR:{head_rotate},BL:{blink_val}\n"
        try:
            self.serial.write(command.encode())
        except serial.SerialException as e:
            print(f"Serial write error: {e}")
    
    def calibrate(self):
        """
        Calibration mode: Press C to capture current gaze position as center
        """
        print("\n=== CALIBRATION MODE ===")
        print("Look at the CENTER of the screen and press 'C' to set center point")
        print("Look FAR LEFT and press 'L'")
        print("Look FAR RIGHT and press 'R'")
        print("Look FAR UP and press 'U'")
        print("Look FAR DOWN and press 'D'")
        print("Press 'S' to save and exit calibration")
        self.calibration_mode = True
        self.calibration_points = []
    
    def run(self):
        """
        Main loop: capture video, detect eyes, send data to ESP32
        """
        print("\n=== Eye Tracker Started (7 Servo Mode) ===")
        print("Controls:")
        print("  Q - Quit")
        print("  C - Enter calibration mode")
        print("  B - Adjust blink threshold")
        print("\n7 Servos:")
        print("  - Eyeball LR/UD (tracks eyes fast)")
        print("  - Eyelids Left/Right (blink control)")
        print("  - Face Tilt LR/UD (follows gaze naturally)")
        print("  - Head Rotate (follows horizontal gaze)")
        print("\n")
        
        while True:
            success, frame = self.cap.read()
            if not success:
                print("Error: Failed to capture frame")
                break
            
            # Flip frame horizontally for mirror effect
            frame = cv2.flip(frame, 1)
            
            # Convert to RGB for MediaPipe
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            # Process the frame
            results = self.face_mesh.process(rgb_frame)
            
            # Calculate FPS
            curr_time = time.time()
            fps = 1 / (curr_time - self.prev_time)
            self.prev_time = curr_time
            
            if results.multi_face_landmarks:
                face_landmarks = results.multi_face_landmarks[0]
                
                # Draw face mesh (optional, can be disabled for performance)
                # self.mp_drawing.draw_landmarks(
                #     frame, face_landmarks, self.mp_face_mesh.FACEMESH_CONTOURS,
                #     self.drawing_spec, self.drawing_spec)
                
                # Get gaze position
                gaze_x, gaze_y = self.get_gaze_position(face_landmarks, frame.shape[1], frame.shape[0])
                
                # Detect blink
                is_blinking, ear = self.detect_blink(face_landmarks)
                
                # Map to servo angles
                lr_angle, ud_angle = self.map_to_servo_angle(gaze_x, gaze_y)
                
                # Calculate tilt and head rotation (same logic as send_to_esp32)
                tilt_lr = int(90 + (lr_angle - 90) * 0.3)
                tilt_ud = int(90 + (ud_angle - 90) * 0.3)
                head_offset = (lr_angle - 90) * 0.5
                head_rotate = int(90 + head_offset)
                
                # Clamp values
                tilt_lr = max(70, min(110, tilt_lr))
                tilt_ud = max(70, min(110, tilt_ud))
                head_rotate = max(45, min(135, head_rotate))
                
                # Send to ESP32
                self.send_to_esp32(lr_angle, ud_angle, is_blinking)
                
                # Display only the 7 servo values in degrees
                cv2.putText(frame, f"Eyeball LR: {lr_angle}deg", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                cv2.putText(frame, f"Eyeball UD: {ud_angle}deg", (10, 60), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                cv2.putText(frame, f"Tilt LR: {tilt_lr}deg", (10, 90), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                cv2.putText(frame, f"Tilt UD: {tilt_ud}deg", (10, 120), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                cv2.putText(frame, f"Head Rotate: {head_rotate}deg", (10, 150), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 128, 0), 2)
                cv2.putText(frame, f"Blink L: {1 if is_blinking else 0}", (10, 180), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255) if is_blinking else (0, 255, 0), 2)
                cv2.putText(frame, f"Blink R: {1 if is_blinking else 0}", (10, 210), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255) if is_blinking else (0, 255, 0), 2)
                
                # Calibration mode display
                if self.calibration_mode:
                    cv2.putText(frame, "CALIBRATION MODE", (10, frame.shape[0] - 20), 
                               cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            else:
                cv2.putText(frame, "No face detected", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # Display frame
            cv2.imshow('Eye Tracker - Animatronic Eyes', frame)
            
            # Handle keyboard input
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q') or key == ord('Q'):
                break
            elif key == ord('c') or key == ord('C'):
                if not self.calibration_mode:
                    self.calibrate()
                elif results.multi_face_landmarks:
                    gaze_x, gaze_y = self.get_gaze_position(
                        results.multi_face_landmarks[0], 
                        frame.shape[1], frame.shape[0]
                    )
                    self.calibration['center_x'] = gaze_x
                    self.calibration['center_y'] = gaze_y
                    print(f"Center set to: X={gaze_x:.3f}, Y={gaze_y:.3f}")
            elif key == ord('l') or key == ord('L'):
                if self.calibration_mode and results.multi_face_landmarks:
                    gaze_x, _ = self.get_gaze_position(
                        results.multi_face_landmarks[0], 
                        frame.shape[1], frame.shape[0]
                    )
                    left_range = abs(gaze_x - self.calibration['center_x'])
                    print(f"Left range: {left_range:.3f}")
                    self.calibration_points.append(('left', left_range))
            elif key == ord('r') or key == ord('R'):
                if self.calibration_mode and results.multi_face_landmarks:
                    gaze_x, _ = self.get_gaze_position(
                        results.multi_face_landmarks[0], 
                        frame.shape[1], frame.shape[0]
                    )
                    right_range = abs(gaze_x - self.calibration['center_x'])
                    print(f"Right range: {right_range:.3f}")
                    self.calibration_points.append(('right', right_range))
            elif key == ord('u') or key == ord('U'):
                if self.calibration_mode and results.multi_face_landmarks:
                    _, gaze_y = self.get_gaze_position(
                        results.multi_face_landmarks[0], 
                        frame.shape[1], frame.shape[0]
                    )
                    up_range = abs(gaze_y - self.calibration['center_y'])
                    print(f"Up range: {up_range:.3f}")
                    self.calibration_points.append(('up', up_range))
            elif key == ord('d') or key == ord('D'):
                if self.calibration_mode and results.multi_face_landmarks:
                    _, gaze_y = self.get_gaze_position(
                        results.multi_face_landmarks[0], 
                        frame.shape[1], frame.shape[0]
                    )
                    down_range = abs(gaze_y - self.calibration['center_y'])
                    print(f"Down range: {down_range:.3f}")
                    self.calibration_points.append(('down', down_range))
            elif key == ord('s') or key == ord('S'):
                if self.calibration_mode:
                    # Calculate average ranges
                    x_ranges = [p[1] for p in self.calibration_points if p[0] in ['left', 'right']]
                    y_ranges = [p[1] for p in self.calibration_points if p[0] in ['up', 'down']]
                    
                    if x_ranges:
                        self.calibration['range_x'] = sum(x_ranges) / len(x_ranges)
                    if y_ranges:
                        self.calibration['range_y'] = sum(y_ranges) / len(y_ranges)
                    
                    print("\n=== Calibration Saved ===")
                    print(f"Center: X={self.calibration['center_x']:.3f}, Y={self.calibration['center_y']:.3f}")
                    print(f"Range: X={self.calibration['range_x']:.3f}, Y={self.calibration['range_y']:.3f}")
                    
                    self.calibration_mode = False
                    self.calibration_points = []
            elif key == ord('b') or key == ord('B'):
                print(f"Current blink threshold: {self.blink_threshold}")
                try:
                    new_threshold = float(input("Enter new blink threshold (0.15-0.25): "))
                    self.blink_threshold = max(0.15, min(0.25, new_threshold))
                    print(f"Blink threshold set to: {self.blink_threshold}")
                except ValueError:
                    print("Invalid input. Keeping current threshold.")
        
        # Cleanup
        self.cap.release()
        cv2.destroyAllWindows()
        self.serial.close()
        print("\nEye Tracker stopped")

def main():
    """
    Main function to start the eye tracker
    """
    # List available serial ports
    print("Available serial ports:")
    ports = list(serial.tools.list_ports.comports())
    for i, port in enumerate(ports):
        print(f"{i+1}. {port.device}: {port.description}")
    
    if not ports:
        print("No serial ports found!")
        return
    
    # Select port
    if len(sys.argv) > 1:
        serial_port = sys.argv[1]
    else:
        try:
            port_num = int(input(f"\nSelect port (1-{len(ports)}): "))
            serial_port = ports[port_num - 1].device
        except (ValueError, IndexError):
            print("Invalid selection. Using first available port.")
            serial_port = ports[0].device
    
    print(f"\nUsing serial port: {serial_port}")
    
    # Create and run eye tracker
    tracker = EyeTracker(serial_port=serial_port, baud_rate=115200)
    tracker.run()

if __name__ == "__main__":
    main()

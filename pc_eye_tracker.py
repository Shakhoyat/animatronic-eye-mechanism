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
        Calculate gaze position based on face orientation (mirrors the face)
        Returns normalized gaze direction and head orientation, independent of face position in frame
        """
        # Eye landmarks
        left_eye_left = face_landmarks.landmark[33]    # Left corner of left eye
        left_eye_right = face_landmarks.landmark[133]  # Right corner of left eye
        left_eye_top = face_landmarks.landmark[159]    # Top of left eye
        left_eye_bottom = face_landmarks.landmark[145] # Bottom of left eye
        
        right_eye_left = face_landmarks.landmark[362]  # Left corner of right eye
        right_eye_right = face_landmarks.landmark[263] # Right corner of right eye
        right_eye_top = face_landmarks.landmark[386]   # Top of right eye
        right_eye_bottom = face_landmarks.landmark[374] # Bottom of right eye
        
        # Iris landmarks
        left_iris = [469, 470, 471, 472]
        right_iris = [474, 475, 476, 477]
        
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
        left_iris_center = np.mean(left_iris_coords, axis=0)
        right_iris_center = np.mean(right_iris_coords, axis=0)
        
        # Calculate iris position WITHIN the eye (horizontal: 0=left, 0.5=center, 1=right)
        left_eye_width = abs(left_eye_right.x - left_eye_left.x)
        right_eye_width = abs(right_eye_right.x - right_eye_left.x)
        
        left_iris_x_ratio = (left_iris_center[0] - left_eye_left.x) / left_eye_width if left_eye_width > 0 else 0.5
        right_iris_x_ratio = (right_iris_center[0] - right_eye_left.x) / right_eye_width if right_eye_width > 0 else 0.5
        
        # Calculate iris position WITHIN the eye (vertical: 0=top, 0.5=center, 1=bottom)
        left_eye_height = abs(left_eye_bottom.y - left_eye_top.y)
        right_eye_height = abs(right_eye_bottom.y - right_eye_top.y)
        
        left_iris_y_ratio = (left_iris_center[1] - left_eye_top.y) / left_eye_height if left_eye_height > 0 else 0.5
        right_iris_y_ratio = (right_iris_center[1] - right_eye_top.y) / right_eye_height if right_eye_height > 0 else 0.5
        
        # Average both eyes for gaze direction (0.5 = center, independent of face position)
        gaze_x = (left_iris_x_ratio + right_iris_x_ratio) / 2
        gaze_y = (left_iris_y_ratio + right_iris_y_ratio) / 2
        
        # Calculate head orientation (tilt) based on eye line angle
        # If face tilts right, right eye goes up relative to left eye
        left_eye_center_y = (left_eye_top.y + left_eye_bottom.y) / 2
        right_eye_center_y = (right_eye_top.y + right_eye_bottom.y) / 2
        head_tilt_lr = right_eye_center_y - left_eye_center_y  # Positive = tilted right
        
        # Calculate head pitch (up/down tilt) based on nose-to-eye distance
        nose_tip = face_landmarks.landmark[1]
        forehead = face_landmarks.landmark[10]
        chin = face_landmarks.landmark[152]
        
        # Ratio of nose position between forehead and chin (0.5 = neutral)
        face_height = abs(chin.y - forehead.y)
        nose_ratio = (nose_tip.y - forehead.y) / face_height if face_height > 0 else 0.5
        
        # Get face center for visualization
        face_center = [nose_tip.x, nose_tip.y]
        
        # Store points for visualization
        viz_points = {
            'left_iris': left_iris_center,
            'right_iris': right_iris_center,
            'face_center': face_center,
            'left_eye_left': [left_eye_left.x, left_eye_left.y],
            'left_eye_right': [left_eye_right.x, left_eye_right.y],
            'left_eye_top': [left_eye_top.x, left_eye_top.y],
            'left_eye_bottom': [left_eye_bottom.x, left_eye_bottom.y],
            'right_eye_left': [right_eye_left.x, right_eye_left.y],
            'right_eye_right': [right_eye_right.x, right_eye_right.y],
            'right_eye_top': [right_eye_top.x, right_eye_top.y],
            'right_eye_bottom': [right_eye_bottom.x, right_eye_bottom.y],
            'forehead': [forehead.x, forehead.y],
            'chin': [chin.x, chin.y],
            'head_tilt_lr': head_tilt_lr,
            'nose_ratio': nose_ratio
        }
        
        return gaze_x, gaze_y, viz_points
    
    def detect_blink(self, face_landmarks):
        """
        Detect eye openness using Eye Aspect Ratio (continuous 0-1, not binary)
        Returns openness ratio and EAR value
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
        
        # Convert EAR to continuous openness (0=closed, 1=fully open)
        # Typical EAR range: 0.15 (closed) to 0.3 (open)
        min_ear = 0.15
        max_ear = 0.30
        openness = (ear - min_ear) / (max_ear - min_ear)
        openness = max(0.0, min(1.0, openness))  # Clamp to 0-1
        
        return openness, ear
    
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
    
    def send_to_esp32(self, eye_left_lr, eye_right_lr, eyelid_left, eyelid_right, 
                      tilt_left, tilt_right, head_rotate):
        """
        Send all 7 servo positions to ESP32 via serial
        Format: "S1:90,S2:90,S3:120,S4:120,S5:90,S6:90,S7:90\n"
        
        S1: Left Eye LR (60-120)
        S2: Right Eye LR (60-120)
        S3: Left Eyelid (70-120, 120=open, 70=closed)
        S4: Right Eyelid (70-120, 120=open, 70=closed)
        S5: Tilt Left Servo (60-120)
        S6: Tilt Right Servo (60-120)
        S7: Head Rotate (60-120)
        """
        command = f"S1:{eye_left_lr},S2:{eye_right_lr},S3:{eyelid_left},S4:{eyelid_right},S5:{tilt_left},S6:{tilt_right},S7:{head_rotate}\n"
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
        print("\n7 Servos Configuration:")
        print("  S1 & S2 - Left/Right Eye LR (60-120°)")
        print("  S3 & S4 - Left/Right Eyelid (70-120°, 120=open)")
        print("  S5 & S6 - Left/Right Tilt Servos (head tilt)")
        print("  S7 - Head Rotation LR (60-120°)")
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
                
                # Get gaze position and head orientation (face-relative, not image-relative)
                gaze_x, gaze_y, viz_points = self.get_gaze_position(face_landmarks, frame.shape[1], frame.shape[0])
                
                # Detect eye openness (continuous, not binary)
                eye_openness, ear = self.detect_blink(face_landmarks)
                
                # Map gaze to servo angles (gaze_x and gaze_y are now 0-1 ratios within eye)
                # Center is 0.5, so we map to servo range
                eye_lr_angle = int(90 + (gaze_x - 0.5) * 60)  # 0.5 ± range maps to 90 ± 30
                eye_lr_angle = max(60, min(120, eye_lr_angle))
                
                # Servo 1 & 2: Left and Right Eyeball LR (60-120 degrees)
                # Both eyes move together to mirror your eye movement
                eye_left_lr = eye_lr_angle
                eye_right_lr = eye_lr_angle
                
                # Servo 3 & 4: Left and Right Eyelid (70-120 degrees, continuous based on openness)
                # openness: 1.0=fully open (120°), 0.0=fully closed (70°)
                eyelid_left = int(70 + (eye_openness * 50))  # Maps 0-1 to 70-120
                eyelid_right = int(70 + (eye_openness * 50))
                eyelid_left = max(70, min(120, eyelid_left))
                eyelid_right = max(70, min(120, eyelid_right))
                
                # Servo 5 & 6: Head Tilt based on FACE ORIENTATION, not position in image
                # Uses vertical gaze within eye (0.5 = center, <0.5 = looking up, >0.5 = looking down)
                pitch_offset = (gaze_y - 0.5) * 60  # ±30 degrees
                
                # Also incorporate face pitch from nose ratio
                face_pitch = (viz_points['nose_ratio'] - 0.5) * 40  # Additional pitch from face angle
                
                # Combine eye gaze and face pitch
                total_pitch = pitch_offset + face_pitch
                
                tilt_left = int(90 + total_pitch)
                tilt_right = int(90 + total_pitch)
                tilt_left = max(60, min(120, tilt_left))
                tilt_right = max(60, min(120, tilt_right))
                
                # Servo 7: Head Rotation LR (mirrors horizontal gaze)
                head_rotate = eye_lr_angle  # Same as eyeball movement
                
                # Draw visualization points on frame
                frame_h, frame_w = frame.shape[:2]
                
                # Draw left iris (green)
                left_iris_px = (int(viz_points['left_iris'][0] * frame_w), 
                               int(viz_points['left_iris'][1] * frame_h))
                cv2.circle(frame, left_iris_px, 5, (0, 255, 0), -1)
                
                # Draw right iris (green)
                right_iris_px = (int(viz_points['right_iris'][0] * frame_w), 
                                int(viz_points['right_iris'][1] * frame_h))
                cv2.circle(frame, right_iris_px, 5, (0, 255, 0), -1)
                
                # Draw face center/nose (magenta)
                face_center_px = (int(viz_points['face_center'][0] * frame_w), 
                                 int(viz_points['face_center'][1] * frame_h))
                cv2.circle(frame, face_center_px, 8, (255, 0, 255), -1)
                
                # Draw forehead and chin (cyan)
                forehead_px = (int(viz_points['forehead'][0] * frame_w), 
                              int(viz_points['forehead'][1] * frame_h))
                chin_px = (int(viz_points['chin'][0] * frame_w), 
                          int(viz_points['chin'][1] * frame_h))
                cv2.circle(frame, forehead_px, 4, (255, 255, 0), -1)
                cv2.circle(frame, chin_px, 4, (255, 255, 0), -1)
                
                # Draw eye boundaries (yellow)
                for key in ['left_eye_left', 'left_eye_right', 'left_eye_top', 'left_eye_bottom',
                           'right_eye_left', 'right_eye_right', 'right_eye_top', 'right_eye_bottom']:
                    px = (int(viz_points[key][0] * frame_w), int(viz_points[key][1] * frame_h))
                    cv2.circle(frame, px, 2, (0, 255, 255), -1)
                
                # Send to ESP32
                self.send_to_esp32(eye_left_lr, eye_right_lr, eyelid_left, eyelid_right, 
                                  tilt_left, tilt_right, head_rotate)
                
                # Display exactly 7 servo values in degrees
                cv2.putText(frame, f"S1 Eye Left LR:    {eye_left_lr:3d}", (10, 40), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                cv2.putText(frame, f"S2 Eye Right LR:   {eye_right_lr:3d}", (10, 75), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                cv2.putText(frame, f"S3 Eyelid Left:    {eyelid_left:3d} [{eye_openness:.2f}]", (10, 110), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
                cv2.putText(frame, f"S4 Eyelid Right:   {eyelid_right:3d} [{eye_openness:.2f}]", (10, 145), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
                cv2.putText(frame, f"S5 Tilt Left:      {tilt_left:3d}", (10, 180), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                cv2.putText(frame, f"S6 Tilt Right:     {tilt_right:3d}", (10, 215), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                cv2.putText(frame, f"S7 Head Rotate:    {head_rotate:3d}", (10, 250), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 128, 0), 2)
                
                # Add legend and debug info
                cv2.putText(frame, "Green: Iris | Magenta: Nose | Yellow: Face/Eye Points", 
                           (10, frame_h - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.putText(frame, f"Gaze: X={gaze_x:.2f} Y={gaze_y:.2f} | Face Pitch={viz_points['nose_ratio']:.2f}", 
                           (10, frame_h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
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
                    gaze_x, gaze_y, _ = self.get_gaze_position(
                        results.multi_face_landmarks[0], 
                        frame.shape[1], frame.shape[0]
                    )
                    self.calibration['center_x'] = gaze_x
                    self.calibration['center_y'] = gaze_y
                    print(f"Center set to: X={gaze_x:.3f}, Y={gaze_y:.3f}")
            elif key == ord('l') or key == ord('L'):
                if self.calibration_mode and results.multi_face_landmarks:
                    gaze_x, _, _ = self.get_gaze_position(
                        results.multi_face_landmarks[0], 
                        frame.shape[1], frame.shape[0]
                    )
                    left_range = abs(gaze_x - self.calibration['center_x'])
                    print(f"Left range: {left_range:.3f}")
                    self.calibration_points.append(('left', left_range))
            elif key == ord('r') or key == ord('R'):
                if self.calibration_mode and results.multi_face_landmarks:
                    gaze_x, _, _ = self.get_gaze_position(
                        results.multi_face_landmarks[0], 
                        frame.shape[1], frame.shape[0]
                    )
                    right_range = abs(gaze_x - self.calibration['center_x'])
                    print(f"Right range: {right_range:.3f}")
                    self.calibration_points.append(('right', right_range))
            elif key == ord('u') or key == ord('U'):
                if self.calibration_mode and results.multi_face_landmarks:
                    _, gaze_y, _ = self.get_gaze_position(
                        results.multi_face_landmarks[0], 
                        frame.shape[1], frame.shape[0]
                    )
                    up_range = abs(gaze_y - self.calibration['center_y'])
                    print(f"Up range: {up_range:.3f}")
                    self.calibration_points.append(('up', up_range))
            elif key == ord('d') or key == ord('D'):
                if self.calibration_mode and results.multi_face_landmarks:
                    _, gaze_y, _ = self.get_gaze_position(
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

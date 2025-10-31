import cv2
import mediapipe as mp
import math
import serial
import serial.tools.list_ports
import time
import json

# Initialize FaceMesh
mp_face_mesh = mp.solutions.face_mesh
face_mesh = mp_face_mesh.FaceMesh(refine_landmarks=True, max_num_faces=1)

# --- Utility Functions ---
def calc_distance(p1, p2):
    return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

def smooth(prev, new, alpha=0.3):
    if prev is None:
        return new
    return alpha * new + (1 - alpha) * prev

def map_range(value, in_min, in_max, out_min, out_max):
    value = max(min(value, in_max), in_min)  # clamp
    return out_min + (value - in_min) * (out_max - out_min) / (in_max - in_min)

# --- Configurable Ranges ---
# Eyelid servo range
servo_left_lid_min = 30
servo_left_lid_max = 70
lid_gap_min = 3
lid_gap_max = 11

# Eyeball intermediate mapping ranges (eye-nose distance to intermediate values)
left_eye_intermediate_min = 80
left_eye_intermediate_max = 120
right_eye_intermediate_min = 80
right_eye_intermediate_max = 120

# Final servo output range (both eyes)
left_eye_servo_min = 35
left_eye_servo_max = 155
right_eye_servo_min = 35
right_eye_servo_max = 155

# Expected eyeball-nose distance range (calibration needed)
eye_nose_min = 90     # near (eye moves inward)
eye_nose_max = 155     # far (eye moves outward)

# Baseline distance range (iris to bottom baseline)
baseline_dist_min = 100    # minimum distance from iris to baseline (adjust based on your setup)
baseline_dist_max = 250   # maximum distance from iris to baseline (adjust based on your setup)

# Servo angle mapping for baseline distance (90 to 270 degrees)
baseline_servo_min = 90   # 90 degrees
baseline_servo_max = 270  # 270 degrees

# Face rotation (left-right) detection range
# Rotation ratio typically ranges from -0.3 to +0.3 for natural head rotation
rotation_ratio_left_max = -0.3   # Full left rotation
rotation_ratio_right_max = 0.3   # Full right rotation

# Servo angle mapping for face rotation (left-right movement)
rotation_servo_min = 0     # Full left rotation
rotation_servo_max = 180   # Full right rotation

# --- Serial Communication Setup ---
def find_esp32_port():
    """Find and return the ESP32 COM port"""
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if 'USB' in port.description or 'CH340' in port.description or 'CP210' in port.description:
            return port.device
    # If no specific port found, return None
    return None

# Try to establish serial connection
esp32_port = find_esp32_port()
ser = None

if esp32_port:
    try:
        ser = serial.Serial(esp32_port, 115200, timeout=1)
        time.sleep(2)  # Wait for connection to stabilize
        print(f"Connected to ESP32 on {esp32_port}")
    except Exception as e:
        print(f"Could not connect to ESP32: {e}")
        ser = None
else:
    print("ESP32 not found. Available ports:")
    for port in serial.tools.list_ports.comports():
        print(f"  {port.device}: {port.description}")

# Webcam
cap = cv2.VideoCapture(0)

# Fixed Reference Box (static coordinates)
# These will be set based on your webcam resolution
FIXED_BOX_LEFT = 150
FIXED_BOX_RIGHT = 490
FIXED_BOX_TOP = 100
FIXED_BOX_BOTTOM = 400
FIXED_BASELINE_Y = 400  # Fixed horizontal baseline at bottom of box
FIXED_CENTER_X = 320   # Fixed vertical center line

# Smoothing vars
smoothed_left_lid = smoothed_right_lid = None
smoothed_left_eye_nose = smoothed_right_eye_nose = None
smoothed_rotation_ratio = None

while cap.isOpened():
    success, frame = cap.read()
    if not success:
        continue

    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = face_mesh.process(rgb_frame)

    if results.multi_face_landmarks:
        for face_landmarks in results.multi_face_landmarks:
            h, w, _ = frame.shape

            # Landmarks
            LEFT_UPPER = 159
            LEFT_LOWER = 145
            RIGHT_UPPER = 386
            RIGHT_LOWER = 374
            NOSE_CENTER = 1
            LEFT_IRIS = 473
            RIGHT_IRIS = 468
            
            # Face rotation landmarks
            LEFT_FACE_EDGE = 234   # Left edge of face
            RIGHT_FACE_EDGE = 454  # Right edge of face

            # Coordinates
            left_upper = (int(face_landmarks.landmark[LEFT_UPPER].x * w),
                          int(face_landmarks.landmark[LEFT_UPPER].y * h))
            left_lower = (int(face_landmarks.landmark[LEFT_LOWER].x * w),
                          int(face_landmarks.landmark[LEFT_LOWER].y * h))
            right_upper = (int(face_landmarks.landmark[RIGHT_UPPER].x * w),
                           int(face_landmarks.landmark[RIGHT_UPPER].y * h))
            right_lower = (int(face_landmarks.landmark[RIGHT_LOWER].x * w),
                           int(face_landmarks.landmark[RIGHT_LOWER].y * h))
            nose = (int(face_landmarks.landmark[NOSE_CENTER].x * w),
                    int(face_landmarks.landmark[NOSE_CENTER].y * h))
            left_iris = (int(face_landmarks.landmark[LEFT_IRIS].x * w),
                         int(face_landmarks.landmark[LEFT_IRIS].y * h))
            right_iris = (int(face_landmarks.landmark[RIGHT_IRIS].x * w),
                          int(face_landmarks.landmark[RIGHT_IRIS].y * h))
            
            # Face rotation reference points
            left_face = (int(face_landmarks.landmark[LEFT_FACE_EDGE].x * w),
                        int(face_landmarks.landmark[LEFT_FACE_EDGE].y * h))
            right_face = (int(face_landmarks.landmark[RIGHT_FACE_EDGE].x * w),
                         int(face_landmarks.landmark[RIGHT_FACE_EDGE].y * h))

            # --- Eyelid Gaps ---
            left_gap = calc_distance(left_upper, left_lower)
            right_gap = calc_distance(right_upper, right_lower)

            smoothed_left_lid = smooth(smoothed_left_lid, left_gap)
            smoothed_right_lid = smooth(smoothed_right_lid, right_gap)

            left_lid_mapped = map_range(smoothed_left_lid, lid_gap_min, lid_gap_max,
                                        servo_left_lid_min, servo_left_lid_max)
            right_lid_mapped = map_range(smoothed_right_lid, lid_gap_min, lid_gap_max,
                                         servo_left_lid_min, servo_left_lid_max)

            # --- Eye–Nose Distance ---
            left_eye_nose = calc_distance(nose, left_iris)
            right_eye_nose = calc_distance(nose, right_iris)
            
            # --- Iris to Fixed Baseline Distance (new reference measurement) ---
            # Using fixed baseline coordinates
            baseline_y = FIXED_BASELINE_Y
            baseline_left_x = FIXED_BOX_LEFT
            baseline_right_x = FIXED_BOX_RIGHT
            
            # Distance from iris to fixed baseline (vertical distance)
            left_iris_baseline_dist = abs(left_iris[1] - baseline_y)
            right_iris_baseline_dist = abs(right_iris[1] - baseline_y)
            
            # Map baseline distance to servo angles (90 to 270 degrees)
            left_baseline_servo_angle = map_range(left_iris_baseline_dist, baseline_dist_min, baseline_dist_max,
                                                  baseline_servo_min, baseline_servo_max)
            right_baseline_servo_angle = map_range(right_iris_baseline_dist, baseline_dist_min, baseline_dist_max,
                                                   baseline_servo_min, baseline_servo_max)
            
            # Horizontal distance from iris to fixed face center line
            face_center_x = FIXED_CENTER_X
            left_iris_center_dist = abs(left_iris[0] - face_center_x)
            right_iris_center_dist = abs(right_iris[0] - face_center_x)
            
            # --- Face Rotation (Left-Right) Detection ---
            # Calculate distance from nose to each side of face
            left_side_dist = calc_distance(nose, left_face)
            right_side_dist = calc_distance(nose, right_face)
            
            # Calculate rotation ratio (independent of position)
            # When face turns left: left_side_dist increases, right_side_dist decreases
            # When face turns right: left_side_dist decreases, right_side_dist increases
            total_dist = left_side_dist + right_side_dist
            if total_dist > 0:
                rotation_ratio = (right_side_dist - left_side_dist) / total_dist
            else:
                rotation_ratio = 0
            
            # Apply smoothing to rotation ratio for more gradual changes
            smoothed_rotation_ratio = smooth(smoothed_rotation_ratio, rotation_ratio, alpha=0.2)
            
            # Map face rotation to servo angle with calibrated range
            # rotation_ratio ranges approximately from -0.3 (left) to +0.3 (right) for natural rotation
            face_rotation_servo = map_range(smoothed_rotation_ratio, rotation_ratio_left_max, rotation_ratio_right_max,
                                           rotation_servo_min, rotation_servo_max)
            
            # --- Prepare JSON Data ---
            data_json = {
                "left_lid": int(left_lid_mapped),
                "right_lid": int(right_lid_mapped),
                "left_baseline": int(left_baseline_servo_angle),
                "right_baseline": int(right_baseline_servo_angle),
                "rotation": int(face_rotation_servo),
                "left_eye_nose": round(smoothed_left_eye_nose, 1),
                "right_eye_nose": round(smoothed_right_eye_nose, 1),
                "left_baseline_dist": round(left_iris_baseline_dist, 1),
                "right_baseline_dist": round(right_iris_baseline_dist, 1),
                "rotation_ratio": round(smoothed_rotation_ratio, 3)
            }
            
            print(f"Left: baseline={left_iris_baseline_dist:.1f}px → {left_baseline_servo_angle:.1f}°")
            print(f"Right: baseline={right_iris_baseline_dist:.1f}px → {right_baseline_servo_angle:.1f}°")
            print(f"Face Rotation: ratio={smoothed_rotation_ratio:.3f} → {face_rotation_servo:.1f}° (L:{left_side_dist:.1f} R:{right_side_dist:.1f})")
            
            # --- Send Data to ESP32 via Serial as JSON ---
            if ser and ser.is_open:
                json_string = json.dumps(data_json) + "\n"
                try:
                    ser.write(json_string.encode())
                    print(f"Sent JSON to ESP32: {json_string.strip()}")
                except Exception as e:
                    print(f"Error sending data: {e}")

            smoothed_left_eye_nose =  left_eye_nose
            smoothed_right_eye_nose =  right_eye_nose

            # Map to intermediate range first (eye-nose distance to intermediate values)
            left_eye_intermediate = map_range(smoothed_left_eye_nose, eye_nose_min, eye_nose_max,
                                             left_eye_intermediate_min, left_eye_intermediate_max)
            right_eye_intermediate = map_range(smoothed_right_eye_nose, eye_nose_min, eye_nose_max,
                                              right_eye_intermediate_min, right_eye_intermediate_max)
            
            # Then map intermediate values to final servo range (40-70)
            left_eye_servo = map_range(left_eye_intermediate, left_eye_intermediate_min, left_eye_intermediate_max,
                                       left_eye_servo_min, left_eye_servo_max)
            right_eye_servo = map_range(right_eye_intermediate, right_eye_intermediate_min, right_eye_intermediate_max,
                                        right_eye_servo_min, right_eye_servo_max)

            # --- Draw Points ---
            cv2.circle(frame, nose, 3, (0, 0, 255), -1)
            cv2.circle(frame, left_iris, 3, (0, 255, 255), -1)
            cv2.circle(frame, right_iris, 3, (255, 0, 255), -1)
            cv2.line(frame, nose, left_iris, (0, 255, 255), 1)
            cv2.line(frame, nose, right_iris, (255, 0, 255), 1)
            
            # --- Draw Fixed Reference Box ---
            # Fixed face boundary rectangle
            cv2.rectangle(frame, 
                         (FIXED_BOX_LEFT, FIXED_BOX_TOP), 
                         (FIXED_BOX_RIGHT, FIXED_BOX_BOTTOM), 
                         (255, 255, 0), 2)  # Yellow box
            
            # Fixed baseline horizontal line
            cv2.line(frame, (FIXED_BOX_LEFT, FIXED_BASELINE_Y), (FIXED_BOX_RIGHT, FIXED_BASELINE_Y), (0, 255, 0), 2)  # Green baseline
            
            # Fixed vertical center line
            cv2.line(frame, 
                    (FIXED_CENTER_X, FIXED_BOX_TOP), 
                    (FIXED_CENTER_X, FIXED_BOX_BOTTOM), 
                    (255, 0, 0), 1)  # Blue center line
            
            # Draw iris to fixed baseline lines
            cv2.line(frame, left_iris, (left_iris[0], FIXED_BASELINE_Y), (0, 255, 255), 1)
            cv2.line(frame, right_iris, (right_iris[0], FIXED_BASELINE_Y), (255, 0, 255), 1)
            
            # Draw face rotation indicator (lines from nose to face edges)
            cv2.line(frame, nose, left_face, (255, 128, 0), 1)   # Orange line to left face edge
            cv2.line(frame, nose, right_face, (255, 128, 0), 1)  # Orange line to right face edge
            cv2.circle(frame, left_face, 3, (255, 128, 0), -1)   # Left face reference point
            cv2.circle(frame, right_face, 3, (255, 128, 0), -1)  # Right face reference point

            # --- Display Text ---
            cv2.putText(frame, f"Lid Left: {left_lid_mapped:.1f}", (30, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(frame, f"Lid Right: {right_lid_mapped:.1f}", (30, 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            cv2.putText(frame, f"Left Eye-Nose: {smoothed_left_eye_nose:.1f}px", (30, 110),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.putText(frame, f"Right Eye-Nose: {smoothed_right_eye_nose:.1f}px", (30, 140),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
            
            # New baseline measurements
            cv2.putText(frame, f"Left Baseline: {left_iris_baseline_dist:.1f}px", (30, 170),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(frame, f"Right Baseline: {right_iris_baseline_dist:.1f}px", (30, 200),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            # Baseline servo angles (90-270 degrees)
            cv2.putText(frame, f"Left Angle: {left_baseline_servo_angle:.1f}", (30, 230),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            cv2.putText(frame, f"Right Angle: {right_baseline_servo_angle:.1f}", (30, 260),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            
            # Face rotation servo angle
            cv2.putText(frame, f"Face Rotation: {face_rotation_servo:.1f}", (30, 290),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 128, 0), 2)

            cv2.putText(frame, f"Servo L-Eye: {left_eye_servo:.1f}", (30, 320),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 200), 2)
            cv2.putText(frame, f"Servo R-Eye: {right_eye_servo:.1f}", (30, 350),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 100, 255), 2)

    cv2.imshow("Eyelid & Eyeball Servo Mapping", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

# Close serial connection
if ser and ser.is_open:
    ser.close()
    print("Serial connection closed")

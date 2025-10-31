import cv2
import mediapipe as mp
import math

# Initialize FaceMesh
mp_face_mesh = mp.solutions.face_mesh
face_mesh = mp_face_mesh.FaceMesh(refine_landmarks=True, max_num_faces=1)

# Function to calculate Euclidean distance
def calc_distance(p1, p2):
    return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

# Smooth distance
def smooth(prev, new, alpha=0.3):
    if prev is None:
        return new
    return alpha*new + (1-alpha)*prev

# Start webcam
cap = cv2.VideoCapture(0)

# Previous smoothed distances
smoothed_left = None
smoothed_right = None

while cap.isOpened():
    success, frame = cap.read()
    if not success:
        continue

    # Convert BGR to RGB
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = face_mesh.process(rgb_frame)

    if results.multi_face_landmarks:
        for face_landmarks in results.multi_face_landmarks:
            h, w, _ = frame.shape

            # Eyelid landmark indexes
            LEFT_UPPER = 159
            LEFT_LOWER = 145
            RIGHT_UPPER = 386
            RIGHT_LOWER = 374

            # Get coordinates
            left_upper = (int(face_landmarks.landmark[LEFT_UPPER].x * w),
                          int(face_landmarks.landmark[LEFT_UPPER].y * h))
            left_lower = (int(face_landmarks.landmark[LEFT_LOWER].x * w),
                          int(face_landmarks.landmark[LEFT_LOWER].y * h))
            right_upper = (int(face_landmarks.landmark[RIGHT_UPPER].x * w),
                           int(face_landmarks.landmark[RIGHT_UPPER].y * h))
            right_lower = (int(face_landmarks.landmark[RIGHT_LOWER].x * w),
                           int(face_landmarks.landmark[RIGHT_LOWER].y * h))

            # Calculate distances (eyelid gap)
            left_gap = calc_distance(left_upper, left_lower)
            right_gap = calc_distance(right_upper, right_lower)

            # Smooth distances
            smoothed_left = smooth(smoothed_left, left_gap)
            smoothed_right = smooth(smoothed_right, right_gap)

            # Display on frame
            cv2.putText(frame, f"Left Eye Gap: {smoothed_left:.2f}", (30, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(frame, f"Right Eye Gap: {smoothed_right:.2f}", (30, 80),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    # Show frame
    cv2.imshow("Eyelid Gap", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

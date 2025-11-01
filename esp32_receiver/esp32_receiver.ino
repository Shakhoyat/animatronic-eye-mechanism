/*
 * ESP32 Eye Tracking Receiver with ESP32Servo
 * Receives servo angle data via USB Serial in JSON format and controls servos
 */

#include <ESP32Servo.h>
#include <ArduinoJson.h>

// Serial buffer
String serialBuffer = "";

// Servo objects (7 servos total)
Servo servoLeftLid;           // Servo 1
Servo servoRightLid;          // Servo 2
Servo servoLeftEyeVertical;   // Servo 3
Servo servoRightEyeVertical;  // Servo 4
Servo servoLeftEyeHorizontal; // Servo 5
Servo servoRightEyeHorizontal;// Servo 6
Servo servoHeadRotation;      // Servo 7

// GPIO pin assignments for servos (adjust these to your actual wiring)
#define PIN_LEFT_LID 13
#define PIN_RIGHT_LID 12
#define PIN_LEFT_EYE_VERTICAL 14
#define PIN_RIGHT_EYE_VERTICAL 27
#define PIN_LEFT_EYE_HORIZONTAL 26
#define PIN_RIGHT_EYE_HORIZONTAL 25
#define PIN_HEAD_ROTATION 33

// Servo pulse width settings (microseconds)
#define SERVO_MIN_PULSE 500   // Minimum pulse width in microseconds
#define SERVO_MAX_PULSE 2500  // Maximum pulse width in microseconds

// LED for status indication
#define STATUS_LED 2

// Variables to store received servo angles
float leftLid = 50.0;
float rightLid = 50.0;
float leftEyeVertical = 180.0;
float rightEyeVertical = 180.0;
float leftEyeHorizontal = 90.0;
float rightEyeHorizontal = 90.0;
float headRotation = 90.0;

// Timeout settings
unsigned long lastPacketTime = 0;
const unsigned long TIMEOUT_MS = 1000;  // 1 second timeout

void setup() {
  Serial.begin(115200);
  Serial.println("\n\nESP32 Eye Tracking Receiver Starting...");
  Serial.println("=== USB Serial Mode ===");
  
  // Setup status LED
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);
  
  // Allow allocation of all timers for servos
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  
  // Attach servos to GPIO pins (7 servos)
  servoLeftLid.setPeriodHertz(50);    // Standard 50Hz servo
  servoLeftLid.attach(PIN_LEFT_LID, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
  
  servoRightLid.setPeriodHertz(50);
  servoRightLid.attach(PIN_RIGHT_LID, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
  
  servoLeftEyeVertical.setPeriodHertz(50);
  servoLeftEyeVertical.attach(PIN_LEFT_EYE_VERTICAL, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
  
  servoRightEyeVertical.setPeriodHertz(50);
  servoRightEyeVertical.attach(PIN_RIGHT_EYE_VERTICAL, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
  
  servoLeftEyeHorizontal.setPeriodHertz(50);
  servoLeftEyeHorizontal.attach(PIN_LEFT_EYE_HORIZONTAL, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
  
  servoRightEyeHorizontal.setPeriodHertz(50);
  servoRightEyeHorizontal.attach(PIN_RIGHT_EYE_HORIZONTAL, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
  
  servoHeadRotation.setPeriodHertz(50);
  servoHeadRotation.attach(PIN_HEAD_ROTATION, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
  
  Serial.println("All 7 servos initialized");
  
  // Set all servos to neutral position
  servoLeftLid.write(50);
  servoRightLid.write(50);
  servoLeftEyeVertical.write(180);
  servoRightEyeVertical.write(180);
  servoLeftEyeHorizontal.write(90);
  servoRightEyeHorizontal.write(90);
  servoHeadRotation.write(90);
  
  Serial.println("Servos set to neutral position");
  Serial.println("Ready to receive data via USB Serial...");
  
  // LED on to indicate ready
  digitalWrite(STATUS_LED, HIGH);
}

// Parse serial data format: JSON
void parseSerialData(String data) {
  // Print raw received JSON
  Serial.println("=== Received JSON ===");
  Serial.println(data);
  
  // Create JSON document
  StaticJsonDocument<256> doc;
  
  // Deserialize JSON
  DeserializationError error = deserializeJson(doc, data);
  
  if (error) {
    Serial.print("JSON parse error: ");
    Serial.println(error.c_str());
    return;
  }
  
  // Extract values from JSON with proper mapping
  // JSON keys from Python: left_lid, right_lid, left_baseline, right_baseline, rotation
  leftLid = doc["left_lid"] | 50;                    // Servo 1: Left eyelid
  rightLid = doc["right_lid"] | 50;                  // Servo 2: Right eyelid
  leftEyeVertical = doc["left_baseline"] | 180;      // Servo 3: Left eye vertical
  rightEyeVertical = doc["right_baseline"] | 180;    // Servo 4: Right eye vertical
  headRotation = doc["rotation"] | 90;               // Servo 7: Head rotation
  
  // Servos 5 & 6: Left/Right eye horizontal (can be derived from left_eye_nose if needed)
  // For now, using eye_nose values or keeping at neutral if not provided
  if (doc.containsKey("left_eye_nose")) {
    leftEyeHorizontal = doc["left_eye_nose"].as<float>();
  }
  if (doc.containsKey("right_eye_nose")) {
    rightEyeHorizontal = doc["right_eye_nose"].as<float>();
  }
  
  // Print all parsed JSON values
  Serial.println("=== Parsed Values ===");
  Serial.printf("Servo 1 - Left Lid: %.1f\n", leftLid);
  Serial.printf("Servo 2 - Right Lid: %.1f\n", rightLid);
  Serial.printf("Servo 3 - Left Eye Vertical: %.1f\n", leftEyeVertical);
  Serial.printf("Servo 4 - Right Eye Vertical: %.1f\n", rightEyeVertical);
  Serial.printf("Servo 5 - Left Eye Horizontal: %.1f\n", leftEyeHorizontal);
  Serial.printf("Servo 6 - Right Eye Horizontal: %.1f\n", rightEyeHorizontal);
  Serial.printf("Servo 7 - Head Rotation: %.1f\n", headRotation);
  
  // Print additional debug data if available
  if (doc.containsKey("left_baseline_dist")) {
    Serial.printf("  [Debug] Left Baseline Dist: %.1f\n", doc["left_baseline_dist"].as<float>());
  }
  if (doc.containsKey("right_baseline_dist")) {
    Serial.printf("  [Debug] Right Baseline Dist: %.1f\n", doc["right_baseline_dist"].as<float>());
  }
  if (doc.containsKey("rotation_ratio")) {
    Serial.printf("  [Debug] Rotation Ratio: %.3f\n", doc["rotation_ratio"].as<float>());
  }
  Serial.println("====================\n");
  
  // Write to all 7 servos
  servoLeftLid.write(leftLid);                    // Servo 1: Left eyelid
  servoRightLid.write(rightLid);                  // Servo 2: Right eyelid
  servoLeftEyeVertical.write(leftEyeVertical);    // Servo 3: Left eye vertical
  servoRightEyeVertical.write(rightEyeVertical);  // Servo 4: Right eye vertical
  servoLeftEyeHorizontal.write(leftEyeHorizontal);  // Servo 5: Left eye horizontal
  servoRightEyeHorizontal.write(rightEyeHorizontal); // Servo 6: Right eye horizontal
  servoHeadRotation.write(headRotation);          // Servo 7: Head rotation
  
  lastPacketTime = millis();
}

void loop() {
  // USB Serial Mode - Read from Serial
  if (Serial.available()) {
    while (Serial.available()) {
      char c = Serial.read();
      
      if (c == '\n') {
        // Process complete line
        if (serialBuffer.length() > 0) {
          parseSerialData(serialBuffer);
          serialBuffer = "";
          digitalWrite(STATUS_LED, HIGH);  // Blink on data received
        }
      } else if (c != '\r') {
        // Add character to buffer (ignore carriage return)
        serialBuffer += c;
        
        // Prevent buffer overflow
        if (serialBuffer.length() > 100) {
          serialBuffer = "";
        }
      }
    }
  }
  
  // Check for timeout (no data received)
  if (millis() - lastPacketTime > TIMEOUT_MS && lastPacketTime > 0) {
    // Return all 7 servos to neutral position on timeout
    servoLeftLid.write(50);                   // Servo 1: Neutral
    servoRightLid.write(50);                  // Servo 2: Neutral
    servoLeftEyeVertical.write(180);          // Servo 3: Neutral
    servoRightEyeVertical.write(180);         // Servo 4: Neutral
    servoLeftEyeHorizontal.write(90);         // Servo 5: Neutral
    servoRightEyeHorizontal.write(90);        // Servo 6: Neutral
    servoHeadRotation.write(90);              // Servo 7: Neutral
    
    // Blink LED to indicate timeout
    digitalWrite(STATUS_LED, millis() % 1000 < 500);
  } else if (lastPacketTime > 0) {
    digitalWrite(STATUS_LED, HIGH);
  }
  
  delay(1);  // Small delay to prevent watchdog reset
}

/*
 * ESP32 Eye Tracking Receiver with ESP32Servo
 * Receives servo angle data via USB Serial in JSON format and controls servos
 */

#include <ESP32Servo.h>
#include <ArduinoJson.h>

// Serial buffer
String serialBuffer = "";

// Servo objects
Servo servoLeftLid;
Servo servoRightLid;
Servo servoLeftEyeVertical;
Servo servoRightEyeVertical;
Servo servoHeadRotation;

// GPIO pin assignments for servos
#define PIN_LEFT_LID 13
#define PIN_RIGHT_LID 12
#define PIN_LEFT_EYE_VERTICAL 14
#define PIN_RIGHT_EYE_VERTICAL 27
#define PIN_HEAD_ROTATION 26

// Servo pulse width settings (microseconds)
#define SERVO_MIN_PULSE 500   // Minimum pulse width in microseconds
#define SERVO_MAX_PULSE 2500  // Maximum pulse width in microseconds

// LED for status indication
#define STATUS_LED 2

// Variables to store received servo angles
float leftLid = 90.0;
float rightLid = 90.0;
float leftBaseline = 180.0;
float rightBaseline = 180.0;
float rotation = 90.0;

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
  
  // Attach servos to GPIO pins
  servoLeftLid.setPeriodHertz(50);    // Standard 50Hz servo
  servoLeftLid.attach(PIN_LEFT_LID, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
  
  servoRightLid.setPeriodHertz(50);
  servoRightLid.attach(PIN_RIGHT_LID, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
  
  servoLeftEyeVertical.setPeriodHertz(50);
  servoLeftEyeVertical.attach(PIN_LEFT_EYE_VERTICAL, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
  
  servoRightEyeVertical.setPeriodHertz(50);
  servoRightEyeVertical.attach(PIN_RIGHT_EYE_VERTICAL, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
  
  servoHeadRotation.setPeriodHertz(50);
  servoHeadRotation.attach(PIN_HEAD_ROTATION, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
  
  Serial.println("Servos initialized");
  
  // Set all servos to neutral position
  servoLeftLid.write(50);
  servoRightLid.write(50);
  servoLeftEyeVertical.write(180);
  servoRightEyeVertical.write(180);
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
  
  // Extract values from JSON
  leftLid = doc["left_lid"] | 135;  // Default to 135 if not found
  rightLid = doc["right_lid"] | 135;
  leftBaseline = doc["left_baseline"] | 180;
  rightBaseline = doc["right_baseline"] | 180;
  rotation = doc["rotation"] | 90;
  
  // Print all parsed JSON values
  Serial.println("=== Parsed Values ===");
  Serial.printf("Left Lid: %.1f\n", leftLid);
  Serial.printf("Right Lid: %.1f\n", rightLid);
  Serial.printf("Left Baseline: %.1f\n", leftBaseline);
  Serial.printf("Right Baseline: %.1f\n", rightBaseline);
  Serial.printf("Rotation: %.1f\n", rotation);
  
  // Print additional data if available
  if (doc.containsKey("left_eye_nose")) {
    Serial.printf("Left Eye-Nose: %.1f\n", doc["left_eye_nose"].as<float>());
  }
  if (doc.containsKey("right_eye_nose")) {
    Serial.printf("Right Eye-Nose: %.1f\n", doc["right_eye_nose"].as<float>());
  }
  if (doc.containsKey("left_baseline_dist")) {
    Serial.printf("Left Baseline Dist: %.1f\n", doc["left_baseline_dist"].as<float>());
  }
  if (doc.containsKey("right_baseline_dist")) {
    Serial.printf("Right Baseline Dist: %.1f\n", doc["right_baseline_dist"].as<float>());
  }
  if (doc.containsKey("rotation_ratio")) {
    Serial.printf("Rotation Ratio: %.3f\n", doc["rotation_ratio"].as<float>());
  }
  Serial.println("====================\n");
  
  // Update servos
  servoLeftLid.write(leftLid);
  servoRightLid.write(rightLid);
  servoLeftEyeVertical.write(leftBaseline);
  servoRightEyeVertical.write(rightBaseline);
  servoHeadRotation.write(rotation);
  
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
    // Return to neutral position on timeout
    servoLeftLid.write(50);
    servoRightLid.write(50);
    servoLeftEyeVertical.write(180);
    servoRightEyeVertical.write(180);
    servoHeadRotation.write(90);
    
    // Blink LED to indicate timeout
    digitalWrite(STATUS_LED, millis() % 1000 < 500);
  } else if (lastPacketTime > 0) {
    digitalWrite(STATUS_LED, HIGH);
  }
  
  delay(1);  // Small delay to prevent watchdog reset
}

/*
 * ESP32 Eye Receiver for Animatronic Eye Mechanism (Arduino/C++)
 * Receives eye position data from PC via serial communication
 * Controls servos directly using ESP32's built-in PWM (no external controller needed)
 * 
 * Compatible with: Arduino IDE, PlatformIO
 * Board: ESP32 Dev Module
 * 
 * NO external libraries required - uses ESP32's built-in PWM!
 */

#include <ESP32Servo.h>

// Create Servo objects - 7 servos total
Servo servoEyeLeftLR;     // S1: Left Eye Left/Right
Servo servoEyeRightLR;    // S2: Right Eye Left/Right
Servo servoEyelidLeft;    // S3: Left Eyelid
Servo servoEyelidRight;   // S4: Right Eyelid
Servo servoTiltLeft;      // S5: Left Tilt Servo (bottom of eyes)
Servo servoTiltRight;     // S6: Right Tilt Servo (bottom of eyes)
Servo servoHeadRotate;    // S7: Head Rotation Left/Right

// GPIO pins for servos (adjust these to your wiring)
#define PIN_EYE_LEFT_LR    13  // S1: Left Eye LR
#define PIN_EYE_RIGHT_LR   12  // S2: Right Eye LR
#define PIN_EYELID_LEFT    14  // S3: Left Eyelid
#define PIN_EYELID_RIGHT   27  // S4: Right Eyelid
#define PIN_TILT_LEFT      26  // S5: Left Tilt
#define PIN_TILT_RIGHT     25  // S6: Right Tilt
#define PIN_HEAD_ROTATE    33  // S7: Head Rotate

// Servo limits (adjust these to match your hardware)
struct ServoLimits {
  int min;
  int max;
};

ServoLimits servoLimits[] = {
  {60, 120},   // S1: Left Eye LR
  {60, 120},   // S2: Right Eye LR
  {70, 120},   // S3: Left Eyelid (120=open, 70=closed)
  {70, 120},   // S4: Right Eyelid (120=open, 70=closed)
  {60, 120},   // S5: Left Tilt Servo
  {60, 120},   // S6: Right Tilt Servo
  {60, 120}    // S7: Head Rotate
};

// Serial communication settings
#define SERIAL_BAUD 115200

// State variables
float currentEyeLeftLR = 90.0;
float currentEyeRightLR = 90.0;
float currentEyelidLeft = 120.0;
float currentEyelidRight = 120.0;
float currentTiltLeft = 90.0;
float currentTiltRight = 90.0;
float currentHeadRotate = 90.0;

// Smoothing parameter (0.1 = smooth, 0.9 = responsive)
const float smoothing = 0.3;      // For eyes and head
const float smoothingEyelid = 0.8; // Fast for eyelids

// Timeout settings
unsigned long lastUpdateTime = 0;
const unsigned long timeoutDuration = 1000;  // 1 second

// Serial buffer
String inputBuffer = "";

// Function prototypes
void initializeServos();
void setServoAngle(Servo &servo, int angle);
void parseCommand(String command);
void updatePosition(int s1, int s2, int s3, int s4, int s5, int s6, int s7);
float smoothAngle(float target, float current, float smoothFactor);

void setup() {
  // Initialize serial communication with PC
  Serial.begin(SERIAL_BAUD);
  
  Serial.println("\n=== ESP32 Eye Mechanism Initializing ===");
  Serial.println("Direct PWM Control - No PCA9685 needed!");
  
  // Allow allocation of all timers for servos
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  
  // Attach servos to GPIO pins
  servoEyeLeftLR.attach(PIN_EYE_LEFT_LR, 500, 2500);
  servoEyeRightLR.attach(PIN_EYE_RIGHT_LR, 500, 2500);
  servoEyelidLeft.attach(PIN_EYELID_LEFT, 500, 2500);
  servoEyelidRight.attach(PIN_EYELID_RIGHT, 500, 2500);
  servoTiltLeft.attach(PIN_TILT_LEFT, 500, 2500);
  servoTiltRight.attach(PIN_TILT_RIGHT, 500, 2500);
  servoHeadRotate.attach(PIN_HEAD_ROTATE, 500, 2500);
  
  delay(100);
  
  // Initialize servos to neutral position
  initializeServos();
  
  Serial.println("Eye Mechanism initialized - 7 Servos");
  Serial.println("Waiting for serial data from PC...");
  Serial.println("Format: S1:90,S2:90,S3:120,S4:120,S5:90,S6:90,S7:90");
}

void loop() {
  // Check for serial data from USB
  while (Serial.available() > 0) {
    char c = Serial.read();
    
    if (c == '\n') {
      // Process complete command
      if (inputBuffer.length() > 0) {
        parseCommand(inputBuffer);
        lastUpdateTime = millis();
      }
      inputBuffer = "";
    } else {
      inputBuffer += c;
    }
  }
  
  // Timeout check - return to neutral if no data received
  if (millis() - lastUpdateTime > timeoutDuration) {
    if (currentEyeLeftLR != 90.0) {
      Serial.println("Timeout: Returning to neutral position");
      currentEyeLeftLR = 90.0;
      currentEyeRightLR = 90.0;
      currentEyelidLeft = 120.0;
      currentEyelidRight = 120.0;
      currentTiltLeft = 90.0;
      currentTiltRight = 90.0;
      currentHeadRotate = 90.0;
      
      setServoAngle(servoEyeLeftLR, 90);
      setServoAngle(servoEyeRightLR, 90);
      setServoAngle(servoEyelidLeft, 120);
      setServoAngle(servoEyelidRight, 120);
      setServoAngle(servoTiltLeft, 90);
      setServoAngle(servoTiltRight, 90);
      setServoAngle(servoHeadRotate, 90);
      
      lastUpdateTime = millis();
    }
  }
  
  delay(10);  // Small delay to prevent busy loop
}

void initializeServos() {
  Serial.println("Initializing servos to neutral position...");
  
  // Center eyeballs
  setServoAngle(servoEyeLeftLR, 90);
  setServoAngle(servoEyeRightLR, 90);
  
  // Open eyelids (120 = open)
  setServoAngle(servoEyelidLeft, 120);
  setServoAngle(servoEyelidRight, 120);
  
  // Center tilt servos and head rotation
  setServoAngle(servoTiltLeft, 90);
  setServoAngle(servoTiltRight, 90);
  setServoAngle(servoHeadRotate, 90);
  
  delay(500);
  Serial.println("All 7 servos initialized");
}

void setServoAngle(Servo &servo, int angle) {
  // Clamp angle to valid range (0-180)
  angle = constrain(angle, 0, 180);
  
  // Write angle directly to servo
  servo.write(angle);
}

void controlEyelids(bool shouldBlink) {
  /*
   * Control eyelids - open or closed
   */
  if (shouldBlink) {
    // Close eyelids
    setServoAngle(servoEyelidLeft, servoLimits[2].min);   // CLOSED
    setServoAngle(servoEyelidRight, servoLimits[3].min);  // CLOSED
  } else {
    // Open eyelids
    setServoAngle(servoEyelidLeft, servoLimits[2].max);   // OPEN
    setServoAngle(servoEyelidRight, servoLimits[3].max);  // OPEN
  }
}

void blinkEyes() {
  // Close eyelids
  controlEyelids(true);
}

void parseCommand(String command) {
  /*
   * Parse serial command from PC
   * Format: "S1:90,S2:90,S3:120,S4:120,S5:90,S6:90,S7:90"
   * S1 = Left Eye LR
   * S2 = Right Eye LR
   * S3 = Left Eyelid
   * S4 = Right Eyelid
   * S5 = Left Tilt
   * S6 = Right Tilt
   * S7 = Head Rotate
   */
  
  command.trim();
  
  int s1 = 90;   // Default values
  int s2 = 90;
  int s3 = 120;
  int s4 = 120;
  int s5 = 90;
  int s6 = 90;
  int s7 = 90;
  
  // Split by comma and parse
  int startPos = 0;
  int commaPos = 0;
  
  while (commaPos != -1) {
    commaPos = command.indexOf(',', startPos);
    String part;
    
    if (commaPos == -1) {
      part = command.substring(startPos);
    } else {
      part = command.substring(startPos, commaPos);
      startPos = commaPos + 1;
    }
    
    // Parse key:value pair
    int colonPos = part.indexOf(':');
    if (colonPos != -1) {
      String key = part.substring(0, colonPos);
      String value = part.substring(colonPos + 1);
      
      key.trim();
      value.trim();
      
      if (key == "S1") {
        s1 = value.toInt();
      } else if (key == "S2") {
        s2 = value.toInt();
      } else if (key == "S3") {
        s3 = value.toInt();
      } else if (key == "S4") {
        s4 = value.toInt();
      } else if (key == "S5") {
        s5 = value.toInt();
      } else if (key == "S6") {
        s6 = value.toInt();
      } else if (key == "S7") {
        s7 = value.toInt();
      }
    }
  }
  
  // Update position
  updatePosition(s1, s2, s3, s4, s5, s6, s7);
}

void updatePosition(int s1, int s2, int s3, int s4, int s5, int s6, int s7) {
  // Apply smoothing to all servos
  float smooth1 = smoothAngle((float)s1, currentEyeLeftLR, smoothing);
  float smooth2 = smoothAngle((float)s2, currentEyeRightLR, smoothing);
  float smooth3 = smoothAngle((float)s3, currentEyelidLeft, smoothingEyelid);
  float smooth4 = smoothAngle((float)s4, currentEyelidRight, smoothingEyelid);
  float smooth5 = smoothAngle((float)s5, currentTiltLeft, smoothing);
  float smooth6 = smoothAngle((float)s6, currentTiltRight, smoothing);
  float smooth7 = smoothAngle((float)s7, currentHeadRotate, smoothing);
  
  // Clamp to servo limits
  smooth1 = constrain(smooth1, servoLimits[0].min, servoLimits[0].max);
  smooth2 = constrain(smooth2, servoLimits[1].min, servoLimits[1].max);
  smooth3 = constrain(smooth3, servoLimits[2].min, servoLimits[2].max);
  smooth4 = constrain(smooth4, servoLimits[3].min, servoLimits[3].max);
  smooth5 = constrain(smooth5, servoLimits[4].min, servoLimits[4].max);
  smooth6 = constrain(smooth6, servoLimits[5].min, servoLimits[5].max);
  smooth7 = constrain(smooth7, servoLimits[6].min, servoLimits[6].max);
  
  // Update current positions
  currentEyeLeftLR = smooth1;
  currentEyeRightLR = smooth2;
  currentEyelidLeft = smooth3;
  currentEyelidRight = smooth4;
  currentTiltLeft = smooth5;
  currentTiltRight = smooth6;
  currentHeadRotate = smooth7;
  
  // Write to servos
  setServoAngle(servoEyeLeftLR, (int)currentEyeLeftLR);
  setServoAngle(servoEyeRightLR, (int)currentEyeRightLR);
  setServoAngle(servoEyelidLeft, (int)currentEyelidLeft);
  setServoAngle(servoEyelidRight, (int)currentEyelidRight);
  setServoAngle(servoTiltLeft, (int)currentTiltLeft);
  setServoAngle(servoTiltRight, (int)currentTiltRight);
  setServoAngle(servoHeadRotate, (int)currentHeadRotate);
}

float smoothAngle(float target, float current, float smoothFactor) {
  // Apply exponential smoothing
  return current + (target - current) * smoothFactor;
}

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
Servo servoEyeballLR;    // Eyeball Left/Right movement
Servo servoEyeballUD;    // Eyeball Up/Down movement
Servo servoEyelidLeft;   // Left eyelid
Servo servoEyelidRight;  // Right eyelid
Servo servoTiltLR;       // Face tilt Left/Right
Servo servoTiltUD;       // Face tilt Up/Down
Servo servoHeadRotate;   // Head rotation Left/Right

// GPIO pins for servos (adjust these to your wiring)
#define PIN_EYEBALL_LR    13  // Eyeball Left/Right
#define PIN_EYEBALL_UD    12  // Eyeball Up/Down
#define PIN_EYELID_LEFT   14  // Left eyelid
#define PIN_EYELID_RIGHT  27  // Right eyelid
#define PIN_TILT_LR       26  // Face tilt Left/Right
#define PIN_TILT_UD       25  // Face tilt Up/Down
#define PIN_HEAD_ROTATE   33  // Head rotation

// Servo limits (adjust these to match your hardware)
struct ServoLimits {
  int min;
  int max;
};

ServoLimits servoLimits[] = {
  {60, 120},   // EYEBALL_LR: Eyeball Left/Right (±30° from center)
  {60, 120},   // EYEBALL_UD: Eyeball Up/Down (±30° from center)
  {10, 90},    // EYELID_LEFT: Left eyelid (10=closed, 90=open)
  {10, 90},    // EYELID_RIGHT: Right eyelid (10=closed, 90=open)
  {0, 180},    // TILT_LR: Face tilt Left/Right (full range)
  {0, 180},    // TILT_UD: Face tilt Up/Down (full range)
  {0, 180}     // HEAD_ROTATE: Head rotation (full range)
};

// Serial communication settings
#define SERIAL_BAUD 115200

// State variables
float currentEyeballLR = 90.0;
float currentEyeballUD = 90.0;
float currentTiltLR = 90.0;
float currentTiltUD = 90.0;
float currentHeadRotate = 90.0;
bool isBlinking = false;
unsigned long blinkStartTime = 0;
const int blinkDuration = 150;  // milliseconds

// Smoothing parameter (0.1 = smooth, 0.9 = responsive)
const float smoothing = 0.3;
const float smoothingHead = 0.1;  // Slower smoothing for head movements

// Timeout settings
unsigned long lastUpdateTime = 0;
const unsigned long timeoutDuration = 1000;  // 1 second

// Serial buffer
String inputBuffer = "";

// Function prototypes
void initializeServos();
void setServoAngle(Servo &servo, int angle);
void controlEyelids(bool blinkState);
void blinkEyes();
void parseCommand(String command);
void updatePosition(int eyeLR, int eyeUD, int tiltLR, int tiltUD, int headRotate, bool blinkState);
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
  servoEyeballLR.attach(PIN_EYEBALL_LR, 500, 2500);  // Min/max pulse width in microseconds
  servoEyeballUD.attach(PIN_EYEBALL_UD, 500, 2500);
  servoEyelidLeft.attach(PIN_EYELID_LEFT, 500, 2500);
  servoEyelidRight.attach(PIN_EYELID_RIGHT, 500, 2500);
  servoTiltLR.attach(PIN_TILT_LR, 500, 2500);
  servoTiltUD.attach(PIN_TILT_UD, 500, 2500);
  servoHeadRotate.attach(PIN_HEAD_ROTATE, 500, 2500);
  
  delay(100);
  
  // Initialize servos to neutral position
  initializeServos();
  
  Serial.println("Eye Mechanism initialized");
  Serial.println("Waiting for serial data from PC...");
  Serial.println("Format: LR:90,UD:90,BL:0");
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
  
  // Handle blink duration
  if (isBlinking) {
    if (millis() - blinkStartTime > blinkDuration) {
      isBlinking = false;
      // Reopen eyes
      controlUDAndLids((int)currentUD);
    }
  }
  
  // Timeout check - return to neutral if no data received
  if (millis() - lastUpdateTime > timeoutDuration) {
    if (currentEyeballLR != 90.0 || currentEyeballUD != 90.0) {
      Serial.println("Timeout: Returning to neutral position");
      currentEyeballLR = 90.0;
      currentEyeballUD = 90.0;
      currentTiltLR = 90.0;
      currentTiltUD = 90.0;
      currentHeadRotate = 90.0;
      setServoAngle(servoEyeballLR, 90);
      setServoAngle(servoEyeballUD, 90);
      setServoAngle(servoTiltLR, 90);
      setServoAngle(servoTiltUD, 90);
      setServoAngle(servoHeadRotate, 90);
      controlEyelids(false);
      lastUpdateTime = millis();
    }
  }
  
  delay(10);  // Small delay to prevent busy loop
}

void initializeServos() {
  Serial.println("Initializing servos to neutral position...");
  
  // Center eyeball position
  setServoAngle(servoEyeballLR, 90);
  setServoAngle(servoEyeballUD, 90);
  
  // Open eyelids
  setServoAngle(servoEyelidLeft, servoLimits[2].max);   // OPEN
  setServoAngle(servoEyelidRight, servoLimits[3].max);  // OPEN
  
  // Center face tilt and head rotation
  setServoAngle(servoTiltLR, 90);
  setServoAngle(servoTiltUD, 90);
  setServoAngle(servoHeadRotate, 90);
  
  delay(500);
  Serial.println("Servos initialized");
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
   * Format: "ELR:90,EUD:90,TLR:90,TUD:90,HR:90,BL:0"
   * ELR = Eyeball Left/Right
   * EUD = Eyeball Up/Down
   * TLR = Tilt Left/Right
   * TUD = Tilt Up/Down
   * HR = Head Rotate
   * BL = Blink
   */
  
  command.trim();
  
  int eyeLR = 90;    // Default to center
  int eyeUD = 90;
  int tiltLR = 90;
  int tiltUD = 90;
  int headRotate = 90;
  bool blinkState = false;
  
  // Split by comma
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
      
      if (key == "ELR") {
        eyeLR = value.toInt();
      } else if (key == "EUD") {
        eyeUD = value.toInt();
      } else if (key == "TLR") {
        tiltLR = value.toInt();
      } else if (key == "TUD") {
        tiltUD = value.toInt();
      } else if (key == "HR") {
        headRotate = value.toInt();
      } else if (key == "BL") {
        blinkState = (value.toInt() != 0);
      }
    }
  }
  
  // Update position
  updatePosition(eyeLR, eyeUD, tiltLR, tiltUD, headRotate, blinkState);
}

void updatePosition(int eyeLR, int eyeUD, int tiltLR, int tiltUD, int headRotate, bool blinkState) {
  // Apply smoothing to eyeball movements (fast)
  float smoothEyeLR = smoothAngle((float)eyeLR, currentEyeballLR, smoothing);
  float smoothEyeUD = smoothAngle((float)eyeUD, currentEyeballUD, smoothing);
  
  // Apply smoothing to head movements (slower for more natural motion)
  float smoothTiltLR = smoothAngle((float)tiltLR, currentTiltLR, smoothingHead);
  float smoothTiltUD = smoothAngle((float)tiltUD, currentTiltUD, smoothingHead);
  float smoothHeadRot = smoothAngle((float)headRotate, currentHeadRotate, smoothingHead);
  
  // Clamp to servo limits
  smoothEyeLR = constrain(smoothEyeLR, servoLimits[0].min, servoLimits[0].max);
  smoothEyeUD = constrain(smoothEyeUD, servoLimits[1].min, servoLimits[1].max);
  smoothTiltLR = constrain(smoothTiltLR, servoLimits[4].min, servoLimits[4].max);
  smoothTiltUD = constrain(smoothTiltUD, servoLimits[5].min, servoLimits[5].max);
  smoothHeadRot = constrain(smoothHeadRot, servoLimits[6].min, servoLimits[6].max);
  
  // Update current positions
  currentEyeballLR = smoothEyeLR;
  currentEyeballUD = smoothEyeUD;
  currentTiltLR = smoothTiltLR;
  currentTiltUD = smoothTiltUD;
  currentHeadRotate = smoothHeadRot;
  
  // Handle blinking
  if (blinkState && !isBlinking) {
    // Start new blink
    isBlinking = true;
    blinkStartTime = millis();
    blinkEyes();
  }
  
  // Update servo positions
  setServoAngle(servoEyeballLR, (int)currentEyeballLR);
  setServoAngle(servoEyeballUD, (int)currentEyeballUD);
  setServoAngle(servoTiltLR, (int)currentTiltLR);
  setServoAngle(servoTiltUD, (int)currentTiltUD);
  setServoAngle(servoHeadRotate, (int)currentHeadRotate);
  
  // Update eyelids (if not blinking)
  if (!isBlinking) {
    controlEyelids(false);
  }
}

float smoothAngle(float target, float current, float smoothFactor) {
  // Apply exponential smoothing
  return current + (target - current) * smoothFactor;
}

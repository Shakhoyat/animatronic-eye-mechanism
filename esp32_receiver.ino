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

// Create Servo objects
Servo servoLR;  // Left/Right movement
Servo servoUD;  // Up/Down movement
Servo servoTL;  // Top Left eyelid
Servo servoBL;  // Bottom Left eyelid
Servo servoTR;  // Top Right eyelid
Servo servoBR;  // Bottom Right eyelid

// GPIO pins for servos (adjust these to your wiring)
#define PIN_LR  13  // Left/Right servo
#define PIN_UD  12  // Up/Down servo
#define PIN_TL  14  // Top Left eyelid
#define PIN_BL  27  // Bottom Left eyelid
#define PIN_TR  26  // Top Right eyelid
#define PIN_BR  25  // Bottom Right eyelid

// Servo limits (adjust these to match your hardware)
struct ServoLimits {
  int min;
  int max;
};

ServoLimits servoLimits[] = {
  {60, 120},   // LR: Left/Right movement range
  {60, 120},   // UD: Up/Down movement range
  {90, 10},    // TL: Top Left eyelid (inverted)
  {10, 90},    // BL: Bottom Left eyelid
  {10, 90},    // TR: Top Right eyelid
  {90, 10}     // BR: Bottom Right eyelid (inverted)
};

// Serial communication settings
#define SERIAL_BAUD 115200

// State variables
float currentLR = 90.0;
float currentUD = 90.0;
bool isBlinking = false;
unsigned long blinkStartTime = 0;
const int blinkDuration = 150;  // milliseconds

// Smoothing parameter (0.1 = smooth, 0.9 = responsive)
const float smoothing = 0.3;

// Timeout settings
unsigned long lastUpdateTime = 0;
const unsigned long timeoutDuration = 1000;  // 1 second

// Serial buffer
String inputBuffer = "";

// Function prototypes
void initializeServos();
void setServoAngle(Servo &servo, int angle);
void controlUDAndLids(int udAngle);
void blinkEyes();
void parseCommand(String command);
void updatePosition(int lrAngle, int udAngle, bool blinkState);
float smoothAngle(float target, float current);

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
  servoLR.attach(PIN_LR, 500, 2500);  // Min/max pulse width in microseconds
  servoUD.attach(PIN_UD, 500, 2500);
  servoTL.attach(PIN_TL, 500, 2500);
  servoBL.attach(PIN_BL, 500, 2500);
  servoTR.attach(PIN_TR, 500, 2500);
  servoBR.attach(PIN_BR, 500, 2500);
  
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
    if (currentLR != 90.0 || currentUD != 90.0) {
      Serial.println("Timeout: Returning to neutral position");
      currentLR = 90.0;
      currentUD = 90.0;
      setServoAngle(servoLR, 90);
      controlUDAndLids(90);
      lastUpdateTime = millis();
    }
  }
  
  delay(10);  // Small delay to prevent busy loop
}

void initializeServos() {
  Serial.println("Initializing servos to neutral position...");
  
  // Center eye position
  setServoAngle(servoLR, 90);
  setServoAngle(servoUD, 90);
  
  // Open eyelids
  setServoAngle(servoTL, servoLimits[2].max);  // OPEN
  setServoAngle(servoBL, servoLimits[3].max);  // OPEN
  setServoAngle(servoTR, servoLimits[4].max);  // OPEN
  setServoAngle(servoBR, servoLimits[5].max);  // OPEN
  
  delay(500);
  Serial.println("Servos initialized");
}

void setServoAngle(Servo &servo, int angle) {
  // Clamp angle to valid range (0-180)
  angle = constrain(angle, 0, 180);
  
  // Write angle directly to servo
  servo.write(angle);
}

void controlUDAndLids(int udAngle) {
  /*
   * Move UD servo and adjust eyelids based on vertical eye position
   * Eyelids partially close when looking up/down for realistic effect
   */
  
  // Get limits
  int udMin = servoLimits[1].min;
  int udMax = servoLimits[1].max;
  int tlMin = servoLimits[2].min;  // min=closed, max=open
  int tlMax = servoLimits[2].max;
  int trMin = servoLimits[4].min;
  int trMax = servoLimits[4].max;
  int blMin = servoLimits[3].min;
  int blMax = servoLimits[3].max;
  int brMin = servoLimits[5].min;
  int brMax = servoLimits[5].max;
  
  // Normalize UD position (0 = looking down, 1 = looking up)
  float udRange = udMax - udMin;
  float udProgress = 0.5;
  if (udRange > 0) {
    udProgress = (float)(udAngle - udMin) / udRange;
  }
  
  // Clamp to valid range
  udProgress = constrain(udProgress, 0.0, 1.0);
  
  // Calculate eyelid closing factors
  // When looking up, close top lids slightly
  // When looking down, close bottom lids slightly
  float topCloseFactor = 0.6 * (1.0 - udProgress);     // Higher when looking down
  float bottomCloseFactor = 0.6 * udProgress;          // Higher when looking up
  
  // Calculate target eyelid positions
  int tlTarget = tlMin + (tlMax - tlMin) * (1.0 - topCloseFactor);
  int trTarget = trMin + (trMax - trMin) * (1.0 - topCloseFactor);
  int blTarget = blMin + (blMax - blMin) * (1.0 - bottomCloseFactor);
  int brTarget = brMin + (brMax - brMin) * (1.0 - bottomCloseFactor);
  
  // Move servos
  setServoAngle(servoUD, udAngle);
  setServoAngle(servoTL, tlTarget);
  setServoAngle(servoTR, trTarget);
  setServoAngle(servoBL, blTarget);
  setServoAngle(servoBR, brTarget);
}

void blinkEyes() {
  // Close all eyelids
  setServoAngle(servoTL, servoLimits[2].min);  // CLOSED
  setServoAngle(servoBL, servoLimits[3].min);  // CLOSED
  setServoAngle(servoTR, servoLimits[4].min);  // CLOSED
  setServoAngle(servoBR, servoLimits[5].min);  // CLOSED
}

void parseCommand(String command) {
  /*
   * Parse serial command from PC
   * Format: "LR:90,UD:90,BL:0"
   */
  
  command.trim();
  
  int lrAngle = -1;
  int udAngle = -1;
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
      
      if (key == "LR") {
        lrAngle = value.toInt();
      } else if (key == "UD") {
        udAngle = value.toInt();
      } else if (key == "BL") {
        blinkState = (value.toInt() != 0);
      }
    }
  }
  
  // Update position if valid data received
  if (lrAngle >= 0 && udAngle >= 0) {
    updatePosition(lrAngle, udAngle, blinkState);
  }
}

void updatePosition(int lrAngle, int udAngle, bool blinkState) {
  // Apply smoothing to eye movements
  float smoothLR = smoothAngle((float)lrAngle, currentLR);
  float smoothUD = smoothAngle((float)udAngle, currentUD);
  
  // Clamp to servo limits
  smoothLR = constrain(smoothLR, servoLimits[0].min, servoLimits[0].max);
  smoothUD = constrain(smoothUD, servoLimits[1].min, servoLimits[1].max);
  
  // Update current positions
  currentLR = smoothLR;
  currentUD = smoothUD;
  
  // Handle blinking
  if (blinkState && !isBlinking) {
    // Start new blink
    isBlinking = true;
    blinkStartTime = millis();
    blinkEyes();
  }
  
  // Update servo positions (if not blinking)
  if (!isBlinking) {
    setServoAngle(servoLR, (int)currentLR);
    controlUDAndLids((int)currentUD);
  }
}

float smoothAngle(float target, float current) {
  // Apply exponential smoothing
  return current + (target - current) * smoothing;
}

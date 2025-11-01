/*
 * ESP32 Eye Tracking Receiver with ESP32Servo
 * Receives servo angle data via USB Serial in String format and controls servos
 */

#include <ESP32Servo.h>

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
#define STATUS_LED 23

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

  
  // Set all servos to neutral position
  servoLeftLid.write(50);
  servoRightLid.write(50);
  servoLeftEyeVertical.write(180);
  servoRightEyeVertical.write(180);
  servoLeftEyeHorizontal.write(90);
  servoRightEyeHorizontal.write(90);
  servoHeadRotation.write(90);

  
  // LED on to indicate ready
  //digitalWrite(STATUS_LED, HIGH);
}

int s1 = 90;   // Left eye horizontal
  int s2 = 90;   // Right eye horizontal
  int s3 = 50;   // Left eyelid
  int s4 = 50;   // Right eyelid
  int s5 = 180;  // Left eye vertical
  int s6 = 180;  // Right eye vertical
  int s7 = 90; 

// Parse serial data format: String
// Format: S1:90,S2:90,S3:50,S4:50,S5:180,S6:180,S7:90
void parseSerialData(String command) {
  command.trim();
  
  // Default values
    // Head rotation
  
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
        s1 = 90
      } else if (key == "S2") {
        s2 = 90
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
  
  // Map parsed values to servo variables
  leftEyeHorizontal = s1;    // S1: Left eye horizontal
  rightEyeHorizontal = s2;   // S2: Right eye horizontal
  leftLid = s3;              // S3: Left eyelid
  rightLid = s4;             // S4: Right eyelid
  leftEyeVertical = s5;      // S5: Left eye vertical
  rightEyeVertical = s6;     // S6: Right eye vertical
  headRotation = s7;         // S7: Head rotation
  
  // Write to all 7 servos
  servoLeftEyeHorizontal.write(leftEyeHorizontal);  // Servo 5: Left eye horizontal
  servoRightEyeHorizontal.write(rightEyeHorizontal); // Servo 6: Right eye horizontal
  servoLeftLid.write(leftLid);                      // Servo 1: Left eyelid
  servoRightLid.write(rightLid);                    // Servo 2: Right eyelid
  servoLeftEyeVertical.write(leftEyeVertical);      // Servo 3: Left eye vertical
  servoRightEyeVertical.write(rightEyeVertical);    // Servo 4: Right eye vertical
  servoHeadRotation.write(headRotation);            // Servo 7: Head rotation
  
  lastPacketTime = millis();
}

void loop() {
  // USB Serial Mode - Read from Serial
  if (Serial.available()) {
    digitalWrite(STATUS_LED, HIGH);  // Turn LED ON when receiving data
    
    while (Serial.available()) {
      char c = Serial.read();
      
      
      if (c == '\n') {
        // Process complete line
        if (serialBuffer.length() > 0) {
          parseSerialData(serialBuffer);
          serialBuffer = "";
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
  } else {
    // Turn LED OFF when no data is being received
    digitalWrite(STATUS_LED, LOW);
  }
  
  // Check for timeout (no data received)
  // if (millis() - lastPacketTime > TIMEOUT_MS && lastPacketTime > 0) {
  //   // Return all 7 servos to neutral position on timeout
  //   servoLeftLid.write(50);                   // Servo 1: Neutral
  //   servoRightLid.write(50);                  // Servo 2: Neutral
  //   servoLeftEyeVertical.write(180);          // Servo 3: Neutral
  //   servoRightEyeVertical.write(180);         // Servo 4: Neutral
  //   servoLeftEyeHorizontal.write(90);         // Servo 5: Neutral
  //   servoRightEyeHorizontal.write(90);        // Servo 6: Neutral
  //   servoHeadRotation.write(90);              // Servo 7: Neutral
    
  //   // Blink LED to indicate timeout
  //   digitalWrite(STATUS_LED, millis() % 1000 < 500);
  // }
  digitalWrite(STATUS_LED, LOW); 
  delay(1);  // Small delay to prevent watchdog reset
}

/*
 * Animatronic Eye Tracking Controller
 * ESP32 - Arduino IDE
 * 
 * SAFETY WARNING: Always test servo ranges manually before automation!
 * Use external 5V power supply for servos (2A minimum)
 * 
 * Hardware Requirements:
 * - ESP32 development board
 * - 7 servo motors (4 limited range eye servos + 3 full range base servos)
 * - External 5V power supply for servos
 * 
 * Pin Connections:
 * Pin 2  → Eye Servo 0 (LIMITED: 60°-120°)
 * Pin 4  → Eye Servo 1 (LIMITED: 60°-120°) 
 * Pin 5  → Eye Servo 2 (LIMITED: 70°-110°)
 * Pin 18 → Eye Servo 3 (LIMITED: 60°-120°)
 * Pin 19 → Base Pan (Full: 0°-180°)
 * Pin 21 → Base Tilt (Full: 0°-180°)
 * Pin 22 → Base Roll (Full: 0°-180°)
 */

#include <WiFi.h>
#include <ESP32Servo.h>
#include <ArduinoJson.h>

// ============= CONFIGURATION SECTION =============
// CHANGE THESE VALUES FOR YOUR SETUP

// WiFi Configuration - UPDATE THESE!
const char* ssid = "YOUR_WIFI_NAME";        // Replace with your WiFi name
const char* password = "YOUR_WIFI_PASSWORD"; // Replace with your WiFi password

// Server Configuration
const int serverPort = 8888;

// Hardware Configuration
const int NUM_SERVOS = 7;
const int servoPins[NUM_SERVOS] = {2, 4, 5, 18, 19, 21, 22};

// ============= SAFETY LIMITS STRUCTURE =============
struct ServoLimits {
  int minAngle;
  int maxAngle; 
  int centerAngle;
};

// CRITICAL: Adjust these limits to YOUR hardware's safe range!
// Test each servo manually before using these values!
const ServoLimits servoLimits[NUM_SERVOS] = {
  {60, 120, 90},   // Servo 0: Eye mechanism - CONSERVATIVE LIMITS
  {60, 120, 90},   // Servo 1: Eye mechanism - CONSERVATIVE LIMITS
  {70, 110, 90},   // Servo 2: Eye mechanism - VERY LIMITED (adjust as needed)
  {60, 120, 90},   // Servo 3: Eye mechanism - CONSERVATIVE LIMITS
  {0, 180, 90},    // Servo 4: Base Pan - Full rotation (usually safe)
  {0, 180, 90},    // Servo 5: Base Tilt - Full rotation (usually safe)
  {0, 180, 90}     // Servo 6: Base Roll - Full rotation (usually safe)
};

// ============= GLOBAL VARIABLES =============
Servo servos[NUM_SERVOS];
WiFiServer server(serverPort);
WiFiClient client;

bool emergencyStopActive = false;
unsigned long lastCommandTime = 0;
const unsigned long COMMAND_TIMEOUT = 5000; // 5 seconds

// Movement smoothing
int targetPositions[NUM_SERVOS];
int currentPositions[NUM_SERVOS];
const int MOVEMENT_SPEED = 2; // degrees per update cycle

// ============= SAFETY FUNCTIONS =============

void emergencyStop() {
  Serial.println("🚨 EMERGENCY STOP ACTIVATED!");
  emergencyStopActive = true;
  
  // Stop all servos immediately
  for (int i = 0; i < NUM_SERVOS; i++) {
    if (servos[i].attached()) {
      servos[i].detach();
    }
  }
  
  Serial.println("All servos detached. System halted for safety.");
  Serial.println("Reset ESP32 to resume operation after checking hardware.");
}

bool isAngleSafe(int servoIndex, int angle) {
  if (servoIndex < 0 || servoIndex >= NUM_SERVOS) return false;
  
  return (angle >= servoLimits[servoIndex].minAngle && 
          angle <= servoLimits[servoIndex].maxAngle);
}

int constrainToSafeRange(int servoIndex, int angle) {
  return constrain(angle, 
                   servoLimits[servoIndex].minAngle, 
                   servoLimits[servoIndex].maxAngle);
}

// ============= SERVO CONTROL FUNCTIONS =============

void initializeServos() {
  Serial.println("Initializing servos...");
  
  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].attach(servoPins[i]);
    
    // Set to center position safely
    int centerPos = servoLimits[i].centerAngle;
    servos[i].write(centerPos);
    currentPositions[i] = centerPos;
    targetPositions[i] = centerPos;
    
    Serial.printf("Servo %d: Pin %d, Center %d°, Range %d°-%d°\n", 
                  i, servoPins[i], centerPos, 
                  servoLimits[i].minAngle, servoLimits[i].maxAngle);
  }
  
  delay(1000); // Allow servos to reach center position
  Serial.println("✅ All servos initialized to center positions");
}

void updateServoPositions() {
  if (emergencyStopActive) return;
  
  for (int i = 0; i < NUM_SERVOS; i++) {
    if (currentPositions[i] != targetPositions[i]) {
      // Smooth movement towards target
      if (currentPositions[i] < targetPositions[i]) {
        currentPositions[i] = min(currentPositions[i] + MOVEMENT_SPEED, targetPositions[i]);
      } else {
        currentPositions[i] = max(currentPositions[i] - MOVEMENT_SPEED, targetPositions[i]);
      }
      
      servos[i].write(currentPositions[i]);
    }
  }
}

void setServoTarget(int servoIndex, int angle) {
  if (emergencyStopActive) return;
  
  if (servoIndex < 0 || servoIndex >= NUM_SERVOS) {
    Serial.printf("❌ Invalid servo index: %d\n", servoIndex);
    return;
  }
  
  int safeAngle = constrainToSafeRange(servoIndex, angle);
  
  if (safeAngle != angle) {
    Serial.printf("⚠️ Servo %d: Angle %d° limited to safe range %d°\n", 
                  servoIndex, angle, safeAngle);
  }
  
  targetPositions[servoIndex] = safeAngle;
}

// ============= TESTING FUNCTIONS =============

void testServos() {
  Serial.println("\n🔧 SERVO TESTING MODE");
  Serial.println("Watch each servo carefully! Press reset if any binding occurs!");
  delay(2000);
  
  for (int i = 0; i < NUM_SERVOS; i++) {
    Serial.printf("Testing Servo %d on pin %d...\n", i, servoPins[i]);
    
    // Test center position
    Serial.println("  → Center position");
    setServoTarget(i, servoLimits[i].centerAngle);
    delay(1000);
    updateServoPositions();
    delay(1000);
    
    // Test minimum position
    Serial.println("  → Minimum position");
    setServoTarget(i, servoLimits[i].minAngle);
    for (int j = 0; j < 100; j++) {
      updateServoPositions();
      delay(20);
    }
    delay(1000);
    
    // Test maximum position  
    Serial.println("  → Maximum position");
    setServoTarget(i, servoLimits[i].maxAngle);
    for (int j = 0; j < 100; j++) {
      updateServoPositions();
      delay(20);
    }
    delay(1000);
    
    // Return to center
    Serial.println("  → Back to center");
    setServoTarget(i, servoLimits[i].centerAngle);
    for (int j = 0; j < 100; j++) {
      updateServoPositions();
      delay(20);
    }
    
    Serial.printf("✅ Servo %d test complete\n\n", i);
    delay(2000);
  }
  
  Serial.println("🏁 All servo tests complete!");
  Serial.println("If all servos moved safely, uncomment the testServos() call");
}

// ============= COMMAND PROCESSING =============

void processEyeCommand(JsonObject& data) {
  if (emergencyStopActive) return;
  
  lastCommandTime = millis();
  
  // Extract gaze data with bounds checking
  float gazeX = constrain(data["x"] | 0.0, -1.0, 1.0);
  float gazeY = constrain(data["y"] | 0.0, -1.0, 1.0);
  bool blink = data["blink"] | false;
  
  // Calculate servo positions based on gaze
  // Eye mechanism servos (0-3) - CONSERVATIVE movement ranges
  int eyeServo0 = servoLimits[0].centerAngle + (gazeX * 15); // ±15° range
  int eyeServo1 = servoLimits[1].centerAngle + (gazeY * 15); // ±15° range  
  int eyeServo2 = servoLimits[2].centerAngle + (gazeX * 10); // ±10° range (most limited)
  int eyeServo3 = servoLimits[3].centerAngle + (gazeY * 15); // ±15° range
  
  // Base servos (4-6) - Larger movement ranges
  int basePan = servoLimits[4].centerAngle + (gazeX * 45);   // ±45° range
  int baseTilt = servoLimits[5].centerAngle + (gazeY * 45);  // ±45° range
  int baseRoll = servoLimits[6].centerAngle + (gazeX * 30);  // ±30° range
  
  // Handle blink behavior
  if (blink) {
    // Slight inward movement for blink effect
    eyeServo0 -= 5;
    eyeServo1 -= 5;
  }
  
  // Set all servo targets
  setServoTarget(0, eyeServo0);
  setServoTarget(1, eyeServo1);
  setServoTarget(2, eyeServo2);
  setServoTarget(3, eyeServo3);
  setServoTarget(4, basePan);
  setServoTarget(5, baseTilt);
  setServoTarget(6, baseRoll);
  
  // Debug output
  Serial.printf("Gaze: (%.2f, %.2f) | Blink: %s | Servos: %d,%d,%d,%d,%d,%d,%d\n",
                gazeX, gazeY, blink ? "YES" : "NO",
                eyeServo0, eyeServo1, eyeServo2, eyeServo3, basePan, baseTilt, baseRoll);
}

void processCommand(String command) {
  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, command);
  
  if (error) {
    Serial.println("❌ JSON parse error: " + String(error.c_str()));
    return;
  }
  
  String type = doc["type"] | "";
  
  if (type == "eye_movement") {
    processEyeCommand(doc.as<JsonObject>());
  } else if (type == "emergency_stop") {
    emergencyStop();
  } else if (type == "reset_center") {
    Serial.println("🎯 Resetting to center positions");
    for (int i = 0; i < NUM_SERVOS; i++) {
      setServoTarget(i, servoLimits[i].centerAngle);
    }
  } else {
    Serial.println("❌ Unknown command type: " + type);
  }
}

// ============= NETWORK FUNCTIONS =============

void setupWiFi() {
  Serial.println("🌐 Connecting to WiFi...");
  Serial.println("Network: " + String(ssid));
  
  WiFi.begin(ssid, password);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(1000);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✅ WiFi Connected!");
    Serial.println("IP Address: " + WiFi.localIP().toString());
    Serial.println("Server Port: " + String(serverPort));
    Serial.println("🔗 Use this IP address in your Python script!");
  } else {
    Serial.println("\n❌ WiFi Connection Failed!");
    Serial.println("Check your WiFi credentials and try again.");
    emergencyStop();
  }
}

void handleClient() {
  if (!client.connected()) {
    client = server.available();
    if (client) {
      Serial.println("🔗 Client connected: " + client.remoteIP().toString());
    }
  }
  
  if (client && client.available()) {
    String command = client.readStringUntil('\n');
    command.trim();
    
    if (command.length() > 0) {
      processCommand(command);
    }
  }
  
  // Check for command timeout
  if (millis() - lastCommandTime > COMMAND_TIMEOUT && lastCommandTime != 0) {
    Serial.println("⏰ Command timeout - returning to center");
    for (int i = 0; i < NUM_SERVOS; i++) {
      setServoTarget(i, servoLimits[i].centerAngle);
    }
    lastCommandTime = 0;
  }
}

// ============= MAIN FUNCTIONS =============

void setup() {
  Serial.begin(115200);
  Serial.println("\n🎭 Animatronic Eye Controller Starting...");
  Serial.println("Version 1.0 - Safety First Edition");
  
  // SAFETY CHECK: Uncomment the line below for initial servo testing
  // testServos(); // UNCOMMENT THIS FOR FIRST-TIME SETUP!
  
  // Initialize hardware
  initializeServos();
  
  // Connect to network
  setupWiFi();
  
  if (WiFi.status() == WL_CONNECTED) {
    server.begin();
    Serial.println("✅ Server started - Ready for eye tracking!");
    Serial.println("🎯 System initialized and waiting for commands...");
  }
}

void loop() {
  if (emergencyStopActive) {
    // Emergency stop - do nothing but blink LED
    digitalWrite(2, !digitalRead(2));
    delay(500);
    return;
  }
  
  // Handle network communication
  handleClient();
  
  // Update servo positions smoothly
  updateServoPositions();
  
  // Small delay for system stability
  delay(20);
}
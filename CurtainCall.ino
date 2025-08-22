#include <BTS7960.h>
#include "ZigbeeCore.h"
#include "ep/ZigbeeWindowCovering.h"
#include <AS5600.h>
#include <Wire.h>

// Pin definitions
#define ZIGBEE_ENDPOINT 10
#define MOTOR_L_EN 0
#define MOTOR_R_EN 7
#define MOTOR_L_PWM 6
#define MOTOR_R_PWM 5
#define SDA_PIN 21
#define SCL_PIN 22

// Motor settings
const int MOTOR_SPEED = 200;
const unsigned long TRAVEL_TIME_MS = 30000;  // 30 seconds for full travel
const int UPDATE_INTERVAL_MS = 100;
bool MOTOR_DIRECTION_INVERTED = true;  // Set to true if motor directions are backwards

// AS5600 settings
const unsigned long AS5600_READ_INTERVAL = 50;  // Read every 50ms for responsive control
const float DEGREES_PER_PERCENT = 18.0;  // 5 rotations (1800°) for 0-100%
const int ANGLE_FILTER_SIZE = 3;  // Moving average filter size

// Motor driver
BTS7960 motor(MOTOR_L_EN, MOTOR_R_EN, MOTOR_L_PWM, MOTOR_R_PWM);

// AS5600 magnetic encoder
AS5600 as5600;

// Position tracking
struct {
  uint8_t current = 100;
  uint8_t target = 100;
  bool isMoving = false;
  int direction = 0;  // 1=opening, -1=closing, 0=stopped
  unsigned long startTime = 0;
  float absoluteAngle = 0.0;  // Absolute angle accounting for multiple rotations
  bool calibrated = false;
} curtain;

// AS5600 tracking
struct {
  float lastAngle = 0.0;
  float currentAngle = 0.0;
  float filteredAngle = 0.0;
  float angleHistory[ANGLE_FILTER_SIZE] = {0};
  int historyIndex = 0;
  int rotationCount = 0;
  bool initialized = false;
  unsigned long lastReadTime = 0;
} encoder;

// Zigbee endpoint
ZigbeeWindowCovering zigbeeEndpoint(ZIGBEE_ENDPOINT);

void setup() {
  Serial.begin(115200);
  
  // Initialize motor
  motor.begin();
  motor.enable();
  motor.stop();

  // Initialize AS5600
  Wire.begin(SDA_PIN, SCL_PIN);
  as5600.begin();
  
  Serial.println("AS5600 Magnetic Encoder initialized");
  if (as5600.isConnected()) {
    Serial.println("AS5600 connected successfully");
  } else {
    Serial.println("AS5600 connection failed!");
  }

  // Configure Zigbee
  setupZigbee();
  
  // Connect to network
  connectToZigbee();
  
  // Initialize encoder with default position (100% - fully open)
  // The curtain.current is already set to 100 in the struct initialization
  curtain.target = curtain.current;
  initializeEncoder();
  
  // Set initial position
  zigbeeEndpoint.setLiftPercentage(curtain.current);
}

void loop() {
  // Read encoder frequently for position feedback
  if (millis() - encoder.lastReadTime >= AS5600_READ_INTERVAL) {
    updateEncoder();
    encoder.lastReadTime = millis();
  }
  
  updatePosition();
  
  delay(10);  // Shorter delay for more responsive control
}

void setupZigbee() {
  zigbeeEndpoint.setManufacturerAndModel("LC", "CurtainCall2");
  zigbeeEndpoint.setCoveringType(DRAPERY);
  zigbeeEndpoint.setConfigStatus(true, true, false, true, true, true, true);
  zigbeeEndpoint.setMode(false, true, false, false);
  zigbeeEndpoint.setLimits(0, 200, 0, 0);
  
  // Register callbacks
  zigbeeEndpoint.onOpen(openCurtain);
  zigbeeEndpoint.onClose(closeCurtain);
  zigbeeEndpoint.onGoToLiftPercentage(moveToPosition);
  zigbeeEndpoint.onStop(stopCurtain);
  
  Zigbee.addEndpoint(&zigbeeEndpoint);
}

void connectToZigbee() {
  Serial.println("Starting Zigbee...");
  if (!Zigbee.begin()) {
    Serial.println("Zigbee failed! Rebooting...");
    ESP.restart();
  }
  
  Serial.print("Connecting to network");
  while (!Zigbee.connected()) {
    Serial.print(".");
    delay(100);
  }
  Serial.println("\nConnected!");
}

void updatePosition() {
  if (!curtain.isMoving || !curtain.calibrated) return;
  
  // Calculate current position from absolute angle
  float currentPositionPercent = curtain.absoluteAngle / DEGREES_PER_PERCENT;
  currentPositionPercent = constrain(currentPositionPercent, 0.0, 100.0);
  
  uint8_t newPosition = (uint8_t)round(currentPositionPercent);
  
  // Update position if it changed
  if (newPosition != curtain.current) {
    curtain.current = newPosition;
    zigbeeEndpoint.setLiftPercentage(curtain.current);
    
    static unsigned long lastPositionPrint = 0;
    if (millis() - lastPositionPrint >= 1000) {  // Print every second
      Serial.printf("Position: %d%% (%.1f°)\n", curtain.current, curtain.absoluteAngle);
      lastPositionPrint = millis();
    }
  }
  
  // Check if target reached (within 1% tolerance)
  float targetAngle = curtain.target * DEGREES_PER_PERCENT;
  float angleDifference = abs(curtain.absoluteAngle - targetAngle);
  
  if (angleDifference <= DEGREES_PER_PERCENT) {  // Within 1% tolerance
    stopMotor();
    curtain.current = curtain.target;
    zigbeeEndpoint.setLiftPercentage(curtain.current);
    Serial.printf("Target reached: %d%% (%.1f°)\n", curtain.current, curtain.absoluteAngle);
    return;
  }
  
  // Safety: check for bounds violations
  if (curtain.absoluteAngle <= 0.0 && curtain.direction < 0) {
    stopMotor();
    curtain.current = 0;
    curtain.target = 0;
    zigbeeEndpoint.setLiftPercentage(0);
    Serial.println("Motor stopped: reached 0% limit");
  } else if (curtain.absoluteAngle >= 1800.0 && curtain.direction > 0) {
    stopMotor();
    curtain.current = 100;
    curtain.target = 100;
    zigbeeEndpoint.setLiftPercentage(100);
    Serial.println("Motor stopped: reached 100% limit");
  }
}

void startMotor(int direction) {
  motor.pwm = MOTOR_SPEED;
  
  // Apply direction inversion if needed
  int motorDirection = MOTOR_DIRECTION_INVERTED ? -direction : direction;
  
  if (motorDirection > 0) {
    motor.front();
    Serial.printf("Motor: Forward (target: %s angle)\n", direction > 0 ? "increase" : "decrease");
  } else if (motorDirection < 0) {
    motor.back();
    Serial.printf("Motor: Reverse (target: %s angle)\n", direction > 0 ? "increase" : "decrease");
  }
  
  curtain.isMoving = true;
  curtain.direction = direction;  // Keep original direction for logic
  curtain.startTime = millis();
}

void stopMotor() {
  motor.stop();
  curtain.isMoving = false;
  curtain.direction = 0;
  Serial.println("Motor stopped");
}

void moveToPosition(uint8_t targetPercent) {
  stopMotor();
  
  if (!curtain.calibrated) {
    Serial.println("Encoder not calibrated! Cannot move to position.");
    return;
  }
  
  curtain.target = constrain(targetPercent, 0, 100);
  
  if (abs(curtain.target - curtain.current) <= 1) {  // Within 1% tolerance
    Serial.printf("Already at %d%%\n", curtain.current);
    return;
  }
  
  float targetAngle = curtain.target * DEGREES_PER_PERCENT;
  float currentAngle = curtain.absoluteAngle;
  
  Serial.printf("Moving from %d%% to %d%% (%.1f° to %.1f°)\n", 
    curtain.current, curtain.target, currentAngle, targetAngle);
  
  // Determine direction based on angle difference
  // Positive direction = increasing angle (opening)
  // Negative direction = decreasing angle (closing)
  int direction;
  if (targetAngle > currentAngle) {
    direction = 1;  // Need to increase angle (open)
    Serial.println("Direction: Opening (increasing angle)");
  } else {
    direction = -1; // Need to decrease angle (close)
    Serial.println("Direction: Closing (decreasing angle)");
  }
  
  startMotor(direction);
}

// Zigbee callbacks
void openCurtain() {
  Serial.println("Open command received");
  moveToPosition(100);
}

void closeCurtain() {
  Serial.println("Close command received");
  moveToPosition(0);
}

void stopCurtain() {
  Serial.println("Stop command received");
  stopMotor();
  zigbeeEndpoint.setLiftPercentage(curtain.current);
}

void initializeEncoder() {
  if (!as5600.isConnected()) {
    Serial.println("AS5600: Not connected - cannot initialize encoder");
    return;
  }
  
  // Read initial angle
  uint16_t rawAngle = as5600.readAngle();
  encoder.currentAngle = rawAngle * AS5600_RAW_TO_DEGREES;
  encoder.lastAngle = encoder.currentAngle;
  encoder.filteredAngle = encoder.currentAngle;
  
  // Initialize filter history
  for (int i = 0; i < ANGLE_FILTER_SIZE; i++) {
    encoder.angleHistory[i] = encoder.currentAngle;
  }
  
  encoder.rotationCount = 0;
  encoder.initialized = true;
  
  // Auto-calibrate using stored Zigbee position and current encoder reading
  // Calculate how many rotations are needed to reach the stored position
  float targetAbsoluteAngle = curtain.current * DEGREES_PER_PERCENT;
  
  // Find the closest rotation count that puts us near the target
  encoder.rotationCount = round((targetAbsoluteAngle - encoder.currentAngle) / 360.0);
  curtain.absoluteAngle = encoder.rotationCount * 360.0 + encoder.currentAngle;
  curtain.calibrated = true;
  
  Serial.printf("Encoder auto-calibrated: Raw=%.1f°, Rotations=%d, Absolute=%.1f°, Position=%d%%\n", 
    encoder.currentAngle, encoder.rotationCount, curtain.absoluteAngle, curtain.current);
}

void updateEncoder() {
  if (!as5600.isConnected() || !encoder.initialized) {
    return;
  }
  
  // Read current angle
  uint16_t rawAngle = as5600.readAngle();
  encoder.currentAngle = rawAngle * AS5600_RAW_TO_DEGREES;
  
  // Detect rotation rollover (360° -> 0° or 0° -> 360°)
  float angleDifference = encoder.currentAngle - encoder.lastAngle;
  
  if (angleDifference > 180.0) {
    // Crossed from 360° to 0° (counter-clockwise)
    encoder.rotationCount--;
  } else if (angleDifference < -180.0) {
    // Crossed from 0° to 360° (clockwise)
    encoder.rotationCount++;
  }
  
  // Apply moving average filter
  encoder.angleHistory[encoder.historyIndex] = encoder.currentAngle;
  encoder.historyIndex = (encoder.historyIndex + 1) % ANGLE_FILTER_SIZE;
  
  float sum = 0.0;
  for (int i = 0; i < ANGLE_FILTER_SIZE; i++) {
    sum += encoder.angleHistory[i];
  }
  encoder.filteredAngle = sum / ANGLE_FILTER_SIZE;
  
  // Calculate absolute angle (accounting for multiple rotations)
  float rawAbsoluteAngle = encoder.rotationCount * 360.0 + encoder.filteredAngle;
  
  // Calculate absolute angle but don't auto-stop here to avoid conflicts
  curtain.absoluteAngle = rawAbsoluteAngle;
  
  // Simple bounds enforcement - constrain but don't change motor state here
  if (curtain.absoluteAngle > 1800.0) {
    curtain.absoluteAngle = 1800.0;
  } else if (curtain.absoluteAngle < 0.0) {
    curtain.absoluteAngle = 0.0;
  }
  
  encoder.lastAngle = encoder.currentAngle;
  
  // Debug output every 2 seconds
  static unsigned long lastDebugPrint = 0;
  if (millis() - lastDebugPrint >= 2000) {
    Serial.printf("Encoder: %.1f° (Raw: %.1f°, Rotations: %d)\n", 
      curtain.absoluteAngle, encoder.filteredAngle, encoder.rotationCount);
    lastDebugPrint = millis();
  }
}

void calibratePosition(uint8_t knownPosition) {
  // Call this function to set current position as reference point
  if (!encoder.initialized) {
    Serial.println("Encoder not initialized!");
    return;
  }
  
  stopMotor();  // Stop any movement first
  
  // Reset encoder tracking
  encoder.rotationCount = 0;
  curtain.absoluteAngle = encoder.filteredAngle;
  
  // Set known position
  curtain.current = constrain(knownPosition, 0, 100);
  curtain.target = curtain.current;
  curtain.calibrated = true;
  
  Serial.printf("Position calibrated: %d%% = %.1f° (Raw encoder: %.1f°)\n", 
    curtain.current, curtain.absoluteAngle, encoder.filteredAngle);
}

void calibratePosition() {
  // Default calibration - assumes current position is 0% (closed)
  calibratePosition(0);
}

void emergencyStop() {
  // Emergency stop function - call this if something goes wrong
  stopMotor();
  Serial.println("EMERGENCY STOP ACTIVATED");
  
  // Reset encoder state to prevent further issues
  encoder.rotationCount = 0;
  curtain.absoluteAngle = curtain.current * DEGREES_PER_PERCENT;
  
  Serial.printf("System reset to current position: %d%%\n", curtain.current);
}
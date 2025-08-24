/*
 * CurtainCall - Smart Curtain Controller
 * 
 * Motorized curtain control system with Zigbee connectivity
 * Features precision position tracking using AS5600 magnetic encoder
 * 
 * Hardware:
 * - ESP32 microcontroller
 * - BTS7960 motor driver
 * - AS5600 magnetic rotary encoder
 * - DC motor with magnetic coupling
 * 
 * Author: CurtainCall Project
 * Version: 2.0
 */

#include <BTS7960.h>
#include "ZigbeeCore.h"
#include "ep/ZigbeeWindowCovering.h"
#include <AS5600.h>
#include <Wire.h>

// Hardware pin assignments
#define ZIGBEE_ENDPOINT 10
#define MOTOR_L_EN 0
#define MOTOR_R_EN 7
#define MOTOR_L_PWM 6
#define MOTOR_R_PWM 5
#define SDA_PIN 21
#define SCL_PIN 22

// System configuration
const int MOTOR_SPEED = 200;
const bool MOTOR_DIRECTION_INVERTED = true;
const unsigned long AS5600_READ_INTERVAL = 20;  // 20ms for I2C stability
const float TOTAL_ROTATIONS_0_TO_100 = 5.0;    // Configure for your curtain travel
const float DEGREES_PER_PERCENT = TOTAL_ROTATIONS_0_TO_100 * 360.0 / 100.0;
const float MAX_ANGLE_CHANGE_PER_MS = 15.0;    // Spike detection threshold

// Hardware interfaces
BTS7960 motor(MOTOR_L_EN, MOTOR_R_EN, MOTOR_L_PWM, MOTOR_R_PWM);
AS5600 as5600;
ZigbeeWindowCovering zigbeeEndpoint(ZIGBEE_ENDPOINT);

// System state
struct {
  uint8_t current = 100;      // Current position (0-100%)
  uint8_t target = 100;       // Target position (0-100%)
  bool isMoving = false;      // Movement state
  int direction = 0;          // 1=opening, -1=closing, 0=stopped
  float absoluteAngle = 0.0;  // Absolute angle from 0% position
  bool calibrated = false;    // Calibration status
} curtain;

// Encoder state
struct {
  float lastAngle = 0.0;
  float currentAngle = 0.0;
  float validAngle = 0.0;     // Last validated reading
  int rotationCount = 0;      // Track multiple rotations
  bool initialized = false;
  unsigned long lastReadTime = 0;
} encoder;

void setup() {
  // Initialize hardware
  initializeMotor();
  initializeEncoder();
  initializeZigbee();
  
  // Set initial position
  zigbeeEndpoint.setLiftPercentage(curtain.current);
}

void loop() {
  // Update encoder reading
  if (millis() - encoder.lastReadTime >= AS5600_READ_INTERVAL) {
    updateEncoder();
  }
  
  // Update position and motor control
  updatePosition();
}

void initializeMotor() {
  motor.begin();
  motor.enable();
  motor.stop();
}

void initializeEncoder() {
  // Configure I2C for reliability
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);  // 100kHz for stability
  Wire.setTimeout(1000);
  
  // Initialize AS5600
  as5600.begin();
  
  if (!as5600.isConnected()) {
    return;  // Continue without encoder if not available
  }
  
  // Read initial position
  uint16_t rawAngle = as5600.readAngle();
  encoder.currentAngle = rawAngle * AS5600_RAW_TO_DEGREES;
  encoder.lastAngle = encoder.currentAngle;
  encoder.validAngle = encoder.currentAngle;
  encoder.rotationCount = 0;
  encoder.initialized = true;
  
  // Auto-calibrate based on stored position
  float targetAbsoluteAngle = curtain.current * DEGREES_PER_PERCENT;
  encoder.rotationCount = round((targetAbsoluteAngle - encoder.currentAngle) / 360.0);
  curtain.absoluteAngle = encoder.rotationCount * 360.0 + encoder.validAngle;
  curtain.calibrated = true;
}

void initializeZigbee() {
  // Configure device properties
  zigbeeEndpoint.setManufacturerAndModel("LC", "CurtainCall2");
  zigbeeEndpoint.setCoveringType(DRAPERY);
  zigbeeEndpoint.setConfigStatus(true, true, false, true, true, true, true);
  zigbeeEndpoint.setMode(false, true, false, false);
  zigbeeEndpoint.setLimits(0, 200, 0, 0);
  
  // Register command callbacks
  zigbeeEndpoint.onOpen(openCurtain);
  zigbeeEndpoint.onClose(closeCurtain);
  zigbeeEndpoint.onGoToLiftPercentage(moveToPosition);
  zigbeeEndpoint.onStop(stopCurtain);
  
  Zigbee.addEndpoint(&zigbeeEndpoint);
  
  // Connect to network
  if (!Zigbee.begin()) {
    ESP.restart();  // Restart if Zigbee fails
  }
  
  while (!Zigbee.connected()) {
    delay(100);
  }
}

void updateEncoder() {
  if (!as5600.isConnected() || !encoder.initialized) {
    return;
  }
  
  // Read with retry logic for reliability
  uint16_t rawAngle = 0;
  bool readSuccess = false;
  
  for (int attempt = 0; attempt < 3 && !readSuccess; attempt++) {
    if (attempt > 0) delay(5);
    
    unsigned long startTime = micros();
    if (as5600.isConnected()) {
      rawAngle = as5600.readAngle();
      if ((micros() - startTime) < 20000) {  // Valid if < 20ms
        readSuccess = true;
      }
    }
  }
  
  // Handle failed reads gracefully
  if (!readSuccess) {
    static unsigned long consecutiveErrors = 0;
    consecutiveErrors++;
    
    // Attempt I2C recovery after multiple failures
    if (consecutiveErrors >= 5) {
      Wire.end();
      delay(50);
      Wire.begin(SDA_PIN, SCL_PIN);
      Wire.setClock(100000);
      Wire.setTimeout(1000);
      delay(50);
      consecutiveErrors = 0;
    }
    
    encoder.currentAngle = encoder.validAngle;
    encoder.lastReadTime = millis();
    return;
  }
  
  encoder.currentAngle = rawAngle * AS5600_RAW_TO_DEGREES;
  
  // Spike detection and filtering
  unsigned long currentTime = millis();
  unsigned long deltaTime = max(1UL, currentTime - encoder.lastReadTime);
  
  float angleDifference = encoder.currentAngle - encoder.lastAngle;
  
  // Handle 360° rollover
  if (angleDifference > 180.0) angleDifference -= 360.0;
  else if (angleDifference < -180.0) angleDifference += 360.0;
  
  // Check for unreasonable changes (spikes)
  float maxChange = MAX_ANGLE_CHANGE_PER_MS * deltaTime;
  if (curtain.isMoving) maxChange *= 2.0;  // More lenient when moving
  
  bool isValidReading = (encoder.lastReadTime == 0) || 
                       (abs(angleDifference) <= maxChange) || 
                       (abs(angleDifference) >= 300.0);  // Allow rollovers
  
  if (isValidReading) {
    encoder.validAngle = encoder.currentAngle;
  } else {
    encoder.currentAngle = encoder.validAngle;  // Use last valid reading
  }
  
  // Track rotation count for absolute positioning
  float validAngleDiff = encoder.validAngle - encoder.lastAngle;
  if (validAngleDiff > 180.0) encoder.rotationCount--;
  else if (validAngleDiff < -180.0) encoder.rotationCount++;
  
  // Calculate absolute position
  float rawAbsoluteAngle = encoder.rotationCount * 360.0 + encoder.validAngle;
  curtain.absoluteAngle = constrain(rawAbsoluteAngle, 0.0, 1800.0);
  
  encoder.lastAngle = encoder.validAngle;
  encoder.lastReadTime = currentTime;
}

void updatePosition() {
  if (!curtain.isMoving || !curtain.calibrated) return;
  
  // Calculate current position percentage
  float currentPercent = curtain.absoluteAngle / DEGREES_PER_PERCENT;
  currentPercent = constrain(currentPercent, 0.0, 100.0);
  uint8_t newPosition = (uint8_t)round(currentPercent);
  
  // Update Zigbee when position changes
  if (newPosition != curtain.current) {
    curtain.current = newPosition;
    zigbeeEndpoint.setLiftPercentage(curtain.current);
  }
  
  // Check if target reached
  float targetAngle = curtain.target * DEGREES_PER_PERCENT;
  if (abs(curtain.absoluteAngle - targetAngle) <= DEGREES_PER_PERCENT) {
    stopMotor();
    curtain.current = curtain.target;
    zigbeeEndpoint.setLiftPercentage(curtain.current);
    return;
  }
  
  // Safety limits
  if ((curtain.absoluteAngle <= 0.0 && curtain.direction < 0) ||
      (curtain.absoluteAngle >= 1800.0 && curtain.direction > 0)) {
    stopMotor();
    curtain.current = constrain(round(curtain.absoluteAngle / DEGREES_PER_PERCENT), 0, 100);
    curtain.target = curtain.current;
    zigbeeEndpoint.setLiftPercentage(curtain.current);
  }
}

void startMotor(int direction) {
  motor.pwm = MOTOR_SPEED;
  
  int motorDirection = MOTOR_DIRECTION_INVERTED ? -direction : direction;
  
  if (motorDirection > 0) motor.front();
  else if (motorDirection < 0) motor.back();
  
  curtain.isMoving = true;
  curtain.direction = direction;
}

void stopMotor() {
  motor.stop();
  curtain.isMoving = false;
  curtain.direction = 0;
}

void moveToPosition(uint8_t targetPercent) {
  if (!curtain.calibrated) return;
  
  stopMotor();
  curtain.target = constrain(targetPercent, 0, 100);
  
  if (abs(curtain.target - curtain.current) <= 1) return;  // Already there
  
  float targetAngle = curtain.target * DEGREES_PER_PERCENT;
  int direction = (targetAngle > curtain.absoluteAngle) ? 1 : -1;
  
  startMotor(direction);
}

// Zigbee command callbacks
void openCurtain() {
  moveToPosition(100);
}

void closeCurtain() {
  moveToPosition(0);
}

void stopCurtain() {
  stopMotor();
  zigbeeEndpoint.setLiftPercentage(curtain.current);
}

void calibratePosition(uint8_t knownPosition) {
  if (!encoder.initialized) return;
  
  stopMotor();
  encoder.rotationCount = 0;
  curtain.absoluteAngle = encoder.validAngle;
  curtain.current = constrain(knownPosition, 0, 100);
  curtain.target = curtain.current;
  curtain.calibrated = true;
}

void emergencyStop() {
  stopMotor();
  encoder.rotationCount = 0;
  curtain.absoluteAngle = curtain.current * DEGREES_PER_PERCENT;
}
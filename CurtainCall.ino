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
const unsigned long AS5600_READ_INTERVAL = 20;  // 20ms between reads for maximum I2C stability
const float TOTAL_ROTATIONS_0_TO_100 = 5.0;  // Total rotations from 0% to 100%
const float DEGREES_PER_PERCENT = TOTAL_ROTATIONS_0_TO_100 * 360.0 / 100.0;  // Calculated: 18° per percent
const float MAX_ANGLE_CHANGE_PER_MS = 15.0;  // Maximum degrees change per millisecond (spike threshold)
// At 40 RPM max speed = 0.24°/ms, but allowing for 10ms intervals and fast movements

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
  float validAngle = 0.0;  // Last known good angle
  int rotationCount = 0;
  bool initialized = false;
  unsigned long lastReadTime = 0;
  unsigned long spikeCount = 0;  // Track rejected spikes for debugging
} encoder;

// Zigbee endpoint
ZigbeeWindowCovering zigbeeEndpoint(ZIGBEE_ENDPOINT);

void setup() {
  Serial.begin(115200);
  
  // Initialize motor
  motor.begin();
  motor.enable();
  motor.stop();

  // Initialize AS5600 with conservative I2C settings
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);  // Set I2C clock to 100kHz (Standard Mode for stability)
  Wire.setTimeout(1000);  // Set 1 second timeout
  as5600.begin();
  
  Serial.println("AS5600 Magnetic Encoder initialized");
  
  // Diagnostic: Test I2C bus
  Serial.println("=== I2C Diagnostic ===");
  Serial.printf("SDA Pin: %d, SCL Pin: %d\n", SDA_PIN, SCL_PIN);
  Serial.printf("I2C Clock: 100kHz\n");
  
  // Test AS5600 connection multiple times
  int connectionAttempts = 5;
  int successCount = 0;
  for (int i = 0; i < connectionAttempts; i++) {
    if (as5600.isConnected()) {
      successCount++;
    }
    delay(100);
  }
  
  Serial.printf("AS5600 connection test: %d/%d successful\n", successCount, connectionAttempts);
  
  if (successCount >= 3) {
    Serial.println("AS5600 connected successfully");
  } else {
    Serial.println("AS5600 connection UNSTABLE - CHECK WIRING!");
    Serial.println("Common issues:");
    Serial.println("1. Missing 4.7k pull-up resistors on SDA/SCL");
    Serial.println("2. Loose connections");
    Serial.println("3. Wrong pin assignments");
    Serial.println("4. Power supply issues");
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
  // Read encoder as fast as possible for maximum responsiveness
  if (millis() - encoder.lastReadTime >= AS5600_READ_INTERVAL) {
    updateEncoder();
    // lastReadTime is set inside updateEncoder() for more accurate timing
  }
  
  updatePosition();
  
  // No artificial delay - run at maximum speed
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
  encoder.validAngle = encoder.currentAngle;
  
  encoder.rotationCount = 0;
  encoder.initialized = true;
  
  // Auto-calibrate using stored Zigbee position and current encoder reading
  // Calculate how many rotations are needed to reach the stored position
  float targetAbsoluteAngle = curtain.current * DEGREES_PER_PERCENT;
  
  // Find the closest rotation count that puts us near the target
  encoder.rotationCount = round((targetAbsoluteAngle - encoder.currentAngle) / 360.0);
  curtain.absoluteAngle = encoder.rotationCount * 360.0 + encoder.validAngle;
  curtain.calibrated = true;
  
  Serial.printf("Encoder auto-calibrated: Raw=%.1f°, Rotations=%d, Absolute=%.1f°, Position=%d%%\n", 
    encoder.currentAngle, encoder.rotationCount, curtain.absoluteAngle, curtain.current);
}

void updateEncoder() {
  if (!as5600.isConnected() || !encoder.initialized) {
    return;
  }
  
  // Performance monitoring
  static unsigned long maxReadTime = 0;
  static unsigned long errorCount = 0;
  unsigned long startTime = micros();
  
  // Read current angle with multiple retry attempts
  uint16_t rawAngle = 0;
  bool readSuccess = false;
  
  for (int attempt = 0; attempt < 3 && !readSuccess; attempt++) {
    if (attempt > 0) {
      delay(5);  // Brief delay between retries
    }
    
    unsigned long attemptStart = micros();
    
    // Try to read the angle
    if (as5600.isConnected()) {
      rawAngle = as5600.readAngle();
      unsigned long attemptTime = micros() - attemptStart;
      
      // Consider read successful if it completes reasonably quickly
      if (attemptTime < 20000) {  // Less than 20ms
        readSuccess = true;
        if (attemptTime > maxReadTime) {
          maxReadTime = attemptTime;
        }
      }
    }
  }
  
  // If all attempts failed, handle gracefully
  if (!readSuccess) {
    errorCount++;
    
    // Attempt I2C bus recovery every 5 consecutive failures
    static unsigned long consecutiveErrors = 0;
    consecutiveErrors++;
    
    if (consecutiveErrors >= 5) {
      Serial.printf("I2C recovery: %lu consecutive failures\n", consecutiveErrors);
      // Re-initialize I2C bus
      Wire.end();
      delay(50);  // Longer delay for recovery
      Wire.begin(SDA_PIN, SCL_PIN);
      Wire.setClock(100000);
      Wire.setTimeout(1000);
      delay(50);
      consecutiveErrors = 0;  // Reset counter after recovery attempt
    }
    
    // Use last known good angle to continue operation
    encoder.currentAngle = encoder.validAngle;
    encoder.lastReadTime = millis();
    return;  // Skip this reading
  } else {
    // Reset consecutive error counter on successful read
    static unsigned long consecutiveErrors = 0;
    consecutiveErrors = 0;
  }
  
  encoder.currentAngle = rawAngle * AS5600_RAW_TO_DEGREES;
  
  // Calculate time since last reading for spike detection
  unsigned long currentTime = millis();
  unsigned long deltaTime = currentTime - encoder.lastReadTime;
  if (deltaTime == 0) deltaTime = 1;  // Prevent division by zero
  
  // Spike detection - check if angle change is reasonable
  float angleDifference = encoder.currentAngle - encoder.lastAngle;
  
  // Handle rollover for spike detection (360° -> 0° or 0° -> 360°)
  if (angleDifference > 180.0) {
    angleDifference -= 360.0;
  } else if (angleDifference < -180.0) {
    angleDifference += 360.0;
  }
  
  // Calculate maximum allowed change based on time elapsed
  float maxAllowedChange = MAX_ANGLE_CHANGE_PER_MS * deltaTime;
  
  // Check for spike - but only reject if it's truly unreasonable
  bool isSpike = false;
  
  if (encoder.lastReadTime > 0) {
    // More lenient spike detection when motor is moving
    float adjustedThreshold = maxAllowedChange;
    if (curtain.isMoving) {
      adjustedThreshold *= 2.0;  // Double threshold when motor is active
    }
    
    // Only reject if change is extreme AND not a simple rollover
    if (abs(angleDifference) > adjustedThreshold && abs(angleDifference) < 300.0) {
      isSpike = true;
    }
  }
  
  if (isSpike) {
    // Spike detected - reject this reading
    encoder.spikeCount++;
    encoder.currentAngle = encoder.validAngle;  // Use last valid angle
    
    // Debug spike detection (less frequent)
    static unsigned long lastSpikeReport = 0;
    if (currentTime - lastSpikeReport >= 2000) {  // Report every 2 seconds
      Serial.printf("Spike rejected: %.1f° change in %lums (max: %.1f°)\n", 
        abs(angleDifference), deltaTime, maxAllowedChange);
      lastSpikeReport = currentTime;
    }
  } else {
    // Valid reading - update valid angle
    encoder.validAngle = encoder.currentAngle;
  }
  
  // Detect rotation rollover using valid angle
  float validAngleDifference = encoder.validAngle - encoder.lastAngle;
  
  if (validAngleDifference > 180.0) {
    // Crossed from 360° to 0° (counter-clockwise)
    encoder.rotationCount--;
  } else if (validAngleDifference < -180.0) {
    // Crossed from 0° to 360° (clockwise)
    encoder.rotationCount++;
  }
  
  // Calculate absolute angle using valid angle
  float rawAbsoluteAngle = encoder.rotationCount * 360.0 + encoder.validAngle;
  
  // Apply bounds enforcement for display/position calculation only
  // Keep the raw calculation for internal tracking
  curtain.absoluteAngle = constrain(rawAbsoluteAngle, 0.0, 1800.0);
  
  encoder.lastAngle = encoder.validAngle;
  encoder.lastReadTime = currentTime;
  
  // Debug output every 2 seconds with polling rate info
  static unsigned long lastDebugPrint = 0;
  static unsigned long readCount = 0;
  readCount++;
  
  if (millis() - lastDebugPrint >= 2000) {
    float actualHz = readCount / 2.0;  // Reads per second
    float rawAbsoluteAngle = encoder.rotationCount * 360.0 + encoder.validAngle;
    Serial.printf("Encoder: %.1f° (Raw: %.1f°, Valid: %.1f°, Rotations: %d) Polling: %.1fHz, Max read: %luμs, Spikes: %lu, I2C errors: %lu\n", 
      curtain.absoluteAngle, rawAbsoluteAngle, encoder.validAngle, encoder.rotationCount, actualHz, maxReadTime, encoder.spikeCount, errorCount);
    maxReadTime = 0;  // Reset for next measurement period
    encoder.spikeCount = 0;  // Reset spike count
    errorCount = 0;  // Reset error count
    readCount = 0;  // Reset read count
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
  curtain.absoluteAngle = encoder.validAngle;
  
  // Set known position
  curtain.current = constrain(knownPosition, 0, 100);
  curtain.target = curtain.current;
  curtain.calibrated = true;
  
  Serial.printf("Position calibrated: %d%% = %.1f° (Raw encoder: %.1f°)\n", 
    curtain.current, curtain.absoluteAngle, encoder.validAngle);
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
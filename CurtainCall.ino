#include <BTS7960.h>
#include "ZigbeeCore.h"
#include "ep/ZigbeeWindowCovering.h"
#include <AS5600.h>

// Configuration
#define ZIGBEE_ENDPOINT 10
#define MOTOR_L_EN 0
#define MOTOR_R_EN 7
#define MOTOR_L_PWM 6
#define MOTOR_R_PWM 5
#define AS5600_SDA 21  // I2C SDA pin
#define AS5600_SCL 22  // I2C SCL pin

// Motor settings
const int MOTOR_SPEED = 200;
const int UPDATE_INTERVAL_MS = 100;

// Motor driver
BTS7960 motor(MOTOR_L_EN, MOTOR_R_EN, MOTOR_L_PWM, MOTOR_R_PWM);

// AS5600 rotation sensor
AS5600 as5600;

// Position tracking
struct {
  uint8_t current = 100;
  uint8_t target = 100;
  bool isMoving = false;
  int direction = 0;  // 1=opening, -1=closing, 0=stopped
  float startAngle = 0;
  float currentAngle = 0;
  float targetAngle = 0;
  float totalRotation = 0;  // Total rotation range in degrees
} curtain;

// Zigbee endpoint
ZigbeeWindowCovering zigbeeEndpoint(ZIGBEE_ENDPOINT);

void setup() {
  Serial.begin(115200);
  
  // Initialize I2C for AS5600
  Wire.begin(AS5600_SDA, AS5600_SCL);
  
  // Initialize AS5600
  Serial.println("[DEBUG] Initializing AS5600 sensor...");
  if (!as5600.begin()) {
    Serial.println("[ERROR] AS5600 not found! Check wiring.");
    Serial.println("[DEBUG] Expected I2C address: 0x36");
    Serial.println("[DEBUG] Check SDA (GPIO 21), SCL (GPIO 22), and 3.3V power connections");
    while (1) delay(1000);
  }
  
  Serial.println("[DEBUG] AS5600 sensor found successfully!");
  
  // Configure AS5600
  as5600.setDirection(AS5600_CLOCK_WISE);
  Serial.println("[DEBUG] AS5600 configured for clockwise direction");
  
  // Test sensor reading
  float testAngle = as5600.readAngle();
  Serial.printf("[DEBUG] Initial sensor reading: %.2f°\n", testAngle);
  
  // Initialize motor
  motor.begin();
  motor.enable();
  motor.stop();

  // Configure Zigbee
  setupZigbee();
  
  // Connect to network
  connectToZigbee();
  
  // Initialize rotation tracking
  curtain.currentAngle = as5600.readAngle();
  curtain.startAngle = curtain.currentAngle;
  curtain.totalRotation = 360.0;  // Full rotation range, adjust as needed
  
  Serial.printf("[DEBUG] Initial rotation state:\n");
  Serial.printf("  - Current angle: %.2f°\n", curtain.currentAngle);
  Serial.printf("  - Start angle: %.2f°\n", curtain.startAngle);
  Serial.printf("  - Total rotation range: %.2f°\n", curtain.totalRotation);
  Serial.printf("  - Initial position: %d%%\n", curtain.current);
  
  // Set initial position
  zigbeeEndpoint.setLiftPercentage(curtain.current);
  
  Serial.println("[INFO] CurtainCall initialized with AS5600 rotation sensor");
  Serial.println("[DEBUG] Ready for operation. Send commands via Zigbee or Serial.");
}

void loop() {
  updatePosition();
  delay(50);
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
  if (!curtain.isMoving) return;
  
  float currentAngle = as5600.readAngle();
  float deltaAngle = currentAngle - curtain.currentAngle;
  
  // Debug: Print raw sensor data
  Serial.printf("[DEBUG] Raw angle: %.2f°, Previous: %.2f°, Delta: %.2f°\n", 
                currentAngle, curtain.currentAngle, deltaAngle);
  
  // Normalize deltaAngle to be within -180 to 180 degrees
  if (deltaAngle > 180) {
    deltaAngle -= 360;
    Serial.printf("[DEBUG] Normalized delta angle: %.2f° (wrapped -180)\n", deltaAngle);
  }
  if (deltaAngle < -180) {
    deltaAngle += 360;
    Serial.printf("[DEBUG] Normalized delta angle: %.2f° (wrapped +180)\n", deltaAngle);
  }
  
  curtain.currentAngle = currentAngle;
  
  // Calculate total rotation based on direction
  float previousTotal = curtain.totalRotation;
  curtain.totalRotation += deltaAngle;
  
  Serial.printf("[DEBUG] Total rotation: %.2f° → %.2f° (change: %.2f°)\n", 
                previousTotal, curtain.totalRotation, deltaAngle);
  
  // Normalize total rotation to be within 0 to 360 degrees
  if (curtain.totalRotation > 360) {
    curtain.totalRotation -= 360;
    Serial.printf("[DEBUG] Wrapped total rotation: %.2f°\n", curtain.totalRotation);
  }
  if (curtain.totalRotation < 0) {
    curtain.totalRotation += 360;
    Serial.printf("[DEBUG] Wrapped total rotation: %.2f°\n", curtain.totalRotation);
  }
  
  // Convert total rotation to percentage (0% = closed, 100% = open)
  uint8_t newPosition = constrain(map(curtain.totalRotation, 0, 360, 0, 100), 0, 100);
  
  Serial.printf("[DEBUG] Rotation %.2f° → Position: %d%%\n", curtain.totalRotation, newPosition);
  
  // Check if we've reached the target position
  if ((curtain.direction > 0 && newPosition >= curtain.target) ||
      (curtain.direction < 0 && newPosition <= curtain.target)) {
    // Movement complete
    Serial.printf("[DEBUG] Target reached! Final position: %d%%\n", curtain.target);
    stopMotor();
    curtain.current = curtain.target;
    zigbeeEndpoint.setLiftPercentage(curtain.current);
    Serial.printf("Target reached: %d%%\n", curtain.current);
  } else {
    // Update current position
    curtain.current = newPosition;
    zigbeeEndpoint.setLiftPercentage(curtain.current);
    
    // Periodic status update
    static unsigned long lastUpdate = 0;
    if (millis() - lastUpdate >= UPDATE_INTERVAL_MS) {
      Serial.printf("Position: %d%% (Target: %d%%, Direction: %d)\n", 
                    curtain.current, curtain.target, curtain.direction);
      lastUpdate = millis();
    }
  }
}

void startMotor(int direction) {
  motor.pwm = MOTOR_SPEED;
  
  Serial.printf("[DEBUG] Starting motor - Direction: %d, Speed: %d\n", direction, MOTOR_SPEED);
  
  if (direction > 0) {
    motor.front();
    Serial.println("[INFO] Opening curtain");
  } else if (direction < 0) {
    motor.back();
    Serial.println("[INFO] Closing curtain");
  }
  
  curtain.isMoving = true;
  curtain.direction = direction;
  curtain.currentAngle = as5600.readAngle(); // Record starting angle for movement
  
  Serial.printf("[DEBUG] Motor started - Current angle: %.2f°, Direction: %d\n", 
                curtain.currentAngle, direction);
}

void stopMotor() {
  motor.stop();
  curtain.isMoving = false;
  curtain.direction = 0;
  Serial.println("Motor stopped");
}

void moveToPosition(uint8_t targetPercent) {
  stopMotor();
  
  curtain.target = constrain(targetPercent, 0, 100);
  
  if (curtain.target == curtain.current) {
    Serial.printf("Already at %d%%\n", curtain.current);
    return;
  }
  
  Serial.printf("Moving from %d%% to %d%%\n", curtain.current, curtain.target);
  
  int direction = (curtain.target > curtain.current) ? 1 : -1;
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

// Calibration function to set rotation range
void calibrateRotation() {
  Serial.println("Starting rotation calibration...");
  Serial.println("Move curtain to fully closed position (0%) and press any key");
  
  while (!Serial.available()) {
    delay(100);
  }
  Serial.read(); // Clear the input
  
  float closedAngle = as5600.readAngle();
  Serial.printf("Closed position recorded at %.2f degrees\n", closedAngle);
  
  Serial.println("Move curtain to fully open position (100%) and press any key");
  
  while (!Serial.available()) {
    delay(100);
  }
  Serial.read(); // Clear the input
  
  float openAngle = as5600.readAngle();
  Serial.printf("Open position recorded at %.2f degrees\n", openAngle);
  
  // Calculate rotation range
  float rotationRange = abs(openAngle - closedAngle);
  if (rotationRange > 180) {
    rotationRange = 360 - rotationRange;
  }
  
  curtain.totalRotation = rotationRange;
  curtain.currentAngle = as5600.readAngle();
  
  Serial.printf("Calibration complete. Rotation range: %.2f degrees\n", rotationRange);
}

// Function to get current angle for debugging
float getCurrentAngle() {
  return as5600.readAngle();
}
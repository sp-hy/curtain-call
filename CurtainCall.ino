#include <BTS7960.h>
#include "ZigbeeCore.h"
#include "ep/ZigbeeWindowCovering.h"

// Configuration
#define ZIGBEE_ENDPOINT 10
#define MOTOR_L_EN 0
#define MOTOR_R_EN 7
#define MOTOR_L_PWM 6
#define MOTOR_R_PWM 5

// Motor settings
const int MOTOR_SPEED = 200;
const unsigned long TRAVEL_TIME_MS = 30000;  // 30 seconds for full travel
const int UPDATE_INTERVAL_MS = 100;

// Motor driver
BTS7960 motor(MOTOR_L_EN, MOTOR_R_EN, MOTOR_L_PWM, MOTOR_R_PWM);

// Position tracking
struct {
  uint8_t current = 100;
  uint8_t target = 100;
  bool isMoving = false;
  int direction = 0;  // 1=opening, -1=closing, 0=stopped
  unsigned long startTime = 0;
} curtain;

// Zigbee endpoint
ZigbeeWindowCovering zigbeeEndpoint(ZIGBEE_ENDPOINT);

void setup() {
  Serial.begin(115200);
  
  // Initialize motor
  motor.begin();
  motor.enable();
  motor.stop();

  // Configure Zigbee
  setupZigbee();
  
  // Connect to network
  connectToZigbee();
  
  // Set initial position
  zigbeeEndpoint.setLiftPercentage(curtain.current);
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
  
  unsigned long elapsed = millis() - curtain.startTime;
  unsigned long targetTime = abs(curtain.target - curtain.current) * TRAVEL_TIME_MS / 100;
  
  if (elapsed >= targetTime) {
    // Movement complete
    stopMotor();
    curtain.current = curtain.target;
    zigbeeEndpoint.setLiftPercentage(curtain.current);
    Serial.printf("Position: %d%%\n", curtain.current);
  } else {
    // Update position periodically
    static unsigned long lastUpdate = 0;
    if (millis() - lastUpdate >= UPDATE_INTERVAL_MS) {
      uint8_t newPos = curtain.current + (curtain.direction * (elapsed * 100 / TRAVEL_TIME_MS));
      curtain.current = constrain(newPos, 0, 100);
      zigbeeEndpoint.setLiftPercentage(curtain.current);
      lastUpdate = millis();
    }
  }
}

void startMotor(int direction) {
  motor.pwm = MOTOR_SPEED;
  
  if (direction > 0) {
    motor.front();
    Serial.println("Opening curtain");
  } else if (direction < 0) {
    motor.back();
    Serial.println("Closing curtain");
  }
  
  curtain.isMoving = true;
  curtain.direction = direction;
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
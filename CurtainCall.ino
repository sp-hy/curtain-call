#include <BTS7960.h>
#include "ZigbeeCore.h"
#include "ep/ZigbeeWindowCovering.h"

// Pin definitions
#define ZIGBEE_COVERING_ENDPOINT 10
#define MANUAL_BUTTON_PIN        9
#define L_EN  0
#define R_EN  1
#define L_PWM 2
#define R_PWM 3

// Configuration
#define MAX_LIFT_CM              200
#define MOTOR_SPEED_PWM          10
#define FULL_TRAVEL_TIME_MS      30000
#define BUTTON_DEBOUNCE_MS       100
#define FACTORY_RESET_HOLD_MS    3000
#define POSITION_INCREMENT       20

// Global variables
BTS7960 motor1(L_EN, R_EN, L_PWM, R_PWM);
ZigbeeWindowCovering windowCoveringEndpoint = ZigbeeWindowCovering(ZIGBEE_COVERING_ENDPOINT);

uint8_t currentLiftPercent = 100;
uint8_t targetLiftPercent = 100;
unsigned long moveStartTime = 0;
bool isMoving = false;
int moveDirection = 0;

void setup() {
  Serial.begin(115200);
  pinMode(MANUAL_BUTTON_PIN, INPUT_PULLUP);
  
  motor1.begin();
  motor1.enable();
  motor1.stop();

  // Configure Zigbee endpoint
  windowCoveringEndpoint.setManufacturerAndModel("LC", "CurtainCall2");
  windowCoveringEndpoint.setCoveringType(DRAPERY);
  windowCoveringEndpoint.setConfigStatus(true, true, false, true, true, true, true);
  windowCoveringEndpoint.setMode(false, true, false, false);
  windowCoveringEndpoint.setLimits(0, MAX_LIFT_CM, 0, 0);

  // Register callbacks
  windowCoveringEndpoint.onOpen([]() { moveToPosition(100); });
  windowCoveringEndpoint.onClose([]() { moveToPosition(0); });
  windowCoveringEndpoint.onGoToLiftPercentage(moveToPosition);
  windowCoveringEndpoint.onStop(stopMotor);

  Zigbee.addEndpoint(&windowCoveringEndpoint);

  if (!Zigbee.begin()) {
    Serial.println("Zigbee failed to start!");
    ESP.restart();
  }
  
  while (!Zigbee.connected()) {
    Serial.print(".");
    delay(100);
  }
  Serial.println("\nConnected to Zigbee network");

  windowCoveringEndpoint.setLiftPercentage(currentLiftPercent);
}

void loop() {
  updateMotorPosition();
  
  if (digitalRead(MANUAL_BUTTON_PIN) == LOW) {
    delay(BUTTON_DEBOUNCE_MS);
    
    int buttonPressStart = millis();
    while (digitalRead(MANUAL_BUTTON_PIN) == LOW) {
      delay(50);
      if ((millis() - buttonPressStart) > FACTORY_RESET_HOLD_MS) {
        Serial.println("Factory reset initiated");
        Zigbee.factoryReset();
        delay(30000);
      }
    }
    cyclePosition();
  }
  delay(50);
}

void updateMotorPosition() {
  if (!isMoving) return;
  
  unsigned long elapsedTime = millis() - moveStartTime;
  unsigned long targetTime = abs(targetLiftPercent - currentLiftPercent) * FULL_TRAVEL_TIME_MS / 100;
  
  if (elapsedTime >= targetTime) {
    stopMotor();
    currentLiftPercent = targetLiftPercent;
    windowCoveringEndpoint.setLiftPercentage(currentLiftPercent);
    Serial.printf("Position: %d%%\n", currentLiftPercent);
  } else {
    uint8_t startPercent = (moveDirection > 0) ? targetLiftPercent - abs(targetLiftPercent - currentLiftPercent) : targetLiftPercent + abs(targetLiftPercent - currentLiftPercent);
    uint8_t newPercent = constrain(startPercent + (moveDirection * (elapsedTime * 100 / FULL_TRAVEL_TIME_MS)), 0, 100);
    
    static unsigned long lastUpdate = 0;
    if (millis() - lastUpdate >= 100) {
      currentLiftPercent = newPercent;
      windowCoveringEndpoint.setLiftPercentage(currentLiftPercent);
      lastUpdate = millis();
    }
  }
}

void startMotor(int direction) {
  motor1.pwm = MOTOR_SPEED_PWM;
  motor1.front();
  if (direction < 0) motor1.back();
  
  isMoving = true;
  moveDirection = direction;
  moveStartTime = millis();
  Serial.println(direction > 0 ? "Opening" : "Closing");
}

void stopMotor() {
  motor1.stop();
  isMoving = false;
  moveDirection = 0;
  Serial.println("Stopped");
}

void moveToPosition(uint8_t targetPercent) {
  stopMotor();
  targetLiftPercent = constrain(targetPercent, 0, 100);
  
  if (targetLiftPercent == currentLiftPercent) return;
  
  Serial.printf("Moving to %d%%\n", targetLiftPercent);
  startMotor(targetLiftPercent > currentLiftPercent ? 1 : -1);
}

void cyclePosition() {
  uint8_t newPosition = (currentLiftPercent + POSITION_INCREMENT) % 101;
  Serial.printf("Manual: %d%%\n", newPosition);
  moveToPosition(newPosition);
}
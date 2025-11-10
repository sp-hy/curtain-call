#include "ZigbeeCore.h"
#include "ep/ZigbeeWindowCovering.h"

// Hardware Configuration
#define ZIGBEE_COVERING_ENDPOINT 10
#define MANUAL_BUTTON_PIN        9
#define STEP_PIN                 2
#define DIR_PIN                  3
#define ENABLE_PIN               4

// Position Limits
#define MAX_LIFT_CM              200
#define MIN_LIFT_CM              0

// Motor Configuration
const int STEPS_PER_REVOLUTION = 200;
const int MICROSTEPS = 2;
const int STEPS_FOR_FULL_TRAVEL = 800;

// Timing Settings
const int STEP_DELAY_US = 10;
const int MIN_PULSE_WIDTH_US = 25;
const int POSITION_UPDATE_INTERVAL_MS = 100;

// Control Settings
const int BUTTON_DEBOUNCE_MS = 100;
const int FACTORY_RESET_HOLD_MS = 3000;
const int POSITION_INCREMENT_PERCENT = 20;

// State Variables
uint8_t currentLiftPercent = 100;
uint8_t targetLiftPercent = 100;
long currentStepPosition = STEPS_FOR_FULL_TRAVEL;
long targetStepPosition = STEPS_FOR_FULL_TRAVEL;
bool isMoving = false;
int moveDirection = 0;
unsigned long lastStepTime = 0;

ZigbeeWindowCovering windowCoveringEndpoint = ZigbeeWindowCovering(ZIGBEE_COVERING_ENDPOINT);

void setup() {
  Serial.begin(115200);
  
  initializeHardware();
  configureZigbee();
  connectToZigbee();
  
  windowCoveringEndpoint.setLiftPercentage(currentLiftPercent);
  Serial.println("CurtainCall initialized");
}

void initializeHardware() {
  pinMode(MANUAL_BUTTON_PIN, INPUT_PULLUP);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);
  digitalWrite(ENABLE_PIN, HIGH);
}

void configureZigbee() {
  windowCoveringEndpoint.setManufacturerAndModel("LC", "CurtainCall2");
  windowCoveringEndpoint.setCoveringType(DRAPERY);
  windowCoveringEndpoint.setConfigStatus(true, true, false, true, true, true, true);
  windowCoveringEndpoint.setMode(false, true, false, false);
  windowCoveringEndpoint.setLimits(MIN_LIFT_CM, MAX_LIFT_CM, 0, 0);

  windowCoveringEndpoint.onOpen(openCurtainFully);
  windowCoveringEndpoint.onClose(closeCurtainFully);
  windowCoveringEndpoint.onGoToLiftPercentage(moveToLiftPosition);
  windowCoveringEndpoint.onStop(stopCurtainMotor);
}

void connectToZigbee() {
  Zigbee.addEndpoint(&windowCoveringEndpoint);
  
  if (!Zigbee.begin()) {
    Serial.println("Zigbee failed to start! Rebooting...");
    ESP.restart();
  }
  
  while (!Zigbee.connected()) {
    Serial.print(".");
    delay(100);
  }
  Serial.println("\nConnected to Zigbee network");
}

void loop() {
  updateMotorPosition();
  handleButtonPress();
  delay(10);
}

void handleButtonPress() {
  if (digitalRead(MANUAL_BUTTON_PIN) == LOW) {
    delay(BUTTON_DEBOUNCE_MS);
    
    unsigned long buttonPressStart = millis();
    while (digitalRead(MANUAL_BUTTON_PIN) == LOW) {
      delay(50);
      if ((millis() - buttonPressStart) > FACTORY_RESET_HOLD_MS) {
        Serial.println("Factory reset initiated");
        Zigbee.factoryReset();
        delay(30000);
      }
    }
    cycleThroughPositions();
  }
}

void updateMotorPosition() {
  if (!isMoving || micros() - lastStepTime < STEP_DELAY_US) return;
  
  if (currentStepPosition == targetStepPosition) {
    stopMotor();
    currentLiftPercent = targetLiftPercent;
    windowCoveringEndpoint.setLiftPercentage(currentLiftPercent);
    return;
  }
  
  takeStep();
  lastStepTime = micros();
  
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate >= POSITION_UPDATE_INTERVAL_MS) {
    currentLiftPercent = map(currentStepPosition, 0, STEPS_FOR_FULL_TRAVEL, 0, 100);
    currentLiftPercent = constrain(currentLiftPercent, 0, 100);
    windowCoveringEndpoint.setLiftPercentage(currentLiftPercent);
    lastUpdate = millis();
  }
}

void takeStep() {
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(MIN_PULSE_WIDTH_US);
  digitalWrite(STEP_PIN, LOW);
  
  currentStepPosition += moveDirection;
  currentStepPosition = constrain(currentStepPosition, 0, STEPS_FOR_FULL_TRAVEL);
}

void startMotor(int direction) {
  digitalWrite(ENABLE_PIN, LOW);
  digitalWrite(DIR_PIN, direction > 0 ? HIGH : LOW);
  
  isMoving = true;
  moveDirection = direction;
  lastStepTime = micros();
}

void stopMotor() {
  isMoving = false;
  moveDirection = 0;
  digitalWrite(ENABLE_PIN, HIGH);
}

void moveCurtainToPosition(uint8_t targetPercent) {
  stopMotor();
  
  targetLiftPercent = constrain(targetPercent, 0, 100);
  targetStepPosition = map(targetLiftPercent, 0, 100, 0, STEPS_FOR_FULL_TRAVEL);
  
  if (targetStepPosition == currentStepPosition) return;
  
  int direction = (targetStepPosition > currentStepPosition) ? 1 : -1;
  startMotor(direction);
}

// Zigbee Callbacks
void openCurtainFully() {
  moveCurtainToPosition(100);
}

void closeCurtainFully() {
  moveCurtainToPosition(0);
}

void moveToLiftPosition(uint8_t liftPercentage) {
  moveCurtainToPosition(liftPercentage);
}

void stopCurtainMotor() {
  stopMotor();
  windowCoveringEndpoint.setLiftPercentage(currentLiftPercent);
}

void cycleThroughPositions() {
  uint8_t newPosition = currentLiftPercent + POSITION_INCREMENT_PERCENT;
  if (newPosition > 100) newPosition = 0;
  moveCurtainToPosition(newPosition);
}
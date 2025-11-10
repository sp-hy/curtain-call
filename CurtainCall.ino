#include "ZigbeeCore.h"
#include "ep/ZigbeeWindowCovering.h"
#include <Preferences.h>

// Hardware Configuration - ESP32-C6 XIAO Pin Mapping
#define ZIGBEE_COVERING_ENDPOINT 10
#define MANUAL_BUTTON_PIN        9   // Boot button (built-in, safe to use)
#define STEP_PIN                 0   // GPIO0 - Stepper step/PUL+ signal
#define DIR_PIN                  1   // GPIO1 - Stepper direction/DIR+ signal  
#define ENABLE_PIN               2   // GPIO2 - Stepper enable/ENA+ signal
#define RF_SWITCH_POWER_PIN      3   // GPIO3 - RF switch power control
#define ANTENNA_SELECT_PIN       14  // GPIO14 - External antenna select

// Position Limits - Using step-based positioning only

// Motor Configuration
const int MICROSTEPS = 2;
const int STEPS_FOR_FULL_TRAVEL = 2000;

// Timing Settings - Reliable stepping
const int STEP_DELAY_US = 1000;       // 1ms between steps for reliable operation
const int MIN_PULSE_WIDTH_US = 10;    // 10us pulse width for reliable step detection
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

// Persistent storage for position state
Preferences prefs;

// Save current position to NVS
void savePosition() {
  prefs.putUChar("liftPercent", currentLiftPercent);
  prefs.putLong("stepPosition", currentStepPosition);
  Serial.print("Position saved: ");
  Serial.print(currentLiftPercent);
  Serial.println("%");
}

// Load saved position from NVS
void loadPosition() {
  currentLiftPercent = prefs.getUChar("liftPercent", 100);  // Default to 100% (open)
  currentStepPosition = prefs.getLong("stepPosition", STEPS_FOR_FULL_TRAVEL);
  targetLiftPercent = currentLiftPercent;
  targetStepPosition = currentStepPosition;
  Serial.print("Position loaded: ");
  Serial.print(currentLiftPercent);
  Serial.println("%");
}

void setup() {
  Serial.begin(115200);
  
  // Initialize persistent storage and load saved position
  prefs.begin("curtain", false);  // false = read/write mode
  loadPosition();
  
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
  
  // External antenna configuration
  pinMode(RF_SWITCH_POWER_PIN, OUTPUT);    // RF switch power on
  digitalWrite(RF_SWITCH_POWER_PIN, LOW);
  
  pinMode(ANTENNA_SELECT_PIN, OUTPUT);     // select external antenna
  digitalWrite(ANTENNA_SELECT_PIN, HIGH);
  
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);
  digitalWrite(ENABLE_PIN, HIGH);
}

void configureZigbee() {
  windowCoveringEndpoint.setManufacturerAndModel("LC", "CurtainCall");
  windowCoveringEndpoint.setCoveringType(DRAPERY);
  windowCoveringEndpoint.setConfigStatus(true, true, false, true, true, true, true);
  windowCoveringEndpoint.setMode(false, true, false, false);
  windowCoveringEndpoint.setLimits(0, 100, 0, 0);  // 0-100% limits

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
  delay(5);
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
    savePosition();  // Save position when movement completes
    Serial.print("Movement complete - Position: ");
    Serial.print(currentLiftPercent);
    Serial.println("%");
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

// Calibration function - call this to reset position to known state
void calibratePosition() {
  Serial.println("Calibrating position...");
  // Move to fully closed position (0%)
  moveCurtainToPosition(0);
  while (isMoving) {
    updateMotorPosition();
    delay(10);
  }
  // Reset step counter to ensure accuracy
  currentStepPosition = 0;
  currentLiftPercent = 0;
  windowCoveringEndpoint.setLiftPercentage(0);
  Serial.println("Calibration complete - position reset to 0%");
}
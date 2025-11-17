/**
 * CurtainCall - Smart Zigbee Curtain Controller
 * 
 * CONFIGURATION VIA HOME ASSISTANT:
 * ---------------------------------
 * The "Tilt" control is hijacked for configuration:
 * - Tilt Position (0-1000) = STEPS_FOR_FULL_TRAVEL / 10
 * - To set 2000 steps: Set tilt to 200
 * - To set 5000 steps: Set tilt to 500
 * - Range: 10-1000 tilt = 100-10000 steps
 * 
 * Configuration is saved to flash and persists across reboots.
 */

#include "ZigbeeCore.h"
#include "ep/ZigbeeWindowCovering.h"
#include <Preferences.h>

// ============================================================================
// Hardware Configuration (ESP32-C6 XIAO)
// ============================================================================
#define ZIGBEE_COVERING_ENDPOINT 10
#define MANUAL_BUTTON_PIN        9   // Boot button (built-in)
#define STEP_PIN                 0   // Stepper step/PUL+ signal
#define DIR_PIN                  1   // Stepper direction/DIR+ signal
#define ENABLE_PIN               2   // Stepper enable/ENA+ signal
#define RF_SWITCH_POWER_PIN      3   // RF switch power control
#define ANTENNA_SELECT_PIN       14  // External antenna select

// ============================================================================
// Motion and Motor Settings
// ============================================================================
const int MICROSTEPS = 2;
int STEPS_FOR_FULL_TRAVEL = 2000;   // Total steps for full curtain travel (configurable via Tilt)
const int MIN_PULSE_WIDTH_US = 10;
const int POSITION_UPDATE_INTERVAL_MS = 100;

// Configuration limits (using Tilt attribute as configuration storage)
const int MIN_STEPS_CONFIG = 100;     // Minimum steps (safety limit)
const int MAX_STEPS_CONFIG = 10000;   // Maximum steps (safety limit)

// Motor timing and acceleration
float currentDelayUs = 4000.0f;            // Start speed (microseconds between steps)
const float targetDelayUs = 1600.0f;       // Target running speed (microseconds)
const float accelStepUs = 5.0f;            // Acceleration rate (smaller = slower ramp)

// Motor idle timeout
const unsigned long MOTOR_IDLE_TIMEOUT_MS = 5000;

// ============================================================================
// Control Settings
// ============================================================================
const int BUTTON_DEBOUNCE_MS = 100;
const int FACTORY_RESET_HOLD_MS = 3000;

// ============================================================================
// State Variables
// ============================================================================
uint8_t currentLiftPercent = 100;          // Current curtain position (0-100%)
uint8_t targetLiftPercent = 100;           // Target curtain position (0-100%)
long currentStepPosition = STEPS_FOR_FULL_TRAVEL;  // Current motor step position
long targetStepPosition = STEPS_FOR_FULL_TRAVEL;   // Target motor step position
bool isMoving = false;                     // Motor movement state
int moveDirection = 0;                     // Movement direction (-1, 0, 1)
unsigned long lastStepTime = 0;            // Last step timestamp
unsigned long lastMotionTime = 0;          // Last motion timestamp

// Zigbee and storage objects
ZigbeeWindowCovering windowCoveringEndpoint(ZIGBEE_COVERING_ENDPOINT);
Preferences prefs;

// ============================================================================
// Persistent Storage Functions
// ============================================================================
void savePosition() {
  prefs.putUChar("liftPercent", currentLiftPercent);
  prefs.putLong("stepPosition", currentStepPosition);
  Serial.printf("Position saved: %d%% (%ld steps)\n", currentLiftPercent, currentStepPosition);
}

void saveConfig() {
  prefs.putInt("maxSteps", STEPS_FOR_FULL_TRAVEL);
  Serial.printf("Config saved: %d max steps\n", STEPS_FOR_FULL_TRAVEL);
}

void loadPosition() {
  currentLiftPercent = prefs.getUChar("liftPercent", 100);
  currentStepPosition = prefs.getLong("stepPosition", STEPS_FOR_FULL_TRAVEL);
  targetLiftPercent = currentLiftPercent;
  targetStepPosition = currentStepPosition;
  Serial.printf("Loaded position: %d%% (%ld steps)\n", currentLiftPercent, currentStepPosition);
}

void loadConfig() {
  STEPS_FOR_FULL_TRAVEL = prefs.getInt("maxSteps", 2000);
  STEPS_FOR_FULL_TRAVEL = constrain(STEPS_FOR_FULL_TRAVEL, MIN_STEPS_CONFIG, MAX_STEPS_CONFIG);
  Serial.printf("Loaded config: %d max steps\n", STEPS_FOR_FULL_TRAVEL);
}

// ============================================================================
// Setup and Initialization
// ============================================================================
void setup() {
  Serial.begin(115200);
  prefs.begin("curtain", false);
  loadConfig();      // Load configuration first (max steps)
  loadPosition();    // Then load position (depends on max steps)

  initializeHardware();
  configureZigbee();
  connectToZigbee();

  windowCoveringEndpoint.setLiftPercentage(currentLiftPercent);
  // Set initial tilt to represent current config (steps / 10 to fit in 0-900 range)
  windowCoveringEndpoint.setTiltPosition(STEPS_FOR_FULL_TRAVEL / 10);
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

  pinMode(RF_SWITCH_POWER_PIN, OUTPUT);
  digitalWrite(RF_SWITCH_POWER_PIN, LOW);
  pinMode(ANTENNA_SELECT_PIN, OUTPUT);
  digitalWrite(ANTENNA_SELECT_PIN, HIGH);
}

// ============================================================================
// Zigbee Configuration
// ============================================================================
void configureZigbee() {
  windowCoveringEndpoint.setManufacturerAndModel("LC", "CurtainCall");
  // Use BLIND_LIFT_AND_TILT to expose both lift (position) and tilt (configuration)
  windowCoveringEndpoint.setCoveringType(BLIND_LIFT_AND_TILT);
  windowCoveringEndpoint.setConfigStatus(true, true, false, true, true, true, true);
  windowCoveringEndpoint.setMode(false, true, false, false);
  // Use Tilt range (0-1000) to store configuration: steps / 10 fits in 0-1000 range
  windowCoveringEndpoint.setLimits(0, 100, 0, 1000);

  windowCoveringEndpoint.onOpen(openCurtainFully);
  windowCoveringEndpoint.onClose(closeCurtainFully);
  windowCoveringEndpoint.onGoToLiftPercentage(moveToLiftPosition);
  windowCoveringEndpoint.onGoToTiltPercentage(onTiltPercentageChange);  // Hijack for config
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

// ============================================================================
// Main Loop
// ============================================================================
void loop() {
  handleButtonPress();
  updateMotorPosition();

  // Disable motor driver when idle to reduce power consumption
  if (!isMoving && (millis() - lastMotionTime > MOTOR_IDLE_TIMEOUT_MS)) {
    digitalWrite(ENABLE_PIN, HIGH);
  }

  delayMicroseconds(100);
}

// ============================================================================
// Manual Button Handling
// ============================================================================
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
  }
}

// ============================================================================
// Motor Control Functions
// ============================================================================
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
  lastMotionTime = millis();
  currentDelayUs = 4000.0f; // Reset acceleration ramp
}

void stopMotor() {
  isMoving = false;
  moveDirection = 0;
  lastMotionTime = millis();
  savePosition();
  Serial.printf("Stopped at %d%% (%ld steps)\n", currentLiftPercent, currentStepPosition);
}

// ============================================================================
// Motion Update Loop
// ============================================================================
void updateMotorPosition() {
  static unsigned long lastUpdate = 0;
  
  if (!isMoving) return;

  unsigned long now = micros();
  if (now - lastStepTime < (unsigned long)currentDelayUs) return;

  takeStep();
  lastStepTime = now;

  // Acceleration: gradually increase speed until target is reached
  if (currentDelayUs > targetDelayUs)
    currentDelayUs -= accelStepUs;

  // Stop when target position is reached
  if (currentStepPosition == targetStepPosition) {
    // Update to exact target percentage before stopping
    currentLiftPercent = targetLiftPercent;
    windowCoveringEndpoint.setLiftPercentage(currentLiftPercent);
    stopMotor();
    lastUpdate = millis();  // Reset timer to prevent stale update after stop
    return;
  }

  // Update Zigbee position every 100ms (only during motion)
  if (millis() - lastUpdate >= POSITION_UPDATE_INTERVAL_MS) {
    uint8_t progressPercent = (uint8_t)((float)currentStepPosition / STEPS_FOR_FULL_TRAVEL * 100.0f);
    progressPercent = constrain(progressPercent, 0, 100);
    windowCoveringEndpoint.setLiftPercentage(progressPercent);
    currentLiftPercent = progressPercent;
    lastUpdate = millis();
  }
}

// ============================================================================
// High-Level Curtain Commands
// ============================================================================
void moveCurtainToPosition(uint8_t targetPercent) {
  targetLiftPercent = constrain(targetPercent, 0, 100);
  targetStepPosition = (long)((float)targetLiftPercent / 100.0f * STEPS_FOR_FULL_TRAVEL);

  if (targetStepPosition == currentStepPosition) {
    Serial.println("Target position already reached");
    return;
  }

  int direction = (targetStepPosition > currentStepPosition) ? 1 : -1;
  startMotor(direction);

  Serial.printf("Moving %ld -> %ld steps (%d%% -> %d%%)\n",
                currentStepPosition, targetStepPosition,
                currentLiftPercent, targetLiftPercent);
}

// ============================================================================
// Zigbee Callback Functions
// ============================================================================
void openCurtainFully()  { moveCurtainToPosition(100); }
void closeCurtainFully() { moveCurtainToPosition(0); }
void moveToLiftPosition(uint8_t liftPercentage) { moveCurtainToPosition(liftPercentage); }

void stopCurtainMotor() {
  stopMotor();
  windowCoveringEndpoint.setLiftPercentage(currentLiftPercent);
}

// CONFIGURATION CALLBACK - Hijacking Tilt for configuration storage
// Tilt percentage (0-100%) maps to steps configuration
// We scale it to actual step range: percentage maps to position value
void onTiltPercentageChange(uint8_t tiltPercentage) {
  // Convert tilt percentage to position value (0-1000 range from setLimits)
  uint16_t tiltPosition = (tiltPercentage * 1000) / 100;
  
  // Scale tilt position (0-1000) to steps (100-10000)
  // Multiply by 10 to get actual steps
  int newSteps = tiltPosition * 10;
  newSteps = constrain(newSteps, MIN_STEPS_CONFIG, MAX_STEPS_CONFIG);
  
  if (newSteps != STEPS_FOR_FULL_TRAVEL) {
    STEPS_FOR_FULL_TRAVEL = newSteps;
    saveConfig();
    
    // Recalculate current and target positions based on percentages
    currentStepPosition = (long)((float)currentLiftPercent / 100.0f * STEPS_FOR_FULL_TRAVEL);
    targetStepPosition = (long)((float)targetLiftPercent / 100.0f * STEPS_FOR_FULL_TRAVEL);
    
    Serial.printf("Configuration updated: Max steps set to %d (from tilt %d%%)\n", 
                  STEPS_FOR_FULL_TRAVEL, tiltPercentage);
  }
  
  // Echo back the tilt position to confirm the setting
  windowCoveringEndpoint.setTiltPosition(tiltPosition);
}

// ============================================================================
// Calibration Functions
// ============================================================================
void calibratePosition() {
  Serial.println("Calibrating position...");
  moveCurtainToPosition(0);
  while (isMoving) {
    updateMotorPosition();
    delay(10);
  }
  currentStepPosition = 0;
  currentLiftPercent = 0;
  windowCoveringEndpoint.setLiftPercentage(0);
  Serial.println("Calibration complete");
}

#include <Stepper.h>
#include "ZigbeeCore.h"
#include "ep/ZigbeeWindowCovering.h"

// Pin definitions
#define ZIGBEE_COVERING_ENDPOINT 10
#define MANUAL_BUTTON_PIN        9  // ESP32-C6/H2 Boot button

// Curtain position limits (in centimeters)
#define MAX_LIFT_CM              200  // centimeters from open position (0-200)
#define MIN_LIFT_CM              0

// Stepper motor configuration
const int STEPS_PER_REVOLUTION = 200;  // Adjust for your stepper motor
const int TOTAL_STEPS_RANGE = 2000;    // Total steps for full curtain range
const int MOTOR_SPEED_RPM = 60;        // Motor speed in RPM

// Manual control settings
const int BUTTON_DEBOUNCE_MS = 100;
const int FACTORY_RESET_HOLD_MS = 3000;
const int POSITION_INCREMENT_PERCENT = 20;

// Stepper motor pins: IN1, IN2, IN3, IN4
Stepper curtainStepper(STEPS_PER_REVOLUTION, 0, 6, 7, 5);

// Position tracking variables
int currentStepPosition = TOTAL_STEPS_RANGE;  // Start fully open
uint8_t currentLiftPercent = 100;             // Current position as percentage

// Zigbee window covering endpoint
ZigbeeWindowCovering windowCoveringEndpoint = ZigbeeWindowCovering(ZIGBEE_COVERING_ENDPOINT);

void setup() {
  Serial.begin(115200);
  
  // Configure hardware
  pinMode(MANUAL_BUTTON_PIN, INPUT_PULLUP);
  curtainStepper.setSpeed(MOTOR_SPEED_RPM);

  // Configure Zigbee window covering properties
  windowCoveringEndpoint.setManufacturerAndModel("LC", "CurtainCall");
  windowCoveringEndpoint.setCoveringType(DRAPERY);
  windowCoveringEndpoint.setConfigStatus(true, true, false, true, true, true, true);
  windowCoveringEndpoint.setMode(false, true, false, false);
  windowCoveringEndpoint.setLimits(MIN_LIFT_CM, MAX_LIFT_CM, 0, 0); // Only lift supported

  // Register callback functions
  windowCoveringEndpoint.onOpen(openCurtainFully);
  windowCoveringEndpoint.onClose(closeCurtainFully);
  windowCoveringEndpoint.onGoToLiftPercentage(moveToLiftPosition);
  windowCoveringEndpoint.onStop(stopCurtainMotor);

  // Initialize Zigbee network
  Serial.println("Adding ZigbeeWindowCovering endpoint to Zigbee Core");
  Zigbee.addEndpoint(&windowCoveringEndpoint);

  Serial.println("Starting Zigbee connection...");
  if (!Zigbee.begin()) {
    Serial.println("Zigbee failed to start!");
    Serial.println("Rebooting...");
    ESP.restart();
  }
  
  Serial.println("Connecting to Zigbee network");
  while (!Zigbee.connected()) {
    Serial.print(".");
    delay(100);
  }
  Serial.println();
  Serial.println("Connected to Zigbee network");

  // Set initial position
  windowCoveringEndpoint.setLiftPercentage(currentLiftPercent);
}

void loop() {
  // Handle manual button press
  if (digitalRead(MANUAL_BUTTON_PIN) == LOW) {
    delay(BUTTON_DEBOUNCE_MS);
    
    int buttonPressStart = millis();
    while (digitalRead(MANUAL_BUTTON_PIN) == LOW) {
      delay(50);
      // Check for factory reset (long press)
      if ((millis() - buttonPressStart) > FACTORY_RESET_HOLD_MS) {
        Serial.println("Factory reset initiated - hold detected");
        Zigbee.factoryReset();
        Serial.println("Rebooting in 30 seconds...");
        delay(30000);
      }
    }
    cycleThroughPositions();
  }
  delay(500);
}

// Convert percentage (0-100) to stepper motor steps
int convertPercentageToSteps(uint8_t percentage) {
  return (percentage * TOTAL_STEPS_RANGE) / 100;
}

// Move curtain to specified percentage position
void moveCurtainToPosition(uint8_t targetPercent) {
  int targetSteps = convertPercentageToSteps(targetPercent);
  int stepsToMove = targetSteps - currentStepPosition;
  
  if (stepsToMove != 0) {
    Serial.printf("Moving curtain from %d%% to %d%% (%d steps)\n", 
                  currentLiftPercent, targetPercent, stepsToMove);
    curtainStepper.step(stepsToMove);
    currentStepPosition = targetSteps;
  }
  
  currentLiftPercent = targetPercent;
  windowCoveringEndpoint.setLiftPercentage(currentLiftPercent);
}

// Zigbee callback: Open curtain fully
void openCurtainFully() {
  Serial.println("Opening curtain fully");
  moveCurtainToPosition(100);
}

// Zigbee callback: Close curtain fully
void closeCurtainFully() {
  Serial.println("Closing curtain fully");
  moveCurtainToPosition(0);
}

// Zigbee callback: Move to specific lift percentage
void moveToLiftPosition(uint8_t liftPercentage) {
  Serial.printf("Zigbee command: Move to %d%% lift position\n", liftPercentage);
  moveCurtainToPosition(liftPercentage);
}

// Zigbee callback: Stop motor movement
void stopCurtainMotor() {
  // Note: Stepper.h library doesn't support stopping mid-move
  // Consider using a non-blocking stepper library for true stop functionality
  Serial.println("Stop motor requested (not implemented with Stepper.h)");
  windowCoveringEndpoint.setLiftPercentage(currentLiftPercent);
}

// Manual button control: Cycle through positions in increments
void cycleThroughPositions() {
  uint8_t newPosition = currentLiftPercent + POSITION_INCREMENT_PERCENT;
  
  // Wrap around to 0 if exceeding 100%
  if (newPosition > 100) {
    newPosition = 0;
  }
  
  Serial.printf("Manual control: Moving to %d%%\n", newPosition);
  moveCurtainToPosition(newPosition);
}
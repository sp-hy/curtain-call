#include <BTS7960.h>
#include "ZigbeeCore.h"
#include "ep/ZigbeeWindowCovering.h"

// Pin definitions
#define ZIGBEE_COVERING_ENDPOINT 10
#define MANUAL_BUTTON_PIN        9  // ESP32-C6/H2 Boot button

// BTS7960 motor driver pins
#define L_EN  0   // Left Enable
#define R_EN  1   // Right Enable  
#define L_PWM 2   // Left PWM (reverse)
#define R_PWM 3   // Right PWM (forward)

// Curtain position limits (in centimeters)
#define MAX_LIFT_CM              200  // centimeters from open position (0-200)
#define MIN_LIFT_CM              0

// DC motor configuration
const int MOTOR_SPEED_PWM = 10;           // Motor speed (0-255)
const unsigned long FULL_TRAVEL_TIME_MS = 30000;  // Time for full curtain travel (30 seconds)
const int POSITION_UPDATE_INTERVAL_MS = 100;      // Position update frequency

// Manual control settings
const int BUTTON_DEBOUNCE_MS = 100;
const int FACTORY_RESET_HOLD_MS = 3000;
const int POSITION_INCREMENT_PERCENT = 20;

// BTS7960 motor driver instance
BTS7960 motor1(L_EN, R_EN, L_PWM, R_PWM);

// Position tracking variables
uint8_t currentLiftPercent = 100;             // Current position as percentage
uint8_t targetLiftPercent = 100;              // Target position
unsigned long moveStartTime = 0;              // When movement started
bool isMoving = false;                        // Movement state
int moveDirection = 0;                        // 1 = opening, -1 = closing, 0 = stopped

// Zigbee window covering endpoint
ZigbeeWindowCovering windowCoveringEndpoint = ZigbeeWindowCovering(ZIGBEE_COVERING_ENDPOINT);

void setup() {
  Serial.begin(115200);
  
  // Configure hardware
  pinMode(MANUAL_BUTTON_PIN, INPUT_PULLUP);
  
  // Initialize BTS7960 motor driver using 1337encrypted library
  motor1.begin();
  motor1.enable();
  motor1.stop();

  // Configure Zigbee window covering properties
  windowCoveringEndpoint.setManufacturerAndModel("LC", "CurtainCall2");
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
  // Update motor position if moving
  updateMotorPosition();
  
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
  delay(50);  // Reduced delay for more responsive position updates
}

// Update motor position during movement
void updateMotorPosition() {
  if (!isMoving) return;
  
  unsigned long elapsedTime = millis() - moveStartTime;
  unsigned long targetTime = abs(targetLiftPercent - currentLiftPercent) * FULL_TRAVEL_TIME_MS / 100;
  
  if (elapsedTime >= targetTime) {
    // Movement complete
    stopMotor();
    currentLiftPercent = targetLiftPercent;
    windowCoveringEndpoint.setLiftPercentage(currentLiftPercent);
    Serial.printf("Movement complete. Position: %d%%\n", currentLiftPercent);
  } else {
    // Update current position based on elapsed time
    uint8_t startPercent = (moveDirection > 0) ? targetLiftPercent - abs(targetLiftPercent - currentLiftPercent) : targetLiftPercent + abs(targetLiftPercent - currentLiftPercent);
    uint8_t newPercent = startPercent + (moveDirection * (elapsedTime * 100 / FULL_TRAVEL_TIME_MS));
    
    // Clamp to valid range
    newPercent = constrain(newPercent, 0, 100);
    
    // Update position periodically
    static unsigned long lastUpdate = 0;
    if (millis() - lastUpdate >= POSITION_UPDATE_INTERVAL_MS) {
      currentLiftPercent = newPercent;
      windowCoveringEndpoint.setLiftPercentage(currentLiftPercent);
      lastUpdate = millis();
    }
  }
}

// Start motor movement in specified direction using 1337encrypted library
void startMotor(int direction) {
  motor1.pwm = MOTOR_SPEED_PWM;  // Set speed first
  
  if (direction > 0) {
    // Open curtain (forward)
    motor1.front();
    Serial.println("Motor: Opening curtain");
  } else if (direction < 0) {
    // Close curtain (reverse)
    motor1.back();
    Serial.println("Motor: Closing curtain");
  }
  isMoving = true;
  moveDirection = direction;
  moveStartTime = millis();
}

// Stop motor movement
void stopMotor() {
  motor1.stop();
  isMoving = false;
  moveDirection = 0;
  Serial.println("Motor: Stopped");
}

// Move curtain to specified percentage position
void moveCurtainToPosition(uint8_t targetPercent) {
  // Stop any current movement
  stopMotor();
  
  targetLiftPercent = constrain(targetPercent, 0, 100);
  
  if (targetLiftPercent == currentLiftPercent) {
    Serial.printf("Already at target position: %d%%\n", currentLiftPercent);
    return;
  }
  
  Serial.printf("Moving curtain from %d%% to %d%%\n", currentLiftPercent, targetLiftPercent);
  
  // Determine direction: opening (increasing %) = positive, closing (decreasing %) = negative
  int direction = (targetLiftPercent > currentLiftPercent) ? 1 : -1;
  startMotor(direction);
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
  Serial.println("Stop motor requested");
  stopMotor();
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
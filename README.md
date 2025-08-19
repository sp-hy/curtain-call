# CurtainCall - Smart Curtain Controller with AS5600 Rotation Sensor

A smart curtain controller that uses the AS5600 magnetic rotation sensor for precise position tracking instead of time-based movement.

## Features

- **Precise Position Tracking**: Uses AS5600 magnetic rotation sensor for accurate curtain position measurement
- **Zigbee Integration**: Compatible with Zigbee home automation systems
- **Motor Control**: BTS7960 motor driver for smooth curtain operation
- **Real-time Feedback**: Continuous position updates and status reporting
- **Calibration Support**: Easy setup and calibration for different curtain systems

## Hardware Requirements

### Components
- ESP32 development board
- AS5600 magnetic rotation sensor
- BTS7960 motor driver module
- DC motor for curtain operation
- Magnet for AS5600 sensor
- Power supply (12V recommended for motor)

### Wiring Diagram

```
ESP32 Pin Connections:
- GPIO 21 (SDA) → AS5600 SDA
- GPIO 22 (SCL) → AS5600 SCL
- GPIO 0  → BTS7960 L_EN
- GPIO 7  → BTS7960 R_EN  
- GPIO 6  → BTS7960 L_PWM
- GPIO 5  → BTS7960 R_PWM
- 3.3V    → AS5600 VCC
- GND     → AS5600 GND, BTS7960 GND
- 12V     → BTS7960 VCC (motor power)
```

## AS5600 Sensor Setup

The AS5600 is a magnetic rotation sensor that provides precise angular position measurement:

1. **Mount the sensor** on a fixed part of your curtain mechanism
2. **Attach a magnet** to the rotating part (curtain rod or pulley)
3. **Ensure proper alignment** - the magnet should be centered over the sensor
4. **Check magnetic field strength** - the sensor requires a magnetic field of 30-100 mT

### Sensor Configuration
- **I2C Address**: Default 0x36 (configurable)
- **Resolution**: 12-bit (0.0879° per LSB)
- **Range**: 0-360 degrees
- **Update Rate**: Up to 1kHz

## Installation

1. **Install Required Libraries**:
   - [AS5600 Library](https://github.com/RobTillaart/AS5600) by Rob Tillaart
   - BTS7960 motor driver library
   - Your Zigbee library

2. **Upload the Code**:
   ```bash
   # Upload CurtainCall.ino to your ESP32
   ```

3. **Calibrate the System**:
   - Open Serial Monitor at 115200 baud
   - Follow the calibration prompts to set open/closed positions

## Usage

### Basic Operation
The system automatically tracks curtain position based on rotation angle:

- **0%**: Fully closed position
- **100%**: Fully open position
- **Real-time updates**: Position is continuously monitored and reported

### Zigbee Commands
- `openCurtain()`: Move to 100% open
- `closeCurtain()`: Move to 0% closed  
- `moveToPosition(percentage)`: Move to specific position
- `stopCurtain()`: Stop current movement

### Calibration
Run the calibration function to set your curtain's rotation range:

```cpp
calibrateRotation();
```

This will prompt you to:
1. Move curtain to fully closed position
2. Move curtain to fully open position
3. Automatically calculate the rotation range

## Configuration

### Pin Definitions
```cpp
#define AS5600_SDA 21  // I2C SDA pin
#define AS5600_SCL 22  // I2C SCL pin
#define MOTOR_L_EN 0   // Motor enable left
#define MOTOR_R_EN 7   // Motor enable right
#define MOTOR_L_PWM 6  // Motor PWM left
#define MOTOR_R_PWM 5  // Motor PWM right
```

### Motor Settings
```cpp
const int MOTOR_SPEED = 200;        // PWM value (0-255)
const int UPDATE_INTERVAL_MS = 100; // Position update frequency
```

## Troubleshooting

### AS5600 Issues
- **"AS5600 not found"**: Check I2C wiring and connections
- **Inconsistent readings**: Verify magnet alignment and strength
- **Wrong direction**: Use `as5600.setDirection()` to correct

### Motor Issues
- **Motor not moving**: Check power supply and enable pins
- **Wrong direction**: Swap motor wires or adjust direction logic
- **Jittery movement**: Reduce motor speed or add smoothing

### Position Tracking Issues
- **Incorrect position**: Recalibrate the rotation range
- **Drift over time**: Check for mechanical backlash or sensor mounting
- **Sudden jumps**: Verify magnet is securely attached

## Advanced Features

### Custom Rotation Range
Modify the rotation range for different curtain mechanisms:

```cpp
curtain.totalRotation = 180.0; // For 180-degree rotation
```

### Direction Configuration
Change sensor direction if needed:

```cpp
as5600.setDirection(AS5600_COUNTERCLOCK_WISE);
```

### Error Handling
The system includes error checking for:
- Sensor communication failures
- Invalid position values
- Motor control issues

## License

This project is open source. Please refer to the individual library licenses for the AS5600 and other dependencies.

## Contributing

Contributions are welcome! Please feel free to submit pull requests or open issues for bugs and feature requests.

## Acknowledgments

- [Rob Tillaart](https://github.com/RobTillaart) for the excellent AS5600 library
- The open source community for Zigbee and motor control libraries 
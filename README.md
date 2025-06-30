# CurtainCall - Zigbee Smart Curtain Controller

A Zigbee-enabled smart curtain controller built for ESP32-C6/H2 that allows remote and manual control of motorized curtains through home automation systems.

## Features

- **Zigbee Integration**: Compatible with Zigbee home automation hubs
- **Remote Control**: Open, close, and position curtains via Zigbee commands
- **Manual Control**: Physical button for local operation
- **Position Tracking**: Precise percentage-based positioning (0-100%)
- **Factory Reset**: Long-press button to reset Zigbee settings

## Hardware Requirements

- **Microcontroller**: ESP32-C6 or ESP32-H2 with Zigbee support
- **Stepper Motor**: 200 steps/revolution stepper motor
- **Motor Driver**: Compatible stepper motor driver (ULN2003 or similar)
- **Manual Button**: Connected to pin 9 (uses built-in boot button)

## Pin Connections

| Component | Pin | Description |
|-----------|-----|-------------|
| Stepper IN1 | 0 | Motor control pin 1 |
| Stepper IN2 | 6 | Motor control pin 2 |
| Stepper IN3 | 7 | Motor control pin 3 |
| Stepper IN4 | 5 | Motor control pin 4 |
| Manual Button | 9 | Boot button (INPUT_PULLUP) |

## Configuration

### Curtain Limits
- **Range**: 0-200 cm (adjustable in code)
- **Step Range**: 2000 total steps for full travel
- **Motor Speed**: 60 RPM

### Position Control
- **Increment**: 20% per manual button press
- **Precision**: 1% positioning via Zigbee
- **Direction**: 0% = fully closed, 100% = fully open

## Setup Instructions

1. **Hardware Setup**:
   - Connect stepper motor to pins 0, 6, 7, 5
   - Ensure manual button is connected to pin 9
   - Power the stepper motor appropriately

2. **Software Setup**:
   - Install required libraries: `Stepper.h`, `ZigbeeCore.h`, `ZigbeeWindowCovering.h`
   - Upload the code to your ESP32-C6/H2
   - Monitor serial output for connection status

3. **Zigbee Pairing**:
   - Power on the device
   - The device will automatically attempt to join a Zigbee network
   - Use your Zigbee hub to discover and pair the device
   - Device appears as "LC CurtainCall" window covering

## Usage

### Manual Control
- **Short Press**: Cycle through positions in 20% increments (0% → 20% → 40% → ... → 100% → 0%)
- **Long Press (3+ seconds)**: Factory reset - clears Zigbee settings and reboots

### Zigbee Control
- **Open**: Move curtain to 100% open position
- **Close**: Move curtain to 0% closed position
- **Position**: Set specific percentage (0-100%)
- **Stop**: Halt movement (limited support with current stepper library)

## Zigbee Device Information

- **Manufacturer**: LC
- **Model**: CurtainCall
- **Device Type**: Window Covering (Drapery)
- **Endpoint**: 10
- **Supported Features**: Lift control (up/down movement)

## Serial Monitor Output

The device provides detailed status information via serial output:
- Zigbee connection status
- Position changes and movements
- Button press detection
- Error messages and diagnostics

## Troubleshooting

- **Zigbee Connection Issues**: Check if your hub supports the device type and ensure pairing mode is active
- **Motor Not Moving**: Verify stepper motor connections and power supply
- **Inaccurate Positioning**: Calibrate the `TOTAL_STEPS_RANGE` value for your specific setup
- **Factory Reset**: Hold the manual button for 3+ seconds to reset Zigbee settings

## Customization

Key parameters that can be adjusted in the code:
- `MAX_LIFT_CM`: Maximum curtain travel distance
- `TOTAL_STEPS_RANGE`: Total steps for full curtain range
- `MOTOR_SPEED_RPM`: Stepper motor speed
- `POSITION_INCREMENT_PERCENT`: Manual button step size

## License

This project is open source. Feel free to modify and distribute according to your needs. 
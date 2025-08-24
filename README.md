# CurtainCall - Smart Curtain Controller v2.0

A precision smart curtain controller featuring Zigbee connectivity and magnetic position sensing for accurate, reliable curtain automation.

## Features

- **Precision Position Tracking**: AS5600 magnetic rotation sensor with 0.0879° resolution
- **Zigbee 3.0 Integration**: Full home automation compatibility with window covering cluster
- **Robust Motor Control**: BTS7960 high-current motor driver for smooth, powerful operation
- **Real-time Position Feedback**: Continuous monitoring with 20ms update intervals
- **Advanced Error Handling**: I2C recovery, spike detection, and safety limits
- **Multi-rotation Support**: Handles up to 5 full rotations (1800°) for long curtain runs
- **Auto-calibration**: Intelligent position recovery on power-up

## Bill of Materials (BOM)

### Core Components
| Component | Quantity | Description | Estimated Cost |
|-----------|----------|-------------|----------------|
| ESP32 DevKit | 1 | Main microcontroller with Zigbee 3.0 support | $8-12 |
| AS5600 Magnetic Encoder | 1 | 12-bit contactless position sensor | $3-5 |
| BTS7960 Motor Driver | 1 | 43A high-power motor controller | $8-12 |
| 12V DC Geared Motor | 1 | High-torque motor for curtain operation | $15-25 |
| Diametric Magnet | 1 | 6mm diameter for AS5600 sensor | $2-3 |

### Power & Protection
| Component | Quantity | Description | Estimated Cost |
|-----------|----------|-------------|----------------|
| 12V 3A Power Supply | 1 | Wall adapter or switching supply | $10-15 |
| LM2596 Buck Converter | 1 | 12V to 5V step-down (optional) | $3-5 |
| Fuse Holder & 5A Fuse | 1 | Motor circuit protection | $2-3 |
| TVS Diode (15V) | 2 | Motor voltage spike protection | $1-2 |

### Mechanical Hardware
| Component | Quantity | Description | Estimated Cost |
|-----------|----------|-------------|----------------|
| Motor Mounting Bracket | 1 | Custom or universal motor mount | $5-10 |
| Drive Pulley/Gear | 1 | Connects motor to curtain mechanism | $3-8 |
| Timing Belt/Chain | 1 | Power transmission (if required) | $5-10 |
| Shaft Coupling | 1 | Flexible coupling for sensor mounting | $3-5 |

### Electronics & Wiring
| Component | Quantity | Description | Estimated Cost |
|-----------|----------|-------------|----------------|
| Dupont Jumper Wires | 20 | Male-female and male-male connectors | $3-5 |
| Breadboard/PCB | 1 | Prototyping or permanent mounting | $2-8 |
| Terminal Blocks | 4 | Power and motor connections | $2-4 |
| Heat Shrink Tubing | 1 set | Wire protection and insulation | $3-5 |

**Total Estimated Cost: $75-140** (depending on motor choice and sourcing)

## Wiring Diagrams

### Circuit Schematic

```
                     CurtainCall v2.0 Circuit Diagram
    
    12V PSU ───[FUSE]───┬─── BTS7960 VCC ───┬─── M+ ───┐
        │               │                   │          │
        └─── Buck Conv ─┘                   │          │    ┌─── DC MOTOR ───┐
                 │                          │          │    │               │
                5V                          │        [TVS]  │               │
                                            │          │    │               │
    ┌─────────────── ESP32 DevKit ──────────┼──────────┼────┼───────────────┼───┐
    │                                       │          │    │               │   │
    │  GPIO 21 (SDA) ─────────────────────┐ │          │    │               │   │
    │  GPIO 22 (SCL) ─────────────────────┼─┼──────────┼────┼───────────────┼───┼─ GND
    │  GPIO  0 ───── BTS7960 L_EN         │ │          │    │               │   │
    │  GPIO  7 ───── BTS7960 R_EN         │ │          └──M-┘               │   │
    │  GPIO  6 ───── BTS7960 L_PWM        │ │                               │   │
    │  GPIO  5 ───── BTS7960 R_PWM        │ │                               │   │
    │                                     │ │         ┌─── AS5600 ──────────┐   │
    │  3.3V ──────────────────────────────┼─┼─────────┼─ VCC               │   │
    │  GND ───────────────────────────────┼─┼─────────┼─ GND               │   │
    └─────────────────────────────────────┼─┼─────────┼─────────────────────┼───┘
                                          │ │         │                     │
                                          │ └─────────┼─ SDA               │
                                          └───────────┼─ SCL               │
                                                      │                     │
                                              ┌───────┼─ MAGNET ────────────┘
                                              │       └─────────────────────┐
                                              │                             │
                                        Rotating Shaft/Pulley              │
                                              │                             │
                                              └─────── Curtain Rod ─────────┘

    Components:
    - ESP32: Main controller with Zigbee
    - AS5600: Magnetic rotation encoder  
    - BTS7960: High-current motor driver
    - TVS: Transient voltage suppressor
    - Buck Conv: 12V to 5V converter (optional)
```

### Power Distribution

```
12V Power Supply (3A)
│
├── 5A Fuse ──► BTS7960 Motor Driver (12V)
│                    │
│                    └── DC Motor (12V)
│
└── LM2596 Buck Converter (Optional)
         │
         └── 5V for BTS7960 logic (if needed)

ESP32 provides 3.3V for AS5600 sensor
```

### Pin Assignment Table

| ESP32 Pin | Function | Connected To | Signal Type |
|-----------|----------|--------------|-------------|
| GPIO 21 | I2C SDA | AS5600 SDA | Digital I2C |
| GPIO 22 | I2C SCL | AS5600 SCL | Digital I2C |
| GPIO 0 | Motor Control | BTS7960 L_EN | Digital Output |
| GPIO 7 | Motor Control | BTS7960 R_EN | Digital Output |
| GPIO 6 | Motor PWM | BTS7960 L_PWM | PWM Output |
| GPIO 5 | Motor PWM | BTS7960 R_PWM | PWM Output |
| 3.3V | Power | AS5600 VCC | Power Supply |
| GND | Ground | All GND pins | Common Ground |

## AS5600 Magnetic Encoder Setup

The AS5600 provides precise angular position measurement with contactless operation:

### Physical Installation
1. **Mount the sensor** securely to a stationary part of the curtain mechanism
2. **Attach the magnet** to the rotating element (motor shaft, pulley, or gear)
3. **Maintain 1-3mm air gap** between magnet and sensor for optimal performance
4. **Center alignment** is critical - magnet must be concentric with sensor
5. **Use diametric magnetization** for best linearity and accuracy

### Technical Specifications
- **I2C Address**: 0x36 (factory default, programmable)
- **Resolution**: 12-bit (4096 steps = 0.0879° per step)
- **Angular Range**: Full 360° rotation
- **Magnetic Field**: 30-100 mT optimal range
- **Update Rate**: 20ms intervals (50Hz) for I2C stability
- **Supply Voltage**: 3.3V from ESP32
- **Current Consumption**: 6.5mA typical

## Software Installation

### Required Libraries
1. **AS5600 Library**: `#include <AS5600.h>`
   - [AS5600 Library](https://github.com/RobTillaart/AS5600) by Rob Tillaart
2. **BTS7960 Motor Driver**: `#include <BTS7960.h>`
   - Available through Arduino Library Manager
3. **Zigbee Core**: Built into ESP32 Arduino Core v3.0+
   - `#include "ZigbeeCore.h"`
   - `#include "ep/ZigbeeWindowCovering.h"`

### Installation Steps
1. **Install ESP32 Arduino Core** (v3.0 or later for Zigbee support)
2. **Install required libraries** through Arduino IDE Library Manager
3. **Upload CurtainCall.ino** to your ESP32
4. **Configure your home automation** to recognize the new Zigbee device

### Configuration Parameters
```cpp
// Hardware configuration (CurtainCall.ino lines 24-38)
const int MOTOR_SPEED = 200;                    // PWM value (0-255)
const float TOTAL_ROTATIONS_0_TO_100 = 5.0;    // Total rotations for full travel
const float MAX_ANGLE_CHANGE_PER_MS = 15.0;    // Spike detection threshold
const unsigned long AS5600_READ_INTERVAL = 20;  // Sensor read interval (ms)
```

## Usage

### Basic Operation
The system tracks curtain position through absolute angle measurement:

- **0%**: Fully closed (0° absolute angle)
- **100%**: Fully open (1800° absolute angle = 5 full rotations)
- **Real-time tracking**: 20ms update intervals with spike detection
- **Multi-rotation support**: Handles up to 5 complete rotations
- **Auto-calibration**: Intelligent position recovery on startup

### Zigbee Integration
The device appears as a **Window Covering** in your Zigbee network with these commands:

| Zigbee Command | Function | Description |
|----------------|----------|-------------|
| `Open` | `openCurtain()` | Move to 100% open position |
| `Close` | `closeCurtain()` | Move to 0% closed position |
| `GoToLiftPercentage` | `moveToPosition(uint8_t)` | Move to specific percentage |
| `Stop` | `stopCurtain()` | Immediately stop movement |

### System Features
- **Precise Position Control**: 0.0879° resolution with AS5600 encoder
- **Safety Limits**: Automatic stop at 0% and 100% positions  
- **Error Recovery**: I2C communication recovery and spike filtering
- **Smooth Movement**: BTS7960 provides smooth acceleration/deceleration
- **Status Reporting**: Real-time position updates to Zigbee coordinator

### Initial Setup
1. **Power on** the device - it will join your Zigbee network automatically
2. **Manual positioning** - move curtain to known position (e.g., 50% open)
3. **Auto-calibration** - system calculates absolute position based on stored state
4. **Test operation** - use home automation to verify full range movement

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

### Power Issues
- **Device not starting**: Check 12V power supply capacity (minimum 2A)
- **Random resets**: Verify all ground connections are secure
- **Motor stops unexpectedly**: Check fuse, may need higher current rating

### AS5600 Sensor Issues
- **Position not updating**: Check I2C wiring (SDA/SCL crossed or loose)
- **Erratic readings**: Verify magnet alignment (must be centered over sensor)
- **Sensor timeouts**: System includes automatic I2C recovery after 5 failures
- **Wrong direction**: Adjust `MOTOR_DIRECTION_INVERTED` in code

### Motor Control Issues  
- **Motor not responding**: Verify BTS7960 enable pins and PWM connections
- **Wrong rotation direction**: Check motor wires or toggle direction setting
- **Jerky movement**: Reduce `MOTOR_SPEED` value or check mechanical binding
- **Overshooting target**: Adjust `DEGREES_PER_PERCENT` for your curtain travel

### Zigbee Connectivity
- **Device not joining network**: Ensure ESP32 Arduino Core v3.0+ is installed
- **Commands not working**: Check if device appears as "Window Covering" type
- **Position not updating**: Verify Zigbee coordinator supports position feedback

### Position Tracking
- **Incorrect position after power-up**: System auto-calibrates based on last known position
- **Drift over time**: Check for mechanical backlash in drive system
- **Sudden position jumps**: Spike detection active - check magnet mounting
- **Limited range**: Adjust `TOTAL_ROTATIONS_0_TO_100` for your curtain system

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
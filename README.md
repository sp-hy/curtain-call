# CurtainCall - Zigbee Smart Curtain Device

A Zigbee-enabled smart curtain device based on ESP32 microcontrollers. Features precise stepper motor control with step/direction signaling, persistent position memory that survives power cycles, and seamless integration with Zigbee home automation systems.

## Hardware Requirements

- **ESP32-C6 XIAO** or compatible ESP32 board with zigbee support
  - Recommended: https://www.aliexpress.com/w/wholesale-esp32-c6-xiao.html
- **Stepper Motor Driver**: Step/Dir compatible driver (e.g., A4988, DRV8825)
  - Recommended: https://www.aliexpress.com/item/1005004856826948.html
- **Power Supply**: For stepper motor and driver power
  - 24V DC
  - Recommended: https://www.aliexpress.com/item/1005007087317638.html
- **Buck Converter**: To step down PSU to 5V/3.3V for ESP32 power
  - Input: 24V DC if using above PSU
  - Output: 5V or 3.3V depending on ESP32 requirements
- **Blind Chain**
  - Bead diameter 4.5mm
  - Recommended: https://www.amazon.com.au/HASAYAKI-Beaded-String-Curtain-Connectors/dp/B0C3CXZ96H

## Pin Connections

| Function | Pin | Description |
|----------|-----|-------------|
| Step Signal | 0 | Stepper driver STEP/PUL+ |
| Direction | 1 | Stepper driver DIR+ |
| Enable | 2 | Stepper driver ENA+ |
| RF Power | 3 | RF switch power control |
| Button | 9 | External button (factory reset only) |
| Antenna Select | 14 | External antenna select |
| Ground | GND | Common ground to stepper driver PUL-, DIR-, ENA- |

## Configuration

Key variables in the code that control curtain behavior:

- **STEPS_FOR_FULL_TRAVEL**: Total steps for complete curtain range (default: 2000)
- **STEP_DELAY_US**: Delay between motor steps in microseconds (default: 1000)

Adjust these values based on your specific curtain setup and motor characteristics.


## Setup

1. Connect stepper driver to pins 0, 1, 2 and common GND
2. Upload code to ESP32. If not using the recommended Xiao Variant, remove the code related to external antenna.
3. Pair with Zigbee hub (appears as "LC CurtainCall")
4. Control via home automation (Zigbee commands)

Device automatically saves position and restores on restart.

## 3D Parts
Parts for the motor housing and shaft are included as a FreeCAD project

<img width="1672" height="1240" alt="{85E4A3B8-E166-47C7-9F0F-4D093E4323D5}" src="https://github.com/user-attachments/assets/eda60e0a-6398-4d39-a2c0-4207fc55d064" />


## Factory Reset

To perform a factory reset and clear Zigbee pairing:
1. Connect a momentary button between GPIO 9 and GND
2. Press and hold the button for 3+ seconds
3. Release the button - the device will reset its Zigbee configuration
4. Re-pair with your Zigbee hub as a new device 

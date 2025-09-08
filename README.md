# Custom-I2C-Driver
Project to familiarize myself with the I2C protocol. This project uses an Arduino Uno along with a MCP4725 12-bit digital-to-analog converter. Debugging using a multimeter.

# Custom-I2C-Driver

Project to familiarize myself with the I2C protocol. This project uses an Arduino Uno along with a MCP4725 12-bit digital-to-analog converter. Debugging using a multimeter.

## Components Required
- Arduino Uno R3
- MCP4725 12-bit DAC breakout board
- Breadboard and jumper wires
- Multimeter (for voltage verification)

## Final Wiring Configuration
```
Arduino Uno    →    MCP4725 DAC
────────────────────────────────
Pin 2 (SDA)    →    SDA
Pin 3 (SCL)    →    SCL  
5V             →    VDD
GND            →    GND
```

## MCP4725 Configuration
- I2C Address: 0x60 (A0 tied to GND)
- Resolution: 12-bit (0-4095)
- Output Range: 0V to VDD (5V)
- Custom I2C Speed: 100kHz (10μs timing)

## Implementation

First implemented I2C protocol using wire library, then created custom functions:

- `custom_init()` - Pin initialization and idle state setup
- `custom_start()` - START condition generation
- `custom_stop()` - STOP condition generation
- `custom_write_bit()` - Individual bit transmission with timing control
- `custom_write_byte()` - 8-bit data transmission with ACK detection
- `custom_read_ack()` - Acknowledgment reading with pin direction control
- `custom_test()` - Device presence verification
- `custom_set_voltage()` - Complete MCP4725 communication protocol

## Performance Results

Both Wire library (Phase 1) and custom driver (Phase 3) produced identical voltage outputs

## Custom Driver Features

- No dependency on Arduino Wire library
- Complete manual control of SDA/SCL timing
- Bit-banging implementation using standard GPIO pins
- Comprehensive error handling and acknowledgment detection
- Protocol-compliant START/STOP condition generation

## Skills Demonstrated

- Low-level hardware protocol implementation
- Bit manipulation and timing-critical programming
- GPIO pin control and electrical signal management
- Serial communication protocol design
- Embedded systems debugging methodology

## Custom I2C Driver Functions

```cpp
void custom_init()              // Pin setup and initialization
void custom_start()             // I2C START condition
void custom_stop()              // I2C STOP condition  
void custom_write_bit()         // Single bit transmission
bool custom_write_byte()        // Byte transmission with ACK
bool custom_read_ack()          // Acknowledgment detection
bool custom_test()              // Device connectivity test
bool custom_set_voltage()       // MCP4725 DAC control
```

## Learning Outcomes

After this project I now have a complete understanding of:
- I2C protocol from hardware to software
- Timing-critical embedded programming
- Systematic debugging methodology
- GPIO bit-banging techniques
- Serial communication protocol fundamentals

## Project Details

**Author:** Diego Klish  
**Project Duration:** Three days  
**Technologies:** Arduino C++, I2C Protocol, GPIO Bit-banging, Digital-to-Analog Conversion  
**Hardware:** Arduino Uno, MCP4725 12-bit DAC
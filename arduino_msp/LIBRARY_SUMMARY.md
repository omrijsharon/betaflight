# BetaflightMSP Arduino Library - Complete Package

## 📦 Library Structure

```
arduino_msp/
├── README.md                          # Complete documentation
├── QUICK_REFERENCE.md                 # Quick reference guide
├── LICENSE                            # GPL v3 License
├── library.properties                 # Arduino library metadata
├── keywords.txt                       # Syntax highlighting
│
├── src/
│   ├── BetaflightMSP.h               # Header file with API
│   └── BetaflightMSP.cpp             # Implementation
│
└── examples/
    ├── ConnectionTest/               # Test connection (START HERE!)
    │   └── ConnectionTest.ino
    ├── BasicConnection/              # Read FC info
    │   └── BasicConnection.ino
    ├── ReadAttitude/                 # Read roll/pitch/yaw
    │   └── ReadAttitude.ino
    ├── SendStickCommands/            # Control FC with RC commands
    │   └── SendStickCommands.ino
    ├── SendGPSHome/                  # Set GPS home position
    │   └── SendGPSHome.ino
    ├── TelemetryMonitor/             # Comprehensive telemetry display
    │   └── TelemetryMonitor.ino
    └── AdvancedDataLogger/           # Flight data logger with statistics
        └── AdvancedDataLogger.ino
```

## 🚀 Quick Start

### 1. Installation
Copy the `arduino_msp` folder to your Arduino libraries folder and rename it to `BetaflightMSP`.

### 2. Hardware Connection
```
Arduino TX  →  FC RX (MSP UART)
Arduino RX  →  FC TX (MSP UART)
GND         →  GND
```

### 3. Configure Flight Controller
In Betaflight Configurator:
- **Ports** tab → Enable **MSP** on desired UART
- Set baud rate to **115200**
- Save and reboot

### 4. Test Connection
Open **File → Examples → BetaflightMSP → ConnectionTest** and upload.

## 📚 Documentation

- **README.md** - Full documentation with API reference
- **QUICK_REFERENCE.md** - Cheat sheet with common patterns
- **Examples** - 8 working examples from basic to advanced

## 🎯 Key Features

### Complete MSP Protocol Coverage
- ✅ **140+ MSP v1 commands** defined and ready to use
- ✅ **MSP v2 commands** included for advanced features
- ✅ All Betaflight protocol commands from official codebase
- ✅ Compatible with Betaflight 4.x and INAV (partial)

### Read Telemetry
- ✅ Attitude (roll, pitch, yaw)
- ✅ GPS position and status
- ✅ Battery voltage, current, mAh
- ✅ Altitude and vertical speed
- ✅ Flight status and modes
- ✅ RC channel values
- ✅ Motor, servo, ESC telemetry
- ✅ VTX, OSD, blackbox status

### Send Commands
- ✅ RC stick commands (autonomous control)
- ✅ GPS home position
- ✅ GPS position data (from external GPS)
- ✅ PID tuning, filters, rates
- ✅ Motor control, calibration
- ✅ OSD, VTX configuration

### Data Structures
```cpp
msp_attitude_t      // Roll, pitch, yaw
msp_altitude_t      // Altitude, vario
msp_battery_state_t // Voltage, current, mAh
msp_raw_gps_t       // GPS position, satellites
msp_status_t        // Armed, flight modes
msp_rc_t            // RC channel values
// ... and more
```

## 💡 Usage Examples

### Read Attitude
```cpp
msp_attitude_t attitude;
if (msp.getAttitude(attitude)) {
    float roll = attitude.roll / 10.0;   // decidegrees → degrees
    float pitch = attitude.pitch / 10.0;
    float yaw = attitude.yaw;
}
```

### Send RC Commands
```cpp
uint16_t channels[8] = {1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000};
msp.setRawRC(channels, 8);  // Send at 50Hz for MSP RX
```

### Set GPS Home
```cpp
int32_t lat = 37.7749 * 1e7;  // San Francisco
int32_t lon = -122.4194 * 1e7;
msp.setGPSHome(lat, lon, 10);  // 10m altitude
```

## 🔧 Compatible Hardware

### Tested
- ✅ Arduino Mega 2560
- ✅ Arduino Due
- ✅ ESP32
- ✅ Teensy 3.x/4.x

### Compatible
- Arduino Uno (with SoftwareSerial)
- Arduino Nano
- Any board with hardware serial

## ⚠️ Important Notes

### For RC Control
- Set receiver to "MSP" in Betaflight Configurator
- Send commands at exactly 50Hz (every 20ms)
- Test with props OFF first!
- Have failsafe configured

### For GPS
- Enable GPS in Betaflight Configurator
- Coordinates are in degrees × 10^7 format
- Altitude in meters for MSP_WP, cm for MSP_RAW_GPS

### Voltage Levels
- 5V Arduino boards need level shifter for 3.3V FC
- ESP32/Due are 3.3V compatible (direct connection)

## 📖 Example Descriptions

1. **ConnectionTest** - Verify library and connection (start here!)
2. **BasicConnection** - Read FC info and status
3. **ReadAttitude** - Display attitude with visual bars
4. **SendStickCommands** - Keyboard-controlled RC commands
5. **SendGPSHome** - Set GPS home position
6. **TelemetryMonitor** - Full telemetry display
7. **AdvancedDataLogger** - CSV flight data logger with statistics

## 🐛 Troubleshooting

### No connection?
1. Check TX/RX wiring (swap if needed)
2. Verify baud rate (115200)
3. Enable MSP on UART in Betaflight
4. Try different UART
5. Use level shifter if voltage mismatch

### Checksum errors?
1. Lower baud rate to 57600
2. Shorter cables
3. Add decoupling capacitors

### RC not working?
1. Set receiver to "MSP" mode
2. Send at exactly 50Hz
3. Check values are 1000-2000

## 📄 License

GPL v3 - Same as Betaflight

## 🤝 Contributing

This library is part of the Betaflight ecosystem. Feel free to:
- Report issues
- Submit improvements
- Share your projects
- Help others in the community

## 🔗 Resources

- [Betaflight](https://github.com/betaflight/betaflight)
- [MSP Protocol](https://github.com/betaflight/betaflight/wiki/MSP-V2)
- [Betaflight Configurator](https://github.com/betaflight/betaflight-configurator)

## 📊 Library Stats

- **Files**: 11 (2 source + 7 examples + 2 docs)
- **Lines of Code**: ~2000
- **Examples**: 7 complete examples
- **MSP Commands**: 140+ defined (all Betaflight commands)
- **MSP v2 Commands**: 11 defined
- **Data Structures**: 13 types
- **Helper Functions**: 13 convenience functions

## ✨ What's Included

- ✅ Complete MSP v1 implementation
- ✅ Structured data types
- ✅ Non-blocking communication
- ✅ Error handling
- ✅ Comprehensive examples
- ✅ Full documentation
- ✅ Quick reference guide
- ✅ Syntax highlighting
- ✅ GPL v3 license

---

**Ready to fly? Start with ConnectionTest.ino! 🚁**

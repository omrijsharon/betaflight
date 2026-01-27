# BetaflightMSP - Arduino Library

Arduino library for communicating with Betaflight flight controllers using the MSP (MultiWii Serial Protocol).

## Features

- ✅ Complete MSP v1 protocol support (140+ commands)
- ✅ MSP v2 command definitions included
- ✅ Read telemetry data (attitude, GPS, battery, altitude, etc.)
- ✅ Send commands (RC channels, GPS home position, etc.)
- ✅ Compatible with Betaflight 4.x and later
- ✅ Easy-to-use API with structured data types
- ✅ Non-blocking communication
- ✅ Comprehensive examples
- ✅ All Betaflight MSP commands defined

## Installation

### Arduino IDE

1. Download this library as a ZIP file
2. In Arduino IDE, go to **Sketch** → **Include Library** → **Add .ZIP Library**
3. Select the downloaded ZIP file
4. Restart Arduino IDE

### PlatformIO

Add to your `platformio.ini`:

```ini
lib_deps =
    BetaflightMSP
```

### Manual Installation

1. Copy the `arduino_msp` folder to your Arduino libraries folder:
   - Windows: `Documents\Arduino\libraries\`
   - Mac: `~/Documents/Arduino/libraries/`
   - Linux: `~/Arduino/libraries/`
2. Rename the folder to `BetaflightMSP`
3. Restart Arduino IDE

## Hardware Setup

### Wiring

Connect your Arduino to the flight controller's MSP port:

| Arduino | Flight Controller |
|---------|------------------|
| TX      | RX (MSP UART)    |
| RX      | TX (MSP UART)    |
| GND     | GND              |

**Important Notes:**
- Make sure the UART on the FC is configured for MSP in Betaflight Configurator
- Default baud rate is usually 115200
- For 5V Arduino boards with 3.3V flight controllers, use a level shifter
- ESP32 and Arduino Due are 3.3V compatible

### Flight Controller Configuration

In Betaflight Configurator:

1. Go to **Ports** tab
2. Find the UART you're using
3. Set **Configuration/MSP** to **ON**
4. Set baud rate to **115200** (or your preferred rate)
5. Click **Save and Reboot**

## Quick Start

```cpp
#include <BetaflightMSP.h>

BetaflightMSP msp;

void setup() {
    Serial.begin(115200);
    Serial1.begin(115200);  // Connect to FC
    msp.begin(Serial1);
    
    // Get attitude data
    msp_attitude_t attitude;
    if (msp.getAttitude(attitude)) {
        Serial.print("Roll: ");
        Serial.println(attitude.roll / 10.0);  // Convert decidegrees to degrees
    }
}

void loop() {
    // Your code here
}
```

## API Reference

### Initialization

```cpp
BetaflightMSP msp;
msp.begin(Serial1);  // Pass any Stream object (Serial1, Serial2, etc.)
```

### Reading Telemetry

#### Get API Version
```cpp
msp_api_version_t data;
if (msp.getApiVersion(data)) {
    // data.protocolVersion
    // data.apiVersionMajor
    // data.apiVersionMinor
}
```

#### Get Flight Controller Info
```cpp
msp_fc_variant_t variant;
msp.getFcVariant(variant);  // Returns firmware identifier (e.g., "BTFL")

msp_fc_version_t version;
msp.getFcVersion(version);  // Returns version numbers

msp_board_info_t board;
msp.getBoardInfo(board);    // Returns board information

msp_name_t name;
msp.getName(name);          // Returns craft name
```

#### Get Attitude (Roll, Pitch, Yaw)
```cpp
msp_attitude_t attitude;
if (msp.getAttitude(attitude)) {
    float roll = attitude.roll / 10.0;   // decidegrees → degrees
    float pitch = attitude.pitch / 10.0; // decidegrees → degrees
    float yaw = attitude.yaw;            // degrees
}
```

#### Get Altitude
```cpp
msp_altitude_t altitude;
if (msp.getAltitude(altitude)) {
    float altMeters = altitude.estimatedAltitude / 100.0; // cm → meters
    float varioMs = altitude.vario / 100.0;               // cm/s → m/s
}
```

#### Get Battery Status
```cpp
msp_battery_state_t battery;
if (msp.getBatteryState(battery)) {
    float voltage = battery.voltage16 / 100.0;  // 0.01V → V
    float current = battery.amperage / 100.0;   // 0.01A → A
    uint16_t mAhUsed = battery.mAhDrawn;
    uint8_t cells = battery.cellCount;
}
```

#### Get GPS Data
```cpp
msp_raw_gps_t gps;
if (msp.getRawGPS(gps)) {
    float lat = gps.lat / 1e7;           // degrees
    float lon = gps.lon / 1e7;           // degrees
    float altMeters = gps.altCm / 100.0; // cm → meters
    uint8_t satellites = gps.numSat;
    uint8_t fixType = gps.fixType;       // 0=no fix, 1=2D, 2=3D
}

msp_comp_gps_t compGps;
if (msp.getCompGPS(compGps)) {
    uint16_t distanceToHome = compGps.distanceToHome;  // meters
    uint16_t directionToHome = compGps.directionToHome; // degrees
}
```

#### Get Flight Status
```cpp
msp_status_t status;
if (msp.getStatus(status)) {
    uint16_t cycleTime = status.cycleTime;
    bool armed = status.flightModeFlags & 0x01;
    uint8_t profile = status.configProfileIndex;
}
```

#### Get RC Channels
```cpp
msp_rc_t rc;
if (msp.getRC(rc)) {
    for (int i = 0; i < 18; i++) {
        uint16_t channelValue = rc.channels[i];  // 1000-2000
    }
}
```

### Sending Commands

#### Send RC Stick Commands
```cpp
uint16_t channels[8] = {1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000};
// Order: Roll, Pitch, Throttle, Yaw, AUX1-4
msp.setRawRC(channels, 8);

// Must be sent at 50Hz (every 20ms) for MSP RX to work
```

**⚠️ Safety Warning:** Sending RC commands will override your transmitter when MSP RX is active. Always test with props off!

#### Set GPS Home Position
```cpp
int32_t lat = 37.7749 * 1e7;  // Latitude in degrees * 1e7
int32_t lon = -122.4194 * 1e7; // Longitude in degrees * 1e7
uint16_t altMeters = 10;        // Altitude in meters

msp.setGPSHome(lat, lon, altMeters);
```

#### Send GPS Position
```cpp
uint8_t fixType = 3;      // 3 = 3D fix
uint8_t numSat = 12;      // Number of satellites
int32_t lat = 37.7749 * 1e7;
int32_t lon = -122.4194 * 1e7;
int16_t altM = 10;
uint16_t speedCmS = 0;    // Ground speed in cm/s

msp.setRawGPS(fixType, numSat, lat, lon, altM, speedCmS);
```

### Low-Level API

For advanced users who need direct MSP command access:

```cpp
// Send request and wait for response
uint8_t payload[10] = {0};
if (msp.request(MSP_ATTITUDE, payload, 0, 500)) {
    uint8_t* response = msp.getPayload();
    uint8_t size = msp.getPayloadSize();
}

// Send command without waiting for response
msp.command(MSP_SET_RAW_RC, payload, payloadSize);

// Process incoming data in loop
msp.update();
```

### Using Any MSP Command

The library includes **ALL 140+ MSP commands** from Betaflight! While convenience functions are provided for common commands, you can use ANY command directly:

```cpp
// Example: Read motor telemetry (MSP_MOTOR_TELEMETRY)
if (msp.request(MSP_MOTOR_TELEMETRY, nullptr, 0)) {
    uint8_t* data = msp.getPayload();
    // Parse motor telemetry data according to MSP protocol
}

// Example: Read VTX table band (MSP_VTXTABLE_BAND)
uint8_t bandNumber = 1;
if (msp.request(MSP_VTXTABLE_BAND, &bandNumber, 1)) {
    // Parse VTX band data
}

// Example: Send simplified tuning (MSP_SET_SIMPLIFIED_TUNING)
uint8_t tuningData[10] = {/* your tuning values */};
msp.command(MSP_SET_SIMPLIFIED_TUNING, tuningData, sizeof(tuningData));
```

**Available command categories:**
- Configuration (battery, mixer, RX, features, etc.)
- Telemetry (motors, servos, IMU, GPS, etc.)
- Tuning (PIDs, filters, rates, simplified tuning)
- OSD (config, canvas, video, displayport)
- VTX (config, tables, power levels)
- Storage (blackbox, dataflash, SD card)
- Navigation (GPS rescue, waypoints)
- And more!

See the header file for the complete list of 140+ MSP commands.

## Examples

### 1. BasicConnection
Connect to FC and read basic information (firmware version, board info, etc.)

### 2. ReadAttitude
Read and display roll, pitch, and yaw angles in real-time with visual representation.

### 3. SendStickCommands
Send RC stick commands to control the flight controller. Includes keyboard control interface.

**⚠️ Props off! Test safely!**

### 4. SendGPSHome
Set the GPS home position manually or relay external GPS data.

### 5. TelemetryMonitor
Comprehensive telemetry monitoring displaying battery, GPS, attitude, altitude, and flight status.

## Supported Boards

### Tested
- ✅ Arduino Mega 2560
- ✅ Arduino Due
- ✅ ESP32
- ✅ Teensy 3.x/4.x

### Should Work
- Arduino Uno (with SoftwareSerial)
- Arduino Nano
- Any board with hardware serial ports

## MSP Protocol Support

This library implements **MSP v1** protocol which is compatible with:
- Betaflight 4.x
- Betaflight 3.x (most features)
- INAV (partial compatibility)
- Cleanflight (partial compatibility)

## Data Format Reference

### Angle Units
- **Roll/Pitch**: decidegrees (1/10 degree), range: -1800 to +1800 (-180° to +180°)
- **Yaw**: degrees, range: 0 to 359

### GPS Units
- **Latitude/Longitude**: degrees × 10^7
- **Altitude**: centimeters (MSP_RAW_GPS) or meters (MSP_WP)
- **Speed**: cm/s

### Battery Units
- **Voltage**: 0.01V (e.g., 1234 = 12.34V)
- **Current**: 0.01A (e.g., 512 = 5.12A)
- **Capacity**: mAh

### RC Channels
- **Range**: 1000 to 2000 (1500 = center)
- **Update Rate**: Minimum 50Hz (20ms) for MSP RX

## Troubleshooting

### No Response from Flight Controller

1. **Check wiring**: Ensure TX→RX and RX→TX are correct
2. **Check baud rate**: Must match FC configuration (usually 115200)
3. **Check UART configuration**: MSP must be enabled in Betaflight Configurator
4. **Check voltage levels**: Use level shifter if needed (5V ↔ 3.3V)
5. **Try different UART**: Some UARTs may be used for other purposes

### Checksum Errors

1. Ensure baud rate matches exactly
2. Check for electrical noise on serial lines
3. Add decoupling capacitors near Arduino
4. Use shorter cables
5. Try lower baud rate (57600)

### RC Commands Not Working

1. Configure receiver protocol to "MSP" in Betaflight Configurator
2. Send commands at 50Hz (every 20ms)
3. Check that channels are in valid range (1000-2000)
4. Verify failsafe settings

### GPS Home Not Setting

1. Ensure GPS is enabled on FC
2. Check that coordinates are in correct format (degrees × 1e7)
3. Verify home position in Betaflight Configurator GPS tab

## Performance Tips

- Call `msp.update()` frequently in `loop()` for non-blocking operation
- Don't request data too frequently (max ~100Hz per command)
- Use appropriate timeout values (default 500ms)
- For high-speed telemetry, request different data types in rotation

## Contributing

Contributions are welcome! Please:
1. Fork the repository
2. Create a feature branch
3. Test your changes thoroughly
4. Submit a pull request

## License

This library is licensed under the GNU General Public License v3.0, consistent with Betaflight.

## References

- [Betaflight MSP Protocol](https://github.com/betaflight/betaflight/wiki/MSP-V2)
- [Betaflight GitHub](https://github.com/betaflight/betaflight)
- [MultiWii Serial Protocol](http://www.multiwii.com/wiki/index.php?title=Multiwii_Serial_Protocol)

## Changelog

### Version 1.0.0 (2025-11-27)
- Initial release
- MSP v1 protocol support
- Read telemetry functions
- Send command functions
- Comprehensive examples

## Support

For issues, questions, or suggestions:
- Open an issue on GitHub
- Check existing examples
- Review Betaflight Configurator for comparison

---

**Made with ❤️ for the drone community**

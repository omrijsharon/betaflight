# BetaflightMSP Quick Reference

## Quick Setup

```cpp
#include <BetaflightMSP.h>

BetaflightMSP msp;

void setup() {
    Serial1.begin(115200);  // FC connection
    msp.begin(Serial1);
}
```

## Common Commands Cheat Sheet

### Reading Data

| Function | Returns | Units | Example |
|----------|---------|-------|---------|
| `getAttitude()` | Roll, Pitch, Yaw | decideg, decideg, deg | `attitude.roll / 10.0` |
| `getAltitude()` | Altitude, Vario | cm, cm/s | `altitude.estimatedAltitude / 100.0` |
| `getBatteryState()` | Voltage, Current, mAh | 0.01V, 0.01A, mAh | `battery.voltage16 / 100.0` |
| `getRawGPS()` | Lat, Lon, Alt, etc | deg×1e7, deg×1e7, cm | `gps.lat / 1e7` |
| `getStatus()` | Armed, modes, etc | flags | `status.flightModeFlags & 0x01` |

### Sending Commands

| Function | Purpose | Update Rate | Example |
|----------|---------|-------------|---------|
| `setRawRC()` | Send RC channels | 50Hz (20ms) | `msp.setRawRC(channels, 8)` |
| `setGPSHome()` | Set home position | Once | `msp.setGPSHome(lat, lon, alt)` |
| `setRawGPS()` | Send GPS data | 1-10Hz | `msp.setRawGPS(fix, sats, lat, lon, alt, spd)` |

## Unit Conversions

### Angles
```cpp
// Decidegrees to degrees
float degrees = decidegrees / 10.0;

// Degrees to decidegrees
int16_t decidegrees = degrees * 10;
```

### GPS Coordinates
```cpp
// Degrees to MSP format
int32_t mspCoord = degrees * 1e7;

// MSP format to degrees
float degrees = mspCoord / 1e7;
```

### Altitude
```cpp
// Centimeters to meters
float meters = centimeters / 100.0;

// Meters to centimeters
int32_t centimeters = meters * 100;
```

### Battery
```cpp
// MSP voltage to volts
float volts = voltage16 / 100.0;

// MSP current to amps
float amps = amperage / 100.0;
```

### RC Channels
```cpp
// Standard range: 1000-2000
// Center: 1500
uint16_t channel = 1500;  // Centered stick
```

## Common Patterns

### Reading Attitude at 10Hz
```cpp
void loop() {
    static unsigned long lastRead = 0;
    if (millis() - lastRead >= 100) {  // 100ms = 10Hz
        lastRead = millis();
        
        msp_attitude_t attitude;
        if (msp.getAttitude(attitude)) {
            // Use attitude data
        }
    }
}
```

### Sending RC at 50Hz (Required for MSP RX)
```cpp
void loop() {
    static unsigned long lastSend = 0;
    if (millis() - lastSend >= 20) {  // 20ms = 50Hz
        lastSend = millis();
        msp.setRawRC(channels, 8);
    }
}
```

### Detecting Arm/Disarm
```cpp
static bool wasArmed = false;
msp_status_t status;
if (msp.getStatus(status)) {
    bool isArmed = status.flightModeFlags & 0x01;
    
    if (isArmed && !wasArmed) {
        // Just armed
    }
    if (!isArmed && wasArmed) {
        // Just disarmed
    }
    wasArmed = isArmed;
}
```

### GPS Fix Check
```cpp
msp_raw_gps_t gps;
if (msp.getRawGPS(gps)) {
    if (gps.fixType > 0 && gps.numSat >= 6) {
        // Good GPS fix
        float lat = gps.lat / 1e7;
        float lon = gps.lon / 1e7;
    }
}
```

### Battery Warning
```cpp
msp_battery_state_t battery;
if (msp.getBatteryState(battery)) {
    float voltage = battery.voltage16 / 100.0;
    float cellVoltage = voltage / battery.cellCount;
    
    if (cellVoltage < 3.5) {
        // Warning: Low battery
    }
    if (cellVoltage < 3.3) {
        // Critical: Land now!
    }
}
```

## Flight Controller Configuration

### In Betaflight Configurator

1. **Ports Tab**
   - Select UART (e.g., UART3)
   - Enable "Configuration/MSP"
   - Set baud rate: 115200
   - Save and reboot

2. **For MSP RX (Stick Control)**
   - Configuration Tab → Receiver
   - Set "Receiver" to "MSP"
   - Save and reboot

3. **For GPS**
   - Configuration Tab → GPS
   - Enable GPS
   - Configure GPS rescue if needed

## Troubleshooting

### No Response
- Check TX/RX connections (swap if needed)
- Verify baud rate (115200)
- Check MSP is enabled on UART
- Try different UART

### Checksum Errors
- Reduce baud rate to 57600
- Shorten cables
- Add decoupling capacitors

### RC Not Working
- Set receiver to "MSP" in Betaflight
- Send at exactly 50Hz
- Check channel values (1000-2000)

### GPS Not Working
- Enable GPS in Betaflight
- Check coordinate format (×1e7)
- Verify in GPS tab

## Hardware Recommendations

### Voltage Levels
- **Arduino Mega/Uno/Nano**: Use level shifter (5V → 3.3V)
- **ESP32/Due**: Direct connection (3.3V compatible)
- **Teensy**: Direct connection

### Recommended Boards
- **Best**: ESP32 (fast, 3.3V, multiple UARTs)
- **Good**: Arduino Mega (multiple UARTs)
- **OK**: Arduino Uno (limited to 1 UART via SoftwareSerial)

### Serial Ports
- **Arduino Mega**: Serial1, Serial2, Serial3
- **ESP32**: Serial1, Serial2 (configurable pins)
- **Arduino Due**: Serial1, Serial2, Serial3
- **Teensy**: Serial1, Serial2, Serial3, etc.

## Performance Guidelines

- Maximum request rate: ~100Hz per command
- Recommended rate: 10-20Hz for telemetry
- RC commands: Must be 50Hz for MSP RX
- Timeout: 500ms default (adjustable)

## MSP Command Reference

### 140+ Commands Available!

The library includes **ALL** MSP commands from Betaflight. Here are the most commonly used:

#### Read Commands (Telemetry)
- `MSP_API_VERSION (1)`: API version
- `MSP_FC_VARIANT (2)`: Firmware identifier
- `MSP_FC_VERSION (3)`: Firmware version
- `MSP_BOARD_INFO (4)`: Board information
- `MSP_STATUS (101)`: Flight status & modes
- `MSP_RAW_IMU (102)`: Raw sensor data (acc, gyro, mag)
- `MSP_ATTITUDE (108)`: Roll/pitch/yaw
- `MSP_ALTITUDE (109)`: Altitude/vario
- `MSP_ANALOG (110)`: Battery voltage/current
- `MSP_RAW_GPS (106)`: GPS position/satellites
- `MSP_COMP_GPS (107)`: Distance/direction to home
- `MSP_BATTERY_STATE (130)`: Detailed battery state
- `MSP_MOTOR (104)`: Motor values
- `MSP_SERVO (103)`: Servo positions
- `MSP_RC (105)`: RC channel values
- `MSP_MOTOR_TELEMETRY (139)`: ESC telemetry (RPM, temp)
- `MSP_ESC_SENSOR_DATA (134)`: 32-bit ESC data

#### Configuration Read Commands
- `MSP_PID (112)`: PID values
- `MSP_RC_TUNING (111)`: RC rates
- `MSP_FILTER_CONFIG (92)`: Filter settings
- `MSP_ADVANCED_CONFIG (90)`: Advanced settings
- `MSP_SENSOR_CONFIG (96)`: Sensor configuration
- `MSP_GPS_CONFIG (132)`: GPS settings
- `MSP_COMPASS_CONFIG (133)`: Compass settings
- `MSP_MIXER_CONFIG (42)`: Mixer settings
- `MSP_RX_CONFIG (44)`: Receiver config
- `MSP_ARMING_CONFIG (61)`: Arming settings
- `MSP_FAILSAFE_CONFIG (75)`: Failsafe config

#### OSD & VTX Commands
- `MSP_OSD_CONFIG (84)`: OSD configuration
- `MSP_OSD_CANVAS (189)`: OSD canvas size
- `MSP_VTX_CONFIG (88)`: VTX settings
- `MSP_VTXTABLE_BAND (137)`: VTX band data
- `MSP_VTXTABLE_POWERLEVEL (138)`: VTX power levels

#### Storage Commands
- `MSP_BLACKBOX_CONFIG (80)`: Blackbox settings
- `MSP_DATAFLASH_SUMMARY (70)`: Dataflash info
- `MSP_SDCARD_SUMMARY (79)`: SD card status

#### Write Commands (Control & Config)
- `MSP_SET_RAW_RC (200)`: Send RC channels
- `MSP_SET_RAW_GPS (201)`: Send GPS data
- `MSP_WP (118)`: Set GPS home position
- `MSP_SET_PID (202)`: Set PID values
- `MSP_SET_RC_TUNING (204)`: Set RC rates
- `MSP_SET_FILTER_CONFIG (93)`: Set filter config
- `MSP_SET_ADVANCED_CONFIG (91)`: Set advanced config
- `MSP_ACC_CALIBRATION (205)`: Calibrate accelerometer
- `MSP_MAG_CALIBRATION (206)`: Calibrate magnetometer
- `MSP_RESET_CONF (208)`: Reset configuration
- `MSP_SET_MOTOR (214)`: Set motor test values
- `MSP_REBOOT (68)`: Reboot flight controller

#### Simplified Tuning
- `MSP_SIMPLIFIED_TUNING (140)`: Get simplified tuning
- `MSP_SET_SIMPLIFIED_TUNING (141)`: Set simplified tuning
- `MSP_CALCULATE_SIMPLIFIED_PID (142)`: Calculate PID values
- `MSP_CALCULATE_SIMPLIFIED_GYRO (143)`: Calculate gyro filters
- `MSP_CALCULATE_SIMPLIFIED_DTERM (144)`: Calculate D-term filters

#### MSP V2 Commands
- `MSP2_SENSOR_GPS (0x1F03)`: MSP v2 GPS data
- `MSP2_GET_VTX_DEVICE_STATUS (0x3004)`: VTX device status
- `MSP2_GET_OSD_WARNINGS (0x3005)`: OSD warning text
- `MSP2_SEND_DSHOT_COMMAND (0x3003)`: Send DShot command

**See BetaflightMSP.h for the complete list of 140+ commands!**

## Status Flags

### Flight Mode Flags (flightModeFlags)
- Bit 0: ARMED
- Bit 1: ANGLE_MODE
- Bit 2: HORIZON_MODE
- Bit 3: MAG (heading hold)
- Bit 4: BARO (altitude hold)
- Bit 5: GPS_HOME
- Bit 6: GPS_HOLD

### Sensor Flags (sensor)
- Bit 0: ACC
- Bit 1: BARO
- Bit 2: MAG
- Bit 3: GPS
- Bit 4: SONAR

## Resources

- [Full Documentation](README.md)
- [Betaflight Wiki](https://github.com/betaflight/betaflight/wiki)
- [MSP Protocol](https://github.com/betaflight/betaflight/wiki/MSP-V2)

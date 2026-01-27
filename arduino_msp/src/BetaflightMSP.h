/*
 * BetaflightMSP - Arduino library for communicating with Betaflight flight controllers
 * 
 * This library implements the MSP (MultiWii Serial Protocol) used by Betaflight.
 * Supports MSP v1 protocol for reading telemetry and sending commands.
 * 
 * Compatible with Betaflight 4.x and later
 * 
 * Author: Arduino MSP Library
 * License: GPL v3
 */

#ifndef BETAFLIGHT_MSP_H
#define BETAFLIGHT_MSP_H

#include <Arduino.h>

// MSP Protocol Version
#define MSP_PROTOCOL_VERSION 0

// API Version
#define API_VERSION_MAJOR               1
#define API_VERSION_MINOR               46

// MSP V1 Commands (from msp_protocol.h)
// Basic info commands
#define MSP_API_VERSION                 1    // out message
#define MSP_FC_VARIANT                  2    // out message
#define MSP_FC_VERSION                  3    // out message
#define MSP_BOARD_INFO                  4    // out message
#define MSP_BUILD_INFO                  5    // out message

#define MSP_NAME                        10   // out message - Returns user set board name
#define MSP_SET_NAME                    11   // in message - Sets board name

// Cleanflight MSP commands
#define MSP_BATTERY_CONFIG              32
#define MSP_SET_BATTERY_CONFIG          33

#define MSP_MODE_RANGES                 34   // out message - Returns all mode ranges
#define MSP_SET_MODE_RANGE              35   // in message - Sets a single mode range

#define MSP_FEATURE_CONFIG              36
#define MSP_SET_FEATURE_CONFIG          37

#define MSP_BOARD_ALIGNMENT_CONFIG      38
#define MSP_SET_BOARD_ALIGNMENT_CONFIG  39

#define MSP_CURRENT_METER_CONFIG        40
#define MSP_SET_CURRENT_METER_CONFIG    41

#define MSP_MIXER_CONFIG                42
#define MSP_SET_MIXER_CONFIG            43

#define MSP_RX_CONFIG                   44
#define MSP_SET_RX_CONFIG               45

#define MSP_LED_COLORS                  46
#define MSP_SET_LED_COLORS              47

#define MSP_LED_STRIP_CONFIG            48
#define MSP_SET_LED_STRIP_CONFIG        49

#define MSP_RSSI_CONFIG                 50
#define MSP_SET_RSSI_CONFIG             51

#define MSP_ADJUSTMENT_RANGES           52
#define MSP_SET_ADJUSTMENT_RANGE        53

#define MSP_CF_SERIAL_CONFIG            54
#define MSP_SET_CF_SERIAL_CONFIG        55

#define MSP_VOLTAGE_METER_CONFIG        56
#define MSP_SET_VOLTAGE_METER_CONFIG    57

#define MSP_SONAR_ALTITUDE              58   // out message - get sonar altitude [cm]

#define MSP_PID_CONTROLLER              59
#define MSP_SET_PID_CONTROLLER          60

#define MSP_ARMING_CONFIG               61
#define MSP_SET_ARMING_CONFIG           62

// Baseflight MSP commands
#define MSP_RX_MAP                      64   // out message - get channel map
#define MSP_SET_RX_MAP                  65   // in message - set rx map

#define MSP_REBOOT                      68   // in message - reboot settings

#define MSP_DATAFLASH_SUMMARY           70   // out message - get description of dataflash chip
#define MSP_DATAFLASH_READ              71   // out message - get content of dataflash chip
#define MSP_DATAFLASH_ERASE             72   // in message - erase dataflash chip

#define MSP_FAILSAFE_CONFIG             75   // out message - Returns FC Fail-Safe settings
#define MSP_SET_FAILSAFE_CONFIG         76   // in message - Sets FC Fail-Safe settings

#define MSP_RXFAIL_CONFIG               77   // out message - Returns RXFAIL settings
#define MSP_SET_RXFAIL_CONFIG           78   // in message - Sets RXFAIL settings

#define MSP_SDCARD_SUMMARY              79   // out message - Get the state of the SD card

#define MSP_BLACKBOX_CONFIG             80   // out message - Get blackbox settings
#define MSP_SET_BLACKBOX_CONFIG         81   // in message - Set blackbox settings

#define MSP_TRANSPONDER_CONFIG          82   // out message - Get transponder settings
#define MSP_SET_TRANSPONDER_CONFIG      83   // in message - Set transponder settings

#define MSP_OSD_CONFIG                  84   // out message - Get osd settings
#define MSP_SET_OSD_CONFIG              85   // in message - Set osd settings

#define MSP_OSD_CHAR_READ               86   // out message - Get osd char
#define MSP_OSD_CHAR_WRITE              87   // in message - Set osd char

#define MSP_VTX_CONFIG                  88   // out message - Get vtx settings
#define MSP_SET_VTX_CONFIG              89   // in message - Set vtx settings

// Betaflight Additional Commands
#define MSP_ADVANCED_CONFIG             90
#define MSP_SET_ADVANCED_CONFIG         91

#define MSP_FILTER_CONFIG               92
#define MSP_SET_FILTER_CONFIG           93

#define MSP_PID_ADVANCED                94
#define MSP_SET_PID_ADVANCED            95

#define MSP_SENSOR_CONFIG               96
#define MSP_SET_SENSOR_CONFIG           97

#define MSP_CAMERA_CONTROL              98

#define MSP_SET_ARMING_DISABLED         99

// OSD specific
#define MSP_OSD_VIDEO_CONFIG            180
#define MSP_SET_OSD_VIDEO_CONFIG        181

#define MSP_DISPLAYPORT                 182  // External OSD displayport mode

#define MSP_COPY_PROFILE                183

#define MSP_BEEPER_CONFIG               184
#define MSP_SET_BEEPER_CONFIG           185

#define MSP_SET_TX_INFO                 186  // in message - Used to send runtime information from TX lua scripts
#define MSP_TX_INFO                     187  // out message - Used by TX lua scripts to read information

#define MSP_SET_OSD_CANVAS              188  // in message - Set osd canvas size
#define MSP_OSD_CANVAS                  189  // out message - Get osd canvas size

// Multiwii original MSP commands
#define MSP_STATUS                      101  // out message - cycletime & errors_count & sensor present & box activation
#define MSP_RAW_IMU                     102  // out message - 9 DOF
#define MSP_SERVO                       103  // out message - servos
#define MSP_MOTOR                       104  // out message - motors
#define MSP_RC                          105  // out message - rc channels and more
#define MSP_RAW_GPS                     106  // out message - fix, numsat, lat, lon, alt, speed, ground course
#define MSP_COMP_GPS                    107  // out message - distance home, direction home
#define MSP_ATTITUDE                    108  // out message - 2 angles 1 heading
#define MSP_ALTITUDE                    109  // out message - altitude, variometer
#define MSP_ANALOG                      110  // out message - vbat, powermetersum, rssi if available on RX
#define MSP_RC_TUNING                   111  // out message - rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_PID                         112  // out message - P I D coeff (9 are used currently)
#define MSP_RAW_RX                      113  // out message - BOX setup
#define MSP_FINALFRAME                  114  // out message - powermeter trig
#define MSP_BOXNAMES                    116  // out message - the aux switch names
#define MSP_PIDNAMES                    117  // out message - the PID names
#define MSP_WP                          118  // in message - set home position (lat, lon, altitude in meters)
#define MSP_BOXIDS                      119  // out message - get the permanent IDs associated to BOXes
#define MSP_SERVO_CONFIGURATIONS        120  // out message - All servo configurations
#define MSP_NAV_STATUS                  121  // out message - Returns navigation status
#define MSP_NAV_CONFIG                  122  // out message - Returns navigation parameters
#define MSP_MOTOR_3D_CONFIG             124  // out message - Settings needed for reversible ESCs
#define MSP_RC_DEADBAND                 125  // out message - deadbands for yaw alt pitch roll
#define MSP_SENSOR_ALIGNMENT            126  // out message - orientation of acc,gyro,mag
#define MSP_LED_STRIP_MODECOLOR         127  // out message - Get LED strip mode_color settings
#define MSP_VOLTAGE_METERS              128  // out message - Voltage (per meter)
#define MSP_CURRENT_METERS              129  // out message - Amperage (per meter)
#define MSP_BATTERY_STATE               130  // out message - Connected/Disconnected, Voltage, Current Used
#define MSP_MOTOR_CONFIG                131  // out message - Motor configuration (min/max throttle, etc)
#define MSP_GPS_CONFIG                  132  // out message - GPS configuration
#define MSP_COMPASS_CONFIG              133  // out message - Compass configuration
#define MSP_ESC_SENSOR_DATA             134  // out message - Extra ESC data from 32-Bit ESCs
#define MSP_GPS_RESCUE                  135  // out message - GPS Rescue settings
#define MSP_GPS_RESCUE_PIDS             136  // out message - GPS Rescue throttleP and velocity PIDS + yaw P
#define MSP_VTXTABLE_BAND               137  // out message - vtxTable band/channel data
#define MSP_VTXTABLE_POWERLEVEL         138  // out message - vtxTable powerLevel data
#define MSP_MOTOR_TELEMETRY             139  // out message - Per-motor telemetry data

// Simplified tuning
#define MSP_SIMPLIFIED_TUNING           140  // out message - Simplified tuning values and enabled state
#define MSP_SET_SIMPLIFIED_TUNING       141  // in message - Set simplified tuning positions
#define MSP_CALCULATE_SIMPLIFIED_PID    142  // out message - Calculate PID values based on sliders
#define MSP_CALCULATE_SIMPLIFIED_GYRO   143  // out message - Calculate gyro filter values
#define MSP_CALCULATE_SIMPLIFIED_DTERM  144  // out message - Calculate dterm filter values
#define MSP_VALIDATE_SIMPLIFIED_TUNING  145  // out message - Validate simplified tuning groups

// MSP write commands
#define MSP_SET_RAW_RC                  200  // in message - 8 rc chan
#define MSP_SET_RAW_GPS                 201  // in message - fix, numsat, lat, lon, alt, speed
#define MSP_SET_PID                     202  // in message - P I D coeff (9 are used currently)
#define MSP_SET_RC_TUNING               204  // in message - rc rate, rc expo, rollpitch rate, yaw rate
#define MSP_ACC_CALIBRATION             205  // in message - no param
#define MSP_MAG_CALIBRATION             206  // in message - no param
#define MSP_RESET_CONF                  208  // in message - no param
#define MSP_SET_WP                      209  // in message - sets a given WP (WP#,lat, lon, alt, flags)
#define MSP_SELECT_SETTING              210  // in message - Select Setting Number (0-2)
#define MSP_SET_HEADING                 211  // in message - define a new heading hold direction
#define MSP_SET_SERVO_CONFIGURATION     212  // in message - Servo settings
#define MSP_SET_MOTOR                   214  // in message - PropBalance function

// MSP V2 Commands (from msp_protocol_v2_common.h)
#define MSP2_COMMON_SERIAL_CONFIG       0x1009
#define MSP2_COMMON_SET_SERIAL_CONFIG   0x100A

// MSP V2 Sensors
#define MSP2_SENSOR_GPS                 0x1F03

// MSP V2 Betaflight specific (from msp_protocol_v2_betaflight.h)
#define MSP2_BETAFLIGHT_BIND            0x3000
#define MSP2_MOTOR_OUTPUT_REORDERING    0x3001
#define MSP2_SET_MOTOR_OUTPUT_REORDERING 0x3002
#define MSP2_SEND_DSHOT_COMMAND         0x3003
#define MSP2_GET_VTX_DEVICE_STATUS      0x3004
#define MSP2_GET_OSD_WARNINGS           0x3005  // returns active OSD warning message text
#define MSP2_GET_TEXT                   0x3006
#define MSP2_SET_TEXT                   0x3007
#define MSP2_GET_LED_STRIP_CONFIG_VALUES 0x3008
#define MSP2_SET_LED_STRIP_CONFIG_VALUES 0x3009
#define MSP2_SENSOR_CONFIG_ACTIVE       0x300A

// MSP2_SET_TEXT and MSP2_GET_TEXT variable types
#define MSP2TEXT_PILOT_NAME             1
#define MSP2TEXT_CRAFT_NAME             2
#define MSP2TEXT_PID_PROFILE_NAME       3
#define MSP2TEXT_RATE_PROFILE_NAME      4
#define MSP2TEXT_BUILDKEY               5
#define MSP2TEXT_RELEASENAME            6

// Maximum payload size
#define MSP_MAX_PAYLOAD_SIZE            256

// MSP Message direction
#define MSP_DIRECTION_REQUEST           '<'
#define MSP_DIRECTION_RESPONSE          '>'
#define MSP_DIRECTION_ERROR             '!'

// Data structures for MSP messages
struct msp_api_version_t {
    uint8_t protocolVersion;
    uint8_t apiVersionMajor;
    uint8_t apiVersionMinor;
};

struct msp_fc_variant_t {
    char identifier[5];  // 4 chars + null terminator
};

struct msp_fc_version_t {
    uint8_t versionMajor;
    uint8_t versionMinor;
    uint8_t versionPatchLevel;
};

struct msp_board_info_t {
    char boardIdentifier[5];  // 4 chars + null terminator
    uint16_t hardwareRevision;
    uint8_t boardType;
    uint8_t targetCapabilities;
    uint8_t targetNameLength;
    char targetName[32];
};

struct msp_name_t {
    char name[16];
};

struct msp_status_t {
    uint16_t cycleTime;
    uint16_t i2cErrorCounter;
    uint16_t sensor;
    uint32_t flightModeFlags;
    uint8_t configProfileIndex;
};

struct msp_attitude_t {
    int16_t roll;    // decidegrees (1/10 degree)
    int16_t pitch;   // decidegrees (1/10 degree)
    int16_t yaw;     // degrees
};

struct msp_altitude_t {
    int32_t estimatedAltitude;  // cm
    int16_t vario;              // cm/s
};

struct msp_analog_t {
    uint8_t vbat;           // 0.1V
    uint16_t mAhDrawn;      // mAh
    uint16_t rssi;          // 0-1023
    int16_t amperage;       // 0.01A
    uint16_t voltage;       // 0.01V (added in API 1.42)
};

struct msp_raw_gps_t {
    uint8_t fixType;
    uint8_t numSat;
    int32_t lat;            // degrees * 1e7
    int32_t lon;            // degrees * 1e7
    int16_t altCm;          // cm (changed to cm in API 1.39)
    uint16_t groundSpeed;   // cm/s
    uint16_t groundCourse;  // decidegrees
};

struct msp_comp_gps_t {
    uint16_t distanceToHome;  // meters
    uint16_t directionToHome; // degrees
    uint8_t heartbeat;
};

struct msp_battery_state_t {
    uint8_t cellCount;
    uint16_t capacity;          // mAh
    uint8_t voltage;            // 0.1V
    uint16_t mAhDrawn;          // mAh
    uint16_t amperage;          // 0.01A
    uint8_t batteryState;
    uint16_t voltage16;         // 0.01V
};

struct msp_rc_t {
    uint16_t channels[18];      // RC channel values (1000-2000)
};

class BetaflightMSP {
public:
    BetaflightMSP();
    
    // Initialize the library with a serial port
    void begin(Stream &serialPort);
    
    // Send MSP request and wait for response (blocking)
    bool request(uint16_t cmd, uint8_t *payload, uint8_t payloadSize, uint32_t timeout = 500);
    
    // Send MSP command (no response expected)
    bool command(uint16_t cmd, uint8_t *payload, uint8_t payloadSize);
    
    // Process incoming MSP data (call in loop)
    void update();
    
    // Get received data
    uint8_t* getPayload() { return _rxPayload; }
    uint8_t getPayloadSize() { return _rxPayloadSize; }
    uint16_t getCommand() { return _rxCommand; }
    bool isError() { return _rxError; }
    
    // Convenience functions for common requests
    bool getApiVersion(msp_api_version_t &data);
    bool getFcVariant(msp_fc_variant_t &data);
    bool getFcVersion(msp_fc_version_t &data);
    bool getBoardInfo(msp_board_info_t &data);
    bool getName(msp_name_t &data);
    bool getStatus(msp_status_t &data);
    bool getAttitude(msp_attitude_t &data);
    bool getAltitude(msp_altitude_t &data);
    bool getAnalog(msp_analog_t &data);
    bool getRawGPS(msp_raw_gps_t &data);
    bool getCompGPS(msp_comp_gps_t &data);
    bool getBatteryState(msp_battery_state_t &data);
    bool getRC(msp_rc_t &data);
    
    // Convenience functions for common commands
    bool setRawRC(uint16_t *channels, uint8_t channelCount);
    bool setGPSHome(int32_t lat, int32_t lon, uint16_t altitudeM);
    bool setRawGPS(uint8_t fixType, uint8_t numSat, int32_t lat, int32_t lon, int16_t altM, uint16_t groundSpeed);
    
private:
    Stream *_port;
    
    // TX state
    void sendMSP(uint16_t cmd, uint8_t *payload, uint8_t payloadSize, bool expectResponse);
    
    // RX state
    enum msp_state_t {
        MSP_IDLE,
        MSP_HEADER_START,
        MSP_HEADER_M,
        MSP_HEADER_ARROW,
        MSP_HEADER_SIZE,
        MSP_HEADER_CMD,
        MSP_PAYLOAD,
        MSP_CHECKSUM,
        MSP_COMMAND_RECEIVED
    };
    
    msp_state_t _rxState;
    uint8_t _rxPayload[MSP_MAX_PAYLOAD_SIZE];
    uint8_t _rxPayloadSize;
    uint8_t _rxPayloadIndex;
    uint16_t _rxCommand;
    uint8_t _rxChecksum;
    uint8_t _rxChecksum2;
    bool _rxError;
    bool _responseReceived;
    uint32_t _requestTimeout;
    
    // Helper functions
    void resetRxState();
    bool processReceivedByte(uint8_t c);
    uint8_t calculateChecksum(uint8_t *data, uint8_t length);
};

// Helper functions for data packing/unpacking
inline int16_t read16(uint8_t *buf, uint8_t &offset) {
    int16_t val = buf[offset] | (buf[offset+1] << 8);
    offset += 2;
    return val;
}

inline uint16_t readU16(uint8_t *buf, uint8_t &offset) {
    uint16_t val = buf[offset] | (buf[offset+1] << 8);
    offset += 2;
    return val;
}

inline int32_t read32(uint8_t *buf, uint8_t &offset) {
    int32_t val = buf[offset] | (buf[offset+1] << 8) | (buf[offset+2] << 16) | (buf[offset+3] << 24);
    offset += 4;
    return val;
}

inline uint32_t readU32(uint8_t *buf, uint8_t &offset) {
    uint32_t val = buf[offset] | (buf[offset+1] << 8) | (buf[offset+2] << 16) | (buf[offset+3] << 24);
    offset += 4;
    return val;
}

inline void write16(uint8_t *buf, uint8_t &offset, int16_t val) {
    buf[offset++] = val & 0xFF;
    buf[offset++] = (val >> 8) & 0xFF;
}

inline void writeU16(uint8_t *buf, uint8_t &offset, uint16_t val) {
    buf[offset++] = val & 0xFF;
    buf[offset++] = (val >> 8) & 0xFF;
}

inline void write32(uint8_t *buf, uint8_t &offset, int32_t val) {
    buf[offset++] = val & 0xFF;
    buf[offset++] = (val >> 8) & 0xFF;
    buf[offset++] = (val >> 16) & 0xFF;
    buf[offset++] = (val >> 24) & 0xFF;
}

inline void writeU32(uint8_t *buf, uint8_t &offset, uint32_t val) {
    buf[offset++] = val & 0xFF;
    buf[offset++] = (val >> 8) & 0xFF;
    buf[offset++] = (val >> 16) & 0xFF;
    buf[offset++] = (val >> 24) & 0xFF;
}

#endif // BETAFLIGHT_MSP_H

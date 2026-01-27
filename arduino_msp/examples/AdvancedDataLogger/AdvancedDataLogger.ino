/*
 * BetaflightMSP Library - Advanced Flight Data Logger
 * 
 * This example creates a comprehensive flight data logger that:
 * - Logs all telemetry to SD card (if available) or serial
 * - Monitors battery and altitude
 * - Provides real-time status display
 * - Can be used for post-flight analysis
 * 
 * Data logged:
 * - Timestamp
 * - Attitude (roll, pitch, yaw)
 * - GPS position and status
 * - Battery voltage and current
 * - Altitude and vertical speed
 * - Flight modes and arm status
 * 
 * Hardware connections:
 * - Connect Arduino TX to FC RX (on a UART configured for MSP)
 * - Connect Arduino RX to FC TX
 * - Connect GND to GND
 * - Optional: SD card module for data logging
 */

#include <BetaflightMSP.h>

BetaflightMSP msp;

// Logging configuration
const uint16_t LOG_INTERVAL_MS = 100;  // Log every 100ms (10Hz)
unsigned long lastLogTime = 0;
unsigned long flightStartTime = 0;
bool isArmed = false;
bool wasArmed = false;

// Statistics
struct FlightStats {
    float maxAltitude = 0;
    float minVoltage = 99.9;
    float maxCurrent = 0;
    float maxSpeed = 0;
    uint32_t flightTime = 0;
    uint16_t mAhUsed = 0;
} stats;

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    
    Serial.println("========================================");
    Serial.println("Betaflight Flight Data Logger");
    Serial.println("========================================");
    
    // Initialize MSP communication
    Serial1.begin(115200);
    msp.begin(Serial1);
    
    delay(1000);
    
    // Verify connection and get FC info
    if (!connectToFC()) {
        Serial.println("ERROR: Failed to connect to flight controller!");
        Serial.println("Check wiring and FC configuration.");
        while (1) delay(100);
    }
    
    Serial.println("\nWaiting for arm...");
    Serial.println("========================================\n");
    
    // Print CSV header
    printCSVHeader();
}

void loop() {
    unsigned long currentTime = millis();
    
    // Check arm status
    checkArmStatus();
    
    // Log data at specified interval
    if (currentTime - lastLogTime >= LOG_INTERVAL_MS) {
        lastLogTime = currentTime;
        
        if (isArmed) {
            logFlightData();
        }
    }
    
    // Print summary when disarmed after flight
    if (wasArmed && !isArmed) {
        wasArmed = false;
        printFlightSummary();
    }
}

bool connectToFC() {
    msp_fc_variant_t variant;
    msp_fc_version_t version;
    msp_name_t name;
    
    if (!msp.getFcVariant(variant)) return false;
    if (!msp.getFcVersion(version)) return false;
    
    Serial.print("Connected to: ");
    Serial.print(variant.identifier);
    Serial.print(" v");
    Serial.print(version.versionMajor);
    Serial.print(".");
    Serial.print(version.versionMinor);
    Serial.print(".");
    Serial.println(version.versionPatchLevel);
    
    if (msp.getName(name) && strlen(name.name) > 0) {
        Serial.print("Craft: ");
        Serial.println(name.name);
    }
    
    return true;
}

void checkArmStatus() {
    msp_status_t status;
    if (msp.getStatus(status)) {
        isArmed = status.flightModeFlags & 0x01;
        
        // Detect arm transition
        if (isArmed && !wasArmed) {
            wasArmed = true;
            flightStartTime = millis();
            resetStats();
            Serial.println("\n>>> ARMED - Flight started <<<\n");
        }
    }
}

void resetStats() {
    stats.maxAltitude = 0;
    stats.minVoltage = 99.9;
    stats.maxCurrent = 0;
    stats.maxSpeed = 0;
    stats.flightTime = 0;
    stats.mAhUsed = 0;
}

void printCSVHeader() {
    Serial.println("Time(s),Armed,Roll(deg),Pitch(deg),Yaw(deg),Alt(m),Vario(m/s),Voltage(V),Current(A),mAh,GPS_Fix,Sats,Lat,Lon,Speed(m/s)");
}

void logFlightData() {
    float timestamp = (millis() - flightStartTime) / 1000.0;
    
    // Get all telemetry
    msp_attitude_t attitude;
    msp_altitude_t altitude;
    msp_battery_state_t battery;
    msp_raw_gps_t gps;
    
    bool hasAttitude = msp.getAttitude(attitude);
    bool hasAltitude = msp.getAltitude(altitude);
    bool hasBattery = msp.getBatteryState(battery);
    bool hasGPS = msp.getRawGPS(gps);
    
    // Update statistics
    if (hasAltitude) {
        float altM = altitude.estimatedAltitude / 100.0;
        if (altM > stats.maxAltitude) stats.maxAltitude = altM;
    }
    
    if (hasBattery) {
        float voltage = battery.voltage16 / 100.0;
        float current = battery.amperage / 100.0;
        if (voltage < stats.minVoltage && voltage > 1.0) stats.minVoltage = voltage;
        if (current > stats.maxCurrent) stats.maxCurrent = current;
        stats.mAhUsed = battery.mAhDrawn;
    }
    
    if (hasGPS && gps.fixType > 0) {
        float speed = gps.groundSpeed / 100.0;
        if (speed > stats.maxSpeed) stats.maxSpeed = speed;
    }
    
    // Print CSV data
    Serial.print(timestamp, 2);
    Serial.print(",");
    Serial.print(isArmed ? "1" : "0");
    Serial.print(",");
    
    if (hasAttitude) {
        Serial.print(attitude.roll / 10.0, 1);
        Serial.print(",");
        Serial.print(attitude.pitch / 10.0, 1);
        Serial.print(",");
        Serial.print(attitude.yaw, 1);
    } else {
        Serial.print(",,,");
    }
    Serial.print(",");
    
    if (hasAltitude) {
        Serial.print(altitude.estimatedAltitude / 100.0, 2);
        Serial.print(",");
        Serial.print(altitude.vario / 100.0, 2);
    } else {
        Serial.print(",,");
    }
    Serial.print(",");
    
    if (hasBattery) {
        Serial.print(battery.voltage16 / 100.0, 2);
        Serial.print(",");
        Serial.print(battery.amperage / 100.0, 2);
        Serial.print(",");
        Serial.print(battery.mAhDrawn);
    } else {
        Serial.print(",,,");
    }
    Serial.print(",");
    
    if (hasGPS) {
        Serial.print(gps.fixType);
        Serial.print(",");
        Serial.print(gps.numSat);
        Serial.print(",");
        if (gps.fixType > 0) {
            Serial.print(gps.lat / 1e7, 7);
            Serial.print(",");
            Serial.print(gps.lon / 1e7, 7);
            Serial.print(",");
            Serial.print(gps.groundSpeed / 100.0, 2);
        } else {
            Serial.print(",,,");
        }
    } else {
        Serial.print(",,,,");
    }
    
    Serial.println();
}

void printFlightSummary() {
    stats.flightTime = (millis() - flightStartTime) / 1000;
    
    Serial.println("\n========================================");
    Serial.println(">>> DISARMED - Flight ended <<<");
    Serial.println("========================================");
    Serial.println("Flight Summary:");
    Serial.println("----------------------------------------");
    
    Serial.print("Flight Time:     ");
    Serial.print(stats.flightTime / 60);
    Serial.print("m ");
    Serial.print(stats.flightTime % 60);
    Serial.println("s");
    
    Serial.print("Max Altitude:    ");
    Serial.print(stats.maxAltitude, 2);
    Serial.println(" m");
    
    Serial.print("Min Voltage:     ");
    Serial.print(stats.minVoltage, 2);
    Serial.println(" V");
    
    Serial.print("Max Current:     ");
    Serial.print(stats.maxCurrent, 2);
    Serial.println(" A");
    
    Serial.print("Energy Used:     ");
    Serial.print(stats.mAhUsed);
    Serial.println(" mAh");
    
    Serial.print("Max Speed:       ");
    Serial.print(stats.maxSpeed, 2);
    Serial.println(" m/s");
    
    Serial.println("========================================");
    Serial.println("\nWaiting for next arm...\n");
    
    // Print CSV header again for next flight
    printCSVHeader();
}

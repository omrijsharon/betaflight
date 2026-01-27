/*
 * BetaflightMSP Library - Telemetry Monitor Example
 * 
 * This example demonstrates how to read various telemetry data from the flight controller:
 * - Battery voltage and current
 * - Altitude and vertical speed
 * - GPS position and status
 * - Attitude (roll, pitch, yaw)
 * - Flight status
 * 
 * Hardware connections:
 * - Connect Arduino TX to FC RX (on a UART configured for MSP)
 * - Connect Arduino RX to FC TX
 * - Connect GND to GND
 */

#include <BetaflightMSP.h>

BetaflightMSP msp;

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    
    Serial.println("Betaflight MSP - Telemetry Monitor");
    Serial.println("===================================");
    
    // Initialize MSP communication
    Serial1.begin(115200);
    msp.begin(Serial1);
    
    delay(1000);
    
    // Verify connection
    msp_fc_variant_t variant;
    msp_fc_version_t version;
    if (msp.getFcVariant(variant) && msp.getFcVersion(version)) {
        Serial.print("Connected to: ");
        Serial.print(variant.identifier);
        Serial.print(" v");
        Serial.print(version.versionMajor);
        Serial.print(".");
        Serial.print(version.versionMinor);
        Serial.print(".");
        Serial.println(version.versionPatchLevel);
        
        msp_name_t name;
        if (msp.getName(name) && strlen(name.name) > 0) {
            Serial.print("Craft: ");
            Serial.println(name.name);
        }
    } else {
        Serial.println("Failed to connect to flight controller!");
        while (1) delay(100);
    }
    
    Serial.println("\nStarting telemetry monitoring...\n");
    delay(1000);
}

void loop() {
    Serial.println("========================================");
    
    // Flight Status
    printStatus();
    
    // Battery
    printBattery();
    
    // Attitude
    printAttitude();
    
    // Altitude
    printAltitude();
    
    // GPS
    printGPS();
    
    Serial.println("========================================\n");
    
    delay(1000);  // Update every second
}

void printStatus() {
    msp_status_t status;
    if (msp.getStatus(status)) {
        Serial.println("--- Flight Status ---");
        Serial.print("Cycle Time: ");
        Serial.print(status.cycleTime);
        Serial.println(" us");
        
        Serial.print("Sensors: ");
        if (status.sensor & 0x01) Serial.print("ACC ");
        if (status.sensor & 0x02) Serial.print("BARO ");
        if (status.sensor & 0x04) Serial.print("MAG ");
        if (status.sensor & 0x08) Serial.print("GPS ");
        if (status.sensor & 0x10) Serial.print("SONAR ");
        Serial.println();
        
        Serial.print("Armed: ");
        Serial.println((status.flightModeFlags & 0x01) ? "YES" : "NO");
        
        Serial.print("Profile: ");
        Serial.println(status.configProfileIndex);
    }
}

void printBattery() {
    msp_battery_state_t battery;
    if (msp.getBatteryState(battery)) {
        Serial.println("--- Battery ---");
        Serial.print("Voltage: ");
        Serial.print(battery.voltage16 / 100.0, 2);
        Serial.println(" V");
        
        Serial.print("Current: ");
        Serial.print(battery.amperage / 100.0, 2);
        Serial.println(" A");
        
        Serial.print("Used: ");
        Serial.print(battery.mAhDrawn);
        Serial.print(" / ");
        Serial.print(battery.capacity);
        Serial.println(" mAh");
        
        Serial.print("Cells: ");
        Serial.println(battery.cellCount);
        
        Serial.print("State: ");
        switch (battery.batteryState) {
            case 0: Serial.println("OK"); break;
            case 1: Serial.println("WARNING"); break;
            case 2: Serial.println("CRITICAL"); break;
            default: Serial.println("UNKNOWN"); break;
        }
    }
}

void printAttitude() {
    msp_attitude_t attitude;
    if (msp.getAttitude(attitude)) {
        Serial.println("--- Attitude ---");
        Serial.print("Roll:  ");
        Serial.print(attitude.roll / 10.0, 1);
        Serial.println("°");
        
        Serial.print("Pitch: ");
        Serial.print(attitude.pitch / 10.0, 1);
        Serial.println("°");
        
        Serial.print("Yaw:   ");
        Serial.print(attitude.yaw, 1);
        Serial.println("°");
    }
}

void printAltitude() {
    msp_altitude_t altitude;
    if (msp.getAltitude(altitude)) {
        Serial.println("--- Altitude ---");
        Serial.print("Altitude: ");
        Serial.print(altitude.estimatedAltitude / 100.0, 2);
        Serial.println(" m");
        
        Serial.print("Vario: ");
        Serial.print(altitude.vario / 100.0, 2);
        Serial.println(" m/s");
    }
}

void printGPS() {
    msp_raw_gps_t gps;
    if (msp.getRawGPS(gps)) {
        Serial.println("--- GPS ---");
        
        Serial.print("Fix: ");
        switch (gps.fixType) {
            case 0: Serial.print("NO FIX"); break;
            case 1: Serial.print("2D"); break;
            case 2: Serial.print("3D"); break;
            default: Serial.print("UNKNOWN"); break;
        }
        Serial.print(" | Satellites: ");
        Serial.println(gps.numSat);
        
        if (gps.fixType > 0) {
            Serial.print("Position: ");
            Serial.print(gps.lat / 1e7, 7);
            Serial.print(", ");
            Serial.println(gps.lon / 1e7, 7);
            
            Serial.print("Altitude: ");
            Serial.print(gps.altCm / 100.0, 1);
            Serial.println(" m");
            
            Serial.print("Speed: ");
            Serial.print(gps.groundSpeed / 100.0, 2);
            Serial.println(" m/s");
            
            Serial.print("Course: ");
            Serial.print(gps.groundCourse / 10.0, 1);
            Serial.println("°");
            
            // Distance to home
            msp_comp_gps_t compGps;
            if (msp.getCompGPS(compGps)) {
                Serial.print("Home: ");
                Serial.print(compGps.distanceToHome);
                Serial.print(" m @ ");
                Serial.print(compGps.directionToHome);
                Serial.println("°");
            }
        }
    }
}

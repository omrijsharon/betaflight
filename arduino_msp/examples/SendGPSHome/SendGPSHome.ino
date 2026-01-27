/*
 * BetaflightMSP Library - Send GPS Home Position Example
 * 
 * This example demonstrates how to set the GPS home position on the flight controller
 * using MSP_WP command. This is useful when you want to manually set a home position
 * or relay GPS data from an external GPS module.
 * 
 * The GPS home position is used by GPS rescue mode and other GPS-based features.
 * 
 * GPS coordinates format:
 * - Latitude: degrees * 1e7 (e.g., 37.7749 * 1e7 = 377749000)
 * - Longitude: degrees * 1e7 (e.g., -122.4194 * 1e7 = -1224194000)
 * - Altitude: meters
 * 
 * Hardware connections:
 * - Connect Arduino TX to FC RX (on a UART configured for MSP)
 * - Connect Arduino RX to FC TX
 * - Connect GND to GND
 */

#include <BetaflightMSP.h>

BetaflightMSP msp;

// Example GPS coordinates (San Francisco, CA)
const float HOME_LAT = 37.7749;
const float HOME_LON = -122.4194;
const uint16_t HOME_ALT = 10;  // meters

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    
    Serial.println("Betaflight MSP - Send GPS Home Position Example");
    Serial.println("================================================");
    
    // Initialize MSP communication
    Serial1.begin(115200);
    msp.begin(Serial1);
    
    delay(1000);
    
    // Verify connection
    msp_fc_variant_t variant;
    if (msp.getFcVariant(variant)) {
        Serial.print("Connected to: ");
        Serial.println(variant.identifier);
    } else {
        Serial.println("Failed to connect to flight controller!");
        while (1) delay(100);
    }
    
    Serial.println("\nSetting GPS home position...");
    
    // Convert to MSP format (degrees * 1e7)
    int32_t lat = HOME_LAT * 1e7;
    int32_t lon = HOME_LON * 1e7;
    
    // Send GPS home position
    if (msp.setGPSHome(lat, lon, HOME_ALT)) {
        Serial.println("✓ GPS home position set successfully!");
        Serial.print("  Latitude:  ");
        Serial.print(HOME_LAT, 7);
        Serial.println("°");
        Serial.print("  Longitude: ");
        Serial.print(HOME_LON, 7);
        Serial.println("°");
        Serial.print("  Altitude:  ");
        Serial.print(HOME_ALT);
        Serial.println(" m");
    } else {
        Serial.println("✗ Failed to set GPS home position");
    }
    
    Serial.println("\n================================================");
    Serial.println("You can verify this in Betaflight Configurator:");
    Serial.println("- Go to GPS tab");
    Serial.println("- Check the home position");
    Serial.println("================================================\n");
}

void loop() {
    // Read GPS status to verify
    msp_comp_gps_t compGps;
    if (msp.getCompGPS(compGps)) {
        Serial.print("Distance to home: ");
        Serial.print(compGps.distanceToHome);
        Serial.print(" m | Direction: ");
        Serial.print(compGps.directionToHome);
        Serial.println("°");
    }
    
    // You could also continuously send GPS position from an external GPS module
    // Example: Send simulated GPS position every second
    static unsigned long lastGpsUpdate = 0;
    if (millis() - lastGpsUpdate >= 1000) {
        lastGpsUpdate = millis();
        
        // Simulate GPS data (in real application, read from GPS module)
        uint8_t fixType = 3;      // 3 = 3D fix
        uint8_t numSat = 12;      // Number of satellites
        int32_t lat = HOME_LAT * 1e7;
        int32_t lon = HOME_LON * 1e7;
        int16_t altM = HOME_ALT;
        uint16_t speed = 0;       // Ground speed in cm/s
        
        if (msp.setRawGPS(fixType, numSat, lat, lon, altM, speed)) {
            Serial.print("✓ GPS data sent: ");
            Serial.print(numSat);
            Serial.println(" satellites");
        }
    }
    
    delay(1000);
}

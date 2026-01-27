/*
 * BetaflightMSP Library - Basic Connection Example
 * 
 * This example shows how to connect to a Betaflight flight controller
 * and read basic information like API version, firmware variant, and board info.
 * 
 * Hardware connections:
 * - Connect Arduino TX to FC RX (on a UART configured for MSP)
 * - Connect Arduino RX to FC TX
 * - Connect GND to GND
 * 
 * Note: Make sure the baud rate matches your FC's MSP port configuration
 * (typically 115200 baud)
 */

#include <BetaflightMSP.h>

// Create MSP instance
BetaflightMSP msp;

void setup() {
    // Initialize debug serial
    Serial.begin(115200);
    while (!Serial) delay(10);
    
    Serial.println("Betaflight MSP - Basic Connection Example");
    Serial.println("==========================================");
    
    // Initialize MSP on Serial1 (adjust to your hardware)
    // For Arduino Mega/Due: Serial1, Serial2, Serial3
    // For ESP32: Serial2
    // For Arduino Uno: use SoftwareSerial
    Serial1.begin(115200);
    msp.begin(Serial1);
    
    delay(1000);  // Wait for FC to boot
    
    Serial.println("\nConnecting to flight controller...");
    
    // Get API version
    msp_api_version_t apiVersion;
    if (msp.getApiVersion(apiVersion)) {
        Serial.print("✓ API Version: ");
        Serial.print(apiVersion.protocolVersion);
        Serial.print(".");
        Serial.print(apiVersion.apiVersionMajor);
        Serial.print(".");
        Serial.println(apiVersion.apiVersionMinor);
    } else {
        Serial.println("✗ Failed to get API version");
    }
    
    // Get FC variant (firmware type)
    msp_fc_variant_t fcVariant;
    if (msp.getFcVariant(fcVariant)) {
        Serial.print("✓ Firmware: ");
        Serial.println(fcVariant.identifier);
    } else {
        Serial.println("✗ Failed to get FC variant");
    }
    
    // Get FC version
    msp_fc_version_t fcVersion;
    if (msp.getFcVersion(fcVersion)) {
        Serial.print("✓ Version: ");
        Serial.print(fcVersion.versionMajor);
        Serial.print(".");
        Serial.print(fcVersion.versionMinor);
        Serial.print(".");
        Serial.println(fcVersion.versionPatchLevel);
    } else {
        Serial.println("✗ Failed to get FC version");
    }
    
    // Get board info
    msp_board_info_t boardInfo;
    if (msp.getBoardInfo(boardInfo)) {
        Serial.print("✓ Board: ");
        Serial.print(boardInfo.boardIdentifier);
        if (boardInfo.targetNameLength > 0) {
            Serial.print(" (");
            Serial.print(boardInfo.targetName);
            Serial.print(")");
        }
        Serial.println();
    } else {
        Serial.println("✗ Failed to get board info");
    }
    
    // Get craft name
    msp_name_t name;
    if (msp.getName(name)) {
        if (strlen(name.name) > 0) {
            Serial.print("✓ Craft Name: ");
            Serial.println(name.name);
        } else {
            Serial.println("✓ Craft Name: (not set)");
        }
    } else {
        Serial.println("✗ Failed to get craft name");
    }
    
    Serial.println("\n==========================================");
    Serial.println("Connection successful!");
    Serial.println("==========================================\n");
}

void loop() {
    // Get flight controller status
    msp_status_t status;
    if (msp.getStatus(status)) {
        Serial.print("Cycle Time: ");
        Serial.print(status.cycleTime);
        Serial.print("us | Sensors: 0x");
        Serial.print(status.sensor, HEX);
        Serial.print(" | Flight Modes: 0x");
        Serial.print(status.flightModeFlags, HEX);
        Serial.print(" | Profile: ");
        Serial.println(status.configProfileIndex);
    }
    
    delay(1000);  // Update every second
}

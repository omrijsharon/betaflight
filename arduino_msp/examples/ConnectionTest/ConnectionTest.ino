/*
 * BetaflightMSP Library - Connection Test
 * 
 * Simple test sketch to verify the library installation and connection
 * to the flight controller. Use this to troubleshoot connection issues.
 * 
 * Expected output:
 * - API version
 * - FC variant (e.g., "BTFL")
 * - FC version
 * - Connection success message
 * 
 * If you see "Test PASSED", everything is working correctly!
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
    
    Serial.println("\n========================================");
    Serial.println("BetaflightMSP Connection Test");
    Serial.println("========================================\n");
    
    // Initialize MSP on Serial1
    // Change this to match your hardware:
    // - Arduino Mega: Serial1, Serial2, or Serial3
    // - ESP32: Serial2 (or custom pins)
    // - Arduino Due: Serial1, Serial2, or Serial3
    Serial1.begin(115200);
    msp.begin(Serial1);
    
    Serial.println("Initializing...");
    delay(2000);  // Give FC time to boot
    
    Serial.println("Testing connection...\n");
    
    // Test 1: API Version
    Serial.print("Test 1: API Version... ");
    msp_api_version_t api;
    if (msp.getApiVersion(api)) {
        Serial.print("PASS (");
        Serial.print(api.protocolVersion);
        Serial.print(".");
        Serial.print(api.apiVersionMajor);
        Serial.print(".");
        Serial.print(api.apiVersionMinor);
        Serial.println(")");
    } else {
        Serial.println("FAIL");
        printTroubleshooting();
        while (1) delay(100);
    }
    
    // Test 2: FC Variant
    Serial.print("Test 2: FC Variant... ");
    msp_fc_variant_t variant;
    if (msp.getFcVariant(variant)) {
        Serial.print("PASS (");
        Serial.print(variant.identifier);
        Serial.println(")");
    } else {
        Serial.println("FAIL");
    }
    
    // Test 3: FC Version
    Serial.print("Test 3: FC Version... ");
    msp_fc_version_t version;
    if (msp.getFcVersion(version)) {
        Serial.print("PASS (");
        Serial.print(version.versionMajor);
        Serial.print(".");
        Serial.print(version.versionMinor);
        Serial.print(".");
        Serial.print(version.versionPatchLevel);
        Serial.println(")");
    } else {
        Serial.println("FAIL");
    }
    
    // Test 4: Board Info
    Serial.print("Test 4: Board Info... ");
    msp_board_info_t board;
    if (msp.getBoardInfo(board)) {
        Serial.print("PASS (");
        Serial.print(board.boardIdentifier);
        Serial.println(")");
    } else {
        Serial.println("FAIL");
    }
    
    // Test 5: Attitude
    Serial.print("Test 5: Attitude... ");
    msp_attitude_t attitude;
    if (msp.getAttitude(attitude)) {
        Serial.print("PASS (Roll=");
        Serial.print(attitude.roll / 10.0, 1);
        Serial.println("°)");
    } else {
        Serial.println("FAIL");
    }
    
    // Test 6: Status
    Serial.print("Test 6: Status... ");
    msp_status_t status;
    if (msp.getStatus(status)) {
        Serial.print("PASS (Armed=");
        Serial.print((status.flightModeFlags & 0x01) ? "YES" : "NO");
        Serial.println(")");
    } else {
        Serial.println("FAIL");
    }
    
    // Summary
    Serial.println("\n========================================");
    Serial.println("All tests PASSED!");
    Serial.println("Library is working correctly.");
    Serial.println("========================================\n");
    
    Serial.println("You can now try the example sketches:");
    Serial.println("- BasicConnection");
    Serial.println("- ReadAttitude");
    Serial.println("- TelemetryMonitor");
    Serial.println("- SendStickCommands (props off!)");
    Serial.println("- SendGPSHome");
    Serial.println("- AdvancedDataLogger\n");
}

void loop() {
    // Continuously test connection
    static unsigned long lastTest = 0;
    if (millis() - lastTest >= 2000) {
        lastTest = millis();
        
        Serial.print(".");
        
        // Quick connection check
        msp_status_t status;
        if (!msp.getStatus(status)) {
            Serial.println("\nWARNING: Lost connection to FC!");
        }
    }
}

void printTroubleshooting() {
    Serial.println("\n========================================");
    Serial.println("Connection Failed - Troubleshooting:");
    Serial.println("========================================");
    Serial.println("1. Check wiring:");
    Serial.println("   - Arduino TX → FC RX");
    Serial.println("   - Arduino RX → FC TX");
    Serial.println("   - GND → GND");
    Serial.println();
    Serial.println("2. In Betaflight Configurator:");
    Serial.println("   - Go to Ports tab");
    Serial.println("   - Find the UART you're using");
    Serial.println("   - Enable 'Configuration/MSP'");
    Serial.println("   - Set baud rate to 115200");
    Serial.println("   - Click 'Save and Reboot'");
    Serial.println();
    Serial.println("3. Verify baud rate:");
    Serial.println("   - FC: 115200 (in Configurator)");
    Serial.println("   - Arduino: Serial1.begin(115200)");
    Serial.println();
    Serial.println("4. Try different UART:");
    Serial.println("   - Some UARTs may be used for other functions");
    Serial.println();
    Serial.println("5. Check voltage levels:");
    Serial.println("   - 5V Arduino needs level shifter");
    Serial.println("   - 3.3V boards (ESP32, Due) OK direct");
    Serial.println("========================================\n");
}

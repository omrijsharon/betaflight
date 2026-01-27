/*
 * BetaflightMSP Library - Read Attitude Example
 * 
 * This example demonstrates how to read the flight controller's attitude
 * (roll, pitch, yaw) and display it on the serial monitor.
 * 
 * The attitude values are:
 * - Roll: angle in decidegrees (-1800 to +1800 = -180° to +180°)
 * - Pitch: angle in decidegrees (-1800 to +1800 = -180° to +180°)
 * - Yaw: angle in degrees (0 to 359)
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
    
    Serial.println("Betaflight MSP - Read Attitude Example");
    Serial.println("========================================");
    
    // Initialize MSP communication
    Serial1.begin(115200);
    msp.begin(Serial1);
    
    delay(1000);
    
    // Verify connection
    msp_fc_variant_t variant;
    if (msp.getFcVariant(variant)) {
        Serial.print("Connected to: ");
        Serial.println(variant.identifier);
        Serial.println("Reading attitude data...\n");
    } else {
        Serial.println("Failed to connect to flight controller!");
        while (1) delay(100);
    }
}

void loop() {
    msp_attitude_t attitude;
    
    if (msp.getAttitude(attitude)) {
        // Convert decidegrees to degrees
        float rollDeg = attitude.roll / 10.0;
        float pitchDeg = attitude.pitch / 10.0;
        float yawDeg = attitude.yaw;
        
        // Display as formatted output
        Serial.print("Roll: ");
        Serial.print(rollDeg, 1);
        Serial.print("° | Pitch: ");
        Serial.print(pitchDeg, 1);
        Serial.print("° | Yaw: ");
        Serial.print(yawDeg, 1);
        Serial.println("°");
        
        // Alternative: Visual representation
        printAngleBar("Roll ", rollDeg, 45.0);
        printAngleBar("Pitch", pitchDeg, 45.0);
        
        Serial.println();
    } else {
        Serial.println("Failed to read attitude");
    }
    
    delay(100);  // 10Hz update rate
}

// Helper function to print a visual angle bar
void printAngleBar(const char* label, float angle, float maxAngle) {
    Serial.print(label);
    Serial.print(": ");
    
    // Constrain to max angle
    float constrained = constrain(angle, -maxAngle, maxAngle);
    int barPos = map(constrained * 10, -maxAngle * 10, maxAngle * 10, 0, 40);
    
    // Print bar
    for (int i = 0; i < 41; i++) {
        if (i == 20) {
            Serial.print("|");  // Center marker
        } else if (i == barPos) {
            Serial.print("*");  // Current position
        } else {
            Serial.print("-");
        }
    }
    
    Serial.print(" ");
    Serial.print(angle, 1);
    Serial.println("°");
}

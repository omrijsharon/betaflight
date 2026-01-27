/*
 * BetaflightMSP Library - Send Stick Commands Example
 * 
 * This example demonstrates how to send RC stick commands to the flight controller
 * using MSP_SET_RAW_RC. This is useful for autonomous control or testing.
 * 
 * IMPORTANT SAFETY NOTES:
 * - This will override your RC transmitter input when MSP RX is active
 * - Make sure you have a failsafe configured
 * - Test with props off first!
 * - Be ready to use your transmitter to regain control
 * 
 * RC Channel values range from 1000 to 2000 (1500 is center)
 * Typical channel order: Roll, Pitch, Throttle, Yaw, AUX1, AUX2, etc.
 * 
 * Hardware connections:
 * - Connect Arduino TX to FC RX (on a UART configured for MSP)
 * - Connect Arduino RX to FC TX
 * - Connect GND to GND
 */

#include <BetaflightMSP.h>

BetaflightMSP msp;

// RC channel definitions
const uint8_t CHANNEL_ROLL = 0;
const uint8_t CHANNEL_PITCH = 1;
const uint8_t CHANNEL_THROTTLE = 2;
const uint8_t CHANNEL_YAW = 3;
const uint8_t CHANNEL_AUX1 = 4;
const uint8_t CHANNEL_AUX2 = 5;
const uint8_t CHANNEL_AUX3 = 6;
const uint8_t CHANNEL_AUX4 = 7;

// Number of channels to send
const uint8_t NUM_CHANNELS = 8;

// RC channel values (1000-2000)
uint16_t rcChannels[NUM_CHANNELS];

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    
    Serial.println("Betaflight MSP - Send Stick Commands Example");
    Serial.println("==============================================");
    Serial.println("\nWARNING: This will control your flight controller!");
    Serial.println("Make sure:");
    Serial.println("- Props are OFF");
    Serial.println("- Failsafe is configured");
    Serial.println("- You can regain control with your transmitter");
    Serial.println("\nPress any key to continue...");
    
    while (!Serial.available()) delay(100);
    while (Serial.available()) Serial.read();
    
    // Initialize MSP communication
    Serial1.begin(115200);
    msp.begin(Serial1);
    
    delay(1000);
    
    // Verify connection
    msp_fc_variant_t variant;
    if (msp.getFcVariant(variant)) {
        Serial.print("\nConnected to: ");
        Serial.println(variant.identifier);
    } else {
        Serial.println("\nFailed to connect to flight controller!");
        while (1) delay(100);
    }
    
    // Initialize all channels to center/low
    initializeChannels();
    
    Serial.println("\nSending RC commands...");
    Serial.println("Commands:");
    Serial.println("  'w' - Increase throttle");
    Serial.println("  's' - Decrease throttle");
    Serial.println("  'a' - Roll left");
    Serial.println("  'd' - Roll right");
    Serial.println("  'i' - Pitch forward");
    Serial.println("  'k' - Pitch back");
    Serial.println("  'j' - Yaw left");
    Serial.println("  'l' - Yaw right");
    Serial.println("  'c' - Center all sticks");
    Serial.println("  'x' - Stop (set throttle to minimum)\n");
}

void loop() {
    // Check for keyboard input
    if (Serial.available()) {
        char cmd = Serial.read();
        handleCommand(cmd);
    }
    
    // Send RC commands at 50Hz (required for MSP RX)
    static unsigned long lastSend = 0;
    if (millis() - lastSend >= 20) {  // 50Hz = 20ms
        lastSend = millis();
        
        if (msp.setRawRC(rcChannels, NUM_CHANNELS)) {
            // Successfully sent
            printChannels();
        } else {
            Serial.println("Failed to send RC data");
        }
    }
}

void initializeChannels() {
    // Set all channels to safe defaults
    rcChannels[CHANNEL_ROLL] = 1500;      // Center
    rcChannels[CHANNEL_PITCH] = 1500;     // Center
    rcChannels[CHANNEL_THROTTLE] = 1000;  // Minimum (disarmed)
    rcChannels[CHANNEL_YAW] = 1500;       // Center
    rcChannels[CHANNEL_AUX1] = 1000;      // Off
    rcChannels[CHANNEL_AUX2] = 1000;      // Off
    rcChannels[CHANNEL_AUX3] = 1000;      // Off
    rcChannels[CHANNEL_AUX4] = 1000;      // Off
}

void handleCommand(char cmd) {
    const uint16_t STEP = 50;  // Movement step
    
    switch (cmd) {
        // Throttle
        case 'w':
            rcChannels[CHANNEL_THROTTLE] = constrain(rcChannels[CHANNEL_THROTTLE] + STEP, 1000, 2000);
            break;
        case 's':
            rcChannels[CHANNEL_THROTTLE] = constrain(rcChannels[CHANNEL_THROTTLE] - STEP, 1000, 2000);
            break;
            
        // Roll
        case 'a':
            rcChannels[CHANNEL_ROLL] = constrain(rcChannels[CHANNEL_ROLL] - STEP, 1000, 2000);
            break;
        case 'd':
            rcChannels[CHANNEL_ROLL] = constrain(rcChannels[CHANNEL_ROLL] + STEP, 1000, 2000);
            break;
            
        // Pitch
        case 'i':
            rcChannels[CHANNEL_PITCH] = constrain(rcChannels[CHANNEL_PITCH] + STEP, 1000, 2000);
            break;
        case 'k':
            rcChannels[CHANNEL_PITCH] = constrain(rcChannels[CHANNEL_PITCH] - STEP, 1000, 2000);
            break;
            
        // Yaw
        case 'j':
            rcChannels[CHANNEL_YAW] = constrain(rcChannels[CHANNEL_YAW] - STEP, 1000, 2000);
            break;
        case 'l':
            rcChannels[CHANNEL_YAW] = constrain(rcChannels[CHANNEL_YAW] + STEP, 1000, 2000);
            break;
            
        // Center all
        case 'c':
            rcChannels[CHANNEL_ROLL] = 1500;
            rcChannels[CHANNEL_PITCH] = 1500;
            rcChannels[CHANNEL_YAW] = 1500;
            Serial.println("Centered!");
            break;
            
        // Emergency stop
        case 'x':
            rcChannels[CHANNEL_THROTTLE] = 1000;
            rcChannels[CHANNEL_ROLL] = 1500;
            rcChannels[CHANNEL_PITCH] = 1500;
            rcChannels[CHANNEL_YAW] = 1500;
            Serial.println("STOPPED!");
            break;
    }
}

void printChannels() {
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint >= 500) {  // Print every 500ms
        lastPrint = millis();
        
        Serial.print("R:");
        Serial.print(rcChannels[CHANNEL_ROLL]);
        Serial.print(" P:");
        Serial.print(rcChannels[CHANNEL_PITCH]);
        Serial.print(" T:");
        Serial.print(rcChannels[CHANNEL_THROTTLE]);
        Serial.print(" Y:");
        Serial.print(rcChannels[CHANNEL_YAW]);
        Serial.println();
    }
}

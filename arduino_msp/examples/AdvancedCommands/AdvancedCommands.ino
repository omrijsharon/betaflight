/*
 * BetaflightMSP Library - Advanced Commands Example
 * 
 * This example demonstrates how to use advanced MSP commands beyond
 * the convenience functions. Shows reading motor telemetry, VTX config,
 * and other advanced features.
 * 
 * The library includes 140+ MSP commands from Betaflight!
 * You can use ANY command by calling msp.request() or msp.command()
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
    
    Serial.println("Betaflight MSP - Advanced Commands Example");
    Serial.println("============================================");
    
    // Initialize MSP communication
    Serial1.begin(115200);
    msp.begin(Serial1);
    
    delay(1000);
    
    // Verify connection
    msp_fc_variant_t variant;
    if (msp.getFcVariant(variant)) {
        Serial.print("Connected to: ");
        Serial.println(variant.identifier);
        Serial.println();
    } else {
        Serial.println("Failed to connect to flight controller!");
        while (1) delay(100);
    }
    
    // Demonstrate various advanced commands
    readMotorData();
    Serial.println();
    
    readPIDValues();
    Serial.println();
    
    readFilterConfig();
    Serial.println();
    
    readVTXConfig();
    Serial.println();
    
    readBlackboxConfig();
    Serial.println();
    
    Serial.println("============================================");
    Serial.println("Entering continuous telemetry mode...\n");
}

void loop() {
    // Continuously read motor telemetry if available
    readMotorTelemetry();
    
    delay(500);  // Update every 500ms
}

void readMotorData() {
    Serial.println("Reading Motor Data (MSP_MOTOR)...");
    
    if (msp.request(MSP_MOTOR, nullptr, 0)) {
        uint8_t* data = msp.getPayload();
        uint8_t size = msp.getPayloadSize();
        
        // Each motor is 2 bytes (uint16_t)
        uint8_t numMotors = size / 2;
        Serial.print("Number of motors: ");
        Serial.println(numMotors);
        
        for (uint8_t i = 0; i < numMotors; i++) {
            uint16_t motorValue = data[i*2] | (data[i*2+1] << 8);
            Serial.print("  Motor ");
            Serial.print(i + 1);
            Serial.print(": ");
            Serial.println(motorValue);
        }
    } else {
        Serial.println("  Failed to read motor data");
    }
}

void readMotorTelemetry() {
    // MSP_MOTOR_TELEMETRY - Available on 32-bit ESCs with telemetry
    if (msp.request(MSP_MOTOR_TELEMETRY, nullptr, 0)) {
        uint8_t* data = msp.getPayload();
        uint8_t size = msp.getPayloadSize();
        
        if (size > 0) {
            // Each motor telemetry is 8 bytes
            uint8_t numMotors = size / 8;
            
            Serial.print("Motor Telemetry (");
            Serial.print(numMotors);
            Serial.println(" motors):");
            
            for (uint8_t i = 0; i < numMotors; i++) {
                uint8_t offset = i * 8;
                
                uint16_t rpm = data[offset] | (data[offset+1] << 8);
                uint16_t invalidPercent = data[offset+2] | (data[offset+3] << 8);
                uint8_t temp = data[offset+4];
                uint16_t voltage = data[offset+5] | (data[offset+6] << 8);
                uint16_t current = data[offset+7];
                
                Serial.print("  M");
                Serial.print(i + 1);
                Serial.print(": RPM=");
                Serial.print(rpm * 100);  // RPM is in 100s
                Serial.print(" Temp=");
                Serial.print(temp);
                Serial.print("°C Volt=");
                Serial.print(voltage / 100.0, 2);
                Serial.print("V Curr=");
                Serial.print(current / 100.0, 2);
                Serial.println("A");
            }
            Serial.println();
        }
    }
}

void readPIDValues() {
    Serial.println("Reading PID Values (MSP_PID)...");
    
    if (msp.request(MSP_PID, nullptr, 0)) {
        uint8_t* data = msp.getPayload();
        
        // PID structure: 3 axes (Roll, Pitch, Yaw) x 3 values (P, I, D)
        const char* axes[] = {"Roll", "Pitch", "Yaw"};
        
        for (uint8_t i = 0; i < 3; i++) {
            uint8_t offset = i * 3;
            Serial.print("  ");
            Serial.print(axes[i]);
            Serial.print(": P=");
            Serial.print(data[offset]);
            Serial.print(" I=");
            Serial.print(data[offset + 1]);
            Serial.print(" D=");
            Serial.println(data[offset + 2]);
        }
    } else {
        Serial.println("  Failed to read PID values");
    }
}

void readFilterConfig() {
    Serial.println("Reading Filter Config (MSP_FILTER_CONFIG)...");
    
    if (msp.request(MSP_FILTER_CONFIG, nullptr, 0)) {
        uint8_t* data = msp.getPayload();
        uint8_t offset = 0;
        
        uint8_t gyroLowpassHz = data[offset++];
        uint16_t dTermLowpassHz = data[offset] | (data[offset+1] << 8);
        offset += 2;
        
        Serial.print("  Gyro Lowpass: ");
        Serial.print(gyroLowpassHz);
        Serial.println(" Hz");
        
        Serial.print("  D-Term Lowpass: ");
        Serial.print(dTermLowpassHz);
        Serial.println(" Hz");
    } else {
        Serial.println("  Failed to read filter config");
    }
}

void readVTXConfig() {
    Serial.println("Reading VTX Config (MSP_VTX_CONFIG)...");
    
    if (msp.request(MSP_VTX_CONFIG, nullptr, 0)) {
        uint8_t* data = msp.getPayload();
        uint8_t size = msp.getPayloadSize();
        
        if (size >= 15) {
            uint8_t offset = 0;
            uint8_t vtxType = data[offset++];
            uint8_t band = data[offset++];
            uint8_t channel = data[offset++];
            uint8_t power = data[offset++];
            uint8_t pitmode = data[offset++];
            uint16_t freq = data[offset] | (data[offset+1] << 8);
            offset += 2;
            
            Serial.print("  VTX Type: ");
            Serial.println(vtxType);
            Serial.print("  Band: ");
            Serial.println(band);
            Serial.print("  Channel: ");
            Serial.println(channel);
            Serial.print("  Power: ");
            Serial.println(power);
            Serial.print("  Pit Mode: ");
            Serial.println(pitmode ? "ON" : "OFF");
            Serial.print("  Frequency: ");
            Serial.print(freq);
            Serial.println(" MHz");
        }
    } else {
        Serial.println("  VTX not available or failed to read");
    }
}

void readBlackboxConfig() {
    Serial.println("Reading Blackbox Config (MSP_BLACKBOX_CONFIG)...");
    
    if (msp.request(MSP_BLACKBOX_CONFIG, nullptr, 0)) {
        uint8_t* data = msp.getPayload();
        
        uint8_t supported = data[0];
        uint8_t device = data[1];
        uint8_t rateNum = data[2];
        uint8_t rateDenom = data[3];
        
        Serial.print("  Supported: ");
        Serial.println(supported ? "Yes" : "No");
        
        Serial.print("  Device: ");
        switch(device) {
            case 0: Serial.println("None"); break;
            case 1: Serial.println("Flash"); break;
            case 2: Serial.println("SD Card"); break;
            case 3: Serial.println("Serial"); break;
            default: Serial.println("Unknown"); break;
        }
        
        Serial.print("  Sample Rate: ");
        Serial.print(rateNum);
        Serial.print("/");
        Serial.println(rateDenom);
    } else {
        Serial.println("  Failed to read blackbox config");
    }
}

/*
 * OTHER USEFUL COMMANDS YOU CAN TRY:
 * 
 * MSP_SERVO (103) - Read servo positions
 * MSP_RC_TUNING (111) - Read RC rates
 * MSP_SENSOR_ALIGNMENT (126) - Read sensor orientation
 * MSP_ADVANCED_CONFIG (90) - Read advanced settings
 * MSP_SIMPLIFIED_TUNING (140) - Read simplified tuning
 * MSP_GPS_CONFIG (132) - Read GPS configuration
 * MSP_COMPASS_CONFIG (133) - Read compass configuration
 * MSP_ESC_SENSOR_DATA (134) - Read ESC sensor data
 * MSP_GPS_RESCUE (135) - Read GPS rescue settings
 * MSP_DATAFLASH_SUMMARY (70) - Read dataflash info
 * MSP_SDCARD_SUMMARY (79) - Read SD card status
 * MSP_MOTOR_CONFIG (131) - Read motor configuration
 * 
 * And 130+ more commands! See BetaflightMSP.h for the complete list.
 */

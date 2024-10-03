#include <Arduino.h>
#include <CAN.h>            
#include "IIS328DQTR.h"     

// Initialize the IIS328DQTR accelerometer with I2C address 0x18
IIS328DQTR accelerometer(0x18);

// Device and Sensor Configuration
const uint8_t DEVICE_ADDRESS = 0x02;        // Unique ID for this sensor node
const uint8_t SENSOR_TYPE_ACCEL = 0x02;     // Identifier for Accelerometer
const uint8_t DATA_PARAMETERS = 0x03;       // Number of data parameters (X, Y, Z)

// CAN Configuration
const uint32_t CAN_ID = 0x200;              // CAN ID for Accelerometer
const uint8_t CAN_DATA_LENGTH = 8;          // Length of CAN data in bytes

// Firmware and Hardware Flags
uint8_t firmwareFlags = 0x00;               // Initialize firmware flags
uint8_t hardwareFlags = 0x00;               // Initialize hardware flags

// Function to calculate checksum 
uint8_t calculateChecksum(uint8_t data[], uint8_t length) {
    uint16_t sum = 0;
    for (uint8_t i = 0; i < length; i++) {
        sum += data[i];
    }
    return (uint8_t)(sum & 0xFF);
}

void setup() {
    Serial.begin(115200);
    while (!Serial) {
        ; // Wait for Serial to initialize
    }

    // Initialize CAN bus at 500 kbps
    if (!CAN.begin(500E3)) { // 500 kbps
        Serial.println("CAN bus initialization failed.");
        while (1); 
    }

    Serial.println("CAN bus initialized successfully.");

    // Initialize the accelerometer
    Serial.println("Initializing IIS328DQTR Accelerometer...");
    if (!accelerometer.begin()) {
        Serial.println("Failed to initialize IIS328DQTR!");
        while (1); 
    }
    Serial.println("IIS328DQTR initialized successfully.");

    // Initialize firmware and hardware flags
    firmwareFlags = 0x01; // Firmware version 1
    hardwareFlags = 0x00; // No hardware errors
}

void loop() {
    float x, y, z;

    // Read acceleration data from the accelerometer
    if (accelerometer.readAcceleration(x, y, z)) {
        Serial.print("Acceleration [g]: X=");
        Serial.print(x, 3);
        Serial.print(" Y=");
        Serial.print(y, 3);
        Serial.print(" Z=");
        Serial.println(z, 3);
    } else {
        Serial.println("Failed to read acceleration data.");
        // Set hardware flag for sensor error
        hardwareFlags |= 0x01; // Set bit 0
        x = y = z = 0; // Default values
    }

    // Update diagnostic flags based on conditions
    updateDiagnosticFlags(x, y, z);

    // Prepare CAN frame data
    uint8_t canData[8] = {0}; // Initialize all bytes to 0

    // Structure:
    // Byte 0: Device Address
    // Byte 1: Sensor Type
    // Byte 2: Data Parameters
    // Byte 3: X-axis Data (8-bit integer, scaled)
    // Byte 4: Y-axis Data (8-bit integer, scaled)
    // Byte 5: Z-axis Data (8-bit integer, scaled)
    // Byte 6: Firmware Flags
    // Byte 7: Checksum

    // Assign Device Address, Sensor Type, and Data Parameters
    canData[0] = DEVICE_ADDRESS;
    canData[1] = SENSOR_TYPE_ACCEL;
    canData[2] = DATA_PARAMETERS;

    // Scale acceleration to fit into 8 bits with a scaling factor
    // Example: Multiply by 100 to convert from g to centi-g (cg)
    int8_t xScaled = constrain((int8_t)(x * 100), -128, 127);
    int8_t yScaled = constrain((int8_t)(y * 100), -128, 127);
    int8_t zScaled = constrain((int8_t)(z * 100), -128, 127);

    // Assign X, Y, Z scaled data
    canData[3] = (uint8_t)xScaled;
    canData[4] = (uint8_t)yScaled;
    canData[5] = (uint8_t)zScaled;

    // Assign Firmware Flags
    canData[6] = firmwareFlags;

    // Calculate checksum for bytes 0-6
    canData[7] = calculateChecksum(canData, 7);

    // Send the CAN frame
    CAN.beginPacket(CAN_ID);
    CAN.write(canData, CAN_DATA_LENGTH); 
    if (CAN.endPacket()) {
        Serial.println("CAN message sent successfully.");
    } else {
        Serial.println("Error sending CAN message.");
    }

    delay(500); 
}

// Function to update diagnostic flags based on conditions
void updateDiagnosticFlags(float x, float y, float z) {
    // Example: Set a firmware flag if any axis exceeds ±2g
    if (abs(x) > 2.0 || abs(y) > 2.0 || abs(z) > 2.0) {
        firmwareFlags |= 0x02; // Set bit 1 to indicate high acceleration warning
    } else {
        firmwareFlags &= ~0x02; // Clear bit 1
    }

    // Clear hardware flag if sensor reads valid data
    if (hardwareFlags & 0x01) {
        hardwareFlags &= ~0x01; // Clear bit 0 if no error
    }
}

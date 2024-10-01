#include <Arduino.h>
#include <CAN.h>             
#include "IIS328DQTR.h"     

// Initialize the IIS328DQTR accelerometer with I2C address 0x18
IIS328DQTR accelerometer(0x18);

// Device and Sensor Configuration
const uint8_t DEVICE_ADDRESS = 0x01;        // Unique ID for this sensor node
const uint8_t SENSOR_TYPE_ACCEL = 0x02;      // Identifier for Accelerometer
const uint8_t DATA_PARAMETERS = 0x03;        // Number of data parameters (X, Y, Z)

// CAN Configuration
const uint32_t CAN_ID = 0x200;               // CAN ID for Accelerometer
const uint8_t CAN_DATA_LENGTH = 8;           // Length of CAN data in bytes

// Function to calculate checksum (simple sum modulo 256)
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
    }

    // Prepare CAN frame data
    uint8_t canData[8] = {0}; // Initialize all bytes to 0

    // Structure:
    // Byte 0: Device Address
    // Byte 1: Sensor Type
    // Byte 2: Data Parameters
    // Byte 3: X-axis Data (8-bit integer, scaled)
    // Byte 4: Y-axis Data (8-bit integer, scaled)
    // Byte 5: Z-axis Data (8-bit integer, scaled)
    // Byte 6: Reserved (set to 0x00)
    // Byte 7: Checksum

    // Assign Device Address, Sensor Type, and Data Parameters
    canData[0] = DEVICE_ADDRESS;
    canData[1] = SENSOR_TYPE_ACCEL;
    canData[2] = DATA_PARAMETERS;

    // Scale acceleration to fit into 8 bits with a scaling factor
    // 1 g = 100 mg
    // Ensure values are within -128 to 127
    int8_t xScaled = constrain((int8_t)(x * 100), -128, 127);
    int8_t yScaled = constrain((int8_t)(y * 100), -128, 127);
    int8_t zScaled = constrain((int8_t)(z * 100), -128, 127);

    // Assign X, Y, Z scaled data
    canData[3] = (uint8_t)xScaled;
    canData[4] = (uint8_t)yScaled;
    canData[5] = (uint8_t)zScaled;

    // Reserved byte
    canData[6] = 0x00;

    // Calculate checksum for bytes 0-5
    canData[7] = calculateChecksum(canData, 6);

    // Send the CAN frame
    CAN.beginPacket(CAN_ID);
    CAN.write(canData, CAN_DATA_LENGTH); 
    if (CAN.endPacket()) {
        Serial.println("CAN message sent successfully.");
    } else {
        Serial.println("Error sending CAN message.");
    }

    delay(500); // Delay for readability
}

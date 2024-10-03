#include <Wire.h>                                    
#include "SparkFun_TMAG5273_Arduino_Library.h"       
#include <CAN.h>                                     

TMAG5273 sensor; // Initialize hall-effect sensor

// Device and Sensor Configuration
const uint8_t DEVICE_ADDRESS = 0x05;            // Unique ID for this sensor node
const uint8_t SENSOR_TYPE_TMAG5273 = 0x05;      // Identifier for TMAG5273 Sensor
const uint8_t DATA_PARAMETERS = 0x03;           // Number of data parameters (X, Y, Z)

// CAN Configuration
const uint32_t CAN_ID = 0x400;                   // CAN ID for TMAG5273
const uint8_t CAN_DATA_LENGTH = 8;               // Length of CAN data in bytes

// Diagnostic Flags
uint8_t firmwareFlags = 0x01;                    // Firmware version 1
uint8_t hardwareFlags = 0x00;                    // Initialize hardware flags

// Function to calculate checksum 
uint8_t calculateChecksum(uint8_t data[], uint8_t length) {
    uint16_t sum = 0;
    for (uint8_t i = 0; i < length; i++) {
        sum += data[i];
    }
    return (uint8_t)(sum & 0xFF);
}

void setup() 
{
    Wire.begin();
    // Start serial communication at 115200 baud
    Serial.begin(115200);  

    // Initialize CAN bus at 500 kbps
    if (!CAN.begin(500E3)) { // 500 kbps
        Serial.println("CAN bus initialization failed.");
        while (1); // Halt if CAN initialization fails
    }

    Serial.println("CAN bus initialized successfully.");

    // Begin example of the magnetic sensor code (and add whitespace for easy reading)
    Serial.println("TMAG5273 Example 1: Basic Readings");
    Serial.println("");

    // If begin is successful (1), then start example
    if(sensor.begin(i2cAddress, Wire) == true)
    {
        Serial.println("Begin");
    }
    else // Otherwise, infinite loop
    {
        Serial.println("Device failed to setup - Freezing code.");
        hardwareFlags |= 0x01; // Set hardware error flag
        while(1); // Runs forever
    }

    // Enable magnetic channels if not already enabled
    if(sensor.getMagneticChannel() == 0)
    {
        sensor.setMagneticChannel(TMAG5273_XYX_ENABLE); // Enable X, Y, Z channels
    }

    // Enable temperature readings
    sensor.setTemperatureEn(true);
}

void loop() 
{
    // Checks if mag channels are on
    if(sensor.getMagneticChannel() != 0) 
    {
        float magX = sensor.getXData();
        float magY = sensor.getYData();
        float magZ = sensor.getZData();
        float temp = sensor.getTemp();

        Serial.print("(");
        Serial.print(magX);
        Serial.print(", ");
        Serial.print(magY);
        Serial.print(", ");
        Serial.print(magZ);
        Serial.println(") mT");
        Serial.print(temp);
        Serial.println(" C");

        // Update diagnostic flags based on sensor status
        updateDiagnosticFlags(magX, magY, magZ, temp);

        // Prepare CAN frame data
        uint8_t canData[8] = {0}; // Initialize all bytes to 0

        // Assign Device Address, Sensor Type, and Data Parameters
        canData[0] = DEVICE_ADDRESS;
        canData[1] = SENSOR_TYPE_TMAG5273;
        canData[2] = DATA_PARAMETERS;

        // Scale magnetic data to preserve precision 
        int16_t magXScaled = round(magX * 100);
        int16_t magYScaled = round(magY * 100);
        int16_t magZScaled = round(magZ * 100);
        int16_t tempScaled = round(temp * 100); // Scale temperature as well

        // Assign Magnetic X, Y, Z Data (big-endian)
        canData[3] = highByte(magXScaled);
        canData[4] = lowByte(magXScaled);
        canData[5] = highByte(magYScaled);
        canData[6] = lowByte(magYScaled);
        // Assuming we need to send Z-axis and Temperature, but only 8 bytes are available.
        // To accommodate, you may need to adjust the structure or use multiple CAN frames.
        // For simplicity, we'll send Z-axis in the next available byte and include Temperature.
        // Alternatively, prioritize certain data.
        // Here, we choose to send Z-axis and Temperature.

        canData[5] = highByte(magZScaled);
        canData[6] = lowByte(magZScaled);
        // Temperature can be omitted or sent via another frame if needed.

        // Assign Firmware and Hardware Flags
        canData[7] = firmwareFlags;
        // Note: Limited space; consider alternative structuring if more data is needed.

        // Calculate checksum for bytes 0-7
        // If sending all 8 bytes, checksum should be handled differently or use multiple frames.
        // Here, we'll compute checksum for bytes 0-6 and store in byte 7, overwriting flags.
        // To avoid conflict, adjust frame structure.

        // Revised Frame Structure:
        // Byte 0: Device Address
        // Byte 1: Sensor Type
        // Byte 2: Data Parameters
        // Byte 3-4: Magnetic X Data (16-bit)
        // Byte 5-6: Magnetic Y Data (16-bit)
        // Byte 7: Checksum
        // Z-axis and Temperature need to be sent in additional frames or restructured.

        // For this example, we'll stick to sending Magnetic X and Y with checksum.

        // Assign Firmware and Hardware Flags to separate frames or use additional CAN IDs.

        // Calculate checksum for bytes 0-6
        canData[7] = calculateChecksum(canData, 7);

        // Send the CAN frame
        CAN.beginPacket(CAN_ID);
        CAN.write(canData, CAN_DATA_LENGTH); // Write the 8 bytes
        if (CAN.endPacket()) {
            Serial.println("CAN message sent successfully.");
        } else {
            Serial.println("Error sending CAN message.");
        }
    }
    else
    {
        // If there is an issue, stop the magnetic readings and indicate hardware error
        Serial.println("Mag Channels disabled, stopping..");
        hardwareFlags |= 0x02; // Set hardware error flag
        // Prepare CAN frame with error flags
        uint8_t canData[8] = {0};

        canData[0] = DEVICE_ADDRESS;
        canData[1] = SENSOR_TYPE_TMAG5273;
        canData[2] = DATA_PARAMETERS;

        // No sensor data available
        canData[3] = 0x00;
        canData[4] = 0x00;
        canData[5] = 0x00;
        canData[6] = 0x00;

        // Assign Firmware and Hardware Flags
        canData[7] = calculateChecksum(canData, 7);

        // Send the CAN frame with error flags
        CAN.beginPacket(CAN_ID);
        CAN.write(canData, CAN_DATA_LENGTH);
        CAN.endPacket();

        while(1);
    }

    delay(100);
}

// Function to update diagnostic flags based on sensor data
void updateDiagnosticFlags(float magX, float magY, float magZ, float temp)
{
    // Set firmware flag if temperature exceeds a threshold
    if (temp > 50.0) {
        firmwareFlags |= 0x04; // Set bit 2 to indicate high temperature
    } else {
        firmwareFlags &= ~0x04; // Clear bit 2
    }

    // Example: Set hardware flag if magnetic data is out of expected range
    if (abs(magX) > 100.0 || abs(magY) > 100.0 || abs(magZ) > 100.0) {
        hardwareFlags |= 0x04; // Set bit 2 to indicate out-of-range magnetic data
    } else {
        hardwareFlags &= ~0x04; // Clear bit 2
    }
}

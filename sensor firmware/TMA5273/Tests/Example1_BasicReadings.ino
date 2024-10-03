#include <Wire.h>                                    
#include "SparkFun_TMAG5273_Arduino_Library.h"       
#include <CAN.h>                                     

// Initialize hall-effect sensor
TMAG5273 sensor;

// Device and Sensor Configuration
const uint8_t DEVICE_ADDRESS = 0x05;               // Unique ID for this sensor node
const uint8_t SENSOR_TYPE_TMAG5273 = 0x05;         // Identifier for TMAG5273 Sensor
const uint8_t DATA_PARAMETERS_MAGXY = 0x02;        // Data Parameters for MagX & MagY
const uint8_t DATA_PARAMETERS_MAGZTEMP = 0x03;     // Data Parameters for MagZ & Temperature
const uint8_t DATA_PARAMETERS_DIAGNOSTICS = 0x04;  // Data Parameters for Diagnostics

// CAN Configuration
const uint32_t CAN_ID_MAGXY = 0x400;                // CAN ID for MagX & MagY
const uint32_t CAN_ID_MAGZTEMP = 0x401;             // CAN ID for MagZ & Temperature
const uint32_t CAN_ID_DIAGNOSTICS = 0x402;          // CAN ID for Diagnostic Flags
const uint8_t CAN_DATA_LENGTH = 8;                  // Length of CAN data in bytes

// Diagnostic Flags
uint8_t firmwareFlags = 0x01;                       // Firmware version 1
uint8_t hardwareFlags = 0x00;                       // Initialize hardware flags

// Function to calculate checksum (simple sum modulo 256)
uint8_t calculateChecksum(uint8_t data[], uint8_t length) {
    uint16_t sum = 0;
    for (uint8_t i = 0; i < length; i++) {
        sum += data[i];
    }
    return (uint8_t)(sum & 0xFF);
}

// Function to update diagnostic flags based on sensor data
void updateDiagnosticFlags(float magX, float magY, float magZ, float temp) {
    // Set firmware flag if temperature exceeds a threshold
    if (temp > 50.0) {
        firmwareFlags |= 0x02; // Set bit 1 to indicate high temperature warning
    } else {
        firmwareFlags &= ~0x02; // Clear bit 1
    }

    // Set hardware flag if magnetic data is out of expected range
    if (abs(magX) > 100.0 || abs(magY) > 100.0 || abs(magZ) > 100.0) {
        hardwareFlags |= 0x01; // Set bit 0 to indicate sensor connectivity issue
    } else {
        hardwareFlags &= ~0x01; // Clear bit 0
    }
}

void setup() 
{
    Wire.begin();
    // Start serial communication at 115200 baud
    Serial.begin(115200);  

    // Initialize CAN bus at 500 kbps
    if (!CAN.begin(500E3)) { // 500 kbps
        Serial.println("CAN bus initialization failed.");
        hardwareFlags |= 0x02; // Set hardware error flag
        while(1); // Halt if CAN initialization fails
    }

    Serial.println("CAN bus initialized successfully.");

    // Begin example of the magnetic sensor code (and add whitespace for easy reading)
    Serial.println("TMAG5273 Example 1: Basic Readings with Diagnostics");
    Serial.println("");

    // If begin is successful (true), then start example
    if(sensor.begin(i2cAddress, Wire) == true)
    {
        Serial.println("Sensor initialized successfully.");
    }
    else 
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
    // Check if magnetic channels and temperature readings are enabled
    if(sensor.getMagneticChannel() != 0 && sensor.getTemperatureEn() == true) 
    {
        float magX = sensor.getXData();
        float magY = sensor.getYData();
        float magZ = sensor.getZData();
        float temp = sensor.getTemp();

        Serial.print("Magnetic Fields (mT): X=");
        Serial.print(magX, 2);
        Serial.print(", Y=");
        Serial.print(magY, 2);
        Serial.print(", Z=");
        Serial.println(magZ, 2);
        Serial.print("Temperature: ");
        Serial.print(temp, 2);
        Serial.println(" °C");

        // Update diagnostic flags based on sensor data
        updateDiagnosticFlags(magX, magY, magZ, temp);

        // --- Frame 1: MagX & MagY ---
        uint8_t canDataMagXY[8] = {0}; // Initialize all bytes to 0

        // Assign Device Address, Sensor Type, and Data Parameters
        canDataMagXY[0] = DEVICE_ADDRESS;
        canDataMagXY[1] = SENSOR_TYPE_TMAG5273;
        canDataMagXY[2] = DATA_PARAMETERS_MAGXY;

        // Scale magnetic data to preserve precision (e.g., multiply by 100 for centi-mT)
        int16_t magXScaled = round(magX * 100);
        int16_t magYScaled = round(magY * 100);

        // Assign Magnetic X & Y Data (big-endian)
        canDataMagXY[3] = highByte(magXScaled);
        canDataMagXY[4] = lowByte(magXScaled);
        canDataMagXY[5] = highByte(magYScaled);
        canDataMagXY[6] = lowByte(magYScaled);

        // Calculate checksum for bytes 0-6
        canDataMagXY[7] = calculateChecksum(canDataMagXY, 7);

        // Send the MagX & MagY CAN frame
        CAN.beginPacket(CAN_ID_MAGXY);
        CAN.write(canDataMagXY, CAN_DATA_LENGTH); 
        if (CAN.endPacket()) {
            Serial.println("CAN message for MagX & MagY sent successfully.");
        } else {
            Serial.println("Error sending CAN message for MagX & MagY.");
        }

        // --- Frame 2: MagZ & Temperature ---
        uint8_t canDataMagZTemp[8] = {0}; // Initialize all bytes to 0

        // Assign Device Address, Sensor Type, and Data Parameters
        canDataMagZTemp[0] = DEVICE_ADDRESS;
        canDataMagZTemp[1] = SENSOR_TYPE_TMAG5273;
        canDataMagZTemp[2] = DATA_PARAMETERS_MAGZTEMP;

        // Scale magnetic Z and Temperature data
        int16_t magZScaled = round(magZ * 100);
        int16_t tempScaled = round(temp * 100); 

        // Assign Magnetic Z & Temperature Data (big-endian)
        canDataMagZTemp[3] = highByte(magZScaled);
        canDataMagZTemp[4] = lowByte(magZScaled);
        canDataMagZTemp[5] = highByte(tempScaled);
        canDataMagZTemp[6] = lowByte(tempScaled);

        // Calculate checksum for bytes 0-6
        canDataMagZTemp[7] = calculateChecksum(canDataMagZTemp, 7);

        // Send the MagZ & Temperature CAN frame
        CAN.beginPacket(CAN_ID_MAGZTEMP);
        CAN.write(canDataMagZTemp, CAN_DATA_LENGTH); 
        if (CAN.endPacket()) {
            Serial.println("CAN message for MagZ & Temperature sent successfully.");
        } else {
            Serial.println("Error sending CAN message for MagZ & Temperature.");
        }

        // --- Frame 3: Diagnostic Flags ---
        uint8_t canDataDiagnostics[8] = {0}; // Initialize all bytes to 0

        // Assign Device Address, Sensor Type, and Data Parameters
        canDataDiagnostics[0] = DEVICE_ADDRESS;
        canDataDiagnostics[1] = SENSOR_TYPE_TMAG5273;
        canDataDiagnostics[2] = DATA_PARAMETERS_DIAGNOSTICS;

        // Assign Firmware and Hardware Flags
        canDataDiagnostics[3] = firmwareFlags;
        canDataDiagnostics[4] = hardwareFlags;
        // Bytes 5-6: Reserved (set to 0x00)
        canDataDiagnostics[5] = 0x00;
        canDataDiagnostics[6] = 0x00;

        // Calculate checksum for bytes 0-6
        canDataDiagnostics[7] = calculateChecksum(canDataDiagnostics, 7);

        // Send the Diagnostic Flags CAN frame
        CAN.beginPacket(CAN_ID_DIAGNOSTICS);
        CAN.write(canDataDiagnostics, CAN_DATA_LENGTH); 
        if (CAN.endPacket()) {
            Serial.println("CAN message for Diagnostics sent successfully.");
        } else {
            Serial.println("Error sending CAN message for Diagnostics.");
        }
    }
    else
    {
        // If there is an issue, stop the magnetic readings and indicate hardware error
        Serial.println("Mag Channels disabled, stopping..");
        hardwareFlags |= 0x02; // Set hardware error flag

        // --- Frame 3: Diagnostic Flags with Error ---
        uint8_t canDataDiagnosticsError[8] = {0}; // Initialize all bytes to 0

        canDataDiagnosticsError[0] = DEVICE_ADDRESS;
        canDataDiagnosticsError[1] = SENSOR_TYPE_TMAG5273;
        canDataDiagnosticsError[2] = DATA_PARAMETERS_DIAGNOSTICS;

        // Assign Firmware and Hardware Flags
        canDataDiagnosticsError[3] = firmwareFlags;
        canDataDiagnosticsError[4] = hardwareFlags;
        // Bytes 5-6: Reserved
        canDataDiagnosticsError[5] = 0x00;
        canDataDiagnosticsError[6] = 0x00;

        // Calculate checksum for bytes 0-6
        canDataDiagnosticsError[7] = calculateChecksum(canDataDiagnosticsError, 7);

        // Send the Diagnostic Flags CAN frame with error
        CAN.beginPacket(CAN_ID_DIAGNOSTICS);
        CAN.write(canDataDiagnosticsError, CAN_DATA_LENGTH);
        CAN.endPacket();

        while(1);
    }

    delay(100); // Short delay to prevent flooding the CAN bus
}

// Function to update diagnostic flags based on sensor data
void updateDiagnosticFlags(float magX, float magY, float magZ, float temp)
{
    // Set firmware flag if temperature exceeds a threshold
    if (temp > 50.0) {
        firmwareFlags |= 0x02; // Set bit 1 to indicate high temperature warning
    } else {
        firmwareFlags &= ~0x02; // Clear bit 1
    }

    // Set hardware flag if magnetic data is out of expected range
    if (abs(magX) > 100.0 || abs(magY) > 100.0 || abs(magZ) > 100.0) {
        hardwareFlags |= 0x01; // Set bit 0 to indicate sensor connectivity issue
    } else {
        hardwareFlags &= ~0x01; // Clear bit 0
    }

}


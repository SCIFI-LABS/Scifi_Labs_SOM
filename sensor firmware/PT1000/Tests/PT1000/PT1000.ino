#include <Arduino.h>
#include <CAN.h>            
#include "PT1000.h"

// PT1000 sensor setup
const int analogPin = A0;       // Analog input pin connected to the voltage divider
const float Vcc = 5.0;          // Supply voltage (5V from Arduino)
const float Rref = 1000.0;      // Reference resistor value in ohms (1kΩ)

// Device and Sensor Configuration
const uint8_t DEVICE_ADDRESS = 0x01;     // Unique ID for this sensor node
const uint8_t SENSOR_TYPE_TEMP = 0x01;   // Identifier for Temperature Sensor
const uint8_t DATA_PARAMETERS = 0x01;    // Number of data parameters (1: Temperature)

// CAN Configuration
const uint32_t CAN_ID = 0x100;           // CAN ID for Temperature Sensor
const uint8_t CAN_DATA_LENGTH = 8;       // Length of CAN data in bytes

// Firmware and Hardware Flags
uint8_t firmwareFlags = 0x00;            // Initialize firmware flags
uint8_t hardwareFlags = 0x00;            // Initialize hardware flags

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
        while (1); // Halt if CAN initialization fails
    }

    Serial.println("CAN bus initialized successfully.");

    // Initialize firmware and hardware flags
    firmwareFlags = 0x01; // Firmware version 1
    hardwareFlags = 0x00; // No hardware errors
}

void loop() {
    // Read the PT1000 resistance
    float Rpt1000 = readPT1000(analogPin, Vcc, Rref);

    // Calculate temperature from resistance
    float temperature = calculateTemperature(Rpt1000);

    // Output the results to Serial for debugging
    Serial.print("Resistance: ");
    Serial.print(Rpt1000, 1);
    Serial.print(" Ω | Temperature: ");
    Serial.print(temperature, 2);
    Serial.println(" °C");

    // Update diagnostic flags based on conditions
    updateDiagnosticFlags(temperature);

    // Prepare CAN frame data
    uint8_t canData[8] = {0}; // Initialize all bytes to 0

    // Structure:
    // Byte 0: Device Address
    // Byte 1: Sensor Type
    // Byte 2: Data Parameters
    // Byte 3-4: Temperature Data (16-bit integer, scaled)
    // Byte 5: Firmware Flags
    // Byte 6: Hardware Flags
    // Byte 7: Checksum

    // Assign Device Address, Sensor Type, and Data Parameters
    canData[0] = DEVICE_ADDRESS;
    canData[1] = SENSOR_TYPE_TEMP;
    canData[2] = DATA_PARAMETERS;

    // Scale temperature to preserve two decimal places 
    int16_t tempScaled = round(temperature * 100);

    // Assign Temperature Data (big-endian)
    canData[3] = highByte(tempScaled);
    canData[4] = lowByte(tempScaled);

    // Assign Firmware and Hardware Flags
    canData[5] = firmwareFlags;
    canData[6] = hardwareFlags;

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

    delay(1000);
}

// Function to update diagnostic flags based on conditions
void updateDiagnosticFlags(float temperature) {
    // Set a firmware flag if the temperature exceeds a threshold
    if (temperature > 100.0) {
        firmwareFlags |= 0x02; // Set bit 1 to indicate high-temperature warning
    } else {
        firmwareFlags &= ~0x02; // Clear bit 1
    }

    // Set a hardware flag if the sensor reading is invalid
    if (temperature < -50.0 || temperature > 150.0) {
        hardwareFlags |= 0x01; // Set bit 0 to indicate sensor error
    } else {
        hardwareFlags &= ~0x01; // Clear bit 0
    }
}

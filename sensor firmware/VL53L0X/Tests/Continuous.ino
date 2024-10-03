#include <Wire.h>
#include <VL53L0X.h>
#include <CAN.h>            

VL53L0X sensor;

// Device and Sensor Configuration
const uint8_t DEVICE_ADDRESS = 0x03;       // Unique ID for this sensor node
const uint8_t SENSOR_TYPE_VL53L0X = 0x03;  // Identifier for VL53L0X Sensor
const uint8_t DATA_PARAMETERS = 0x01;      // Number of data parameters (1: Range)

// CAN Configuration
const uint32_t CAN_ID = 0x300;             // CAN ID for VL53L0X
const uint8_t CAN_DATA_LENGTH = 8;         // Length of CAN data in bytes

// Diagnostic Flags
uint8_t firmwareFlags = 0x01;              // Firmware version 1
uint8_t hardwareFlags = 0x00;              // Initialize hardware flags

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
    Serial.begin(9600);
    while (!Serial) {
        ; // Wait for Serial to initialize
    }

    Wire.begin();

    // Initialize CAN bus at 500 kbps
    if (!CAN.begin(500E3)) { // 500 kbps
        Serial.println("CAN bus initialization failed.");
        while (1); // Halt if CAN initialization fails
    }

    Serial.println("CAN bus initialized successfully.");

    sensor.setTimeout(500);
    if (!sensor.init())
    {
        Serial.println("Failed to detect and initialize VL53L0X!");
        hardwareFlags |= 0x01; // Set hardware error flag
        while (1) {}
    }

    // Start continuous back-to-back mode
    sensor.startContinuous();

    Serial.println("VL53L0X initialized successfully.");
}

void loop()
{
    uint16_t range = sensor.readRangeContinuousMillimeters();
    bool timeout = sensor.timeoutOccurred();

    // Output the results to Serial for debugging
    Serial.print("Range: ");
    Serial.print(range);
    Serial.print(" mm");
    if (timeout) { Serial.print(" TIMEOUT"); }
    Serial.println();

    // Update diagnostic flags based on sensor status
    updateDiagnosticFlags(range, timeout);

    // Prepare CAN frame data
    uint8_t canData[8] = {0}; // Initialize all bytes to 0

    // Assign Device Address, Sensor Type, and Data Parameters
    canData[0] = DEVICE_ADDRESS;
    canData[1] = SENSOR_TYPE_VL53L0X;
    canData[2] = DATA_PARAMETERS;

    // Assign Range Data (big-endian)
    canData[3] = highByte(range);
    canData[4] = lowByte(range);

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

// Function to update diagnostic flags based on sensor status
void updateDiagnosticFlags(uint16_t range, bool timeout)
{
    // Set hardware flag if timeout occurs
    if (timeout) {
        hardwareFlags |= 0x02; // Set bit 1 to indicate timeout
    } else {
        hardwareFlags &= ~0x02; // Clear bit 1
    }

    // Set firmware flag if range exceeds a threshold 
    if (range > 2000) {
        firmwareFlags |= 0x04; // Set bit 2 to indicate range exceeds threshold
    } else {
        firmwareFlags &= ~0x04; // Clear bit 2
    }
}

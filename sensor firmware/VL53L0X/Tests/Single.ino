#include <Wire.h>
#include <VL53L0X.h>
#include <CAN.h>            

VL53L0X sensor;

// Device and Sensor Configuration
const uint8_t DEVICE_ADDRESS = 0x04;            // Unique ID for this sensor node
const uint8_t SENSOR_TYPE_VL53L0X_SINGLE = 0x04; // Identifier for VL53L0X Single-Shot
const uint8_t DATA_PARAMETERS = 0x01;           // Number of data parameters (1: Range)

// CAN Configuration
const uint32_t CAN_ID = 0x301;                  // CAN ID for VL53L0X Single-Shot
const uint8_t CAN_DATA_LENGTH = 8;              // Length of CAN data in bytes

// Diagnostic Flags
uint8_t firmwareFlags = 0x01;                   // Firmware version 1
uint8_t hardwareFlags = 0x00;                   // Initialize hardware flags

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

#if defined LONG_RANGE
    // lower the return signal rate limit (default is 0.25 MCPS)
    sensor.setSignalRateLimit(0.1);
    // increase laser pulse periods (defaults are 14 and 10 PCLKs)
    sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
    firmwareFlags |= 0x02; // Set firmware flag for long range mode
#endif

#if defined HIGH_SPEED
    // reduce timing budget to 20 ms (default is about 33 ms)
    sensor.setMeasurementTimingBudget(20000);
    firmwareFlags |= 0x04; // Set firmware flag for high speed
#elif defined HIGH_ACCURACY
    // increase timing budget to 200 ms
    sensor.setMeasurementTimingBudget(200000);
    firmwareFlags |= 0x08; // Set firmware flag for high accuracy
#endif

    Serial.println("VL53L0X Single-Shot initialized successfully.");
}

void loop()
{
    uint16_t range = sensor.readRangeSingleMillimeters();
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
    canData[1] = SENSOR_TYPE_VL53L0X_SINGLE;
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
    if (range > 3000) {
        firmwareFlags |= 0x10; // Set bit 4 to indicate range exceeds threshold
    } else {
        firmwareFlags &= ~0x10; // Clear bit 4
    }
}

#include <Wire.h>               
#include "SparkFun_TMAG5273_Arduino_Library.h" 
#include <CAN.h>                

// Create a new sensor object
TMAG5273 sensor; 

// I2C default address
uint8_t i2cAddress = TMAG5273_I2C_ADDRESS_INITIAL;

// Set constants for setting up device
uint8_t conversionAverage = TMAG5273_X32_CONVERSION;
uint8_t magneticChannel = TMAG5273_XYX_ENABLE;
uint8_t angleCalculationMode = TMAG5273_XY_ANGLE_CALCULATION;

// Device and Sensor Configuration
const uint8_t DEVICE_ADDRESS = 0x03;        // Unique ID for this sensor node
const uint8_t SENSOR_TYPE_ANGLE = 0x03;      // Identifier for Angle Calculation Sensor
const uint8_t DATA_PARAMETERS = 0x01;        // Number of data parameters (1: Angle)

// CAN Configuration
const uint32_t CAN_ID = 0x300;               // CAN ID for Angle Calculation Sensor
const uint8_t CAN_DATA_LENGTH = 8;           // Length of CAN data in bytes

// Firmware and Hardware Flags
uint8_t firmwareFlags = 0x01;                // Initialize firmware flags ( Version 1)
uint8_t hardwareFlags = 0x00;                // Initialize hardware flags

// Function to calculate checksum (simple sum modulo 256)
uint8_t calculateChecksum(uint8_t data[], uint8_t length) {
    uint16_t sum = 0;
    for (uint8_t i = 0; i < length; i++) {
        sum += data[i];
    }
    return (uint8_t)(sum & 0xFF);
}

// Function to update diagnostic flags based on conditions
void updateDiagnosticFlags(float angle) {
    // Set firmware flag if angle exceeds 90 degrees
    if (angle > 90.0) {
        firmwareFlags |= 0x02; // Set bit 1: High-angle warning
    } else {
        firmwareFlags &= ~0x02; // Clear bit 1
    }

    // Example: Set hardware flag if sensor reading is invalid
    if (angle < -180.0 || angle > 180.0) {
        hardwareFlags |= 0x01; // Set bit 0: Sensor connectivity issue
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
        while(1); // Halt if CAN initialization fails
    }
    Serial.println("CAN bus initialized successfully.");

    // Begin example of the magnetic sensor code 
    Serial.println("TMAG5273 Example 3: Angle Calculations with Diagnostics");
    Serial.println("");
    // If begin is successful (true), then start example
    if(sensor.begin(i2cAddress, Wire) == true)
    {
        Serial.println("Sensor initialized successfully.");
    }
    else
    {
        Serial.println("Device failed to setup - Freezing code.");
        while(1); 
    }

    // Set the device to 32x average mode 
    sensor.setConvAvg(conversionAverage);

    // Choose new angle to calculate from
    // Can calculate angles between XYX, YXY, YZY, and XZX
    sensor.setMagneticChannel(magneticChannel);

    // Enable the angle calculation register
    // Can choose between XY, YZ, or XZ priority
    sensor.setAngleEn(angleCalculationMode);

}

void loop() 
{
    if((sensor.getMagneticChannel() != 0) && (sensor.getAngleEn() != 0)) // Checks if mag channels are on - turns on in setup()
    {
        float angle = sensor.getAngleResult();

        Serial.print("Calculated Angle: ");
        Serial.print(angle, 2);
        Serial.println("°");

        // Update diagnostic flags based on the angle
        updateDiagnosticFlags(angle);

        // Prepare CAN frame data
        uint8_t canData[8] = {0}; // Initialize all bytes to 0

        // Structure:
        // Byte 0: Device Address
        // Byte 1: Sensor Type
        // Byte 2: Data Parameters
        // Byte 3: Angle Data (8-bit integer, scaled)
        // Byte 4: Reserved or Additional Data
        // Byte 5: Firmware Flags
        // Byte 6: Hardware Flags
        // Byte 7: Checksum

        // Assign Device Address, Sensor Type, and Data Parameters
        canData[0] = DEVICE_ADDRESS;
        canData[1] = SENSOR_TYPE_ANGLE;
        canData[2] = DATA_PARAMETERS;

        // Scale angle to fit into 8-bit integer (-180° to 180° mapped to -128 to 127)
        // Calculate scaling factor
        float scaledAngle = angle / 180.0 * 127.0; // Map -180°-180° to -127-127
        int8_t angleScaled = constrain((int8_t)round(scaledAngle), -128, 127);

        // Assign Angle Data
        canData[3] = (uint8_t)angleScaled;

        // Byte 4 is reserved; set to 0x00 
        canData[4] = 0x00;

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
    }
    else
    {
        Serial.println("Mag Channels disabled, stopping..");
        while(1);
    }

    delay(1000); 
}


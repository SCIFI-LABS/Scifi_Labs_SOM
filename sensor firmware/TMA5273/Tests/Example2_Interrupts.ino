#include <Wire.h>                                    
#include "SparkFun_TMAG5273_Arduino_Library.h"       
#include <CAN.h>                                     

TMAG5273 sensor;  // Initialize hall-effect sensor

// Device and Sensor Configuration
const uint8_t DEVICE_ADDRESS = 0x06;            // Unique ID for this sensor node
const uint8_t SENSOR_TYPE_TMAG5273_INT = 0x06;  // Identifier for TMAG5273 Interrupt
const uint8_t DATA_PARAMETERS = 0x02;           // Number of data parameters (Interrupt Type, Data)

// CAN Configuration
const uint32_t CAN_ID = 0x401;                   // CAN ID for TMAG5273 Interrupts
const uint8_t CAN_DATA_LENGTH = 8;               // Length of CAN data in bytes

// Diagnostic Flags
uint8_t firmwareFlags = 0x01;                    // Firmware version 1
uint8_t hardwareFlags = 0x00;                    // Initialize hardware flags

// Interrupt Flag
volatile bool interruptFlag = false;

// Function to calculate checksum (simple sum modulo 256)
uint8_t calculateChecksum(uint8_t data[], uint8_t length) {
    uint16_t sum = 0;
    for (uint8_t i = 0; i < length; i++) {
        sum += data[i];
    }
    return (uint8_t)(sum & 0xFF);
}

// ISR to set the interrupt flag
void isr1() 
{
    interruptFlag = true;
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

    // Configure Interrupt Pin
    pinMode(intPin, INPUT);
    // Attach interrupt to pin 4 as a digital, change detection
    attachInterrupt(digitalPinToInterrupt(intPin), isr1, CHANGE);

    // Begin example of the magnetic sensor code 
    Serial.println("TMAG5273 Example 2: Interrupts");
    Serial.println("");

    // If begin is successful (1), then start example
    if (sensor.begin(i2cAddress, Wire) == true)
    {
        Serial.println("Begin");
    } 
    else 
    {
        Serial.println("Device failed to setup - Freezing code.");
        hardwareFlags |= 0x01; // Set hardware error flag
        while(1); // Runs forever
    }

    // Set interrupt through !INT
    sensor.setInterruptMode(TMAG5273_INTERRUPT_THROUGH_INT);

    // Set the !INT pin state - pulse for 10us
    sensor.setIntPinState(true);

    // Enable the interrupt response for the thresholds
    sensor.setThresholdEn(true);

    // Set X, Y, Z, and T Thresholds for interrupt to be triggered
    sensor.setXThreshold(5);            // mT
    //sensor.setYThreshold(5);          // mT
    //sensor.setZThreshold(5);          // mT
    //sensor.setTemperatureThreshold(50);  // C

    Serial.print("X Threshold Set: ");
    Serial.println(sensor.getXThreshold());
}

void loop()
{
    if (interruptFlag == true) 
    {
        interruptFlag = false;
        Serial.println("X Threshold Reached!");

        int xThresh = sensor.getXThreshold();
        Serial.print("X Threshold: ");
        Serial.println(xThresh);

        if (sensor.getMagneticChannel() != 0)  // Checks if mag channels are on - turns on in setup
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
            Serial.println(" °C");

            // Update diagnostic flags based on sensor data
            updateDiagnosticFlags(magX, magY, magZ, temp);

            // Prepare CAN frame data
            uint8_t canData[8] = {0}; // Initialize all bytes to 0

            // Assign Device Address, Sensor Type, and Data Parameters
            canData[0] = DEVICE_ADDRESS;
            canData[1] = SENSOR_TYPE_TMAG5273_INT;
            canData[2] = DATA_PARAMETERS;

            // Assign Interrupt Type 
            canData[3] = 0x01; // 0x01 indicates X Threshold

            // Assign Interrupt Data 
            // Scale magnetic data to preserve precision 
            int16_t magXScaled = round(magX * 100);
            canData[4] = highByte(magXScaled);
            canData[5] = lowByte(magXScaled);

            // Assign Firmware and Hardware Flags
            canData[6] = firmwareFlags;
            canData[7] = calculateChecksum(canData, 7);

            // Send the CAN frame
            CAN.beginPacket(CAN_ID);
            CAN.write(canData, CAN_DATA_LENGTH); 
            if (CAN.endPacket()) {
                Serial.println("CAN message sent successfully.");
            } else {
                Serial.println("Error sending CAN message.");
            }
        } 
        else 
        {
            Serial.println("Mag Channels disabled, stopping..");
            hardwareFlags |= 0x02; // Set hardware error flag
            // Prepare CAN frame with error flags
            uint8_t canData[8] = {0};

            canData[0] = DEVICE_ADDRESS;
            canData[1] = SENSOR_TYPE_TMAG5273_INT;
            canData[2] = DATA_PARAMETERS;

            // No sensor data available
            canData[3] = 0x00;
            canData[4] = 0x00;
            canData[5] = 0x00;

            // Assign Firmware and Hardware Flags
            canData[6] = firmwareFlags;
            canData[7] = calculateChecksum(canData, 7);

            // Send the CAN frame with error flags
            CAN.beginPacket(CAN_ID);
            CAN.write(canData, CAN_DATA_LENGTH);
            CAN.endPacket();

            while(1);
        }
    }
    else
    {
        Serial.println("Checking for Interrupts...");
    }

    delay(500);
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

    // Set hardware flag if magnetic data is out of expected range
    if (abs(magX) > 100.0 || abs(magY) > 100.0 || abs(magZ) > 100.0) {
        hardwareFlags |= 0x04; // Set bit 2 to indicate out-of-range magnetic data
    } else {
        hardwareFlags &= ~0x04; // Clear bit 2
    }
}

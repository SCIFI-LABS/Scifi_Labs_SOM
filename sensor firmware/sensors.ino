#include <Arduino.h>
#include <Wire.h>
#include <CAN.h>

// Sensor-specific libraries
#include "IIS328DQTR.h"        // For IIS328DQTR accelerometer
#include "PT1000.h"            // For PT1000 temperature sensor
#include "SparkFun_TMAG5273_Arduino_Library.h" // For TMAG5273
#include <VL53L0X.h>           // For VL53L0X distance sensor
#include <SimpleFOC.h>         // For motor control 

// ------------------------------
// Accelerometer (IIS328DQTR)
// ------------------------------
IIS328DQTR accelSensor(0x18);  // I2C address 0x18

// Device and Sensor Configuration for Accelerometer
const uint8_t ACCEL_DEVICE_ADDRESS = 0x02;
const uint8_t ACCEL_SENSOR_TYPE = 0x02;
const uint8_t ACCEL_DATA_PARAMETERS = 0x03;  // X, Y, Z

// CAN Configuration for Accelerometer
const uint32_t ACCEL_CAN_ID = 0x200;
const uint8_t CAN_DATA_LENGTH = 8;

// Firmware and Hardware Flags for Accelerometer
uint8_t accelFirmwareFlags = 0x00;
uint8_t accelHardwareFlags = 0x00;

// ------------------------------
// PT1000 Temperature Sensor
// ------------------------------
const int pt1000AnalogPin = A0;       // Analog input pin
const float pt1000Vcc = 5.0;          // Supply voltage
const float pt1000Rref = 1000.0;      // Reference resistor (1kΩ)

// Device and Sensor Configuration for PT1000
const uint8_t TEMP_DEVICE_ADDRESS = 0x01;
const uint8_t TEMP_SENSOR_TYPE = 0x01;
const uint8_t TEMP_DATA_PARAMETERS = 0x01;  // Temperature

// CAN Configuration for Temperature Sensor
const uint32_t TEMP_CAN_ID = 0x100;

// Firmware and Hardware Flags for Temperature Sensor
uint8_t tempFirmwareFlags = 0x00;
uint8_t tempHardwareFlags = 0x00;

// ------------------------------
// Hall-Effect Sensors (TMAG5273)
// ------------------------------
#define TMAG5273_ADDRESS_BASIC      0x22
#define TMAG5273_ADDRESS_INTERRUPT  0x23
#define TMAG5273_ADDRESS_ANGLE      0x24

TMAG5273 tmag5273_basic(TMAG5273_ADDRESS_BASIC);
TMAG5273 tmag5273_interrupt(TMAG5273_ADDRESS_INTERRUPT);
TMAG5273 tmag5273_angle(TMAG5273_ADDRESS_ANGLE);

// Device and Sensor Configuration for TMAG5273 Basic Readings
const uint8_t TMAG_BASIC_DEVICE_ADDRESS = 0x05;
const uint8_t TMAG_BASIC_SENSOR_TYPE = 0x05;
const uint8_t TMAG_BASIC_DATA_PARAMETERS_MAGXY = 0x02;
const uint8_t TMAG_BASIC_DATA_PARAMETERS_MAGZTEMP = 0x03;
const uint8_t TMAG_BASIC_DATA_PARAMETERS_DIAGNOSTICS = 0x04;

// CAN Configuration for TMAG5273 Basic Readings
const uint32_t TMAG_BASIC_CAN_ID_MAGXY = 0x400;
const uint32_t TMAG_BASIC_CAN_ID_MAGZTEMP = 0x401;
const uint32_t TMAG_BASIC_CAN_ID_DIAGNOSTICS = 0x402;

// Diagnostic Flags for TMAG5273 Basic Readings
uint8_t tmagBasicFirmwareFlags = 0x01;
uint8_t tmagBasicHardwareFlags = 0x00;

// ------------------------------
// VL53L0X Sensors
// ------------------------------
#define VL53L0X_XSHUT_CONTINUOUS 2  // XSHUT pin for continuous sensor
#define VL53L0X_XSHUT_SINGLE     3  // XSHUT pin for single-shot sensor

VL53L0X vl53l0x_continuous;  // Continuous readings
VL53L0X vl53l0x_single;      // Single-shot readings

// Device and Sensor Configuration for VL53L0X Continuous
const uint8_t VL53L0X_CONT_DEVICE_ADDRESS = 0x07;
const uint8_t VL53L0X_CONT_SENSOR_TYPE = 0x07;
const uint8_t VL53L0X_CONT_DATA_PARAMETERS = 0x01;  // Range

// CAN Configuration for VL53L0X Continuous
const uint32_t VL53L0X_CONT_CAN_ID = 0x500;

// Diagnostic Flags for VL53L0X Continuous
uint8_t vl53l0xContFirmwareFlags = 0x01;
uint8_t vl53l0xContHardwareFlags = 0x00;

// Device and Sensor Configuration for VL53L0X Single-Shot
const uint8_t VL53L0X_SINGLE_DEVICE_ADDRESS = 0x08;
const uint8_t VL53L0X_SINGLE_SENSOR_TYPE = 0x08;
const uint8_t VL53L0X_SINGLE_DATA_PARAMETERS = 0x01;  // Range

// CAN Configuration for VL53L0X Single-Shot
const uint32_t VL53L0X_SINGLE_CAN_ID = 0x501;

// Diagnostic Flags for VL53L0X Single-Shot
uint8_t vl53l0xSingleFirmwareFlags = 0x01;
uint8_t vl53l0xSingleHardwareFlags = 0x00;

// ------------------------------
// Function Prototypes
// ------------------------------
uint8_t calculateChecksum(uint8_t data[], uint8_t length);

// Accelerometer Functions
void readAccelerometerAndSendCAN();
void updateAccelDiagnosticFlags(float x, float y, float z);

// PT1000 Functions
void readPT1000AndSendCAN();
void updatePT1000DiagnosticFlags(float temperature);
float readPT1000(int analogPin, float Vcc, float Rref);
float calculateTemperature(float resistance);

// TMAG5273 Basic Functions
void readTMAG5273BasicAndSendCAN();
void updateTMAG5273BasicDiagnosticFlags(float magX, float magY, float magZ, float temp);

// VL53L0X Continuous Functions
void readVL53L0XContinuousAndSendCAN();
void updateVL53L0XContDiagnosticFlags(uint16_t range, bool timeout);

// VL53L0X Single-Shot Functions
void readVL53L0XSingleShotAndSendCAN();
void updateVL53L0XSingleDiagnosticFlags(uint16_t range, bool timeout);

// ------------------------------
// Setup Function
// ------------------------------
void setup() {
    // Initialize Serial
    Serial.begin(115200);
    while (!Serial) {
        ; // Wait for Serial to initialize
    }

    // Initialize I2C
    Wire.begin();

    // Initialize CAN bus at 500 kbps
    if (!CAN.begin(500E3)) {
        Serial.println("CAN bus initialization failed.");
        while (1); // Halt if CAN initialization fails
    }
    Serial.println("CAN bus initialized successfully.");

    // Initialize Accelerometer
    Serial.println("Initializing IIS328DQTR Accelerometer...");
    if (!accelSensor.begin()) {
        Serial.println("Failed to initialize IIS328DQTR!");
        while (1);
    }
    Serial.println("IIS328DQTR initialized successfully.");

    // Initialize PT1000 sensor (no initialization needed)

    // Initialize TMAG5273 Basic Readings
    Serial.println("Initializing TMAG5273 Basic Readings...");
    if (tmag5273_basic.begin(TMAG5273_ADDRESS_BASIC, Wire)) {
        Serial.println("TMAG5273 Basic initialized successfully.");
        tmag5273_basic.setMagneticChannel(TMAG5273_XYZ_ENABLE);
        tmag5273_basic.setTemperatureEn(true);
    } else {
        Serial.println("Failed to initialize TMAG5273 Basic.");
        while (1);
    }

    // Initialize VL53L0X sensors
    pinMode(VL53L0X_XSHUT_CONTINUOUS, OUTPUT);
    pinMode(VL53L0X_XSHUT_SINGLE, OUTPUT);

    // Shutdown both sensors
    digitalWrite(VL53L0X_XSHUT_CONTINUOUS, LOW);
    digitalWrite(VL53L0X_XSHUT_SINGLE, LOW);
    delay(10);

    // Initialize VL53L0X Continuous sensor
    digitalWrite(VL53L0X_XSHUT_CONTINUOUS, HIGH);
    delay(10);
    if (!vl53l0x_continuous.init()) {
        Serial.println("Failed to initialize VL53L0X Continuous!");
        while (1);
    }
    vl53l0x_continuous.setAddress(0x30);
    vl53l0x_continuous.startContinuous();

    // Initialize VL53L0X Single-Shot sensor
    digitalWrite(VL53L0X_XSHUT_SINGLE, HIGH);
    delay(10);
    if (!vl53l0x_single.init()) {
        Serial.println("Failed to initialize VL53L0X Single-Shot!");
        while (1);
    }
    vl53l0x_single.setAddress(0x31);

    Serial.println("All sensors initialized successfully.");
}

// ------------------------------
// Loop Function
// ------------------------------
void loop() {
    // Read and send data from all sensors
    readAccelerometerAndSendCAN();
    readPT1000AndSendCAN();
    readTMAG5273BasicAndSendCAN();
    readVL53L0XContinuousAndSendCAN();
    readVL53L0XSingleShotAndSendCAN();


    delay(100);
}

// ------------------------------
// Accelerometer Functions
// ------------------------------
void readAccelerometerAndSendCAN() {
    float x, y, z;

    // Read acceleration data
    if (accelSensor.readAcceleration(x, y, z)) {
        Serial.print("Acceleration [g]: X=");
        Serial.print(x, 3);
        Serial.print(" Y=");
        Serial.print(y, 3);
        Serial.print(" Z=");
        Serial.println(z, 3);
    } else {
        Serial.println("Failed to read acceleration data.");
        accelHardwareFlags |= 0x01; // Set error flag
        x = y = z = 0; // Default values
    }

    // Update diagnostic flags
    updateAccelDiagnosticFlags(x, y, z);

    // Prepare CAN frame
    uint8_t canData[8] = {0};
    canData[0] = ACCEL_DEVICE_ADDRESS;
    canData[1] = ACCEL_SENSOR_TYPE;
    canData[2] = ACCEL_DATA_PARAMETERS;

    // Scale data
    int8_t xScaled = constrain((int8_t)(x * 100), -128, 127);
    int8_t yScaled = constrain((int8_t)(y * 100), -128, 127);
    int8_t zScaled = constrain((int8_t)(z * 100), -128, 127);

    canData[3] = (uint8_t)xScaled;
    canData[4] = (uint8_t)yScaled;
    canData[5] = (uint8_t)zScaled;

    canData[6] = accelFirmwareFlags;

    // Calculate checksum
    canData[7] = calculateChecksum(canData, 7);

    // Send CAN frame
    CAN.beginPacket(ACCEL_CAN_ID);
    CAN.write(canData, CAN_DATA_LENGTH);
    CAN.endPacket();
}

void updateAccelDiagnosticFlags(float x, float y, float z) {
    if (abs(x) > 2.0 || abs(y) > 2.0 || abs(z) > 2.0) {
        accelFirmwareFlags |= 0x02; // High acceleration warning
    } else {
        accelFirmwareFlags &= ~0x02;
    }

    if (accelHardwareFlags & 0x01) {
        accelHardwareFlags &= ~0x01; // Clear error flag if data is valid
    }
}

// ------------------------------
// PT1000 Functions
// ------------------------------
void readPT1000AndSendCAN() {
    float Rpt1000 = readPT1000(pt1000AnalogPin, pt1000Vcc, pt1000Rref);
    float temperature = calculateTemperature(Rpt1000);

    Serial.print("Resistance: ");
    Serial.print(Rpt1000, 1);
    Serial.print(" Ω | Temperature: ");
    Serial.print(temperature, 2);
    Serial.println(" °C");

    updatePT1000DiagnosticFlags(temperature);

    uint8_t canData[8] = {0};
    canData[0] = TEMP_DEVICE_ADDRESS;
    canData[1] = TEMP_SENSOR_TYPE;
    canData[2] = TEMP_DATA_PARAMETERS;

    int16_t tempScaled = round(temperature * 100);
    canData[3] = highByte(tempScaled);
    canData[4] = lowByte(tempScaled);

    canData[5] = tempFirmwareFlags;
    canData[6] = tempHardwareFlags;

    canData[7] = calculateChecksum(canData, 7);

    CAN.beginPacket(TEMP_CAN_ID);
    CAN.write(canData, CAN_DATA_LENGTH);
    CAN.endPacket();
}

void updatePT1000DiagnosticFlags(float temperature) {
    if (temperature > 100.0) {
        tempFirmwareFlags |= 0x02; // High-temperature warning
    } else {
        tempFirmwareFlags &= ~0x02;
    }

    if (temperature < -50.0 || temperature > 150.0) {
        tempHardwareFlags |= 0x01; // Sensor error
    } else {
        tempHardwareFlags &= ~0x01;
    }
}

float readPT1000(int analogPin, float Vcc, float Rref) {
    int analogValue = analogRead(analogPin);
    float voltage = (analogValue / 1023.0) * Vcc;
    float resistance = (Vcc * Rref / voltage) - Rref;
    return resistance;
}

float calculateTemperature(float resistance) {
    // Simplified Callendar-Van Dusen equation for PT1000
    float temperature = (resistance - 1000.0) / 3.85;
    return temperature;
}

// ------------------------------
// TMAG5273 Basic Functions
// ------------------------------
void readTMAG5273BasicAndSendCAN() {
    float magX = tmag5273_basic.getXData();
    float magY = tmag5273_basic.getYData();
    float magZ = tmag5273_basic.getZData();
    float temp = tmag5273_basic.getTemp();

    Serial.print("Magnetic Fields (mT): X=");
    Serial.print(magX, 2);
    Serial.print(", Y=");
    Serial.print(magY, 2);
    Serial.print(", Z=");
    Serial.println(magZ, 2);
    Serial.print("Temperature: ");
    Serial.print(temp, 2);
    Serial.println(" °C");

    updateTMAG5273BasicDiagnosticFlags(magX, magY, magZ, temp);

    // Frame 1: MagX & MagY
    uint8_t canDataMagXY[8] = {0};
    canDataMagXY[0] = TMAG_BASIC_DEVICE_ADDRESS;
    canDataMagXY[1] = TMAG_BASIC_SENSOR_TYPE;
    canDataMagXY[2] = TMAG_BASIC_DATA_PARAMETERS_MAGXY;

    int16_t magXScaled = round(magX * 100);
    int16_t magYScaled = round(magY * 100);

    canDataMagXY[3] = highByte(magXScaled);
    canDataMagXY[4] = lowByte(magXScaled);
    canDataMagXY[5] = highByte(magYScaled);
    canDataMagXY[6] = lowByte(magYScaled);

    canDataMagXY[7] = calculateChecksum(canDataMagXY, 7);

    CAN.beginPacket(TMAG_BASIC_CAN_ID_MAGXY);
    CAN.write(canDataMagXY, CAN_DATA_LENGTH);
    CAN.endPacket();

    // Frame 2: MagZ & Temperature
    uint8_t canDataMagZTemp[8] = {0};
    canDataMagZTemp[0] = TMAG_BASIC_DEVICE_ADDRESS;
    canDataMagZTemp[1] = TMAG_BASIC_SENSOR_TYPE;
    canDataMagZTemp[2] = TMAG_BASIC_DATA_PARAMETERS_MAGZTEMP;

    int16_t magZScaled = round(magZ * 100);
    int16_t tempScaled = round(temp * 100);

    canDataMagZTemp[3] = highByte(magZScaled);
    canDataMagZTemp[4] = lowByte(magZScaled);
    canDataMagZTemp[5] = highByte(tempScaled);
    canDataMagZTemp[6] = lowByte(tempScaled);

    canDataMagZTemp[7] = calculateChecksum(canDataMagZTemp, 7);

    CAN.beginPacket(TMAG_BASIC_CAN_ID_MAGZTEMP);
    CAN.write(canDataMagZTemp, CAN_DATA_LENGTH);
    CAN.endPacket();

    // Frame 3: Diagnostics
    uint8_t canDataDiagnostics[8] = {0};
    canDataDiagnostics[0] = TMAG_BASIC_DEVICE_ADDRESS;
    canDataDiagnostics[1] = TMAG_BASIC_SENSOR_TYPE;
    canDataDiagnostics[2] = TMAG_BASIC_DATA_PARAMETERS_DIAGNOSTICS;

    canDataDiagnostics[3] = tmagBasicFirmwareFlags;
    canDataDiagnostics[4] = tmagBasicHardwareFlags;

    canDataDiagnostics[7] = calculateChecksum(canDataDiagnostics, 7);

    CAN.beginPacket(TMAG_BASIC_CAN_ID_DIAGNOSTICS);
    CAN.write(canDataDiagnostics, CAN_DATA_LENGTH);
    CAN.endPacket();
}

void updateTMAG5273BasicDiagnosticFlags(float magX, float magY, float magZ, float temp) {
    if (temp > 50.0) {
        tmagBasicFirmwareFlags |= 0x02; // High temperature warning
    } else {
        tmagBasicFirmwareFlags &= ~0x02;
    }

    if (abs(magX) > 100.0 || abs(magY) > 100.0 || abs(magZ) > 100.0) {
        tmagBasicHardwareFlags |= 0x01; // Magnetic field out of range
    } else {
        tmagBasicHardwareFlags &= ~0x01;
    }
}

// ------------------------------
// VL53L0X Continuous Functions
// ------------------------------
void readVL53L0XContinuousAndSendCAN() {
    uint16_t range = vl53l0x_continuous.readRangeContinuousMillimeters();
    bool timeout = vl53l0x_continuous.timeoutOccurred();

    Serial.print("VL53L0X Continuous Range: ");
    Serial.print(range);
    Serial.print(" mm");
    if (timeout) {
        Serial.print(" TIMEOUT");
    }
    Serial.println();

    updateVL53L0XContDiagnosticFlags(range, timeout);

    uint8_t canData[8] = {0};
    canData[0] = VL53L0X_CONT_DEVICE_ADDRESS;
    canData[1] = VL53L0X_CONT_SENSOR_TYPE;
    canData[2] = VL53L0X_CONT_DATA_PARAMETERS;

    canData[3] = highByte(range);
    canData[4] = lowByte(range);

    canData[5] = vl53l0xContFirmwareFlags;
    canData[6] = vl53l0xContHardwareFlags;

    canData[7] = calculateChecksum(canData, 7);

    CAN.beginPacket(VL53L0X_CONT_CAN_ID);
    CAN.write(canData, CAN_DATA_LENGTH);
    CAN.endPacket();
}

void updateVL53L0XContDiagnosticFlags(uint16_t range, bool timeout) {
    if (timeout) {
        vl53l0xContHardwareFlags |= 0x02; // Timeout occurred
    } else {
        vl53l0xContHardwareFlags &= ~0x02;
    }

    if (range > 2000) {
        vl53l0xContFirmwareFlags |= 0x04; // Range exceeds threshold
    } else {
        vl53l0xContFirmwareFlags &= ~0x04;
    }
}

// ------------------------------
// VL53L0X Single-Shot Functions
// ------------------------------
void readVL53L0XSingleShotAndSendCAN() {
    uint16_t range = vl53l0x_single.readRangeSingleMillimeters();
    bool timeout = vl53l0x_single.timeoutOccurred();

    Serial.print("VL53L0X Single-Shot Range: ");
    Serial.print(range);
    Serial.print(" mm");
    if (timeout) {
        Serial.print(" TIMEOUT");
    }
    Serial.println();

    updateVL53L0XSingleDiagnosticFlags(range, timeout);

    uint8_t canData[8] = {0};
    canData[0] = VL53L0X_SINGLE_DEVICE_ADDRESS;
    canData[1] = VL53L0X_SINGLE_SENSOR_TYPE;
    canData[2] = VL53L0X_SINGLE_DATA_PARAMETERS;

    canData[3] = highByte(range);
    canData[4] = lowByte(range);

    canData[5] = vl53l0xSingleFirmwareFlags;
    canData[6] = vl53l0xSingleHardwareFlags;

    canData[7] = calculateChecksum(canData, 7);

    CAN.beginPacket(VL53L0X_SINGLE_CAN_ID);
    CAN.write(canData, CAN_DATA_LENGTH);
    CAN.endPacket();
}

void updateVL53L0XSingleDiagnosticFlags(uint16_t range, bool timeout) {
    if (timeout) {
        vl53l0xSingleHardwareFlags |= 0x02; // Timeout occurred
    } else {
        vl53l0xSingleHardwareFlags &= ~0x02;
    }

    if (range > 3000) {
        vl53l0xSingleFirmwareFlags |= 0x10; // Range exceeds threshold
    } else {
        vl53l0xSingleFirmwareFlags &= ~0x10;
    }
}

// ------------------------------
// Checksum Function
// ------------------------------
uint8_t calculateChecksum(uint8_t data[], uint8_t length) {
    uint16_t sum = 0;
    for (uint8_t i = 0; i < length; i++) {
        sum += data[i];
    }
    return (uint8_t)(sum & 0xFF);
}

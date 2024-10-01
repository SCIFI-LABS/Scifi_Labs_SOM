#include <Arduino.h>
#include <CAN.h>            // Built-in CAN library for Arduino Due or similar
#include "PT1000.h"

// PT1000 sensor setup
const int analogPin = A0;       // Analog input pin connected to the voltage divider
const float Vcc = 5.0;          // Supply voltage (5V from Arduino)
const float Rref = 1000.0;      // Reference resistor value in ohms (1kΩ)

// Device and Sensor Configuration
const uint8_t DEVICE_ADDRESS = 0x01;     // Unique ID for this sensor node
const uint8_t SENSOR_TYPE_TEMP = 0x01;    // Identifier for Temperature Sensor
const uint8_t DATA_PARAMETERS = 0x01;     // Number of data parameters (1: Temperature)

// CAN Configuration
const uint32_t CAN_ID = 0x100;            // CAN ID for Temperature Sensor
const uint8_t CAN_DATA_LENGTH = 6;        // Length of CAN data in bytes

// Function to read PT1000 resistance
float readPT1000(int pin, float Vcc, float Rref) {
  int adcValue = analogRead(pin);
  float voltage = (adcValue / 1023.0) * Vcc;
  
  // Prevent division by zero
  if ((Vcc - voltage) == 0) {
    return 0;
  }
  
  float Rpt1000 = (voltage * Rref) / (Vcc - voltage);
  return Rpt1000;
}

// Function to calculate temperature from PT1000 resistance
float calculateTemperature(float resistance) {
  // Callendar-Van Dusen equation coefficients for PT1000
  const float a = 3.9083e-3;
  const float b = -5.775e-7;
  const float r0 = 1000.0; // Resistance at 0°C for PT1000

  // Solving the quadratic equation for temperature
  // Note: This is a simplified approach and may need calibration for accuracy
  if (resistance <= 0) {
    return -273.15; // Absolute zero check
  }

  float temp = (-a * r0 + sqrt(a * a * r0 * r0 - 4 * b * r0 * (r0 - resistance))) / (2 * b * r0);
  return temp;
}

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
    while (1); // Halt if CAN initialization fails
  }

  Serial.println("CAN bus initialized successfully.");
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

  // Prepare CAN frame data
  uint8_t canData[8] = {0}; // Initialize all bytes to 0

  // Structure:
  // Byte 0: Device Address
  // Byte 1: Sensor Type
  // Byte 2: Data Parameters
  // Byte 3-4: Temperature Data (16-bit integer, scaled)
  // Byte 5: Checksum
  // Byte 6-7: Reserved (set to 0)

  // Assign Device Address, Sensor Type, and Data Parameters
  canData[0] = DEVICE_ADDRESS;
  canData[1] = SENSOR_TYPE_TEMP;
  canData[2] = DATA_PARAMETERS;

  // Scale temperature to preserve two decimal places (e.g., 25.67°C -> 2567)
  int16_t tempScaled = temperature * 100;

  // Assign Temperature Data (big-endian)
  canData[3] = highByte(tempScaled);
  canData[4] = lowByte(tempScaled);

  // Calculate checksum for bytes 0-4
  canData[5] = calculateChecksum(canData, 5);

  // Bytes 6 and 7 are reserved for future use or can be set to zero
  canData[6] = 0x00;
  canData[7] = 0x00;

  // Send the CAN frame
  CAN.beginPacket(CAN_ID);
  CAN.write(canData, CAN_DATA_LENGTH); // Write only the first 6 bytes
  CAN.endPacket(); // Send the packet

  Serial.println("CAN message sent successfully.");

  delay(1000); // Delay for readability
}

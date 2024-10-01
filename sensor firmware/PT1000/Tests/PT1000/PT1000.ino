#include "PT1000.h"

const int analogPin = A0;   // Analog input pin connected to the voltage divider
const float Vcc = 5.0;      // Supply voltage (5V from Arduino)
const float Rref = 1000.0;  // Reference resistor value in ohms (1kΩ)

void setup() {
  Serial.begin(9600);
}

void loop() {
  // Read the PT1000 resistance
  float Rpt1000 = readPT1000(analogPin, Vcc, Rref);

  // Calculate temperature from resistance
  float temperature = calculateTemperature(Rpt1000);

  // Output the results
  Serial.print("Resistance: ");
  Serial.print(Rpt1000, 1);
  Serial.print(" Ω | Temperature: ");
  Serial.print(temperature, 2);
  Serial.println(" °C");

  delay(1000); // Delay for readability
}


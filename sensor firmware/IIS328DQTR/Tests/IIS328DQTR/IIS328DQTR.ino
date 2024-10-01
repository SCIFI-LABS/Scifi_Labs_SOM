#include <Arduino.h>
#include "IIS328DQTR.h"


IIS328DQTR accelerometer(0x18);

void setup() {
    Serial.begin(9600);
    while (!Serial) {
        ; 
    }

    Serial.println("Initializing IIS328DQTR Accelerometer...");

    if (!accelerometer.begin()) {
        Serial.println("Failed to initialize IIS328DQTR!");
        while (1); 
    }

    Serial.println("IIS328DQTR initialized successfully.");
}

void loop() {
    float x, y, z;
    if (accelerometer.readAcceleration(x, y, z)) {
        Serial.print("Acceleration [g]: X=");
        Serial.print(x, 3);
        Serial.print(" Y=");
        Serial.print(y, 3);
        Serial.print(" Z=");
        Serial.println(z, 3);
    } else {
        Serial.println("Failed to read acceleration data.");
    }

    delay(500); 
}

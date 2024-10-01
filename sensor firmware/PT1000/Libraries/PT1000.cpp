#include "PT1000.h"

// Function to read PT1000 resistance
float readPT1000(int analogPin, float Vcc, float Rref) {
    int analogValue = analogRead(analogPin);
    float Vout = (analogValue / 1023.0) * Vcc;
    
    // Prevent division by zero
    if ((Vcc - Vout) == 0) {
        return 0;
    }
    
    float Rpt1000 = ((Vcc * Rref) / Vout) - Rref;
    return Rpt1000;
}

// Function to calculate temperature from PT1000 resistance
float calculateTemperature(float Rt) {
    if (Rt <= 0) {
        return -273.15; // Absolute zero check
    }

    // Callendar-Van Dusen equation coefficients for PT1000
    // Rt = R0 * (1 + A*t + B*t^2)
    // Rearranged to solve for t:
    // B*t^2 + A*t + (1 - Rt/R0) = 0
    float A = PT1000_A;
    float B = PT1000_B;
    float C = 1 - (Rt / PT1000_R0);

    // Calculate discriminant
    float discriminant = A * A - 4 * B * C;

    if (discriminant < 0) {
        // No real solution
        return 0;
    }

    // Calculate temperature 
    float t = (-A + sqrt(discriminant)) / (2 * B);
    return t;
}

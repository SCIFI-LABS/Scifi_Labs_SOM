#include "PT1000.h"

float readPT1000(int analogPin, float Vcc, float Rref) {
    int analogValue = analogRead(analogPin);
    float Vout = (analogValue / 1023.0) * Vcc;
    float Rpt1000 = ((Vcc * Rref) / Vout) - Rref;
    return Rpt1000;
}


float calculateTemperature(float Rt) {
    float t; 

    if (Rt >= PT1000_R0) {
        
        t = (-PT1000_A + sqrt(PT1000_A * PT1000_A - 4 * PT1000_B * (1 - Rt / PT1000_R0))) / (2 * PT1000_B);
    } else {
        
        t = (-PT1000_A + sqrt(PT1000_A * PT1000_A - 4 * PT1000_B * (1 - Rt / PT1000_R0))) / (2 * PT1000_B);
    }

    return t;
}

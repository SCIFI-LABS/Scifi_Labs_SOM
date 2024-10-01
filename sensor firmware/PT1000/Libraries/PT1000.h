#ifndef PT1000_H
#define PT1000_H

#include <Arduino.h>

// PT1000 Constants
#define PT1000_R0 1000.0 
#define PT1000_A 3.9083e-3
#define PT1000_B -5.775e-7
#define PT1000_C -4.183e-12 

// Function Declarations
float readPT1000(int analogPin, float Vcc, float Rref);
float calculateTemperature(float Rt);

#endif // PT1000_H

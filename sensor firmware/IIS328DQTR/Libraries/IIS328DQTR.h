#ifndef IIS328DQTR_H
#define IIS328DQTR_H

#include <Arduino.h>
#include <Wire.h>

class IIS328DQTR {
public:
    // Constructor
    IIS328DQTR(uint8_t i2cAddress = 0x18);

    // Initialization
    bool begin();

    // Configuration
    void setDataRate(uint8_t dataRate);
    void setFullScale(uint8_t fullScale);
    void enableAxis(uint8_t axis);
    void disableAxis(uint8_t axis);

    // Data Reading
    bool readAcceleration(float &x, float &y, float &z);

private:
    uint8_t _i2cAddress;

    // Register Addresses
    static const uint8_t WHO_AM_I_REG = 0x0F;
    static const uint8_t CTRL_REG1 = 0x20;
    static const uint8_t CTRL_REG4 = 0x23;
    static const uint8_t OUT_X_L = 0x28;
    static const uint8_t OUT_X_H = 0x29;
    static const uint8_t OUT_Y_L = 0x2A;
    static const uint8_t OUT_Y_H = 0x2B;
    static const uint8_t OUT_Z_L = 0x2C;
    static const uint8_t OUT_Z_H = 0x2D;

    // Helper Functions
    bool writeRegister(uint8_t reg, uint8_t value);
    uint8_t readRegister(uint8_t reg);
    bool readMultipleRegisters(uint8_t startReg, uint8_t *buffer, uint8_t length);
};

#endif // IIS328DQTR_H

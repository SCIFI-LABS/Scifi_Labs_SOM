#include "IIS328DQTR.h"

IIS328DQTR::IIS328DQTR(uint8_t i2cAddress) : _i2cAddress(i2cAddress) {}

bool IIS328DQTR::begin() {
    Wire.begin();

    uint8_t whoami = readRegister(WHO_AM_I_REG);
    if (whoami != 0x32) { 
        return false;
    }

    setDataRate(0x27);   // Example data rate configuration
    setFullScale(0x80);  // Example full-scale configuration

    return true;
}

void IIS328DQTR::setDataRate(uint8_t dataRate) {
    writeRegister(CTRL_REG1, dataRate);
}

void IIS328DQTR::setFullScale(uint8_t fullScale) {
    writeRegister(CTRL_REG4, fullScale);
}

void IIS328DQTR::enableAxis(uint8_t axis) {
    uint8_t reg = readRegister(CTRL_REG1);
    reg |= axis;
    writeRegister(CTRL_REG1, reg);
}

void IIS328DQTR::disableAxis(uint8_t axis) {
    uint8_t reg = readRegister(CTRL_REG1);
    reg &= ~axis;
    writeRegister(CTRL_REG1, reg);
}

bool IIS328DQTR::readAcceleration(float &x, float &y, float &z) {
    uint8_t buffer[6];
    if (!readMultipleRegisters(OUT_X_L, buffer, 6)) {
        return false;
    }

    int16_t x_raw = (int16_t)(buffer[1] << 8 | buffer[0]);
    int16_t y_raw = (int16_t)(buffer[3] << 8 | buffer[2]);
    int16_t z_raw = (int16_t)(buffer[5] << 8 | buffer[4]);

    // Convert raw data to acceleration in g's
    // Assuming the full scale is set appropriately
    x = x_raw * 0.001f; // Example scaling factor
    y = y_raw * 0.001f;
    z = z_raw * 0.001f;

    return true;
}

bool IIS328DQTR::writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(_i2cAddress);
    Wire.write(reg);
    Wire.write(value);
    return (Wire.endTransmission() == 0);
}

uint8_t IIS328DQTR::readRegister(uint8_t reg) {
    Wire.beginTransmission(_i2cAddress);
    Wire.write(reg);
    Wire.endTransmission(false); // Send repeated start
    Wire.requestFrom(_i2cAddress, (uint8_t)1);
    if (Wire.available()) {
        return Wire.read();
    }
    return 0;
}

bool IIS328DQTR::readMultipleRegisters(uint8_t startReg, uint8_t *buffer, uint8_t length) {
    Wire.beginTransmission(_i2cAddress);
    Wire.write(startReg | 0x80); // Set auto-increment bit if required by the sensor
    if (Wire.endTransmission(false) != 0) { 
        return false;
    }

    Wire.requestFrom(_i2cAddress, length);
    uint8_t i = 0;
    while (Wire.available() && i < length) {
        buffer[i++] = Wire.read();
    }

    return (i == length);
}


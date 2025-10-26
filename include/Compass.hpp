#ifndef COMPASS_HPP
#define COMPASS_HPP

#include "driver/i2c.h"
#include <cstdint>

// QMC5883L I2C address (most common for HW-127 modules)
#define QMC5883L_ADDR 0x0D

// HMC5883L address (for genuine chips)
#define HMC5883L_ADDR 0x1E

// QMC5883L register addresses
#define QMC5883L_REG_DATA_X_LSB  0x00
#define QMC5883L_REG_DATA_X_MSB  0x01
#define QMC5883L_REG_DATA_Y_LSB  0x02
#define QMC5883L_REG_DATA_Y_MSB  0x03
#define QMC5883L_REG_DATA_Z_LSB  0x04
#define QMC5883L_REG_DATA_Z_MSB  0x05
#define QMC5883L_REG_STATUS      0x06
#define QMC5883L_REG_TEMP_LSB    0x07
#define QMC5883L_REG_TEMP_MSB    0x08
#define QMC5883L_REG_CONTROL1    0x09
#define QMC5883L_REG_CONTROL2    0x0A
#define QMC5883L_REG_PERIOD      0x0B
#define QMC5883L_REG_CHIP_ID     0x0D

class Compass {
public:
    Compass();
    ~Compass();

    // Initialize the compass
    bool init();

    // Read raw magnetic field values
    bool readRaw(int16_t &x, int16_t &y, int16_t &z);

    // Get heading in degrees (0-359)
    // 0 = North, 90 = East, 180 = South, 270 = West
    float getHeading();

    // Set magnetic declination for your location (in degrees)
    void setDeclination(float declination);

    // Calibrate the compass
    void startCalibration();
    void updateCalibration(int16_t x, int16_t y, int16_t z);
    void finishCalibration();

    // Set calibration values manually
    void setCalibrationOffsets(int16_t x, int16_t y, int16_t z);
    void setCalibrationScales(float x, float y, float z);

    // Check if compass is working
    bool isConnected();

private:
    // I2C configuration
    i2c_port_t i2c_port;

    // Compass configuration
    float declination;
    uint8_t device_address;  // Store the actual device address found
    bool is_qmc;  // Flag to indicate if it's QMC5883L

    // Calibration offsets
    int16_t x_offset;
    int16_t y_offset;
    int16_t z_offset;

    // Calibration scales
    float x_scale;
    float y_scale;
    float z_scale;

    // Calibration min/max values
    int16_t x_min, x_max;
    int16_t y_min, y_max;
    int16_t z_min, z_max;

    bool calibrating;

    // I2C helper functions
    bool writeRegister(uint8_t reg, uint8_t value);
    bool readRegister(uint8_t reg, uint8_t &value);
    bool readRegisters(uint8_t reg, uint8_t *data, size_t len);
};

#endif // COMPASS_HPP

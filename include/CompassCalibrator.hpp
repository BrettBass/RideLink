#ifndef COMPASS_CALIBRATOR_HPP
#define COMPASS_CALIBRATOR_HPP

#include "Compass.hpp"
#include "Display.hpp"
#include "nvs_flash.h"
#include "nvs.h"
#include <vector>

// Structure to hold calibration data
struct CalibrationData {
    // Hard iron offsets
    int16_t x_offset;
    int16_t y_offset;
    int16_t z_offset;

    // Soft iron scale factors
    float x_scale;
    float y_scale;
    float z_scale;

    // Additional calibration for heading accuracy
    float heading_offset;  // Manual offset to match true north

    // Calibration quality metrics
    uint32_t sample_count;
    float coverage_score;  // 0-100% indicating how well the sphere was covered

    // Checksum for data integrity
    uint32_t checksum;

    CalibrationData() :
        x_offset(0), y_offset(0), z_offset(0),
        x_scale(1.0f), y_scale(1.0f), z_scale(1.0f),
        heading_offset(0.0f), sample_count(0), coverage_score(0.0f), checksum(0) {}

    uint32_t calculateChecksum() const {
        uint32_t sum = x_offset + y_offset + z_offset;
        sum += (uint32_t)(x_scale * 1000) + (uint32_t)(y_scale * 1000) + (uint32_t)(z_scale * 1000);
        sum += (uint32_t)(heading_offset * 100);
        sum += sample_count;
        return sum;
    }

    bool isValid() const {
        return checksum == calculateChecksum() && sample_count > 0;
    }
};

class CompassCalibrator {
public:
    CompassCalibrator(Compass& compass, Display& display);
    ~CompassCalibrator();

    // Main calibration process
    bool runCalibration();

    // Run heading calibration (comparing to known direction)
    bool calibrateHeading();

    // Load/Save calibration from/to NVS
    bool loadCalibration();
    bool saveCalibration();
    bool hasStoredCalibration();
    void clearStoredCalibration();

    // Apply loaded calibration to compass
    void applyCalibration();

    // Get calibration data
    const CalibrationData& getCalibrationData() const { return cal_data; }

    // Print calibration info
    void printCalibrationInfo();

private:
    Compass& compass;
    Display& display;
    CalibrationData cal_data;
    nvs_handle_t nvs_handle;

    // Calibration state
    struct Sample {
        int16_t x, y, z;
        uint32_t timestamp;
    };
    std::vector<Sample> samples;

    // Tracking min/max for each axis
    int16_t x_min, x_max;
    int16_t y_min, y_max;
    int16_t z_min, z_max;

    // Sphere coverage tracking (divide sphere into sectors)
    static const int SPHERE_SECTORS = 26;  // 26 directions to cover
    bool sphere_coverage[SPHERE_SECTORS];

    // Visual feedback
    void drawCalibrationScreen();
    void drawProgressBar(int16_t y, float progress, uint16_t color);
    void drawCompassRose(int16_t cx, int16_t cy, int16_t radius);
    void drawCoverageIndicator();
    void drawInstructions(const char* text);
    void drawLiveData(int16_t x, int16_t y, int16_t z);

    // Calibration calculations
    void updateMinMax(int16_t x, int16_t y, int16_t z);
    void calculateOffsets();
    void calculateScales();
    float calculateCoverageScore();
    int getSphereSection(int16_t x, int16_t y, int16_t z);

    // NVS storage
    bool initNVS();
    static constexpr const char* NVS_NAMESPACE = "compass_cal";
    static constexpr const char* NVS_KEY = "cal_data";
};

#endif // COMPASS_CALIBRATOR_HPP

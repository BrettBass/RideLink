#include "CompassCalibrator.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include <cmath>
#include <algorithm>
#include <cstring>

static const char* TAG = "COMPASS_CAL";

// Button for user input (BOOT button on most ESP32 boards)
#define BUTTON_GPIO GPIO_NUM_0

CompassCalibrator::CompassCalibrator(Compass& compass, Display& display)
    : compass(compass)
    , display(display)
    , nvs_handle(0)
    , x_min(32767), x_max(-32768)
    , y_min(32767), y_max(-32768)
    , z_min(32767), z_max(-32768)
{
    memset(sphere_coverage, 0, sizeof(sphere_coverage));
    initNVS();
}

CompassCalibrator::~CompassCalibrator() {
    if (nvs_handle) {
        nvs_close(nvs_handle);
    }
}

bool CompassCalibrator::initNVS() {
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Open NVS handle
    ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS handle: %s", esp_err_to_name(ret));
        return false;
    }

    return true;
}

bool CompassCalibrator::loadCalibration() {
    size_t length = sizeof(CalibrationData);
    esp_err_t ret = nvs_get_blob(nvs_handle, NVS_KEY, &cal_data, &length);

    if (ret == ESP_OK && cal_data.isValid()) {
        ESP_LOGI(TAG, "Loaded calibration from NVS:");
        ESP_LOGI(TAG, "  Offsets: X=%d, Y=%d, Z=%d",
                 cal_data.x_offset, cal_data.y_offset, cal_data.z_offset);
        ESP_LOGI(TAG, "  Scales: X=%.3f, Y=%.3f, Z=%.3f",
                 cal_data.x_scale, cal_data.y_scale, cal_data.z_scale);
        ESP_LOGI(TAG, "  Heading offset: %.1f°", cal_data.heading_offset);
        ESP_LOGI(TAG, "  Samples: %lu, Coverage: %.1f%%",
                 cal_data.sample_count, cal_data.coverage_score);
        return true;
    }

    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "No stored calibration found");
    } else if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error loading calibration: %s", esp_err_to_name(ret));
    } else if (!cal_data.isValid()) {
        ESP_LOGW(TAG, "Stored calibration data is invalid (checksum mismatch)");
    }

    return false;
}

bool CompassCalibrator::saveCalibration() {
    cal_data.checksum = cal_data.calculateChecksum();

    esp_err_t ret = nvs_set_blob(nvs_handle, NVS_KEY, &cal_data, sizeof(CalibrationData));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error saving calibration: %s", esp_err_to_name(ret));
        return false;
    }

    ret = nvs_commit(nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error committing calibration: %s", esp_err_to_name(ret));
        return false;
    }

    ESP_LOGI(TAG, "Calibration saved to NVS");
    return true;
}

bool CompassCalibrator::hasStoredCalibration() {
    size_t length = sizeof(CalibrationData);
    CalibrationData temp_data;
    esp_err_t ret = nvs_get_blob(nvs_handle, NVS_KEY, &temp_data, &length);
    return (ret == ESP_OK && temp_data.isValid());
}

void CompassCalibrator::clearStoredCalibration() {
    esp_err_t ret = nvs_erase_key(nvs_handle, NVS_KEY);
    if (ret == ESP_OK) {
        nvs_commit(nvs_handle);
        ESP_LOGI(TAG, "Cleared stored calibration");
    }
}

void CompassCalibrator::applyCalibration() {
    compass.setCalibrationOffsets(cal_data.x_offset, cal_data.y_offset, cal_data.z_offset);
    compass.setCalibrationScales(cal_data.x_scale, cal_data.y_scale, cal_data.z_scale);
    ESP_LOGI(TAG, "Applied calibration to compass");
}

bool CompassCalibrator::runCalibration() {
    ESP_LOGI(TAG, "==============================================");
    ESP_LOGI(TAG, "Starting Interactive Compass Calibration");
    ESP_LOGI(TAG, "==============================================");

    // Configure button
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << BUTTON_GPIO);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    // Reset calibration state
    samples.clear();
    samples.reserve(1000);
    memset(sphere_coverage, 0, sizeof(sphere_coverage));
    x_min = y_min = z_min = 32767;
    x_max = y_max = z_max = -32768;

    // Temporarily disable existing calibration
    compass.setCalibrationOffsets(0, 0, 0);
    compass.setCalibrationScales(1.0f, 1.0f, 1.0f);

    // Clear screen and show initial instructions
    display.fillScreen(Color::BLACK);
    drawInstructions("COMPASS CALIBRATION");

    // Draw instruction text
    const int16_t textY = 40;
    const int16_t lineHeight = 20;

    display.fillRect(10, textY, 220, lineHeight * 7, Color::BLACK);
    // In a real implementation, you'd have a text drawing function
    // For now, we'll use visual indicators

    // Draw visual instructions using shapes
    // "Rotate in all directions" indicator
    display.drawCircle(120, 80, 20, Color::CYAN);
    display.drawArrow(120, 80, 15, ArrowDirection::UP, Color::WHITE);
    display.drawArrow(120, 80, 15, ArrowDirection::RIGHT, Color::WHITE);
    display.drawArrow(120, 80, 15, ArrowDirection::DOWN, Color::WHITE);
    display.drawArrow(120, 80, 15, ArrowDirection::LEFT, Color::WHITE);

    ESP_LOGI(TAG, "Instructions:");
    ESP_LOGI(TAG, "1. Rotate device slowly in ALL directions");
    ESP_LOGI(TAG, "2. Make figure-8 patterns");
    ESP_LOGI(TAG, "3. Tilt forward/backward/left/right");
    ESP_LOGI(TAG, "4. Press BOOT button when done");
    ESP_LOGI(TAG, "5. Try to fill the coverage indicator!");

    vTaskDelay(pdMS_TO_TICKS(3000));

    // Main calibration loop
    uint32_t startTime = esp_log_timestamp();
    uint32_t lastDisplayUpdate = 0;
    bool calibrating = true;

    while (calibrating) {
        // Check button
        if (gpio_get_level(BUTTON_GPIO) == 0) {
            // Button pressed - check if we have enough samples
            if (samples.size() >= 100) {
                calibrating = false;
                ESP_LOGI(TAG, "Calibration stopped by user");
            } else {
                ESP_LOGW(TAG, "Need at least 100 samples (currently %d)", samples.size());
            }
        }

        // Read compass data
        int16_t x, y, z;
        if (compass.readRaw(x, y, z)) {
            // Store sample
            Sample s = {x, y, z, esp_log_timestamp()};
            samples.push_back(s);

            // Update min/max
            updateMinMax(x, y, z);

            // Update sphere coverage
            int section = getSphereSection(x, y, z);
            if (section >= 0 && section < SPHERE_SECTORS) {
                sphere_coverage[section] = true;
            }

            // Update display every 100ms
            uint32_t now = esp_log_timestamp();
            if (now - lastDisplayUpdate >= 100) {
                lastDisplayUpdate = now;
                drawCalibrationScreen();
                drawLiveData(x, y, z);
                drawCoverageIndicator();

                // Show progress
                float progress = std::min(1.0f, samples.size() / 500.0f);
                drawProgressBar(200, progress, Color::GREEN);

                // Log progress periodically
                static int log_counter = 0;
                if (++log_counter % 10 == 0) {
                    float coverage = calculateCoverageScore();
                    ESP_LOGI(TAG, "Samples: %d, Coverage: %.1f%%, Ranges: X[%d,%d] Y[%d,%d] Z[%d,%d]",
                             samples.size(), coverage,
                             x_min, x_max, y_min, y_max, z_min, z_max);
                }
            }
        }

        // Auto-stop after 60 seconds or 1000 samples
        if (samples.size() >= 1000 || (esp_log_timestamp() - startTime) > 60000) {
            calibrating = false;
            ESP_LOGI(TAG, "Calibration auto-stopped");
        }

        vTaskDelay(pdMS_TO_TICKS(20)); // 50Hz sampling
    }

    // Calculate calibration parameters
    calculateOffsets();
    calculateScales();
    cal_data.sample_count = samples.size();
    cal_data.coverage_score = calculateCoverageScore();

    // Show results
    display.fillScreen(Color::BLACK);
    drawInstructions("CALIBRATION COMPLETE");

    // Draw checkmark
    display.fillCircle(120, 120, 30, Color::GREEN);
    display.drawLine(100, 120, 110, 130, Color::WHITE);
    display.drawLine(110, 130, 130, 110, Color::WHITE);
    display.drawLine(100, 119, 110, 129, Color::WHITE);
    display.drawLine(110, 129, 130, 109, Color::WHITE);
    display.drawLine(100, 118, 110, 128, Color::WHITE);
    display.drawLine(110, 128, 130, 108, Color::WHITE);

    ESP_LOGI(TAG, "==============================================");
    ESP_LOGI(TAG, "Calibration Results:");
    ESP_LOGI(TAG, "  Samples collected: %lu", cal_data.sample_count);
    ESP_LOGI(TAG, "  Coverage score: %.1f%%", cal_data.coverage_score);
    ESP_LOGI(TAG, "  Offsets: X=%d, Y=%d, Z=%d",
             cal_data.x_offset, cal_data.y_offset, cal_data.z_offset);
    ESP_LOGI(TAG, "  Scales: X=%.3f, Y=%.3f, Z=%.3f",
             cal_data.x_scale, cal_data.y_scale, cal_data.z_scale);
    ESP_LOGI(TAG, "==============================================");

    vTaskDelay(pdMS_TO_TICKS(3000));

    // Save calibration
    if (saveCalibration()) {
        ESP_LOGI(TAG, "Calibration saved successfully!");
        display.fillScreen(Color::BLACK);
        drawInstructions("SAVED TO MEMORY");
        display.fillCircle(120, 120, 20, Color::BLUE);
    } else {
        ESP_LOGE(TAG, "Failed to save calibration!");
        display.fillScreen(Color::BLACK);
        drawInstructions("SAVE FAILED");
        display.fillCircle(120, 120, 20, Color::RED);
    }

    vTaskDelay(pdMS_TO_TICKS(2000));

    // Apply the calibration
    applyCalibration();

    return true;
}

bool CompassCalibrator::calibrateHeading() {
    ESP_LOGI(TAG, "==============================================");
    ESP_LOGI(TAG, "Starting Heading Calibration");
    ESP_LOGI(TAG, "Point device to NORTH and press BOOT button");
    ESP_LOGI(TAG, "==============================================");

    // Configure button
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << BUTTON_GPIO);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    display.fillScreen(Color::BLACK);
    drawInstructions("POINT TO NORTH");
    drawCompassRose(120, 120, 60);

    // Draw north indicator
    display.fillTriangle(120, 50, 110, 70, 130, 70, Color::RED);

    // Wait for button press
    while (gpio_get_level(BUTTON_GPIO) == 1) {
        // Show current heading
        float heading = compass.getHeading();

        // Draw current heading arrow
        int16_t arrowAngle = (int16_t)(360 - heading);
        display.updateArrow(arrowAngle, Color::YELLOW);

        // Log heading periodically
        static uint32_t lastLog = 0;
        uint32_t now = esp_log_timestamp();
        if (now - lastLog >= 1000) {
            lastLog = now;
            ESP_LOGI(TAG, "Current heading: %.1f° (should be 0° when pointing north)", heading);
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }

    // Button pressed - calculate offset
    float currentHeading = compass.getHeading();
    cal_data.heading_offset = -currentHeading;  // Offset to make current heading 0

    if (cal_data.heading_offset < -180) cal_data.heading_offset += 360;
    if (cal_data.heading_offset > 180) cal_data.heading_offset -= 360;

    ESP_LOGI(TAG, "Heading offset calculated: %.1f°", cal_data.heading_offset);

    // Save updated calibration
    saveCalibration();

    display.fillScreen(Color::BLACK);
    drawInstructions("HEADING CALIBRATED");
    display.fillCircle(120, 120, 30, Color::GREEN);

    vTaskDelay(pdMS_TO_TICKS(2000));

    return true;
}

void CompassCalibrator::drawCalibrationScreen() {
    // Clear screen
    display.fillScreen(Color::BLACK);

    // Title
    drawInstructions("CALIBRATING...");

    // Draw rotation indicator animation
    static int animFrame = 0;
    animFrame = (animFrame + 1) % 8;

    int16_t centerX = 120;
    int16_t centerY = 100;
    int16_t radius = 30;

    // Draw rotating arrow
    float angle = animFrame * 45.0f;
    int16_t endX = centerX + (int16_t)(radius * cos(angle * M_PI / 180));
    int16_t endY = centerY + (int16_t)(radius * sin(angle * M_PI / 180));
    display.drawLine(centerX, centerY, endX, endY, Color::CYAN);
    display.fillCircle(endX, endY, 3, Color::CYAN);
}

void CompassCalibrator::drawProgressBar(int16_t y, float progress, uint16_t color) {
    int16_t barWidth = 200;
    int16_t barHeight = 20;
    int16_t x = (240 - barWidth) / 2;

    // Draw border
    display.drawRect(x, y, barWidth, barHeight, Color::WHITE);

    // Fill progress
    int16_t fillWidth = (int16_t)(barWidth * progress);
    display.fillRect(x, y, fillWidth, barHeight, color);
}

void CompassCalibrator::drawCompassRose(int16_t cx, int16_t cy, int16_t radius) {
    // Draw circle
    display.drawCircle(cx, cy, radius, Color::WHITE);

    // Draw cardinal directions
    // North
    display.drawLine(cx, cy - radius, cx, cy - radius + 10, Color::RED);
    display.fillTriangle(cx, cy - radius - 5, cx - 3, cy - radius, cx + 3, cy - radius, Color::RED);

    // South
    display.drawLine(cx, cy + radius, cx, cy + radius - 10, Color::WHITE);

    // East
    display.drawLine(cx + radius, cy, cx + radius - 10, cy, Color::WHITE);

    // West
    display.drawLine(cx - radius, cy, cx - radius + 10, cy, Color::WHITE);
}

void CompassCalibrator::drawCoverageIndicator() {
    // Draw coverage dots in a circle pattern
    int16_t centerX = 60;
    int16_t centerY = 180;
    int16_t radius = 30;

    int covered = 0;
    for (int i = 0; i < SPHERE_SECTORS; i++) {
        float angle = (i * 360.0f / SPHERE_SECTORS) * M_PI / 180;
        int16_t x = centerX + (int16_t)(radius * cos(angle));
        int16_t y = centerY + (int16_t)(radius * sin(angle));

        uint16_t color = sphere_coverage[i] ? Color::GREEN : Color::GRAY;
        display.fillCircle(x, y, 2, color);

        if (sphere_coverage[i]) covered++;
    }

    // Draw coverage percentage in center
    float coverage = (covered * 100.0f) / SPHERE_SECTORS;

    // Visual indicator of coverage quality
    uint16_t centerColor = Color::RED;
    if (coverage > 30) centerColor = Color::YELLOW;
    if (coverage > 60) centerColor = Color::GREEN;

    display.fillCircle(centerX, centerY, 10, centerColor);
}

void CompassCalibrator::drawInstructions(const char* text) {
    // Draw title bar
    display.fillRect(0, 0, 240, 30, Color::BLUE);

    // Since we don't have text rendering, draw a simple indicator
    // In a real implementation, you'd render the text here
    int textLen = strlen(text);
    int16_t barWidth = std::min(200, textLen * 8);
    int16_t x = (240 - barWidth) / 2;
    display.fillRect(x, 10, barWidth, 10, Color::WHITE);
}

void CompassCalibrator::drawLiveData(int16_t x, int16_t y, int16_t z) {
    // Draw live magnetometer values as bars
    int16_t barX = 170;
    int16_t barY = 60;
    int16_t barHeight = 100;
    int16_t barWidth = 15;
    int16_t spacing = 20;

    // Normalize values to bar height
    float scale = barHeight / 4000.0f;  // Assuming typical range of ±2000

    // X bar (red)
    int16_t xHeight = abs((int16_t)(x * scale));
    xHeight = std::min(xHeight, barHeight);
    display.fillRect(barX, barY + barHeight - xHeight, barWidth, xHeight, Color::RED);

    // Y bar (green)
    int16_t yHeight = abs((int16_t)(y * scale));
    yHeight = std::min(yHeight, barHeight);
    display.fillRect(barX + spacing, barY + barHeight - yHeight, barWidth, yHeight, Color::GREEN);

    // Z bar (blue)
    int16_t zHeight = abs((int16_t)(z * scale));
    zHeight = std::min(zHeight, barHeight);
    display.fillRect(barX + 2*spacing, barY + barHeight - zHeight, barWidth, zHeight, Color::BLUE);
}

void CompassCalibrator::updateMinMax(int16_t x, int16_t y, int16_t z) {
    if (x < x_min) x_min = x;
    if (x > x_max) x_max = x;
    if (y < y_min) y_min = y;
    if (y > y_max) y_max = y;
    if (z < z_min) z_min = z;
    if (z > z_max) z_max = z;
}

void CompassCalibrator::calculateOffsets() {
    // Hard iron correction - find the center of the ellipsoid
    cal_data.x_offset = (x_min + x_max) / 2;
    cal_data.y_offset = (y_min + y_max) / 2;
    cal_data.z_offset = (z_min + z_max) / 2;
}

void CompassCalibrator::calculateScales() {
    // Soft iron correction - normalize the ellipsoid to a sphere
    int16_t x_range = x_max - x_min;
    int16_t y_range = y_max - y_min;
    int16_t z_range = z_max - z_min;

    // Find average range
    int16_t avg_range = (x_range + y_range + z_range) / 3;

    // Calculate scale factors
    if (x_range > 0) cal_data.x_scale = (float)avg_range / x_range;
    else cal_data.x_scale = 1.0f;

    if (y_range > 0) cal_data.y_scale = (float)avg_range / y_range;
    else cal_data.y_scale = 1.0f;

    if (z_range > 0) cal_data.z_scale = (float)avg_range / z_range;
    else cal_data.z_scale = 1.0f;
}

float CompassCalibrator::calculateCoverageScore() {
    int covered = 0;
    for (int i = 0; i < SPHERE_SECTORS; i++) {
        if (sphere_coverage[i]) covered++;
    }
    return (covered * 100.0f) / SPHERE_SECTORS;
}

int CompassCalibrator::getSphereSection(int16_t x, int16_t y, int16_t z) {
    // Map 3D vector to one of 26 sphere sections (like a soccer ball)
    // This is a simplified mapping - you could make it more sophisticated

    // Normalize vector
    float mag = sqrtf(x*x + y*y + z*z);
    if (mag < 0.1f) return -1;

    float nx = x / mag;
    float ny = y / mag;
    float nz = z / mag;

    // Convert to spherical coordinates
    float theta = atan2f(ny, nx) * 180.0f / M_PI;  // Azimuth
    float phi = asinf(nz) * 180.0f / M_PI;         // Elevation

    // Map to section (0-25)
    int azimuthSection = (int)((theta + 180.0f) / 45.0f);  // 8 sections
    int elevationSection = (int)((phi + 90.0f) / 60.0f);    // 3 sections

    if (azimuthSection < 0) azimuthSection = 0;
    if (azimuthSection > 7) azimuthSection = 7;
    if (elevationSection < 0) elevationSection = 0;
    if (elevationSection > 2) elevationSection = 2;

    // Special case for poles (top and bottom)
    if (phi > 60) return 24;  // North pole
    if (phi < -60) return 25; // South pole

    return elevationSection * 8 + azimuthSection;
}

void CompassCalibrator::printCalibrationInfo() {
    ESP_LOGI(TAG, "==============================================");
    ESP_LOGI(TAG, "Current Calibration Info:");

    if (cal_data.isValid()) {
        ESP_LOGI(TAG, "  Status: VALID");
        ESP_LOGI(TAG, "  Hard Iron Offsets:");
        ESP_LOGI(TAG, "    X: %d", cal_data.x_offset);
        ESP_LOGI(TAG, "    Y: %d", cal_data.y_offset);
        ESP_LOGI(TAG, "    Z: %d", cal_data.z_offset);
        ESP_LOGI(TAG, "  Soft Iron Scales:");
        ESP_LOGI(TAG, "    X: %.3f", cal_data.x_scale);
        ESP_LOGI(TAG, "    Y: %.3f", cal_data.y_scale);
        ESP_LOGI(TAG, "    Z: %.3f", cal_data.z_scale);
        ESP_LOGI(TAG, "  Heading Offset: %.1f°", cal_data.heading_offset);
        ESP_LOGI(TAG, "  Sample Count: %lu", cal_data.sample_count);
        ESP_LOGI(TAG, "  Coverage Score: %.1f%%", cal_data.coverage_score);
    } else {
        ESP_LOGI(TAG, "  Status: INVALID or NOT CALIBRATED");
    }

    ESP_LOGI(TAG, "==============================================");
}

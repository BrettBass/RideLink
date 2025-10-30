#include <cstdio>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "GPS.hpp"
#include "Compass.hpp"
#include "Display.hpp"
#include "CompassCalibrator.hpp"

static const char* TAG = "MAIN";

// Button GPIO (BOOT button on most ESP32 boards)
#define BUTTON_GPIO GPIO_NUM_0
#define BUTTON_HOLD_TIME_MS 3000  // Hold for 3 seconds to enter calibration

// GPS, Compass, Display, and Calibrator instances
GPS gps;
Compass compass;
Display display;
CompassCalibrator* calibrator = nullptr;

// Task handles
TaskHandle_t gps_task_handle = NULL;
TaskHandle_t compass_task_handle = NULL;

// Calibration state
bool calibration_mode = false;

// Function to check for calibration button hold
bool checkCalibrationButton() {
    if (gpio_get_level(BUTTON_GPIO) == 0) {  // Button pressed (active low)
        uint32_t pressStart = esp_log_timestamp();

        // Wait for button to be held or released
        while (gpio_get_level(BUTTON_GPIO) == 0) {
            uint32_t holdTime = esp_log_timestamp() - pressStart;

            // Show visual feedback for button hold
            if (holdTime > 500) {
                float progress = (float)holdTime / BUTTON_HOLD_TIME_MS;
                progress = std::min(1.0f, progress);

                // Draw progress circle
                int16_t radius = (int16_t)(30 * progress);
                display.fillCircle(120, 120, radius, Color::YELLOW);
            }

            if (holdTime >= BUTTON_HOLD_TIME_MS) {
                return true;  // Long press detected
            }

            vTaskDelay(pdMS_TO_TICKS(50));
        }

        // Button was released before hold time
        display.fillScreen(Color::BLACK);
    }

    return false;
}

// Task to continuously read GPS data
void gps_task(void* param) {
    uint32_t last_display = 0;
    uint32_t no_fix_count = 0;

    while (!calibration_mode) {
        // Read GPS data continuously
        gps.read();

        // Display status every second
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (now - last_display >= 1000) {
            last_display = now;

            if (gps.isFixed()) {
                // We have a fix - display position
                gps.printStatus();
                no_fix_count = 0;
            } else {
                // No fix yet
                no_fix_count++;

                if (no_fix_count % 5 == 1) {
                    ESP_LOGI(TAG, "Waiting for GPS fix... (Sats: %d)",
                             gps.getSatelliteCount());
                }

                if (no_fix_count == 30) {
                    ESP_LOGW(TAG, "No fix after 30 seconds. Are you outdoors?");
                }
            }
        }

        // Small delay to prevent watchdog
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    vTaskDelete(NULL);
}

// Task to read compass and display arrow
void compass_task(void* param) {
    uint32_t last_display = 0;
    uint32_t last_update = 0;
    uint32_t last_button_check = 0;

    // Get calibration data if available
    float heading_offset = 0;
    if (calibrator && calibrator->getCalibrationData().isValid()) {
        heading_offset = calibrator->getCalibrationData().heading_offset;
    }

    while (!calibration_mode) {
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;

        // Check for calibration button every 500ms
        if (now - last_button_check >= 500) {
            last_button_check = now;
            if (checkCalibrationButton()) {
                calibration_mode = true;
                break;
            }
        }

        // Read raw compass values for debugging
        int16_t x, y, z;
        bool read_success = compass.readRaw(x, y, z);

        // Update display at 10Hz for smooth arrow movement
        if (now - last_update >= 100) {
            last_update = now;

            float heading = compass.getHeading();

            // Apply heading offset from calibration
            heading += heading_offset;

            // Normalize to 0-359
            while (heading < 0) heading += 360;
            while (heading >= 360) heading -= 360;

            // Calculate arrow angle (arrow points north)
            // If heading is 90° (facing east), arrow should point 270° (left) to show north
            int16_t arrow_angle = (int16_t)(360 - heading);

            // Update arrow on display
            display.updateArrow(arrow_angle, Color::GREEN);

            // Draw additional UI elements
            // Show GPS status indicator
            if (gps.isFixed()) {
                display.fillCircle(220, 20, 5, Color::GREEN);
            } else {
                display.drawCircle(220, 20, 5, Color::RED);
            }

            // Show calibration status indicator
            if (calibrator && calibrator->getCalibrationData().isValid()) {
                float coverage = calibrator->getCalibrationData().coverage_score;
                uint16_t cal_color = Color::RED;
                if (coverage > 30) cal_color = Color::YELLOW;
                if (coverage > 60) cal_color = Color::GREEN;
                display.fillCircle(20, 20, 5, cal_color);
            } else {
                display.drawCircle(20, 20, 5, Color::GRAY);
            }

            // Log every second with values
            if (now - last_display >= 1000) {
                last_display = now;
                if (read_success) {
                    ESP_LOGI(TAG, "Heading: %.1f° | Arrow: %d° | GPS: %s | Cal: %s",
                             heading, arrow_angle,
                             gps.isFixed() ? "FIX" : "NO FIX",
                             (calibrator && calibrator->getCalibrationData().isValid()) ? "YES" : "NO");
                } else {
                    ESP_LOGE(TAG, "Failed to read compass!");
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50)); // 20Hz compass reading
    }

    vTaskDelete(NULL);
}

// Function to run calibration
void runCalibrationMode() {
    ESP_LOGI(TAG, "=================================");
    ESP_LOGI(TAG, "    CALIBRATION MODE");
    ESP_LOGI(TAG, "=================================");

    // Show menu
    display.fillScreen(Color::BLACK);

    // Draw menu options
    display.fillRect(20, 50, 200, 40, Color::BLUE);   // Option 1
    display.fillRect(20, 100, 200, 40, Color::GREEN);  // Option 2
    display.fillRect(20, 150, 200, 40, Color::ORANGE); // Option 3

    ESP_LOGI(TAG, "Calibration Options:");
    ESP_LOGI(TAG, "1. Quick press = Magnetometer calibration");
    ESP_LOGI(TAG, "2. Hold 1 sec = Heading calibration");
    ESP_LOGI(TAG, "3. Hold 5 sec = Clear calibration");

    // Wait for button input
    uint32_t wait_start = esp_log_timestamp();
    while ((esp_log_timestamp() - wait_start) < 10000) {  // 10 second timeout
        if (gpio_get_level(BUTTON_GPIO) == 0) {
            uint32_t press_start = esp_log_timestamp();

            // Wait for release or timeout
            while (gpio_get_level(BUTTON_GPIO) == 0) {
                vTaskDelay(pdMS_TO_TICKS(50));
                if ((esp_log_timestamp() - press_start) > 5000) break;
            }

            uint32_t press_duration = esp_log_timestamp() - press_start;

            if (press_duration > 5000) {
                // Clear calibration
                ESP_LOGI(TAG, "Clearing calibration...");
                calibrator->clearStoredCalibration();
                display.fillScreen(Color::RED);
                vTaskDelay(pdMS_TO_TICKS(1000));
            } else if (press_duration > 1000) {
                // Heading calibration
                ESP_LOGI(TAG, "Starting heading calibration...");
                calibrator->calibrateHeading();
            } else {
                // Magnetometer calibration
                ESP_LOGI(TAG, "Starting magnetometer calibration...");
                calibrator->runCalibration();
            }

            break;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // Exit calibration mode
    calibration_mode = false;
    display.fillScreen(Color::BLACK);
}

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "=================================");
    ESP_LOGI(TAG, "    RideLink - GPS + Compass");
    ESP_LOGI(TAG, "    With Smart Calibration");
    ESP_LOGI(TAG, "=================================");
    ESP_LOGI(TAG, "");

    // Configure button GPIO
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << BUTTON_GPIO);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    // Initialize Display first
    ESP_LOGI(TAG, "Initializing Display...");
    if (!display.init()) {
        ESP_LOGE(TAG, "Failed to initialize Display!");
        return;
    }
    ESP_LOGI(TAG, "✓ Display initialized successfully");
    display.fillScreen(Color::BLACK);
    ESP_LOGI(TAG, "");

    // Initialize GPS
    ESP_LOGI(TAG, "Initializing GPS...");
    if (!gps.begin()) {
        ESP_LOGE(TAG, "Failed to initialize GPS!");
        ESP_LOGE(TAG, "Check wiring: GPS TX -> GPIO16 (RX2), GPS RX -> GPIO17 (TX2)");
        return;
    }
    ESP_LOGI(TAG, "✓ GPS initialized successfully");
    ESP_LOGI(TAG, "");

    // Initialize Compass
    ESP_LOGI(TAG, "Initializing Compass...");
    if (!compass.init()) {
        ESP_LOGE(TAG, "Failed to initialize Compass!");
        ESP_LOGE(TAG, "Check wiring: SDA -> GPIO21, SCL -> GPIO22");
        return;
    }
    ESP_LOGI(TAG, "✓ Compass initialized successfully");
    ESP_LOGI(TAG, "");

    // Initialize Calibrator
    ESP_LOGI(TAG, "Initializing Calibration System...");
    calibrator = new CompassCalibrator(compass, display);

    // Check for stored calibration
    if (calibrator->hasStoredCalibration()) {
        ESP_LOGI(TAG, "Found stored calibration data!");
        calibrator->loadCalibration();
        calibrator->printCalibrationInfo();
        calibrator->applyCalibration();

        // Show calibration loaded indicator
        display.fillScreen(Color::BLACK);
        display.fillCircle(120, 120, 30, Color::GREEN);
        display.drawCircle(120, 120, 35, Color::GREEN);
        vTaskDelay(pdMS_TO_TICKS(1000));
        display.fillScreen(Color::BLACK);
    } else {
        ESP_LOGW(TAG, "No stored calibration found");
        ESP_LOGI(TAG, "Hold BOOT button for 3 seconds to calibrate");

        // Show no calibration indicator
        display.fillScreen(Color::BLACK);
        display.drawCircle(120, 120, 30, Color::YELLOW);
        display.drawLine(100, 100, 140, 140, Color::YELLOW);
        display.drawLine(100, 140, 140, 100, Color::YELLOW);
        vTaskDelay(pdMS_TO_TICKS(2000));
        display.fillScreen(Color::BLACK);
    }
    ESP_LOGI(TAG, "");

    // Set magnetic declination for your location
    // Find yours at: https://www.magnetic-declination.com/
    // Hidden Hills, CA is approximately +11.5°
    compass.setDeclination(11.5);
    ESP_LOGI(TAG, "Set magnetic declination to 11.5° (Hidden Hills, CA)");
    ESP_LOGI(TAG, "");

    // Instructions
    ESP_LOGI(TAG, "==============================================");
    ESP_LOGI(TAG, "INSTRUCTIONS:");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "NORMAL MODE:");
    ESP_LOGI(TAG, "  - Green arrow points to magnetic north");
    ESP_LOGI(TAG, "  - Top-right dot: GPS status (green=fix, red=no fix)");
    ESP_LOGI(TAG, "  - Top-left dot: Calibration quality");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "CALIBRATION:");
    ESP_LOGI(TAG, "  - Hold BOOT button for 3 seconds");
    ESP_LOGI(TAG, "  - Follow on-screen instructions");
    ESP_LOGI(TAG, "  - Calibration is saved permanently");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "CALIBRATION MENU:");
    ESP_LOGI(TAG, "  - Quick press: Magnetometer calibration");
    ESP_LOGI(TAG, "  - Hold 1 sec: Heading calibration");
    ESP_LOGI(TAG, "  - Hold 5 sec: Clear calibration");
    ESP_LOGI(TAG, "==============================================");
    ESP_LOGI(TAG, "");

    // Main loop
    while (true) {
        if (!calibration_mode) {
            // Create/restart tasks if needed
            if (gps_task_handle == NULL) {
                xTaskCreate(gps_task, "gps_task", 4096, NULL, 5, &gps_task_handle);
            }
            if (compass_task_handle == NULL) {
                xTaskCreate(compass_task, "compass_task", 4096, NULL, 5, &compass_task_handle);
            }

            vTaskDelay(pdMS_TO_TICKS(1000));
        } else {
            // Stop tasks for calibration
            if (gps_task_handle != NULL) {
                vTaskDelete(gps_task_handle);
                gps_task_handle = NULL;
            }
            if (compass_task_handle != NULL) {
                vTaskDelete(compass_task_handle);
                compass_task_handle = NULL;
            }

            // Run calibration
            runCalibrationMode();

            // Re-apply calibration after calibration mode
            if (calibrator->hasStoredCalibration()) {
                calibrator->loadCalibration();
                calibrator->applyCalibration();
            }
        }
    }

    // Cleanup (never reached)
    delete calibrator;
}

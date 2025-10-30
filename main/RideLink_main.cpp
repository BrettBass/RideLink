#include <cstdio>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "GPS.hpp"
#include "Compass.hpp"

static const char* TAG = "MAIN";

// GPS and Compass instances
GPS gps;
Compass compass;

// Calibration state
bool calibrating = false;
uint32_t calibration_start_time = 0;
const uint32_t CALIBRATION_DURATION_MS = 15000; // 15 seconds

// GPS-based compass heading calibration
bool gps_heading_calibration = false;
double last_gps_lat = 0, last_gps_lon = 0;
float compass_offset = 0.0f;  // Degrees to add to compass reading

// Task to continuously read GPS data
void gps_task(void* param) {
    uint32_t last_display = 0;
    uint32_t no_fix_count = 0;
    bool first_fix = true;

    while (true) {
        // Read GPS data continuously
        gps.read();

        // Display status every second
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (now - last_display >= 1000) {
            last_display = now;

            if (gps.isFixed()) {
                // We have a fix - display position
                gps.printStatus();

                // Get packet that would be sent via LoRa
                GPSPacket packet = gps.getPacket();
                ESP_LOGI(TAG, "Packet ready: %.6f, %.6f [%d bytes]",
                         packet.lat, packet.lon, sizeof(GPSPacket));

                // GPS heading calibration mode
                if (gps_heading_calibration) {
                    if (first_fix) {
                        // Store first position
                        last_gps_lat = gps.getLatitude();
                        last_gps_lon = gps.getLongitude();
                        first_fix = false;
                        ESP_LOGI(TAG, "");
                        ESP_LOGI(TAG, "GPS HEADING CALIBRATION:");
                        ESP_LOGI(TAG, "Walk forward in a straight line (at least 10 meters)");
                        ESP_LOGI(TAG, "Keep the device orientation FIXED - don't rotate it!");
                        ESP_LOGI(TAG, "");
                    } else {
                        // Calculate distance moved
                        double current_lat = gps.getLatitude();
                        double current_lon = gps.getLongitude();
                        uint32_t distance = GPS::calculateDistance(last_gps_lat, last_gps_lon,
                                                                   current_lat, current_lon);

                        if (distance > 10) {  // Moved at least 10 meters
                            // Calculate GPS heading
                            uint16_t gps_heading = GPS::calculateHeading(last_gps_lat, last_gps_lon,
                                                                         current_lat, current_lon);

                            // Get current compass heading
                            float compass_heading = compass.getHeading();

                            // Calculate offset needed
                            compass_offset = gps_heading - compass_heading;

                            // Normalize offset to -180 to 180
                            while (compass_offset > 180) compass_offset -= 360;
                            while (compass_offset < -180) compass_offset += 360;

                            ESP_LOGI(TAG, "");
                            ESP_LOGI(TAG, "===========================================");
                            ESP_LOGI(TAG, "GPS HEADING CALIBRATION COMPLETE!");
                            ESP_LOGI(TAG, "GPS heading:     %.1f°", (float)gps_heading);
                            ESP_LOGI(TAG, "Compass heading: %.1f°", compass_heading);
                            ESP_LOGI(TAG, "Offset needed:   %.1f°", compass_offset);
                            ESP_LOGI(TAG, "===========================================");
                            ESP_LOGI(TAG, "");

                            gps_heading_calibration = false;
                        } else {
                            ESP_LOGI(TAG, "Keep walking... (moved %d meters, need 10+)", distance);
                        }
                    }
                }

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
}

// Task to read compass and handle calibration
void compass_task(void* param) {
    uint32_t last_display = 0;

    while (true) {
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;

        // Check if we're in calibration mode
        if (calibrating) {
            // Read compass during calibration (updateCalibration called inside readRaw)
            int16_t x, y, z;
            compass.readRaw(x, y, z);

            // Check if calibration time is up
            if (now - calibration_start_time >= CALIBRATION_DURATION_MS) {
                compass.finishCalibration();
                calibrating = false;
                ESP_LOGI(TAG, "Magnetic calibration complete!");
                ESP_LOGI(TAG, "");
                ESP_LOGI(TAG, "Next: GPS heading calibration (when GPS fix acquired)");
            }
        } else {
            // Normal operation - display heading every second
            if (now - last_display >= 1000) {
                last_display = now;

                float heading = compass.getHeading();

                // Apply GPS-derived offset
                float corrected_heading = heading + compass_offset;

                // Normalize to 0-359
                while (corrected_heading < 0) corrected_heading += 360;
                while (corrected_heading >= 360) corrected_heading -= 360;

                ESP_LOGI(TAG, "Compass: %.1f° (raw) -> %.1f° (corrected)",
                         heading, corrected_heading);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50)); // 20Hz compass reading
    }
}

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "=================================");
    ESP_LOGI(TAG, "    RideLink - GPS + Compass");
    ESP_LOGI(TAG, "=================================");
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

    // Set magnetic declination for your location
    // Find yours at: https://www.magnetic-declination.com/
    // Hidden Hills, CA is approximately +11.5°
    compass.setDeclination(11.5);

    // Start calibration
    ESP_LOGI(TAG, "Starting 15-second magnetic field calibration...");
    ESP_LOGI(TAG, "ROTATE the device in ALL directions during calibration!");
    compass.startCalibration();
    calibrating = true;
    calibration_start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

    // Create tasks
    xTaskCreate(gps_task, "gps_task", 4096, NULL, 5, NULL);
    xTaskCreate(compass_task, "compass_task", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "GPS NOTES:");
    ESP_LOGI(TAG, "- GPS needs clear sky view (go outdoors!)");
    ESP_LOGI(TAG, "- First fix can take 30-60 seconds");
    ESP_LOGI(TAG, "- Module must see 4+ satellites for position");
    ESP_LOGI(TAG, "");

    // Wait for magnetic calibration to complete
    vTaskDelay(pdMS_TO_TICKS(CALIBRATION_DURATION_MS + 1000));

    // Enable GPS heading calibration
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Enabling GPS heading calibration...");
    ESP_LOGI(TAG, "Once GPS fix acquired, walk straight for 10+ meters");
    gps_heading_calibration = true;

    // Main loop
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

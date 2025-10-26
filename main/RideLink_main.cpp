#include <cstdio>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "GPS.hpp"

static const char *TAG = "GPS_TEST";

GPS gps;

void gps_test_task(void *pvParameters)
{
    ESP_LOGI(TAG, "GPS Test Task Started");

    // Initialize GPS (RX=GPIO16, TX=GPIO17 are ESP32 defaults for UART2)
    if (!gps.init(16, 17, 9600)) {
        ESP_LOGE(TAG, "Failed to initialize GPS!");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "GPS initialized. Waiting for satellite fix...");
    ESP_LOGI(TAG, "This can take 30-60 seconds outdoors with clear sky view.");
    ESP_LOGI(TAG, "NOTE: GPS won't work indoors!");

    bool first_fix = false;
    int no_fix_count = 0;

    while (true) {
        // Update GPS (reads and parses NMEA data)
        bool new_data = gps.update();

        // Check if we have a valid fix
        if (gps.isValid()) {
            if (!first_fix) {
                ESP_LOGI(TAG, "");
                ESP_LOGI(TAG, "========================================");
                ESP_LOGI(TAG, "GPS FIX ACQUIRED!");
                ESP_LOGI(TAG, "========================================");
                first_fix = true;
            }

            // Display info every time we get new data
            if (new_data) {
                gps.displayInfo();

                // Get location
                Location loc = gps.getLocation();

                // Example: Calculate distance and bearing to a target location
                // (Replace these with your actual target coordinates)
                double target_lat = 37.7749;  // San Francisco
                double target_lon = -122.4194;

                int distance = gps.distanceTo(target_lat, target_lon);
                int bearing = gps.bearingTo(target_lat, target_lon);

                ESP_LOGI(TAG, "Distance to target: %d meters, Bearing: %d°",
                        distance, bearing);
            }

            no_fix_count = 0;
        } else {
            // No fix yet
            no_fix_count++;

            // Show progress every 5 seconds
            if (no_fix_count % 50 == 0) {
                ESP_LOGI(TAG, "Still waiting for GPS fix... (%d satellites visible)",
                        gps.getSatellites());
                ESP_LOGI(TAG, "Make sure you're outdoors with clear view of sky!");
            }
        }

        // Update at 10Hz
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

extern "C" void app_main(void)
{
    printf("GPS Test Program Starting...\n");

    // Create GPS task
    xTaskCreate(gps_test_task, "GPS_Test", 8192, NULL, 5, NULL);
}

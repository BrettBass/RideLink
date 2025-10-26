#include <cstdio>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "GPS.hpp"

static const char* TAG = "MAIN";

// GPS instance
GPS gps;

// Task to continuously read GPS data
void gps_task(void* param) {
    uint32_t last_display = 0;
    uint32_t no_fix_count = 0;

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

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "=================================");
    ESP_LOGI(TAG, "    GPS Module Test");
    ESP_LOGI(TAG, "=================================");
    ESP_LOGI(TAG, "Using UART2 on TX2/RX2 pins");
    ESP_LOGI(TAG, "");

    // Initialize GPS
    if (!gps.begin()) {
        ESP_LOGE(TAG, "Failed to initialize GPS!");
        ESP_LOGE(TAG, "Check wiring: GPS TX -> GPIO16 (RX2), GPS RX -> GPIO17 (TX2)");
        return;
    }

    ESP_LOGI(TAG, "GPS initialized successfully");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "IMPORTANT:");
    ESP_LOGI(TAG, "- GPS needs clear sky view (go outdoors!)");
    ESP_LOGI(TAG, "- First fix can take 30-60 seconds");
    ESP_LOGI(TAG, "- Module must see 4+ satellites for position");
    ESP_LOGI(TAG, "");

    // Create GPS reading task
    xTaskCreate(gps_task, "gps_task", 4096, NULL, 5, NULL);

    // Main loop - could handle other tasks here
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


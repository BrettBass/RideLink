#include <cstdio>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "GPS.hpp"

static const char *TAG = "GPS_DIAG";

// Use GPIO25/26 - safe pins that won't interfere with flashing
#define GPS_RX_PIN 25
#define GPS_TX_PIN 26

GPS gps;

void gps_diagnostic_task(void *pvParameters)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "GPS DIAGNOSTIC TEST");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Pin Configuration:");
    ESP_LOGI(TAG, "  GPS TX  -> ESP32 GPIO%d (RX)", GPS_RX_PIN);
    ESP_LOGI(TAG, "  GPS RX  -> ESP32 GPIO%d (TX)", GPS_TX_PIN);
    ESP_LOGI(TAG, "  GPS VCC -> ESP32 5V (or 3.3V)");
    ESP_LOGI(TAG, "  GPS GND -> ESP32 GND");
    ESP_LOGI(TAG, "");

    // Initialize GPS
    if (!gps.init(GPS_RX_PIN, GPS_TX_PIN, 9600)) {
        ESP_LOGE(TAG, "Failed to initialize GPS UART!");
        ESP_LOGE(TAG, "Check wiring and try again.");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "✓ GPS UART initialized");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Listening for GPS data...");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "What to expect:");
    ESP_LOGI(TAG, "  1. GPS data received (NMEA sentences)");
    ESP_LOGI(TAG, "  2. Satellites detected (3+ needed for fix)");
    ESP_LOGI(TAG, "  3. GPS fix acquired (only works OUTDOORS!)");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "NOTE: GPS will NOT work indoors!");
    ESP_LOGI(TAG, "      Take device outside for testing.");
    ESP_LOGI(TAG, "");

    bool data_received = false;
    bool fix_acquired = false;
    int loop_count = 0;
    int max_satellites = 0;

    while (true) {
        // Update GPS
        bool new_data = gps.update();

        if (new_data) {
            if (!data_received) {
                ESP_LOGI(TAG, "");
                ESP_LOGI(TAG, "✓✓✓ GPS DATA RECEIVED! ✓✓✓");
                ESP_LOGI(TAG, "GPS module is working and sending NMEA data");
                ESP_LOGI(TAG, "");
                data_received = true;
            }

            // Track max satellites seen
            if (gps.getSatellites() > max_satellites) {
                max_satellites = gps.getSatellites();
                ESP_LOGI(TAG, "Satellites detected: %d (max so far)", max_satellites);
            }

            // Check if we have a fix
            if (gps.isValid()) {
                if (!fix_acquired) {
                    ESP_LOGI(TAG, "");
                    ESP_LOGI(TAG, "🎉🎉🎉 GPS FIX ACQUIRED! 🎉🎉🎉");
                    ESP_LOGI(TAG, "");
                    fix_acquired = true;
                }

                // Show location data
                gps.displayInfo();

                Location loc = gps.getLocation();
                ESP_LOGI(TAG, "Coordinates: %.6f, %.6f", loc.lat, loc.lon);
                ESP_LOGI(TAG, "Copy to Google Maps: %.6f,%.6f", loc.lat, loc.lon);
                ESP_LOGI(TAG, "");
            } else {
                // No fix yet, but getting data
                ESP_LOGI(TAG, "Searching... Satellites: %d (need 4+ for fix)",
                        gps.getSatellites());
            }
        }

        // Status updates every 10 seconds if no data
        if (!data_received && loop_count % 100 == 0 && loop_count > 0) {
            ESP_LOGI(TAG, "");
            ESP_LOGI(TAG, "Still waiting for GPS data...");
            ESP_LOGI(TAG, "");
            ESP_LOGI(TAG, "Troubleshooting checklist:");
            ESP_LOGI(TAG, "  [ ] Is GPS LED blinking?");
            ESP_LOGI(TAG, "      YES = GPS has power");
            ESP_LOGI(TAG, "      NO  = Check power wiring");
            ESP_LOGI(TAG, "");
            ESP_LOGI(TAG, "  [ ] Is wiring correct?");
            ESP_LOGI(TAG, "      GPS TX  -> ESP32 GPIO%d", GPS_RX_PIN);
            ESP_LOGI(TAG, "      GPS RX  -> ESP32 GPIO%d", GPS_TX_PIN);
            ESP_LOGI(TAG, "      GPS VCC -> 5V or 3.3V");
            ESP_LOGI(TAG, "      GPS GND -> GND");
            ESP_LOGI(TAG, "");
            ESP_LOGI(TAG, "  [ ] Try different baud rate?");
            ESP_LOGI(TAG, "      Most NEO-6M use 9600");
            ESP_LOGI(TAG, "      Some use 4800 or 38400");
            ESP_LOGI(TAG, "");
        }

        // After 30 seconds with data but no fix
        if (data_received && !fix_acquired && loop_count == 300) {
            ESP_LOGI(TAG, "");
            ESP_LOGI(TAG, "GPS is working but no fix yet.");
            ESP_LOGI(TAG, "This is NORMAL if you're indoors!");
            ESP_LOGI(TAG, "");
            ESP_LOGI(TAG, "To get GPS fix:");
            ESP_LOGI(TAG, "  1. Go OUTDOORS");
            ESP_LOGI(TAG, "  2. Clear view of sky");
            ESP_LOGI(TAG, "  3. Wait 30-60 seconds");
            ESP_LOGI(TAG, "");
        }

        loop_count++;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

extern "C" void app_main(void)
{
    printf("\n");
    printf("╔════════════════════════════════════════╗\n");
    printf("║     ESP32 GPS Diagnostic Tool         ║\n");
    printf("║     NEO-6M Module Test                 ║\n");
    printf("╚════════════════════════════════════════╝\n");
    printf("\n");

    // Create diagnostic task
    xTaskCreate(gps_diagnostic_task, "GPS_Diag", 8192, NULL, 5, NULL);
}


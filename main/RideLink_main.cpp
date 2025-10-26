#include <cstdio>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"
#include "Display.hpp"
#include "Compass.hpp"
#include "GPS.hpp"

static const char *TAG = "NAVIGATION";

#define LED_PIN GPIO_NUM_12

// Change these to your destination coordinates!
// Use Google Maps to get coordinates: Right-click location → Copy coordinates
#define TARGET_LATITUDE   37.7749    // Example: San Francisco
#define TARGET_LONGITUDE  -122.4194

Display display;
Compass compass;
GPS gps;

void led_setup(const gpio_num_t led_pin);
void led_blink_task(void *pvParameters);
void navigation_task(void *pvParameters);

extern "C" void app_main(void)
{
    printf("\n");
    printf("╔════════════════════════════════════════╗\n");
    printf("║   RideLink Navigation System           ║\n");
    printf("║   GPS + Compass + Display              ║\n");
    printf("╚════════════════════════════════════════╝\n");
    printf("\n");

    led_setup(LED_PIN);

    // Initialize display
    if (!display.init()) {
        ESP_LOGE(TAG, "Failed to initialize display!");
        return;
    }

    // Initialize compass
    if (!compass.init()) {
        ESP_LOGE(TAG, "Failed to initialize compass!");
        // Continue anyway
    }

    // Initialize GPS (using safe pins GPIO25/26)
    if (!gps.init(25, 26, 9600)) {
        ESP_LOGE(TAG, "Failed to initialize GPS!");
        // Continue anyway - can use compass-only mode
    }

    // Create tasks
    xTaskCreate(led_blink_task, "LED_Blink", 2048, NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(navigation_task, "Navigation", 8192, NULL, tskIDLE_PRIORITY + 2, NULL);

    printf("RideLink: Tasks created. System running...\n");
}

void led_setup(const gpio_num_t led_pin)
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pin_bit_mask = (1ULL << led_pin);
    gpio_config(&io_conf);
}

void led_blink_task(void *pvParameters)
{
    uint8_t led_state = 0;
    while (true) {
        led_state = !led_state;
        gpio_set_level(LED_PIN, led_state);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void navigation_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Navigation task started");
    ESP_LOGI(TAG, "Free heap: %lu bytes", esp_get_free_heap_size());

    vTaskDelay(pdMS_TO_TICKS(500));

    // Startup sequence - flash colors
    ESP_LOGI(TAG, "Initializing display...");
    display.fillScreen(Color::RED);
    vTaskDelay(pdMS_TO_TICKS(300));
    display.fillScreen(Color::GREEN);
    vTaskDelay(pdMS_TO_TICKS(300));
    display.fillScreen(Color::BLUE);
    vTaskDelay(pdMS_TO_TICKS(300));
    display.fillScreen(Color::BLACK);

    // Test compass
    int16_t x, y, z;
    if (!compass.readRaw(x, y, z)) {
        ESP_LOGE(TAG, "Compass not responding!");
        display.fillScreen(Color::RED);
        vTaskDelay(pdMS_TO_TICKS(2000));
    } else {
        ESP_LOGI(TAG, "✓ Compass OK! Raw: X=%d, Y=%d, Z=%d", x, y, z);
    }

    // ===== OPTION: CALIBRATE COMPASS =====
    // Uncomment this section if you want to calibrate now
    /*
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== COMPASS CALIBRATION ===");
    ESP_LOGI(TAG, "Place device flat and slowly spin 2 full rotations");
    ESP_LOGI(TAG, "Starting in 3 seconds...");

    display.fillScreen(Color::YELLOW);
    vTaskDelay(pdMS_TO_TICKS(3000));

    compass.startCalibration();
    ESP_LOGI(TAG, ">>> SPIN NOW! <<<");

    for (int i = 0; i < 150; i++) {  // 15 seconds
        compass.readRaw(x, y, z);
        display.fillScreen((i / 5) % 2 == 0 ? Color::CYAN : Color::MAGENTA);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    compass.finishCalibration();
    display.fillScreen(Color::GREEN);
    vTaskDelay(pdMS_TO_TICKS(1000));
    display.fillScreen(Color::BLACK);
    */

    // ===== OR: USE PRE-SET CALIBRATION =====
    // Uncomment and use your calibration values from previous run
    // compass.setCalibrationOffsets(202, 263, -433);
    // compass.setCalibrationScales(1.06, 0.95, 0.99);
    // ESP_LOGI(TAG, "Using pre-set compass calibration");

    // Optional: Set magnetic declination for your location
    // Find yours at: https://www.magnetic-declination.com/
    // compass.setDeclination(3.5);  // Example: 3.5° East

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, "NAVIGATION SYSTEM READY");
    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, "Target: %.6f, %.6f", TARGET_LATITUDE, TARGET_LONGITUDE);
    ESP_LOGI(TAG, "");

    // State tracking
    bool gps_fix = false;
    bool gps_ever_had_fix = false;
    int no_gps_counter = 0;
    int log_counter = 0;

    // Main navigation loop
    while (true) {
        // Update GPS
        gps.update();

        // Get compass heading
        float heading = compass.getHeading();

        // Check GPS status
        if (gps.isValid()) {
            // ===== GPS NAVIGATION MODE =====

            if (!gps_fix) {
                ESP_LOGI(TAG, "");
                ESP_LOGI(TAG, "🎉 GPS FIX ACQUIRED! 🎉");
                ESP_LOGI(TAG, "Switching to GPS navigation mode");
                ESP_LOGI(TAG, "");
                gps_fix = true;
                gps_ever_had_fix = true;
                display.fillScreen(Color::GREEN);
                vTaskDelay(pdMS_TO_TICKS(500));
                display.fillScreen(Color::BLACK);
            }

            // Get current location
            Location loc = gps.getLocation();

            // Calculate bearing to target
            int bearing_to_target = gps.bearingTo(TARGET_LATITUDE, TARGET_LONGITUDE);

            // Calculate distance to target
            int distance_to_target = gps.distanceTo(TARGET_LATITUDE, TARGET_LONGITUDE);

            // Calculate arrow direction
            // Arrow shows which way to turn to face target
            float arrow_angle = bearing_to_target - heading;
            while (arrow_angle < 0) arrow_angle += 360;
            while (arrow_angle >= 360) arrow_angle -= 360;

            // Update display - GREEN arrow for GPS mode
            display.updateArrow((int16_t)arrow_angle, Color::GREEN);

            // Log periodically
            if (log_counter++ % 10 == 0) {
                ESP_LOGI(TAG, "GPS MODE | Dist: %dm | Bearing: %d° | Heading: %.0f° | Arrow: %.0f°",
                        distance_to_target, bearing_to_target, heading, arrow_angle);
                ESP_LOGI(TAG, "  Position: %.6f, %.6f | Sats: %d",
                        loc.lat, loc.lon, gps.getSatellites());
            }

            // Check if we're close to destination
            if (distance_to_target < 10) {
                ESP_LOGI(TAG, "🎯 ARRIVED! You're within 10m of target!");
                display.fillScreen(Color::GREEN);
                vTaskDelay(pdMS_TO_TICKS(300));
                display.fillScreen(Color::BLACK);
            }

            no_gps_counter = 0;

        } else {
            // ===== COMPASS-ONLY MODE (No GPS fix) =====

            if (gps_fix) {
                ESP_LOGW(TAG, "GPS fix lost! Switching to compass-only mode");
                gps_fix = false;
            }

            // Show compass direction (North-pointing arrow)
            float arrow_angle = 360.0f - heading;
            while (arrow_angle >= 360) arrow_angle -= 360;

            // YELLOW arrow for compass-only mode
            display.updateArrow((int16_t)arrow_angle, Color::YELLOW);

            // Log status
            no_gps_counter++;
            if (no_gps_counter % 50 == 0) {
                if (!gps_ever_had_fix) {
                    ESP_LOGI(TAG, "COMPASS MODE | Heading: %.0f° | Waiting for GPS fix...",
                            heading);
                    ESP_LOGI(TAG, "  Satellites visible: %d (need 4+ for fix)",
                            gps.getSatellites());

                    if (no_gps_counter == 50) {
                        ESP_LOGI(TAG, "");
                        ESP_LOGI(TAG, "📍 For GPS fix: Go OUTDOORS with clear sky view!");
                        ESP_LOGI(TAG, "   GPS will NOT work indoors.");
                        ESP_LOGI(TAG, "");
                    }
                } else {
                    ESP_LOGI(TAG, "COMPASS MODE | Heading: %.0f° | GPS searching...",
                            heading);
                }
            }
        }

        // Update at 10Hz
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

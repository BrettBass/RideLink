#include <cstdio>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "GPS.hpp"
#include "Compass.hpp"
#include "Display.hpp"
#include "CompassCalibrator.hpp"
#include "LoRa.hpp"

static const char* TAG = "MAIN";

// Button GPIO (BOOT button on most ESP32 boards)
#define BUTTON_GPIO GPIO_NUM_0
#define BUTTON_HOLD_TIME_MS 3000  // Hold for 3 seconds to enter calibration

// Device configuration
#define THIS_DEVICE_ID 1  // Change to 2 for the second device
#define LORA_FREQUENCY 915.0  // 915 MHz for US, 868 MHz for EU

// GPS, Compass, Display, LoRa, and Calibrator instances
GPS gps;
Compass compass;
Display display;
LoRa lora;
CompassCalibrator* calibrator = nullptr;

// Peer device data
RideLinkPacket peer_packet;
bool peer_found = false;
uint32_t last_peer_time = 0;

// Task handles
TaskHandle_t gps_task_handle = NULL;
TaskHandle_t compass_task_handle = NULL;
TaskHandle_t lora_task_handle = NULL;

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

// Task to handle LoRa communication
void lora_task(void* param) {
    uint32_t last_broadcast = 0;
    const uint32_t BROADCAST_INTERVAL = 2000;  // Send location every 2 seconds
    
    // Log the actual packet size
    ESP_LOGI(TAG, "RideLinkPacket size: %d bytes", sizeof(RideLinkPacket));
    
    // Put LoRa in receive mode
    lora.receive();
    
    while (!calibration_mode) {
        uint32_t now = esp_log_timestamp();
        
        // Send our location periodically if we have GPS fix
        if (now - last_broadcast >= BROADCAST_INTERVAL) {
            last_broadcast = now;
            
            if (gps.isFixed()) {
                GPSPacket gps_data = gps.getPacket();
                float heading = compass.getHeading();
                
                if (lora.broadcastLocation(gps_data, (int16_t)heading)) {
                    ESP_LOGI(TAG, "Broadcast location: %.6f, %.6f | Heading: %.0f°", 
                             gps_data.lat, gps_data.lon, heading);
                } else {
                    ESP_LOGE(TAG, "Failed to broadcast location");
                }
                
                // Go back to receive mode
                lora.receive();
            }
        }
        
        // Check for incoming packets with improved debugging
        if (lora.available()) {
            ESP_LOGI(TAG, "LoRa packet available!");
            
            // Try to receive as raw bytes first to see what we're getting
            uint8_t buffer[64];
            uint8_t length = lora.receiveBytes(buffer, sizeof(buffer));
            
            if (length > 0) {
                ESP_LOGI(TAG, "Raw packet received: %d bytes", length);
                ESP_LOGI(TAG, "First 4 bytes: %02X %02X %02X %02X", 
                         buffer[0], buffer[1], buffer[2], buffer[3]);
                
                // Check if it's the right size for our packet
                if (length == sizeof(RideLinkPacket)) {
                    // Copy to packet structure
                    memcpy(&peer_packet, buffer, sizeof(RideLinkPacket));
                    
                    // Verify checksum
                    if (peer_packet.verifyChecksum()) {
                        // Check if it's from the other device
                        if (peer_packet.device_id != THIS_DEVICE_ID) {
                            peer_found = true;
                            last_peer_time = now;
                            peer_packet.rssi = lora.getPacketRSSI();
                            
                            ESP_LOGI(TAG, "=== PEER FOUND ===");
                            ESP_LOGI(TAG, "Device ID: %d", peer_packet.device_id);
                            ESP_LOGI(TAG, "Location: %.6f, %.6f", peer_packet.gps.lat, peer_packet.gps.lon);
                            ESP_LOGI(TAG, "Heading: %d°", peer_packet.compass_heading);
                            ESP_LOGI(TAG, "RSSI: %d dBm", peer_packet.rssi);
                            ESP_LOGI(TAG, "Battery: %d%%", peer_packet.battery_level);
                            
                            // Calculate distance if we have GPS fix
                            if (gps.isFixed()) {
                                uint32_t distance = GPS::calculateDistance(
                                    gps.getLatitude(), gps.getLongitude(),
                                    peer_packet.gps.lat, peer_packet.gps.lon
                                );
                                ESP_LOGI(TAG, "Distance: %lu meters", (unsigned long)distance);
                            }
                        } else {
                            ESP_LOGD(TAG, "Received own packet, ignoring");
                        }
                    } else {
                        ESP_LOGW(TAG, "Packet checksum failed!");
                    }
                } else {
                    ESP_LOGW(TAG, "Packet size mismatch: got %d, expected %d", 
                             length, sizeof(RideLinkPacket));
                }
            }
            
            // Back to receive mode
            lora.receive();
        }
        
        // Check if peer connection lost (no packet for 10 seconds)
        if (peer_found && (now - last_peer_time > 10000)) {
            ESP_LOGW(TAG, "Lost connection to peer device");
            peer_found = false;
        }
        
        vTaskDelay(pdMS_TO_TICKS(50));
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

            // Calculate arrow angle based on target
            int16_t arrow_angle = 0;
            uint16_t arrow_color = Color::GRAY;  // Gray when no target
            const char* mode_text = "NO TARGET";
            uint32_t distance_m = 0;

            if (peer_found) {
                // We have a peer device - point to it
                if (gps.isFixed()) {
                    // Both devices have GPS - calculate precise bearing
                    double my_lat = gps.getLatitude();
                    double my_lon = gps.getLongitude();

                    // Calculate bearing from current position to peer
                    uint16_t bearing_to_peer = GPS::calculateHeading(
                        my_lat, my_lon, 
                        peer_packet.gps.lat, peer_packet.gps.lon
                    );

                    // Calculate distance to peer
                    distance_m = GPS::calculateDistance(
                        my_lat, my_lon,
                        peer_packet.gps.lat, peer_packet.gps.lon
                    );

                    // Calculate arrow angle (accounting for display inversion)
                    float arrow_angle_f = (360.0f - heading) + (float)bearing_to_peer;

                    // Normalize to 0-359
                    while (arrow_angle_f >= 360) arrow_angle_f -= 360;
                    while (arrow_angle_f < 0) arrow_angle_f += 360;

                    arrow_angle = (int16_t)arrow_angle_f;
                    arrow_color = Color::GREEN;  // Green for peer tracking
                    mode_text = "PEER FOUND";
                } else {
                    // No GPS fix - can't calculate bearing to peer
                    arrow_angle = (int16_t)(360 - heading);  // Just show north
                    arrow_color = Color::YELLOW;  // Yellow - peer found but no GPS
                    mode_text = "PEER (NO GPS)";
                }
            } else if (gps.isFixed()) {
                // No peer but have GPS - point north
                arrow_angle = (int16_t)(360 - heading);
                arrow_color = Color::CYAN;  // Cyan for north with GPS
                mode_text = "NORTH";
            } else {
                // No peer, no GPS - just show magnetic north
                arrow_angle = (int16_t)(360 - heading);
                arrow_color = Color::GRAY;  // Gray for basic compass
                mode_text = "MAG NORTH";
            }

            // Update arrow on display
            display.updateArrow(arrow_angle, arrow_color);

            // Draw status indicators
            // GPS status (top-right)
            if (gps.isFixed()) {
                display.fillCircle(220, 20, 5, Color::GREEN);
            } else {
                display.drawCircle(220, 20, 5, Color::RED);
            }

            // Calibration status (top-left)
            if (calibrator && calibrator->getCalibrationData().isValid()) {
                float coverage = calibrator->getCalibrationData().coverage_score;
                uint16_t cal_color = Color::RED;
                if (coverage > 30) cal_color = Color::YELLOW;
                if (coverage > 60) cal_color = Color::GREEN;
                display.fillCircle(20, 20, 5, cal_color);
            } else {
                display.drawCircle(20, 20, 5, Color::GRAY);
            }

            // Peer connection status (center-top)
            if (peer_found) {
                display.fillCircle(120, 20, 5, Color::GREEN);
                // Show distance if available
                if (distance_m > 0) {
                    // Display distance (you'd need to add text rendering for this)
                    // For now, use circle size to indicate distance
                    int radius = 3;
                    if (distance_m < 100) radius = 8;
                    else if (distance_m < 500) radius = 6;
                    else if (distance_m < 1000) radius = 4;
                    display.fillCircle(120, 35, radius, Color::BLUE);
                }
            } else {
                display.drawCircle(120, 20, 5, Color::RED);
            }

            // Log every second with values
            if (now - last_display >= 1000) {
                last_display = now;
                if (read_success) {
                    if (peer_found && gps.isFixed()) {
                        ESP_LOGI(TAG, "Mode: %s | Heading: %.1f° | Bearing: %d° | Distance: %lu m | RSSI: %d dBm",
                                 mode_text, heading, arrow_angle, (unsigned long)distance_m, peer_packet.rssi);
                    } else {
                        ESP_LOGI(TAG, "Mode: %s | Heading: %.1f° | Arrow: %d°",
                                 mode_text, heading, arrow_angle);
                    }
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
    ESP_LOGI(TAG, "    RideLink - Peer Locator");
    ESP_LOGI(TAG, "    Device ID: %d", THIS_DEVICE_ID);
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

    // Initialize LoRa
    ESP_LOGI(TAG, "Initializing LoRa...");
    lora.setDeviceId(THIS_DEVICE_ID);
    if (!lora.begin(LORA_FREQUENCY)) {
        ESP_LOGE(TAG, "Failed to initialize LoRa!");
        ESP_LOGE(TAG, "Check wiring:");
        ESP_LOGE(TAG, "  MISO -> GPIO19");
        ESP_LOGE(TAG, "  MOSI -> GPIO23");
        ESP_LOGE(TAG, "  SCK  -> GPIO18");
        ESP_LOGE(TAG, "  NSS  -> GPIO32");
        ESP_LOGE(TAG, "  RST  -> GPIO12");
        ESP_LOGE(TAG, "  DIO0 -> GPIO2");
        return;
    }
    ESP_LOGI(TAG, "✓ LoRa initialized successfully");
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
    ESP_LOGI(TAG, "RIDELINK PEER LOCATOR - INSTRUCTIONS:");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "DEVICE ID: %d", THIS_DEVICE_ID);
    ESP_LOGI(TAG, "RideLinkPacket size: %d bytes", sizeof(RideLinkPacket));
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "ARROW COLORS:");
    ESP_LOGI(TAG, "  - GREEN: Pointing to peer device");
    ESP_LOGI(TAG, "  - YELLOW: Peer found but no GPS fix");
    ESP_LOGI(TAG, "  - CYAN: Pointing north (GPS fix, no peer)");
    ESP_LOGI(TAG, "  - GRAY: Magnetic north (no GPS, no peer)");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "STATUS INDICATORS:");
    ESP_LOGI(TAG, "  - Top-right dot: GPS (green=fix, red=no fix)");
    ESP_LOGI(TAG, "  - Top-center dot: Peer (green=connected, red=searching)");
    ESP_LOGI(TAG, "  - Top-left dot: Calibration quality");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "FEATURES:");
    ESP_LOGI(TAG, "  - Automatically finds and tracks peer device");
    ESP_LOGI(TAG, "  - Shows distance when both have GPS fix");
    ESP_LOGI(TAG, "  - Signal strength indicator (RSSI)");
    ESP_LOGI(TAG, "  - Battery level sharing");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "CALIBRATION:");
    ESP_LOGI(TAG, "  - Hold BOOT button for 3 seconds");
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
            if (lora_task_handle == NULL) {
                xTaskCreate(lora_task, "lora_task", 4096, NULL, 5, &lora_task_handle);
            }

            vTaskDelay(pdMS_TO_TICKS(1000));

            // Print LoRa status every 30 seconds
            static uint32_t last_status = 0;
            if (esp_log_timestamp() - last_status > 30000) {
                last_status = esp_log_timestamp();
                lora.printStatus();
            }
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
            if (lora_task_handle != NULL) {
                vTaskDelete(lora_task_handle);
                lora_task_handle = NULL;
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

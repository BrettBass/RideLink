#include <cstdio>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "LoRa.hpp"

static const char* TAG = "LORA_TEST";

// Device configuration - CHANGE THIS FOR EACH DEVICE
#define THIS_DEVICE_ID 1  // Change to 2 for the second device
#define LORA_FREQUENCY 915.0  // 915 MHz for US, 868 MHz for EU

LoRa lora;

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "=================================");
    ESP_LOGI(TAG, "    LoRa Simple Test");
    ESP_LOGI(TAG, "    Device ID: %d", THIS_DEVICE_ID);
    ESP_LOGI(TAG, "=================================");

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

    ESP_LOGI(TAG, "✓ LoRa initialized");

    // Print initial registers
    lora.printRegisters();

    uint32_t counter = 0;
    uint32_t last_send = 0;
    const uint32_t SEND_INTERVAL = 3000;  // Send every 3 seconds

    // Put in receive mode
    lora.receive();
    ESP_LOGI(TAG, "Listening for packets...");

    while (true) {
        uint32_t now = esp_log_timestamp();

        // Send a simple test packet periodically
        if (now - last_send >= SEND_INTERVAL) {
            last_send = now;

            // Create a simple test message
            char message[50];
            snprintf(message, sizeof(message), "Hello from Device %d, count: %lu",
                     THIS_DEVICE_ID, (unsigned long)counter++);

            ESP_LOGI(TAG, "");
            ESP_LOGI(TAG, ">>> SENDING: %s", message);

            // Send the message
            lora.idle();  // Must go to idle before sending
            if (lora.sendBytes((uint8_t*)message, strlen(message))) {
                ESP_LOGI(TAG, "✓ Sent successfully");
            } else {
                ESP_LOGE(TAG, "✗ Send failed!");
            }

            // Back to receive mode
            lora.receive();
        }

        // Check for received packets
        if (lora.available()) {
            uint8_t buffer[255];
            uint8_t length = lora.receiveBytes(buffer, sizeof(buffer));

            if (length > 0) {
                buffer[length] = '\0';  // Null terminate for string printing

                ESP_LOGI(TAG, "");
                ESP_LOGI(TAG, "<<< RECEIVED %d bytes:", length);
                ESP_LOGI(TAG, "    Message: %s", (char*)buffer);
                ESP_LOGI(TAG, "    RSSI: %d dBm", lora.getPacketRSSI());
                ESP_LOGI(TAG, "    SNR: %d dB", lora.getPacketSNR());
                ESP_LOGI(TAG, "");
            }
        }

        // Print stats every 30 seconds
        static uint32_t last_stats = 0;
        if (now - last_stats > 30000) {
            last_stats = now;
            lora.printStatus();
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

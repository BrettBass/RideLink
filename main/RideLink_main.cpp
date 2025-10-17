#include <cstdio>
#include "freertos/FreeRTOS.h"
#include "freertos/FreeRTOSConfig_arch.h"
#include "freertos/idf_additions.h"
#include "driver/gpio.h"
#include "soc/gpio_num.h"
#include "esp_log.h"
#include "esp_system.h"
#include "Display.hpp"

static const char *TAG = "RIDE_LINK";

#define LED_PIN GPIO_NUM_12

Display display;

void led_setup(const gpio_num_t led_pin);
void led_blink_task(void *pvParameters);
void display_test_task(void *pvParameters);

extern "C" void app_main(void)
{
    printf("RideLink: C++ Application Starting Up...\n");

    led_setup(LED_PIN);

    // Initialize display
    if (!display.init()) {
        ESP_LOGE(TAG, "Failed to initialize display!");
        return;
    }

    // Create tasks with larger stack for display task
    xTaskCreate(led_blink_task, "LED_Blink", 2048, NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(display_test_task, "Display_Test", 8192, NULL, tskIDLE_PRIORITY + 2, NULL);

    printf("RideLink: Tasks created. app_main exiting...\n");
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
        printf("RideLink: C++ Application running...\n");
        led_state = !led_state;
        gpio_set_level(LED_PIN, led_state);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void display_test_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Display test task started");
    ESP_LOGI(TAG, "Free heap: %lu bytes", esp_get_free_heap_size());

    // Wait for display to be ready
    vTaskDelay(pdMS_TO_TICKS(500));

    // First test - fill with solid colors to verify display is working
    ESP_LOGI(TAG, "Testing solid colors...");
    display.fillScreen(Color::RED);
    vTaskDelay(pdMS_TO_TICKS(1000));
    display.fillScreen(Color::GREEN);
    vTaskDelay(pdMS_TO_TICKS(1000));
    display.fillScreen(Color::BLUE);
    vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_LOGI(TAG, "Color test complete, starting arrow rotation");
    ESP_LOGI(TAG, "Free heap after color test: %lu bytes", esp_get_free_heap_size());

    // Clear screen to black
    display.fillScreen(Color::BLACK);

    int angle = 0;

    // Infinite loop - smoothly rotate arrow 360 degrees
    while (true) {
        ESP_LOGI(TAG, "Drawing arrow at angle: %d degrees", angle);

        // Use updateArrow for smooth animation (only redraws if angle changed enough)
        display.updateArrow(angle, Color::GREEN);

        // Increment angle
        angle = (angle + 10) % 360;

        // Small delay for smooth rotation
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


#include <cstdio>
#include "freertos/FreeRTOS.h"
#include "freertos/FreeRTOSConfig_arch.h"
#include "freertos/idf_additions.h"
#include "driver/gpio.h"
#include "soc/gpio_num.h"

// Include the project-wide configuration header
//#include "config.h"

//static const gpio_num_t led_pin = GPIO_NUM_12;
#define LED_PIN GPIO_NUM_12

void led_setup(const gpio_num_t led_pin);
void led_blink_task(void *pvParameters);

// The main entry point must be declared with C linkage for the IDF to find it
extern "C" void app_main(void)
{
    printf("RideLink: C++ Application Starting Up...\n");


    led_setup(LED_PIN);


    xTaskCreate(
        led_blink_task,
        "LED_Blink",
        configMINIMAL_STACK_SIZE,
        NULL,
        tskIDLE_PRIORITY + 1,
        NULL);

    printf("RideLink: LED Blinking Task Created. app_main exiting...\n");
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
    return;
}

void led_blink_task(void *pvParameters)
{
    uint8_t led_state = 0;
    while (true) {
        printf("RideLink: C++ Application running...\n");
        led_state = !led_state;
        gpio_set_level(LED_PIN, led_state);
        vTaskDelay(100);
        //vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1000ms
    }
}

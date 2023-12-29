/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/touch_pad.h"
#include "soc/rtc_periph.h"
#include "soc/sens_periph.h"
#include "driver/gpio.h"
#include "esp_rom_gpio.h"
#include <touch.h>

static const char *TAG = "Touch pad";

static QueueHandle_t que_touch = NULL;

uint16_t ledstate = 0x0000;

/**
 * @brief led initialize peripherals
 *
 * @param none
 */
void led_init(void)
{
    esp_rom_gpio_pad_select_gpio(LED_BLUE_1);
    esp_rom_gpio_pad_select_gpio(LED_BLUE_2);
    esp_rom_gpio_pad_select_gpio(LED_BLUE_3);
    esp_rom_gpio_pad_select_gpio(LED_BLUE_4);

    esp_rom_gpio_pad_select_gpio(LED_RED_1);
    esp_rom_gpio_pad_select_gpio(LED_RED_2);
    esp_rom_gpio_pad_select_gpio(LED_RED_3);
    esp_rom_gpio_pad_select_gpio(LED_RED_4);

    esp_rom_gpio_pad_select_gpio(RL_1);
    esp_rom_gpio_pad_select_gpio(RL_2);
    esp_rom_gpio_pad_select_gpio(RL_3);
    esp_rom_gpio_pad_select_gpio(RL_4);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(LED_BLUE_1, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_BLUE_2, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_BLUE_3, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_BLUE_4, GPIO_MODE_OUTPUT);

    gpio_set_direction(LED_RED_1, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_RED_2, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_RED_3, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_RED_4, GPIO_MODE_OUTPUT);

    gpio_set_direction(RL_1, GPIO_MODE_OUTPUT);
    gpio_set_direction(RL_2, GPIO_MODE_OUTPUT);
    gpio_set_direction(RL_3, GPIO_MODE_OUTPUT);
    gpio_set_direction(RL_4, GPIO_MODE_OUTPUT);

    gpio_set_level(LED_BLUE_1, 1);
    gpio_set_level(LED_BLUE_2, 1);
    gpio_set_level(LED_BLUE_3, 1);
    gpio_set_level(LED_BLUE_4, 1);
}

void toggle_led_color(gpio_num_t red_pin, gpio_num_t blue_pin, LedColor *currentColor)
{
    *currentColor = (*currentColor == LED_RED) ? LED_BLUE : LED_RED;
    gpio_set_level(red_pin, (*currentColor == LED_RED) ? 1 : 0);
    gpio_set_level(blue_pin, (*currentColor == LED_BLUE) ? 1 : 0);
}

/*
  Handle an interrupt triggered when a pad is touched.
  Recognize what pad has been touched and save it in a table.
 */
static void touchsensor_interrupt_cb(void *arg)
{
    int task_awoken = pdFALSE;
    touch_event_t evt;

    evt.intr_mask = touch_pad_read_intr_status_mask();
    evt.pad_status = touch_pad_get_status();
    evt.pad_num = touch_pad_get_current_meas_channel();

    xQueueSendFromISR(que_touch, &evt, &task_awoken);
    if (task_awoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static void tp_example_set_thresholds(void)
{
    uint32_t touch_value;
    for (int i = 0; i < TOUCH_BUTTON_NUM; i++) {
        //read benchmark value
        touch_pad_read_benchmark(button[i], &touch_value);
        //set interrupt threshold.
        touch_pad_set_thresh(button[i], touch_value * button_threshold[i]);
        ESP_LOGI(TAG, "touch pad [%d] base %"PRIu32", thresh %"PRIu32,
                 button[i], touch_value, (uint32_t)(touch_value * button_threshold[i]));
    }
}

static void touchsensor_filter_set(touch_filter_mode_t mode)
{
    /* Filter function */
    touch_filter_config_t filter_info = {
        .mode = mode,           // Test jitter and filter 1/4.
        .debounce_cnt = 1,      // 1 time count.
        .noise_thr = 0,         // 50%
        .jitter_step = 4,       // use for jitter mode.
        .smh_lvl = TOUCH_PAD_SMOOTH_IIR_2,
    };
    touch_pad_filter_set_config(&filter_info);
    touch_pad_filter_enable();
    ESP_LOGI(TAG, "touch pad filter init");
}

static void tp_threshold_read_task(void *pvParameter)
{
    touch_event_t evt = {0};
    /* Wait touch sensor init done */
    vTaskDelay(50 / portTICK_PERIOD_MS);
    tp_example_set_thresholds();

    while (1) {
        int ret = xQueueReceive(que_touch, &evt, (TickType_t)portMAX_DELAY);
        if (ret != pdTRUE) {
            continue;
        }
        if (evt.intr_mask & TOUCH_PAD_INTR_MASK_ACTIVE) {
            ESP_LOGI(TAG, "TouchSensor [%"PRIu32"] be activated, status mask 0x%"PRIu32"", evt.pad_num, evt.pad_status);

            int button_index = -1;
            for (int i = 0; i < TOUCH_BUTTON_NUM; i++) {
                if (button[i] == evt.pad_num) {
                    button_index = i;
                    break;
                }
            }

            if (button_index != -1) {
                // Found the corresponding button
                toggle_led_color(leds[button_index].redPin, leds[button_index].bluePin, &leds[button_index].currentColor);
            }
        }
        if (evt.intr_mask & TOUCH_PAD_INTR_MASK_INACTIVE) {
            ESP_LOGI(TAG, "TouchSensor [%"PRIu32"] be inactivated, status mask 0x%"PRIu32, evt.pad_num, evt.pad_status);
        }
        if (evt.intr_mask & TOUCH_PAD_INTR_MASK_SCAN_DONE) {
            ESP_LOGI(TAG, "The touch sensor group measurement is done [%"PRIu32"].", evt.pad_num);
        }
        if (evt.intr_mask & TOUCH_PAD_INTR_MASK_TIMEOUT) {
            /* Add your exception handling in here. */
            ESP_LOGI(TAG, "Touch sensor channel %"PRIu32" measure timeout. Skip this exception channel!!", evt.pad_num);
            touch_pad_timeout_resume(); // Point on the next channel to measure.
        }
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

static void touch_read_raw_task(void *pvParameter)
{
    uint32_t touch_value;

    /* Wait touch sensor init done */
    vTaskDelay(100 / portTICK_PERIOD_MS);
    //printf("Touch Sensor read, the output format is: \nTouchpad num:[raw data]\n\n");

    while (1) {
        for (int i = 0; i < TOUCH_BUTTON_NUM; i++) {
            touch_pad_read_raw_data(button[i], &touch_value);    // read raw data.
            // check_touch_and_toggle_LED(i, touch_value);
            //printf("T%d: [%4"PRIu32"] ", button[i], touch_value);
            printf("%4"PRIu32",", touch_value);
            //ESP_LOGI("Status", "%d", touch_pad_get_status());

        }
        printf("\n");
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    if (que_touch == NULL) {
        que_touch = xQueueCreate(TOUCH_BUTTON_NUM, sizeof(touch_event_t));
    }
    // Initialize touch pad peripheral, it will start a timer to run a filter
    ESP_LOGI(TAG, "Initializing touch pad");
    /* Initialize touch pad peripheral. */
    touch_pad_init();
    for (int i = 0; i < TOUCH_BUTTON_NUM; i++) {
        touch_pad_config(button[i]);
    }
    led_init();

#if TOUCH_CHANGE_CONFIG
    /* If you want change the touch sensor default setting, please write here(after initialize). There are examples: */
    touch_pad_set_measurement_interval(TOUCH_PAD_SLEEP_CYCLE_DEFAULT);
    touch_pad_set_charge_discharge_times(TOUCH_PAD_MEASURE_CYCLE_DEFAULT);
    touch_pad_set_voltage(TOUCH_PAD_HIGH_VOLTAGE_THRESHOLD, TOUCH_PAD_LOW_VOLTAGE_THRESHOLD, TOUCH_PAD_ATTEN_VOLTAGE_THRESHOLD);
    touch_pad_set_idle_channel_connect(TOUCH_PAD_IDLE_CH_CONNECT_DEFAULT);
    for (int i = 0; i < TOUCH_BUTTON_NUM; i++) {
        touch_pad_set_cnt_mode(button[i], TOUCH_PAD_SLOPE_DEFAULT, TOUCH_PAD_TIE_OPT_DEFAULT);
    }
#endif

#if TOUCH_BUTTON_DENOISE_ENABLE
    /* Denoise setting at TouchSensor 0. */
    touch_pad_denoise_t denoise = {
        /* The bits to be cancelled are determined according to the noise level. */
        .grade = TOUCH_PAD_DENOISE_BIT4,
        /* By adjusting the parameters, the reading of T0 should be approximated to the reading of the measured channel. */
        .cap_level = TOUCH_PAD_DENOISE_CAP_L4,
    };
    touch_pad_denoise_set_config(&denoise);
    touch_pad_denoise_enable();
    ESP_LOGI(TAG, "Denoise function init");
#endif

    /* Filter setting */
    touchsensor_filter_set(TOUCH_PAD_FILTER_IIR_16);
    touch_pad_timeout_set(true, SOC_TOUCH_PAD_THRESHOLD_MAX);
    /* Register touch interrupt ISR, enable intr type. */
    touch_pad_isr_register(touchsensor_interrupt_cb, NULL, TOUCH_PAD_INTR_MASK_ALL);
    /* If you have other touch algorithm, you can get the measured value after the `TOUCH_PAD_INTR_MASK_SCAN_DONE` interrupt is generated. */
    touch_pad_intr_enable(TOUCH_PAD_INTR_MASK_ACTIVE | TOUCH_PAD_INTR_MASK_INACTIVE | TOUCH_PAD_INTR_MASK_TIMEOUT);

    /* Enable touch sensor clock. Work mode is "timer trigger". */
    touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER);
    touch_pad_fsm_start();

    // Start a task to show what pads have been touched
    xTaskCreate(&tp_threshold_read_task, "touch_pad_read_task", 4096, NULL, 5, NULL);
    xTaskCreate(&touch_read_raw_task, "touch_pad_read_task", 4096, NULL, 6, NULL);

}

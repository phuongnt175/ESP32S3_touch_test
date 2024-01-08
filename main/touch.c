/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */
/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
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
/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/
static const char *TAG = "Touch pad";

static QueueHandle_t que_touch = NULL;

uint16_t ledstate = 0x0000;

const uint8_t KEY_ON       = 80;
const uint8_t KEY_OFF      = 60;
const uint8_t KEY_RELEASE  = 50;
const uint8_t KEY_NOISE    = 20;
#define THRESHOLD_DETECT_SMALL_NOISE        40
#define THRESHOLD_DETECT_MEDIUM_NOISE       60
#define THRESHOLD_DETECT_BIG_NOISE          KEY_ON

static uint8_t countTimeSmallDown[TOUCH_BUTTON_NUM]  = { 0 };
static uint8_t countTimeMediumDown[TOUCH_BUTTON_NUM] = { 0 };
static uint8_t countTimeHighDown[TOUCH_BUTTON_NUM]   = { 0 };

static uint8_t countTimeSmallUp[TOUCH_BUTTON_NUM]    = { 0 };
static uint8_t countTimeMediumUp[TOUCH_BUTTON_NUM]   = { 0 };
static uint8_t countTimeHighUp[TOUCH_BUTTON_NUM]     = { 0 };

static TouchParameters_str touchBuffQueue[TOUCH_BUTTON_NUM];
static uint8_t touchModuleMode = UPDATE_BASE_MODE;

/******************************************************************************/
/*                          PRIVATE FUNCTIONS DECLERATION                     */
/******************************************************************************/

static uint8_t TOUCH_GetTouchValue(void);
static void TOUCH_ChangeBaseValue(uint8_t channel, uint8_t ratio);
static void TOUCH_CalibBaseLine(void);
static uint8_t TOUCH_UpdateBaseLine(void);
void TOUCH_MainProcessFuntion(uint8_t enableUpdateBase);

/******************************************************************************/
/*                                  FUNCTIONS                                 */
/******************************************************************************/

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
}

int find_button_index(gpio_num_t red_pin, gpio_num_t blue_pin)
{
    for (int i = 0; i < TOUCH_BUTTON_NUM; i++) {
        if (leds[i].redPin == red_pin && leds[i].bluePin == blue_pin) {
            return i;
        }
    }
    return -1;  // Button not found
}

void toggle_led_color(gpio_num_t red_pin, gpio_num_t blue_pin, LedColor *currentColor, uint16_t *ledstate)
{
    *currentColor = (*currentColor == LED_RED) ? LED_BLUE : LED_RED;
    gpio_set_level(red_pin, (*currentColor == LED_RED) ? 1 : 0);
    gpio_set_level(blue_pin, (*currentColor == LED_BLUE) ? 1 : 0);

    // Toggle bitmask
    int button_index = find_button_index(red_pin, blue_pin);
    if (button_index != -1) {
        *ledstate ^= (1 << button_index);
        // Set relay based on LED status
        if (*currentColor == LED_RED) {
            // Red LED is on, set relay high
            gpio_set_level(leds[button_index].relayPin, 1);
        } else {
            // Blue LED is on, set relay low
            gpio_set_level(leds[button_index].relayPin, 0);
        }
    }
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
        // ESP_LOGI(TAG, "touch pad [%d] base %"PRIu32", thresh %"PRIu32,
        //         button[i], touch_value, (uint32_t)(touch_value * button_threshold[i]));
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

int find_button_index_from_pad(touch_pad_t pad)
{
    for (int i = 0; i < TOUCH_BUTTON_NUM; i++) {
        if (button[i] == pad) {
            return i;
        }
    }
    return -1;  // Button not found
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
            //ESP_LOGI(TAG, "TouchSensor [%"PRIu32"] be activated, status mask 0x%"PRIu32"", evt.pad_num, evt.pad_status);
            // Find the corresponding button
            int button_index = find_button_index_from_pad(evt.pad_num);
            if (button_index != -1) {
                //ESP_LOGI(TAG, "Button %d pressed", button_index);
                // Found the corresponding button
                toggle_led_color(leds[button_index].redPin, leds[button_index].bluePin, &leds[button_index].currentColor, &ledstate);
            }
        }
        
        if (evt.intr_mask & TOUCH_PAD_INTR_MASK_INACTIVE) {
            //ESP_LOGI(TAG, "TouchSensor [%"PRIu32"] be inactivated, status mask 0x%"PRIu32, evt.pad_num, evt.pad_status);
            // ESP_LOGI(TAG, "inactive");
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
            touch_pad_read_raw_data(button[i], &touch_value);  // read raw data.
            //printf("T%d: [%4"PRIu32"] ", button[i], touch_value);
            printf("%4"PRIu32",", touch_value);
            printf("%4"PRIu32",", touchBuffQueue[i].baseTouchValue);
            //ESP_LOGI("Status", "%d", touch_pad_get_status());
        }
        printf("\n");
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
}

static void update_base_line_task(void *pvParameter)
{
    vTaskDelay(100 / portTICK_PERIOD_MS);
    while(1)
    {
        TOUCH_MainProcessFuntion(1);
        vTaskDelay(800 / portTICK_PERIOD_MS);
    }
}

static void touch_handle_task(void *pvParameter)
{
    vTaskDelay(100 / portTICK_PERIOD_MS);
    uint8_t button_states[TOUCH_BUTTON_NUM] = {0};  // 0: Released, 1: Pressed, 2: Held
    uint32_t hold_time[TOUCH_BUTTON_NUM] = {0};

    while (1)
    {
        if (TOUCH_GetTouchValue())
        {
            for (int i = 0; i < TOUCH_BUTTON_NUM; i++)
            {
                int touch_diff = touchBuffQueue[i].touchValue - touchBuffQueue[i].baseTouchValue;
                int touch_threshold = touchBuffQueue[i].touchValue * button_threshold[i];
                if ((touch_diff > touch_threshold) && (touchBuffQueue[i].touchValue > touchBuffQueue[i].baseTouchValue))
                {
                    // Button is pressed
                    if (button_states[i] == 0)
                    {
                        button_states[i] = 1;
                        hold_time[i] = esp_log_timestamp();
                        toggle_led_color(leds[i].redPin, leds[i].bluePin, &leds[i].currentColor, &ledstate);
                    }
                    else
                    {
                        // Button is being held
                        if (button_states[i] == 1)
                        {
                            button_states[i] = 2;
                            uint32_t current_time = esp_log_timestamp();
                            uint32_t elapsed_time = current_time - hold_time[i];

                            if (elapsed_time > 100)
                            {

                            }
                            hold_time[i] = current_time;
                        }
                    }
                }
                else
                {
                    // Button is released
                    if (button_states[i] != 0)
                    {
                        button_states[i] = 0;
                    }
                }
            }
        }
        vTaskDelay(50 / portTICK_PERIOD_MS);
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
    //xTaskCreate(&tp_threshold_read_task, "touch_pad_read_task", 4096, NULL, 5, NULL);
    xTaskCreate(&touch_read_raw_task, "touch_pad_read_raw_task", 4096, NULL, 6, NULL);
    xTaskCreate(&update_base_line_task, "update_base_line_task", 4096, NULL, 6, NULL);
    xTaskCreate(&touch_handle_task, "touch_handle_task", 4096, NULL, 6, NULL);
}

/**
 * @func   TOUCH_GetTouchValue
 * @brief  None
 * @param  Byte: Get touch value success or not
 * @retval None
 */
static uint8_t TOUCH_GetTouchValue(void)
{
    uint32_t touch_value;
	uint8_t i;
    for(i = 0; i < TOUCH_BUTTON_NUM; i++){
        touch_pad_read_raw_data(button[i], &touch_value);
        touchBuffQueue[i].touchValue = touch_value;
    }
    if(touch_value != 0)
    {
        return 1;
    }
    return 0;
}

/**
 * @func   TOUCH_ChangeBaseValue
 * @brief  
 * @param  
 * @retval None
 */
static void TOUCH_ChangeBaseValue(uint8_t channel, uint8_t ratio)
{
	uint8_t shitfValue = 0;
	
	switch(ratio){
		case 4:   // 1:4
			shitfValue = 2;
			break;
			
		case 8:   // 1:8
			shitfValue = 3;
			break;
			
		case 16:  // 1:16
			shitfValue = 4;
			break;
			
		default: 
        break;
	}
	
	if(ratio <= 16){
	    touchBuffQueue[channel].baseTouchValue =  \
	    		((--ratio) * touchBuffQueue[channel].baseTouchValue +    \
	    				touchBuffQueue[channel].touchValue) >> shitfValue;
	}
}

/**
 * @func   TOUCH_CalibBaseLine
 * @brief  This function measures the change in capacitance of each element
 *         within a sensor and updates the baseline tracking in the event that
 *         no change exceeds the detection threshold.
 * @param  
 * @retval None
 */

static void TOUCH_CalibBaseLine(void)
{
	uint8_t i;

	for(i = 0; i < TOUCH_BUTTON_NUM; i++){
		if(touchBuffQueue[i].touchValue 
				< (uint16_t)touchBuffQueue[i].baseTouchValue){
			countTimeSmallUp[i]  = 0;
			countTimeMediumUp[i] = 0;
			countTimeHighUp[i] = 0;
			if((touchBuffQueue[i].baseTouchValue 
					- touchBuffQueue[i].touchValue) <= THRESHOLD_DETECT_SMALL_NOISE) {
				countTimeSmallDown[i]++;
				countTimeMediumDown[i] = 0;
				if(countTimeSmallDown[i] >= TIMES_OF_SMALL_DOWN){
					countTimeSmallDown[i] = 0;
					TOUCH_ChangeBaseValue(i,4);  // 1:4
				}		
			}
			else {
				countTimeSmallDown[i] = 0;
				countTimeMediumDown[i]++;
				if(countTimeMediumDown[i] >= TIMES_OF_MEDIUM_DOWN){
					countTimeMediumDown[i] = 0;
					TOUCH_ChangeBaseValue(i,8);  // 1:8
				}
			}
		}
		else{
			countTimeSmallDown[i] = 0;
			countTimeMediumDown[i] = 0;
			if((touchBuffQueue[i].touchValue 
					- touchBuffQueue[i].baseTouchValue) <= THRESHOLD_DETECT_SMALL_NOISE ) {
				countTimeSmallUp[i]++;
				countTimeMediumUp[i] = 0;
				countTimeHighUp[i] = 0;
				if(countTimeSmallUp[i] >= TIMES_OF_SMALL_UP){
					countTimeSmallUp[i] = 0;
					TOUCH_ChangeBaseValue(i,4);  //1:4
				}
			}else if((touchBuffQueue[i].touchValue 
					- touchBuffQueue[i].baseTouchValue) <= THRESHOLD_DETECT_MEDIUM_NOISE) {
				countTimeSmallUp[i] = 0;
				countTimeMediumUp[i]++;
				countTimeHighUp[i] = 0;
				if(countTimeMediumUp[i] >= TIMES_OF_MEDIUM_UP){
					countTimeMediumUp[i] = 0;
					TOUCH_ChangeBaseValue(i,8);  // 1:8
				}
			}else if(((touchBuffQueue[i].touchValue 
					- touchBuffQueue[i].baseTouchValue) > THRESHOLD_DETECT_MEDIUM_NOISE)
							&&((touchBuffQueue[i].touchValue - touchBuffQueue[i].baseTouchValue) 
									<= THRESHOLD_DETECT_BIG_NOISE)){
				countTimeSmallUp[i] = 0;
				countTimeMediumUp[i] = 0;
				countTimeHighUp[i]++;
				if(countTimeHighUp[i] >= TIMES_OF_HIGH_UP){
					countTimeHighUp[i] = 0;
					TOUCH_ChangeBaseValue(i,16); // 1:16
				}
			}
		}	
	}
}

/**
 * @func   TOUCH_UpdateBase
 * @brief  Update Base Line for all touch pad
 * @param  None
 * @retval BYTE: Update Base Status
 */
static uint8_t TOUCH_UpdateBaseLine(void)
{
	static uint8_t updateBaseCnt = 0;
	static uint8_t countTimeUpdateBase = 0;
	uint8_t i;

	if(TOUCH_GetTouchValue()){
		
		if(updateBaseCnt > 0) {
			updateBaseCnt++;
			for(i = 0; i < TOUCH_BUTTON_NUM; i++) {
				touchBuffQueue[i].baseTouchValue = (touchBuffQueue[i].baseTouchValue 
						+ touchBuffQueue[i].touchValue) >> 1;
			}
		}
		else {
			for(i = 0; i < TOUCH_BUTTON_NUM; i++) {
				touchBuffQueue[i].baseTouchValue = touchBuffQueue[i].touchValue;
			}
			updateBaseCnt++;
		}
		if(updateBaseCnt >= TIME_SCAN_ONE_CHANNEL_IN_UPDATE_BASE_MODE) {
			updateBaseCnt = 0;
			countTimeUpdateBase++;
			if(countTimeUpdateBase == 0xFF) {
				countTimeUpdateBase = 1;
			}
			return UPDATE_BASE_SUCCESS;
		}
		else {
		    return UPDATE_BASE_CHANGE_STEP;
		}
	}
	return UPDATE_BASE_IN_PROGRESS;
}

/**
 * @func   TOUCH_MainProcessFuntion
 * @brief  Main function for touch module
 *         includes modes: Normal - Scan all channel, detect power noise, scan one channel
 *                         Update base mode. 
 * @param  None
 * @retval None
 */
void TOUCH_MainProcessFuntion(uint8_t enableUpdateBase)
{	
	switch(touchModuleMode) {
	case NORMAL_MODE:
        if(enableUpdateBase == ENABLE_UPDATE_BASE) {
			TOUCH_CalibBaseLine();
		}
        touchModuleMode = UPDATE_BASE_MODE;
    break;        

		
	case UPDATE_BASE_MODE:
		switch(TOUCH_UpdateBaseLine()) {
		case UPDATE_BASE_CHANGE_STEP:
			//TOUCH_EnableTsiModuleAutoRun(TOUCH_SCAN_ALL_CHANEL, 0, NUMBER_PAD);
			break;
			
		case UPDATE_BASE_IN_PROGRESS:
			break;
			
		case UPDATE_BASE_SUCCESS:
			touchModuleMode = NORMAL_MODE;
			//TOUCH_EnableTsiModuleAutoRun(TOUCH_SCAN_ALL_CHANEL, 0, NUMBER_PAD);
		    break;
		    
		default:
			
			break;
		}
	}
}
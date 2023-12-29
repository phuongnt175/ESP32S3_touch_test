#include <stdio.h>

#define TOUCH_BUTTON_NUM    4
#define TOUCH_CHANGE_CONFIG 0

#define LED_RED_1 GPIO_NUM_41
#define LED_RED_2 GPIO_NUM_47
#define LED_RED_3 GPIO_NUM_11
#define LED_RED_4 GPIO_NUM_9

#define LED_BLUE_1 GPIO_NUM_42
#define LED_BLUE_2 GPIO_NUM_48
#define LED_BLUE_3 GPIO_NUM_10
#define LED_BLUE_4 GPIO_NUM_20

#define RL_1 GPIO_NUM_16
#define RL_2 GPIO_NUM_18
#define RL_3 GPIO_NUM_8
#define RL_4 GPIO_NUM_19

#define TOUCH_PIN_1  GPIO_NUM_14
#define TOUCH_PIN_2  GPIO_NUM_2
#define TOUCH_PIN_3  GPIO_NUM_13
#define TOUCH_PIN_4  GPIO_NUM_5

typedef struct touch_msg {
    touch_pad_intr_mask_t intr_mask;
    uint32_t pad_num;
    uint32_t pad_status;
    uint32_t pad_val;
} touch_event_t;

#define TOUCH_BUTTON_NUM    4
#define TOUCH_BUTTON_DENOISE_ENABLE    1
#define TOUCH_CHANGE_CONFIG            0

static const touch_pad_t button[TOUCH_BUTTON_NUM] = {
    TOUCH_PAD_NUM14,     
    TOUCH_PAD_NUM2,     
    TOUCH_PAD_NUM13,   
    TOUCH_PAD_NUM5,
    // If this pad be touched, other pads no response.
};

/*
 * Touch threshold. The threshold determines the sensitivity of the touch.
 * This threshold is derived by testing changes in readings from different touch channels.
 * If (raw_data - benchmark) > benchmark * threshold, the pad be activated.
 * If (raw_data - benchmark) < benchmark * threshold, the pad be inactivated.
 */
static const float button_threshold[TOUCH_BUTTON_NUM] = {
    0.005, // 20%.  0,5%
    0.005, // 20%.  0,5%
    0.005, // 20%.  0,5%
    0.005, // 10%.  0,5%
};
// Define LED color enum
typedef enum {
    LED_RED,
    LED_BLUE
} LedColor;

typedef enum {
    BUTTON_RELEASED,
    BUTTON_PRESSED,
    BUTTON_HOLD,
} ButtonState;

// Define LED structure
typedef struct {
    gpio_num_t redPin;
    gpio_num_t bluePin;
    LedColor currentColor;
} Led;

// Define button structure
typedef struct {
    ButtonState buttonState;
    uint32_t lastTouchValue;
} TouchButton;

// Create instances for LEDs
Led leds[TOUCH_BUTTON_NUM] = {
    {LED_RED_1, LED_BLUE_1, LED_RED},
    {LED_RED_2, LED_BLUE_2, LED_RED},
    {LED_RED_3, LED_BLUE_3, LED_RED},
    {LED_RED_4, LED_BLUE_4, LED_RED}
};

// Create instances for touch buttons
TouchButton buttons[TOUCH_BUTTON_NUM];

/**
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
 ******************************************************************************
 ** This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether 
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * COPYRIGHT(c) 2017 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l0xx_hal.h"

#include <math.h>
#include <string.h>

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

I2C_HandleTypeDef hi2c1;

TSC_HandleTypeDef htsc;

UART_HandleTypeDef huart1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_I2C1_Init(void);
static void MX_TSC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

#define RED_CHANNEL TIM_CHANNEL_2
#define GREEN_CHANNEL TIM_CHANNEL_3
#define BLUE_CHANNEL TIM_CHANNEL_4

// 'Channel' is one of the above options
// 'Value' is between 0-255
void set_led_brightness(uint32_t channel, uint8_t value) {
      TIM_OC_InitTypeDef sConfigOC;
      sConfigOC.OCMode = TIM_OCMODE_PWM1;
      sConfigOC.Pulse = value*4;
      sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW; // Reverse polarity here so high values=bright
      sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
      HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, channel);
      HAL_TIM_PWM_Start(&htim2, channel);
}

#define TOUCH_SIZE_X 4
#define TOUCH_SIZE_Y 3

typedef struct _touch_pad {
    uint32_t channel_io;
    uint32_t sample_io;
    uint32_t sample_group;
    int32_t calibration_no_touch;
    int32_t calibration_touch;
} touch_pad_t;

touch_pad_t touch_matrix[TOUCH_SIZE_X + TOUCH_SIZE_Y] = {
    // X PADS
    {TSC_GROUP1_IO2, TSC_GROUP1_IO1, TSC_GROUP1_IDX, 265, 35}, // 2 diamonds closest to chip (x1)
    {TSC_GROUP1_IO3, TSC_GROUP1_IO1, TSC_GROUP1_IDX, 249, 37}, // (x2)
    {TSC_GROUP1_IO4, TSC_GROUP1_IO1, TSC_GROUP1_IDX, 265, 39}, // (x3)
    {TSC_GROUP2_IO2, TSC_GROUP2_IO1, TSC_GROUP2_IDX, 231, 45}, // 2 diamonds closest to end (x4)
    // Y PADS
    {TSC_GROUP2_IO3, TSC_GROUP2_IO1, TSC_GROUP2_IDX, 284, 45}, // Y, LED side (y1)
    {TSC_GROUP2_IO4, TSC_GROUP2_IO1, TSC_GROUP2_IDX, 226, 34}, // (y2)
    {TSC_GROUP4_IO2, TSC_GROUP4_IO1, TSC_GROUP4_IDX, 290, 47}, // Y, Switch side (y3)
};

#define SAMPLE_X 0
#define SAMPLE_Y 1

#define COMMAND_START       0xFD
#define COMMAND_MOUSE_TYPE  0x02
#define COMMAND_CONSUMER_TYPE 0x03

// Index is 0 for first pad at that index
// Return in range 1 to 100
int sample_touch_at (int index, int what_to_sample) {

    if(what_to_sample == SAMPLE_X) {
        if(index >= TOUCH_SIZE_X) {
            printf("Trying to sample out-of-range X touch pad");
            return 0;
        }
    } else {
        if(index >= TOUCH_SIZE_Y) {
            printf("Trying to sample out-of-range Y touch pad");
            return 0;
        }
        index += 4;
    }


    TSC_IOConfigTypeDef touch_conf;
    touch_conf.ShieldIOs = 0;
    touch_conf.ChannelIOs = touch_matrix[index].channel_io;
    touch_conf.SamplingIOs = touch_matrix[index].sample_io;

    HAL_TSC_IOConfig(&htsc, &touch_conf);

    HAL_TSC_IODischarge(&htsc, ENABLE);

    HAL_Delay(1); // Wait for everything to discharge

    if(HAL_TSC_Start(&htsc) != HAL_OK) {
        printf("Error in HAL_TSC_Start");
    }

    while(HAL_TSC_GetState(&htsc) == HAL_TSC_STATE_BUSY) {
        // Wait for the reading to complete
    }

    __HAL_TSC_CLEAR_FLAG(&htsc, (TSC_FLAG_EOA | TSC_FLAG_MCE));

    if( HAL_TSC_GroupGetStatus(&htsc, touch_matrix[index].sample_group)
            == TSC_GROUP_COMPLETED) {
        int v = HAL_TSC_GroupGetValue(&htsc, touch_matrix[index].sample_group);
        // Scale from 1 - 100
        v = touch_matrix[index].calibration_no_touch - v;
        int w = touch_matrix[index].calibration_no_touch - touch_matrix[index].calibration_touch;
        v = (100*v) / w;
        if( v > 100 ) v = 100;
        if( v < 1 ) v = 1;
        return v;
    }

    printf("Touch read didn't complete?\n");

    return -1;
}

// Takes an integer from 0 to 512 (should represent 000 to 5.12)
// Returns a value from 1 to 99 (1 corresponds to 0 degrees, 99 corresponds to 90 degrees)
// The output is scaled as that's what the interpolation needs
int fast_dodgy_atan(int x) {

    static const uint8_t atan_lut[512] =
        {1,1,2,3,4,4,5,6,6,7,8,9,9,10,11,11,12,13,14,14,15,16,16,17,18,18,19,20,20,21,22,22,
         23,24,24,25,25,26,27,27,28,29,29,30,30,31,32,32,33,33,34,34,35,36,36,37,37,38,38,39,
         39,40,40,41,41,42,42,43,43,44,44,45,45,46,46,47,47,48,48,49,49,49,50,50,51,51,52,52,
         52,53,53,54,54,54,55,55,55,56,56,56,57,57,58,58,58,59,59,59,60,60,60,61,61,61,62,62,
         62,62,63,63,63,64,64,64,64,65,65,65,66,66,66,66,67,67,67,67,68,68,68,68,69,69,69,69,
         70,70,70,70,70,71,71,71,71,72,72,72,72,72,73,73,73,73,73,74,74,74,74,74,75,75,75,75,
         75,75,76,76,76,76,76,77,77,77,77,77,77,78,78,78,78,78,78,78,79,79,79,79,79,79,80,80,
         80,80,80,80,80,80,81,81,81,81,81,81,81,82,82,82,82,82,82,82,82,83,83,83,83,83,83,83,
         83,83,84,84,84,84,84,84,84,84,84,85,85,85,85,85,85,85,85,85,86,86,86,86,86,86,86,86,
         86,86,86,87,87,87,87,87,87,87,87,87,87,87,88,88,88,88,88,88,88,88,88,88,88,88,89,89,
         89,89,89,89,89,89,89,89,89,89,89,90,90,90,90,90,90,90,90,90,90,90,90,90,90,91,91,91,
         91,91,91,91,91,91,91,91,91,91,91,91,92,92,92,92,92,92,92,92,92,92,92,92,92,92,92,92,
         92,93,93,93,93,93,93,93,93,93,93,93,93,93,93,93,93,93,93,94,94,94,94,94,94,94,94,94,
         94,94,94,94,94,94,94,94,94,94,94,94,95,95,95,95,95,95,95,95,95,95,95,95,95,95,95,95,
         95,95,95,95,95,95,96,96,96,96,96,96,96,96,96,96,96,96,96,96,96,96,96,96,96,96,96,96,
         96,96,96,96,97,97,97,97,97,97,97,97,97,97,97,97,97,97,97,97,97,97,97,97,97,97,97,97,
         97,97,97,97,98,98,98,98,98,98,98,98,98,98,98,98,98,98,98,98,98,98,98,98,98,98,98,98,
         98,98,98,98,98,98,98,98,98,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,99,
         99,99,99,99};

    if( x > 511 ) x = 511;
    if( x < 0 ) x = 0;

    return atan_lut[x];
}

typedef struct _interpolation_result {
    int position;
    int pressure;
} interpolation_result_t;

// Takes individual pressure values
// Translates into a position (from 0 to 100) and a touch pressure
// (Note the position may not be especially meaningful for low pressures!)
interpolation_result_t interpolate_touch_values_x(int *values) {
    int x_total = 0, y_total = 0;

    // Create a vector sum, such that final vector angle will
    // correspond to the interpolated position

    x_total += values[0]*100;
    // sin(0)=0
    x_total += values[1]*87;
    y_total += values[1]*50;
    x_total += values[2]*50;
    y_total += values[2]*87;
    y_total += values[3]*100;
    // cos(90)=0

    interpolation_result_t result;
    result.position = fast_dodgy_atan((100*y_total)/x_total);

    result.pressure = x_total*x_total + y_total*y_total;

    return result;
}

interpolation_result_t interpolate_touch_values_y(int *values) {
    int x_total = 0, y_total = 0;

    x_total += values[0]*100;
    // sin(0)=0
    x_total += values[1]*71;
    y_total += values[1]*71;
    y_total += values[2]*100;
    // cos(90)=0

    interpolation_result_t result;
    result.position = fast_dodgy_atan((100*y_total)/x_total);
    result.pressure = x_total*x_total + y_total*y_total;

    return result;
}

typedef struct _average_result {
    int x;
    int y;
} average_result_t;

#define N_AV 5

average_result_t average_deltas(int x_now, int y_now, int justPressed) {


    static int last_x = 0;
    static int last_y = 0;

    static int averages_x[N_AV];
    static int averages_y[N_AV];

    static int index_x = 0;
    static int index_y = 0;

    int8_t dx = x_now - last_x;
    int8_t dy = y_now - last_y;

    averages_x[index_x] = dx;
    averages_y[index_y] = dy;

    ++index_x;
    ++index_y;
    if(index_x >= N_AV) index_x = 0;
    if(index_y >= N_AV) index_y = 0;

    // Create a rolling average
    int sum_x = 0;
    int sum_y = 0;
    for(int i = 0; i != N_AV; ++i) {
        sum_x += averages_x[i];
        sum_y += averages_y[i];
    }

    // Quench instantaneous movement on a press
    if(justPressed) {
        for(int i = 0; i != N_AV; ++i) {
            averages_x[i] = 0;
            averages_y[i] = 0;
        }
        sum_x = sum_y = 0;
    }

    last_x = x_now;
    last_y = y_now;

    average_result_t result = {sum_x, sum_y};

    return result;
}

typedef struct _press_result {
    int just_pressed;
    int just_released;
    int just_tapped;
    int currently_pressed;
} press_result_t;

press_result_t track_presses(int current_pressure) {

    static int previously_pressed = 0;
    static int ticks_when_pressed = 0;

    press_result_t result = {0, 0, 0, 0};

    result.currently_pressed = current_pressure > 4000;

    if(previously_pressed == 0 && current_pressure > 4000) {
        result.just_pressed = 1;
        ticks_when_pressed = HAL_GetTick();
    }

    // Bit of hysteresis here
    if(previously_pressed == 1 && current_pressure < 2000) {
        result.just_released = 1;
    }

    if(result.just_released && HAL_GetTick() - ticks_when_pressed < 100) {
        result.just_tapped = 1;
    }

    previously_pressed = (current_pressure > 4000);

    return result;
}

#define MOUSE_BUTTON_LEFT   (uint8_t)1
#define MOUSE_BUTTON_RIGHT  (uint8_t)(1 << 1)

void issue_hid_mouse_command(int8_t x_move, int8_t y_move, int8_t scroll, uint8_t buttons) {

    /*
     * BUTTONS = bitmask
     *  LSB = LEFT CLICK (LATCHES TO PRESSED)
     *  2nd to LSB = RIGHT CLICK (LATCHES TO PRESSED)
     * x_move is 8-bit 2-s complement relative movement
     * y_move is 8-bit 2-s complement relative movement
     * scroll is 8-bit 2-s complement relative movement
     */

    uint8_t hid_command[] = {
        COMMAND_START,
        5, // Rest of command length
        COMMAND_MOUSE_TYPE,
        (uint8_t) buttons,
        (uint8_t) x_move,
        (uint8_t) y_move,
        (uint8_t) scroll,
    };

    // Assumes total command length of 7
    HAL_UART_Transmit(&huart1, (uint8_t*)&hid_command, sizeof(hid_command), 100);
}

#define VOLUME_UP_LOW_BYTE 0x10
#define VOLUME_UP_HIGH_BYTE 0x00
#define VOLUME_DOWN_LOW_BYTE 0x20
#define VOLUME_DOWN_HIGH_BYTE 0x00
#define NEXT_LOW_BYTE 0x00
#define NEXT_HIGH_BYTE 0x01
#define PREV_LOW_BYTE 0x00
#define PREV_HIGH_BYTE 0x02
#define PLAY_PAUSE_LOW_BYTE 0x80
#define PLAY_PAUSE_HIGH_BYTE 0x00
#define RELEASE 0x00

void issue_hid_consumer_report(int8_t command_low_byte, int8_t command_high_byte) {

	uint8_t hid_command[] = {
		COMMAND_START,
		3, // Length of command
		COMMAND_CONSUMER_TYPE,
		(uint8_t) command_low_byte,
		(uint8_t) command_high_byte,
	};
	
	HAL_UART_Transmit(&huart1, (uint8_t*)&hid_command, sizeof(hid_command), 100);
		
}

void move_and_tap_mouse(average_result_t deltas, press_result_t presses) {

    int mouse_dx = -deltas.y/2;
    int mouse_dy = -deltas.x/2;
    int buttons = 0;

    // Ignore the movement if it's probably bogus
    if(presses.just_pressed || !presses.currently_pressed) {
        mouse_dx = mouse_dy = 0;
    }

    if(presses.just_tapped) {
        buttons = MOUSE_BUTTON_LEFT;
    }

    issue_hid_mouse_command( mouse_dx, mouse_dy, 0, buttons);
}

void scroll_mouse(average_result_t deltas, press_result_t presses) {

    int scroll_delta = -deltas.x/15;

    // Ignore the movement if it's probably bogus
    if(presses.just_pressed || !presses.currently_pressed) {
        scroll_delta = 0;
    }

    issue_hid_mouse_command( 0, 0, scroll_delta, 0);
}


void media_control(average_result_t deltas, press_result_t presses) {

	int volume_control = deltas.x/15;
	//int skip_control = deltas.y/15;

	uint8_t command_low_byte = RELEASE;
	uint8_t command_high_byte = RELEASE;
	
	// Ignore the movement if it's probably bogus
    if(presses.just_pressed || !presses.currently_pressed) {
        volume_control = 0;
        //skip_control = 0;
    }
    
	// Key must be released after press
	
	// Control volume
	if(volume_control > 0) {
		command_low_byte = VOLUME_UP_LOW_BYTE;
		command_high_byte = VOLUME_UP_HIGH_BYTE;
	} else if(volume_control < 0) {
		command_low_byte = VOLUME_DOWN_LOW_BYTE;
		command_high_byte = VOLUME_DOWN_HIGH_BYTE;
	}
	
	/*// Control skips (NOT YET WORKING)
	if(skip_control > 0 && !presses.just_released) {
		issue_hid_consumer_report(NEXT_LOW_BYTE, NEXT_HIGH_BYTE);
		issue_hid_consumer_report(RELEASE, RELEASE);
	} else if(skip_control < 0 && !presses.just_released) {
		issue_hid_consumer_report(PREV_LOW_BYTE, PREV_HIGH_BYTE);
		issue_hid_consumer_report(RELEASE, RELEASE);
	}*/
	
	// Control play/pause
	if(presses.just_tapped) {
		command_low_byte = PLAY_PAUSE_LOW_BYTE;
		command_high_byte = PLAY_PAUSE_HIGH_BYTE;
	}
	
	issue_hid_consumer_report(command_low_byte, command_high_byte);
	issue_hid_consumer_report(RELEASE, RELEASE);
	
}

void bt_send_cmd(char *s) {
    HAL_UART_Transmit(&huart1, (unsigned char*)s, strlen(s), 100);
    HAL_Delay(250);
}

void bt_send(char *s) {
    HAL_UART_Transmit(&huart1, (unsigned char*)s, strlen(s), 100);
}

void program_rn42_module() {
    printf("**PROGRAMMING RN-42 MODULE**\n");

    HAL_Delay(500);

    bt_send_cmd("$$$");
    bt_send_cmd("SF,1\r");
    bt_send_cmd("S-,SmartCutlery\r");
    bt_send_cmd("SN,SmartCutlery\r");
    bt_send_cmd("SS,Keyboard/Mouse\r");
    bt_send_cmd("SM,4\r");
    bt_send_cmd("S~,6\r");
    bt_send_cmd("SH,003C\r");
    bt_send_cmd("SY,0001\r");
    bt_send_cmd("SW,8000\r");
    bt_send_cmd("SI,0200\r");
    bt_send_cmd("SJ,0200\r");
    bt_send_cmd("R,1\r");

    printf("**FINISHED PROGRAMMING RN-42 MODULE**\n");
}

typedef struct _touch {
    int x;
    int y;
    int pressure;
    int ignore;
} touch_t;

// Any pad > this will contribute to the 'ignore' count
// Board dependent quantity:
// Sebastian's Version == 4
// Jamon's Version == 7

#define IGNORE_MIN_PRESSURE 7

// > N pads will force an ignore
#define IGNORE_PADS 6

touch_t sample_touch() {

    int x1 = sample_touch_at(0, SAMPLE_X);
    int x2 = sample_touch_at(1, SAMPLE_X);
    int x3 = sample_touch_at(2, SAMPLE_X);
    int x4 = sample_touch_at(3, SAMPLE_X);

    int y1 = sample_touch_at(0, SAMPLE_Y);
    int y2 = sample_touch_at(1, SAMPLE_Y);
    int y3 = sample_touch_at(2, SAMPLE_Y);

    int ignore_count =
        (x1 > IGNORE_MIN_PRESSURE) +
        (x2 > IGNORE_MIN_PRESSURE) +
        (x3 > IGNORE_MIN_PRESSURE) +
        (x4 > IGNORE_MIN_PRESSURE) +
        (y1 > IGNORE_MIN_PRESSURE) +
        (y2 > IGNORE_MIN_PRESSURE) +
        (y3 > IGNORE_MIN_PRESSURE);

    if( ignore_count > IGNORE_PADS ) {
        touch_t result;
        result.x = 50;
        result.y = 50;
        result.pressure = 0;
        result.ignore = 1;
        return result;
    }

    int values_x[4] = {x1, x2, x3, x4};
    int values_y[3] = {y1, y2, y3};

    interpolation_result_t interp_x = interpolate_touch_values_x(values_x);
    interpolation_result_t interp_y = interpolate_touch_values_y(values_y);

    int pressure_final = (interp_x.pressure + interp_y.pressure)/10000;

    touch_t result;
    result.x = interp_x.position;
    result.y = interp_y.position;
    result.pressure = pressure_final;
    result.ignore = 0;

    return result;
}

void rgb_cycle() {
    set_led_brightness(RED_CHANNEL, 100);
    set_led_brightness(BLUE_CHANNEL, 0);
    set_led_brightness(GREEN_CHANNEL, 0);
    HAL_Delay(50);
    set_led_brightness(RED_CHANNEL, 0);
    set_led_brightness(BLUE_CHANNEL, 100);
    set_led_brightness(GREEN_CHANNEL, 0);
    HAL_Delay(50);
    set_led_brightness(RED_CHANNEL, 0);
    set_led_brightness(BLUE_CHANNEL, 0);
    set_led_brightness(GREEN_CHANNEL, 100);
    HAL_Delay(50);
    set_led_brightness(RED_CHANNEL, 0);
    set_led_brightness(BLUE_CHANNEL, 0);
    set_led_brightness(GREEN_CHANNEL, 0);
}

#define PWROFF_PUSHTIME_MS 1000
#define PWROFF_INACTIVE_TIME_MS 60000
#define MODESWITCH_PUSHTIME_MS 100

enum modes {
    MODE_MOUSE = 0,
    MODE_SCROLL,
    MODE_MEDIA,
    // Insert extra modes here
    MODE_N
};

int main(void)
{

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_ADC_Init();
    MX_I2C1_Init();
    MX_TSC_Init();
    MX_USART1_UART_Init();
    MX_TIM2_Init();

    // Latch power on
    HAL_GPIO_WritePin(PWR_LATCH_GPIO_Port, PWR_LATCH_Pin, GPIO_PIN_SET);

    //program_rn42_module();

    // Cycle LEDs to show the world we are now alive
    for( int i = 0; i != 3; ++i ) {
        rgb_cycle();
    }

    printf("Starting main loop\n");

    rgb_cycle();

    // Boot time in systicks
    int tickStart = HAL_GetTick();

    int current_mode = MODE_MEDIA;

    int lastButtonUnpressedTicks = tickStart;

    int lastTouchTicks = tickStart;

    while(1) {

        touch_t touch = sample_touch();

        printf("X=%03d Y=%03d P=%05d\n", touch.x, touch.y, touch.pressure);

        press_result_t presses = track_presses(touch.pressure);

        average_result_t deltas = average_deltas(touch.x, touch.y, presses.just_pressed);

        int target_brightness = touch.ignore ? 20 : 100;

        switch(current_mode) {
            case MODE_MOUSE:
                set_led_brightness(RED_CHANNEL, target_brightness);
                move_and_tap_mouse(deltas, presses);
            break;
            case MODE_SCROLL:
                set_led_brightness(GREEN_CHANNEL, target_brightness);
                scroll_mouse(deltas, presses);
            break;
            case MODE_MEDIA:
            	set_led_brightness(BLUE_CHANNEL, target_brightness);
            	media_control(deltas, presses);
            break;
        }

        // If power button is held down for longer than specific time
        // turn the device off. If it is pressed for a short time, change modes.
        if(HAL_GPIO_ReadPin(PWR_BUTTON_GPIO_Port, PWR_BUTTON_Pin)) {
            if(HAL_GetTick() - lastButtonUnpressedTicks > PWROFF_PUSHTIME_MS) {
                break;
            }
        } else {
            if(HAL_GetTick() - lastButtonUnpressedTicks > MODESWITCH_PUSHTIME_MS) {
                ++current_mode;
                if(current_mode >= MODE_N) {
                    current_mode = 0;
                }
                rgb_cycle();
            }
            lastButtonUnpressedTicks = HAL_GetTick();
        }

        // Resting thumb or press keeps the thing alive
        if(touch.ignore || presses.currently_pressed) {
            lastTouchTicks = HAL_GetTick();
        }

        // If we haven't touched the device in a while,
        // power it off automatically
        if(HAL_GetTick() - lastTouchTicks >= PWROFF_INACTIVE_TIME_MS) {
            break;
        }

    }

    // Flash red to indicate shutting down

    set_led_brightness(RED_CHANNEL, 0);
    set_led_brightness(GREEN_CHANNEL, 0);
    set_led_brightness(BLUE_CHANNEL, 0);

    set_led_brightness(RED_CHANNEL, 150);
    HAL_Delay(100);
    set_led_brightness(RED_CHANNEL, 0);
    HAL_Delay(100);
    set_led_brightness(RED_CHANNEL, 150);
    HAL_Delay(100);

    // Leave red LED on for possible fading effect

    HAL_GPIO_WritePin(PWR_LATCH_GPIO_Port, PWR_LATCH_Pin, GPIO_PIN_RESET);
    // Power is cut here ^
    HAL_Delay(1000);

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Configure the main internal regulator output voltage 
    */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
    RCC_OscInitStruct.MSIState = RCC_MSI_ON;
    RCC_OscInitStruct.MSICalibrationValue = 0;
    RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
        |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
    PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
    PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    /**Configure the Systick interrupt time 
    */
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /* SysTick_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC init function */
static void MX_ADC_Init(void)
{

    ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
    hadc.Instance = ADC1;
    hadc.Init.OversamplingMode = DISABLE;
    hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
    hadc.Init.Resolution = ADC_RESOLUTION_12B;
    hadc.Init.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
    hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
    hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc.Init.ContinuousConvMode = DISABLE;
    hadc.Init.DiscontinuousConvMode = DISABLE;
    hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc.Init.DMAContinuousRequests = DISABLE;
    hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
    hadc.Init.LowPowerAutoWait = DISABLE;
    hadc.Init.LowPowerFrequencyMode = DISABLE;
    hadc.Init.LowPowerAutoPowerOff = DISABLE;
    if (HAL_ADC_Init(&hadc) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    /**Configure for the selected ADC regular channel to be converted. 
    */
    sConfig.Channel = ADC_CHANNEL_8;
    sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
    if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    /**Configure for the selected ADC regular channel to be converted. 
    */
    sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
    if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

    hi2c1.Instance = I2C1;
    hi2c1.Init.Timing = 0x00000708;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    /**Configure Analogue filter 
    */
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    /**Configure Digital filter 
    */
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1024;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim2);

}

/* TSC init function */
static void MX_TSC_Init(void)
{

    /**Configure the TSC peripheral 
    */
    htsc.Instance = TSC;
    htsc.Init.CTPulseHighLength = TSC_CTPH_2CYCLES;
    htsc.Init.CTPulseLowLength = TSC_CTPL_2CYCLES;
    htsc.Init.SpreadSpectrum = ENABLE;
    htsc.Init.SpreadSpectrumDeviation = 1;
    htsc.Init.SpreadSpectrumPrescaler = TSC_SS_PRESC_DIV1;
    htsc.Init.PulseGeneratorPrescaler = TSC_PG_PRESC_DIV2;
    htsc.Init.MaxCountValue = TSC_MCV_511;
    htsc.Init.IODefaultMode = TSC_IODEF_OUT_PP_LOW;
    htsc.Init.SynchroPinPolarity = TSC_SYNC_POLARITY_FALLING;
    htsc.Init.AcquisitionMode = TSC_ACQ_MODE_NORMAL;
    htsc.Init.MaxCountInterrupt = DISABLE;
    htsc.Init.ChannelIOs = TSC_GROUP1_IO2;
    htsc.Init.SamplingIOs = TSC_GROUP1_IO1;
    if (HAL_TSC_Init(&htsc) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(PWR_LATCH_GPIO_Port, PWR_LATCH_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin : PWR_BUTTON_Pin */
    GPIO_InitStruct.Pin = PWR_BUTTON_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(PWR_BUTTON_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : PWR_LATCH_Pin */
    GPIO_InitStruct.Pin = PWR_LATCH_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(PWR_LATCH_GPIO_Port, &GPIO_InitStruct);
}


/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void _Error_Handler(char * file, int line)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    while(1) 
    {
    }
    /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */

}

#endif

/**
 * @}
 */ 

/**
 * @}
 */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

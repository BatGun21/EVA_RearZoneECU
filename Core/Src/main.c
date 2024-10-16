/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ws28xx.h"
#include "math.h"
#include "stdlib.h"
#include "stdbool.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    OFF_STATE,
    STARTUP_MODE,
    POSITION_LIGHT_MODE,  // Marker Light
    BRAKE_LIGHT_MODE,
    TURN_SIGNAL_MODE,     // Turn Signal (Amber wave)
    HAZARD_MODE,           // Hazard (Amber blink)
	REVERSE_LIGHT_MODE    // White Light
} LED_State;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define Clock_Frequency 36000 //KHz

// Middle strip definition
#define MIDDLE_LED_COUNT 144
#define TOP_RIGHT_LED_COUNT 46
#define TOP_LEFT_LED_COUNT 46
#define LEFT_BOTTOM_LED_COUNT 46
#define RIGHT_BOTTOM_LED_COUNT 46
#define NUMBER_PLATE_LED_COUNT 28
#define HMSL_LED_COUNT 28
#define BRAKE_BRIGHTNESS 255
#define REVERSE_LIGHT_BRIGHTNESS 255
#define NUMBER_PLATE_BRIGHTNESS 255
#define POSITION_BRIGHTNESS 100
#define STARTUP_WAVE_SPEED 20
#define WAVE_PACKET_SIZE_MIDDLE 3
#define MIDDLE_LED_MID_INDEX (MIDDLE_LED_COUNT / 2)

#define WAVE_PACKET_SIZE 7
#define WAVE_SPEED 5 // Increase number to reduce speed
#define WAVE_STEP_SIZE 1
#define WAVE_PAUSE 10

#define HAZARD_BLINK_DELAY 500
#define DRL_BRIGHTNESS 255 // Max 255
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
WS28XX_HandleTypeDef ws_pa9;  // Middle LED Strip
WS28XX_HandleTypeDef ws_pa8;  // Left Top LED Strip
WS28XX_HandleTypeDef ws_pa10; // Right Top LED Strip
WS28XX_HandleTypeDef ws_pa0;  // Left Bottom LED Strip
WS28XX_HandleTypeDef ws_pa2;  // Right Bottom LED Strip
WS28XX_HandleTypeDef ws_pa1;  // Number Plate Light
WS28XX_HandleTypeDef ws_pa11; // HMSL
LED_State current_state_pa9 = OFF_STATE;
LED_State current_state_pa8 = OFF_STATE;
LED_State current_state_pa10 = OFF_STATE;

//Control flags
int brake_signal_received = 0;     // 1 if brake signal is received, 0 otherwise
int headlamp_signal_received = 1;  // 1 if headlamp is on, 0 otherwise
int startup_signal_received = 0;   // Initially active for startup mode
int turn_signal_left_received = 0;
int turn_signal_right_received = 0;
int hazard_signal_received = 0;
int reverse_signal_received = 0;

// Timing variables for wave animation
static int wave_count_middle = 0;
static int frame_left = MIDDLE_LED_MID_INDEX;
static int frame_right = MIDDLE_LED_MID_INDEX;
static uint8_t wave_direction = 0;  // 0 = outward, 1 = inward
static int last_update_time = 0;
static int sequential_turn_on = 0;
int frame_pa8 = 0;   // Left Top Strip (PA8)
int frame_pa10 = 0;  // Right Top Strip (PA10)

// Flickering prevention flags
int is_position_light_displayed = 0;
int is_brake_light_displayed = 0;
int is_position_light_displayed_left = 0;  // Left Top Strip (PA8)
int is_position_light_displayed_right = 0;  // Right Top Strip (PA10)
int is_reverse_light_displayed_left = 0;
int is_reverse_light_displayed_right = 0;
int is_number_plate_light_displayed = 0;
int is_hmsl_light_displayed = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void HandleMiddleStripState(void);
void HandleLeftTopStripState(void);
void HandleRightTopStripState(void);
void UpdateTurnSignalWave(WS28XX_HandleTypeDef* ws, int frame, int pixel_count);
void UpdateHazardBlink(int pixel_count);
void UpdateStartupWaveForMiddle(WS28XX_HandleTypeDef* ws);
void UpdatePositionLight(WS28XX_HandleTypeDef* ws, int pixel_count, int* is_position_light_displayed);
void UpdateBrakeLight(WS28XX_HandleTypeDef* ws, int pixel_count, int* is_brake_light_displayed);
void ResetLEDStrip(WS28XX_HandleTypeDef* ws, int pixel_count);
void HandleBottomLeftStripState(void);
void HandleBottomRightStripState(void);
void HandleNumberPlateLightState(void);
void HandleHMSLState(void);
void UpdateReverseLight(WS28XX_HandleTypeDef* ws, int pixel_count, int* is_light_displayed);
void UpdateNumberPlateLight(WS28XX_HandleTypeDef* ws, int pixel_count, int* is_light_displayed);
void UpdateHMSLLight(WS28XX_HandleTypeDef* ws, int pixel_count, int* is_light_displayed);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  WS28XX_Init(&ws_pa8, &htim1, 36, TIM_CHANNEL_1, TOP_LEFT_LED_COUNT);   // Initialize for PA8
  WS28XX_Init(&ws_pa9, &htim1, 36, TIM_CHANNEL_2, MIDDLE_LED_COUNT); // Initialize for PA9 (middle strip)
  WS28XX_Init(&ws_pa10, &htim1, 36, TIM_CHANNEL_3, TOP_RIGHT_LED_COUNT); // Initialize for PA10

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HandleMiddleStripState();
	  HandleRightTopStripState();
	  HandleLeftTopStripState();
	  HandleBottomLeftStripState();
	  HandleBottomRightStripState();
	  HandleNumberPlateLightState();
	  HandleHMSLState();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HandleMiddleStripState(void)
{
    // Check for brake light first
    if (brake_signal_received) {
        current_state_pa9 = BRAKE_LIGHT_MODE;
    }
    // If headlamp is on, switch to position light mode
    else if (headlamp_signal_received) {
        current_state_pa9 = POSITION_LIGHT_MODE;
    }
    // If no signals, check for startup mode
    else if (startup_signal_received) {
        current_state_pa9 = STARTUP_MODE;
    }
    // Default to off mode
    else {
        current_state_pa9 = OFF_STATE;
    }

    // State machine handling
    switch (current_state_pa9) {
        case BRAKE_LIGHT_MODE:
        	UpdateBrakeLight(&ws_pa9, MIDDLE_LED_COUNT, &is_brake_light_displayed);
        	is_position_light_displayed = 0;
            break;

        case POSITION_LIGHT_MODE:
        	UpdatePositionLight(&ws_pa9, MIDDLE_LED_COUNT, &is_position_light_displayed);
        	is_brake_light_displayed = 0;
            break;

        case STARTUP_MODE:
            UpdateStartupWaveForMiddle(&ws_pa9);  // Handle the red startup wave
            is_brake_light_displayed = 0;
            is_position_light_displayed = 0;
            break;

        case OFF_STATE:
        default:
            ResetLEDStrip(&ws_pa9, MIDDLE_LED_COUNT);  // Turn off all LEDs
            WS28XX_Update(&ws_pa9);
            is_brake_light_displayed = 0;
            is_position_light_displayed = 0;
            break;
    }
}

void HandleLeftTopStripState(void)
{
    // Determine the current state based on the signals received
    if (hazard_signal_received) {
        current_state_pa8 = HAZARD_MODE;
    }
    else if (turn_signal_left_received) {
        current_state_pa8 = TURN_SIGNAL_MODE;
    }
    else if (headlamp_signal_received) {
        current_state_pa8 = POSITION_LIGHT_MODE;  // Position light = marker light
    }
    else {
        current_state_pa8 = OFF_STATE;
    }

    // State machine handling for PA8 (Left Top Strip)
    switch (current_state_pa8) {
        case TURN_SIGNAL_MODE:
            UpdateTurnSignalWave(&ws_pa8, frame_pa8, TOP_LEFT_LED_COUNT);  // Amber wave effect
            frame_pa8 += WAVE_STEP_SIZE;
            if (frame_pa8 >= TOP_LEFT_LED_COUNT) frame_pa8 = 0;
            HAL_Delay(WAVE_SPEED);
            is_position_light_displayed_left = 0;  // Reset position light flag
            break;

        case HAZARD_MODE:
            UpdateHazardBlink(TOP_LEFT_LED_COUNT);  // Amber blink
            HAL_Delay(HAZARD_BLINK_DELAY);
            is_position_light_displayed_left = 0;  // Reset position light flag
            break;

        case POSITION_LIGHT_MODE:
            UpdatePositionLight(&ws_pa8, TOP_LEFT_LED_COUNT, &is_position_light_displayed_left);  // Marker/position light
            break;

        case OFF_STATE:
        default:
            ResetLEDStrip(&ws_pa8, TOP_LEFT_LED_COUNT);  // Turn off all LEDs
            WS28XX_Update(&ws_pa8);
            is_position_light_displayed_left = 0;  // Reset position light flag
            break;
    }
}

void HandleRightTopStripState(void)
{
    // Determine the current state based on the signals received
    if (hazard_signal_received) {
        current_state_pa10 = HAZARD_MODE;
    }
    else if (turn_signal_right_received) {
        current_state_pa10 = TURN_SIGNAL_MODE;
    }
    else if (headlamp_signal_received) {
        current_state_pa10 = POSITION_LIGHT_MODE;  // Position light = marker light
    }
    else {
        current_state_pa10 = OFF_STATE;
    }

    // State machine handling for PA10 (Right Top Strip)
    switch (current_state_pa10) {
        case TURN_SIGNAL_MODE:
            UpdateTurnSignalWave(&ws_pa10, frame_pa10, TOP_RIGHT_LED_COUNT);  // Amber wave effect
            frame_pa10 += WAVE_STEP_SIZE;
            if (frame_pa10 >= TOP_RIGHT_LED_COUNT) frame_pa10 = 0;
            HAL_Delay(WAVE_SPEED);
            is_position_light_displayed_right = 0;  // Reset position light flag
            break;

        case HAZARD_MODE:
            UpdateHazardBlink(TOP_RIGHT_LED_COUNT);  // Amber blink
            HAL_Delay(HAZARD_BLINK_DELAY);
            is_position_light_displayed_right = 0;  // Reset position light flag
            break;

        case POSITION_LIGHT_MODE:
            UpdatePositionLight(&ws_pa10, TOP_RIGHT_LED_COUNT, &is_position_light_displayed_right);  // Marker/position light
            break;

        case OFF_STATE:
        default:
            ResetLEDStrip(&ws_pa10, TOP_RIGHT_LED_COUNT);  // Turn off all LEDs
            WS28XX_Update(&ws_pa10);
            is_position_light_displayed_right = 0;  // Reset position light flag
            break;
    }
}

void UpdateStartupWaveForMiddle(WS28XX_HandleTypeDef* ws)
{
    int current_time = HAL_GetTick();

    // First phase: Two passes (outward and inward)
    if (wave_count_middle < 2) {
        if (current_time - last_update_time >= STARTUP_WAVE_SPEED) {
            // Outward wave movement
            if (wave_direction == 0) {
                ResetLEDStrip(ws, MIDDLE_LED_COUNT);  // Clear LEDs
                for (int i = 0; i < WAVE_PACKET_SIZE_MIDDLE; i++) {
                    if (frame_left - i >= 0) {
                        WS28XX_SetPixel_RGBW_565(ws, frame_left - i, COLOR_RGB565_RED, POSITION_BRIGHTNESS);
                    }
                    if (frame_right + i < MIDDLE_LED_COUNT) {
                        WS28XX_SetPixel_RGBW_565(ws, frame_right + i, COLOR_RGB565_RED, POSITION_BRIGHTNESS);
                    }
                }
                WS28XX_Update(ws);

                frame_left--;
                frame_right++;

                if (frame_left < 0 && frame_right >= MIDDLE_LED_COUNT) {
                    wave_direction = 1;  // Switch to inward direction
                }
            }
            // Inward wave movement
            else if (wave_direction == 1) {
                ResetLEDStrip(ws, MIDDLE_LED_COUNT);  // Clear LEDs
                for (int i = 0; i < WAVE_PACKET_SIZE_MIDDLE; i++) {
                    if (frame_left + i < MIDDLE_LED_MID_INDEX) {
                        WS28XX_SetPixel_RGBW_565(ws, frame_left + i, COLOR_RGB565_RED, POSITION_BRIGHTNESS);
                    }
                    if (frame_right - i >= MIDDLE_LED_MID_INDEX) {
                        WS28XX_SetPixel_RGBW_565(ws, frame_right - i, COLOR_RGB565_RED, POSITION_BRIGHTNESS);
                    }
                }
                WS28XX_Update(ws);

                frame_left++;
                frame_right--;

                if (frame_left >= MIDDLE_LED_MID_INDEX && frame_right <= MIDDLE_LED_MID_INDEX) {
                    frame_left = MIDDLE_LED_MID_INDEX;
                    frame_right = MIDDLE_LED_MID_INDEX;
                    wave_direction = 0;  // Reset for the next outward wave
                    wave_count_middle++;
                }
            }
            last_update_time = current_time;
        }
    }
    // Second phase: Sequential turn-on
    else if (sequential_turn_on < MIDDLE_LED_MID_INDEX) {
        if (current_time - last_update_time >= STARTUP_WAVE_SPEED * 2) {
            WS28XX_SetPixel_RGBW_565(ws, MIDDLE_LED_MID_INDEX - sequential_turn_on, COLOR_RGB565_RED, POSITION_BRIGHTNESS);
            WS28XX_SetPixel_RGBW_565(ws, MIDDLE_LED_MID_INDEX + sequential_turn_on, COLOR_RGB565_RED, POSITION_BRIGHTNESS);

            WS28XX_Update(ws);
            sequential_turn_on++;

            last_update_time = current_time;
        }
    }
    // Transition complete (go to OFF_STATE after the startup wave)
    else {
        wave_count_middle = 0;
        sequential_turn_on = 0;
        startup_signal_received = 0;  // Disable startup signal after animation completes
        current_state_pa9 = OFF_STATE;  // Move to off state
    }
}

void UpdatePositionLight(WS28XX_HandleTypeDef* ws, int pixel_count, int* is_position_light_displayed)
{
    if (*is_position_light_displayed) {
        return;  // Avoid redundant refreshes (prevent flickering)
    }

    ResetLEDStrip(ws, pixel_count);  // Clear previous state

    for (int i = 0; i < pixel_count; i++) {
        WS28XX_SetPixel_RGBW_565(ws, i, COLOR_RGB565_RED, POSITION_BRIGHTNESS);  // 50% brightness red
    }

    WS28XX_Update(ws);
    *is_position_light_displayed = 1;
}

void UpdateBrakeLight(WS28XX_HandleTypeDef* ws, int pixel_count, int* is_brake_light_displayed)
{
    if (*is_brake_light_displayed) {
        return;  // Avoid redundant refreshes (prevent flickering)
    }

    ResetLEDStrip(ws, pixel_count);  // Clear previous state

    for (int i = 0; i < pixel_count; i++) {
        WS28XX_SetPixel_RGBW_565(ws, i, COLOR_RGB565_RED, BRAKE_BRIGHTNESS);  // Full brightness red
    }

    WS28XX_Update(ws);
    *is_brake_light_displayed = 1;
}

void UpdateTurnSignalWave(WS28XX_HandleTypeDef* ws, int frame, int pixel_count)
{
    ResetLEDStrip(ws, pixel_count);  // Clear all LEDs
    for (int i = 0; i < pixel_count; i++) {
        if (i >= frame && i < frame + WAVE_PACKET_SIZE) {
            WS28XX_SetPixel_RGBW_565(ws, i, COLOR_RGB565_AMBER, 255);  // Amber wave
        }
    }
    WS28XX_Update(ws);
}

void UpdateHazardBlink(int pixel_count)
{
    static int blink_on = 0;

    for (int i = 0; i < pixel_count; i++) {
        if (blink_on) {
        	WS28XX_SetPixel_RGBW_565(&ws_pa10, i, COLOR_RGB565_AMBER, BRAKE_BRIGHTNESS);   // Amber on for right strip
            WS28XX_SetPixel_RGBW_565(&ws_pa8, i, COLOR_RGB565_AMBER, BRAKE_BRIGHTNESS);   // Amber on for left strip
        } else {
            WS28XX_SetPixel_RGBW_565(&ws_pa8, i, COLOR_RGB565_BLACK, 0);  // Off for left strip
            WS28XX_SetPixel_RGBW_565(&ws_pa10, i, COLOR_RGB565_BLACK, 0);  // Off for right strip
        }
    }

    blink_on = !blink_on;
    WS28XX_Update(&ws_pa8);
    WS28XX_Update(&ws_pa10);
}

void HandleBottomLeftStripState(void)
{
    if (reverse_signal_received) {
        UpdateReverseLight(&ws_pa0, LEFT_BOTTOM_LED_COUNT, &is_reverse_light_displayed_left);
    } else {
        ResetLEDStrip(&ws_pa0, LEFT_BOTTOM_LED_COUNT);
        WS28XX_Update(&ws_pa0);
        is_reverse_light_displayed_left = 0;
    }
}

void HandleBottomRightStripState(void)
{
    if (reverse_signal_received) {
        UpdateReverseLight(&ws_pa2, RIGHT_BOTTOM_LED_COUNT, &is_reverse_light_displayed_right);
    } else {
        ResetLEDStrip(&ws_pa2, RIGHT_BOTTOM_LED_COUNT);
        WS28XX_Update(&ws_pa2);
        is_reverse_light_displayed_right = 0;
    }
}

void HandleNumberPlateLightState(void)
{
    if (headlamp_signal_received) {
        UpdateNumberPlateLight(&ws_pa1, NUMBER_PLATE_LED_COUNT, &is_number_plate_light_displayed);
    } else {
        ResetLEDStrip(&ws_pa1, NUMBER_PLATE_LED_COUNT);
        WS28XX_Update(&ws_pa1);
        is_number_plate_light_displayed = 0;
    }
}

void HandleHMSLState(void)
{
    if (brake_signal_received) {
        UpdateHMSLLight(&ws_pa11, HMSL_LED_COUNT, &is_hmsl_light_displayed);
    } else {
        ResetLEDStrip(&ws_pa11, HMSL_LED_COUNT);
        WS28XX_Update(&ws_pa11);
        is_hmsl_light_displayed = 0;
    }
}

/* Helper Functions */
void UpdateReverseLight(WS28XX_HandleTypeDef* ws, int pixel_count, int* is_light_displayed)
{
    if (*is_light_displayed) {
        return;  // Avoid redundant refreshes (prevent flickering)
    }

    ResetLEDStrip(ws, pixel_count);  // Clear previous state

    for (int i = 0; i < pixel_count; i++) {
        WS28XX_SetPixel_RGBW_565(ws, i, COLOR_RGB565_WHITE, REVERSE_LIGHT_BRIGHTNESS);
    }

    WS28XX_Update(ws);
    *is_light_displayed = 1;
}

void UpdateNumberPlateLight(WS28XX_HandleTypeDef* ws, int pixel_count, int* is_light_displayed)
{
    if (*is_light_displayed) {
        return;  // Avoid redundant refreshes (prevent flickering)
    }

    ResetLEDStrip(ws, pixel_count);  // Clear previous state

    for (int i = 0; i < pixel_count; i++) {
        WS28XX_SetPixel_RGBW_565(ws, i, COLOR_RGB565_WHITE, NUMBER_PLATE_BRIGHTNESS);
    }

    WS28XX_Update(ws);
    *is_light_displayed = 1;
}

void UpdateHMSLLight(WS28XX_HandleTypeDef* ws, int pixel_count, int* is_light_displayed)
{
    if (*is_light_displayed) {
        return;  // Avoid redundant refreshes (prevent flickering)
    }

    ResetLEDStrip(ws, pixel_count);  // Clear previous state

    for (int i = 0; i < pixel_count; i++) {
        WS28XX_SetPixel_RGBW_565(ws, i, COLOR_RGB565_RED, BRAKE_BRIGHTNESS);
    }

    WS28XX_Update(ws);
    *is_light_displayed = 1;
}

void ResetLEDStrip(WS28XX_HandleTypeDef* ws, int pixel_count)
{
    for (int i = 0; i < pixel_count; i++) {
        WS28XX_SetPixel_RGBW_565(ws, i, COLOR_RGB565_BLACK, 0);  // Turn off each LED
    }
    WS28XX_Update(ws);
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

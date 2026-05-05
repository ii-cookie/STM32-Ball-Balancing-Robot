	/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include "bsp_ov7725.h"
#include "bsp_sccb.h"
#include "pid.h"
#include "robot_controller.h"
#include <stdio.h>


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
SRAM_HandleTypeDef hsram1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FSMC_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
volatile uint8_t Ov7725_vsync ;
PIDController ball_pid;

/* === Modes === */
volatile uint8_t shape_state = 0;
volatile uint8_t square_mode = 0;
volatile uint8_t circle_mode = 0;
volatile uint8_t joystick_mode = 0;

/* === Square Mode Variables === */
uint8_t current_side = 0;                   // 0: Top, 1: Right, 2: Bottom, 3: Left
float progress = 0.0f;
uint32_t last_move_time = 0;
const float MOVE_SPEED = 0.06f;
const uint32_t CORNER_SETTLE_TIME_MS = 500;
const float square_length = 16.0f;

uint32_t corner_arrival_time = 0;
uint8_t waiting_at_corner = 0;

/* === Circle Mode Variables === */
float circle_angle = 0.0f;
uint8_t circle_phase = 0;           // 0 = moving to start position, 1 = rotating
const float CIRCLE_RADIUS = 10.0f;
const float CIRCLE_SPEED = 0.2f;          // Radians per update

uint32_t circle_start_time = 0;
uint32_t last_circle_time = 0;

/* === Common === */
const float center_x = 36.0f;
const float center_y = 32.0f;

float square_corners[4][2] = {
    {center_x - square_length, center_y - square_length},   // Top-Left
    {center_x + square_length, center_y - square_length},   // Top-Right
    {center_x + square_length, center_y + square_length},   // Bottom-Right
    {center_x - square_length, center_y + square_length}    // Bottom-Left
};

float target_x = center_x;
float target_y = center_y;

#define BALL_SETTLE_THRESHOLD  11.0f
#define BALL_SETTLE_TIME_MS    350

uint32_t stable_start_time = 0;
uint8_t ball_is_stable = 0;
volatile uint8_t circle_mode_request = 0;
volatile uint8_t square_mode_request = 0;

/* === Offset Calibration Mode === */
volatile uint8_t offset_mode = 0;           // 0 = normal, 1 = offset calibration mode

float base_offset = 1000.0f;  // Default center offset
/* === Offset Adjustment Mode === */

float offset_x = 0.0f;      // Virtual offset in X direction
float offset_y = 0.0f;      // Virtual offset in Y direction

float offset_magnitude = 0.0f;
float offset_angle_deg = 0.0f;


// ==========joystick control==========
#define BTN_UP_PRESSED      (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10) == GPIO_PIN_RESET)
#define BTN_DOWN_PRESSED    (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11) == GPIO_PIN_RESET)
#define BTN_LEFT_PRESSED    (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12) == GPIO_PIN_RESET)
#define BTN_RIGHT_PRESSED   (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2)  == GPIO_PIN_RESET)
#define BTN_SET_PRESSED     (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8)  == GPIO_PIN_RESET)
#define BTN_RST_PRESSED     (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_RESET)

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE BEGIN 4 */

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
  MX_FSMC_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
// ============PID VALUES============
  	PID_Init(&ball_pid,
  		   0.05f,      // kp
  		   0.000f,     // ki
  		   0.07f,    // kd
  		   0.95f,        // alpha (filter)
  		   0.01f,         // beta
  		   30.0f,        // max_theta
  		   1);           // 1 = use tanh
	RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_MCO) | RCC_CFGR_MCO_SYSCLK;
	
	LCD_INIT();

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); //PC8
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4); //PC9
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1); //PB6

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	while(Ov7725_Init() != SUCCESS);
	Ov7725_vsync = 0;


	while (1)
	{
	    if (Ov7725_vsync == 2)
	    {
	        FIFO_PREPARE;
	        ImagDisp();
	        Ov7725_vsync = 0;
	        float theta, phi;
	        float curr_x = (float)centroid_x;
	        float curr_y = (float)centroid_y;

	        char print_target[16];
	        sprintf(print_target, "target: (%d,%d)", (int)target_x, (int)target_y);
	        LCD_DrawString(4, 60, (uint8_t *)print_target);

	        char print_centroid[16];
	        sprintf(print_centroid, "centroid: (%d,%d)", (int)centroid_x, (int)centroid_y);
	        LCD_DrawString(4, 75, (uint8_t *)print_centroid);

	        uint32_t now = HAL_GetTick();

	        /* ====================== SET BUTTON: Toggle Joystick Mode ====================== */
	                static uint32_t last_set_press = 0;
	                if (BTN_SET_PRESSED && (now - last_set_press > 250))   // 250ms debounce
	                {
	                    last_set_press = now;
	                    joystick_mode = !joystick_mode;     // Toggle between 0 and 1

	                    // Visual feedback
	                    LCD_Clear(80, 0, 80, 60, WHITE);
	                    if (joystick_mode)
	                    {
	                         target_x = center_x;
	                         target_y = center_y;
	             			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
	             			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
	             			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	             			HAL_Delay(100);
	                    }
						else
						{
	                         target_x = center_x;
	                         target_y = center_y;
		             			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
		             			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
		             			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
		             			HAL_Delay(100);
						}
	                }



	                /* ====================== JOYSTICK + RST BUTTON HANDLING ====================== */
	                static uint32_t last_joy_update = 0;
	                static uint32_t last_rst_press = 0;

	                /* ====================== RST BUTTON: Toggle Offset Mode ====================== */
	                if (BTN_RST_PRESSED && (now - last_rst_press > 250))
	                {
	                    last_rst_press = now;
	                    offset_mode = !offset_mode;
	                    joystick_mode = 0;                    // Disable normal joystick mode when offset mode is active

	                    if (offset_mode)
	                    {
	                         target_x = center_x;
	                         target_y = center_y;
	             			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
	             			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
	             			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	             			HAL_Delay(100);
	                    }
						else
						{
	                         target_x = center_x;
	                         target_y = center_y;
		             			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
		             			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
		             			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
		             			HAL_Delay(100);
						}

	                    LCD_Clear(80, 0, 80, 60, WHITE);
	                    if (offset_mode)
	                    {
	                        LCD_DrawString(4, 120, (uint8_t*)"OFFSET MODE");
	                    }
	                    else
	                    {
	                        LCD_Clear(4, 120, 200, 150, WHITE);
	                    }
	                }

	                /* ====================== JOYSTICK CONTROL ====================== */
	                if (now - last_joy_update >= 50)        // 50ms update rate
	                {
	                    last_joy_update = now;
	                    float step = 0.8f;                  // How fast offset changes

	                    if (offset_mode)
	                    {
	                        /* ==================== OFFSET ADJUSTMENT MODE ==================== */
	                        if (BTN_UP_PRESSED)    offset_y -= step;
	                        if (BTN_DOWN_PRESSED)  offset_y += step;
	                        if (BTN_LEFT_PRESSED)  offset_x -= step;
	                        if (BTN_RIGHT_PRESSED) offset_x += step;

	                        //Soft limits
	                        if (offset_x < -15.0f) offset_x = -15.0f;
	                        if (offset_x >  15.0f) offset_x =  15.0f;
	                        if (offset_y < -15.0f) offset_y = -15.0f;
	                        if (offset_y >  15.0f) offset_y =  15.0f;

	                        // Convert to polar for nice display
	                        offset_magnitude = sqrtf(offset_x*offset_x + offset_y*offset_y);
	                        if (offset_magnitude > 0.01f)
	                            offset_angle_deg = atan2f(offset_y, offset_x) * 180.0f / M_PI;
	                    }
	                    else if (joystick_mode)
	                    {
	                        /* ==================== NORMAL TARGET MOVING MODE ==================== */
	                        float pos_step = 1.5f;
	                        if (BTN_UP_PRESSED)    target_y -= pos_step;
	                        if (BTN_DOWN_PRESSED)  target_y += pos_step;
	                        if (BTN_LEFT_PRESSED)  target_x -= pos_step;
	                        if (BTN_RIGHT_PRESSED) target_x += pos_step;

	                        if (target_x < 15.0f) target_x = 15.0f;
	                        if (target_x > 65.0f) target_x = 65.0f;
	                        if (target_y < 10.0f) target_y = 10.0f;
	                        if (target_y > 45.0f) target_y = 45.0f;
	                    }
	                }


			if (offset_mode)
			{
				char offset_coordinate[32];
				sprintf(offset_coordinate, "Offset: %d, %d      ", (int)offset_x, (int)offset_y);
				LCD_DrawString(4, 135, (uint8_t*)offset_coordinate);
			}

	        // key button press
	  	  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET && HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET) {//k1 and k2
		        PID_Compute(&ball_pid, target_x, target_y, curr_x, curr_y, &theta, &phi, 1);
		        Robot_Goto_Spherical(theta, phi, 1400);
		        HAL_Delay(50);
		        Robot_Goto_Spherical(theta, phi, 700);


				continue;
	  	      HAL_Delay(50);
	  	    }
	  	/* ====================== BUTTON HANDLING (Edge Triggered) ====================== */
	  	static uint32_t last_k1_time = 0;
	  	static uint32_t last_k2_time = 0;
	  	static uint8_t  prev_k1_state = GPIO_PIN_RESET;
	  	static uint8_t  prev_k2_state = GPIO_PIN_RESET;


	  	// K1 (PA0) and K2 (PC13) handling - detect rising edge only
	  	if (now - last_k1_time > 200)   // 200ms debounce
	  	{
	  	    uint8_t current_k1 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);

	  	    if (current_k1 == GPIO_PIN_SET && prev_k1_state == GPIO_PIN_RESET)
	  	    {
	  	        // Rising edge detected on K1
	  	        last_k1_time = now;

	  	        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET)
	  	        {
	  	            // Both K1 + K2 pressed → Special function
	  	            PID_Compute(&ball_pid, target_x, target_y, curr_x, curr_y, &theta, &phi, 1);
	  	            Robot_Goto_Spherical(theta, phi, 1400);
	  	            HAL_Delay(50);
	  	            Robot_Goto_Spherical(theta, phi, 700);
	  	        }
	  	        else
	  	        {
	  	            // Only K1 pressed
	  	            square_mode_request = 0;
	  	            square_mode = !square_mode;
	  	            LCD_Clear(80, 0, 80, 60, WHITE);
	  	            corner_arrival_time = HAL_GetTick();

	  	            if (square_mode)
	  	            {
	  	                target_x = square_corners[0][0];
	  	                target_y = square_corners[0][1];
	  	                current_side = 3;
	  	                progress = 0.0f;
	  	                waiting_at_corner = 1;
	  	                ball_is_stable = 0;
	  	                stable_start_time = now;
	  	                last_move_time = now;
	  	            }
	  	            else
	  	            {
	  	                target_x = center_x;
	  	                target_y = center_y;
	  	                waiting_at_corner = 0;
	  	                ball_is_stable = 0;
	  	            }
	  	        }
	  	    }
	  	    prev_k1_state = current_k1;
	  	}

	  	if (now - last_k2_time > 200)   // 200ms debounce
	  	{
	  	    uint8_t current_k2 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);

	  	    if (current_k2 == GPIO_PIN_SET && prev_k2_state == GPIO_PIN_RESET)
	  	    {
	  	        // Rising edge detected on K2
	  	        last_k2_time = now;

	  	        // Only trigger K2 if K1 is NOT pressed (to avoid conflict with both-pressed case)
	  	        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) != GPIO_PIN_SET)
	  	        {
	  	            circle_mode_request = 0;
	  	            circle_mode = !circle_mode;
	  	            LCD_Clear(80, 0, 80, 60, WHITE);

	  	            if (circle_mode)
	  	            {
	  	                square_mode = 0;
	  	                circle_phase = 0;
	  	                circle_angle = 0.0f;
	  	                last_circle_time = now;
	  	                ball_is_stable = 0;
	  	                target_x = center_x + CIRCLE_RADIUS;
	  	                target_y = center_y;
	  	            }
	  	            else
	  	            {
	  	                circle_phase = 0;
	  	                target_x = center_x;
	  	                target_y = center_y;
	  	            }
	  	        }
	  	    }
	  	    prev_k2_state = current_k2;
	  	}

	        PID_Compute(&ball_pid, target_x, target_y, curr_x, curr_y, &theta, &phi, 0);
	        Robot_Goto_Spherical(theta, phi, base_offset);


	        if (square_mode)
	        {
	            float dx = curr_x - target_x;
	            float dy = curr_y - target_y;
	            float dist = sqrtf(dx * dx + dy * dy);

	            uint32_t now = HAL_GetTick();

	            if (waiting_at_corner)
	            {
	                // === WAIT AT CORNER FOR STABILIZATION (2 seconds) ===
	                if (!ball_is_stable)
	                {
	                    if (dist <= BALL_SETTLE_THRESHOLD)
	                    {
	                        ball_is_stable = 1;
	                        stable_start_time = now;
	                    }
	                }
	                else
	                {
	                    // Ball is considered stable, check if 2 seconds have passed
	                    if (now - stable_start_time >= CORNER_SETTLE_TIME_MS)   // 2 seconds
	                    {
	                        // Time to move to next corner
	                        current_side = (current_side + 1) % 4;
	                        progress = 0.0f;
	                        waiting_at_corner = 0;
	                        ball_is_stable = 0;
	                        last_move_time = now;

	                        // Set target to start of next side
	                        switch (current_side)
	                        {
	                            case 0: target_x = square_corners[0][0]; target_y = square_corners[0][1]; break;
	                            case 1: target_x = square_corners[1][0]; target_y = square_corners[1][1]; break;
	                            case 2: target_x = square_corners[2][0]; target_y = square_corners[2][1]; break;
	                            case 3: target_x = square_corners[3][0]; target_y = square_corners[3][1]; break;
	                        }
	                    }
	                }
	            }
	            else
	            {
	                // === MOVING ALONG THE SIDE WITH DECELERATION ===
	                if (now - last_move_time >= 20)
	                {
	                    last_move_time = now;

	                    // Deceleration near the corner
	                    float speed_factor = 1.0f;
	                    if (progress > 0.9f)	// deccel for how long
	                    {
	                        speed_factor = 1.0f - (progress - 0.7f) * 3.0f;   // linear slow down
	                        if (speed_factor < 0.01f) speed_factor = 0.01f;   // minimum speed
	                    }

	                    progress += MOVE_SPEED * speed_factor;

	                    if (progress >= 1.0f)
	                    {
	                        progress = 1.0f;
	                        waiting_at_corner = 1;
	                        ball_is_stable = 0;
	                        stable_start_time = now;
	                    }

	                    // Update target position along the current side
	                    float pos = progress * square_length * 2.0f;

	                    switch (current_side)
	                    {
	                        case 0: // Top: left to right
	                            target_x = square_corners[0][0] + pos;
	                            target_y = square_corners[0][1];
	                            break;
	                        case 1: // Right: top to bottom
	                            target_x = square_corners[1][0];
	                            target_y = square_corners[1][1] + pos;
	                            break;
	                        case 2: // Bottom: right to left
	                            target_x = square_corners[2][0] - pos;
	                            target_y = square_corners[2][1];
	                            break;
	                        case 3: // Left: bottom to top
	                            target_x = square_corners[3][0];
	                            target_y = square_corners[3][1] - pos;
	                            break;
	                    }
	                }
	            }

	            if (progress > 0.85f && !waiting_at_corner)
	            {
	                if (dist <= BALL_SETTLE_THRESHOLD)
	                {
	                    if (!ball_is_stable)
	                    {
	                        ball_is_stable = 1;
	                        stable_start_time = now;
	                    }
	                }
	                else
	                {
	                    ball_is_stable = 0;
	                }
	            }
	        }
	        else if (circle_mode)
	        {
	            float dx = curr_x - target_x;
	            float dy = curr_y - target_y;
	            float dist = sqrtf(dx * dx + dy * dy);

	            if (circle_phase == 0)
	            {
	                if (now - last_circle_time >= 2000)
	                {
	                    last_circle_time = now;
	                    float target_start_x = center_x + CIRCLE_RADIUS;
	                    float target_start_y = center_y - 10;
	                    target_x = target_x * 0.92f + target_start_x * 0.08f;
	                    target_y = target_y * 0.92f + target_start_y * 0.08f;

	                    if (dist <= BALL_SETTLE_THRESHOLD)
	                    {
	                        if (!ball_is_stable)
	                        {
	                            ball_is_stable = 1;
	                            circle_start_time = now;
	                        }
	                        else if (now - circle_start_time > BALL_SETTLE_TIME_MS)
	                        {
	                            circle_phase = 1;
	                            circle_angle = 0.0f;
	                            ball_is_stable = 0;
	                            last_circle_time = now;
	                        }
	                    }
	                    else
	                    {
	                        ball_is_stable = 0;
	                    }
	                }
	            }
	            else  // circle_phase == 1
	            {
	                if (now - last_circle_time >= 20)
	                {
	                    last_circle_time = now;
	                    circle_angle += CIRCLE_SPEED;
	                    if (circle_angle >= 6.283185f)  // 2 * PI
	                        circle_angle -= 6.283185f;

	                    target_x = center_x + CIRCLE_RADIUS * cosf(circle_angle);
	                    target_y = center_y + CIRCLE_RADIUS * sinf(circle_angle);
	                }
	            }
	        }

	        if (square_mode)
	        {
	            LCD_DrawLine(square_corners[0][0], square_corners[0][1], square_corners[1][0], square_corners[1][1], RED);
	            LCD_DrawLine(square_corners[1][0], square_corners[1][1], square_corners[2][0], square_corners[2][1], RED);
	            LCD_DrawLine(square_corners[2][0], square_corners[2][1], square_corners[3][0], square_corners[3][1], RED);
	            LCD_DrawLine(square_corners[3][0], square_corners[3][1], square_corners[0][0], square_corners[0][1], RED);


	            LCD_Clear((uint16_t)target_x - 4, (uint16_t)target_y - 4, 8, 8, YELLOW);
	        }
	        else
	        {
	            LCD_Clear((uint16_t)target_x - 8, (uint16_t)target_y - 1, 16, 2, YELLOW);
	            LCD_Clear((uint16_t)target_x - 1, (uint16_t)target_y - 8, 2, 16, YELLOW);
	        }
	    }


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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 799;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 71;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 19999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 799;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_5, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC4 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB11 PB12 PB13
                           PB14 PB15 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD3 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC10 PC11 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* FSMC initialization function */
static void MX_FSMC_Init(void)
{

  /* USER CODE BEGIN FSMC_Init 0 */

  /* USER CODE END FSMC_Init 0 */

  FSMC_NORSRAM_TimingTypeDef Timing = {0};

  /* USER CODE BEGIN FSMC_Init 1 */

  /* USER CODE END FSMC_Init 1 */

  /** Perform the SRAM1 memory initialization sequence
  */
  hsram1.Instance = FSMC_NORSRAM_DEVICE;
  hsram1.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram1.Init */
  hsram1.Init.NSBank = FSMC_NORSRAM_BANK1;
  hsram1.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
  hsram1.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;
  hsram1.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
  hsram1.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
  hsram1.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram1.Init.WrapMode = FSMC_WRAP_MODE_DISABLE;
  hsram1.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
  hsram1.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
  hsram1.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
  hsram1.Init.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;
  hsram1.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram1.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
  /* Timing */
  Timing.AddressSetupTime = 15;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 255;
  Timing.BusTurnAroundDuration = 15;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FSMC_ACCESS_MODE_A;
  /* ExtTiming */

  if (HAL_SRAM_Init(&hsram1, &Timing, NULL) != HAL_OK)
  {
    Error_Handler( );
  }

  /** Disconnect NADV
  */

  __HAL_AFIO_FSMCNADV_DISCONNECTED();

  /* USER CODE BEGIN FSMC_Init 2 */

  /* USER CODE END FSMC_Init 2 */
}

/* USER CODE BEGIN 4 */

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

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE BEGIN PV */
// --- Timing constants in milliseconds ---

#define TIMEOUT_MS   (1000U)  // TIMEOUT
#define PWM_RESOLUTION (4096U) //PWM-Resolution

// UART RX buffer for QUAD frame: 2 bytes (little-endian: low, then high)
static uint8_t rx_bytes[2];

static volatile uint8_t  rx_done = 0;   // set in ISR
static volatile uint16_t current_pwm=0; //Current PWM CCR
static volatile uint16_t target_pwm=0;  //Target PWM CCR

// UART decoding
static uint8_t dir = 0;
static uint32_t pwm_channel = 0;
static uint8_t mag = 0;
static uint32_t last_toggle  = 0;
static uint32_t power_until  = 0; 

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static inline uint8_t stc_from_TL(uint16_t quad)
{
  // quad layout: [TL(0:3)][TR(4:7)][BL(8:11)][BR(12:15)]
  uint8_t TL= (uint8_t)((quad >> 0) & 0x0F);  // TL nibble
  uint8_t TR= (uint8_t)((quad >> 4) & 0x0F);  // TL nibble
  uint8_t BL= (uint8_t)((quad >> 8) & 0x0F);  // TL nibble
  uint8_t BR= (uint8_t)((quad >> 12) & 0x0F);  // TL nibble

  return TL;
}

static inline uint8_t mag_from_stc(uint8_t stc)
{
  // stc bit0 = dir, bits1..3 = mag
  return (uint8_t)((stc >> 1) & 0x07);   // 0..7 (we'll clamp to 0..5)
}

static inline uint8_t dir_from_stc(uint8_t stc)
{
  return (uint8_t)(stc & 0x01); // Gets direction bit from
}

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
  MX_UART5_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart5, rx_bytes, 2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    const uint32_t now = HAL_GetTick();

    // New frame arrived?
    if (rx_done) {
      HAL_NVIC_DisableIRQ(UART5_IRQn);
      rx_done = 0;
      // uint8_t debug[] = "Hello World";
      // HAL_UART_Transmit_IT(&huart5, debug, 12);
      // Reconstruct quad word: little-endian (low, then high)
      uint16_t quad = (uint16_t)rx_bytes[0] | ((uint16_t)rx_bytes[1] << 8);
      HAL_NVIC_EnableIRQ(UART5_IRQn);

      // Decode TL nibble -> stc -> magnitude
      uint8_t stc = stc_from_TL(quad);
      mag = mag_from_stc(stc);
      dir = dir_from_stc(stc);
      if (mag > 5) mag = 5;
      target_pwm= (mag==5) ? 4095 : 820*mag;

      pwm_channel = (dir==0) ? TIM_CHANNEL_1 : TIM_CHANNEL_2;
      // Start/extend blink burst window
      power_until = now + TIMEOUT_MS;
    }
    
    //Timeout after 1 Second
    if (now>power_until){
      target_pwm=0;
    }

    __HAL_TIM_SET_COMPARE(&htim2, (pwm_channel==0)? TIM_CHANNEL_2: TIM_CHANNEL_1, 0);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart5) {
    // signal main loop that two bytes are ready
    rx_done = 1;
    // re-arm for the next 2-byte quad frame
    HAL_UART_Receive_IT(&huart5, rx_bytes, 2);
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{

  if (htim == &htim4){
    // Check if we have reached the required speed and do speed ramping if not
    if (current_pwm < target_pwm){
      current_pwm = (current_pwm+10 > 4095) ? 4095 : current_pwm+10;
      __HAL_TIM_SET_COMPARE(&htim2, pwm_channel, current_pwm);
    }
    else if (current_pwm>target_pwm)
    {
      current_pwm = (current_pwm-10 < target_pwm) ? target_pwm : current_pwm-10;
      __HAL_TIM_SET_COMPARE(&htim2, pwm_channel, current_pwm);
    }
    
  }
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
#ifdef USE_FULL_ASSERT
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

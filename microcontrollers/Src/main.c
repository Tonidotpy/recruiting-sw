/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	DANGER,
	WAITING,
	RUNNING
} State;

typedef uint8_t bool_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FALSE 0
#define TRUE 1

#define SENSOR_TIMER htim13
#define VOLTAGE_TIMER htim14

#define SENSOR_ADC hadc1
#define VOLTAGE_ADC hadc2

#define UNDER_LED_PERIPHERAL GPIOA
#define OVER_LED_PERIPHERAL GPIOB
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

TIM_HandleTypeDef htim13;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

// Pins
static const uint16_t BUTTON_PIN = GPIO_PIN_13;
static const uint16_t UNDER_LED_PIN = GPIO_PIN_4;
static const uint16_t OVER_LED_PIN  = GPIO_PIN_0;

// Timeouts
static const uint32_t VOLTAGE_POLL_TIMEOUT  = 150;
static const uint32_t SENSOR_POLL_TIMEOUT   = 150;
static const uint32_t UART_TRANSMIT_TIMEOUT = 200;

// Messages
static const char WAIT_MSG[] = "Board in waiting state - please press the emergency button\r\n";
static const char DANGER_MSG[] = "Danger, voltage out of the expected range!\r\n";
static const char VOLTAGE_ERROR_MSG[] = "Error while reading voltage value!\r\n";
static const char SENSOR_ERROR_MSG[] = "Error while reading sensor value!\r\n";
static const char VOLTAGE_BUSY_MSG[] = "Waiting for voltage ADC conversion\r\n";
static const char SENSOR_BUSY_MSG[] = "Waiting for sensor ADC conversion\r\n";
static const char LOWER_SENSOR_THRESHOLD_MSG[] = "Sensor output too low - Maybe sensor is missing\r\n";
static const char UPPER_SENSOR_THRESHOLD_MSG[] = "Sensor output too high\r\n";

// Maximum voltage value in mV
static const uint16_t MAX_VOLTAGE_VALUE = 5000;
static const uint16_t MAX_SENSOR_VALUE = 3300;
// System voltage range in mV
static const uint16_t UNDERVOLTAGE = 1800;
static const uint16_t OVERVOLTAGE  = 2700;
// Sensor value range in Gs
static const int LOWER_SENSOR_THRESHOLD = -1000l;
static const int UPPER_SENSOR_THRESHOLD = 1000l;

static State current_state;
static uint16_t voltage;
static uint16_t sensor_val;

static bool_t can_read_voltage;
static bool_t can_read_sensor;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM13_Init(void);
static void MX_TIM14_Init(void);
static void MX_ADC2_Init(void);
/* USER CODE BEGIN PFP */
void danger(void);
void wait(void);
void run(void);

void uart_print(const char*);

void read_voltage(void);
void read_sensor(void);
void set_under_led(bool_t);   // Set undervoltage led
void set_over_led(bool_t);    // Set overvoltage led
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
  current_state = RUNNING;

  voltage    = 0;
  sensor_val = 0;

  can_read_voltage = FALSE;
  can_read_sensor  = FALSE;
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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM13_Init();
  MX_TIM14_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
  // Start timers
  __HAL_TIM_CLEAR_IT(&SENSOR_TIMER ,TIM_IT_UPDATE);
  __HAL_TIM_CLEAR_IT(&VOLTAGE_TIMER ,TIM_IT_UPDATE);
  HAL_TIM_Base_Start_IT(&SENSOR_TIMER);
	HAL_TIM_Base_Start_IT(&VOLTAGE_TIMER);
  
  // Enable ADC
  HAL_ADC_Start(&SENSOR_ADC);
  HAL_ADC_Start(&VOLTAGE_ADC);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		switch (current_state)
		{
			case DANGER:
				danger();
				break;
			case WAITING:
				wait();
				break;
			case RUNNING:
				run();
				break;
		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }

  // Stop timers
	HAL_TIM_Base_Stop_IT(&SENSOR_TIMER);
	HAL_TIM_Base_Stop_IT(&VOLTAGE_TIMER);

  // Disable ADC
  HAL_ADC_Stop(&SENSOR_ADC);
  HAL_ADC_Stop(&VOLTAGE_ADC);
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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 8400 - 1;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 2000 - 1;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 8400 - 1;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 3500 - 1;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, A2_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(A3_GPIO_Port, A3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : A2_Pin LD2_Pin */
  GPIO_InitStruct.Pin = A2_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : A3_Pin */
  GPIO_InitStruct.Pin = A3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(A3_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
// Button interrupt handler
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	// Check if button is pressed
	if (GPIO_Pin == BUTTON_PIN) {
		if (current_state == WAITING) {
			current_state = RUNNING;
		}
		else {
			current_state = WAITING;
		}
	}
}
// Timer callback function
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	// Check system voltage value
	if (htim == &VOLTAGE_TIMER) {
    can_read_voltage = TRUE;
  }

  // Check sensor value
	if (current_state == RUNNING && htim == &SENSOR_TIMER) {
    can_read_sensor = TRUE;
  } 
}

void danger(void) {
  // Read system voltage
  if (can_read_voltage) {
    read_voltage();
    can_read_voltage = FALSE;
    
    // Print voltage value
    char msg[40];
    snprintf(msg, 40, "[%10lu] Voltage: %hu mV\r\n", HAL_GetTick() / 10ul, voltage);
    uart_print(msg);

    // Turn on led based on voltage value
    if (voltage < UNDERVOLTAGE) {
      if (!HAL_GPIO_ReadPin(UNDER_LED_PERIPHERAL, UNDER_LED_PIN)) {
        set_under_led(TRUE);
        set_over_led(FALSE);
        uart_print("Undervoltage led - ON\r\n");
        uart_print("Overvoltage led  - OFF\r\n");
      }
    }
    else if (voltage > OVERVOLTAGE) {
      if (!HAL_GPIO_ReadPin(OVER_LED_PERIPHERAL, OVER_LED_PIN)) {
        set_under_led(FALSE);
        set_over_led(TRUE);
        uart_print("Undervoltage led - OFF\r\n");
        uart_print("Overvoltage led  - ON\r\n");
      }
    }
    else {
      // Exit danger state
      set_under_led(FALSE);
      set_over_led(FALSE);
      uart_print("Undervoltage led - OFF\r\n");
      uart_print("Overvoltage led  - OFF\r\n");
      current_state = RUNNING;
    }
  }
}
void wait(void) {
	uart_print(WAIT_MSG);
	HAL_Delay(500);
}
void run(void) {
  // Read sensor value
  if (can_read_sensor) {
    read_sensor();
    can_read_sensor = FALSE;

    // Print sensor value
    char msg[40];
    double gauss_val = ((double)sensor_val - 1650.0) / 1.065;
    int unit = trunc(gauss_val * 1000.0);
    uint16_t decimal = abs(unit) % 1000;
    unit /= 1000;
    snprintf(msg, 40, "[%10lu] Sensor: %d.%03hu Gs\r\n", HAL_GetTick() / 10ul, unit, decimal);
    uart_print(msg);

    // Check if sensor value is in range
    if (unit < LOWER_SENSOR_THRESHOLD) {
      uart_print(LOWER_SENSOR_THRESHOLD_MSG);
    }
    else if (unit > UPPER_SENSOR_THRESHOLD) {
      uart_print(UPPER_SENSOR_THRESHOLD_MSG);
    }
  }

  // Read system voltage
  if (can_read_voltage) {
		read_voltage();
    can_read_voltage = FALSE;

    // Print voltage value
    char msg[40];
    snprintf(msg, 40, "[%10lu] Voltage: %hu mV\r\n", HAL_GetTick() / 10ul, voltage);
    uart_print(msg);

    // Check if voltage is in range
    if (voltage < UNDERVOLTAGE || voltage > OVERVOLTAGE) {
        uart_print(DANGER_MSG);
        current_state = DANGER;
    }
  }
}

void read_voltage(void) {
  HAL_ADC_Start(&VOLTAGE_ADC);
  HAL_StatusTypeDef voltage_status = HAL_ADC_PollForConversion(&VOLTAGE_ADC, VOLTAGE_POLL_TIMEOUT);
  HAL_ADC_Stop(&VOLTAGE_ADC);

  switch (voltage_status)
  {
    case HAL_OK:
      voltage = HAL_ADC_GetValue(&VOLTAGE_ADC) / 4096.0 * MAX_VOLTAGE_VALUE;
      break;
    case HAL_ERROR:
      uart_print(VOLTAGE_ERROR_MSG);
      break;
    case HAL_BUSY:
      uart_print(VOLTAGE_BUSY_MSG);
      break;
    case HAL_TIMEOUT:
      break;
  }
}
void read_sensor(void) {
  HAL_ADC_Start(&SENSOR_ADC);
  HAL_StatusTypeDef sensor_status = HAL_ADC_PollForConversion(&SENSOR_ADC, SENSOR_POLL_TIMEOUT);
  HAL_ADC_Stop(&SENSOR_ADC);

  switch (sensor_status)
  {
    case HAL_OK:
      sensor_val = HAL_ADC_GetValue(&SENSOR_ADC) / 4096.0 * MAX_SENSOR_VALUE;
      break;
    case HAL_ERROR:
      uart_print(SENSOR_ERROR_MSG);
      break;
    case HAL_BUSY:
      uart_print(SENSOR_BUSY_MSG);
      break;
    case HAL_TIMEOUT:
      break;
  }
}

// Set undervoltage led
void set_under_led(bool_t val) {
  HAL_GPIO_WritePin(UNDER_LED_PERIPHERAL, UNDER_LED_PIN, val);    
}
// Set overvoltage led
void set_over_led(bool_t val) {
  HAL_GPIO_WritePin(OVER_LED_PERIPHERAL, OVER_LED_PIN, val);    
}

void uart_print(const char* msg) {
	HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), UART_TRANSMIT_TIMEOUT);
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

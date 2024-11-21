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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cli.h"
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

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* Definitions for ExecuteCommand */
osThreadId_t ExecuteCommandHandle;
const osThreadAttr_t ExecuteCommand_attributes = {
  .name = "ExecuteCommand",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for PWM */
osThreadId_t PWMHandle;
const osThreadAttr_t PWM_attributes = {
  .name = "PWM",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

// Flags and indexes
volatile uint8_t enterPressed = 0;

// Stores the last received character
uint8_t textChar[2];
// rxBuffer index
uint8_t rxIdx;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
void ExecuteCommand(void *argument);
void StartPWM(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
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
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  // HAL_UART_Receive_IT(&huart3, textChar, 1);
  HAL_UART_Receive_IT(&huart2, textChar, 1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  // Set the initial angle of the servo motor to 0°
  uint8_t pulse = (uint8_t)((DUTY_CYCLE_LOWER_BOUND / 100.0) * 255);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pulse);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of ExecuteCommand */
  ExecuteCommandHandle = osThreadNew(ExecuteCommand, NULL, &ExecuteCommand_attributes);
  /* creation of PWM */
  PWMHandle = osThreadNew(StartPWM, NULL, &PWM_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_USART3
                              |RCC_PERIPHCLK_TIM34;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.Tim34ClockSelection = RCC_TIM34CLK_HCLK;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
    Error_Handler();
  }
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void) {
  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 5625;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 255;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
    Error_Handler();
  }

  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void) {
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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

  if (HAL_UART_Init(&huart2) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */
}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void) {
  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

  if (HAL_UART_Init(&huart3) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, DMUX_C_Pin|DMUX_B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MUX_B_Pin|MUX_C_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DMUX_A_Pin|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DMUX_C_Pin DMUX_B_Pin */
  GPIO_InitStruct.Pin = DMUX_C_Pin|DMUX_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : MUX_A_Pin */
  GPIO_InitStruct.Pin = MUX_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MUX_A_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MUX_B_Pin MUX_C_Pin LD2_Pin */
  GPIO_InitStruct.Pin = MUX_B_Pin|MUX_C_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DMUX_A_Pin PB8 PB9 */
  GPIO_InitStruct.Pin = DMUX_A_Pin|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
 * @brief Callback function for the UART reception.
 * 
 * This function is called when a character is received from the UART3 peripheral.
 * The received character is stored in the rxBuffer and echoed back to the terminal
 * console connected to the UART2 peripheral. Supports backspace and Enter characters.
 * 
 * @param huart 
 * @retval None
 * @note The expected UART instance to receive data is USART3, which is connected to the
 *       HC-05 Bluetooth module. The UART2 instance is used to echo the received characters
 *       back to the terminal console, which corresponds to the USB virtual COM port from
 *       the Nucleo board.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  // If the interrupt is not from USART3, ignore it
  if (huart->Instance != USART2)
    return;

  // Add the received character to the buffer. Ignore Enter and Backspace
  if (textChar[0] != 13 && textChar[0] != 8) {
    rxBuffer[rxIdx++] = textChar[0];
    // Echo the received character
    // HAL_UART_Transmit(&huart2, textChar, 1, 100);
  }

  // Handle backspace. Remove the last character from the buffer
  if (textChar[0] == 8 && rxIdx > 0) {
    rxIdx--;
    rxBuffer[rxIdx] = 0;
    // Erase the last character from the terminal
    // HAL_UART_Transmit(&huart2, (uint8_t*)"\b \b", 3, 100);
  }

  // Handle Enter. Reset the buffer and handle the command
  if (textChar[0] == 13) {
    // Reset the buffer and the index
    rxIdx = 0;

    // Send the received command to the command buffer if there is enough
    // space in the buffe r
    if (cmdCount < CMD_VCTR_SIZE) {
      ParseCommand();
    }

    // Echo a newline character
    // HAL_UART_Transmit(&huart2, (uint8_t*)"\n\r", 2, 100);
    memset(rxBuffer, 0, RX_BUFFER_SIZE);
  }

  // Restart the interrupt-driven reception of a single character
  // HAL_UART_Receive_IT(&huart3, textChar, 1);
  HAL_UART_Receive_IT(&huart2, textChar, 1);
}

/**
 * @brief Selects the input channel of the Mux
 * 
 * Sets the appropriate pin values to let a signal come from the desired channel,
 * which can be 1 out 8 input channels
 * 
 * @param channel Channel to be selected
 * @retval None
 */
void SelectInputChannel(uint8_t channel) {
  // Calculate the state for each pin
  uint8_t A = (channel & 0x01); // bit 0
  uint8_t B = (channel & 0x02) >> 1; // bit 1
  uint8_t C = (channel & 0x04) >> 2; // bit 2

  // Set each pin state
  HAL_GPIO_WritePin(MUX_A_GPIO_Port, MUX_A_Pin, A ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MUX_B_GPIO_Port, MUX_B_Pin, B ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MUX_C_GPIO_Port, MUX_C_Pin, C ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * @brief Selects the output channel of the DMux.
 * 
 * Sets the appropriate pin values to pass a signal through the desired channel,
 * which can be 1 out 8 output channels
 * 
 * @param channel Channel to be selected
 * @retval None
 */
void SelectOutputChannel(uint8_t channel) {
  // Calculate the state for each pin
  uint8_t A = (channel & 0x01); // bit 0
  uint8_t B = (channel & 0x02) >> 1; // bit 1
  uint8_t C = (channel & 0x04) >> 2; // bit 2

  // Set each pin state
  HAL_GPIO_WritePin(DMUX_A_GPIO_Port, DMUX_A_Pin, A ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DMUX_B_GPIO_Port, DMUX_B_Pin, B ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DMUX_C_GPIO_Port, DMUX_C_Pin, C ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
/* USER CODE END 4 */

/**
 * @brief Function implementing the ExecuteCommand thread.
 * 
 * @param argument 
 */
void ExecuteCommand(void *argument) {
  while (1) {
    // Ignore if there is no command in the buffer
    if (cmdCount == 0)
      continue;

    CommandVector *cmdVctr = &circBuffer[cmdReadIdx];

    if (strcmp(cmdVctr->cmdName, "PWM") == 0) {
      ExecutePWM(cmdVctr);
    }

    cmdReadIdx = (cmdReadIdx + 1) % CIRC_BUFFER_SIZE;
    cmdCount--;
  }
}

/* USER CODE BEGIN Header_StartPWM */
/**
* @brief Function implementing the PWM thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPWM */
void StartPWM(void *argument) {
  /* USER CODE BEGIN StartPWM */
  /* Infinite loop */
  while (1) {
    // // Ignore id there is no command in the buffer
    // if (cmdCount == 0)
    //   continue;

    // if (strcmp(cmdVctr[cmdReadIdx][0], "PWM") == 0) {
    //   // Get the ID of the actuator
    //   char id[ARG_LENGTH];
    //   strcpy(id, cmdVctr[cmdReadIdx][1]);

    //   // Get the type of signal
    //   char param[ARG_LENGTH];
    //   strcpy(param, cmdVctr[cmdReadIdx][2]);
    //   float dutyCycle;

    //   // Check if the input is a percentage or a duty cycle value
    //   if (strcmp(param, "-p") == 0) {
    //     dutyCycle = atof(cmdVctr[cmdReadIdx][3]);
    //     dutyCycle = map(dutyCycle, 0, 100, DUTY_CYCLE_LOWER_BOUND, DUTY_CYCLE_UPPER_BOUND);
    //   } else {
    //     dutyCycle = atof(param);
    //     dutyCycle = clamp(dutyCycle, DUTY_CYCLE_LOWER_BOUND, DUTY_CYCLE_UPPER_BOUND);
    //   }

    //   // Select the MUX input channel in zero-index notation
    //   SelectInputChannel(0);

    //   // Select the DMux output channel in zero-index notation
    //   SelectOutputChannel((uint8_t) atoi(id) - 1);

    //   // Calculate the pulse width and set the duty cycle of the signal
    //   uint8_t pulse = (uint8_t)((dutyCycle / 100.0) * 255);
    //   __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pulse);

    //   // Increase the read index and decrease the command count
    //   cmdReadIdx = (cmdReadIdx + 1) % CMD_VCTR_SIZE;
    //   cmdCount--;
    // }
  }
  /* USER CODE END StartPWM */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1);
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

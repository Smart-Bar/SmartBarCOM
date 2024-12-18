/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define DMUX_C_Pin GPIO_PIN_0
#define DMUX_C_GPIO_Port GPIOC
#define DMUX_B_Pin GPIO_PIN_1
#define DMUX_B_GPIO_Port GPIOC
#define MUX_C_Pin GPIO_PIN_0
#define MUX_C_GPIO_Port GPIOA
#define MUX_B_Pin GPIO_PIN_1
#define MUX_B_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define MUX_A_Pin GPIO_PIN_4
#define MUX_A_GPIO_Port GPIOA
#define DMUX_A_Pin GPIO_PIN_0
#define DMUX_A_GPIO_Port GPIOB
#define PWM2_Pin GPIO_PIN_14
#define PWM2_GPIO_Port GPIOB
#define ENC_B_Pin GPIO_PIN_6
#define ENC_B_GPIO_Port GPIOC
#define PWM1_Pin GPIO_PIN_7
#define PWM1_GPIO_Port GPIOC
#define PH2_Pin GPIO_PIN_8
#define PH2_GPIO_Port GPIOC
#define PH1_Pin GPIO_PIN_9
#define PH1_GPIO_Port GPIOC
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define ENC_A_Pin GPIO_PIN_8
#define ENC_A_GPIO_Port GPIOB
#define ENC_A_EXTI_IRQn EXTI9_5_IRQn
#define PID_Pin GPIO_PIN_9
#define PID_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

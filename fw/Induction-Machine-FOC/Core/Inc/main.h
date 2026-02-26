/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32g4xx_hal.h"

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
#define PHASE_B_CURR_Pin GPIO_PIN_0
#define PHASE_B_CURR_GPIO_Port GPIOC
#define PHASE_C_CURR_Pin GPIO_PIN_1
#define PHASE_C_CURR_GPIO_Port GPIOC
#define VTSO_Pin GPIO_PIN_2
#define VTSO_GPIO_Port GPIOC
#define PHASE_A_CURR_Pin GPIO_PIN_0
#define PHASE_A_CURR_GPIO_Port GPIOA
#define VBUS_SENS_Pin GPIO_PIN_1
#define VBUS_SENS_GPIO_Port GPIOA
#define FAULT_Pin GPIO_PIN_6
#define FAULT_GPIO_Port GPIOA
#define UL_PWM_Pin GPIO_PIN_7
#define UL_PWM_GPIO_Port GPIOA
#define VL_PWM_Pin GPIO_PIN_0
#define VL_PWM_GPIO_Port GPIOB
#define WL_PWM_Pin GPIO_PIN_1
#define WL_PWM_GPIO_Port GPIOB
#define RELAY1_Pin GPIO_PIN_11
#define RELAY1_GPIO_Port GPIOB
#define RELAY2_Pin GPIO_PIN_12
#define RELAY2_GPIO_Port GPIOB
#define RELAY6_Pin GPIO_PIN_6
#define RELAY6_GPIO_Port GPIOC
#define RELAY7_Pin GPIO_PIN_7
#define RELAY7_GPIO_Port GPIOC
#define RELAY8_Pin GPIO_PIN_8
#define RELAY8_GPIO_Port GPIOC
#define RELAY9_Pin GPIO_PIN_9
#define RELAY9_GPIO_Port GPIOC
#define UH_PWM_Pin GPIO_PIN_8
#define UH_PWM_GPIO_Port GPIOA
#define VH_PWM_Pin GPIO_PIN_9
#define VH_PWM_GPIO_Port GPIOA
#define WH_PWM_Pin GPIO_PIN_10
#define WH_PWM_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

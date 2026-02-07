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

void HAL_HRTIM_MspPostInit(HRTIM_HandleTypeDef *hhrtim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

// Motor control functions for 3-phase AC induction motor
void Motor_Start(void);
void Motor_Stop(void);
void Motor_SetFrequency(float frequency);
void Motor_SetAmplitude(float amplitude);
void Motor_Update(void);
void Update_3Phase_PWM(float phase, float amplitude);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RELAY10_Pin GPIO_PIN_13
#define RELAY10_GPIO_Port GPIOC
#define RELAY9_Pin GPIO_PIN_14
#define RELAY9_GPIO_Port GPIOC
#define RCC_OSC_IN_Pin GPIO_PIN_0
#define RCC_OSC_IN_GPIO_Port GPIOF
#define RCC_OSC_OUT_Pin GPIO_PIN_1
#define RCC_OSC_OUT_GPIO_Port GPIOF
#define RELAY4_Pin GPIO_PIN_1
#define RELAY4_GPIO_Port GPIOB
#define RELAY5_Pin GPIO_PIN_2
#define RELAY5_GPIO_Port GPIOB
#define RELAY1_Pin GPIO_PIN_10
#define RELAY1_GPIO_Port GPIOA
#define RELAY7_Pin GPIO_PIN_11
#define RELAY7_GPIO_Port GPIOA
#define RELAY8_Pin GPIO_PIN_12
#define RELAY8_GPIO_Port GPIOA
#define RELAY2_Pin GPIO_PIN_3
#define RELAY2_GPIO_Port GPIOB
#define RELAY3_Pin GPIO_PIN_5
#define RELAY3_GPIO_Port GPIOB
#define RELAY6_Pin GPIO_PIN_6
#define RELAY6_GPIO_Port GPIOB
#define RELAY11_Pin GPIO_PIN_7
#define RELAY11_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

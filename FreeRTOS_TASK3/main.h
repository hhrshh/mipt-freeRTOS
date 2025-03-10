/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Task1_Pin GPIO_PIN_1
#define Task1_GPIO_Port GPIOA
#define Task2_Pin GPIO_PIN_2
#define Task2_GPIO_Port GPIOA
#define Task3_Pin GPIO_PIN_3
#define Task3_GPIO_Port GPIOA
#define TaskIdle_Pin GPIO_PIN_4
#define TaskIdle_GPIO_Port GPIOA
#define LA_Pin GPIO_PIN_0
#define LA_GPIO_Port GPIOB
#define LB_Pin GPIO_PIN_1
#define LB_GPIO_Port GPIOB
#define LC_Pin GPIO_PIN_2
#define LC_GPIO_Port GPIOB
#define L3_Pin GPIO_PIN_10
#define L3_GPIO_Port GPIOB
#define LD_Pin GPIO_PIN_12
#define LD_GPIO_Port GPIOB
#define LE_Pin GPIO_PIN_4
#define LE_GPIO_Port GPIOB
#define LF_Pin GPIO_PIN_5
#define LF_GPIO_Port GPIOB
#define LG_Pin GPIO_PIN_6
#define LG_GPIO_Port GPIOB
#define LP_Pin GPIO_PIN_7
#define LP_GPIO_Port GPIOB
#define L1_Pin GPIO_PIN_8
#define L1_GPIO_Port GPIOB
#define L2_Pin GPIO_PIN_9
#define L2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

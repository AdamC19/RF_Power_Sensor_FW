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
#include "stm32f0xx_hal.h"

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
#define DISP2_B_Pin GPIO_PIN_13
#define DISP2_B_GPIO_Port GPIOC
#define DISP2_G_Pin GPIO_PIN_14
#define DISP2_G_GPIO_Port GPIOC
#define DISP2_C_Pin GPIO_PIN_15
#define DISP2_C_GPIO_Port GPIOC
#define DISP2_D_Pin GPIO_PIN_0
#define DISP2_D_GPIO_Port GPIOF
#define DISP2_E_Pin GPIO_PIN_1
#define DISP2_E_GPIO_Port GPIOF
#define ADC_DRDY_Pin GPIO_PIN_0
#define ADC_DRDY_GPIO_Port GPIOA
#define ADC_DRDY_EXTI_IRQn EXTI0_1_IRQn
#define BTN_3_Pin GPIO_PIN_1
#define BTN_3_GPIO_Port GPIOA
#define BTN_2_Pin GPIO_PIN_2
#define BTN_2_GPIO_Port GPIOA
#define BTN_1_Pin GPIO_PIN_3
#define BTN_1_GPIO_Port GPIOA
#define ADC_CS_Pin GPIO_PIN_4
#define ADC_CS_GPIO_Port GPIOA
#define DISP2_DP_Pin GPIO_PIN_0
#define DISP2_DP_GPIO_Port GPIOB
#define DISP1_DP_Pin GPIO_PIN_1
#define DISP1_DP_GPIO_Port GPIOB
#define CHAR_3_Pin GPIO_PIN_8
#define CHAR_3_GPIO_Port GPIOA
#define CHAR_2_Pin GPIO_PIN_11
#define CHAR_2_GPIO_Port GPIOA
#define CHAR_1_Pin GPIO_PIN_12
#define CHAR_1_GPIO_Port GPIOA
#define CHAR_4_Pin GPIO_PIN_6
#define CHAR_4_GPIO_Port GPIOF
#define DISP1_A_Pin GPIO_PIN_7
#define DISP1_A_GPIO_Port GPIOF
#define DISP1_F_Pin GPIO_PIN_15
#define DISP1_F_GPIO_Port GPIOA
#define DISP1_B_Pin GPIO_PIN_3
#define DISP1_B_GPIO_Port GPIOB
#define DISP1_G_Pin GPIO_PIN_4
#define DISP1_G_GPIO_Port GPIOB
#define DISP1_C_Pin GPIO_PIN_5
#define DISP1_C_GPIO_Port GPIOB
#define DISP1_D_Pin GPIO_PIN_6
#define DISP1_D_GPIO_Port GPIOB
#define DISP1_E_Pin GPIO_PIN_7
#define DISP1_E_GPIO_Port GPIOB
#define DISP2_A_Pin GPIO_PIN_8
#define DISP2_A_GPIO_Port GPIOB
#define DISP2_F_Pin GPIO_PIN_9
#define DISP2_F_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

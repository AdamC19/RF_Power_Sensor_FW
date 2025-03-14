/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
        * Free pins are configured automatically as Analog (this feature is enabled through
        * the Code Generation settings)
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, DISP2_B_Pin|DISP2_G_Pin|DISP2_C_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, DISP2_D_Pin|DISP2_E_Pin|CHAR_4_Pin|DISP1_A_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ADC_CS_Pin|CHAR_3_Pin|CHAR_2_Pin|CHAR_1_Pin
                          |DISP1_F_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DISP2_DP_Pin|DISP1_DP_Pin|DISP1_B_Pin|DISP1_G_Pin
                          |DISP1_C_Pin|DISP1_D_Pin|DISP1_E_Pin|DISP2_A_Pin
                          |DISP2_F_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DISP2_B_Pin DISP2_G_Pin DISP2_C_Pin */
  GPIO_InitStruct.Pin = DISP2_B_Pin|DISP2_G_Pin|DISP2_C_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : DISP2_D_Pin DISP2_E_Pin CHAR_4_Pin DISP1_A_Pin */
  GPIO_InitStruct.Pin = DISP2_D_Pin|DISP2_E_Pin|CHAR_4_Pin|DISP1_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : ADC_DRDY_Pin */
  GPIO_InitStruct.Pin = ADC_DRDY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ADC_DRDY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BTN_3_Pin BTN_2_Pin BTN_1_Pin */
  GPIO_InitStruct.Pin = BTN_3_Pin|BTN_2_Pin|BTN_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ADC_CS_Pin CHAR_3_Pin CHAR_2_Pin CHAR_1_Pin
                           DISP1_F_Pin */
  GPIO_InitStruct.Pin = ADC_CS_Pin|CHAR_3_Pin|CHAR_2_Pin|CHAR_1_Pin
                          |DISP1_F_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DISP2_DP_Pin DISP1_DP_Pin DISP1_B_Pin DISP1_G_Pin
                           DISP1_C_Pin DISP1_D_Pin DISP1_E_Pin DISP2_A_Pin
                           DISP2_F_Pin */
  GPIO_InitStruct.Pin = DISP2_DP_Pin|DISP1_DP_Pin|DISP1_B_Pin|DISP1_G_Pin
                          |DISP1_C_Pin|DISP1_D_Pin|DISP1_E_Pin|DISP2_A_Pin
                          |DISP2_F_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB10 PB11 PB12
                           PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12
                          |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */

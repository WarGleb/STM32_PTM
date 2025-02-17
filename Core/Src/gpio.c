/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
     PC9   ------> RCC_MCO_2
     PA8   ------> RCC_MCO_1
*/
void MX_GPIO_Init(void)
{

	GPIO_InitTypeDef GPIO_InitStruct = {0};

	  /* GPIO Ports Clock Enable */
	  __HAL_RCC_GPIOE_CLK_ENABLE();
	  __HAL_RCC_GPIOF_CLK_ENABLE();
	  __HAL_RCC_GPIOH_CLK_ENABLE();
	  __HAL_RCC_GPIOC_CLK_ENABLE();
	  __HAL_RCC_GPIOA_CLK_ENABLE();
	  __HAL_RCC_GPIOG_CLK_ENABLE();
	  __HAL_RCC_GPIOB_CLK_ENABLE();
	  __HAL_RCC_GPIOD_CLK_ENABLE();

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(GPIOG, UART7_REDE_Pin|PUMP_OUT_CTRL_Pin|HV_PUMP_LOCK_Pin|RSVD3_Pin
	                          |RSVD1_Pin|RSVD0_Pin|CS_DAC0_Pin, GPIO_PIN_RESET);

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(ETH_nRST_GPIO_Port, ETH_nRST_Pin, GPIO_PIN_RESET);

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(GPIOD, USART3_REDE_Pin|SYS_LED_Pin|CS_ADC0_3_Pin|RST_ADC0_3_Pin, GPIO_PIN_RESET);

	  /*Configure GPIO pins : PEPin PEPin */
	  GPIO_InitStruct.Pin = INSW1_Pin|INSW0_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_PULLUP;
	  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	  /*Configure GPIO pins : PGPin PGPin PGPin PGPin
	                           PGPin PGPin PGPin */
	  GPIO_InitStruct.Pin = UART7_REDE_Pin|PUMP_OUT_CTRL_Pin|HV_PUMP_LOCK_Pin|RSVD3_Pin
	                          |RSVD1_Pin|RSVD0_Pin|CS_DAC0_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	  /*Configure GPIO pin : PtPin */
	  GPIO_InitStruct.Pin = ETH_nRST_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(ETH_nRST_GPIO_Port, &GPIO_InitStruct);

	  /*Configure GPIO pins : PDPin PDPin PDPin PDPin */
	  GPIO_InitStruct.Pin = USART3_REDE_Pin|SYS_LED_Pin|CS_ADC0_3_Pin|RST_ADC0_3_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	  /*Configure GPIO pin : PtPin */
	  GPIO_InitStruct.Pin = HVP_CLIM_CMP_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	  GPIO_InitStruct.Pull = GPIO_PULLUP;
	  HAL_GPIO_Init(HVP_CLIM_CMP_GPIO_Port, &GPIO_InitStruct);

	  /*Configure GPIO pin : PA8 */
	  GPIO_InitStruct.Pin = GPIO_PIN_8;
	  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	  /*Configure GPIO pins : PDPin PDPin PDPin PDPin */
	  GPIO_InitStruct.Pin = ALARM_ADC0_Pin|ALARM_ADC1_Pin|ALARM_ADC2_Pin|ALARM_ADC3_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	  GPIO_InitStruct.Pull = GPIO_PULLUP;
	  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	  /* EXTI interrupt init*/
	  HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
	  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

	  HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
	  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

	  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
	  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

	  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
	  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

#define RMII_MDC_Pin 			GPIO_PIN_1
#define RMII_MDC_GPIO_Port 		GPIOC
#define RMII_REF_CLK_Pin 		GPIO_PIN_1
#define RMII_REF_CLK_GPIO_Port 	GPIOA
#define RMII_MDIO_Pin 			GPIO_PIN_2
#define RMII_MDIO_GPIO_Port 	GPIOA
#define RMII_CRS_DV_Pin 		GPIO_PIN_7
#define RMII_CRS_DV_GPIO_Port 	GPIOA
#define RMII_RXD0_Pin 			GPIO_PIN_4
#define RMII_RXD0_GPIO_Port 	GPIOC
#define RMII_RXD1_Pin 			GPIO_PIN_5
#define RMII_RXD1_GPIO_Port 	GPIOC
#define RMII_TX_EN_Pin 			GPIO_PIN_11
#define RMII_TX_EN_GPIO_Port 	GPIOB
#define RMII_TXD0_Pin 			GPIO_PIN_12
#define RMII_TXD0_GPIO_Port 	GPIOB
#define RMII_TXD1_Pin 			GPIO_PIN_13
#define RMII_TXD1_GPIO_Port 	GPIOB
#define ETH_nRST_Pin 			GPIO_PIN_15
#define ETH_nRST_GPIO_Port 		GPIOE


#define SYS_LED_Pin 			GPIO_PIN_13
#define SYS_LED_GPIO_Port 		GPIOD

#define INSW1_Pin 				GPIO_PIN_3
#define INSW1_GPIO_Port 		GPIOE
#define INSW0_Pin 				GPIO_PIN_4
#define INSW0_GPIO_Port 		GPIOE
#define ENDEBUG_UART3_Pin		GPIO_PIN_3
#define ENDEBUG_UART3_GPIO_Port	GPIOE
#define PCB_ADDR_LOCK_Pin 		GPIO_PIN_4
#define PCB_ADDR_LOCK_GPIO_Port GPIOE

#define USART3_REDE_Pin 		GPIO_PIN_10
#define USART3_REDE_GPIO_Port 	GPIOD
#define USART3_TX_Pin 			GPIO_PIN_8
#define USART3_TX_GPIO_Port 	GPIOD
#define USART3_RX_Pin 			GPIO_PIN_9
#define USART3_RX_GPIO_Port 	GPIOD

#define UART7_REDE_Pin 			GPIO_PIN_1
#define UART7_REDE_GPIO_Port 	GPIOG
#define UART7_TX_Pin 			GPIO_PIN_8
#define UART7_TX_GPIO_Port 		GPIOE
#define UART7_RX_Pin 			GPIO_PIN_7
#define UART7_RX_GPIO_Port 		GPIOE

#define PUMP_OUT_CTRL_Pin 		GPIO_PIN_2
#define PUMP_OUT_CTRL_GPIO_Port GPIOG
#define HV_PUMP_LOCK_Pin 		GPIO_PIN_4
#define HV_PUMP_LOCK_GPIO_Port 	GPIOG
#define HVP_CLIM_CMP_Pin 		GPIO_PIN_6
#define HVP_CLIM_CMP_GPIO_Port 	GPIOG
#define HVP_CLIM_CMP_EXTI_IRQn 	EXTI9_5_IRQn

#define RST_ADC0_3_Pin 			GPIO_PIN_1
#define RST_ADC0_3_GPIO_Port 	GPIOD
#define ALARM_ADC0_Pin 			GPIO_PIN_2
#define ALARM_ADC0_GPIO_Port 	GPIOD
#define ALARM_ADC0_EXTI_IRQn 	EXTI2_IRQn
#define ALARM_ADC1_Pin 			GPIO_PIN_3
#define ALARM_ADC1_GPIO_Port 	GPIOD
#define ALARM_ADC1_EXTI_IRQn 	EXTI3_IRQn
#define ALARM_ADC2_Pin 			GPIO_PIN_4
#define ALARM_ADC2_GPIO_Port 	GPIOD
#define ALARM_ADC2_EXTI_IRQn 	EXTI4_IRQn
#define ALARM_ADC3_Pin 			GPIO_PIN_5
#define ALARM_ADC3_GPIO_Port 	GPIOD
#define ALARM_ADC3_EXTI_IRQn 	EXTI9_5_IRQn
#define RSVD3_Pin 				GPIO_PIN_9
#define RSVD3_GPIO_Port 		GPIOG
#define RSVD1_Pin 				GPIO_PIN_10
#define RSVD1_GPIO_Port 		GPIOG
#define RSVD0_Pin 				GPIO_PIN_11
#define RSVD0_GPIO_Port 		GPIOG

#define CS_ADC0_3_Pin 			GPIO_PIN_0
#define CS_ADC0_3_GPIO_Port 	GPIOD
#define CS_DAC0_Pin 			GPIO_PIN_12
#define CS_DAC0_GPIO_Port 		GPIOG

#define TMS_Pin 				GPIO_PIN_13
#define TMS_GPIO_Port 			GPIOA
#define TCK_Pin 				GPIO_PIN_14
#define TCK_GPIO_Port 			GPIOA



/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

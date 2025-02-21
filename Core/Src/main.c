/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "crc.h"
#include "dma.h"
#include "eth.h"
#include "i2c.h"
#include "rng.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "ads867x.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#define DEBUG_RELAY_TEST

extern TIM_HandleTypeDef* BeepTim;

/* Define the SPI handle for the ADS867x driver.
   If not defined externally, default to hspi3.
*/
#ifndef ADS867X_SPI_HANDLE
#define ADS867X_SPI_HANDLE hspi3
#endif

/* Definitions for daisy chain mode */
#ifndef NUM_DEVICES
  #define NUM_DEVICES   4                  // Number of ADS867x devices in the chain
#endif
#ifndef ADS_BYTES
  #define ADS_BYTES     (NUM_DEVICES * 4)    // Total number of bytes (4 bytes per device)
#endif

/* Public function prototypes. */
void SysTask_Init(void);
void DataExchUART_Init(void);
void DataExchCAN_Init(void);
void EthernetTask_Init(void);

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* ------------------ ADC Block Reading Function ------------------ */
/**
  * @brief Read full raw data block from the daisy chain of ADS867x devices.
  *        This function uses only functions from the ADS867x driver.
  * @param device: Pointer to ADC instance structure.
  * @param pBuffer: Pointer to the buffer where the complete data block will be stored.
  *                 The buffer size must be at least ADS_BYTES bytes.
  * @param size: Size of the buffer in bytes.
  */
static void ADS867x_ReadADC_Block(ADS867x_GInst_t* device, uint8_t *pBuffer, uint8_t size)
{
    uint16_t timeout = 10000;
    uint8_t i;

    if (size < ADS_BYTES)
    {
        /* Buffer too small – handle error as needed */
        return;
    }

    /* Fill the buffer with NOP commands for all devices */
    for (i = 0; i < ADS_BYTES; i++) {
        pBuffer[i] = ADS867x_NOP;
    }

    /* Transmit the buffer to generate clock pulses for shifting data from all devices */
    device->spi_tx(pBuffer, ADS_BYTES);
    while (*device->tx_byte_cnt > 0)
    {
        if (--timeout == 0) break;
    }

    // Optionally, add a short delay:
    // device->delay(1);

    timeout = 10000;
    /* Receive ADS_BYTES bytes of data */
    device->spi_rx(pBuffer, ADS_BYTES);
    while (*device->rx_byte_cnt > 0)
    {
        if (--timeout == 0) break;
    }
}
/* ------------------ End of ADC Block Reading Function ------------------ */

/* ------------------ ADC Polling Task Code ------------------ */

/* Declare a queue handle to store ADC data (the complete 16-byte block) */
QueueHandle_t adcDataQueueHandle = NULL;

/* Prototype for the ADC polling task */
void StartAdcTask(void *argument);

/**
  * @brief Create ADC task and its queue.
  * Call this function after all peripherals are initialized, before starting the scheduler.
  */
void CreateAdcTask(void)
{
    /* Create a queue to hold 10 blocks of ADC data (each block is ADS_BYTES bytes) */
    adcDataQueueHandle = xQueueCreate(10, sizeof(uint8_t) * ADS_BYTES);
    if (adcDataQueueHandle == NULL)
    {
         /* Queue creation failed */
         Error_Handler();
    }

    /* Create the ADC polling task with low priority (lowest: tskIDLE_PRIORITY) */
    if (xTaskCreate(StartAdcTask, "ADC_Task", 256, NULL, tskIDLE_PRIORITY, NULL) != pdPASS)
    {
         /* Task creation failed */
         Error_Handler();
    }
}

/**
  * @brief ADC polling task.
  * This task initializes the ADC via ADS867x_Init(), then periodically polls the entire daisy chain
  * by calling ADS867x_ReadADC_Block() (which uses only functions from the ADS867x driver).
  * The complete raw data block (ADS_BYTES bytes, e.g. 16 bytes for 4 devices) is then posted to a FreeRTOS queue.
  * The task runs at a low priority.
  */
void StartAdcTask(void *argument)
{
    ADS867x_GInst_t ads_inst;
    /* Initialize the ADC instance with pointers to SPI functions and counters.
       All operations use ADS867X_SPI_HANDLE (default hspi3).
    */
    ads_inst.tx_byte_cnt = &ADS867X_SPI_HANDLE.TxXferCount;
    ads_inst.rx_byte_cnt = &ADS867X_SPI_HANDLE.RxXferCount;
    ads_inst.delay     = HAL_Delay;
    ads_inst.spi_tx    = ADS867x_SPI_Tx;
    ads_inst.spi_rx    = ADS867x_SPI_Rx;

    /* Initialize ADC settings using the driver function */
    ADS867x_Init();

    /* Buffer to hold the complete raw data block (ADS_BYTES bytes) */
    uint8_t adcBuffer[ADS_BYTES];

    for (;;)
    {
         /* Poll the ADC chain using the ADS867x driver function */
         ADS867x_ReadADC_Block(&ads_inst, adcBuffer, ADS_BYTES);

         /* Post the full data block to the FreeRTOS queue */
         xQueueSend(adcDataQueueHandle, adcBuffer, portMAX_DELAY);

         /* Delay before next polling cycle (adjust period as necessary) */
         vTaskDelay(pdMS_TO_TICKS(100));
    }
}
/* ------------------ End of ADC Polling Task Code ------------------ */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  //MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI6_Init();
  MX_SPI3_Init();
  MX_UART7_Init();
  MX_TIM4_Init();
  MX_CRC_Init();
  MX_RNG_Init();
  MX_CAN1_Init();
  MX_SPI4_Init();

  /* Start up beep. */
#ifdef DEBUG_RELAY_TEST
  HAL_GPIO_WritePin(PUMP_OUT_CTRL_GPIO_Port, PUMP_OUT_CTRL_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(HV_PUMP_LOCK_GPIO_Port, HV_PUMP_LOCK_Pin, GPIO_PIN_SET);
#endif

  for (uint8_t i = 3; i > 0; i--) {
      HAL_TIM_PWM_Start(BeepTim, TIM_CHANNEL_1);
      HAL_Delay(150);
      HAL_TIM_PWM_Stop(BeepTim, TIM_CHANNEL_1);
      HAL_Delay(150);
  }

#ifdef DEBUG_RELAY_TEST
  HAL_GPIO_WritePin(PUMP_OUT_CTRL_GPIO_Port, PUMP_OUT_CTRL_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(HV_PUMP_LOCK_GPIO_Port, HV_PUMP_LOCK_Pin, GPIO_PIN_RESET);
#endif

  /* Call initialization functions for other FreeRTOS objects */
  SysTask_Init();
  DataExchUART_Init();
  DataExchCAN_Init();
  EthernetTask_Init();

  /* Create ADC polling task (low priority) */
  CreateAdcTask();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  while (1)
  {
  }
}

/**
  * @brief System Clock Configuration
  * @retval None.
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
      * in the RCC_OscInitTypeDef structure.
      */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 15;
    RCC_OscInitStruct.PLL.PLLN = 216;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 8;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /** Activate the Over-Drive mode */
    if (HAL_PWREx_EnableOverDrive() != HAL_OK)
    {
      Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
      Error_Handler();
    }
    HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSE, RCC_MCODIV_1);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode.
  * @note   This function is called when TIM7 interrupt occurs, inside HAL_TIM_IRQHandler().
  *         It directly calls HAL_IncTick() to increment the global "uwTick" used as application time base.
  * @param  htim : TIM handle.
  * @retval None.
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None.
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error occurred.
  * @param  file: pointer to the source file name.
  * @param  line: assert_param error line source number.
  * @retval None.
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* Example: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

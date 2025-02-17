/*
 * @brief DataExchETH.c
 *
 * Created on: Jul 16, 2023
 * Author: asw3005
 *
 */

/* Includes */
#include "stm32f4xx_hal.h"
#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"

/* Public function prototypes. */
void EthernetTask_Init(void);

/* External variables. */
extern void MX_LWIP_Init(void);

/* Semaphore handles. */
/* Queue handles. */
/* Task handles. */
TaskHandle_t Ethernet_th;
/* Timer handles. */
/* Private function prototypes. */
static void Ethernet_Ti(void* const argument);

/*  */
void EthernetTask_Init(void) {

	/* Callback's register functions. */
	/* Creating semaphores. */
	/* Creating queues. */
	/* Creating timers. */
	/* Creating tasks. */
	xTaskCreate(Ethernet_Ti, "Eth_Task", configMINIMAL_STACK_SIZE * 2, NULL, osPriorityNormal, &Ethernet_th);

}

/*
 * @brie Init ethernet interface.f
 */
static void Ethernet_Ti(void* const argument) {

	/* init code for LWIP */
	vTaskDelay(1000);
	MX_LWIP_Init();

	for(;;) {
		vTaskDelay(1000);
	}
}

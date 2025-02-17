/*
 * @brief SysTask.c
 *
 * Created on: Jul 16, 2023
 * Author: asw3005
 *
 */

/* Includes. */
#include "stm32f4xx_hal.h"
#include "main.h"
#include "Common.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"
#include "timers.h"

#define NO_ADDR_BUTTON

/* Public function prototypes. */
void SysTask_Init(void);

/* External variables. */
//extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef* ExtMemI2C;
extern TIM_HandleTypeDef htim4;
extern SemaphoreHandle_t ConSuccess_sh;
extern SemaphoreHandle_t ConAborted_sh;


/* Variables. */
TIM_HandleTypeDef* BeepTim = &htim4;

/* Semaphore's handles. */
SemaphoreHandle_t ViewAddr_sh;
SemaphoreHandle_t ClrScreen_sh;
SemaphoreHandle_t DotToggle_sh;
SemaphoreHandle_t BtnPressed_sh;
SemaphoreHandle_t BtnTimeout_sh;
SemaphoreHandle_t SoftBtnPressed_sh;

/* Queue's handles. */
QueueHandle_t ChLed_qh;
QueueHandle_t SetAddr_qh;
QueueHandle_t DisplayAddr_qh;


/* Task's handles. */
TaskHandle_t Beep_th;
TaskHandle_t Display_th;
TaskHandle_t AddrBtn_th;

/* Timer's handles. */
TimerHandle_t System_th;
TimerHandle_t Screen_th;
TimerHandle_t ConTimeout_th;

/* Private variables. */
//static I2C_HandleTypeDef* ExtMemI2C = &hi2c2;

/* Private function prototypes. */
static void AddrBtn_Ti(void* const param);
static void Display_Ti(void* const param);
static void Beep_Ti(void* const param);
static void SysTim_Callback(TimerHandle_t xTimer);
static void ScreenTim_Callback(TimerHandle_t xTimer);
static void ConTimeout_Callback(TimerHandle_t xTimer);

/*  */
void SysTask_Init(void) {

	/* Init Beep perif. */
	//BeepTim = htim1;

	/* Callback's register functions. */
	/* Creating semaphores. */
	BtnPressed_sh = xSemaphoreCreateBinary();
	BtnTimeout_sh = xSemaphoreCreateBinary();
	SoftBtnPressed_sh = xSemaphoreCreateBinary();
	ClrScreen_sh = xSemaphoreCreateBinary();
	DotToggle_sh = xSemaphoreCreateBinary();
	ViewAddr_sh = xSemaphoreCreateBinary();

	/* Creating queues. */
	ChLed_qh = xQueueCreate(6, sizeof(LedChData_t));
	SetAddr_qh = xQueueCreate(1, sizeof(uint8_t));
	DisplayAddr_qh = xQueueCreate(1, sizeof(uint8_t));


	/* Creating timers. */
	ConTimeout_th = xTimerCreate("ConTim", pdMS_TO_TICKS(DEFAULT_CONNECTION_TIMEOUT), osTimerOnce, NULL, ConTimeout_Callback);
	Screen_th = xTimerCreate("ScreenTim", pdMS_TO_TICKS(SCREEN_TIMEOUT), osTimerOnce, NULL, ScreenTim_Callback);
	System_th = xTimerCreate("SysTim", pdMS_TO_TICKS(500), osTimerPeriodic, NULL, SysTim_Callback);
	if (System_th != NULL) {
		xTimerStart(System_th, 0);
	}

	/* Creating tasks. */
	xTaskCreate(Display_Ti, "Display", configMINIMAL_STACK_SIZE, NULL, osPriorityNormal, &Display_th);
	xTaskCreate(AddrBtn_Ti, "Btn_Task", configMINIMAL_STACK_SIZE, NULL, osPriorityNormal, &AddrBtn_th);
	xTaskCreate(Beep_Ti, "Beep_Task", configMINIMAL_STACK_SIZE, NULL, osPriorityNormal, &Beep_th);
}

/*
	@brief Display task implementation.
*/
static void Display_Ti(void* const param) {

	/* Device address variable. */
	uint8_t BoardAddr = 0, PrevAddr = TYPE_UNDEFINED, isWriteDelayed = 0, isFirstPowerUp = 1;


	/* External memory data. */
	M24128_MEM_DATA_t UserData = {
		.BYTE_ADDR = DEV_ADDR_BYTE,
		.PAGE_ADDR = DEV_ADDR_PAGE
	};

	/* General screen struct instance. */

	/* Software press button for first power up to view current device address. */
	BTN_SWPress();

	for (; /*(__)*/;) {

		/* Display the device address. */
		if (xQueueReceive(DisplayAddr_qh, &BoardAddr, 0) == pdPASS) {

			/* Displaying fixed address on the screen. */
			//TLC592x_Set_Address(&TlcLedScreen0, BoardAddr);

			/* Check address changing. */
			if (BoardAddr != PrevAddr) {
				PrevAddr = BoardAddr;
				isWriteDelayed = 1;
			}
		}

		/* Toggle the dot on the LED screen. */
		if (xSemaphoreTake(DotToggle_sh, 0) == pdPASS) {
			/* Toggle dot0 on the display. */

		}

		/* Clear display after timeout. */
		if (xSemaphoreTake(ClrScreen_sh, 0) == pdPASS) {
			//TLC592x_Clr_Display(&TlcLedScreen0);

			/* EEPROM delayed write handler. */
			if (isWriteDelayed) {
				if (!isFirstPowerUp || PrevAddr == 0) {
					isWriteDelayed = 0;
					HAL_I2C_Mem_Write(ExtMemI2C, EEP_DATA_ADDR_SHIFTED, UserData.MemAddr, I2C_MEMADD_SIZE_16BIT, &PrevAddr, 1, 10);
				}
				isFirstPowerUp = 0;
			}
		}
	}
}

/*
 * @brief Address button read.
 */
static void AddrBtn_Ti(void* const param) {

	/* Device address and state (number of how many the button pressed.) */
	uint8_t BoardAddr = 0, PeekAddr = 0;
	uint8_t state = 0;

	/* External memory data. */
	M24128_MEM_DATA_t UserData = {
		.BYTE_ADDR = DEV_ADDR_BYTE,
		.PAGE_ADDR = DEV_ADDR_PAGE
	};

	/* Reading saved device address. */
	HAL_I2C_Mem_Read(ExtMemI2C, EEP_DATA_ADDR_SHIFTED, UserData.MemAddr, I2C_MEMADD_SIZE_16BIT, &BoardAddr, 1, 10);

	/* Address error check. */
	if (BoardAddr > MAX_MPCB_NUMBER) {
		//xTimerChangePeriod(System_th, 50, 0);
		BoardAddr = 0;
	}

	for (;/*__*/;) {

		xSemaphoreTake(BtnPressed_sh, portMAX_DELAY);
		vTaskDelay(150);
		#ifndef NO_ADDR_BUTTON
		if (!HAL_GPIO_ReadPin(ADDR_SEL_GPIO_Port, ADDR_SEL_Pin) || xSemaphoreTake(SoftBtnPressed_sh, 0) == pdPASS) {
		#else
		if (xSemaphoreTake(SoftBtnPressed_sh, 0) == pdPASS) {
		#endif /* NO_ADDR_BUTTON */
			/* If timeout timer activated we'll give state of variable to zero. */
			if (xSemaphoreTake(BtnTimeout_sh, 0) == pdPASS) { state = 0; }
			if (state == 1) { state = 2; }
			if (state != 2) { state = 1; }

			/* Blocking the multiple address change. */
			if (xSemaphoreTake(ViewAddr_sh, 0) == pdPASS) {
				state = 1;
			}

			/* Is there a remote address request? */
			if (xQueuePeek(SetAddr_qh, &PeekAddr, 0) == pdPASS) {
				state = 2;
			}

			/**/
			#ifndef NO_ADDR_BUTTON
			do {
			#endif /* NO_ADDR_BUTTON */
				if (state == 2) {
					/* Check hardware address change lock. */
					if (HAL_GPIO_ReadPin(PCB_ADDR_LOCK_GPIO_Port, PCB_ADDR_LOCK_Pin)) {
						BoardAddr++;
						/* Remote address change. */
						xQueueReceive(SetAddr_qh, &BoardAddr, 0);
						if (BoardAddr > MAX_MPCB_NUMBER) { BoardAddr = 1; }
					}
				}
				vTaskDelay(200);
				xQueueSendToBack(DisplayAddr_qh, &BoardAddr, 0);
			#ifndef NO_ADDR_BUTTON
			} while (!HAL_GPIO_ReadPin(ADDR_SEL_GPIO_Port, ADDR_SEL_Pin));
			#endif /* NO_ADDR_BUTTON */
		}
		xTimerStart(Screen_th, 0);
	}
}


/*
	@brief Sound of MBS, three times beep.
*/
static void Beep_Ti(void* const param) {

	for (;/*)_(*/;) {

		/* Beep break connection. */
		if (xSemaphoreTake(ConAborted_sh, 0) == pdPASS) {
			for (uint8_t i = 3; i > 0; i--) {
				HAL_TIM_PWM_Start(BeepTim, TIM_CHANNEL_1);
				vTaskDelay(100);
				HAL_TIM_PWM_Stop(BeepTim, TIM_CHANNEL_1);
				vTaskDelay(100);
			}
		}

		/* Beep of successful connection. */
		if (xSemaphoreTake(ConSuccess_sh, 0) == pdPASS) {
			for (uint8_t i = 2; i > 0; i--) {
				HAL_TIM_PWM_Start(BeepTim, TIM_CHANNEL_1);
				vTaskDelay(100);
				HAL_TIM_PWM_Stop(BeepTim, TIM_CHANNEL_1);
				vTaskDelay(100);
			}
		}
	}
}

/*
	@brief Led timer callback.
*/
static void ScreenTim_Callback(TimerHandle_t xTimer)
{
	xSemaphoreGive(ClrScreen_sh);
	xSemaphoreGive(BtnTimeout_sh);
}

/*
 * @brief System timer callback.
 */
static void SysTim_Callback(TimerHandle_t xTimer) {

	/* Toggling dot on the LED screen. */
	xSemaphoreGive(DotToggle_sh);

	/* Toggling the sys led on the board. */
	HAL_GPIO_TogglePin(SYS_LED_GPIO_Port, SYS_LED_Pin);
}

/*
 * @brief Connection timeout timer callback.
 */
static void ConTimeout_Callback(TimerHandle_t xTimer) {
	xSemaphoreGive(ConAborted_sh);
}

/* Interrupt callbacks. */

/*
	@brief  EXTI line detection callbacks.
	@param  GPIO_Pin: Specifies the pins connected EXTI line
	@retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	if (GPIO_PIN_6 == GPIO_Pin) {
		xSemaphoreGiveFromISR(BtnPressed_sh, NULL);
	}
}



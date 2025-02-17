/*
 * @brief DataExchUART.c
 *
 *  Created on: Jul 16, 2023
 *  Author: asw3005
 *
 */

#include "stm32f4xx_hal.h"
#include "main.h"
#include "Common.h"
#include "stdio.h"
#include "stdlib.h"
#include "inttypes.h"
#include "DataExchUART.h"
#include "CRCCalc.h"

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"

/* Public function prototypes. */
void DataExchUART_Init(void);

/* External variables. */
extern I2C_HandleTypeDef 	hi2c2;
extern UART_HandleTypeDef 	huart3;
extern DMA_HandleTypeDef 	hdma_usart3_rx;
extern QueueHandle_t 		SetAddr_qh;
extern QueueHandle_t 		CanDataTx_qh;
extern TaskHandle_t 		Beep_th;
extern TimerHandle_t 		ConTimeout_th;
extern TIM_HandleTypeDef* 	BeepTim;

I2C_HandleTypeDef* ExtMemI2C = &hi2c2;

/* Semaphore handles. */
SemaphoreHandle_t 			ConSuccess_sh;
SemaphoreHandle_t 			ConAborted_sh;
SemaphoreHandle_t 			ConTimeout_sh;
SemaphoreHandle_t 			UART3DMA_EndOfTx_sh;
SemaphoreHandle_t 			UART3DMA_RxBuffFull_sh;
SemaphoreHandle_t 			UART3_msh;

/* Queue handles. */
QueueHandle_t 				CmdRxPool_qh;

/* Task handles. */
TaskHandle_t 				SysCmdMgmt_th;
TaskHandle_t 				CmdParse_th;


/* Timer handles. */
/* Private data structs. */
//static I2C_HandleTypeDef* ExtMemI2C = &hi2c2;
static UART_HandleTypeDef* CmdInOutUART = &huart3;
static UartTxRxData_t UartTxRxData = { 0 };

/* Private function prototypes. */
static void SysCmdMgmt_Ti(void* const param);
static void CmdParse_Ti(void* const param);

/* Callbacks. */
static void UART_RxFullCallback(UART_HandleTypeDef *huart);
static void UART_TxFullCallback(UART_HandleTypeDef *huart);

//static void UART_RxFullCallback(DMA_HandleTypeDef* hdma);

static void CmdParse_Error(UartTxRxData_t *param);
static void CmdParse_FlushBuff(UartTxRxData_t* param);
static int16_t CmdParse_SymbCnt(UartTxRxData_t* param, char key_symbol, uint8_t start_pos);
static int16_t CmdParse_RAWSymbCnt(char* data, uint8_t size, char key_symbol, uint8_t start_pos, uint8_t match_index);



/*
 * @brief Init UART data and command exchange tasks.
 */
void DataExchUART_Init(void) {

	/* Callback's register functions. */
	HAL_UART_RegisterCallback(CmdInOutUART, HAL_UART_RX_COMPLETE_CB_ID, UART_RxFullCallback);
	HAL_UART_RegisterCallback(CmdInOutUART, HAL_UART_TX_COMPLETE_CB_ID, UART_TxFullCallback);

	/* Creating semaphores. */
	UART3DMA_EndOfTx_sh = xSemaphoreCreateBinary();
	UART3DMA_RxBuffFull_sh = xSemaphoreCreateBinary();
	UART3_msh = xSemaphoreCreateMutex();

	ConSuccess_sh = xSemaphoreCreateBinary();
	ConAborted_sh = xSemaphoreCreateBinary();
	ConTimeout_sh = xSemaphoreCreateBinary();

	/* Creating queues. */
	CmdRxPool_qh = xQueueCreate(25, sizeof(CmdCode_t));


	/* Creating timers. */
	/* Creating tasks. */
	xTaskCreate(SysCmdMgmt_Ti, "CmdMgmt", 256, NULL, osPriorityNormal, &SysCmdMgmt_th);
	xTaskCreate(CmdParse_Ti, "CmdParse", 256, NULL, osPriorityNormal, &CmdParse_th);
}

/*
	@brief
*/
static void SysCmdMgmt_Ti(void* const param) {

	/* Private variables. */
	uint8_t BoardAddr = 63;
	uint8_t isConnActive = 0;
	char char_buff[150];

	/* Data structs. */
	DEV_Data_t DevParams = { 0 };
	CmdCode_t CmdCode = { 0 };

	/* External memory data. */
	M24128_MEM_DATA_t UserData = {
		.BYTE_ADDR = DEV_ADDR_BYTE,
		.PAGE_ADDR = DEV_ADDR_PAGE
	};

	/*  */
	//HAL_Delay(150);


	for (;/*_*/;) {

		if (xQueueReceive(CmdRxPool_qh, &CmdCode, 0) == pdPASS) {
			/* Deactivate the command's selector. */
			if (xSemaphoreTake(ConTimeout_sh, 0) == pdPASS) {
				isConnActive = 0;
			}

			/* Is there a connection command? */
			if (CmdCode.CommandCode == MCS_ConnectTo) {

				/* Reading saved device address. */
				//HAL_I2C_Mem_Read(ExtMemI2C, EEP_DATA_ADDR_SHIFTED, UserData.MemAddr, I2C_MEMADD_SIZE_16BIT, &BoardAddr,	1, 10);
				/* Force the board address to zero. */
				BoardAddr = 0;

				/* Getting the received address. */
				DevParams.ConnAddr = atoi(CmdCode.CmdParams);

				if (DevParams.ConnAddr == BoardAddr) {
					isConnActive = 1;
					xTimerStart(ConTimeout_th, 0);
					UART_SendMessage((uint8_t*)&char_buff, sprintf(char_buff, "RFG_ConnectTo %d is OK.\r\n", DevParams.ConnAddr));
					xSemaphoreGive(ConSuccess_sh);
				}
			}


			/* Command code selector. */
			if (isConnActive) {
				/* To exclude non control parameters send no code. */

				switch (CmdCode.FPrefixCode) {
				/* Generator units command set. */

				/* Motherboard's command set. */
				case MCS:
					switch (CmdCode.CommandCode) {
						/* Mother PCB commands (general command set). */
					case MCS_GETSysInfo:
						UART_SendMessage((uint8_t*)&char_buff, sprintf(char_buff, "FreeRTOS Kernel V10.3.1.\r\nFirmware 01.27.24\r\nHardware V0.1\r\n"));
						break;
					case MCS_SETAddr:
						DevParams.ConnAddr = atoi(CmdCode.CmdParams);
						xQueueSendToBack(SetAddr_qh, &DevParams.ConnAddr, 0);
						BTN_SWPress();
						UART_SendMessage((uint8_t*)&char_buff, sprintf(char_buff, "Address is %d now.\r\n", DevParams.ConnAddr));
						break;
					case MCS_SetParams:
						UART_SendMessage((uint8_t*)&char_buff, sprintf(char_buff, "RFG_SetParams is OK.\r\n"));
						break;
					case MCS_GetParams:
						DEV_RWGenEepAll(EEP_READ_FROM);
						UART_SendMessage((uint8_t*)&char_buff, sprintf(char_buff,  "Conn addr %d\r\n"
															   	   	   	   	   	   "Output state %d\r\n",
																				   DevParams.ConnAddr,
																				   DevParams.OutState));
						__NOP();
						break;
					default:
						break;
					}
					break;

				/* Global command set. */
				}
			}
		}
	}

}

/*
 * @brief This function receive and process incoming command.
 */
static void CmdParse_Ti(void* const param) {

	/* Private data structs. */
	CmdCode_t CmdCode = { 0 };

	/* Take the semaphore once it's created. */
	xSemaphoreTake(UART3DMA_RxBuffFull_sh, 0);

	/* Initializing UART rx counter. */
	UartTxRxData.isBuffBoundaryOver = 0;
	//UartTxRxData.cReadIndexBuff0 = sizeof(UartTxRxData.RxDataBuff);

	/* Start uart data receiving. The DMA RX channel must be in the CIRCULAR mode! */
	HAL_UART_Receive_DMA(CmdInOutUART, &UartTxRxData.RxDataBuff[0], sizeof(UartTxRxData.RxDataBuff));

	for (;/*__*/;) {

		if (xSemaphoreTake(UART3DMA_RxBuffFull_sh, 0) == pdPASS) {
			UartTxRxData.isBuffBoundaryOver = 1;
		}

		/* Normal state for a read. */
		if (UartTxRxData.cReadIndexBuff0 < (sizeof(UartTxRxData.RxDataBuff) - hdma_usart3_rx.Instance->NDTR) ||
			((UartTxRxData.cReadIndexBuff0 > (sizeof(UartTxRxData.RxDataBuff) - hdma_usart3_rx.Instance->NDTR)) && UartTxRxData.isBuffBoundaryOver)) {	//! added () to first logic and && 01.31.23

			/* Delay for waiting a whole message. */
			HAL_Delay(10);

			if (xSemaphoreTake(UART3DMA_RxBuffFull_sh, 0) == pdPASS) {
				UartTxRxData.isBuffBoundaryOver = 1;
			}

			/* If corruption data state is occurred. */
			if (UartTxRxData.isBuffBoundaryOver && (UartTxRxData.cReadIndexBuff0 <= (sizeof(UartTxRxData.RxDataBuff) - hdma_usart3_rx.Instance->NDTR))) {
				/* 11.26.23. */
				if (!hdma_usart3_rx.Instance->NDTR) {
					CmdParse_Error(&UartTxRxData);
				}
			}
			/* Read the command to the temporary buffer before processing it. */
			else {
				while (!UartTxRxData.isSeqReadCmpl) {

					/* Flush temporary buffer. */
					for (uint8_t i = 0; i < sizeof(UartTxRxData.TmpCmdBuff); i++) {
						UartTxRxData.TmpCmdBuff[i] = 0;
					}

					while (UartTxRxData.TmpCmdBuff[UartTxRxData.cReadLimitCnt - 1] != '\r') {

						/* Reading incoming symbols to the buffer. */
						UartTxRxData.TmpCmdBuff[UartTxRxData.cReadLimitCnt] = UartTxRxData.RxDataBuff[UartTxRxData.cReadIndexBuff0];
						UartTxRxData.cReadIndexBuff0++;
						UartTxRxData.cReadLimitCnt++;

						/* Have you read enough bytes? */
						if (UartTxRxData.cReadIndexBuff0 == (sizeof(UartTxRxData.RxDataBuff) - hdma_usart3_rx.Instance->NDTR)) {
							UartTxRxData.isSeqReadCmpl = 1;
						}

						/* Command length fault. Not realized yet. */
						if (UartTxRxData.cReadLimitCnt > MAX_CMD_LENGTH) {
							UartTxRxData.isCmdLengthFault = 1;
							break;
						}
						/* If a pending read event has occurred. */
						//else if (UartTxRxData.isSeqReadCmpl && UartTxRxData.RxDataBuff[UartTxRxData.cReadIndexBuff0] != '\r') {
						//	UartTxRxData.isDelayedRead = 1;
						//	break;
						//}

						/* Check UART_RX_BUFF_SIZE for max value. */
						if (UartTxRxData.cReadIndexBuff0 == UART_RX_BUFF_SIZE) {
							if (UartTxRxData.isBuffBoundaryOver) {
								UartTxRxData.cReadIndexBuff0 = 0;
								UartTxRxData.isBuffBoundaryOver = 0;
								/* Have you read enough bytes? */
								if (UartTxRxData.cReadIndexBuff0 == (sizeof(UartTxRxData.RxDataBuff) - hdma_usart3_rx.Instance->NDTR)) {
									UartTxRxData.isSeqReadCmpl = 1;
								}
							}
						}
					}


					/* Processing the read command. */
					if (!UartTxRxData.isCmdLengthFault) {

						/*  */
						UartTxRxData.cReadLimitCnt = 0;

						/* Getting first prefix code. */
						CmdCode.FPrefixCode = Hard_CRC32ETH(&UartTxRxData.TmpCmdBuff[FPREFIX_OFFSET], CmdParse_SymbCnt(&UartTxRxData, '_', 0));

						/* Is this motherboard or general command? */
						if (CmdCode.FPrefixCode == MCS) {

							UartTxRxData.SymbParamCnt = CMD_PARAM_OFFSET;

							/* Counting the symbols of command. */
							UartTxRxData.SymbCmdCnt = CmdParse_SymbCnt(&UartTxRxData, ' ', 0);
							if (!UartTxRxData.SymbCmdCnt) {
								UartTxRxData.SymbParamCnt = 0;
								UartTxRxData.SymbCmdCnt = CmdParse_SymbCnt(&UartTxRxData, '\r', 0);
							}

							/* Getting the command code. */
							if (UartTxRxData.SymbCmdCnt) {
								CmdCode.CommandCode = Hard_CRC32ETH(&UartTxRxData.TmpCmdBuff[FPREFIX_OFFSET], UartTxRxData.SymbCmdCnt);
							}

							/* Getting the symbols of parameter. */
							if (UartTxRxData.SymbParamCnt) {
								UartTxRxData.SymbParamCnt = CmdParse_SymbCnt(&UartTxRxData, '\r', UartTxRxData.SymbCmdCnt + CMD_PARAM_OFFSET - 1);
								for (uint8_t i = 0; i < UartTxRxData.SymbParamCnt; i++) {
									CmdCode.CmdParams[i] = UartTxRxData.TmpCmdBuff[UartTxRxData.SymbCmdCnt + i + CMD_PARAM_OFFSET - 1];
								}
								/* !!!!!!! Deleted ++UartTxRxData.SymbParamCnt. */
								CmdCode.CmdParams[UartTxRxData.SymbParamCnt] = '\0';
							}
						}
						else {
							CmdCode.FPrefixCode = NO_CODE_RECEIVED;
						}

						if (CmdCode.FPrefixCode != NO_CODE_RECEIVED) {
							/* Push the date to queue, counting amount of complete correct sequences, resetting counters if necessary. */
							UartTxRxData.CmdSeqCnt++;
							xQueueSendToBack(CmdRxPool_qh, &CmdCode, 0);
						}
					}
					/* Call command length fault handler. */
					else {
						UartTxRxData.isCmdLengthFault = 0;
						CmdParse_Error(&UartTxRxData);
					}
				}
				UartTxRxData.isSeqReadCmpl = 0;
			}
		}
	}
}

/*
 * @brief Command fault or buffer overrun handler.
 *
 * @param param: UART data struct to RX/TX data.
*/
static void CmdParse_Error(UartTxRxData_t *param) {

	HAL_UART_DMAStop(CmdInOutUART);
	//HAL_Delay(100);
	param->isBuffBoundaryOver = 0;
	param->isSeqReadCmpl = 0;
	param->CmdSeqCnt = 0;
	param->cReadLimitCnt = 0;
	param->cReadIndexBuff0 = 0;
	param->isSeqReadCmpl = 1;
	hdma_usart3_rx.Instance->NDTR = sizeof(param->RxDataBuff);
	HAL_Delay(1000);
	HAL_UART_Receive_DMA(CmdInOutUART, &UartTxRxData.RxDataBuff[0], sizeof(UartTxRxData.RxDataBuff));

}

/*
	@brief Flush DAM RX buffer.

	@param param: UART data struct to RX/TX data.
*/
static void CmdParse_FlushBuff(UartTxRxData_t* param) {

	HAL_UART_DMAStop(CmdInOutUART);
	for (uint16_t i = sizeof(param->RxDataBuff) - 1; i > 0; i--) {
		param->RxDataBuff[i] = 0;
	}
	UartTxRxData.CmdSeqCnt = 0;
	HAL_UART_Receive_DMA(CmdInOutUART, &UartTxRxData.RxDataBuff[0], sizeof(UartTxRxData.RxDataBuff));
}

/*
	@brief Command symbol's counter.

	@param param: UART data struct to RX/TX data.
	@param key_symbol: this symbol is a stop position when you are looking for. Counter doesn't
	count it.
	@param start_pos: It is just a position offset.
*/
static int16_t CmdParse_SymbCnt(UartTxRxData_t* param, char key_symbol, uint8_t start_pos) {

	uint16_t SymbCnt = 0;

	while (param->TmpCmdBuff[start_pos + SymbCnt] != key_symbol) {
		SymbCnt++;
		if (SymbCnt == (sizeof(param->TmpCmdBuff) - start_pos)) {
			return 0;
		}
	}

	return SymbCnt;
}

/*
	@brief Command symbol's counter.

	@param param: Pointer to data.
	@param size: size of data struct.
	@param key_symbol: this symbol is a stop position when you are looking for.
	@param start_pos: It is just a position offset, from 0 to size - 1 of data massive.
*/
static int16_t CmdParse_RAWSymbCnt(char* data, uint8_t size, char key_symbol, uint8_t start_pos, uint8_t match_index) {

	uint16_t SymbCnt = 0;

	/* Check data boundary. */
	if (start_pos > size - 1) {
		return 0;
	}

	/* Start counting amount of symbols. */
	while (match_index) {
		if (data[start_pos + SymbCnt] == key_symbol) {
			match_index--;
		}
		SymbCnt++;
		if (SymbCnt == (size - start_pos)) {
			return 0;
		}
	}

	return SymbCnt;
}

/*
	@brief Send a simple message over UART.
*/
void UART_SendMessage(uint8_t* data, uint16_t amount) {

	xSemaphoreTake(UART3_msh, portMAX_DELAY);
	HAL_UART_Transmit(CmdInOutUART, data, amount, 25);
	xSemaphoreGive(UART3_msh);
}

/* Interrupt callbacks. */


/*
	@brief UART callback of RX transfer.
	@param  _huart : service struct of UART.
	@retval None
 */
static void UART_RxFullCallback(UART_HandleTypeDef* huart) {
	/* Stream two handler (RX USART data). */
	__NOP();
	xSemaphoreGiveFromISR(UART3DMA_RxBuffFull_sh, NULL);
};

/*
	@brief UART callback of TX transfer.
	@param  _huart : service struct of UART.
	@retval None
 */
static void UART_TxFullCallback(UART_HandleTypeDef* huart) {
	/* Stream two handler (TX USART data). */
	__NOP();
	xSemaphoreGiveFromISR(UART3DMA_EndOfTx_sh, NULL);
};







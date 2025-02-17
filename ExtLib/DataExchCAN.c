/*
 * @brief DataExchCAN.c
 *
 * Created on: Jul 16, 2023
 * Author: asw3005
 *
 */

#include "stm32f4xx_hal.h"
#include "main.h"
#include "common.h"
#include "stdio.h"
#include "stdlib.h"
#include "inttypes.h"
#include "DataExchCAN.h"
#include "DataExchUART.h"
#include "CRCCalc.h"

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "queue.h"
#include "semphr.h"


/* Public function prototypes. */
void DataExchCAN_Init(void);

/* External variables. */
extern CAN_HandleTypeDef hcan1;
extern QueueHandle_t ChLed_qh;

/* Semaphore handles. */
SemaphoreHandle_t CAN_msh;

/* Queue handles. */
QueueHandle_t CanDataRx_qh;
QueueHandle_t CanDataTx_qh;

/* Task handles. */
TaskHandle_t CmdCANTxRx_th;

/* Timer handles. */
/* Private data structs. */
static CAN_HandleTypeDef* CmdInOutCAN = &hcan1;

/* Public function prototypes. */
/* Private function prototypes. */
static void CmdCANTxRx_Ti(void* const param);
static void CAN_Config(CAN_HandleTypeDef* hcan);
static void CAN_ConfigBanks(CAN_HandleTypeDef *hcan, const uint8_t bank);

/* Callbacks. */
static void CANRxFifo0Callback(struct __CAN_HandleTypeDef* hcan);
static void CANRxFifo1Callback(struct __CAN_HandleTypeDef* hcan);

/*
 * @brief Init data and command exchange tasks.
*/
void DataExchCAN_Init(void) {

	/* Callback's register functions. */
	HAL_CAN_RegisterCallback(CmdInOutCAN, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID, CANRxFifo0Callback);
	HAL_CAN_RegisterCallback(CmdInOutCAN, HAL_CAN_RX_FIFO1_MSG_PENDING_CB_ID, CANRxFifo1Callback);

	/* Init CAN. */
	CAN_Config(CmdInOutCAN);
	CAN_ConfigBanks(CmdInOutCAN, 0);
	HAL_CAN_Start(CmdInOutCAN);

	/* Creating semaphores. */
	CAN_msh = xSemaphoreCreateMutex();

	/* Creating queues. */
	CanDataRx_qh = xQueueCreate(20, sizeof(CAN_RxQueueData_t));
	CanDataTx_qh = xQueueCreate(20, sizeof(CAN_TxQueueData_t));

	/* Creating timers. */
	/* Creating tasks. */
	xTaskCreate(CmdCANTxRx_Ti, "CmdCAN", 196, NULL, osPriorityNormal, &CmdCANTxRx_th);
}

/*
 * @brief CAN command task handler.
 */
static void CmdCANTxRx_Ti(void* const param) {

	/* Variables. */
	uint8_t isValueErr = 0;
	char char_buff[150];
	LedChData_t Leds = { 0 };

	/* Temporary text message. */
	DEV_CharBuff_t tmp_char;

	/* Data structs for CAN communication. */
	CAN_DevId_t DevId = { 0 };
	CAN_TxQueueData_t CAN_TxData = { 0 };
	CAN_RxQueueData_t CAN_RxData = { 0 };

	CAN_TxHeaderTypeDef CanTxHeader = {
	.StdId = CAN_DEFAULT_STDID,
	.ExtId = CAN_DEFAULT_EXTDID,
	.IDE = CAN_ID_STD,
	.RTR = CAN_RTR_DATA,
	.DLC = CAN_DEFAULT_DLC,
	.TransmitGlobalTime = DISABLE
	};

	/* Mailbox number. */
	uint32_t TxMailboxId;

	for (;/*_*/;) {

		if (xQueueReceive(CanDataTx_qh, &CAN_TxData, 0) == pdPASS) {

			if ((CAN_TxData.SPrefixCode == TYPE_UNDEFINED && CAN_TxData.Address < MIN_CAN_ADDR + UCS1_CMD_BASE_OFFSET - 1) || (CAN_TxData.Address > MAX_CAN_ADDR + UCS1_CMD_BASE_OFFSET - 1)) {
				UART_SendMessage((uint8_t*)& char_buff, sprintf(char_buff, "Number of channel error. It has to be from %d to %d.\r\n", MIN_CAN_ADDR, MAX_CAN_ADDR));
			} else {

				/* SGU parameters check. */
				if ((CAN_TxData.CmdCode == BCS_SETAmplitude) || (CAN_TxData.CmdCode == UCS_SETAmplitude)) {
					if ((CAN_TxData.Phase < DEV_MIN_AMPL) || (CAN_TxData.Phase > DEV_MAX_AMPL)) {
						isValueErr = 1;
						UART_SendMessage((uint8_t*)& char_buff, sprintf(char_buff, "Amplitude value error. It has to be from %d to %d.\r\n", DEV_MIN_AMPL, DEV_MAX_AMPL));
					}
				}

				if ((CAN_TxData.CmdCode == BCS_SETFrequency) || (CAN_TxData.CmdCode == UCS_SETFrequency)) {
					if ((CAN_TxData.Phase < DEV_MIN_FREQ) || (CAN_TxData.Phase > DEV_MAX_FREQ)) {
						isValueErr = 1;
						UART_SendMessage((uint8_t*)& char_buff, sprintf(char_buff, "Frequency value error. It has to be from %d to %d.\r\n", DEV_MIN_FREQ, DEV_MAX_FREQ));
					}
				}

				if ((CAN_TxData.CmdCode == BCS_SETPhase) || (CAN_TxData.CmdCode == UCS_SETPhase)) {
					if ((CAN_TxData.Phase < DEV_MIN_PHASE) || (CAN_TxData.Phase > DEV_MAX_PHASE)) {
						isValueErr = 1;
						UART_SendMessage((uint8_t*)& char_buff, sprintf(char_buff, "Phase value error. It has to be from %d to %d.\r\n", DEV_MIN_PHASE, DEV_MAX_PHASE));
					}
				}

				/* Another device parameter check. */

				/* Answering to devices. */
				if (!isValueErr /*& (CAN_TxData.DevType == HVU)*/) {

					/* Answers' selector. */
					switch (CAN_TxData.CmdCode) {
						case UCS_EnDisDevice:
							UART_SendMessage((uint8_t*)&char_buff, sprintf(char_buff, "UCS_EnDisDevice is OK.\r\n"));
							break;
						case UCS_SetChannels:
							UART_SendMessage((uint8_t*)&char_buff, sprintf(char_buff, "UCS_SetChannels is OK.\r\n"));
							break;
						case BCS_SETPhase:
							UART_SendMessage((uint8_t*)&char_buff, sprintf(char_buff, "BCS_SETPhase is OK.\r\n"));
							break;
						case BCS_SETAmplitude:
							UART_SendMessage((uint8_t*)&char_buff, sprintf(char_buff, "BCS_SETAmplitude is OK.\r\n"));
							break;
						case BCS_SETFrequency:
							UART_SendMessage((uint8_t*)&char_buff, sprintf(char_buff, "BCS_SETFrequency is OK.\r\n"));
							break;
						default:
							break;
					}

					DevId.ID_ADDRESS = CAN_TxData.Address;
					DevId.ID_TODO = CAN_TxData.Command;
					CanTxHeader.StdId = DevId.Identifier;
					/* Operational pause to send message thru the CAN. */
					HAL_Delay(50);
					HAL_CAN_AddTxMessage(CmdInOutCAN, &CanTxHeader, CAN_TxData.DevTypeParts, &TxMailboxId);
				}
				isValueErr = 0;
			}
		}

		if (xQueueReceive(CanDataRx_qh, &CAN_RxData, 0) == pdPASS) {

			/* Update the list of available devices. */
			DEV_WriteDevType(CAN_RxData.CHANNEL, CAN_RxData.Type, CAN_RxData.SUBTYPE);
			/* Update LED channels. */
			Leds.Channel = CAN_RxData.CHANNEL;
			Leds.State = CAN_RxData.STATE;
			xQueueSendToBack(ChLed_qh, &Leds, 0);

			/* State, type and subtype decoding to char. */
			DEV_NameDecode(CAN_RxData.STATE, CAN_RxData.Type, CAN_RxData.SUBTYPE, &tmp_char);

			if ((CAN_RxData.Identifier & 0x000F) == UCS_GET_STATE) {
				/* Send info to console. */
				UART_SendMessage((uint8_t*)& char_buff,
					sprintf(char_buff,
						"Device:\r\n"
					"    Address = %d\r\n"
					"    Type = %s\r\n"
					"    Subtype = %s\r\n"
					"    State = %s\r\n"
					"    Parameter = %d\r\n",
					CAN_RxData.CHANNEL, tmp_char.dev_id, tmp_char.dev_subid, tmp_char.state, CAN_RxData.GenParam));
			}
			__NOP();
		}
	}
}

/*
 * @brief
 *
 * @param hcan : pointer to a CAN_HandleTypeDef structure that contains
 *        the configuration information for the specified CAN.
 *
 **/
static void CAN_Config(CAN_HandleTypeDef* hcan) {
	/* Activate initialization mode. */
	hcan->Instance->FMR |= CAN_FMR_FINIT;


	/* These bits are not matter if the MCU has only one CAN interface. Default value is 14. */
	hcan->Instance->FMR |= 14 << CAN_FMR_CAN2SB_Pos;
	/* Enable mask mode for all banks. By defalt is identifier mask mode. */
	hcan->Instance->FM1R = 0;
	/* Filter scale is 16 bit value. By default the banks are 16 bit scale configuration. */
	hcan->Instance->FS1R = 0;
	/* FIFO assignemt. Bank zero and fourteen prefer are FIFO0, all remaining banks are in the FIFO1. */
	hcan->Instance->FFA1R = (CAN_FFA1R_FFA & ~CAN_FFA1R_FFA0 /*| CAN_FFA1R_FFA14*/);
	/* Activate specific filterbanks. */
	hcan->Instance->FA1R |= CAN_FA1R_FACT0 | CAN_FA1R_FACT14;
	/* Can interrupt configuration. */
	hcan->Instance->IER = CAN_IER_TMEIE | CAN_IER_FMPIE0 | CAN_IER_FMPIE1 | CAN_IER_ERRIE;
	/* Set-up identifiers. It does't matter here because its particular settings will set above. */
	hcan->Instance->sFilterRegister[0].FR1 = 0x00000000;
	hcan->Instance->sFilterRegister[0].FR2 = 0x00000000;
	hcan->Instance->sFilterRegister[14].FR1 = 0x00000000;
	hcan->Instance->sFilterRegister[14].FR2 = 0x00000000;

	/* Deactivate initialization mode. */
	hcan->Instance->FMR &= ~CAN_FMR_FINIT;
}

/*
 * @brief Set-up 16-bit list mode identifier to specific bank.
 *
 * @param hcan : Pointer to a CAN_HandleTypeDef structure that contains
 *        the configuration information for the specified CAN.
 * @param bank : Specifies the filter bank which will be initialized. This parameter must be a number between Min_Data = 0
 *        and Max_Data = 27.
 *
 **/
static void CAN_ConfigBanks(CAN_HandleTypeDef *hcan, const uint8_t bank) {
	CAN_FilterBankRegOrg16_t CanRegOrg16;

	/* Activate initialization mode. */
	hcan->Instance->FMR |= CAN_FMR_FINIT;

	/* Set-up identifiers. The device receive all messages. */
	CanRegOrg16.IDE = 0;
	CanRegOrg16.RTR = 0;
	CanRegOrg16.EXID18_16 = 0;
	CanRegOrg16.STID31_21 = 0;
	hcan->Instance->sFilterRegister[bank].FR1 = (uint32_t)CanRegOrg16.FilterBankReg;
	//CanRegOrg16.STID31_21 = 0;
	hcan->Instance->sFilterRegister[bank].FR1 |= (uint32_t)CanRegOrg16.FilterBankReg << 16U;

	/* Deactivate initialization mode. */
	hcan->Instance->FMR &= ~CAN_FMR_FINIT;
}

/**
  * @brief  Rx FIFO 0 message pending callback.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  *
  */
static void CANRxFifo0Callback(struct __CAN_HandleTypeDef* hcan)
{
	CAN_RxQueueData_t rx_data;
	CAN_RxHeaderTypeDef CanRxHeader;


	/* Reads the data from CAN mail box. */
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CanRxHeader, rx_data.Data);
	/* Sends read data to the queue. */
	rx_data.Identifier = CanRxHeader.StdId;
	xQueueSendToBackFromISR(CanDataRx_qh, (uint8_t*)& rx_data.Identifier, NULL);
}

/**
  * @brief  Rx FIFO 1 message pending callback.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  *
  */
static void CANRxFifo1Callback(struct __CAN_HandleTypeDef* hcan)
{
	CAN_RxQueueData_t rx_data;
	CAN_RxHeaderTypeDef CanRxHeader;

	/* Reads the data from CAN mail box. */
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &CanRxHeader, rx_data.Data);
	/* Sends read data to the queue. */
	rx_data.Identifier = CanRxHeader.StdId;
	xQueueSendToBackFromISR(CanDataRx_qh, (uint8_t*)& rx_data.Identifier, NULL);
}



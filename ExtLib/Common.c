/*
 * @brief Common.c
 *
 * Created on: Jul 16, 2023
 * Author: asw3005
 *
 */
#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "main.h"
#include "Common.h"
#include "DataExchCAN.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

/* External variables. */
extern I2C_HandleTypeDef hi2c1;
extern QueueHandle_t CanDataTx_qh;
extern SemaphoreHandle_t SoftBtnPressed_sh;

/* Private variables. */
static I2C_HandleTypeDef* ExtMemI2C = &hi2c1;

static M24128_MEM_DATA_t UserData = {
	.BYTE_ADDR = GEN_DATA_BYTE,
	.PAGE_ADDR = GEN_DATA_PAGE
};



/*
 * @brief Read all generators' data from external memory on the board.
 *
 * @param ReadWrite : 0 to write, 1 to read.
 */
void DEV_RWGenEepAll(uint8_t ReadWrite) {

	UserData.BYTE_ADDR = GEN_DATA_BYTE;
	UserData.PAGE_ADDR = GEN_DATA_PAGE;
	if (!ReadWrite) {
	/*  */
	}
}

/*
 * @brief Getter for DEV_TypeList_t struct.
 */
DEV_List_t* DEV_GetType(void) {

	static DEV_List_t DevList = { 0 };
	return &DevList;
}

/*
 * @brief Device available list.
 */
void DEV_WriteDevType(uint8_t dev_channel, uint32_t dev_type, uint8_t dev_subtype) {

	if ((dev_channel) <= MAX_UPCB_NUMBER) {
		DEV_GetType()->DevType[dev_channel - 1] = dev_type;
		DEV_GetType()->DevSubType[dev_channel - 1] = dev_subtype;
	}
}

/*
 * @brief Update the list of available device .
 */
void DEV_UpdateState(uint32_t dev_type, uint32_t sprefix, uint8_t address) {

	CAN_TxQueueData_t CAN_TxData = { 0 };

	CAN_TxData.DevType = dev_type;
	CAN_TxData.SPrefixCode = sprefix;

	/* Clear the list of devices. */
	HAL_Delay(100);
	DEV_WriteDevType(address, NO_DEVICE_AVAILABLE, NO_DEVICE_AVAILABLE);
	CAN_TxData.Address = address;
	CAN_TxData.Command = UCS_DEV_LINK;
	xQueueSendToBack(CanDataTx_qh, &CAN_TxData, 0);
	HAL_Delay(75);
}

/*
 * @brief Update the list of available devices.
 */
void DEV_UpdateList() {

	CAN_TxQueueData_t CAN_TxData = { 0 };

	CAN_TxData.DevType		= TYPE_UNDEFINED;
	CAN_TxData.SPrefixCode	= UCS;

	/* Waiting module available. */
	HAL_Delay(50);

	/* Clear list of available devices. */
	for (uint8_t i = 1; i <= MAX_UPCB_NUMBER; i++) {
		DEV_WriteDevType(i, NO_DEVICE_AVAILABLE, NO_DEVICE_AVAILABLE);
	}

	/*  */
	for (uint8_t i = 1; i <= MAX_UPCB_NUMBER; i++) {
		CAN_TxData.Address = UCS1_CMD_BASE_OFFSET + i - 1;
		CAN_TxData.Command = UCS_DEV_LINK;
		xQueueSendToBack(CanDataTx_qh, &CAN_TxData, 0);
		HAL_Delay(75);
	}
}

/*
 * @brief Update devices' parameters.
 */
void DEV_UpdateParams() {

	CAN_TxQueueData_t CAN_TxData = { 0 };

	CAN_TxData.DevType		= DEV;
	CAN_TxData.SPrefixCode	= UCS;

	/* Waiting module available. */
	HAL_Delay(50);

	/* Set phase to all available devices. */
	CAN_TxData.CmdCode		= BCS_SETAmplitude;
	for (uint8_t i = 1; i <= MAX_UPCB_NUMBER; i++) {
		if (DEV_GetType()->DevType[i - 1] == DEV) {
			CAN_TxData.Address = UCS1_CMD_BASE_OFFSET + i - 1;
			CAN_TxData.Command = UCS_SET_AMPLITUDE;
//			for(uint8_t j = 0; j < MAX_SGU_CHANNELS; j++) {
//				CAN_TxData.Phase = DEV_GetGenData()[i - 1].Amplitude[j];
//				xQueueSendToBack(CanDataTx_qh, &CAN_TxData, 0);
//				HAL_Delay(75);
//			}
		}
	}

}

/*
 * @brief Device name decoder.
 **/
void DEV_NameDecode(uint8_t State, uint32_t DevType, uint8_t DevSubtype, DEV_CharBuff_t* CharBuff) {

	/* State decoding to char. */
	switch (State) {
		case DEV_OFF:
			sprintf(&CharBuff->state[0], "OFF");
			break;
		case DEV_ON:
			sprintf(&CharBuff->state[0], "ON");
			break;
		default:
			sprintf(&CharBuff->state[0], "N.A.");
			break;
	}

	/* Select the right type of addressed device. */
	switch (DevType) {
		case DEV:
			sprintf(&CharBuff->dev_id[0], "DEV");
			break;
		default:
			sprintf(&CharBuff->dev_id[0], "N.A.");
			break;
	}

	/* Subtype decoding to char. */
	switch (DevSubtype) {
		case SGU0:
			sprintf(&CharBuff->dev_subid[0], "SGU0");
			break;
		default:
			sprintf(&CharBuff->dev_subid[0], "N.A.");
			break;
	}
}

/*
 * @brief Convert decimal to state binary byte.
 */
uint8_t DEV_DecToStateByte(uint32_t DecValue) {

	/* . */
	uint8_t StateByte = 0;

	/* Disable all if value is bigger than. */
	if (DecValue > 99999999) {
		return 0;
	}
	/* Conversion if it's ok. */
	for (uint8_t i = 7; i > 0; i--) {
		if ((DecValue % 10) > 0) {
			StateByte |= 0x80;
		}
		DecValue /= 10;
		StateByte >>= 1;
	}
	if (DecValue  > 0) {
		StateByte |= 0x80;
	}
	return StateByte;
}

/*
 * @brief Led channel control.
 *
 * @param channel: can have there values -	CH1, CH2, CH3, CH4, CH5, CH6, CH_ALL
 * @param state: If state equal to 0, led is power off. If state above 0, led is power on.
 *
 */
void LED_Ctrl(uint8_t channel, uint8_t state) {

	switch (channel) {

	case CH1:
//		if (state) { HAL_GPIO_WritePin(LED1_CH1_GPIO_Port, LED1_CH1_Pin, GPIO_PIN_RESET); }
//		else { HAL_GPIO_WritePin(LED1_CH1_GPIO_Port, LED1_CH1_Pin, GPIO_PIN_SET); }
		break;
	case CH2:
		if (state) {  }
		else {  }
		break;

	case CH_ALL:
		if (state) {

		} else {

		}
		break;
	default:
		break;
	}

}

/*
 * @brief Clock output return.
 * Returning certain number of ad9520's clock output for the installed device.
 */
uint8_t DEV_GetClockChannel(uint8_t dev_channel) {

	const uint8_t CH_NUMBER[6] = {
			3, 4, 5, 8, 7, 6
	};

	return CH_NUMBER[dev_channel];
}

/*
 * @brief Software button trigger.
 */
void BTN_SWPress(void) {

	/* Giving the semaphore. */
	xSemaphoreGive(SoftBtnPressed_sh);
	/* Software triggered external interrupt input. It's like press a button on the pin 10. */
	EXTI->SWIER = EXTI_SWIER_SWIER10;
}



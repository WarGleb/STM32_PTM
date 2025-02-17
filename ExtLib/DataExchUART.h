/*
 * DataExchUART.h
 *
 * Created on: Jul 16, 2023
 * Author: asw3005
 */

#ifndef DATAEXCHUART_H_
#define DATAEXCHUART_H_

#include "stm32f4xx.h"

/*
 * @brief General command and UART constants.
 */
typedef enum {

	/* */
	FIRST_PARAM_POS			= 0,
	SECOND_PARAM_POS		= 2,
	UART_RX_BUFF_SIZE		= 300,
	UART_TX_BUFF_SIZE		= 100,

	/* */
	FPREFIX_OFFSET			= 0,
	SPREFIX_OFFSET			= 2,
	CMD_PARAM_OFFSET		= 2,
	PREFIX_LENGTH			= 3,
	MAX_TMP_BUFF_LENGTH		= 50,
	MAX_CMD_LENGTH			= 50,
	MAX_PARAM_LENGTH		= 30

} CMD_CONST_t;

/*
 * @brief Parameters' indexes.
 */
typedef union {

	uint8_t data[10];
	struct {
		uint8_t FirstIndex;
		uint8_t SecondIndex;
		uint8_t ThirdIndex;
		uint8_t FourthIndex;
		uint8_t FifthIndex;
	};

} CMD_PIndex_t;

/*
 * @brief Code set of commands.
 */
typedef struct {
	/* First prefix code. */
	uint32_t FPrefixCode;
	/* Second prefix code. */
	uint32_t SPrefixCode;
	/* Basic command code. */
	uint32_t CommandCode;
	/* Command's parameters. */
	char CmdParams[MAX_PARAM_LENGTH];

} CmdCode_t;

/*
 * @brief UART communication struct.
 */
typedef struct {
	uint8_t isSeqReadCmpl;
	uint8_t isDelayedRead;
	uint8_t isCmdLengthFault;
	uint8_t isBuffBoundaryOver;
	uint16_t CmdSeqCnt;
	uint8_t SymbCmdCnt;
	uint8_t SymbParamCnt;
	uint8_t LastSymbCnt;
	uint8_t cReadLimitCnt;
	uint16_t cReadIndexBuff0;
	uint16_t cReadIndexBuff1;
	uint8_t TmpCmdBuff[MAX_TMP_BUFF_LENGTH];
	uint8_t RxDataBuff[UART_RX_BUFF_SIZE];
	uint8_t TxDataBuff[UART_TX_BUFF_SIZE];

} UartTxRxData_t;

/* Public function prototypes. */
void UART_SendMessage(uint8_t* data, uint16_t amount);

#endif /* DATAEXCHUART_H_ */

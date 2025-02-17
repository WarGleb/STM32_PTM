/*
 * DataExchCAN.h
 *
 *  Created on: Jul 16, 2023
 *      Author: asw3005
 */

#ifndef DATAEXCHCAN_H_
#define DATAEXCHCAN_H_

#include "stm32f4xx_hal.h"

 /*
  * @brief Command set of high voltage unit on the CAN bus.
  *
  **/
typedef enum {

	/* General command set. */
	GCS_EN_DIS_DEV,
	GCS_VIEW_ADDR,
	GCS_RESERVED3,
	GCS_RESERVED4,
	GCS_RESERVED5,
	GCS_RESERVED6,
	GCS_RESERVED7,

	/* Global command offsets. */
	GCS_CMD_BASE_OFFSET 			= 0,
	SGU_BCS_CMD_BASE_OFFSET,
	TYPE2_BCS_CMD_BASE_OFFSET,
	TYPE3_BCS_CMD_BASE_OFFSET,
	TYPE4_BCS_CMD_BASE_OFFSET,
	UCS1_CMD_BASE_OFFSET,

	/* Broadcast command set. */
	BCS_EN_DIS_DEV					= 0,
	BCS_VIEW_ADDR,
	BCS_SET_CHANNELS,
	BCS_SET_PHASE,
	BCS_SET_AMPLITUDE,
	BCS_SET_FREQUENCY,
	BCS_RESERVED6,
	BCS_RESERVED7,

	/* Unicast command set. */
	UCS_DEV_LINK					= 0,
	UCS_EN_DIS_DEV,
	UCS_SET_ADDR,
	UCS_LOCK_ADDR,
	UCS_SET_CHANNELS,
	UCS_SET_PHASE,
	UCS_SET_AMPLITUDE,
	UCS_SET_FREQUENCY,
	UCS_GET_STATE



} CAN_CMD_SET_t;

/*
 * @brief CAN default constants.
 */
typedef enum {

	MIN_CAN_ADDR			= 1,
	MAX_CAN_ADDR			= 6,
	CAN_DEFAULT_STDID 		= 0xFF,
	CAN_DEFAULT_EXTDID 		= 0xFF,
	CAN_DEFAULT_DLC 		= 0x08

} CAN_DEFAULT_CONST_t;

/*
 * @brief Queue data type.
 *
 **/
typedef struct __attribute__((aligned(1), packed)) {

	uint32_t CmdCode;
	uint32_t SPrefixCode;
	uint8_t Address;
	uint8_t Command;
	/* Device type to identify module. */
	union {
		uint32_t DevType;
		struct {
			uint8_t DevTypeParts[4];
		};

	};

	/* Parameters set. */
	union {
		struct {
			union {
				/* Address setting. */
				struct {
					uint8_t SetAddress;
					uint8_t Reserved0;
				};
				/* Address locking. */
				struct {
					uint8_t LockAddr;
					uint8_t Reserved1;
				};
				/* En/Dis device. */
				struct {
					uint8_t EnDisDev;
					uint8_t Reserved2;
				};
				/* Set channels. */
				struct {
					uint8_t Channels;
					uint8_t Reserved3;
				};
			};
			/* Gap data. */
			uint8_t Data[2];
		};
		/* DDS parameters set. */
		struct {
			union {
				float Phase;
				float Amplitude;
				float Frequency;
			};
		};
	};

} CAN_TxQueueData_t;

/*
 * @brief CAN RX queue data type.
 *
 **/
typedef struct __attribute__((aligned(1), packed)) {

	uint16_t Identifier;
	union {
		uint8_t Data[8];
		struct {
			/* It occupies five bytes (CmdType + SubTypeChSt). */
			/* Modules' type. */
			uint32_t Type;
			/* Subtype bits. */
			union {
				uint8_t SubTypeChSt;
				struct {
					uint8_t SUBTYPE : 4;
					uint8_t CHANNEL : 3;
					uint8_t STATE 	: 1;
				};
			};
			/* Every struct below must be three bytes at all. */
			/* Generator parameters. */
			struct {
				union {
					uint8_t GenParam;
					struct {
						uint8_t PARAM_TYPE 	: 3;
						uint8_t RESERVED0 	: 5;
					};
				};
				union {
					uint16_t Reserved0;
				};
			};
		};
	};

} CAN_RxQueueData_t;

/*
 * @brief Struct of device identifier and command data type.
 *
 **/
typedef union __attribute__((aligned(1), packed)) {
	uint16_t Identifier;
	struct {
		uint16_t ID_TODO		: 4;
		uint16_t ID_ADDRESS		: 7;
		uint16_t RESERVED15_11	: 5;
	};
} CAN_DevId_t;

/*
 * @brief 16 bits filter bank register organization data type.
 *
 **/
typedef union __attribute__((aligned(1), packed)) {

	uint16_t FilterBankReg;

	struct {
		/* Extended identifier. */
		uint16_t EXID18_16	: 3;
		/* A dominant single identifier extension (IDE) bit means that a standard CAN identifier with no extension is being
		 * transmitted. */
		uint16_t IDE		: 1;
		/* The single remote transmission request bit is dominant when information is required from another node. */
		uint16_t RTR		: 1;
		/* Standard identifier. */
		uint16_t STID31_21	: 11;
	};

} CAN_FilterBankRegOrg16_t;

/*
 * @brief 32 bits filter bank register organization data type.
 *
 **/
typedef union __attribute__((aligned(1), packed)) {
	struct {
		uint16_t FilterBankReg_LSB;
		uint16_t FilterBankReg_MSB;
	};
	struct {
		/* Must be zero. */
		uint32_t RESERVED0	: 1;
		uint32_t RTR		: 1;
		uint32_t IDE		: 1;
		/* Extended identifier. */
		uint32_t EXID20_3	: 18;
		/* Standard identifier. */
		uint32_t STID31_21	: 11;
	};

} CAN_FilterBankRegOrg32_t;

/* public function prototypes. */


#endif /* DATAEXCHCAN_H_ */

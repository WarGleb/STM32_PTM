/*
 * @brief Common driver for ADS8671, ADS8675 from Texas Instruments.
 *        Modified for daisy chain mode (N devices in series: one long shift register).
 * Created 08.02.21 by asw3005. Modified 21.02.XX by ChatGPT.
 *
 **/

#include "ads867x.h"
#include "spi.h"

/* Select the SPI handle used by the ADS867x driver.
   To change the SPI instance, define ADS867X_SPI_HANDLE externally.
   Default is hspi3.
*/
#ifndef ADS867X_SPI_HANDLE
  #define ADS867X_SPI_HANDLE hspi3
#endif

/* Define the number of devices in the daisy chain.
   Each device transmits 32 bits (4 bytes).
*/
#define NUM_DEVICES   4                  // For example, 4 devices in the chain
#define ADS_BYTES     (NUM_DEVICES * 4)    // Total number of bytes transmitted/received via SPI

/* External variables. */
extern SPI_HandleTypeDef ADS867X_SPI_HANDLE;  // hspi3 must be defined in your project

/* Private function prototypes. */
static void ADS867x_SPI_CS(GPIO_TypeDef* gpio, uint16_t gpio_pin, uint8_t state);

/*
 * @brief ADC initialization function.
 *
 **/
void ADS867x_Init(void) {
	
	/* General data structure of the ADC unit. */
	ADS867x_GInst_t ads8671 = { 		
		.delay = HAL_Delay,
		.spi_tx = ADS867x_SPI_Tx,
		.spi_rx = ADS867x_SPI_Rx
	};
	
	/* Set SPI configuration and GPIO function for the SDO1 pin. */
	// ADS867x_SdoCtrl(&ads8671, 0, 0, ADS867x_SDO1_GPO);
	/* Enable LED on the ADC's pin. */
	// ADS867x_SdoPinSetReset(&ads8671, 1);
	/* Enable ADC's test data sequence. */
	ADS867x_DataOutCtrl(&ads8671, ADS867x_CONVDATA, 0, 0, ADS867x_ACTIVE_IN_DO_NOT_INCL, ADS867x_ACTIVE_VDD_DO_NOT_INCL, 0);
	/* Select ADC input range. */
	ADS867x_RangeSel(&ads8671, ADS867x_P1_25VREF, 0);
}

/*
 * @brief Get voltage from ADC.
 *
 **/
float ADS867x_GetVoltage(void) {
	
	/* General data structure of the ADC unit.
	   Note that we now use ADS867X_SPI_HANDLE for accessing the SPI counters.
	*/
	ADS867x_GInst_t ads8671 = { 
		.tx_byte_cnt = &ADS867X_SPI_HANDLE.TxXferCount,
		.rx_byte_cnt = &ADS867X_SPI_HANDLE.RxXferCount,
		.delay = HAL_Delay,
		.spi_tx = ADS867x_SPI_Tx,
		.spi_rx = ADS867x_SPI_Rx
	};

	/* Example calculation of voltage based on the received data.
	   Here, the result (for example, from the last device in the chain) is extracted.
	*/
	return ADS867x_VALUE_OF_DIVISION * (uint16_t)(ADS867x_ReadADC(&ads8671).DataWord_HSW_MSB >> 18);
}

/*
 * @brief Read ADC conversion data in daisy chain mode.
 *        In daisy chain mode, the ADCs are connected in series, so the total shift register length
 *        is NUM_DEVICES * 32 bits. To read the data, a buffer of NUM_DEVICES*4 bytes (each byte contains the NOP command)
 *        is transmitted, and then the same amount of data is received.
 *
 * @param *device : Instance of the general data structure ADS867x_GInst_t.
 * @return 14-bit ADC data from one selected device (for example, from the last device in the chain).
 **/
ADS867x_OutputDataWord_t ADS867x_ReadADC(ADS867x_GInst_t* device)
{
	uint16_t timeout = 10000;
	uint8_t buffer[ADS_BYTES];
	uint8_t i;
	ADS867x_OutputDataWord_t result;
	
	/* Fill the buffer with NOP values for all devices */
	for (i = 0; i < ADS_BYTES; i++) {
		buffer[i] = ADS867x_NOP;
	}
	
	/* Transmit the buffer to generate clock pulses for shifting data from all devices */
	device->spi_tx(buffer, ADS_BYTES);
	
	while (*device->tx_byte_cnt > 0) {
		timeout--;
		if (timeout == 0) { __NOP(); break; }
	}

	// Optionally, add a short delay:
	// device->delay(1);
	
	/* Receive NUM_DEVICES*4 bytes of data */
	device->spi_rx(buffer, ADS_BYTES);
	timeout = 10000;
	while (*device->rx_byte_cnt > 0) {
		timeout--;
		if (timeout == 0) { __NOP(); break; }
	}

	/*
	 * Parse the received buffer.
	 * It is assumed that the data from each device is transmitted consecutively.
	 * Here, as an example, the result from the last device in the chain is selected.
	 */
	result.DataWord_HSW_MSB = buffer[ADS_BYTES - 4];
	result.DataWord_HSW_LSB = buffer[ADS_BYTES - 3];
	result.DataWord_LSW_MSB = buffer[ADS_BYTES - 2];
	result.DataWord_LSW_LSB = buffer[ADS_BYTES - 1];
	
	return result;
}

/*
 * @brief Read device register.
 * 
 * @param *device : Instance of the general data structure ADS867x_GInst_t.
 * @param address : Device register address to be read.
 *
 **/
uint16_t ADS867x_R_REG(ADS867x_GInst_t* device, uint8_t address)
{
	uint16_t Data = 0;
	
	device->data.ADDRESS = address;
	device->data.COMMAND = ADS867x_READ_HWORD;
	device->data.REG_DATA_LSB = 0;
	device->data.REG_DATA_MSB = 0;
	device->spi_tx(&device->data.Command, 4);
	
	device->delay(1);
	device->spi_rx(&device->data.DataWord_LSW_MSB, 2);
	device->delay(1);
	Data = ((uint16_t)device->data.DataWord_LSW_MSB << 8) | (uint16_t)device->data.DataWord_LSW_LSB;
	return Data;
}

/*
 * @brief Controls the reset and power-down features.
 * 
 * @param *device : Instance of the general data structure ADS867x_GInst_t.
 * @param pwrdn : 0 puts the converter into active mode, 1 puts the converter into power-down mode.
 * @param nap_en : 0 disables NAP mode, 1 enables NAP mode if CONVST/CS is held high after the current conversion.
 * @param rstn_app : If 0, RST functions as a POR reset (full initialization); if 1, as an application reset.
 * @param in_al_dis : 0 enables the input alarm, 1 disables it.
 * @param vdd_al_dis : 0 enables the VDD alarm, 1 disables it.
 *
 **/
void ADS867x_RstPwdn(ADS867x_GInst_t* device, uint8_t pwrdn, uint8_t nap_en, uint8_t rstn_app, uint8_t in_al_dis, uint8_t vdd_al_dis)
{
	ADS867x_RstPwrCtrl_t RstPwrCtlReg;
	
	RstPwrCtlReg.PWRDN = pwrdn;
	RstPwrCtlReg.NAP_EN = nap_en;
	RstPwrCtlReg.RSTn_APP = rstn_app;
	RstPwrCtlReg.IN_AL_DIS = in_al_dis;
	RstPwrCtlReg.VDD_AL_DIS = vdd_al_dis;
	RstPwrCtlReg.WKEY = ADS867x_WKEY;
	
	device->data.ADDRESS = ADS867x_RST_PWRCTL_LSW;
	device->data.COMMAND = ADS867x_WRITE_MSB;
	device->data.REG_DATA_LSB = 0;
	device->data.REG_DATA_MSB = ADS867x_WKEY;
	device->spi_tx(&device->data.Command, 4);
	device->delay(1);
	device->data.COMMAND = ADS867x_WRITE_LSB;
	device->data.REG_DATA_LSB = (uint8_t)RstPwrCtlReg.RstPwrCtrlReg_LSW;
	device->spi_tx(&device->data.Command, 4);
}

/*
 * @brief Configures the protocol used for writing data.
 * 
 * @param *device : Instance of the general data structure ADS867x_GInst_t.
 * @param protocol : Selects the SPI protocol, see ADS867x_SPI_PROTOCOL_t enum (default ADS867x_CPOL0_CPHASE0).
 *
 **/
void ADS867x_SdiCtrl(ADS867x_GInst_t* device, ADS867x_SPI_PROTOCOL_t protocol)
{
	// ADS867x_SdiCtrl_t SdiCtrlReg;
	// SdiCtrlReg.SDI_MODE = protocol;
	
	device->data.ADDRESS = ADS867x_SDI_CTL_LSW;
	device->data.COMMAND = ADS867x_WRITE_LSB;
	device->data.REG_DATA_LSB = protocol;
	device->data.REG_DATA_MSB = 0;
	device->spi_tx(&device->data.Command, 4);	
}

/*
 * @brief Controls the data protocol used to transmit data from the SDO-x pins.
 * NOTE: This function resets the GPO pin (GPO_VAL) to 0.
 * 
 * @param *device : Instance of the general data structure ADS867x_GInst_t.
 * @param sdo_mode : If 0xb, SDO mode follows the same SPI protocol as SDI (default).
 * @param ssync_clk : If 0, external SCLK is selected (no division, default); if 1, internal SCLK is selected.
 * @param sdo1_config : Configuration for ALARM/SDO-1/GPO, see ADS867x_SDO1_MODE_t.
 *
 **/
void ADS867x_SdoCtrl(ADS867x_GInst_t* device, uint8_t sdo_mode, uint8_t ssync_clk, ADS867x_SDO1_MODE_t sdo1_config)
{
	ADS867x_SdoCtrl_t SdoCtrlReg;
	
	SdoCtrlReg.SDO_MODE = sdo_mode;
	SdoCtrlReg.SSYNC_CLK = ssync_clk;
	SdoCtrlReg.SDO1_CONFIG = sdo1_config;
	SdoCtrlReg.GPO_VAL = 0;
	
	device->data.ADDRESS = ADS867x_SDO_CTL_LSW;
	device->data.COMMAND = ADS867x_WRITE_HWORD;
	device->data.REG_DATA_LSB = SdoCtrlReg.SdoCtrlReg_LSW;
	device->data.REG_DATA_MSB = SdoCtrlReg.SdoCtrlReg_LSW >> 8;
	device->spi_tx(&device->data.Command, 4);		
}

/*
 * @brief Drives the logical level of the general purpose pin (SDO1 used as a GPO).
 * 
 * @param *device : Instance of the general data structure ADS867x_GInst_t.
 * @param sdo_val : 1-bit value for the output (0 or 1).
 *
 **/
void ADS867x_SdoPinSetReset(ADS867x_GInst_t* device, uint8_t sdo_val)
{	
	device->data.ADDRESS = ADS867x_SDO_CTL_LSW;	
	if (sdo_val) {
		device->data.COMMAND = ADS867x_SET_HWORD;
	}
	else {
		device->data.COMMAND = ADS867x_RESET_HWORD;
	}	
	device->data.REG_DATA_LSB = 0;
	device->data.REG_DATA_MSB = 0x10;
	device->spi_tx(&device->data.Command, 4);
}

/*
 * @brief Selects the data format for the output data.
 * 
 * @param *device : Instance of the general data structure ADS867x_GInst_t.
 * @param data_val : Defines the data value, see ADS867x_DataVal enum.
 * @param par_en : 0 means data without parity; 1 means data with two parity bits.
 * @param range_incl : 0 means do not include the range value; 1 means include it.
 * @param in_active_alarm_incl : Include input alarm flags.
 * @param vdd_active_alarm_incl : Include VDD alarm flags.
 * @param device_addr_incl : 0 means do not include; 1 means include.
 *
 **/
void ADS867x_DataOutCtrl(ADS867x_GInst_t* device, ADS867x_DATA_VAL_t data_val, ADS867x_PAR_t par_en, ADS867x_RANGE_INCL_t range_incl, 
	ADS867x_IN_ACTIVE_ALARM_t in_active_alarm_incl, ADS867x_VDD_ACTIVE_ALARM_t vdd_active_alarm_incl, ADS867x_DEV_ADDR_INCL_t device_addr_incl)
{
	ADS867x_DataOut_t DataOutReg;
	
	DataOutReg.DATA_VAL = data_val;
	DataOutReg.PAR_EN = par_en;
	DataOutReg.RANGE_INCL = range_incl;
	DataOutReg.IN_ACTIVE_ALARM_INCL = in_active_alarm_incl;
	DataOutReg.VDD_ACTIVE_ALARM_INCL = vdd_active_alarm_incl;
	DataOutReg.DEVICE_ADDR_INCL = device_addr_incl;
	
	device->data.ADDRESS = ADS867x_DATAOUT_CTL_LSW;
	device->data.COMMAND = ADS867x_WRITE_HWORD;
	device->data.REG_DATA_LSB = DataOutReg.DataOutReg_LSW;
	device->data.REG_DATA_MSB = DataOutReg.DataOutReg_LSW >> 8;
	device->spi_tx(&device->data.Command, 4);
}

/*
 * @brief Selects either the internal or external reference and input range.
 * 
 * @param *device : Instance of the general data structure ADS867x_GInst_t.
 * @param range_sel : Selection of the input signal range, see ADS867x_INPUT_RANGE_t.
 * @param intref_dis : 0 to enable the internal reference voltage, 1 to disable.
 *
 **/
void ADS867x_RangeSel(ADS867x_GInst_t* device, ADS867x_INPUT_RANGE_t range_sel, uint8_t intref_dis)
{
	ADS867x_RangeSel_t RangeSelReg;
	
	RangeSelReg.RANGE_SEL = range_sel;
	RangeSelReg.INTREF_DIS = intref_dis;
	
	device->data.ADDRESS = ADS867x_RANGE_SEL_LSW;
	device->data.COMMAND = ADS867x_WRITE_HWORD;
	device->data.REG_DATA_LSB = RangeSelReg.RangeSelReg_LSW;
	device->data.REG_DATA_MSB = RangeSelReg.RangeSelReg_LSW >> 8;
	device->spi_tx(&device->data.Command, 4);	
}

/*
 * @brief Returns the output condition of the alarm flags.
 * 
 * @param *device : Instance of the general data structure ADS867x_GInst_t.
 *
 **/
ADS867x_Alarm_t ADS867x_ReadAlarm(ADS867x_GInst_t* device)
{
	ADS867x_Alarm_t AlarmReg;
	
	device->data.ADDRESS = ADS867x_ALARM_LSW;
	device->data.COMMAND = ADS867x_READ_HWORD;
	device->data.REG_DATA_LSB = 0;
	device->data.REG_DATA_MSB = 0;
	device->spi_tx(&device->data.Command, 4);
	device->delay(1);
	device->spi_rx((uint8_t*)&device->data, 2);
	AlarmReg.AlarmReg_LSW = ((uint16_t)device->data.REG_DATA_MSB << 8) | (uint16_t)device->data.REG_DATA_LSB;
	
	return AlarmReg;
}

/*
 * @brief Sets the hysteresis and high threshold for the input alarm.
 * 
 * @param *device : Instance of the general data structure ADS867x_GInst_t.
 * @param inp_alrm_high_th : 14-bit threshold for comparison.
 * @param inp_alrm_hyst : 4-bit hysteresis value.
 *
 **/
void ADS867x_SetAlarmHTh(ADS867x_GInst_t* device, uint16_t inp_alrm_high_th, uint8_t inp_alrm_hyst)
{
	ADS867x_AlarmHTh_t AlarmHthReg;
	
	AlarmHthReg.RESERVED1_0 = 0;
	AlarmHthReg.RESERVED27_24 = 0;
	AlarmHthReg.INP_ALRM_HIGH_TH = inp_alrm_high_th;	
	AlarmHthReg.INP_ALRM_HYST = inp_alrm_hyst;
	
	device->data.ADDRESS = ADS867x_ALARM_H_TH_LSW;
	device->data.COMMAND = ADS867x_WRITE_HWORD;
	device->data.REG_DATA_LSB = AlarmHthReg.AlarmHTh_LSW;
	device->data.REG_DATA_MSB = AlarmHthReg.AlarmHTh_LSW >> 8;
	device->spi_tx(&device->data.Command, 4);
	device->delay(1);
	device->data.ADDRESS = ADS867x_ALARM_H_TH_HSW;
	device->data.REG_DATA_LSB = AlarmHthReg.AlarmHTh_HSW;
	device->data.REG_DATA_MSB = AlarmHthReg.AlarmHTh_HSW >> 8;
	device->spi_tx(&device->data.Command, 4);	
}

/*
 * @brief Sets the low threshold for the input alarm.
 * 
 * @param *device : Instance of the general data structure ADS867x_GInst_t.
 * @param inp_alrm_low_th : 14-bit threshold for comparison.
 *
 **/
void ADS867x_SetAlarmLTh(ADS867x_GInst_t* device, uint16_t inp_alrm_low_th)
{
	ADS867x_AlarmLTh_t AlarmLThReg;
	
	AlarmLThReg.RESERVED1_0 = 0;
	AlarmLThReg.INP_ALRM_LOW_TH = inp_alrm_low_th;
	
	device->data.ADDRESS = ADS867x_ALARM_L_TH_LSW;
	device->data.COMMAND = ADS867x_WRITE_HWORD;
	device->data.REG_DATA_LSB = AlarmLThReg.AlarmLTh_LSW;
	device->data.REG_DATA_MSB = AlarmLThReg.AlarmLTh_LSW >> 8;
	device->spi_tx(&device->data.Command, 4);	
}


/* Hardware dependent functions. */

/*
 * @brief Transmits data via SPI.
 *
 **/
void ADS867x_SPI_Tx(uint8_t *pData, uint8_t size) {
	
	ADS867x_SPI_CS(CS_ADC_GPIO_Port, CS_ADC_Pin, 0);
	HAL_SPI_Transmit(&ADS867X_SPI_HANDLE, pData, size, 10);
	ADS867x_SPI_CS(CS_ADC_GPIO_Port, CS_ADC_Pin, 1);
}

/*
 * @brief Receives data via SPI.
 *
 **/
void ADS867x_SPI_Rx(uint8_t *pData, uint8_t size) {
	
	ADS867x_SPI_CS(CS_ADC_GPIO_Port, CS_ADC_Pin, 0);
	HAL_SPI_Receive(&ADS867X_SPI_HANDLE, pData, size, 10);
	ADS867x_SPI_CS(CS_ADC_GPIO_Port, CS_ADC_Pin, 1);
}

/*
 * @brief SPI chip select.
 * 
 * @param gpio : For example, CS_ADC_GPIO_Port.
 * @param gpio_pin : For example, CS_ADC_Pin.
 * @param state : GPIO_PIN_SET or GPIO_PIN_RESET.
 *
 **/
static void ADS867x_SPI_CS(GPIO_TypeDef* gpio, uint16_t gpio_pin, uint8_t state) {
	
	if (state > 0) {
		gpio->BSRR = gpio_pin;
	}
	else {
		gpio->BSRR = gpio_pin << 16;
	}	
}

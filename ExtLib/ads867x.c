/*
 * @brief Common driver for ADS8671, ADS8675 from Texas Instruments.
 *        Modified for daisy chain mode (N devices in series: one long shift register).
 * Created 08.02.21 by asw3005. Modified 21.02.XX by ChatGPT.
 *
 **/

#include "ads867x.h"
#include "spi.h"

/* Определение количества устройств в цепочке.
   Каждое устройство передаёт 32 бита (4 байта). */
#define NUM_DEVICES   2                  // Например, 2 устройства в цепочке
#define ADS_BYTES     (NUM_DEVICES * 4)    // Общее число байт, передаваемых/принимаемых по SPI

/* External variables. */
extern SPI_HandleTypeDef hspi1;

/* Private function prototypes. */
static void ADS867x_SPI_CS(GPIO_TypeDef* gpio, uint16_t gpio_pin, uint8_t state);

/*
 * @brief ADC init function.
 *
 **/
void ADS867x_Init(void) {
	
	/* General data struct of ADC unit. */
	ADS867x_GInst_t ads8671 = { 		
		.delay = HAL_Delay,
		.spi_tx = ADS867x_SPI_Tx,
		.spi_rx = ADS867x_SPI_Rx
	};
	
	/* Set SPI configuration and GPIO function of SDO1 pin. */
	// ADS867x_SdoCtrl(&ads8671, 0, 0, ADS867x_SDO1_GPO);	
	/* Enable LED on the ADC's pin. */
	// ADS867x_SdoPinSetReset(&ads8671, 1);	
	/* Enable ADC's test data sequence. */
	ADS867x_DataOutCtrl(&ads8671, ADS867x_CONVDATA, 0, 0, ADS867x_ACTIVE_IN_DO_NOT_INCL, ADS867x_ACTIVE_VDD_DO_NOT_INCL, 0);
	/* Selecting ADC input range. */
	ADS867x_RangeSel(&ads8671, ADS867x_P1_25VREF, 0);
}

/*
 * @brief Get voltage from ADC.
 *
 **/
float ADS867x_GetVoltage(void) {
	
	/* General data struct of ADC unit. */
	ADS867x_GInst_t ads8671 = { 
		.tx_byte_cnt = &hspi1.TxXferCount,
		.rx_byte_cnt = &hspi1.RxXferCount,
		.delay = HAL_Delay,
		.spi_tx = ADS867x_SPI_Tx,
		.spi_rx = ADS867x_SPI_Rx
	};

	/* Пример расчёта напряжения на основе полученных данных.
	   Здесь извлекается результат (например, от последнего устройства в цепочке). */
	return ADS867x_VALUE_OF_DIVISION * (uint16_t)(ADS867x_ReadADC(&ads8671).DataWord_HSW_MSB >> 18);
}

/*
 * @brief Read ADC conversion data in daisy chain mode.
 *        В режиме daisy chain ADC-ы соединены последовательно, поэтому суммарная длина сдвигового регистра равна NUM_DEVICES * 32 бит.
 *        Для чтения данных отправляется буфер из NUM_DEVICES*4 байт (каждый байт содержит команду NOP),
 *        после чего принимается такой же объём данных.
 *
 * @param *device : Instance of the general data struct ADS867x_GInst_t.
 * @return 14-bit ADC data from one selected device (например, от последнего в цепочке).
 **/
ADS867x_OutputDataWord_t ADS867x_ReadADC(ADS867x_GInst_t* device)
{
	uint16_t timeout = 10000;
	uint8_t buffer[ADS_BYTES];
	uint8_t i;
	ADS867x_OutputDataWord_t result;
	
	/* Заполняем буфер значениями NOP для всех устройств */
	for (i = 0; i < ADS_BYTES; i++) {
		buffer[i] = ADS867x_NOP;
	}
	
	/* Передаём буфер – генерируем такты для сдвига данных от всех устройств */
	device->spi_tx(buffer, ADS_BYTES);
	
	while (*device->tx_byte_cnt > 0) {
		timeout--;
		if (timeout == 0) { __NOP(); break; }
	}
	
	// При необходимости можно добавить короткую задержку:
	// device->delay(1);
	
	/* Принимаем NUM_DEVICES*4 байт данных */
	device->spi_rx(buffer, ADS_BYTES);
	timeout = 10000;
	while (*device->rx_byte_cnt > 0) {
		timeout--;
		if (timeout == 0) { __NOP(); break; }
	}
	
	/* 
	 * Парсинг полученного буфера.
	 * Предполагается, что данные каждого устройства передаются последовательно.
	 * Здесь в качестве примера выбирается результат последнего устройства в цепочке.
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
 * @param *device : Instance of the general data struct ADS867x_GInst_t.
 * @param address : Device address that you wish.
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
	Data  = ((uint16_t)device->data.DataWord_LSW_MSB << 8) | (uint16_t)device->data.DataWord_LSW_LSB;	
	return Data;
}

/*
 * @brief Controls the reset and power-down features.
 * 
 * @param *device : Instance of the general data struct ADS867x_GInst_t.
 * @param pwrdn : 0 puts the converter into active mode, 1 puts the converter into power-down mode.
 * @param nap_en : 0 disables the NAP mode of the converter, 1 enables NAP mode if CONVST/CS удерживается после завершения текущего преобразования.
 * @param rstn_app : Если 0, RST работает как POR reset (полная инициализация), если 1 – как application reset.
 * @param in_al_dis : 0 – input alarm включён, 1 – отключён.
 * @param vdd_al_dis : 0 – VDD alarm включён, 1 – отключён.
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
 * @param *device : Instance of the general data struct ADS867x_GInst_t.
 * @param protocol : Selects the SPI protocol, см. ADS867x_SPI_PROTOCOL_t enum (по умолчанию ADS867x_CPOL0_CPHASE0).
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
 * @brief Controls data protocol used to transmit data from the SDO-x pins.
 * NOTE: Функция сбрасывает GPO pin (GPO_VAL) в 0.
 * 
 * @param *device : Instance of the general data struct ADS867x_GInst_t.
 * @param sdo_mode : Если 0xb – SDO mode соответствует SPI протоколу SDI (по умолчанию).
 * @param ssync_clk : Если 0 – внешняя SCLK (без деления, по умолчанию), 1 – внутренняя SCLK.
 * @param sdo1_config : Конфигурация ALARM/SDO-1/GPO, см. ADS867x_SDO1_MODE_t.
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
 * @brief Drives logical level of general purpose pin (SDO1 as a GPO).
 * 
 * @param *device : Instance of the general data struct ADS867x_GInst_t.
 * @param sdo_val : 1-битное значение для выхода (0 или 1).
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
 * @brief Selects data format for the output data.
 * 
 * @param *device : Instance of the general data struct ADS867x_GInst_t.
 * @param data_val : Определяет значение данных, см. ADS867x_DataVal enum.
 * @param par_en : 0 – данные без паритета, 1 – с двумя битами паритета.
 * @param range_incl : 0 – не включать значение диапазона, 1 – включить.
 * @param in_active_alarm_incl : Включить флаги input ALARM.
 * @param vdd_active_alarm_incl : Включить флаги VDD ALARM.
 * @param device_addr_incl : 0 – не включать, 1 – включать.
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
 * @brief Selects either internal or external reference and input range.
 * 
 * @param *device : Instance of the general data struct ADS867x_GInst_t.
 * @param range_sel : Выбор диапазона входного сигнала, см. ADS867x_INPUT_RANGE_t.
 * @param intref_dis : 0 – включить внутреннее опорное напряжение, 1 – отключить.
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
 * @brief Return output condition of the alarm flags.
 * 
 * @param *device : Instance of the general data struct ADS867x_GInst_t.
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
 * @brief Sets hysteresis and high threshold for the input alarm.
 * 
 * @param *device : Instance of the general data struct ADS867x_GInst_t.
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
 * @brief Sets low threshold for the input alarm.
 * 
 * @param *device : Instance of the general data struct ADS867x_GInst_t.
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
	HAL_SPI_Transmit(&hspi1, pData, size, 10);
	ADS867x_SPI_CS(CS_ADC_GPIO_Port, CS_ADC_Pin, 1);
}

/*
 * @brief Receives data via SPI.
 *
 **/
void ADS867x_SPI_Rx(uint8_t *pData, uint8_t size) {
	
	ADS867x_SPI_CS(CS_ADC_GPIO_Port, CS_ADC_Pin, 0);
	HAL_SPI_Receive(&hspi1, pData, size, 10);
	ADS867x_SPI_CS(CS_ADC_GPIO_Port, CS_ADC_Pin, 1);
}

/*
 * @brief SPI chip select.
 * 
 * @param gpio : Например, CS_ADC_GPIO_Port.
 * @param gpio_pin : Например, CS_ADC_Pin.
 * @param state : GPIO_PIN_SET или GPIO_PIN_RESET.
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

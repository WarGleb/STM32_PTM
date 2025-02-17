/*
 * @brief Common driver for ADS8671, ADS8675 from the Texas Instruments.
 * Created 08.02.21 by asw3005. 
 *
 **/

#include "ads867x.h"
#include "spi.h"

/* External variables. */
extern SPI_HandleTypeDef hspi1;


/* Private function prototypes. */
void ADS867x_Init(void) {
    /* Общая структура данных ADC */
    ADS867x_GInst_t ads867x = {
        .delay = HAL_Delay,
        .spi_tx = ADS867x_SPI_Tx,
        .spi_rx = ADS867x_SPI_Rx
    };

    /* Настройка SPI-протокола */
    ADS867x_SdiCtrl(&ads867x, ADS867x_CPOL0_CPHASE0);

    /* Настройка SDO для daisy-chain */
    ADS867x_SdoCtrl(&ads867x, 0b11, 0b0, ADS867x_SDO1_2BITMODE); // Режим daisy-chain

    /* Выбор выходного формата данных */
    ADS867x_DataOutCtrl(&ads867x, ADS867x_CONVDATA, 0, 0, ADS867x_ACTIVE_IN_DO_NOT_INCL, ADS867x_ACTIVE_VDD_DO_NOT_INCL, 0);

    /* Выбор диапазона входного сигнала */
    ADS867x_RangeSel(&ads867x, ADS867x_P1_25VREF, 0);
}


/*
	@brief
*/
float ADS867x_GetVoltage_DaisyChain(ADS867x_GInst_t *device, uint8_t device_count, uint8_t device_index) {
    if (device_index >= device_count) {
        return -1.0f; // Error: invalid device index
    }

    uint8_t spi_tx_buffer[4 * device_count];  // Buffer for transmitting NOP commands
    uint8_t spi_rx_buffer[4 * device_count];  // Buffer for receiving ADC data

    // Fill the transmission buffer with NOP commands
    for (uint8_t i = 0; i < 4 * device_count; i++) {
        spi_tx_buffer[i] = ADS867x_NOP;
    }

    // Select the chain (CS low)
    HAL_GPIO_WritePin(CS_ADC_GPIO_Port, CS_ADC_Pin, GPIO_PIN_RESET);

    // Transmit and receive data through the daisy chain
    device->spi_tx(spi_tx_buffer, 4 * device_count);
    device->spi_rx(spi_rx_buffer, 4 * device_count);

    // Deselect the chain (CS high)
    HAL_GPIO_WritePin(CS_ADC_GPIO_Port, CS_ADC_Pin, GPIO_PIN_SET);

    // Extract data from the target device
    uint8_t *data_ptr = &spi_rx_buffer[4 * device_index];
    uint32_t raw_data = (data_ptr[0] << 24) | (data_ptr[1] << 16) | (data_ptr[2] << 8) | data_ptr[3];

    // Convert raw data to voltage
    return ADS867x_VALUE_OF_DIVISION * (float)(raw_data >> 18);
}

/*
 * @brief Read ADC convertion data.
 * 
 * @param *device : Instance of the general data struct ADS867x_GInst_t.
 * @return 14-bit ADC data type of ADS867x_OutputDataWord_t.
 *
 **/
ADS867x_OutputDataWord_t ADS867x_ReadADC_DaisyChain(ADS867x_GInst_t* device, uint8_t device_count) {
    uint16_t timeout = 10000;
    ADS867x_OutputDataWord_t DataWord = {0};

    // Настройка команды для чтения данных
    device->data.ADDRESS = ADS867x_NOP;  // NOP для последующего устройства
    device->data.COMMAND = ADS867x_READ_HWORD;
    device->data.REG_DATA_LSB = 0;
    device->data.REG_DATA_MSB = 0;

    // Отправка данных через SPI
    for (uint8_t i = 0; i < device_count; i++) {
        device->spi_tx(&device->data.Command, 4);  // Передача команды через цепочку

        // Ожидание завершения передачи
        while (*device->tx_byte_cnt > 0) {
            timeout--;
            if (timeout == 0) {
                __NOP();
                return DataWord; // Вернуть пустой результат при ошибке
            }
        }

        // Получение данных от устройства
        device->spi_rx(&device->data.DataWord_HSW_MSB, 4);

        timeout = 10000;
        while (*device->rx_byte_cnt > 0) {
            timeout--;
            if (timeout == 0) {
                __NOP();
                return DataWord; // Вернуть пустой результат при ошибке
            }
        }
    }

    // Копирование данных из структуры
    DataWord.DataWord_LSW_LSB = device->data.DataWord_LSW_LSB;
    DataWord.DataWord_LSW_MSB = device->data.DataWord_LSW_MSB;
    DataWord.DataWord_HSW_LSB = device->data.DataWord_HSW_LSB;
    DataWord.DataWord_HSW_MSB = device->data.DataWord_HSW_MSB;

    return DataWord;
}


/*
 * @brief Read device register.
 * 
 * @param *device : Instance of the general data struct ADS867x_GInst_t.
 * @param address : Device address that you wish.
 *
 **/

uint16_t ADS867x_R_REG_DaisyChain(ADS867x_GInst_t* devices[], uint8_t device_count, uint8_t target_device_index, uint8_t address)
{
    uint16_t Data = 0;

    // Подготовка команд для всех устройств в цепочке
    for (uint8_t i = 0; i < device_count; i++) {
        if (i == target_device_index) {
            devices[i]->data.ADDRESS = address;
            devices[i]->data.COMMAND = ADS867x_READ_HWORD;
        } else {
            devices[i]->data.ADDRESS = ADS867x_NOP;
            devices[i]->data.COMMAND = ADS867x_NOP;
        }
        devices[i]->data.REG_DATA_LSB = 0;
        devices[i]->data.REG_DATA_MSB = 0;
    }

    // Отправка команд по SPI
    for (uint8_t i = 0; i < device_count; i++) {
        devices[i]->spi_tx(&devices[i]->data.Command, 4);
    }

    // Небольшая задержка
    devices[0]->delay(1);

    // Получение данных из всех устройств
    for (uint8_t i = 0; i < device_count; i++) {
        devices[i]->spi_rx(&devices[i]->data.DataWord_LSW_MSB, 2);
    }

    // Небольшая задержка
    devices[0]->delay(1);

    // Извлечение данных из целевого устройства
    Data = ((uint16_t)devices[target_device_index]->data.DataWord_LSW_MSB << 8) |
           (uint16_t)devices[target_device_index]->data.DataWord_LSW_LSB;

    return Data;
}

/*
 * @brief Controls the reset and power-down features.
 * 
 * @param *device : Instance of the general data struct ADS867x_GInst_t.
 * @param pwrdn : 0 puts the converter into active mode, 1 puts the converter into power-down mode.
 * @param nap_en : 0 disables the NAP mode of the converter, 1 enables the converter to enter NAP mode if CONVST/CS 
 *        is held high after the current conversion completes.
 * @param rstn_app : If 0 RST pin functions as a POR class reset (causes full device initialization) if 1 RST pin 
 *        functions as an application reset (only user-programmed modes are cleared).
 * @param in_al_dis : If 0 input alarm is enabled, 1 input alarm is disabled.
 * @param vdd_al_dis : If 0 VDD alarm is enabled, 1 VDD alarm is disabled.
 *
 **/
void ADS867x_RstPwdn_DaisyChain(ADS867x_GInst_t* devices[], uint8_t device_count, uint8_t pwrdn, uint8_t nap_en, uint8_t rstn_app, uint8_t in_al_dis, uint8_t vdd_al_dis) {
    ADS867x_RstPwrCtrl_t RstPwrCtlReg;

    RstPwrCtlReg.PWRDN = pwrdn;
    RstPwrCtlReg.NAP_EN = nap_en;
    RstPwrCtlReg.RSTn_APP = rstn_app;
    RstPwrCtlReg.IN_AL_DIS = in_al_dis;
    RstPwrCtlReg.VDD_AL_DIS = vdd_al_dis;
    RstPwrCtlReg.WKEY = ADS867x_WKEY;

    for (uint8_t i = 0; i < device_count; i++) {
        devices[i]->data.ADDRESS = ADS867x_RST_PWRCTL_LSW;
        devices[i]->data.COMMAND = ADS867x_WRITE_MSB;
        devices[i]->data.REG_DATA_LSB = 0;
        devices[i]->data.REG_DATA_MSB = ADS867x_WKEY;
        devices[i]->spi_tx(&devices[i]->data.Command, 4);
        devices[i]->delay(1);

        devices[i]->data.COMMAND = ADS867x_WRITE_LSB;
        devices[i]->data.REG_DATA_LSB = (uint8_t)RstPwrCtlReg.RstPwrCtrlReg_LSW;
        devices[i]->spi_tx(&devices[i]->data.Command, 4);
    }
}


/*
 * @brief Configures the protocol used for writing data.
 * 
 * @param *device : Instance of the general data struct ADS867x_GInst_t.
 * @param protocol : Selects the SPI protocol, see ADS867x_SPI_PROTOCOL_t enum (default ADS867x_CPOL0_CPHASE0).
 *
 **/
void ADS867x_SdiCtrl(ADS867x_GInst_t* device, ADS867x_SPI_PROTOCOL_t protocol, uint8_t daisy_chain_enable) {
    ADS867x_SdiCtrl_t SdiCtrlReg = {0};

    // Настройка режима SPI протокола
    SdiCtrlReg.SDI_MODE = protocol;

    // Если включен режим daisy-chain, активируем соответствующий бит
    if (daisy_chain_enable) {
        SdiCtrlReg.DAISY_ENABLE = 1;
    }

    device->data.ADDRESS = ADS867x_SDI_CTL_LSW;
    device->data.COMMAND = ADS867x_WRITE_LSB;
    device->data.REG_DATA_LSB = *((uint8_t*)&SdiCtrlReg);
    device->data.REG_DATA_MSB = 0;

    // Передача данных через SPI
    device->spi_tx(&device->data.Command, 4);
}


/*
 * @brief Controls data protocol used to transmit data from the SDO-x pins of the device.
 * NOTE. This function resets the GPO pin (GPO_VAL) to zero.
 * 
 * @param *device : Instance of the general data struct ADS867x_GInst_t.
 * @param sdo_mode : If 0xb, SDO mode follows the same SPI protocol as that used for SDI (default), see the SDI_CTL_REG register.
 *        If 10b, invalid configuration. If 11b SDO mode follows the ADC master clock or source-synchronous protocol.
 * @param ssync_clk : If 0b, external SCLK selected (no division, default), 1b - internal clock selected (no division).
 * @param sdo1_config : It used to configure ALARM/SDO-1/GPO, see the ADS867x_SDO1_MODE_t enum.
 *
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
 * @brief Drives logical level of general purpoise pin (SDO1 as a GPO pin, that alternative function should be 
 * selected in advance by ADS867x_SdoCtrl function above).
 * 
 * @param *device : Instance of the general data struct ADS867x_GInst_t.
 * @param gpo_val : 1-bit value for the output on the GPO pin (can be 0 or 1).
 *
 **/
void ADS867x_SdoPinSetReset_DaisyChain(ADS867x_GInst_t* device, uint8_t sdo_mode, uint8_t ssync_clk)
{
    // Настройка SDO_CTL для режима daisy-chain
    ADS867x_SdoCtrl_t SdoCtrlReg;
    SdoCtrlReg.SDO_MODE = sdo_mode;       // Выбор режима SDO (0b = SPI протокол)
    SdoCtrlReg.SSYNC_CLK = ssync_clk;     // Выбор источника тактового сигнала (0 = внешний SCLK)
    SdoCtrlReg.SDO1_CONFIG = 0;           // Дополнительная настройка (0 = триггерное состояние)

    // Настройка регистра управления
    device->data.ADDRESS = ADS867x_SDO_CTL_LSW;
    device->data.COMMAND = ADS867x_WRITE_HWORD; // Полное обновление 16-битного слова
    device->data.REG_DATA_LSB = SdoCtrlReg.SdoCtrlReg_LSW;
    device->data.REG_DATA_MSB = SdoCtrlReg.SdoCtrlReg_LSW >> 8;

    // Передача данных через SPI
    device->spi_tx(&device->data.Command, 4);
}


/*
 * @brief Selects data format for the output data.
 * 
 * @param *device : Instance of the general data struct ADS867x_GInst_t.
 * @param data_val : It controls the data value output by the converter, see the ADS867x_DataVal enum.
 * @param par_en : If 0b output data does not contain parity information, if 1b two parity bits (ADC output
 *	      and output data frame) are appended to the LSBs of the output data. See ADS867x_PAR_t enum.
 * @param range_incl : 0b do not include the range configuration register value, 1b include the range configuration
 *        register value. See ADS867x_RANGE_INCL_t enum.
 * @param in_active_alarm_incl : Control to include the active input ALARM flags in the SDO-x output bit stream.
 *        See the ADS867x_InActiveAlarm enum.
 * @param vdd_active_alarm_incl : Control to include the active VDD ALARM flags in the SDO-x output bit stream.
 *        See the ADS867x_VddActiveAlarm enum.
 * @param device_addr_incl :  0b do not include the register value, 1b include the register value. See ADS867x_DEV_ADDR_INCL_t enum.
 *
 **/
void ADS867x_DataOutCtrl_DaisyChain(ADS867x_GInst_t* device, ADS867x_DATA_VAL_t data_val, ADS867x_PAR_t par_en,
                                    ADS867x_RANGE_INCL_t range_incl, ADS867x_IN_ACTIVE_ALARM_t in_active_alarm_incl,
                                    ADS867x_VDD_ACTIVE_ALARM_t vdd_active_alarm_incl, ADS867x_DEV_ADDR_INCL_t device_addr_incl,
                                    uint8_t daisy_chain_mode)
{
    ADS867x_DataOut_t DataOutReg;

    // Настройка регистра данных
    DataOutReg.DATA_VAL = data_val;
    DataOutReg.PAR_EN = par_en;
    DataOutReg.RANGE_INCL = range_incl;
    DataOutReg.IN_ACTIVE_ALARM_INCL = in_active_alarm_incl;
    DataOutReg.VDD_ACTIVE_ALARM_INCL = vdd_active_alarm_incl;
    DataOutReg.DEVICE_ADDR_INCL = device_addr_incl;

    // Если включен режим daisy-chain
    if (daisy_chain_mode) {
        DataOutReg.DATA_VAL |= 0x01; // Условное значение для активации daisy-chain
    }

    device->data.ADDRESS = ADS867x_DATAOUT_CTL_LSW;
    device->data.COMMAND = ADS867x_WRITE_HWORD;
    device->data.REG_DATA_LSB = DataOutReg.DataOutReg_LSW;
    device->data.REG_DATA_MSB = DataOutReg.DataOutReg_LSW >> 8;

    // Отправка данных через SPI
    device->spi_tx(&device->data.Command, 4);
}


/*
 * @brief Selects either internal or external reference and selects input range.
 * 
 * @param *device : Instance of the general data struct ADS867x_GInst_t.
 * @param range_sel : It selects one of nine input ranges, see the ADS867x_INPUT_RANGE_t enum.
 * @param intref_dis : 0 enables internal voltage reference, 1 disables this one.
 *
 **/
void ADS867x_RangeSel(ADS867x_GInst_t* device, ADS867x_INPUT_RANGE_t range, uint8_t device_id) {
    // Подготовка данных для записи в регистр выбора диапазона
    uint16_t range_data = (uint16_t)range;

    // Установка уникального идентификатора устройства (device_id) в случае работы в режиме daisy-chain
    device->data.ADDRESS = ADS867x_RANGE_SEL_LSW | (device_id << 4);

    // Команда записи младшего слова регистра
    device->data.COMMAND = ADS867x_WRITE_LSB;
    device->data.REG_DATA_LSB = range_data & 0xFF; // Младший байт
    device->data.REG_DATA_MSB = 0; // Старший байт не используется для LSB

    // Передача данных
    device->spi_tx(&device->data.Command, 4);
    device->delay(1);

    // Команда записи старшего слова регистра
    device->data.ADDRESS = ADS867x_RANGE_SEL_HSW | (device_id << 4);
    device->data.COMMAND = ADS867x_WRITE_LSB;
    device->data.REG_DATA_LSB = (range_data >> 8) & 0xFF; // Старший байт
    device->data.REG_DATA_MSB = 0; // Старший байт не используется для LSB

    // Передача данных
    device->spi_tx(&device->data.Command, 4);
    device->delay(1);
}

/*
 * @brief Return output condition of the alarm flags.
 * 
 * @param *device : Instance of the general data struct ADS867x_GInst_t.
 *
 **/
ADS867x_Alarm_t ADS867x_ReadAlarm_DaisyChain(ADS867x_GInst_t* device, uint8_t num_devices) {
    ADS867x_Alarm_t AlarmRegs[num_devices];
    uint8_t spi_rx_buffer[2 * num_devices]; // Буфер для всех устройств
    
    // Установить адрес и команду
    device->data.ADDRESS = ADS867x_ALARM_LSW;
    device->data.COMMAND = ADS867x_READ_HWORD;
    device->data.REG_DATA_LSB = 0;
    device->data.REG_DATA_MSB = 0;
    
    // Передача команды через SPI
    device->spi_tx(&device->data.Command, 4);
    device->delay(1);
    
    // Чтение данных с устройств через SPI
    device->spi_rx(spi_rx_buffer, 2 * num_devices);
    
    // Распаковка данных для каждого устройства
    for (uint8_t i = 0; i < num_devices; i++) {
        AlarmRegs[i].AlarmReg_LSW = (uint16_t)(spi_rx_buffer[2 * i] << 8 | spi_rx_buffer[2 * i + 1]);
    }
    
    return AlarmRegs[0]; // Вернуть данные первого устройства (или переделать под конкретный запрос)
}


/*
 * @brief Sets hysteresis and high threshold for the input alarm.
 * 
 * @param *device : Instance of the general data struct ADS867x_GInst_t.
 * @param inp_alrm_high_th : 14-bit threshold for comparison is INP_ALRM_HIGH_TH.
 * @param inp_alrm_hyst : 4-bit hysteresis value for the input ALARM.
 *
 **/
void ADS867x_SetAlarmHTh_Daisy(ADS867x_GInst_t* device, uint16_t inp_alrm_high_th, uint8_t inp_alrm_hyst, uint8_t device_addr)
{
    ADS867x_AlarmHTh_t AlarmHthReg;
    
    AlarmHthReg.RESERVED1_0 = 0;
    AlarmHthReg.RESERVED27_24 = 0;
    AlarmHthReg.INP_ALRM_HIGH_TH = inp_alrm_high_th;    
    AlarmHthReg.INP_ALRM_HYST = inp_alrm_hyst;
    
    // Адресация устройства в режиме daisy-chain
    device->data.ADDRESS = ADS867x_ALARM_H_TH_LSW | (device_addr << 8);
    device->data.COMMAND = ADS867x_WRITE_HWORD;
    device->data.REG_DATA_LSB = AlarmHthReg.AlarmHTh_LSW;
    device->data.REG_DATA_MSB = AlarmHthReg.AlarmHTh_LSW >> 8;
    device->spi_tx(&device->data.Command, 4);
    device->delay(1);
    
    device->data.ADDRESS = ADS867x_ALARM_H_TH_HSW | (device_addr << 8);
    device->data.REG_DATA_LSB = AlarmHthReg.AlarmHTh_HSW;
    device->data.REG_DATA_MSB = AlarmHthReg.AlarmHTh_HSW >> 8;
    device->spi_tx(&device->data.Command, 4);    
}


/*
 * @brief Sets low threshold for the input alarm.
 * 
 * @param *device : Instance of the general data struct ADS867x_GInst_t.
 * @param inp_alrm_low_th : 14-bit threshold for comparison is INP_ALRM_LOW_TH.
 *
 **/
void ADS867x_SetAlarmLTh(ADS867x_GInst_t* device, uint8_t device_id, uint16_t inp_alrm_low_th)
{
    ADS867x_AlarmLTh_t AlarmLThReg;

    // Настройка регистра для нижнего порога тревоги
    AlarmLThReg.RESERVED1_0 = 0;
    AlarmLThReg.INP_ALRM_LOW_TH = inp_alrm_low_th;

    // Установка адреса устройства в цепочке (daisy-chain)
    device->data.ADDRESS = ADS867x_ALARM_L_TH_LSW | (device_id << 4); // device_id сдвигается для адресации
    device->data.COMMAND = ADS867x_WRITE_HWORD;
    device->data.REG_DATA_LSB = AlarmLThReg.AlarmLTh_LSW;
    device->data.REG_DATA_MSB = AlarmLThReg.AlarmLTh_LSW >> 8;

    // Передача данных
    device->spi_tx(&device->data.Command, 4);
}


/* Hardware dependent functions. */

/*
 * @brief
 *
 **/
static void ADS867x_SPI_Tx(uint8_t *pData, uint8_t size) {
    HAL_StatusTypeDef status;

    // Ensure the size matches the requirements of daisy-chain communication.
    if (size % 4 != 0) {
        // Size must be a multiple of 4 bytes (32 bits) for proper daisy-chain operation.
        return;
    }

    // Chip Select Low to start communication
    HAL_GPIO_WritePin(CS_ADC_GPIO_Port, CS_ADC_Pin, GPIO_PIN_RESET);

    // Transmit data
    status = HAL_SPI_Transmit(&hspi1, pData, size, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        // Handle transmission error
        HAL_GPIO_WritePin(CS_ADC_GPIO_Port, CS_ADC_Pin, GPIO_PIN_SET);
        return;
    }

    // Delay to ensure proper data propagation in daisy-chain mode
    HAL_Delay(1); // Adjust delay based on clock speed and chain length

    // Chip Select High to end communication
    HAL_GPIO_WritePin(CS_ADC_GPIO_Port, CS_ADC_Pin, GPIO_PIN_SET);
}


/*
 * @brief
 * 
 *
 **/
void ADS867x_SPI_Rx_Daisy(ADS867x_GInst_t *device, uint8_t *pData, uint16_t size, uint8_t num_devices) {
    uint16_t timeout = 10000;
    uint8_t total_size = size * num_devices;

    // Ensure that the CS line is active for the entire chain
    HAL_GPIO_WritePin(CS_ADC_GPIO_Port, CS_ADC_Pin, GPIO_PIN_RESET);

    // Start SPI reception to read data from the entire chain
    HAL_SPI_Receive(&hspi1, pData, total_size, timeout);

    // Ensure that the CS line is inactive after the transaction
    HAL_GPIO_WritePin(CS_ADC_GPIO_Port, CS_ADC_Pin, GPIO_PIN_SET);

    // Process received data for each device in the chain
    for (uint8_t i = 0; i < num_devices; ++i) {
        uint8_t *device_data = pData + (i * size);
        // Example: Perform device-specific data processing
        ProcessDeviceData(device, device_data, size);
    }
}

/* Notes for Daisy-Chain Mode:
 * 1. Ensure proper timing: tSU_CSCK, tHT_CKDO, tSU_DSYCK, and tHT_CKDSY.
 * 2. Verify that SDO of one ADC is connected to SDI of the next ADC.
 * 3. Match SPI protocol (CPOL = 0, CPHASE = 0) with datasheet requirements.
 * 4. Confirm SCLK frequency does not exceed 17 MHz.
 */


/*
 * @brief SPI chip select.
 * 
 * @param gpio : Either CS_ADC_GPIO_Port or CS_DAC_GPIO_Port.
 * @param gpio_pin : Either CS_ADC_Pin or CS_DAC_Pin.
 * @param state : Either GPIO_PIN_SET or GPIO_PIN_RESET.
 *
 **/
static void ADS867x_SPI_CS(GPIO_TypeDef* gpio, uint16_t gpio_pin, uint8_t state, GPIO_TypeDef* daisy_gpio, uint16_t daisy_pin) {
    if (state) {
        // Установка CS в высокий уровень
        HAL_GPIO_WritePin(gpio, gpio_pin, GPIO_PIN_SET);

        // Обновление DAISY-пина для завершения передачи
        if (daisy_gpio != NULL) {
            HAL_GPIO_WritePin(daisy_gpio, daisy_pin, GPIO_PIN_RESET);
        }
    } else {
        // Установка CS в низкий уровень
        HAL_GPIO_WritePin(gpio, gpio_pin, GPIO_PIN_RESET);

        // Активируем DAISY для следующего устройства
        if (daisy_gpio != NULL) {
            HAL_GPIO_WritePin(daisy_gpio, daisy_pin, GPIO_PIN_SET);
        }
    }
}

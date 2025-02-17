/*
 * @brief Common driver for INA226 from Texas Instruments.
 * Created by Gggggleb
 *
 **/

#include "ina226.h"
#include "i2c.h"

/*
 * @brief Initializes the INA226 with default configuration.
 * @param hi2c: I2C handle.
 * @param address: I2C address of INA226.
 */
void INA226_Init(I2C_HandleTypeDef *hi2c, uint8_t address) {
    INA226_Config_t default_config = {
        .avg = 0x0000,  // No averaging
        .bus_conv_time = 0x001C,  // 1.1 ms conversion time
        .shunt_conv_time = 0x001C,  // 1.1 ms conversion time
        .mode = INA226_MODE_SHUNT_BUS_CONTINUOUS
    };
    INA226_SetConfiguration(hi2c, address, &default_config);
}

/*
 * @brief Writes to an INA226 register.
 * @param hi2c: I2C handle.
 * @param address: I2C address of INA226.
 * @param reg: Register address to write.
 * @param data: Data to write.
 * @return HAL status.
 */
HAL_StatusTypeDef INA226_WriteRegister(I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t reg, uint16_t data) {
    uint8_t buffer[3];
    buffer[0] = reg;
    buffer[1] = (data >> 8) & 0xFF;  // MSB
    buffer[2] = data & 0xFF;         // LSB
    return HAL_I2C_Master_Transmit(hi2c, (address << 1), buffer, 3, HAL_MAX_DELAY);
}

/*
 * @brief Reads from an INA226 register.
 * @param hi2c: I2C handle.
 * @param address: I2C address of INA226.
 * @param reg: Register address to read.
 * @param data: Pointer to store the read data.
 * @return HAL status.
 */
HAL_StatusTypeDef INA226_ReadRegister(I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t reg, uint16_t *data) {
    uint8_t buffer[2];
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(hi2c, (address << 1), &reg, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        return status;
    }
    status = HAL_I2C_Master_Receive(hi2c, (address << 1), buffer, 2, HAL_MAX_DELAY);
    if (status == HAL_OK) {
        *data = (buffer[0] << 8) | buffer[1];
    }
    return status;
}

/*
 * @brief Sets the configuration of INA226.
 * @param hi2c: I2C handle.
 * @param address: I2C address of INA226.
 * @param config: Pointer to the configuration structure.
 * @return HAL status.
 */
HAL_StatusTypeDef INA226_SetConfiguration(I2C_HandleTypeDef *hi2c, uint8_t address, INA226_Config_t *config) {
    uint16_t config_data = (config->avg << 9) | (config->bus_conv_time << 6) |
                           (config->shunt_conv_time << 3) | (config->mode);
    return INA226_WriteRegister(hi2c, address, INA226_REG_CONFIGURATION, config_data);
}

/*
 * @brief Calibrates the INA226.
 * @param hi2c: I2C handle.
 * @param address: I2C address of INA226.
 * @param shunt_resistance: Shunt resistor value in ohms.
 * @param max_expected_current: Maximum expected current in amperes.
 * @return HAL status.
 */
HAL_StatusTypeDef INA226_Calibrate(I2C_HandleTypeDef *hi2c, uint8_t address, float shunt_resistance, float max_expected_current) {
    float current_lsb = max_expected_current / 32768.0f;
    uint16_t calibration = (uint16_t)(0.00512 / (current_lsb * shunt_resistance));
    return INA226_WriteRegister(hi2c, address, INA226_REG_CALIBRATION, calibration);
}

/*
 * @brief Reads the bus voltage from INA226.
 * @param hi2c: I2C handle.
 * @param address: I2C address of INA226.
 * @param voltage: Pointer to store the bus voltage in volts.
 * @return HAL status.
 */
HAL_StatusTypeDef INA226_ReadBusVoltage(I2C_HandleTypeDef *hi2c, uint8_t address, float *voltage) {
    uint16_t raw_data;
    HAL_StatusTypeDef status = INA226_ReadRegister(hi2c, address, INA226_REG_BUS_VOLTAGE, &raw_data);
    if (status == HAL_OK) {
        *voltage = raw_data * 0.00125f;  // 1.25 mV per LSB
    }
    return status;
}

/*
 * @brief Reads the shunt voltage from INA226.
 * @param hi2c: I2C handle.
 * @param address: I2C address of INA226.
 * @param shunt_voltage: Pointer to store the shunt voltage in volts.
 * @return HAL status.
 */
HAL_StatusTypeDef INA226_ReadShuntVoltage(I2C_HandleTypeDef *hi2c, uint8_t address, float *shunt_voltage) {
    uint16_t raw_data;
    HAL_StatusTypeDef status = INA226_ReadRegister(hi2c, address, INA226_REG_SHUNT_VOLTAGE, &raw_data);
    if (status == HAL_OK) {
        *shunt_voltage = raw_data * 0.0000025f;  // 2.5 uV per LSB
    }
    return status;
}

/*
 * @brief Reads the current from INA226.
 * @param hi2c: I2C handle.
 * @param address: I2C address of INA226.
 * @param current: Pointer to store the current in amperes.
 * @return HAL status.
 */
HAL_StatusTypeDef INA226_ReadCurrent(I2C_HandleTypeDef *hi2c, uint8_t address, float *current) {
    uint16_t raw_data;
    HAL_StatusTypeDef status = INA226_ReadRegister(hi2c, address, INA226_REG_CURRENT, &raw_data);
    if (status == HAL_OK) {
        *current = raw_data * 0.001;  // Current_LSB assumed to be 1 mA/bit for simplicity
    }
    return status;
}

/*
 * @brief Reads the power from INA226.
 * @param hi2c: I2C handle.
 * @param address: I2C address of INA226.
 * @param power: Pointer to store the power in watts.
 * @return HAL status.
 */
HAL_StatusTypeDef INA226_ReadPower(I2C_HandleTypeDef *hi2c, uint8_t address, float *power) {
    uint16_t raw_data;
    HAL_StatusTypeDef status = INA226_ReadRegister(hi2c, address, INA226_REG_POWER, &raw_data);
    if (status == HAL_OK) {
        *power = raw_data * 0.025;  // Power_LSB assumed to be 25 mW/bit for simplicity
    }
    return status;
}

/*
 * @brief Sets the alert limit of INA226.
 * @param hi2c: I2C handle.
 * @param address: I2C address of INA226.
 * @param alert_limit: Alert limit value to set.
 * @return HAL status.
 */
HAL_StatusTypeDef INA226_SetAlertLimit(I2C_HandleTypeDef *hi2c, uint8_t address, uint16_t alert_limit) {
    return INA226_WriteRegister(hi2c, address, INA226_REG_ALERT_LIMIT, alert_limit);
}

/*
 * @brief Reads the alert limit of INA226.
 * @param hi2c: I2C handle.
 * @param address: I2C address of INA226.
 * @param alert_limit: Pointer to store the alert limit value.
 * @return HAL status.
 */
HAL_StatusTypeDef INA226_ReadAlertLimit(I2C_HandleTypeDef *hi2c, uint8_t address, uint16_t *alert_limit) {
    return INA226_ReadRegister(hi2c, address, INA226_REG_ALERT_LIMIT, alert_limit);
}

/*
 * @brief Sets the Mask/Enable register of INA226.
 * @param hi2c: I2C handle.
 * @param address: I2C address of INA226.
 * @param mask_enable: Value to set in Mask/Enable register.
 * @return HAL status.
 */
HAL_StatusTypeDef INA226_SetMaskEnable(I2C_HandleTypeDef *hi2c, uint8_t address, uint16_t mask_enable) {
    return INA226_WriteRegister(hi2c, address, INA226_REG_MASK_ENABLE, mask_enable);
}

/*
 * @brief Reads the Mask/Enable register of INA226.
 * @param hi2c: I2C handle.
 * @param address: I2C address of INA226.
 * @param mask_enable: Pointer to store the Mask/Enable register value.
 * @return HAL status.
 */
HAL_StatusTypeDef INA226_ReadMaskEnable(I2C_HandleTypeDef *hi2c, uint8_t address, uint16_t *mask_enable) {
    return INA226_ReadRegister(hi2c, address, INA226_REG_MASK_ENABLE, mask_enable);
}

/*
 * @brief Reads the Manufacturer ID register of INA226.
 * @param hi2c: I2C handle.
 * @param address: I2C address of INA226.
 * @param manufacturer_id: Pointer to store the Manufacturer ID value.
 * @return HAL status.
 */
HAL_StatusTypeDef INA226_ReadManufacturerID(I2C_HandleTypeDef *hi2c, uint8_t address, uint16_t *manufacturer_id) {
    return INA226_ReadRegister(hi2c, address, INA226_REG_MANUFACTURER_ID, manufacturer_id);
}

/*
 * @brief Reads the Die ID register of INA226.
 * @param hi2c: I2C handle.
 * @param address: I2C address of INA226.
 * @param die_id: Pointer to store the Die ID value.
 * @return HAL status.
 */
HAL_StatusTypeDef INA226_ReadDieID(I2C_HandleTypeDef *hi2c, uint8_t address, uint16_t *die_id) {
    return INA226_ReadRegister(hi2c, address, INA226_REG_DIE_ID, die_id);
}

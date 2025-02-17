/*
 * @brief Common header for INA226 from Texas Instruments.
 * Created by Ggggggleb
 *
 */

#ifndef INA226_H_
#define INA226_H_

#ifndef NULL
#define NULL (void*)0
#endif /* NULL */

#include "stm32f4xx_hal.h"

/* I2C configuration. */
#define INA226_I2C_ADDRESS          0x40  /* Default I2C address for INA226 */

/* INA226 Register Map */
typedef enum {
    INA226_REG_CONFIGURATION = 0x00, /* Configuration register */
    INA226_REG_SHUNT_VOLTAGE = 0x01, /* Shunt voltage register */
    INA226_REG_BUS_VOLTAGE   = 0x02, /* Bus voltage register */
    INA226_REG_POWER         = 0x03, /* Power register */
    INA226_REG_CURRENT       = 0x04, /* Current register */
    INA226_REG_CALIBRATION   = 0x05, /* Calibration register */
    INA226_REG_MASK_ENABLE   = 0x06, /* Mask/Enable register */
    INA226_REG_ALERT_LIMIT   = 0x07, /* Alert limit register */
    INA226_REG_MANUFACTURER_ID = 0xFE, /* Manufacturer ID register */
    INA226_REG_DIE_ID        = 0xFF  /* Die ID register */
} INA226_REG_MAP_t;

/* Operating Modes */
typedef enum {
    INA226_MODE_POWER_DOWN = 0x00,
    INA226_MODE_SHUNT_TRIGGERED = 0x01,
    INA226_MODE_BUS_TRIGGERED = 0x02,
    INA226_MODE_SHUNT_BUS_TRIGGERED = 0x03,
    INA226_MODE_POWER_DOWN_ALT = 0x04,
    INA226_MODE_SHUNT_CONTINUOUS = 0x05,
    INA226_MODE_BUS_CONTINUOUS = 0x06,
    INA226_MODE_SHUNT_BUS_CONTINUOUS = 0x07
} INA226_OPERATING_MODE_t;

/* Configuration structure */
typedef struct {
    uint16_t avg;            /* Averaging mode */
    uint16_t bus_conv_time;  /* Bus voltage conversion time */
    uint16_t shunt_conv_time; /* Shunt voltage conversion time */
    INA226_OPERATING_MODE_t mode; /* Operating mode */
} INA226_Config_t;

/* Public function prototypes */
void INA226_Init(I2C_HandleTypeDef *hi2c, uint8_t address);
HAL_StatusTypeDef INA226_WriteRegister(I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t reg, uint16_t data);
HAL_StatusTypeDef INA226_ReadRegister(I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t reg, uint16_t *data);
HAL_StatusTypeDef INA226_SetConfiguration(I2C_HandleTypeDef *hi2c, uint8_t address, INA226_Config_t *config);
HAL_StatusTypeDef INA226_Calibrate(I2C_HandleTypeDef *hi2c, uint8_t address, float shunt_resistance, float max_expected_current);
HAL_StatusTypeDef INA226_ReadBusVoltage(I2C_HandleTypeDef *hi2c, uint8_t address, float *voltage);
HAL_StatusTypeDef INA226_ReadShuntVoltage(I2C_HandleTypeDef *hi2c, uint8_t address, float *shunt_voltage);
HAL_StatusTypeDef INA226_ReadCurrent(I2C_HandleTypeDef *hi2c, uint8_t address, float *current);
HAL_StatusTypeDef INA226_ReadPower(I2C_HandleTypeDef *hi2c, uint8_t address, float *power);
HAL_StatusTypeDef INA226_SetAlertLimit(I2C_HandleTypeDef *hi2c, uint8_t address, uint16_t alert_limit);
HAL_StatusTypeDef INA226_ReadAlertLimit(I2C_HandleTypeDef *hi2c, uint8_t address, uint16_t *alert_limit);
HAL_StatusTypeDef INA226_SetMaskEnable(I2C_HandleTypeDef *hi2c, uint8_t address, uint16_t mask_enable);
HAL_StatusTypeDef INA226_ReadMaskEnable(I2C_HandleTypeDef *hi2c, uint8_t address, uint16_t *mask_enable);
HAL_StatusTypeDef INA226_ReadManufacturerID(I2C_HandleTypeDef *hi2c, uint8_t address, uint16_t *manufacturer_id);
HAL_StatusTypeDef INA226_ReadDieID(I2C_HandleTypeDef *hi2c, uint8_t address, uint16_t *die_id);

#endif /* INA226_H_ */

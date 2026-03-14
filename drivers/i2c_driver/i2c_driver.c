/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "i2c_driver.h"
#include "common.h"
#if (HWREV == 1)
#include "i2c.h"
#endif
#include "main.h"

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define I2C_TIMEOUT_MS (100U)

/*---------------------------------------------------------------------------
 * Private Variables (peripheral -> handle mapping, same pattern as uart_driver)
 *---------------------------------------------------------------------------*/
#if (HWREV == 1)
extern I2C_HandleTypeDef hi2c5;

typedef struct {
    I2C_HandleTypeDef *hi2c;
} i2c_port_t;

static const i2c_port_t i2c_ports[I2C_COUNT] = {
    [I2C_EEPROM] = { .hi2c = &hi2c5 },
};

static i2c_driver_status_E hal_to_status(HAL_StatusTypeDef status)
{
    if (status == HAL_OK)
        return I2C_DRIVER_SUCCESS;
    if (status == HAL_TIMEOUT)
        return I2C_DRIVER_TIMEOUT;
    return I2C_DRIVER_ERROR;
}
#endif

/*---------------------------------------------------------------------------
 * Public Function Implementations
 *---------------------------------------------------------------------------*/

#if (HWREV == 1)

i2c_driver_status_E i2c_driver_write_register_16(i2c_id_t id, uint16_t dev_addr, uint16_t reg,
                                                 const uint8_t* data, uint8_t len)
{
    if (id >= I2C_COUNT || data == NULL || len == 0U)
        return I2C_DRIVER_ERROR;

    I2C_HandleTypeDef *hi2c = i2c_ports[id].hi2c;
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(hi2c, dev_addr << 1, reg,
                                                 I2C_MEMADD_SIZE_16BIT, (uint8_t*)data, len,
                                                 I2C_TIMEOUT_MS);
    return hal_to_status(status);
}

i2c_driver_status_E i2c_driver_read_register_16(i2c_id_t id, uint16_t dev_addr, uint16_t reg, uint8_t* data, uint8_t len)
{
    if (id >= I2C_COUNT || data == NULL || len == 0U)
        return I2C_DRIVER_ERROR;

    I2C_HandleTypeDef *hi2c = i2c_ports[id].hi2c;
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c, dev_addr << 1, reg,
                                                I2C_MEMADD_SIZE_16BIT, data, len,
                                                I2C_TIMEOUT_MS);
    return hal_to_status(status);
}

#elif (HWREV == 0)

i2c_driver_status_E i2c_driver_write_register_16(i2c_id_t id, uint16_t dev_addr, uint16_t reg, const uint8_t* data, uint8_t len)
{
    (void)id;
    (void)dev_addr;
    (void)reg;
    (void)data;
    (void)len;
    return I2C_DRIVER_ERROR;
}

i2c_driver_status_E i2c_driver_read_register_16(i2c_id_t id, uint16_t dev_addr, uint16_t reg, uint8_t* data, uint8_t len)
{
    (void)id;
    (void)dev_addr;
    (void)reg;
    (void)data;
    (void)len;
    return I2C_DRIVER_ERROR;
}

#endif
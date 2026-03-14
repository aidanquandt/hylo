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
 * Private Variables
 *---------------------------------------------------------------------------*/
#if (HWREV == 1)
extern I2C_HandleTypeDef hi2c5;
#define I2C_DRIVER (&hi2c5)
#endif

/*---------------------------------------------------------------------------
 * Public Function Implementations
 *---------------------------------------------------------------------------*/

#if (HWREV == 1)

i2c_driver_status_E i2c_driver_write_register_16(uint16_t dev_addr, uint16_t reg,
                                                 const uint8_t* data, uint8_t len)
{
    if (data == NULL || len == 0U)
    {
        return I2C_DRIVER_ERROR;
    }

    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(I2C_DRIVER, dev_addr << 1, reg,
                                                  I2C_MEMADD_SIZE_16BIT, (uint8_t*)data, len,
                                                  I2C_TIMEOUT_MS);

    if (status == HAL_OK)
    {
        return I2C_DRIVER_SUCCESS;
    }
    else if (status == HAL_TIMEOUT)
    {
        return I2C_DRIVER_TIMEOUT;
    }
    else
    {
        return I2C_DRIVER_ERROR;
    }
}

i2c_driver_status_E i2c_driver_read_register_16(uint16_t dev_addr, uint16_t reg, uint8_t* data, uint8_t len)
{
    if (data == NULL || len == 0U)
    {
        return I2C_DRIVER_ERROR;
    }

    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(I2C_DRIVER, dev_addr << 1, reg,
                                                 I2C_MEMADD_SIZE_16BIT, data, len,
                                                 I2C_TIMEOUT_MS);


    if (status == HAL_OK)
    {
        return I2C_DRIVER_SUCCESS;
    }
    else if (status == HAL_TIMEOUT)
    {
        return I2C_DRIVER_TIMEOUT;
    }
    else
    {
        return I2C_DRIVER_ERROR;
    }
}

#elif (HWREV == 0)

i2c_driver_status_E platform_i2c_write_register_16(uint16_t dev_addr, uint16_t reg, const uint8_t* data, uint8_t len)
{
    (void)dev_addr;
    (void)reg;
    (void)data;
    (void)len;
    return I2C_DRIVER_ERROR;
}

i2c_driver_status_E platform_i2c_read_register_16(uint16_t dev_addr, uint16_t reg, uint8_t* data, uint8_t len)
{
    (void)dev_addr;
    (void)reg;
    (void)data;
    (void)len;
    return I2C_DRIVER_ERROR;
}

#endif
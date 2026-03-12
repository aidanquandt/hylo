#include "eeprom_driver.h"
#include "FreeRTOS.h"
#include "platform_i2c.h"
#include "task.h"

/*---------------------------------------------------------------------------
 * Private Helpers
 *---------------------------------------------------------------------------*/
static eeprom_status_e eeprom_from_platform_status(platform_i2c_status_E status)
{
	if (status == PLATFORM_I2C_SUCCESS)
	{
		return EEPROM_STATUS_OK;
	}
	else if (status == PLATFORM_I2C_TIMEOUT)
	{
		return EEPROM_STATUS_TIMEOUT;
	}
	else
	{
		return EEPROM_STATUS_ERROR;
	}
}

static bool eeprom_range_is_valid(uint16_t start_reg_addr, uint16_t len)
{
	if (len == 0U)
	{
		return false;
	}

	if (start_reg_addr >= M24C64_SIZE_BYTES)
	{
		return false;
	}

	uint32_t end_addr = (uint32_t)start_reg_addr + (uint32_t)len;
	return (end_addr <= M24C64_SIZE_BYTES);
}

/*---------------------------------------------------------------------------
 * Public Function Implementations
 *---------------------------------------------------------------------------*/
eeprom_status_e eeprom_write_u8(uint16_t reg_addr, uint8_t value)
{
	if (!eeprom_range_is_valid(reg_addr, 1U))
	{
		return EEPROM_STATUS_INVALID_PARAM;
	}

	platform_i2c_status_E status =
		platform_i2c_write_register_16(M24C64_I2C_ADDR_7BIT, reg_addr, &value, 1U);

	if (status == PLATFORM_I2C_SUCCESS)
	{
		vTaskDelay(pdMS_TO_TICKS(M24C64_WRITE_CYCLE_DELAY_MS));
	}

	return eeprom_from_platform_status(status);
}

eeprom_status_e eeprom_read_u8(uint16_t reg_addr, uint8_t* value)
{
	if (value == NULL || !eeprom_range_is_valid(reg_addr, 1U))
	{
		return EEPROM_STATUS_INVALID_PARAM;
	}

	platform_i2c_status_E status =
		platform_i2c_read_register_16(M24C64_I2C_ADDR_7BIT, reg_addr, value, 1U);

	return eeprom_from_platform_status(status);
}

eeprom_status_e eeprom_write_buffer(uint16_t start_reg_addr, const uint8_t* data, uint16_t len)
{
	if (data == NULL || !eeprom_range_is_valid(start_reg_addr, len))
	{
		return EEPROM_STATUS_INVALID_PARAM;
	}

	for (uint16_t i = 0; i < len; i++)
	{
		platform_i2c_status_E status = platform_i2c_write_register_16(
			M24C64_I2C_ADDR_7BIT, (uint16_t)(start_reg_addr + i), &data[i], 1U);

		if (status != PLATFORM_I2C_SUCCESS)
		{
			return eeprom_from_platform_status(status);
		}

		vTaskDelay(pdMS_TO_TICKS(M24C64_WRITE_CYCLE_DELAY_MS));
	}

	return EEPROM_STATUS_OK;
}

eeprom_status_e eeprom_read_buffer(uint16_t start_reg_addr, uint8_t* data, uint16_t len)
{
	if (data == NULL || !eeprom_range_is_valid(start_reg_addr, len))
	{
		return EEPROM_STATUS_INVALID_PARAM;
	}

	uint16_t offset = 0U;
	while (offset < len)
	{
		uint16_t remaining  = (uint16_t)(len - offset);
		uint8_t chunk_len   = (remaining > 255U) ? 255U : (uint8_t)remaining;
		uint16_t chunk_addr = (uint16_t)(start_reg_addr + offset);

		platform_i2c_status_E status =
			platform_i2c_read_register_16(M24C64_I2C_ADDR_7BIT, chunk_addr, &data[offset], chunk_len);

		if (status != PLATFORM_I2C_SUCCESS)
		{
			return eeprom_from_platform_status(status);
		}

		offset = (uint16_t)(offset + chunk_len);
	}

	if (offset == len)
	{
		return EEPROM_STATUS_OK;
	}

	return EEPROM_STATUS_ERROR;
}

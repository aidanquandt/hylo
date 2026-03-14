#pragma once

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "common.h"

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define M24C64_I2C_ADDR_7BIT (0x50U)
#define M24C64_SIZE_BYTES (8192U)
#define M24C64_WRITE_CYCLE_DELAY_MS (10U)

/* Named EEPROM register/address regions */
#define M24C64_REG_START (0x0000U)
#define M24C64_REG_END (0x1FFFU)

/*---------------------------------------------------------------------------
 * Typedefs
 *---------------------------------------------------------------------------*/
typedef enum
{
	EEPROM_STATUS_OK = 0,
	EEPROM_STATUS_ERROR,
	EEPROM_STATUS_TIMEOUT,
	EEPROM_STATUS_INVALID_PARAM,
} eeprom_status_e;

/*---------------------------------------------------------------------------
 * Public Function Prototypes
 *---------------------------------------------------------------------------*/
eeprom_status_e eeprom_driver_write_buffer(uint16_t start_reg_addr, const uint8_t* data, uint16_t len);
eeprom_status_e eeprom_driver_read_buffer(uint16_t start_reg_addr, uint8_t* data, uint16_t len);
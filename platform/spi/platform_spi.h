#pragma once

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "common.h"

/*---------------------------------------------------------------------------
 * Typedefs
 *---------------------------------------------------------------------------*/

typedef enum
{
    PLATFORM_SPI_SUCCESS = 0,
    PLATFORM_SPI_ERROR   = -1,
    PLATFORM_SPI_TIMEOUT = -2,
} platform_spi_status_E;

typedef enum
{
    PLATFORM_SPI_SPEED_SLOW = 0, // ~2 MHz for initialization
    PLATFORM_SPI_SPEED_FAST = 1, // ~20 MHz for normal operation
} platform_spi_speed_E;

typedef enum
{
    PLATFORM_SPI_CS_UWB = 0, // UWB radio (DW3000) on SPI1
    PLATFORM_SPI_CS_IMU = 1, // IMU (BMI323) on SPI5
    PLATFORM_SPI_CS_COUNT
} platform_spi_cs_E;

/*---------------------------------------------------------------------------
 * Public Function Prototypes
 *---------------------------------------------------------------------------*/
platform_spi_status_E platform_spi_transfer(const uint8_t* tx_data, uint8_t* rx_data,
                                            uint16_t length);
platform_spi_status_E platform_spi_transmit(const uint8_t* data, uint16_t length);
platform_spi_status_E platform_spi_receive(uint8_t* data, uint16_t length);
platform_spi_status_E platform_spi_set_speed(platform_spi_speed_E speed);
void platform_spi_cs_low(platform_spi_cs_E cs_pin);
void platform_spi_cs_high(platform_spi_cs_E cs_pin);

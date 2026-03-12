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
    SPI_DRIVER_SUCCESS = 0,
    SPI_DRIVER_ERROR   = -1,
    SPI_DRIVER_TIMEOUT = -2,
} spi_driver_status_E;

typedef enum
{
    SPI_DRIVER_SPEED_SLOW = 0,
    SPI_DRIVER_SPEED_FAST = 1,
} spi_driver_speed_E;

typedef enum
{
    SPI_DRIVER_CS_UWB   = 0,
    SPI_DRIVER_CS_IMU_0 = 1,
    SPI_DRIVER_CS_IMU_1 = 2,
    SPI_DRIVER_CS_IMU_2 = 3,
    SPI_DRIVER_CS_IMU_3 = 4,
    SPI_DRIVER_CS_COUNT
} spi_driver_cs_E;

#define SPI_DRIVER_CS_IMU SPI_DRIVER_CS_IMU_0

/*---------------------------------------------------------------------------
 * Public Function Prototypes
 *---------------------------------------------------------------------------*/
spi_driver_status_E spi_driver_transfer(const uint8_t* tx_data, uint8_t* rx_data,
                                        uint16_t length);
spi_driver_status_E spi_driver_transmit(const uint8_t* data, uint16_t length);
spi_driver_status_E spi_driver_receive(uint8_t* data, uint16_t length);
spi_driver_status_E spi_driver_set_speed(spi_driver_speed_E speed);
void spi_driver_cs_low(spi_driver_cs_E cs_pin);
void spi_driver_cs_high(spi_driver_cs_E cs_pin);

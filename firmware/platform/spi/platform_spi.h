/*---------------------------------------------------------------------------
 * @file    platform_spi.h
 * @brief   Generic SPI hardware abstraction layer
 *---------------------------------------------------------------------------*/
#pragma once

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "common.h"

/*---------------------------------------------------------------------------
 * Types
 *---------------------------------------------------------------------------*/

typedef enum {
    PLATFORM_SPI_SUCCESS = 0,
    PLATFORM_SPI_ERROR = -1,
    PLATFORM_SPI_TIMEOUT = -2,
} platform_spi_status_E;

typedef enum {
    PLATFORM_SPI_SPEED_SLOW = 0,  // ~2 MHz for initialization
    PLATFORM_SPI_SPEED_FAST = 1,  // ~20 MHz for normal operation
} platform_spi_speed_E;

typedef enum {
    PLATFORM_SPI_CS_DW3000 = 0,  // DW3000 UWB on SPI1
    PLATFORM_SPI_CS_BMI323 = 1,  // BMI323 IMU on SPI5
    PLATFORM_SPI_CS_COUNT
} platform_spi_cs_E;

/*---------------------------------------------------------------------------
 * Public Function Prototypes
 *---------------------------------------------------------------------------*/

/**
 * @brief Transmit and receive data over SPI
 * @param tx_data Pointer to transmit buffer (can be NULL to send dummy bytes)
 * @param rx_data Pointer to receive buffer (can be NULL to discard received data)
 * @param length Number of bytes to transfer
 * @return Platform SPI status
 */
platform_spi_status_E platform_spi_transfer(const uint8_t *tx_data, uint8_t *rx_data, uint16_t length);

/**
 * @brief Transmit data over SPI
 * @param data Pointer to data to transmit
 * @param length Number of bytes to transmit
 * @return Platform SPI status
 */
platform_spi_status_E platform_spi_transmit(const uint8_t *data, uint16_t length);

/**
 * @brief Receive data over SPI
 * @param data Pointer to buffer for received data
 * @param length Number of bytes to receive
 * @return Platform SPI status
 */
platform_spi_status_E platform_spi_receive(uint8_t *data, uint16_t length);

/**
 * @brief Set SPI speed
 * @param speed Desired SPI speed (slow or fast)
 * @return Platform SPI status
 */
platform_spi_status_E platform_spi_set_speed(platform_spi_speed_E speed);

/**
 * @brief Set chip select low (assert)
 * @param cs_pin Chip select identifier (for multi-device support)
 */
void platform_spi_cs_low(platform_spi_cs_E cs_pin);

/**
 * @brief Set chip select high (deassert)
 * @param cs_pin Chip select identifier (for multi-device support)
 */
void platform_spi_cs_high(platform_spi_cs_E cs_pin);

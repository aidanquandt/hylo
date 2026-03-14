#pragma once

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "common.h"

/*---------------------------------------------------------------------------
 * SPI Driver - Interface-based API with per-bus mutex
 *
 * Two usage patterns:
 *
 * 1. ATOMIC (single transfer): Pass the interface (UWB, IMU_0, etc.) to
 *    transmit/receive/transfer. The driver takes the bus mutex, asserts CS,
 *    performs the operation, deasserts CS, and releases the mutex.
 *    Use for: single read or write.
 *
 * 2. SESSION (multi-step): Call spi_driver_acquire(interface), then one or
 *    more transmit/receive/transfer (no interface arg) and optionally
 *    set_speed, then spi_driver_release(interface). CS stays low and the
 *    bus is held for the whole sequence.
 *    Use for: UWB header+body, IMU addr+data, or changing speed.
 *
 * IMPORTANT: After acquire(), release() must always be called (including on
 * error paths), or the bus stays locked and CS held.
 *---------------------------------------------------------------------------*/

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

/** Selects which SPI device/interface (maps to HW bus + CS per HWREV). */
typedef enum
{
    SPI_DRIVER_INTERFACE_UWB   = 0,
    SPI_DRIVER_INTERFACE_IMU_0 = 1,
    SPI_DRIVER_INTERFACE_IMU_1 = 2,
    SPI_DRIVER_INTERFACE_IMU_2 = 3,
    SPI_DRIVER_INTERFACE_IMU_3 = 4,
    SPI_DRIVER_INTERFACE_COUNT
} spi_driver_interface_E;

#define SPI_DRIVER_INTERFACE_IMU SPI_DRIVER_INTERFACE_IMU_0

/*---------------------------------------------------------------------------
 * Public Function Prototypes
 *---------------------------------------------------------------------------*/

/** Call before any SPI use. Creates per-bus mutexes. */
void spi_driver_init(void);

/** Atomic: one transaction (mutex + CS) per call. */
spi_driver_status_E spi_driver_transmit(spi_driver_interface_E interface, const uint8_t* data,
                                        uint16_t length);
spi_driver_status_E spi_driver_receive(spi_driver_interface_E interface, uint8_t* data,
                                       uint16_t length);
spi_driver_status_E spi_driver_transfer(spi_driver_interface_E interface, const uint8_t* tx_data,
                                        uint8_t* rx_data, uint16_t length);

/** Session: hold bus and CS across multiple ops. Only valid between acquire and release.
 *  Returns SPI_DRIVER_SUCCESS if bus was acquired; call release() only after success. */
spi_driver_status_E spi_driver_acquire(spi_driver_interface_E interface);
void spi_driver_release(spi_driver_interface_E interface);

/** Session-only: pass same interface as acquire(); use after acquire(), before release(). */
spi_driver_status_E spi_driver_transfer_in_session(spi_driver_interface_E interface,
                                                   const uint8_t* tx_data, uint8_t* rx_data,
                                                   uint16_t length);
spi_driver_status_E spi_driver_transmit_in_session(spi_driver_interface_E interface,
                                                  const uint8_t* data, uint16_t length);
spi_driver_status_E spi_driver_receive_in_session(spi_driver_interface_E interface,
                                                  uint8_t* data, uint16_t length);

/** Session-only: set SPI clock speed for this interface. Use between acquire and release. */
spi_driver_status_E spi_driver_set_speed(spi_driver_interface_E interface,
                                         spi_driver_speed_E speed);

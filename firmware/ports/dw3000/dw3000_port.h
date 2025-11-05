/*---------------------------------------------------------------------------
 * @file    dw3000_port.h
 * @brief   Port layer for DW3000 UWB driver - Adapter between driver and platform
 *---------------------------------------------------------------------------*/
#pragma once

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "common.h"
#include "deca_interface.h"

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define DW3000_CS_PIN  (0U)  // CS identifier for platform layer

/** Return codes from port layer functions */
#define DW3000_SUCCESS  (0)   // Operation successful
#define DW3000_ERROR    (-1)  // Operation failed

/*---------------------------------------------------------------------------
 * Public Function Prototypes
 *---------------------------------------------------------------------------*/

/**
 * @brief Initialize the DW3000 port layer
 * @return Pointer to configured dwchip_s structure, or NULL on failure
 */
dwchip_t* dw3000_port_init(void);

/**
 * @brief Probe and initialize the DW3000 device
 * @param chip Pointer to chip structure from dw3000_port_init()
 * @return DW3000_SUCCESS on success, DW3000_ERROR on failure
 * @note This function performs both hardware probe and device initialization
 */
int dw3000_port_probe_and_init(dwchip_t *chip);

/**
 * @brief Wake up the DW3000 device using GPIO
 */
void dw3000_port_wakeup_device(void);

/**
 * @brief Check if DW3000 device ID is valid
 * @return 0 on success, negative error code on failure
 */
int dw3000_port_check_device_id(void);

/**
 * @brief Read DW3000 device ID register
 * @return 32-bit device ID value
 */
uint32_t dw3000_port_read_device_id(void);

/**
 * @brief Read DW3000 IC temperature
 * @return Temperature in degrees Celsius
 */
float dw3000_port_read_temperature(void);

/**
 * @brief Read DW3000 IC voltage
 * @return Voltage in volts
 */
float dw3000_port_read_voltage(void);

/**
 * @brief Read both temperature and voltage in single register access (optimized)
 * @param temperature Pointer to store temperature (can be NULL)
 * @param voltage Pointer to store voltage (can be NULL)
 */
void dw3000_port_read_temp_and_voltage(float *temperature, float *voltage);

/*---------------------------------------------------------------------------
 * Platform Compatibility Functions (required by Qorvo driver)
 *---------------------------------------------------------------------------*/

/**
 * @brief Microsecond delay
 * @param time_us Delay time in microseconds
 */
void deca_usleep(unsigned long time_us);

/**
 * @brief Millisecond delay
 * @param time_ms Delay time in milliseconds
 */
void deca_sleep(unsigned int time_ms);

/**
 * @brief Lock mutex for DW3000 access
 * @return IRQ status before disabling interrupts
 */
decaIrqStatus_t decamutexon(void);

/**
 * @brief Unlock mutex for DW3000 access
 * @param s IRQ status to restore
 */
void decamutexoff(decaIrqStatus_t s);

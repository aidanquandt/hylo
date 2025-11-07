/*---------------------------------------------------------------------------
 * @file    bmi323_port.h
 * @brief   Port layer for BMI323 sensor driver - Adapter between driver and platform
 *---------------------------------------------------------------------------*/
#pragma once

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "common.h"
#include "bmi3.h"

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define BMI323_CS_PIN  (1U)  // CS identifier for platform layer (adjust as needed)

/** Return codes from port layer functions */
#define BMI323_SUCCESS  (0)   // Operation successful
#define BMI323_ERROR    (-1)  // Operation failed

/*---------------------------------------------------------------------------
 * Public Function Prototypes
 *---------------------------------------------------------------------------*/

/**
 * @brief Initialize the BMI323 port layer
 * @return Pointer to configured bmi3_dev structure, or NULL on failure
 */
struct bmi3_dev* bmi323_port_init(void);

/**
 * @brief Probe and initialize the BMI323 device
 * @param dev Pointer to device structure from bmi323_port_init()
 * @return BMI323_SUCCESS on success, BMI323_ERROR on failure
 * @note This function performs both hardware probe and device initialization
 */
int bmi323_port_probe_and_init(struct bmi3_dev *dev);

/**
 * @brief Read BMI323 chip ID to verify communication
 * @param dev Pointer to device structure
 * @return 0 on success, negative error code on failure
 */
int bmi323_port_check_device_id(struct bmi3_dev *dev);

/**
 * @brief Delay function for BMI323 driver
 * @param period_us Delay period in microseconds
 * @param intf_ptr Interface pointer (unused)
 */
void bmi323_port_delay_us(uint32_t period_us, void *intf_ptr);

/**
 * @brief SPI read function for BMI323 driver
 * @param reg_addr Register address to read from
 * @param reg_data Buffer to store read data
 * @param len Number of bytes to read
 * @param intf_ptr Interface pointer (unused)
 * @return BMI3_OK on success, error code on failure
 */
int8_t bmi323_port_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);

/**
 * @brief SPI write function for BMI323 driver
 * @param reg_addr Register address to write to
 * @param reg_data Data to write
 * @param len Number of bytes to write
 * @param intf_ptr Interface pointer (unused)
 * @return BMI3_OK on success, error code on failure
 */
int8_t bmi323_port_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);

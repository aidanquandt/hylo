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
#include "platform_spi.h"

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define BMI323_CS_PIN  PLATFORM_SPI_CS_BMI323

/** Return codes from port layer functions */
#define BMI323_SUCCESS  (0)   // Operation successful
#define BMI323_ERROR    (-1)  // Operation failed

/*---------------------------------------------------------------------------
 * Public Types
 *---------------------------------------------------------------------------*/

/** Sensor data structure for accelerometer and gyroscope */
typedef struct {
    float x;
    float y;
    float z;
} bmi323_sensor_data_t;

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
 * @brief Check if BMI323 device ID is valid
 * @param dev Pointer to device structure
 * @return BMI323_SUCCESS on success, BMI323_ERROR on failure
 */
int bmi323_port_check_device_id(struct bmi3_dev *dev);

/**
 * @brief Read BMI323 chip ID register
 * @param dev Pointer to device structure
 * @return 8-bit chip ID value (0x43 for BMI323, 0x44 for BMI330), or 0 on error
 */
uint8_t bmi323_port_read_chip_id(struct bmi3_dev *dev);

/**
 * @brief Read BMI323 IC temperature
 * @param dev Pointer to device structure
 * @return Temperature in degrees Celsius
 */
float bmi323_port_read_temperature(struct bmi3_dev *dev);

/**
 * @brief Read accelerometer data
 * @param dev Pointer to device structure
 * @param accel Pointer to structure to store accelerometer data (in g)
 * @return BMI323_SUCCESS on success, BMI323_ERROR on failure
 */
int bmi323_port_read_accel(struct bmi3_dev *dev, bmi323_sensor_data_t *accel);

/**
 * @brief Read gyroscope data
 * @param dev Pointer to device structure
 * @param gyro Pointer to structure to store gyroscope data (in deg/s)
 * @return BMI323_SUCCESS on success, BMI323_ERROR on failure
 */
int bmi323_port_read_gyro(struct bmi3_dev *dev, bmi323_sensor_data_t *gyro);

/**
 * @brief Read both accelerometer and gyroscope data in single operation (optimized)
 * @param dev Pointer to device structure
 * @param accel Pointer to structure to store accelerometer data (in g), or NULL to skip
 * @param gyro Pointer to structure to store gyroscope data (in deg/s), or NULL to skip
 * @return BMI323_SUCCESS on success, BMI323_ERROR on failure
 */
int bmi323_port_read_accel_and_gyro(struct bmi3_dev *dev, bmi323_sensor_data_t *accel, bmi323_sensor_data_t *gyro);

/**
 * @brief Configure accelerometer settings
 * @param dev Pointer to device structure
 * @param range Accelerometer range (2g, 4g, 8g, 16g)
 * @param odr Output data rate in Hz
 * @return BMI323_SUCCESS on success, BMI323_ERROR on failure
 */
int bmi323_port_configure_accel(struct bmi3_dev *dev, uint8_t range, uint16_t odr);

/**
 * @brief Configure gyroscope settings
 * @param dev Pointer to device structure
 * @param range Gyroscope range (125, 250, 500, 1000, 2000 deg/s)
 * @param odr Output data rate in Hz
 * @return BMI323_SUCCESS on success, BMI323_ERROR on failure
 */
int bmi323_port_configure_gyro(struct bmi3_dev *dev, uint16_t range, uint16_t odr);

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


/*---------------------------------------------------------------------------
 * @file    imu_port.h
 * @brief   Port layer for IMU sensor driver - Adapter between driver and platform
 * 
 * @note Thread Safety: This implementation is NOT thread-safe. Callers must
 *       ensure exclusive access to IMU operations through external synchronization.
 * 
 * @note Initialization: Must call imu_port_init() before any other functions.
 *       Hardware SPI peripheral must be initialized before calling imu_port_init().
 *---------------------------------------------------------------------------*/
#pragma once

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "common.h"
#include "platform_spi.h"

/*---------------------------------------------------------------------------
 * Forward Declarations
 *---------------------------------------------------------------------------*/
/** 
 * Opaque IMU device handle - application code cannot access internals.
 * Actual structure is defined in imu_port.c to hide implementation details.
 */
typedef struct imu_dev_s imu_dev_t;

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define IMU_PORT_CS_PIN  PLATFORM_SPI_CS_IMU

/** Return codes from port layer functions */
typedef enum {
    IMU_PORT_SUCCESS          =  0,   ///< Operation completed successfully
    IMU_PORT_ERROR_NULL_PTR   = -1,   ///< NULL pointer passed to function
    IMU_PORT_ERROR_COMM_FAIL  = -2,   ///< SPI communication failure
    IMU_PORT_ERROR_INVALID_ID = -3,   ///< Device ID does not match expected value
    IMU_PORT_ERROR_TIMEOUT    = -4,   ///< Operation timed out
    IMU_PORT_ERROR_INIT_FAIL  = -5,   ///< Device initialization failed
    IMU_PORT_ERROR_CONFIG     = -6,   ///< Invalid configuration parameter
    IMU_PORT_ERROR_UNKNOWN    = -99   ///< Unknown/unspecified error
} imu_port_status_t;

/** Accelerometer range settings */
typedef enum {
    IMU_ACCEL_RANGE_2G  = 0x00,  ///< ±2g
    IMU_ACCEL_RANGE_4G  = 0x01,  ///< ±4g
    IMU_ACCEL_RANGE_8G  = 0x02,  ///< ±8g
    IMU_ACCEL_RANGE_16G = 0x03   ///< ±16g
} imu_accel_range_t;

/** Gyroscope range settings */
typedef enum {
    IMU_GYRO_RANGE_125DPS  = 0x00,  ///< ±125 deg/s
    IMU_GYRO_RANGE_250DPS  = 0x01,  ///< ±250 deg/s
    IMU_GYRO_RANGE_500DPS  = 0x02,  ///< ±500 deg/s
    IMU_GYRO_RANGE_1000DPS = 0x03,  ///< ±1000 deg/s
    IMU_GYRO_RANGE_2000DPS = 0x04   ///< ±2000 deg/s
} imu_gyro_range_t;

/** Output Data Rate (ODR) settings */
typedef enum {
    IMU_ODR_0_78HZ   = 0x01,  ///< 0.78125 Hz
    IMU_ODR_1_5625HZ = 0x02,  ///< 1.5625 Hz
    IMU_ODR_3_125HZ  = 0x03,  ///< 3.125 Hz
    IMU_ODR_6_25HZ   = 0x04,  ///< 6.25 Hz
    IMU_ODR_12_5HZ   = 0x05,  ///< 12.5 Hz
    IMU_ODR_25HZ     = 0x06,  ///< 25 Hz
    IMU_ODR_50HZ     = 0x07,  ///< 50 Hz
    IMU_ODR_100HZ    = 0x08,  ///< 100 Hz
    IMU_ODR_200HZ    = 0x09,  ///< 200 Hz
    IMU_ODR_400HZ    = 0x0A,  ///< 400 Hz
    IMU_ODR_800HZ    = 0x0B,  ///< 800 Hz
    IMU_ODR_1600HZ   = 0x0C,  ///< 1600 Hz
    IMU_ODR_3200HZ   = 0x0D,  ///< 3200 Hz
    IMU_ODR_6400HZ   = 0x0E   ///< 6400 Hz
} imu_odr_t;

/*---------------------------------------------------------------------------
 * Public Types
 *---------------------------------------------------------------------------*/

/** Sensor data structure for accelerometer and gyroscope */
typedef struct {
    float x;
    float y;
    float z;
} imu_sensor_data_t;

/*---------------------------------------------------------------------------
 * Public Function Prototypes
 *---------------------------------------------------------------------------*/

/**
 * @brief Initialize the IMU port layer
 * 
 * Initializes the static IMU device structure with SPI callbacks and interface settings.
 * Must be called before any other IMU port functions.
 * 
 * @pre SPI peripheral hardware must be initialized
 * 
 * @return Pointer to IMU device handle on success, NULL on failure
 * 
 * @note This function does NOT probe or configure the IMU hardware.
 *       Call imu_port_probe_and_init() to complete initialization.
 */
imu_dev_t* imu_port_init(void);

/**
 * @brief Probe and initialize the IMU device
 * 
 * Performs SPI mode selection sequence, probes the device, and runs initialization.
 * This includes device reset and loading of default configuration.
 * 
 * @param[in] dev Pointer to IMU device handle from imu_port_init()
 * 
 * @return IMU_PORT_SUCCESS on success
 * @return IMU_PORT_ERROR_NULL_PTR if dev is NULL
 * @return IMU_PORT_ERROR_COMM_FAIL if SPI communication fails
 * @return IMU_PORT_ERROR_INIT_FAIL if device initialization fails
 * 
 * @note This function includes a 2ms delay for mode switching
 */
imu_port_status_t imu_port_probe_and_init(imu_dev_t *dev);

/**
 * @brief Check if IMU device ID is valid
 * 
 * Reads and validates the chip ID register against expected values.
 * 
 * @param[in] dev Pointer to IMU device handle
 * 
 * @return IMU_PORT_SUCCESS if chip ID is valid
 * @return IMU_PORT_ERROR_NULL_PTR if dev is NULL
 * @return IMU_PORT_ERROR_INVALID_ID if chip ID doesn't match expected values
 */
imu_port_status_t imu_port_check_device_id(imu_dev_t *dev);

/**
 * @brief Read IMU chip ID register
 * 
 * @param[in] dev Pointer to IMU device handle
 * 
 * @return 8-bit chip ID value (0x43 for BMI323, 0x44 for BMI330)
 * @return 0 on error or if dev is NULL
 */
uint8_t imu_port_read_chip_id(imu_dev_t *dev);

/**
 * @brief Read IMU IC temperature
 * 
 * @param[in] dev Pointer to IMU device handle
 * 
 * @return Temperature in degrees Celsius
 * @return 0.0f on error or if dev is NULL
 */
float imu_port_read_temperature(imu_dev_t *dev);

/**
 * @brief Read accelerometer data
 * 
 * @param[in]  dev   Pointer to IMU device handle
 * @param[out] accel Pointer to structure to store accelerometer data (in g)
 * 
 * @return IMU_PORT_SUCCESS on success
 * @return IMU_PORT_ERROR_NULL_PTR if dev or accel is NULL
 * @return IMU_PORT_ERROR_COMM_FAIL if SPI read fails
 */
imu_port_status_t imu_port_read_accel(imu_dev_t *dev, imu_sensor_data_t *accel);

/**
 * @brief Read gyroscope data
 * 
 * @param[in]  dev  Pointer to IMU device handle
 * @param[out] gyro Pointer to structure to store gyroscope data (in deg/s)
 * 
 * @return IMU_PORT_SUCCESS on success
 * @return IMU_PORT_ERROR_NULL_PTR if dev or gyro is NULL
 * @return IMU_PORT_ERROR_COMM_FAIL if SPI read fails
 */
imu_port_status_t imu_port_read_gyro(imu_dev_t *dev, imu_sensor_data_t *gyro);

/**
 * @brief Read both accelerometer and gyroscope data in single operation
 * 
 * Optimized function to read both sensors with a single driver call.
 * Either accel or gyro can be NULL to skip reading that sensor.
 * 
 * @param[in]  dev   Pointer to IMU device handle
 * @param[out] accel Pointer to accelerometer data (in g), or NULL to skip
 * @param[out] gyro  Pointer to gyroscope data (in deg/s), or NULL to skip
 * 
 * @return IMU_PORT_SUCCESS on success
 * @return IMU_PORT_ERROR_NULL_PTR if dev is NULL
 * @return IMU_PORT_ERROR_COMM_FAIL if SPI read fails
 */
imu_port_status_t imu_port_read_accel_and_gyro(imu_dev_t *dev, imu_sensor_data_t *accel, imu_sensor_data_t *gyro);

/**
 * @brief Configure accelerometer settings
 * 
 * Sets accelerometer measurement range and output data rate.
 * Configuration is validated before being applied to hardware.
 * 
 * @param[in] dev   Pointer to IMU device handle
 * @param[in] range Accelerometer range (see imu_accel_range_t)
 * @param[in] odr   Output data rate (see imu_odr_t)
 * 
 * @return IMU_PORT_SUCCESS on success
 * @return IMU_PORT_ERROR_NULL_PTR if dev is NULL
 * @return IMU_PORT_ERROR_CONFIG if range or odr is invalid
 * @return IMU_PORT_ERROR_COMM_FAIL if configuration fails
 */
imu_port_status_t imu_port_configure_accel(imu_dev_t *dev, imu_accel_range_t range, imu_odr_t odr);

/**
 * @brief Configure gyroscope settings
 * 
 * Sets gyroscope measurement range and output data rate.
 * Configuration is validated before being applied to hardware.
 * 
 * @param[in] dev   Pointer to IMU device handle
 * @param[in] range Gyroscope range (see imu_gyro_range_t)
 * @param[in] odr   Output data rate (see imu_odr_t)
 * 
 * @return IMU_PORT_SUCCESS on success
 * @return IMU_PORT_ERROR_NULL_PTR if dev is NULL
 * @return IMU_PORT_ERROR_CONFIG if range or odr is invalid
 * @return IMU_PORT_ERROR_COMM_FAIL if configuration fails
 */
imu_port_status_t imu_port_configure_gyro(imu_dev_t *dev, imu_gyro_range_t range, imu_odr_t odr);

/**
 * @brief Perform a soft reset on the IMU device
 * 
 * Performs a software reset of the IMU chip. This resets all registers
 * to their default values and requires re-initialization afterward.
 * 
 * @param[in] dev Pointer to IMU device handle
 * 
 * @pre dev must not be NULL
 * @pre Device must have been initialized (imu_port_probe_and_init() must have been called)
 * 
 * @return IMU_PORT_SUCCESS on success
 * @return IMU_PORT_ERROR_NULL_PTR if dev is NULL
 * @return IMU_PORT_ERROR_COMM_FAIL if reset command fails
 */
imu_port_status_t imu_port_soft_reset(imu_dev_t *dev);

/**
 * @brief Delay function for IMU driver
 * @param period_us Delay period in microseconds
 * @param intf_ptr Interface pointer (unused)
 */
void imu_port_delay_us(uint32_t period_us, void *intf_ptr);

/**
 * @brief SPI read function for IMU driver
 * @param reg_addr Register address to read from
 * @param reg_data Buffer to store read data
 * @param len Number of bytes to read
 * @param intf_ptr Interface pointer (unused)
 * @return BMI3_OK on success, error code on failure
 */
int8_t imu_port_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);

/**
 * @brief SPI write function for IMU driver
 * @param reg_addr Register address to write to
 * @param reg_data Data to write
 * @param len Number of bytes to write
 * @param intf_ptr Interface pointer (unused)
 * @return BMI3_OK on success, error code on failure
 */
int8_t imu_port_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);


#pragma once

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "common.h"
#include "feature_config.h"
#include "spi_driver.h"

/*---------------------------------------------------------------------------
 * Forward Declarations
 *---------------------------------------------------------------------------*/
typedef struct imu_dev_s imu_dev_t;

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define IMU_DRIVER_CS_PIN SPI_DRIVER_INTERFACE_IMU

/*---------------------------------------------------------------------------
 * Typedefs
 *---------------------------------------------------------------------------*/
typedef enum
{
#if FEATURE_IMUS_POPULATED
    IMU_DEVICE_0 = 0,
    IMU_DEVICE_1,
    IMU_DEVICE_2,
    IMU_DEVICE_3,
#endif
    IMU_NUM_DEVICES
} imu_device_e;

#define IMU_MAX_DEVICES (4U)

typedef enum
{
    IMU_DRIVER_SUCCESS          = 0,
    IMU_DRIVER_ERROR_NULL_PTR   = -1,
    IMU_DRIVER_ERROR_COMM_FAIL  = -2,
    IMU_DRIVER_ERROR_INVALID_ID = -3,
    IMU_DRIVER_ERROR_TIMEOUT    = -4,
    IMU_DRIVER_ERROR_INIT_FAIL  = -5,
    IMU_DRIVER_ERROR_CONFIG     = -6,
    IMU_DRIVER_ERROR_UNKNOWN    = -99
} imu_driver_status_t;

typedef enum
{
    IMU_ACCEL_RANGE_2G  = 0x00,
    IMU_ACCEL_RANGE_4G  = 0x01,
    IMU_ACCEL_RANGE_8G  = 0x02,
    IMU_ACCEL_RANGE_16G = 0x03
} imu_accel_range_t;

typedef enum
{
    IMU_GYRO_RANGE_125DPS  = 0x00,
    IMU_GYRO_RANGE_250DPS  = 0x01,
    IMU_GYRO_RANGE_500DPS  = 0x02,
    IMU_GYRO_RANGE_1000DPS = 0x03,
    IMU_GYRO_RANGE_2000DPS = 0x04
} imu_gyro_range_t;

typedef enum
{
    IMU_ODR_0_78HZ   = 0x01,
    IMU_ODR_1_5625HZ = 0x02,
    IMU_ODR_3_125HZ  = 0x03,
    IMU_ODR_6_25HZ   = 0x04,
    IMU_ODR_12_5HZ   = 0x05,
    IMU_ODR_25HZ     = 0x06,
    IMU_ODR_50HZ     = 0x07,
    IMU_ODR_100HZ    = 0x08,
    IMU_ODR_200HZ    = 0x09,
    IMU_ODR_400HZ    = 0x0A,
    IMU_ODR_800HZ    = 0x0B,
    IMU_ODR_1600HZ   = 0x0C,
    IMU_ODR_3200HZ   = 0x0D,
    IMU_ODR_6400HZ   = 0x0E
} imu_odr_t;

/*---------------------------------------------------------------------------
 * Public Function Prototypes
 *---------------------------------------------------------------------------*/
imu_dev_t* imu_driver_init(imu_device_e device);
imu_driver_status_t imu_driver_probe_and_init(imu_dev_t* dev);
imu_driver_status_t imu_driver_check_device_id(imu_dev_t* dev);
uint8_t imu_driver_read_chip_id(imu_dev_t* dev);
float imu_driver_read_temperature(imu_dev_t* dev);
imu_driver_status_t imu_driver_read_accel(imu_dev_t* dev, vec3_t* accel);
imu_driver_status_t imu_driver_read_gyro(imu_dev_t* dev, vec3_t* gyro);
imu_driver_status_t imu_driver_read_accel_and_gyro(imu_dev_t* dev, vec3_t* accel, vec3_t* gyro);
imu_driver_status_t imu_driver_configure_accel(imu_dev_t* dev, imu_accel_range_t range, imu_odr_t odr);
imu_driver_status_t imu_driver_configure_gyro(imu_dev_t* dev, imu_gyro_range_t range, imu_odr_t odr);
imu_driver_status_t imu_driver_soft_reset(imu_dev_t* dev);

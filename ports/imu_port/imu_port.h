#pragma once

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "common.h"
#include "feature_config.h"
#include "platform_spi.h"

/*---------------------------------------------------------------------------
 * Forward Declarations
 *---------------------------------------------------------------------------*/
typedef struct imu_dev_s imu_dev_t;

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define IMU_PORT_CS_PIN PLATFORM_SPI_CS_IMU

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
    IMU_NUM_DEVICES /* last: 0 or 4; use for iteration and bounds */
} imu_device_e;

/* Max devices for static array sizing (avoids zero-length arrays when IMU_NUM_DEVICES is 0). */
#define IMU_MAX_DEVICES (4U)

typedef enum
{
    IMU_PORT_SUCCESS          = 0,
    IMU_PORT_ERROR_NULL_PTR   = -1,
    IMU_PORT_ERROR_COMM_FAIL  = -2,
    IMU_PORT_ERROR_INVALID_ID = -3,
    IMU_PORT_ERROR_TIMEOUT    = -4,
    IMU_PORT_ERROR_INIT_FAIL  = -5,
    IMU_PORT_ERROR_CONFIG     = -6,
    IMU_PORT_ERROR_UNKNOWN    = -99
} imu_port_status_t;

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
 * Public Types
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
 * Public Function Prototypes
 *---------------------------------------------------------------------------*/
imu_dev_t* imu_port_init(imu_device_e device);
imu_port_status_t imu_port_probe_and_init(imu_dev_t* dev);
imu_port_status_t imu_port_check_device_id(imu_dev_t* dev);
uint8_t imu_port_read_chip_id(imu_dev_t* dev);
float imu_port_read_temperature(imu_dev_t* dev);
imu_port_status_t imu_port_read_accel(imu_dev_t* dev, vec3_t* accel);
imu_port_status_t imu_port_read_gyro(imu_dev_t* dev, vec3_t* gyro);
imu_port_status_t imu_port_read_accel_and_gyro(imu_dev_t* dev, vec3_t* accel, vec3_t* gyro);
imu_port_status_t imu_port_configure_accel(imu_dev_t* dev, imu_accel_range_t range, imu_odr_t odr);
imu_port_status_t imu_port_configure_gyro(imu_dev_t* dev, imu_gyro_range_t range, imu_odr_t odr);
imu_port_status_t imu_port_soft_reset(imu_dev_t* dev);
void imu_port_delay_us(uint32_t period_us, void* intf_ptr);
int8_t imu_port_spi_read(uint8_t reg_addr, uint8_t* reg_data, uint32_t len, void* intf_ptr);
int8_t imu_port_spi_write(uint8_t reg_addr, const uint8_t* reg_data, uint32_t len, void* intf_ptr);

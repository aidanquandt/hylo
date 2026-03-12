/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "imu_port.h"
#include "FreeRTOS.h"
#include "bmi323.h"
#include "gpio.h"
#include "platform_os.h"
#include "platform_spi.h"
#include "platform_timer.h"
#include "task.h"
#include <string.h>

/*---------------------------------------------------------------------------
 * Typedefs
 *---------------------------------------------------------------------------*/
struct imu_dev_s
{
    struct bmi3_dev bmi_dev; ///< Wrapped vendor driver device structure (BMI323-specific)
};

/*---------------------------------------------------------------------------
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
STATIC int8_t imu_spi_read(uint8_t reg_addr, uint8_t* reg_data, uint32_t len, void* intf_ptr);
STATIC int8_t imu_spi_write(uint8_t reg_addr, const uint8_t* reg_data, uint32_t len,
                            void* intf_ptr);
STATIC void imu_delay_us(uint32_t period_us, void* intf_ptr);
STATIC bool validate_accel_range(imu_accel_range_t range);
STATIC bool validate_gyro_range(imu_gyro_range_t range);
STATIC bool validate_odr(imu_odr_t odr);

/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/
STATIC struct imu_dev_s imu_devices[IMU_MAX_DEVICES];

STATIC const platform_spi_cs_E imu_cs_pins[IMU_MAX_DEVICES] = {
    PLATFORM_SPI_CS_IMU_0,
    PLATFORM_SPI_CS_IMU_1,
    PLATFORM_SPI_CS_IMU_2,
    PLATFORM_SPI_CS_IMU_3,
};

/*---------------------------------------------------------------------------
 * Public Function Implementations
 *---------------------------------------------------------------------------*/

imu_dev_t* imu_port_init(imu_device_e device)
{
    if (IMU_NUM_DEVICES == 0U || (uint8_t)device >= (uint8_t)IMU_MAX_DEVICES)
    {
        return NULL;
    }

    uint8_t idx = (uint8_t)device;
    struct imu_dev_s* dev = &imu_devices[idx];
    dev->bmi_dev.intf           = BMI3_SPI_INTF;
    dev->bmi_dev.read           = imu_spi_read;
    dev->bmi_dev.write          = imu_spi_write;
    dev->bmi_dev.delay_us       = imu_delay_us;
    dev->bmi_dev.intf_ptr       = (void*)&imu_cs_pins[idx];
    dev->bmi_dev.read_write_len = 32;

    return dev;
}

imu_port_status_t imu_port_probe_and_init(imu_dev_t* dev)
{
    if (dev == NULL)
    {
        return IMU_PORT_ERROR_NULL_PTR;
    }

    platform_spi_cs_E cs = *(const platform_spi_cs_E*)dev->bmi_dev.intf_ptr;

    platform_spi_cs_low(cs);
    platform_os_delay_us_blocking(100);
    platform_spi_cs_high(cs);
    platform_os_delay_us_blocking(500);

    uint8_t dummy_rx[3] = {0};
    platform_spi_cs_low(cs);
    uint8_t dummy_cmd = 0x80;
    platform_spi_transmit(&dummy_cmd, 1);
    platform_spi_receive(dummy_rx, 3);
    platform_spi_cs_high(cs);
    vTaskDelay(pdMS_TO_TICKS(2));

    int8_t rslt = bmi323_init(&dev->bmi_dev);

    if (rslt != BMI3_OK)
    {
        return IMU_PORT_ERROR_INIT_FAIL;
    }

    return IMU_PORT_SUCCESS;
}

imu_port_status_t imu_port_check_device_id(imu_dev_t* dev)
{
    if (dev == NULL)
    {
        return IMU_PORT_ERROR_NULL_PTR;
    }

    uint8_t chip_id = imu_port_read_chip_id(dev);

    if (chip_id != 0x43 && chip_id != 0x44)
    {
        return IMU_PORT_ERROR_INVALID_ID;
    }

    return IMU_PORT_SUCCESS;
}

uint8_t imu_port_read_chip_id(imu_dev_t* dev)
{
    if (dev == NULL)
    {
        return 0;
    }

    uint8_t chip_id_buf[2] = {0};
    int8_t rslt            = bmi3_get_regs(BMI3_REG_CHIP_ID, chip_id_buf, 2, &dev->bmi_dev);

    if (rslt != BMI3_OK)
    {
        return 0;
    }

    return chip_id_buf[0];
}

float imu_port_read_temperature(imu_dev_t* dev)
{
    if (dev == NULL)
    {
        return 0.0f;
    }

    struct bmi3_sensor_data sensor_data = {0};
    sensor_data.type                    = BMI323_TEMP;

    int8_t rslt = bmi323_get_sensor_data(&sensor_data, 1, &dev->bmi_dev);

    if (rslt != BMI3_OK)
    {
        return 0.0f;
    }

    return ((int16_t)sensor_data.sens_data.temp.temp_data / 512.0f) + 23.0f;
}

imu_port_status_t imu_port_read_accel(imu_dev_t* dev, vec3_t* accel)
{
    if (dev == NULL || accel == NULL)
    {
        return IMU_PORT_ERROR_NULL_PTR;
    }

    struct bmi3_sensor_data sensor_data = {0};
    sensor_data.type                    = BMI323_ACCEL;

    int8_t rslt = bmi323_get_sensor_data(&sensor_data, 1, &dev->bmi_dev);
    if (rslt != BMI3_OK)
    {
        return IMU_PORT_ERROR_COMM_FAIL;
    }

    accel->x = (sensor_data.sens_data.acc.x / 16384.0f) * GRAVITY_MAGNITUDE;
    accel->y = (sensor_data.sens_data.acc.y / 16384.0f) * GRAVITY_MAGNITUDE;
    accel->z = (sensor_data.sens_data.acc.z / 16384.0f) * GRAVITY_MAGNITUDE;

    return IMU_PORT_SUCCESS;
}

imu_port_status_t imu_port_read_gyro(imu_dev_t* dev, vec3_t* gyro)
{
    if (dev == NULL || gyro == NULL)
    {
        return IMU_PORT_ERROR_NULL_PTR;
    }

    struct bmi3_sensor_data sensor_data = {0};
    sensor_data.type                    = BMI323_GYRO;

    int8_t rslt = bmi323_get_sensor_data(&sensor_data, 1, &dev->bmi_dev);
    if (rslt != BMI3_OK)
    {
        return IMU_PORT_ERROR_COMM_FAIL;
    }

    gyro->x = (sensor_data.sens_data.gyr.x / 16.4f) * DEG_TO_RAD;
    gyro->y = (sensor_data.sens_data.gyr.y / 16.4f) * DEG_TO_RAD;
    gyro->z = (sensor_data.sens_data.gyr.z / 16.4f) * DEG_TO_RAD;

    return IMU_PORT_SUCCESS;
}

imu_port_status_t imu_port_read_accel_and_gyro(imu_dev_t* dev, vec3_t* accel, vec3_t* gyro)
{
    if (dev == NULL)
    {
        return IMU_PORT_ERROR_NULL_PTR;
    }

    struct bmi3_sensor_data sensor_data[2] = {0};
    sensor_data[0].type                    = BMI323_ACCEL;
    sensor_data[1].type                    = BMI323_GYRO;

    int8_t rslt = bmi323_get_sensor_data(sensor_data, 2, &dev->bmi_dev);
    if (rslt != BMI3_OK)
    {
        return IMU_PORT_ERROR_COMM_FAIL;
    }

    if (accel != NULL)
    {
        accel->x = (sensor_data[0].sens_data.acc.x / 16384.0f) * GRAVITY_MAGNITUDE;
        accel->y = (sensor_data[0].sens_data.acc.y / 16384.0f) * GRAVITY_MAGNITUDE;
        accel->z = (sensor_data[0].sens_data.acc.z / 16384.0f) * GRAVITY_MAGNITUDE;
    }

    if (gyro != NULL)
    {
        gyro->x = (sensor_data[1].sens_data.gyr.x / 16.4f) * DEG_TO_RAD;
        gyro->y = (sensor_data[1].sens_data.gyr.y / 16.4f) * DEG_TO_RAD;
        gyro->z = (sensor_data[1].sens_data.gyr.z / 16.4f) * DEG_TO_RAD;
    }

    return IMU_PORT_SUCCESS;
}

imu_port_status_t imu_port_configure_accel(imu_dev_t* dev, imu_accel_range_t range, imu_odr_t odr)
{
    if (dev == NULL)
    {
        return IMU_PORT_ERROR_NULL_PTR;
    }

    if (!validate_accel_range(range) || !validate_odr(odr))
    {
        return IMU_PORT_ERROR_CONFIG;
    }

    struct bmi3_sens_config config = {0};
    config.type                    = BMI323_ACCEL;

    int8_t rslt = bmi323_get_sensor_config(&config, 1, &dev->bmi_dev);
    if (rslt != BMI3_OK)
    {
        return IMU_PORT_ERROR_COMM_FAIL;
    }

    config.cfg.acc.range    = (uint8_t)range;
    config.cfg.acc.odr      = (uint16_t)odr;
    config.cfg.acc.acc_mode = BMI3_ACC_MODE_NORMAL; // Enable accelerometer
    config.cfg.acc.bwp      = BMI3_ACC_BW_ODR_QUARTER; // 50Hz cutoff @ 200Hz ODR
    config.cfg.acc.avg_num  = BMI3_ACC_AVG4; // Average 4 samples for noise reduction

    rslt = bmi323_set_sensor_config(&config, 1, &dev->bmi_dev);
    if (rslt != BMI3_OK)
    {
        return IMU_PORT_ERROR_COMM_FAIL;
    }

    return IMU_PORT_SUCCESS;
}

imu_port_status_t imu_port_configure_gyro(imu_dev_t* dev, imu_gyro_range_t range, imu_odr_t odr)
{
    if (dev == NULL)
    {
        return IMU_PORT_ERROR_NULL_PTR;
    }

    if (!validate_gyro_range(range) || !validate_odr(odr))
    {
        return IMU_PORT_ERROR_CONFIG;
    }

    struct bmi3_sens_config config = {0};
    config.type                    = BMI323_GYRO;

    int8_t rslt = bmi323_get_sensor_config(&config, 1, &dev->bmi_dev);
    if (rslt != BMI3_OK)
    {
        return IMU_PORT_ERROR_COMM_FAIL;
    }

    config.cfg.gyr.range    = (uint16_t)range;
    config.cfg.gyr.odr      = (uint16_t)odr;
    config.cfg.gyr.gyr_mode = BMI3_GYR_MODE_NORMAL; // Enable gyroscope
    config.cfg.gyr.bwp      = BMI3_GYR_BW_ODR_QUARTER; // 50Hz cutoff @ 200Hz ODR
    config.cfg.gyr.avg_num  = BMI3_GYR_AVG4; // Average 4 samples for noise reduction

    rslt = bmi323_set_sensor_config(&config, 1, &dev->bmi_dev);
    if (rslt != BMI3_OK)
    {
        return IMU_PORT_ERROR_COMM_FAIL;
    }

    return IMU_PORT_SUCCESS;
}

imu_port_status_t imu_port_soft_reset(imu_dev_t* dev)
{
    if (dev == NULL)
    {
        return IMU_PORT_ERROR_NULL_PTR;
    }

    int8_t rslt = bmi3_soft_reset(&dev->bmi_dev);
    if (rslt != BMI3_OK)
    {
        return IMU_PORT_ERROR_COMM_FAIL;
    }

    vTaskDelay(pdMS_TO_TICKS(5));

    return IMU_PORT_SUCCESS;
}

imu_port_status_t imu_port_calibrate_gyro(imu_dev_t* dev)
{
    if (dev == NULL)
    {
        return IMU_PORT_ERROR_NULL_PTR;
    }

    struct bmi3_self_calib_rslt sc_rslt = {0};

    /* Calibrate gyro offset — device must be stationary.
     * Blocks ~430ms internally (10 polls x 43ms). */
    int8_t rslt = bmi323_perform_gyro_sc(BMI3_SC_OFFSET_EN, BMI3_SC_APPLY_CORR_EN, &sc_rslt,
                                         &dev->bmi_dev);

    if (rslt != BMI3_OK)
    {
        return IMU_PORT_ERROR_COMM_FAIL;
    }

    if (sc_rslt.gyro_sc_rslt != BMI3_TRUE)
    {
        return IMU_PORT_ERROR_CONFIG;
    }

    return IMU_PORT_SUCCESS;
}

imu_port_status_t imu_port_calibrate_accel(imu_dev_t* dev)
{
    if (dev == NULL)
    {
        return IMU_PORT_ERROR_NULL_PTR;
    }

    /* Board mounting: chip X maps to body -Z (see imu_transform_accel).
     * When device is flat (body +Z up), gravity is -1g on chip X axis. */
    struct bmi3_accel_foc_g_value accel_g_value = {
        .x    = 1,
        .y    = 0,
        .z    = 0,
        .sign = 1
    };

    int8_t rslt = bmi323_perform_accel_foc(&accel_g_value, &dev->bmi_dev);

    if (rslt != BMI3_OK)
    {
        return IMU_PORT_ERROR_COMM_FAIL;
    }

    return IMU_PORT_SUCCESS;
}

/*---------------------------------------------------------------------------
 * Private Function Implementations
 *---------------------------------------------------------------------------*/

STATIC bool validate_accel_range(imu_accel_range_t range)
{
    switch (range)
    {
        case IMU_ACCEL_RANGE_2G:
        case IMU_ACCEL_RANGE_4G:
        case IMU_ACCEL_RANGE_8G:
        case IMU_ACCEL_RANGE_16G:
            return true;
        default:
            return false;
    }
}

STATIC bool validate_gyro_range(imu_gyro_range_t range)
{
    switch (range)
    {
        case IMU_GYRO_RANGE_125DPS:
        case IMU_GYRO_RANGE_250DPS:
        case IMU_GYRO_RANGE_500DPS:
        case IMU_GYRO_RANGE_1000DPS:
        case IMU_GYRO_RANGE_2000DPS:
            return true;
        default:
            return false;
    }
}

STATIC bool validate_odr(imu_odr_t odr)
{
    switch (odr)
    {
        case IMU_ODR_0_78HZ:
        case IMU_ODR_1_5625HZ:
        case IMU_ODR_3_125HZ:
        case IMU_ODR_6_25HZ:
        case IMU_ODR_12_5HZ:
        case IMU_ODR_25HZ:
        case IMU_ODR_50HZ:
        case IMU_ODR_100HZ:
        case IMU_ODR_200HZ:
        case IMU_ODR_400HZ:
        case IMU_ODR_800HZ:
        case IMU_ODR_1600HZ:
        case IMU_ODR_3200HZ:
        case IMU_ODR_6400HZ:
            return true;
        default:
            return false;
    }
}

/*---------------------------------------------------------------------------
 * Private Function Implementations
 *---------------------------------------------------------------------------*/

STATIC int8_t imu_spi_read(uint8_t reg_addr, uint8_t* reg_data, uint32_t len, void* intf_ptr)
{
    if (intf_ptr == NULL || reg_data == NULL)
    {
        return BMI3_E_NULL_PTR;
    }

    platform_spi_cs_E cs = *(const platform_spi_cs_E*)intf_ptr;
    platform_spi_cs_low(cs);

    uint8_t tx_buf[len + 1];
    uint8_t rx_buf[len + 1];

    memset(tx_buf, 0, len + 1);
    tx_buf[0] = reg_addr;

    if (platform_spi_transfer(tx_buf, rx_buf, len + 1) != PLATFORM_SPI_SUCCESS)
    {
        platform_spi_cs_high(cs);
        return BMI3_E_COM_FAIL;
    }

    for (uint32_t i = 0; i < len; i++)
    {
        reg_data[i] = rx_buf[i + 1];
    }

    platform_spi_cs_high(cs);

    return BMI3_OK;
}

STATIC int8_t imu_spi_write(uint8_t reg_addr, const uint8_t* reg_data, uint32_t len, void* intf_ptr)
{
    if (intf_ptr == NULL || reg_data == NULL)
    {
        return BMI3_E_NULL_PTR;
    }

    platform_spi_cs_E cs = *(const platform_spi_cs_E*)intf_ptr;
    platform_spi_cs_low(cs);

    uint8_t addr_byte = reg_addr & 0x7F;
    if (platform_spi_transmit(&addr_byte, 1) != PLATFORM_SPI_SUCCESS)
    {
        platform_spi_cs_high(cs);
        return BMI3_E_COM_FAIL;
    }

    if (platform_spi_transmit(reg_data, len) != PLATFORM_SPI_SUCCESS)
    {
        platform_spi_cs_high(cs);
        return BMI3_E_COM_FAIL;
    }

    platform_spi_cs_high(cs);

    return BMI3_OK;
}

STATIC void imu_delay_us(uint32_t period_us, void* intf_ptr)
{
    (void)intf_ptr;
    platform_os_delay_us_blocking(period_us);
}

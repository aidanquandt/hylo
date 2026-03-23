/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "imu_driver.h"
#include "FreeRTOS.h"
#include "bmi323.h"
#include "gpio.h"
#include "os_driver.h"
#include "spi_driver.h"
#include "timer_driver.h"
#include "task.h"
#include <string.h>

/*---------------------------------------------------------------------------
 * Typedefs
 *---------------------------------------------------------------------------*/
struct imu_dev_s
{
    struct bmi3_dev bmi_dev;
};

/*---------------------------------------------------------------------------
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
STATIC int8_t imu_spi_read(uint8_t reg_addr, uint8_t* reg_data, uint32_t len, void* intf_ptr);
STATIC int8_t imu_spi_write(uint8_t reg_addr, const uint8_t* reg_data, uint32_t len, void* intf_ptr);
STATIC void imu_delay_us(uint32_t period_us, void* intf_ptr);
STATIC bool validate_accel_range(imu_accel_range_t range);
STATIC bool validate_gyro_range(imu_gyro_range_t range);
STATIC bool validate_odr(imu_odr_t odr);

/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/
STATIC struct imu_dev_s imu_devices[IMU_MAX_DEVICES];

STATIC const spi_driver_cs_E imu_cs_pins[IMU_MAX_DEVICES] = {
    // SPI_DRIVER_CS_IMU_0,
    // SPI_DRIVER_CS_IMU_1,
    // SPI_DRIVER_CS_IMU_2,
    // SPI_DRIVER_CS_IMU_3,
};

/*---------------------------------------------------------------------------
 * Public Function Implementations
 *---------------------------------------------------------------------------*/

imu_dev_t* imu_driver_init(imu_device_e device)
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

imu_driver_status_t imu_driver_probe_and_init(imu_dev_t* dev)
{
    if (dev == NULL)
    {
        return IMU_DRIVER_ERROR_NULL_PTR;
    }

    spi_driver_cs_E cs = *(const spi_driver_cs_E*)dev->bmi_dev.intf_ptr;

    spi_driver_cs_low(cs);
    os_driver_delay_us_blocking(100);
    spi_driver_cs_high(cs);
    os_driver_delay_us_blocking(500);

    uint8_t dummy_rx[3] = {0};
    spi_driver_cs_low(cs);
    uint8_t dummy_cmd = 0x80;
    spi_driver_transmit(&dummy_cmd, 1);
    spi_driver_receive(dummy_rx, 3);
    spi_driver_cs_high(cs);
    vTaskDelay(pdMS_TO_TICKS(2));

    int8_t rslt = bmi323_init(&dev->bmi_dev);

    if (rslt != BMI3_OK)
    {
        return IMU_DRIVER_ERROR_INIT_FAIL;
    }

    return IMU_DRIVER_SUCCESS;
}

imu_driver_status_t imu_driver_check_device_id(imu_dev_t* dev)
{
    if (dev == NULL)
    {
        return IMU_DRIVER_ERROR_NULL_PTR;
    }

    uint8_t chip_id = imu_driver_read_chip_id(dev);

    if (chip_id != 0x43 && chip_id != 0x44)
    {
        return IMU_DRIVER_ERROR_INVALID_ID;
    }

    return IMU_DRIVER_SUCCESS;
}

uint8_t imu_driver_read_chip_id(imu_dev_t* dev)
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

float imu_driver_read_temperature(imu_dev_t* dev)
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

imu_driver_status_t imu_driver_read_accel(imu_dev_t* dev, vec3_t* accel)
{
    if (dev == NULL || accel == NULL)
    {
        return IMU_DRIVER_ERROR_NULL_PTR;
    }

    struct bmi3_sensor_data sensor_data = {0};
    sensor_data.type                    = BMI323_ACCEL;

    int8_t rslt = bmi323_get_sensor_data(&sensor_data, 1, &dev->bmi_dev);
    if (rslt != BMI3_OK)
    {
        return IMU_DRIVER_ERROR_COMM_FAIL;
    }

    accel->x = (sensor_data.sens_data.acc.x / 16384.0f) * GRAVITY_MAGNITUDE;
    accel->y = (sensor_data.sens_data.acc.y / 16384.0f) * GRAVITY_MAGNITUDE;
    accel->z = (sensor_data.sens_data.acc.z / 16384.0f) * GRAVITY_MAGNITUDE;

    return IMU_DRIVER_SUCCESS;
}

imu_driver_status_t imu_driver_read_gyro(imu_dev_t* dev, vec3_t* gyro)
{
    if (dev == NULL || gyro == NULL)
    {
        return IMU_DRIVER_ERROR_NULL_PTR;
    }

    struct bmi3_sensor_data sensor_data = {0};
    sensor_data.type                    = BMI323_GYRO;

    int8_t rslt = bmi323_get_sensor_data(&sensor_data, 1, &dev->bmi_dev);
    if (rslt != BMI3_OK)
    {
        return IMU_DRIVER_ERROR_COMM_FAIL;
    }

    gyro->x = (sensor_data.sens_data.gyr.x / 16.4f) * DEG_TO_RAD;
    gyro->y = (sensor_data.sens_data.gyr.y / 16.4f) * DEG_TO_RAD;
    gyro->z = (sensor_data.sens_data.gyr.z / 16.4f) * DEG_TO_RAD;

    return IMU_DRIVER_SUCCESS;
}

imu_driver_status_t imu_driver_read_accel_and_gyro(imu_dev_t* dev, vec3_t* accel, vec3_t* gyro)
{
    if (dev == NULL)
    {
        return IMU_DRIVER_ERROR_NULL_PTR;
    }

    struct bmi3_sensor_data sensor_data[2] = {0};
    sensor_data[0].type                    = BMI323_ACCEL;
    sensor_data[1].type                    = BMI323_GYRO;

    int8_t rslt = bmi323_get_sensor_data(sensor_data, 2, &dev->bmi_dev);
    if (rslt != BMI3_OK)
    {
        return IMU_DRIVER_ERROR_COMM_FAIL;
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

    return IMU_DRIVER_SUCCESS;
}

imu_driver_status_t imu_driver_configure_accel(imu_dev_t* dev, imu_accel_range_t range, imu_odr_t odr)
{
    if (dev == NULL)
    {
        return IMU_DRIVER_ERROR_NULL_PTR;
    }

    if (!validate_accel_range(range) || !validate_odr(odr))
    {
        return IMU_DRIVER_ERROR_CONFIG;
    }

    struct bmi3_sens_config config = {0};
    config.type                    = BMI323_ACCEL;

    int8_t rslt = bmi323_get_sensor_config(&config, 1, &dev->bmi_dev);
    if (rslt != BMI3_OK)
    {
        return IMU_DRIVER_ERROR_COMM_FAIL;
    }

    config.cfg.acc.range    = (uint8_t)range;
    config.cfg.acc.odr      = (uint16_t)odr;
    config.cfg.acc.acc_mode = BMI3_ACC_MODE_NORMAL;
    config.cfg.acc.bwp      = BMI3_ACC_BW_ODR_QUARTER;
    config.cfg.acc.avg_num  = BMI3_ACC_AVG4;

    rslt = bmi323_set_sensor_config(&config, 1, &dev->bmi_dev);
    if (rslt != BMI3_OK)
    {
        return IMU_DRIVER_ERROR_COMM_FAIL;
    }

    return IMU_DRIVER_SUCCESS;
}

imu_driver_status_t imu_driver_configure_gyro(imu_dev_t* dev, imu_gyro_range_t range, imu_odr_t odr)
{
    if (dev == NULL)
    {
        return IMU_DRIVER_ERROR_NULL_PTR;
    }

    if (!validate_gyro_range(range) || !validate_odr(odr))
    {
        return IMU_DRIVER_ERROR_CONFIG;
    }

    struct bmi3_sens_config config = {0};
    config.type                    = BMI323_GYRO;

    int8_t rslt = bmi323_get_sensor_config(&config, 1, &dev->bmi_dev);
    if (rslt != BMI3_OK)
    {
        return IMU_DRIVER_ERROR_COMM_FAIL;
    }

    config.cfg.gyr.range    = (uint16_t)range;
    config.cfg.gyr.odr      = (uint16_t)odr;
    config.cfg.gyr.gyr_mode = BMI3_GYR_MODE_NORMAL;
    config.cfg.gyr.bwp      = BMI3_GYR_BW_ODR_QUARTER;
    config.cfg.gyr.avg_num  = BMI3_GYR_AVG4;

    rslt = bmi323_set_sensor_config(&config, 1, &dev->bmi_dev);
    if (rslt != BMI3_OK)
    {
        return IMU_DRIVER_ERROR_COMM_FAIL;
    }

    return IMU_DRIVER_SUCCESS;
}

imu_driver_status_t imu_driver_soft_reset(imu_dev_t* dev)
{
    if (dev == NULL)
    {
        return IMU_DRIVER_ERROR_NULL_PTR;
    }

    int8_t rslt = bmi3_soft_reset(&dev->bmi_dev);
    if (rslt != BMI3_OK)
    {
        return IMU_DRIVER_ERROR_COMM_FAIL;
    }

    vTaskDelay(pdMS_TO_TICKS(5));

    return IMU_DRIVER_SUCCESS;
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

STATIC int8_t imu_spi_read(uint8_t reg_addr, uint8_t* reg_data, uint32_t len, void* intf_ptr)
{
    if (intf_ptr == NULL || reg_data == NULL)
    {
        return BMI3_E_NULL_PTR;
    }

    spi_driver_cs_E cs = *(const spi_driver_cs_E*)intf_ptr;
    spi_driver_cs_low(cs);

    uint8_t tx_buf[len + 1];
    uint8_t rx_buf[len + 1];

    memset(tx_buf, 0, len + 1);
    tx_buf[0] = reg_addr;

    if (spi_driver_transfer(tx_buf, rx_buf, len + 1) != SPI_DRIVER_SUCCESS)
    {
        spi_driver_cs_high(cs);
        return BMI3_E_COM_FAIL;
    }

    for (uint32_t i = 0; i < len; i++)
    {
        reg_data[i] = rx_buf[i + 1];
    }

    spi_driver_cs_high(cs);

    return BMI3_OK;
}

STATIC int8_t imu_spi_write(uint8_t reg_addr, const uint8_t* reg_data, uint32_t len, void* intf_ptr)
{
    if (intf_ptr == NULL || reg_data == NULL)
    {
        return BMI3_E_NULL_PTR;
    }

    spi_driver_cs_E cs = *(const spi_driver_cs_E*)intf_ptr;
    spi_driver_cs_low(cs);

    uint8_t addr_byte = reg_addr & 0x7F;
    if (spi_driver_transmit(&addr_byte, 1) != SPI_DRIVER_SUCCESS)
    {
        spi_driver_cs_high(cs);
        return BMI3_E_COM_FAIL;
    }

    if (spi_driver_transmit(reg_data, len) != SPI_DRIVER_SUCCESS)
    {
        spi_driver_cs_high(cs);
        return BMI3_E_COM_FAIL;
    }

    spi_driver_cs_high(cs);

    return BMI3_OK;
}

STATIC void imu_delay_us(uint32_t period_us, void* intf_ptr)
{
    (void)intf_ptr;
    os_driver_delay_us_blocking(period_us);
}

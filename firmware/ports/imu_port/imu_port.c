/*---------------------------------------------------------------------------
 * @file    imu_port.c
 * @brief   Port layer implementation for IMU sensor driver
 *          Translates Bosch driver callbacks to platform API calls
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "imu_port.h"
#include "platform_os.h"
#include "platform_spi.h"
#include "platform_timer.h"
#include "bmi323.h"
#include "gpio.h"
#include <string.h>

/*---------------------------------------------------------------------------
 * Type Definitions
 *---------------------------------------------------------------------------*/

/** Complete definition of opaque IMU device structure */
struct imu_dev_s {
    struct bmi3_dev bmi_dev;  ///< Wrapped BMI323 device structure
};

/*---------------------------------------------------------------------------
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
STATIC int8_t imu_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
STATIC int8_t imu_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);
STATIC void imu_delay_us(uint32_t period_us, void *intf_ptr);
STATIC bool validate_accel_range(imu_accel_range_t range);
STATIC bool validate_gyro_range(imu_gyro_range_t range);
STATIC bool validate_odr(imu_odr_t odr);

/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/

/** Static IMU device instance */
STATIC struct imu_dev_s imu_device = {
    .bmi_dev = {
        .intf = BMI3_SPI_INTF,
        .read = imu_spi_read,
        .write = imu_spi_write,
        .delay_us = imu_delay_us,
        .intf_ptr = NULL,
        .read_write_len = 32,  // Max read/write length
    }
};

/*---------------------------------------------------------------------------
 * Public Function Implementations
 *---------------------------------------------------------------------------*/

imu_dev_t* imu_port_init(void)
{
    return &imu_device;
}

imu_port_status_t imu_port_probe_and_init(imu_dev_t *dev)
{
    if (dev == NULL) {
        return IMU_PORT_ERROR_NULL_PTR;
    }
    
    // CRITICAL: Current IMU (BMI323) requires a RISING EDGE on CSB after power-up to select SPI mode
    // Workaround: Force CS LOW briefly, then HIGH to create rising edge
    // This assumes the device has been powered up long enough (called after 2s delay in test module)
    
    // Force CS LOW
    platform_spi_cs_low(IMU_PORT_CS_PIN);
    platform_os_delay_us_blocking(100);  // Hold LOW briefly
    
    // Set CS HIGH - this creates the rising edge needed for SPI mode selection
    platform_spi_cs_high(IMU_PORT_CS_PIN);
    platform_os_delay_us_blocking(500);  // Wait for mode switch (datasheet: 200µs min, use 500µs for safety)
    
    // Perform a dummy SPI read to fully activate SPI mode
    uint8_t dummy_rx[3] = {0};
    platform_spi_cs_low(IMU_PORT_CS_PIN);
    uint8_t dummy_cmd = 0x80;  // Read chip ID register
    platform_spi_transmit(&dummy_cmd, 1);
    platform_spi_receive(dummy_rx, 3);
    platform_spi_cs_high(IMU_PORT_CS_PIN);
    platform_os_delay_ms(2);  // Longer delay after dummy read before first real transaction
    
    // Now the IMU should be in SPI mode

    // Initialize the IMU sensor (this will do soft reset internally)
    int8_t rslt = bmi323_init(&dev->bmi_dev);

    if (rslt != BMI3_OK) {
        return IMU_PORT_ERROR_INIT_FAIL;
    }
    
    return IMU_PORT_SUCCESS;
}

imu_port_status_t imu_port_check_device_id(imu_dev_t *dev)
{
    if (dev == NULL) {
        return IMU_PORT_ERROR_NULL_PTR;
    }
    
    uint8_t chip_id = imu_port_read_chip_id(dev);
    
    // Verify chip ID matches expected IMU (BMI323 = 0x43, BMI330 = 0x44)
    if (chip_id != 0x43 && chip_id != 0x44) {
        return IMU_PORT_ERROR_INVALID_ID;
    }
    
    return IMU_PORT_SUCCESS;
}

uint8_t imu_port_read_chip_id(imu_dev_t *dev)
{
    if (dev == NULL) {
        return 0;
    }
    
    uint8_t chip_id_buf[2] = {0};
    int8_t rslt = bmi3_get_regs(BMI3_REG_CHIP_ID, chip_id_buf, 2, &dev->bmi_dev);
    
    if (rslt != BMI3_OK) {
        return 0;
    }
    
    // Return first byte which contains the chip ID
    return chip_id_buf[0];
}

float imu_port_read_temperature(imu_dev_t *dev)
{
    if (dev == NULL) {
        return 0.0f;
    }
    
    // Read temperature sensor data
    struct bmi3_sensor_data sensor_data = {0};
    sensor_data.type = BMI323_TEMP;
    
    int8_t rslt = bmi323_get_sensor_data(&sensor_data, 1, &dev->bmi_dev);
    
    if (rslt != BMI3_OK) {
        return 0.0f;
    }
    
    // Convert raw temperature to degrees Celsius
    // Current IMU (BMI323) formula: temp_deg_c = (temp_data / 512.0) + 23.0
    return (sensor_data.sens_data.temp.x / 512.0f) + 23.0f;
}

imu_port_status_t imu_port_read_accel(imu_dev_t *dev, imu_sensor_data_t *accel)
{
    if (dev == NULL || accel == NULL) {
        return IMU_PORT_ERROR_NULL_PTR;
    }
    
    // Read accelerometer data
    struct bmi3_sensor_data sensor_data = {0};
    sensor_data.type = BMI323_ACCEL;
    
    int8_t rslt = bmi323_get_sensor_data(&sensor_data, 1, &dev->bmi_dev);
    if (rslt != BMI3_OK) {
        return IMU_PORT_ERROR_COMM_FAIL;
    }
    
    // Convert to g (assuming ±2g range, adjust based on actual config)
    accel->x = sensor_data.sens_data.acc.x / 16384.0f;
    accel->y = sensor_data.sens_data.acc.y / 16384.0f;
    accel->z = sensor_data.sens_data.acc.z / 16384.0f;
    
    return IMU_PORT_SUCCESS;
}

imu_port_status_t imu_port_read_gyro(imu_dev_t *dev, imu_sensor_data_t *gyro)
{
    if (dev == NULL || gyro == NULL) {
        return IMU_PORT_ERROR_NULL_PTR;
    }
    
    // Read gyroscope data
    struct bmi3_sensor_data sensor_data = {0};
    sensor_data.type = BMI323_GYRO;
    
    int8_t rslt = bmi323_get_sensor_data(&sensor_data, 1, &dev->bmi_dev);
    if (rslt != BMI3_OK) {
        return IMU_PORT_ERROR_COMM_FAIL;
    }
    
    // Convert to deg/s (assuming ±2000 deg/s range, adjust based on actual config)
    gyro->x = sensor_data.sens_data.gyr.x / 16.4f;
    gyro->y = sensor_data.sens_data.gyr.y / 16.4f;
    gyro->z = sensor_data.sens_data.gyr.z / 16.4f;
    
    return IMU_PORT_SUCCESS;
}

imu_port_status_t imu_port_read_accel_and_gyro(imu_dev_t *dev, imu_sensor_data_t *accel, imu_sensor_data_t *gyro)
{
    if (dev == NULL) {
        return IMU_PORT_ERROR_NULL_PTR;
    }
    
    // Optimized: Read both sensors in single operation
    struct bmi3_sensor_data sensor_data[2] = {0};
    sensor_data[0].type = BMI323_ACCEL;
    sensor_data[1].type = BMI323_GYRO;
    
    int8_t rslt = bmi323_get_sensor_data(sensor_data, 2, &dev->bmi_dev);
    if (rslt != BMI3_OK) {
        return IMU_PORT_ERROR_COMM_FAIL;
    }
    
    // Convert accelerometer data (if requested)
    if (accel != NULL) {
        accel->x = sensor_data[0].sens_data.acc.x / 16384.0f;
        accel->y = sensor_data[0].sens_data.acc.y / 16384.0f;
        accel->z = sensor_data[0].sens_data.acc.z / 16384.0f;
    }
    
    // Convert gyroscope data (if requested)
    if (gyro != NULL) {
        gyro->x = sensor_data[1].sens_data.gyr.x / 16.4f;
        gyro->y = sensor_data[1].sens_data.gyr.y / 16.4f;
        gyro->z = sensor_data[1].sens_data.gyr.z / 16.4f;
    }
    
    return IMU_PORT_SUCCESS;
}

imu_port_status_t imu_port_configure_accel(imu_dev_t *dev, imu_accel_range_t range, imu_odr_t odr)
{
    if (dev == NULL) {
        return IMU_PORT_ERROR_NULL_PTR;
    }
    
    // Validate parameters
    if (!validate_accel_range(range) || !validate_odr(odr)) {
        return IMU_PORT_ERROR_CONFIG;
    }
    
    struct bmi3_sens_config config = {0};
    config.type = BMI323_ACCEL;
    
    // Get current configuration
    int8_t rslt = bmi323_get_sensor_config(&config, 1, &dev->bmi_dev);
    if (rslt != BMI3_OK) {
        return IMU_PORT_ERROR_COMM_FAIL;
    }
    
    // Set new range, ODR, and enable sensor
    config.cfg.acc.range = (uint8_t)range;
    config.cfg.acc.odr = (uint16_t)odr;
    config.cfg.acc.acc_mode = BMI3_ACC_MODE_NORMAL;  // Enable accelerometer
    
    // Apply configuration
    rslt = bmi323_set_sensor_config(&config, 1, &dev->bmi_dev);
    if (rslt != BMI3_OK) {
        return IMU_PORT_ERROR_COMM_FAIL;
    }
    
    return IMU_PORT_SUCCESS;
}

imu_port_status_t imu_port_configure_gyro(imu_dev_t *dev, imu_gyro_range_t range, imu_odr_t odr)
{
    if (dev == NULL) {
        return IMU_PORT_ERROR_NULL_PTR;
    }
    
    // Validate parameters
    if (!validate_gyro_range(range) || !validate_odr(odr)) {
        return IMU_PORT_ERROR_CONFIG;
    }
    
    struct bmi3_sens_config config = {0};
    config.type = BMI323_GYRO;
    
    // Get current configuration
    int8_t rslt = bmi323_get_sensor_config(&config, 1, &dev->bmi_dev);
    if (rslt != BMI3_OK) {
        return IMU_PORT_ERROR_COMM_FAIL;
    }
    
    // Set new range, ODR, and enable sensor
    config.cfg.gyr.range = (uint16_t)range;
    config.cfg.gyr.odr = (uint16_t)odr;
    config.cfg.gyr.gyr_mode = BMI3_GYR_MODE_NORMAL;  // Enable gyroscope
    
    // Apply configuration
    rslt = bmi323_set_sensor_config(&config, 1, &dev->bmi_dev);
    if (rslt != BMI3_OK) {
        return IMU_PORT_ERROR_COMM_FAIL;
    }
    
    return IMU_PORT_SUCCESS;
}

/*---------------------------------------------------------------------------
 * Private Function Implementations (Validation Helpers)
 *---------------------------------------------------------------------------*/

STATIC bool validate_accel_range(imu_accel_range_t range)
{
    switch (range) {
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
    switch (range) {
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
    switch (odr) {
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
 * Private Function Implementations (SPI Callbacks)
 *---------------------------------------------------------------------------*/

STATIC int8_t imu_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    (void)intf_ptr;  // Unused
    
    if (reg_data == NULL) {
        return BMI3_E_NULL_PTR;
    }
    
    // BMI323 SPI Read Protocol:
    // The driver (bmi3.c) does: read(reg_addr, temp_buf, len + dummy_byte)
    // Then extracts: data[i] = temp_buf[i + dummy_byte]
    //
    // With dummy_byte = 1:
    //   - Driver wants 2 bytes, calls read() with len = 3
    //   - We must return buffer where:
    //     * temp_buf[0] = junk/dummy (will be skipped by driver)
    //     * temp_buf[1] = first data byte
    //     * temp_buf[2] = second data byte
    //
    // SPI transaction for chip ID (reg 0x00, len=3):
    //   TX: [0x80, 0x00, 0x00, 0x00]
    //   RX: [junk, junk, byte0, byte1]  ← 2 junk bytes: addr response + dummy response
    //
    // We need to return [junk, byte0, byte1] so driver gets [byte0, byte1]
    
    platform_spi_cs_low(IMU_PORT_CS_PIN);
    
    // SPI transaction for BMI323:
    // - Send: 1 address byte + len dummy bytes (to clock out response)
    // - Receive: 1 junk byte (during addr) + 1 BMI323 dummy + (len-1) data bytes
    // - Total: len+1 bytes transmitted and received
    uint8_t tx_buf[len + 1];
    uint8_t rx_buf[len + 1];
    
    memset(tx_buf, 0, len + 1);
    tx_buf[0] = reg_addr;
    
    if (platform_spi_transfer(tx_buf, rx_buf, len + 1) != PLATFORM_SPI_SUCCESS) {
        platform_spi_cs_high(IMU_PORT_CS_PIN);
        return BMI3_E_COM_FAIL;
    }
    
    // BMI323 SPI read protocol (from datasheet section 5.3):
    // TX: [addr, dummy, dummy, ...]
    // RX: [junk, dummy, data0, data1, ...]
    //
    // We skip the address echo (rx_buf[0]) by copying from rx_buf[i+1].
    // The BMI3 driver then skips dev->dummy_byte positions to skip the BMI323 dummy byte.
    // This two-stage skipping gives the driver the clean data it expects.
    for (uint32_t i = 0; i < len; i++) {
        reg_data[i] = rx_buf[i + 1];  // Skip address echo, driver will skip dummy
    }

    platform_spi_cs_high(IMU_PORT_CS_PIN);
    
    return BMI3_OK;
}

STATIC int8_t imu_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    (void)intf_ptr;  // Unused
    
    if (reg_data == NULL) {
        return BMI3_E_NULL_PTR;
    }
    
    platform_spi_cs_low(IMU_PORT_CS_PIN);
    
    // Send register address with write bit clear (bit 7 = 0)
    uint8_t addr_byte = reg_addr & 0x7F;
    if (platform_spi_transmit(&addr_byte, 1) != PLATFORM_SPI_SUCCESS) {
        platform_spi_cs_high(IMU_PORT_CS_PIN);
        return BMI3_E_COM_FAIL;
    }
    
    // Write the data
    if (platform_spi_transmit(reg_data, len) != PLATFORM_SPI_SUCCESS) {
        platform_spi_cs_high(IMU_PORT_CS_PIN);
        return BMI3_E_COM_FAIL;
    }
    
    platform_spi_cs_high(IMU_PORT_CS_PIN);
    
    return BMI3_OK;
}

STATIC void imu_delay_us(uint32_t period_us, void *intf_ptr)
{
    (void)intf_ptr;  // Unused
    platform_os_delay_us_blocking(period_us);
}


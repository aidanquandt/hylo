/*---------------------------------------------------------------------------
 * @file    bmi323_port.c
 * @brief   Port layer implementation for BMI323 sensor driver
 *          Translates Bosch driver callbacks to platform API calls
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "bmi323_port.h"
#include "platform_spi.h"
#include "platform_timer.h"
#include "bmi323.h"
#include <string.h>

/*---------------------------------------------------------------------------
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
STATIC int8_t bmi323_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
STATIC int8_t bmi323_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);
STATIC void bmi323_delay_us(uint32_t period_us, void *intf_ptr);

/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/
STATIC struct bmi3_dev bmi323_device = {
    .intf = BMI3_SPI_INTF,
    .read = bmi323_spi_read,
    .write = bmi323_spi_write,
    .delay_us = bmi323_delay_us,
    .intf_ptr = NULL,
    .read_write_len = 32,  // Max read/write length
};

/*---------------------------------------------------------------------------
 * Public Function Implementations
 *---------------------------------------------------------------------------*/

struct bmi3_dev* bmi323_port_init(void)
{
    // Port layer returns pointer to static device structure
    // SPI peripheral is configured in main system init
    return &bmi323_device;
}

int bmi323_port_probe_and_init(struct bmi3_dev *dev)
{
    if (dev == NULL) {
        return BMI323_ERROR;
    }
    
    // Initialize the BMI323 sensor
    int8_t rslt = bmi323_init(dev);
    if (rslt != BMI3_OK) {
        return BMI323_ERROR;
    }
    
    // Verify chip ID
    if (bmi323_port_check_device_id(dev) != BMI323_SUCCESS) {
        return BMI323_ERROR;
    }
    
    return BMI323_SUCCESS;
}

int bmi323_port_check_device_id(struct bmi3_dev *dev)
{
    if (dev == NULL) {
        return BMI323_ERROR;
    }
    
    uint8_t chip_id = bmi323_port_read_chip_id(dev);
    
    // Verify chip ID (BMI323 = 0x43, BMI330 = 0x44)
    if (chip_id != 0x43 && chip_id != 0x44) {
        return BMI323_ERROR;
    }
    
    return BMI323_SUCCESS;
}

uint8_t bmi323_port_read_chip_id(struct bmi3_dev *dev)
{
    if (dev == NULL) {
        return 0;
    }
    
    uint8_t chip_id = 0;
    int8_t rslt = bmi3_get_regs(BMI3_REG_CHIP_ID, &chip_id, 1, dev);
    
    if (rslt != BMI3_OK) {
        return 0;
    }
    
    return chip_id;
}

float bmi323_port_read_temperature(struct bmi3_dev *dev)
{
    if (dev == NULL) {
        return 0.0f;
    }
    
    // Read temperature sensor data
    struct bmi3_sensor_data sensor_data = {0};
    sensor_data.type = BMI323_TEMP;
    
    int8_t rslt = bmi323_get_sensor_data(&sensor_data, 1, dev);
    
    if (rslt != BMI3_OK) {
        return 0.0f;
    }
    
    // Convert raw temperature to degrees Celsius
    // BMI323 temp formula: temp_deg_c = (temp_data / 512.0) + 23.0
    return (sensor_data.sens_data.temp.x / 512.0f) + 23.0f;
}

int bmi323_port_read_accel(struct bmi3_dev *dev, bmi323_sensor_data_t *accel)
{
    if (dev == NULL || accel == NULL) {
        return BMI323_ERROR;
    }
    
    // Read accelerometer data
    struct bmi3_sensor_data sensor_data = {0};
    sensor_data.type = BMI323_ACCEL;
    
    int8_t rslt = bmi323_get_sensor_data(&sensor_data, 1, dev);
    if (rslt != BMI3_OK) {
        return BMI323_ERROR;
    }
    
    // Convert to g (assuming ±2g range, adjust based on actual config)
    accel->x = sensor_data.sens_data.acc.x / 16384.0f;
    accel->y = sensor_data.sens_data.acc.y / 16384.0f;
    accel->z = sensor_data.sens_data.acc.z / 16384.0f;
    
    return BMI323_SUCCESS;
}

int bmi323_port_read_gyro(struct bmi3_dev *dev, bmi323_sensor_data_t *gyro)
{
    if (dev == NULL || gyro == NULL) {
        return BMI323_ERROR;
    }
    
    // Read gyroscope data
    struct bmi3_sensor_data sensor_data = {0};
    sensor_data.type = BMI323_GYRO;
    
    int8_t rslt = bmi323_get_sensor_data(&sensor_data, 1, dev);
    if (rslt != BMI3_OK) {
        return BMI323_ERROR;
    }
    
    // Convert to deg/s (assuming ±2000 deg/s range, adjust based on actual config)
    gyro->x = sensor_data.sens_data.gyr.x / 16.4f;
    gyro->y = sensor_data.sens_data.gyr.y / 16.4f;
    gyro->z = sensor_data.sens_data.gyr.z / 16.4f;
    
    return BMI323_SUCCESS;
}

int bmi323_port_read_accel_and_gyro(struct bmi3_dev *dev, bmi323_sensor_data_t *accel, bmi323_sensor_data_t *gyro)
{
    if (dev == NULL) {
        return BMI323_ERROR;
    }
    
    // Optimized: Read both sensors in single operation
    struct bmi3_sensor_data sensor_data[2] = {0};
    sensor_data[0].type = BMI323_ACCEL;
    sensor_data[1].type = BMI323_GYRO;
    
    int8_t rslt = bmi323_get_sensor_data(sensor_data, 2, dev);
    if (rslt != BMI3_OK) {
        return BMI323_ERROR;
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
    
    return BMI323_SUCCESS;
}

int bmi323_port_configure_accel(struct bmi3_dev *dev, uint8_t range, uint16_t odr)
{
    if (dev == NULL) {
        return BMI323_ERROR;
    }
    
    struct bmi3_sens_config config = {0};
    config.type = BMI323_ACCEL;
    
    // Get current configuration
    int8_t rslt = bmi323_get_sensor_config(&config, 1, dev);
    if (rslt != BMI3_OK) {
        return BMI323_ERROR;
    }
    
    // Set new range and ODR
    config.cfg.acc.range = range;
    config.cfg.acc.odr = odr;
    
    // Apply configuration
    rslt = bmi323_set_sensor_config(&config, 1, dev);
    if (rslt != BMI3_OK) {
        return BMI323_ERROR;
    }
    
    return BMI323_SUCCESS;
}

int bmi323_port_configure_gyro(struct bmi3_dev *dev, uint16_t range, uint16_t odr)
{
    if (dev == NULL) {
        return BMI323_ERROR;
    }
    
    struct bmi3_sens_config config = {0};
    config.type = BMI323_GYRO;
    
    // Get current configuration
    int8_t rslt = bmi323_get_sensor_config(&config, 1, dev);
    if (rslt != BMI3_OK) {
        return BMI323_ERROR;
    }
    
    // Set new range and ODR
    config.cfg.gyr.range = range;
    config.cfg.gyr.odr = odr;
    
    // Apply configuration
    rslt = bmi323_set_sensor_config(&config, 1, dev);
    if (rslt != BMI3_OK) {
        return BMI323_ERROR;
    }
    
    return BMI323_SUCCESS;
}

/*---------------------------------------------------------------------------
 * Private Function Implementations (SPI Callbacks)
 *---------------------------------------------------------------------------*/

STATIC int8_t bmi323_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    (void)intf_ptr;  // Unused
    
    if (reg_data == NULL) {
        return BMI3_E_NULL_PTR;
    }
    
    platform_spi_cs_low(BMI323_CS_PIN);
    
    // Send register address with read bit set (bit 7 = 1)
    uint8_t addr_byte = reg_addr | 0x80;
    if (platform_spi_transmit(&addr_byte, 1) != PLATFORM_SPI_SUCCESS) {
        platform_spi_cs_high(BMI323_CS_PIN);
        return BMI3_E_COM_FAIL;
    }
    
    // BMI323 requires a dummy byte before data (per datasheet)
    uint8_t dummy = 0xFF;
    if (platform_spi_transmit(&dummy, 1) != PLATFORM_SPI_SUCCESS) {
        platform_spi_cs_high(BMI323_CS_PIN);
        return BMI3_E_COM_FAIL;
    }
    
    // Read the actual data
    if (platform_spi_receive(reg_data, len) != PLATFORM_SPI_SUCCESS) {
        platform_spi_cs_high(BMI323_CS_PIN);
        return BMI3_E_COM_FAIL;
    }
    
    platform_spi_cs_high(BMI323_CS_PIN);
    
    return BMI3_OK;
}

STATIC int8_t bmi323_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    (void)intf_ptr;  // Unused
    
    if (reg_data == NULL) {
        return BMI3_E_NULL_PTR;
    }
    
    platform_spi_cs_low(BMI323_CS_PIN);
    
    // Send register address with write bit clear (bit 7 = 0)
    uint8_t addr_byte = reg_addr & 0x7F;
    if (platform_spi_transmit(&addr_byte, 1) != PLATFORM_SPI_SUCCESS) {
        platform_spi_cs_high(BMI323_CS_PIN);
        return BMI3_E_COM_FAIL;
    }
    
    // Write the data
    if (platform_spi_transmit(reg_data, len) != PLATFORM_SPI_SUCCESS) {
        platform_spi_cs_high(BMI323_CS_PIN);
        return BMI3_E_COM_FAIL;
    }
    
    platform_spi_cs_high(BMI323_CS_PIN);
    
    return BMI3_OK;
}

STATIC void bmi323_delay_us(uint32_t period_us, void *intf_ptr)
{
    (void)intf_ptr;  // Unused
    platform_delay_us(period_us);
}


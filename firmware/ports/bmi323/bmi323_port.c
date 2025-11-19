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
#include "gpio.h"
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

// Debug variables (watch in debugger)
STATIC volatile uint8_t debug_chip_id_raw[8] = {0};  // Raw SPI transaction result
STATIC volatile uint8_t debug_chip_id_parsed = 0;     // Parsed chip ID value
STATIC volatile int8_t debug_last_spi_result = 0;     // Last SPI operation result
STATIC volatile int8_t debug_init_result = 0;         // Result from bmi323_init()

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
    
    // CRITICAL: BMI323 requires a RISING EDGE on CSB after power-up to select SPI mode
    // Workaround: Force CS LOW briefly, then HIGH to create rising edge
    // This assumes the device has been powered up long enough (called after 2s delay in test module)
    
    // Force CS LOW
    platform_spi_cs_low(BMI323_CS_PIN);
    platform_delay_us(100);  // Hold LOW briefly
    
    // Set CS HIGH - this creates the rising edge needed for SPI mode selection
    platform_spi_cs_high(BMI323_CS_PIN);
    platform_delay_us(250);  // Wait for mode switch (datasheet: 200µs min)
    
    // Perform a dummy SPI read to fully activate SPI mode
    uint8_t dummy_rx[3] = {0};
    platform_spi_cs_low(BMI323_CS_PIN);
    uint8_t dummy_cmd = 0x80;  // Read chip ID register
    platform_spi_transmit(&dummy_cmd, 1);
    platform_spi_receive(dummy_rx, 3);
    platform_spi_cs_high(BMI323_CS_PIN);
    platform_delay_ms(1);
    
    // Now the BMI323 should be in SPI mode
    
    // First verify chip ID to ensure SPI communication works
    uint8_t chip_id_buf[2] = {0};
    int8_t rslt = bmi3_get_regs(BMI3_REG_CHIP_ID, chip_id_buf, 2, dev);
    if (rslt != BMI3_OK) {
        return (int)rslt;  // SPI communication failed
    }
    
    // Store raw chip ID read in debug buffer
    debug_chip_id_raw[0] = chip_id_buf[0];
    debug_chip_id_raw[1] = chip_id_buf[1];
    
    // Chip ID is in the first byte of the 2-byte read
    uint8_t chip_id = chip_id_buf[0];
    debug_chip_id_parsed = chip_id;
    
    // Verify chip ID is valid
    // Note: BMI323 shuttle boards may report 0x44 instead of documented 0x43
    if (chip_id != 0x43 && chip_id != 0x44 && chip_id != 0x48) {
        return BMI3_E_DEV_NOT_FOUND;
    }
    
    // Initialize the BMI323 sensor (this will do soft reset internally)
    rslt = bmi323_init(dev);
    debug_init_result = rslt;  // Store for debugging

    if (rslt != BMI3_OK) {
        return (int)rslt;  // Return actual API error code for debugging
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
    
    uint8_t chip_id_buf[2] = {0};
    int8_t rslt = bmi3_get_regs(BMI3_REG_CHIP_ID, chip_id_buf, 2, dev);
    
    debug_last_spi_result = rslt;
    
    if (rslt != BMI3_OK) {
        return 0;
    }
    
    // Store raw result in debug buffer
    debug_chip_id_raw[0] = chip_id_buf[0];
    debug_chip_id_raw[1] = chip_id_buf[1];
    debug_chip_id_parsed = chip_id_buf[0];
    
    // Return first byte which contains the chip ID
    return chip_id_buf[0];
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
    
    // Set new range, ODR, and enable sensor
    config.cfg.acc.range = range;
    config.cfg.acc.odr = odr;
    config.cfg.acc.acc_mode = BMI3_ACC_MODE_NORMAL;  // Enable accelerometer
    
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
    
    // Set new range, ODR, and enable sensor
    config.cfg.gyr.range = range;
    config.cfg.gyr.odr = odr;
    config.cfg.gyr.gyr_mode = BMI3_GYR_MODE_NORMAL;  // Enable gyroscope
    
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
    
    platform_spi_cs_low(BMI323_CS_PIN);
    
    uint8_t tx_buf[len + 1];
    uint8_t rx_buf[len + 1];
    
    tx_buf[0] = reg_addr;
    for (uint32_t i = 1; i <= len; i++) {
        tx_buf[i] = 0;
    }
    
    if (platform_spi_transfer(tx_buf, rx_buf, len + 1) != PLATFORM_SPI_SUCCESS) {
        platform_spi_cs_high(BMI323_CS_PIN);
        return BMI3_E_COM_FAIL;
    }
    
    // The BMI323 has 2 dummy bytes in SPI reads:
    // - rx_buf[0] = junk from address byte transmission  
    // - rx_buf[1] = junk from first dummy byte
    // - rx_buf[2+] = actual data
    // 
    // Skip both junk bytes and return actual data starting at rx_buf[2]
    // The driver does NOT skip anything further (dummy_byte=1 just increases the len)
    for (uint32_t i = 0; i < len; i++) {
        reg_data[i] = rx_buf[i + 2];  // Skip 2 junk bytes
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


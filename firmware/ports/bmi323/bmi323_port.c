/*---------------------------------------------------------------------------
 * @file    bmi323_port.c
 * @brief   Port layer implementation for BMI323 sensor driver
 * @note    Thread safety: BMI323 API is not inherently thread-safe.
 *          If accessed from multiple tasks, add critical sections using
 *          platform_critical_enter/exit around API calls.
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "bmi323_port.h"
#include "platform_spi.h"
#include "platform_timer.h"
#include <string.h>

/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/
static struct bmi3_dev bmi323_device;

/*---------------------------------------------------------------------------
 * Private Function Implementations
 *---------------------------------------------------------------------------*/

/**
 * @brief Delay function for BMI323 driver
 */
void bmi323_port_delay_us(uint32_t period_us, void *intf_ptr)
{
    (void)intf_ptr;  // Unused
    platform_delay_us(period_us);
}

/**
 * @brief SPI read function for BMI323 driver
 * @note BMI323 uses SPI mode: dummy byte after register address
 */
int8_t bmi323_port_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
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

/**
 * @brief SPI write function for BMI323 driver
 */
int8_t bmi323_port_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
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

/*---------------------------------------------------------------------------
 * Public Function Implementations
 *---------------------------------------------------------------------------*/

/**
 * @brief Initialize the BMI323 port layer
 */
struct bmi3_dev* bmi323_port_init(void)
{
    // Zero out the device structure
    memset(&bmi323_device, 0, sizeof(bmi323_device));
    
    // Configure the device structure
    bmi323_device.intf = BMI3_SPI_INTF;
    bmi323_device.read = bmi323_port_spi_read;
    bmi323_device.write = bmi323_port_spi_write;
    bmi323_device.delay_us = bmi323_port_delay_us;
    bmi323_device.intf_ptr = NULL;  // Not used in this implementation
    bmi323_device.read_write_len = 32;  // Max read/write length (adjust if needed)
    
    return &bmi323_device;
}

/**
 * @brief Check BMI323 device ID to verify communication
 */
int bmi323_port_check_device_id(struct bmi3_dev *dev)
{
    if (dev == NULL) {
        return BMI323_ERROR;
    }
    
    int8_t rslt;
    uint8_t chip_id = 0;
    
    // Read chip ID
    rslt = bmi3_get_regs(BMI3_REG_CHIP_ID, &chip_id, 1, dev);
    if (rslt != BMI3_OK) {
        return BMI323_ERROR;
    }
    
    // Verify chip ID (BMI323 = 0x43, BMI330 = 0x44)
    if (chip_id != 0x43 && chip_id != 0x44) {
        return BMI323_ERROR;
    }
    
    return BMI323_SUCCESS;
}

/**
 * @brief Probe and initialize the BMI323 device
 */
int bmi323_port_probe_and_init(struct bmi3_dev *dev)
{
    if (dev == NULL) {
        return BMI323_ERROR;
    }
    
    int8_t rslt;
    
    // Initialize the BMI323 sensor
    rslt = bmi3_init(dev);
    if (rslt != BMI3_OK) {
        return BMI323_ERROR;
    }
    
    // Verify chip ID
    if (bmi323_port_check_device_id(dev) != BMI323_SUCCESS) {
        return BMI323_ERROR;
    }
    
    return BMI323_SUCCESS;
}

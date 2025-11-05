/*---------------------------------------------------------------------------
 * @file    dw3000_port.c
 * @brief   Port layer implementation for DW3000 UWB driver
 *          Translates Qorvo driver callbacks to platform API calls
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "dw3000_port.h"
#include "platform_spi.h"
#include "platform_gpio.h"
#include "platform_timer.h"
#include "deca_device_api.h"
#include "FreeRTOS.h"
#include "task.h"

/*---------------------------------------------------------------------------
 * External Driver Declaration
 *---------------------------------------------------------------------------*/
extern const struct dwt_driver_s dw3000_driver;

/*---------------------------------------------------------------------------
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
STATIC int32_t dw3000_spi_read(uint16_t headerLength, uint8_t *headerBuffer,
                                 uint16_t readLength, uint8_t *readBuffer);
STATIC int32_t dw3000_spi_write(uint16_t headerLength, const uint8_t *headerBuffer,
                                  uint16_t bodyLength, const uint8_t *bodyBuffer);
STATIC int32_t dw3000_spi_write_crc(uint16_t headerLength, const uint8_t *headerBuffer,
                                      uint16_t bodyLength, const uint8_t *bodyBuffer,
                                      uint8_t crc8);
STATIC void dw3000_spi_set_slow_rate(void);
STATIC void dw3000_spi_set_fast_rate(void);

/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/
STATIC struct dwt_spi_s dw3000_spi_fns = {
    .readfromspi = dw3000_spi_read,
    .writetospi = dw3000_spi_write,
    .writetospiwithcrc = dw3000_spi_write_crc,
    .setslowrate = dw3000_spi_set_slow_rate,
    .setfastrate = dw3000_spi_set_fast_rate,
};

STATIC dwchip_t dw3000_chip = {
    .SPI = &dw3000_spi_fns,
    .wakeup_device_with_io = dw3000_port_wakeup_device,
};

/*---------------------------------------------------------------------------
 * Public Function Implementations
 *---------------------------------------------------------------------------*/

dwchip_t* dw3000_port_init(void)
{
    // Port layer returns pointer to static chip structure
    // GPIO and timing peripherals are configured in main system init
    return &dw3000_chip;
}

int dw3000_port_probe_and_init(dwchip_t *chip)
{
    if (chip == NULL) {
        return DW3000_ERROR;
    }
    
    // Set up driver list for probe
    static const struct dwt_driver_s *driver_list[] = { &dw3000_driver };
    
    // Probe the device to detect and configure driver
    struct dwt_probe_s probe_data = {
        .dw = chip,
        .spi = &dw3000_spi_fns,
        .wakeup_device_with_io = dw3000_port_wakeup_device,
        .driver_list = (struct dwt_driver_s **)driver_list,
        .dw_driver_num = 1
    };
    
    int ret = dwt_probe(&probe_data);
    if (ret != DWT_SUCCESS) {
        return DW3000_ERROR;
    }
    
    // Initialize device - reads calibration data from OTP
    ret = dwt_initialise(DWT_READ_OTP_ALL);
    if (ret != DWT_SUCCESS) {
        return DW3000_ERROR;
    }
    
    return DW3000_SUCCESS;
}

void dw3000_port_wakeup_device(void)
{
    // Toggle CS pin to wake up DW3000 from sleep
    // The DW3000 wakes up on CS rising edge after being low for >500us
    platform_spi_cs_low(DW3000_CS_PIN);
    platform_delay_us(600);  // Wait >500us using precise microsecond delay
    platform_spi_cs_high(DW3000_CS_PIN);
    platform_delay_ms(1);  // Wait 1ms for device to fully wake up
}

int dw3000_port_check_device_id(void)
{
    int ret = dwt_check_dev_id();
    return (ret == DWT_SUCCESS) ? DW3000_SUCCESS : DW3000_ERROR;
}

uint32_t dw3000_port_read_device_id(void)
{
    return dwt_readdevid();
}

float dw3000_port_read_temperature(void)
{
    // Read combined temperature and voltage register
    uint16_t raw_reading = dwt_readtempvbat();
    
    // Extract and convert temperature (upper 8 bits)
    uint8_t raw_temp = (uint8_t)(raw_reading >> 8);
    return dwt_convertrawtemperature(raw_temp);
}

float dw3000_port_read_voltage(void)
{
    // Read combined temperature and voltage register
    uint16_t raw_reading = dwt_readtempvbat();
    
    // Extract and convert voltage (lower 8 bits)
    uint8_t raw_voltage = (uint8_t)(raw_reading & 0xFFU);
    return dwt_convertrawvoltage(raw_voltage);
}

void dw3000_port_read_temp_and_voltage(float *temperature, float *voltage)
{
    // Optimized: Read both from single register access
    uint16_t raw_reading = dwt_readtempvbat();
    
    // Extract and convert both values
    uint8_t raw_temp = (uint8_t)(raw_reading >> 8);
    uint8_t raw_voltage = (uint8_t)(raw_reading & 0xFFU);
    
    if (temperature != NULL) {
        *temperature = dwt_convertrawtemperature(raw_temp);
    }
    
    if (voltage != NULL) {
        *voltage = dwt_convertrawvoltage(raw_voltage);
    }
}

/*---------------------------------------------------------------------------
 * Private Function Implementations - SPI Callbacks for Qorvo Driver
 *---------------------------------------------------------------------------*/

STATIC int32_t dw3000_spi_read(uint16_t headerLength, uint8_t *headerBuffer,
                                 uint16_t readLength, uint8_t *readBuffer)
{
    if ((headerBuffer == NULL) || (readBuffer == NULL)) {
        return DWT_ERROR;
    }

    platform_spi_cs_low(DW3000_CS_PIN);

    // Transmit header
    if (platform_spi_transmit(headerBuffer, headerLength) != PLATFORM_SPI_SUCCESS) {
        platform_spi_cs_high(DW3000_CS_PIN);
        return DWT_ERROR;
    }

    // Small delay to ensure DW3000 is ready
    for (volatile int i = 0; i < 10; i++);

    // Receive data
    if (platform_spi_receive(readBuffer, readLength) != PLATFORM_SPI_SUCCESS) {
        platform_spi_cs_high(DW3000_CS_PIN);
        return DWT_ERROR;
    }

    platform_spi_cs_high(DW3000_CS_PIN);

    return DWT_SUCCESS;
}

STATIC int32_t dw3000_spi_write(uint16_t headerLength, const uint8_t *headerBuffer,
                                  uint16_t bodyLength, const uint8_t *bodyBuffer)
{
    if (headerBuffer == NULL) {
        return DWT_ERROR;
    }

    platform_spi_cs_low(DW3000_CS_PIN);

    // Transmit header
    if (platform_spi_transmit(headerBuffer, headerLength) != PLATFORM_SPI_SUCCESS) {
        platform_spi_cs_high(DW3000_CS_PIN);
        return DWT_ERROR;
    }

    // Transmit body if present
    if ((bodyLength > 0U) && (bodyBuffer != NULL)) {
        if (platform_spi_transmit(bodyBuffer, bodyLength) != PLATFORM_SPI_SUCCESS) {
            platform_spi_cs_high(DW3000_CS_PIN);
            return DWT_ERROR;
        }
    }

    platform_spi_cs_high(DW3000_CS_PIN);

    return DWT_SUCCESS;
}

STATIC int32_t dw3000_spi_write_crc(uint16_t headerLength, const uint8_t *headerBuffer,
                                      uint16_t bodyLength, const uint8_t *bodyBuffer,
                                      uint8_t crc8)
{
    if (headerBuffer == NULL) {
        return DWT_ERROR;
    }

    platform_spi_cs_low(DW3000_CS_PIN);

    // Transmit header
    if (platform_spi_transmit(headerBuffer, headerLength) != PLATFORM_SPI_SUCCESS) {
        platform_spi_cs_high(DW3000_CS_PIN);
        return DWT_ERROR;
    }

    // Transmit body if present
    if ((bodyLength > 0U) && (bodyBuffer != NULL)) {
        if (platform_spi_transmit(bodyBuffer, bodyLength) != PLATFORM_SPI_SUCCESS) {
            platform_spi_cs_high(DW3000_CS_PIN);
            return DWT_ERROR;
        }
    }

    // Transmit CRC
    if (platform_spi_transmit(&crc8, 1) != PLATFORM_SPI_SUCCESS) {
        platform_spi_cs_high(DW3000_CS_PIN);
        return DWT_ERROR;
    }

    platform_spi_cs_high(DW3000_CS_PIN);

    return DWT_SUCCESS;
}

STATIC void dw3000_spi_set_slow_rate(void)
{
    // Set SPI to slow speed for initialization
    platform_spi_set_speed(PLATFORM_SPI_SPEED_SLOW);
}

STATIC void dw3000_spi_set_fast_rate(void)
{
    // Set SPI to fast speed for normal operation
    platform_spi_set_speed(PLATFORM_SPI_SPEED_FAST);
}

/*---------------------------------------------------------------------------
 * Platform Compatibility Functions Required by Qorvo Driver
 *---------------------------------------------------------------------------*/

void deca_usleep(unsigned long time_us)
{
    platform_delay_us((uint32_t)time_us);
}

void deca_sleep(unsigned int time_ms)
{
    platform_delay_ms((uint32_t)time_ms);
}

decaIrqStatus_t decamutexon(void)
{
    // Enter critical section to prevent concurrent access to DW3000
    // FreeRTOS automatically saves interrupt state
    taskENTER_CRITICAL();
    return 0;  // Return value not used with FreeRTOS critical sections
}

void decamutexoff(decaIrqStatus_t s)
{
    (void)s;  // Parameter not used with FreeRTOS critical sections
    
    // Exit critical section and restore interrupts
    taskEXIT_CRITICAL();
}

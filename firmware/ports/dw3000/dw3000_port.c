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
#include "platform_critical.h"
#include "deca_device_api.h"

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

void dw3000_port_set_pan_id(uint16_t pan_id)
{
    dwt_setpanid(pan_id);
}

void dw3000_port_set_address(uint16_t address)
{
    dwt_setaddress16(address);
}

int dw3000_port_configure_tx(uint8_t channel)
{
    dwt_config_t config = {
        .chan = channel,
        .txPreambLength = DWT_PLEN_128,
        .rxPAC = DWT_PAC8,
        .txCode = 9,
        .rxCode = 9,
        .sfdType = DWT_SFD_DW_8,
        .dataRate = DWT_BR_6M8,
        .phrMode = DWT_PHRMODE_STD,
        .phrRate = DWT_PHRRATE_STD,
        .sfdTO = (129 + 8 - 8),
        .stsMode = DWT_STS_MODE_OFF,
        .stsLength = DWT_STS_LEN_64,
        .pdoaMode = DWT_PDOA_M0
    };
    
    if (dwt_configure(&config) != DWT_SUCCESS) {
        return DW3000_ERROR;
    }
    
    // Enable 802.15.4 frame filtering - allow data frames
    dwt_configureframefilter(DWT_FF_ENABLE_802_15_4, DWT_FF_DATA_EN);
    
    // Set TX power
    dwt_txconfig_t txconfig = {
        .PGdly = 0x34,
        .power = 0xFEFEFEFEUL
    };
    dwt_configuretxrf(&txconfig);
    
    return DW3000_SUCCESS;
}

int dw3000_port_configure_rx(uint8_t channel)
{
    // Use same config as TX
    if (dw3000_port_configure_tx(channel) != DW3000_SUCCESS) {
        return DW3000_ERROR;
    }
    
    // Enable receiver
    if (dwt_rxenable(DWT_START_RX_IMMEDIATE) != DWT_SUCCESS) {
        return DW3000_ERROR;
    }
    
    return DW3000_SUCCESS;
}

int dw3000_port_send_message(const uint8_t *data, uint16_t length)
{
    if (data == NULL || length == 0 || length > 127) {
        return DW3000_ERROR;
    }
    
    // Write data to TX buffer
    dwt_writetxdata(length, (uint8_t*)data, 0);
    
    // Set frame length (data + 2-byte CRC)
    dwt_writetxfctrl(length + 2, 0, 0);
    
    // Start transmission (immediate, no response expected)
    if (dwt_starttx(DWT_START_TX_IMMEDIATE) != DWT_SUCCESS) {
        return DW3000_ERROR;
    }
    
    // Wait for TX to complete by polling status
    uint32_t status = 0;
    uint32_t timeout = 1000;  // 1000 iterations
    while (timeout--) {
        status = dwt_readsysstatuslo();
        if (status & DWT_INT_TXFRS_BIT_MASK) {
            // TX complete - clear flag
            dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
            return DW3000_SUCCESS;
        }
        platform_delay_us(10);
    }
    
    // Timeout
    return DW3000_ERROR;
}

int dw3000_port_receive_message(uint8_t *data, uint16_t max_length, uint16_t *received_length)
{
    if (data == NULL || received_length == NULL) {
        return DW3000_ERROR;
    }
    
    // Read system status register
    uint32_t status = dwt_readsysstatuslo();
    
    // Check for any RX errors first and clear them
    if (status & (DWT_INT_RXPHE_BIT_MASK | DWT_INT_RXFCE_BIT_MASK | 
                  DWT_INT_RXFSL_BIT_MASK | DWT_INT_RXFTO_BIT_MASK |
                  DWT_INT_RXOVRR_BIT_MASK | DWT_INT_RXPTO_BIT_MASK)) {
        // Clear error flags and restart RX
        dwt_writesysstatuslo(SYS_STATUS_ALL_RX_ERR);
        dwt_forcetrxoff();
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        return DW3000_ERROR;
    }
    
    // Check if good frame received (RXFCG bit set)
    if ((status & DWT_INT_RXFCG_BIT_MASK) == 0) {
        // No good frame received
        return DW3000_ERROR;
    }
    
    // Get frame length
    uint8_t rng = 0;
    uint16_t frame_len = dwt_getframelength(&rng);
    
    if (frame_len == 0 || frame_len > max_length) {
        // Frame too large or invalid - force off, clear status and re-enable RX
        dwt_forcetrxoff();
        dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR);
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        return DW3000_ERROR;
    }
    
    // Read received data (subtract 2-byte CRC from length)
    uint16_t data_len = (frame_len > 2) ? (frame_len - 2) : 0;
    dwt_readrxdata(data, data_len, 0);
    *received_length = data_len;
    
    // Force transceiver off, clear ALL status flags, then re-enable receiver
    dwt_forcetrxoff();
    dwt_writesysstatuslo(SYS_STATUS_ALL_RX_GOOD | SYS_STATUS_ALL_RX_ERR);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    
    return DW3000_SUCCESS;
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
    // Uses platform abstraction to remain RTOS-agnostic
    return (decaIrqStatus_t)platform_critical_enter();
}

void decamutexoff(decaIrqStatus_t s)
{
    // Exit critical section and restore interrupts
    platform_critical_exit((platform_critical_state_t)s);
}

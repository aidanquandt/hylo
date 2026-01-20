/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "uwb_port.h"
#include "deca_device_api.h"
#include "deca_interface.h"
#include "platform_gpio.h"
#include "platform_os.h"
#include "platform_spi.h"
#include "platform_timer.h"
#include "uart_manager.h"

/*---------------------------------------------------------------------------
 * Type Definitions
 *---------------------------------------------------------------------------*/
struct uwb_dev_s
{
    dwchip_t dw_chip;
    uint64_t last_tx_ts;
    uint64_t last_rx_ts;
};

/*---------------------------------------------------------------------------
 * External Driver Declaration
 *---------------------------------------------------------------------------*/
extern const struct dwt_driver_s dw3000_driver;

/*---------------------------------------------------------------------------
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
STATIC int32_t uwb_spi_read(uint16_t headerLength, uint8_t* headerBuffer, uint16_t readLength,
                            uint8_t* readBuffer);
STATIC int32_t uwb_spi_write(uint16_t headerLength, const uint8_t* headerBuffer,
                             uint16_t bodyLength, const uint8_t* bodyBuffer);
STATIC int32_t uwb_spi_write_crc(uint16_t headerLength, const uint8_t* headerBuffer,
                                 uint16_t bodyLength, const uint8_t* bodyBuffer, uint8_t crc8);
STATIC void uwb_spi_set_slow_rate(void);
STATIC void uwb_spi_set_fast_rate(void);
STATIC void uwb_wakeup_device_impl(void);
STATIC bool validate_channel(const uwb_channel_t channel);
/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/
STATIC struct dwt_spi_s uwb_spi_fns = {
    .readfromspi = uwb_spi_read,
    .writetospi = uwb_spi_write,
    .writetospiwithcrc = uwb_spi_write_crc,
    .setslowrate = uwb_spi_set_slow_rate,
    .setfastrate = uwb_spi_set_fast_rate,
};

STATIC struct uwb_dev_s uwb_device = {.dw_chip = {
                                          .SPI = &uwb_spi_fns,
                                          .wakeup_device_with_io = uwb_wakeup_device_impl,
                                      }};

/*---------------------------------------------------------------------------
 * Public Function Implementations
 *---------------------------------------------------------------------------*/
uwb_dev_t* uwb_port_init(void)
{
    return &uwb_device;
}

uwb_port_status_t uwb_port_probe_and_init(uwb_dev_t* dev)
{
    if (dev == NULL)
    {
        return UWB_PORT_ERROR_NULL_PTR;
    }

    static const struct dwt_driver_s* driver_list[] = {&dw3000_driver};

    struct dwt_probe_s probe_data = {.dw = &dev->dw_chip,
                                     .spi = &uwb_spi_fns,
                                     .wakeup_device_with_io = uwb_wakeup_device_impl,
                                     .driver_list = (struct dwt_driver_s**)driver_list,
                                     .dw_driver_num = 1};

    int ret = dwt_probe(&probe_data);
    if (ret != DWT_SUCCESS)
    {
        return UWB_PORT_ERROR_COMM_FAIL;
    }

    dwt_softreset(0);
    platform_os_delay_ms(3);

    uint32_t timeout = 100;
    while (!dwt_checkidlerc() && timeout--)
    {
        platform_os_delay_ms(10);
    }
    if (timeout == 0)
    {
        return UWB_PORT_ERROR_INIT_FAIL;
    }

    ret = dwt_initialise(DWT_READ_OTP_ALL);
    if (ret != DWT_SUCCESS)
    {
        return UWB_PORT_ERROR_INIT_FAIL;
    }

    return UWB_PORT_SUCCESS;
}

void uwb_port_wakeup_device(uwb_dev_t* dev)
{
    if (dev == NULL)
    {
        return;
    }
    uwb_wakeup_device_impl();
}

uwb_port_status_t uwb_port_soft_reset(uwb_dev_t* dev)
{
    if (dev == NULL)
    {
        return UWB_PORT_ERROR_NULL_PTR;
    }

    dwt_softreset(0);
    platform_os_delay_ms(3);

    return UWB_PORT_SUCCESS;
}

STATIC void uwb_wakeup_device_impl(void)
{
    platform_spi_cs_low(UWB_PORT_CS_PIN);
    platform_os_delay_us_blocking(600);
    platform_spi_cs_high(UWB_PORT_CS_PIN);
    platform_os_delay_ms(1);
}

uwb_port_status_t uwb_port_check_device_id(uwb_dev_t* dev)
{
    if (dev == NULL)
    {
        return UWB_PORT_ERROR_NULL_PTR;
    }

    int ret = dwt_check_dev_id();
    if (ret != DWT_SUCCESS)
    {
        return UWB_PORT_ERROR_INVALID_ID;
    }

    return UWB_PORT_SUCCESS;
}

uint32_t uwb_port_read_device_id(uwb_dev_t* dev)
{
    if (dev == NULL)
    {
        return 0;
    }

    return dwt_readdevid();
}

float uwb_port_read_temperature(uwb_dev_t* dev)
{
    if (dev == NULL)
    {
        return 0.0f;
    }

    uint16_t raw_reading = dwt_readtempvbat();

    uint8_t raw_temp = (uint8_t)(raw_reading >> 8);
    return dwt_convertrawtemperature(raw_temp);
}

float uwb_port_read_voltage(uwb_dev_t* dev)
{
    if (dev == NULL)
    {
        return 0.0f;
    }

    uint16_t raw_reading = dwt_readtempvbat();

    uint8_t raw_voltage = (uint8_t)(raw_reading & 0xFFU);
    return dwt_convertrawvoltage(raw_voltage);
}

void uwb_port_read_temp_and_voltage(uwb_dev_t* dev, float* temperature, float* voltage)
{
    if (dev == NULL || (temperature == NULL && voltage == NULL))
    {
        return;
    }

    uint16_t raw_reading = dwt_readtempvbat();

    uint8_t raw_temp = (uint8_t)(raw_reading >> 8);
    uint8_t raw_voltage = (uint8_t)(raw_reading & 0xFFU);

    if (temperature != NULL)
    {
        *temperature = dwt_convertrawtemperature(raw_temp);
    }

    if (voltage != NULL)
    {
        *voltage = dwt_convertrawvoltage(raw_voltage);
    }
}

void uwb_port_set_pan_id(uwb_dev_t* dev, uint16_t pan_id)
{
    if (dev == NULL)
    {
        return;
    }
    dwt_setpanid(pan_id);
}

void uwb_port_set_address(uwb_dev_t* dev, uint16_t address)
{
    if (dev == NULL)
    {
        return;
    }
    dwt_setaddress16(address);
}

/*---------------------------------------------------------------------------
 * Private Function Implementations
 *---------------------------------------------------------------------------*/

STATIC bool validate_channel(const uwb_channel_t channel)
{
    switch (channel)
    {
        case UWB_CHANNEL_1:
        case UWB_CHANNEL_2:
        case UWB_CHANNEL_3:
        case UWB_CHANNEL_4:
        case UWB_CHANNEL_5:
        case UWB_CHANNEL_7:
        case UWB_CHANNEL_9:
            return true;
        default:
            return false;
    }
}

STATIC int32_t uwb_spi_read(uint16_t headerLength, uint8_t* headerBuffer, uint16_t readLength,
                            uint8_t* readBuffer)
{
    if ((headerBuffer == NULL) || (readBuffer == NULL))
    {
        return DWT_ERROR;
    }

    platform_spi_cs_low(UWB_PORT_CS_PIN);

    if (platform_spi_transmit(headerBuffer, headerLength) != PLATFORM_SPI_SUCCESS)
    {
        platform_spi_cs_high(UWB_PORT_CS_PIN);
        return DWT_ERROR;
    }

    for (volatile int i = 0; i < 10; i++)
        ;

    if (platform_spi_receive(readBuffer, readLength) != PLATFORM_SPI_SUCCESS)
    {
        platform_spi_cs_high(UWB_PORT_CS_PIN);
        return DWT_ERROR;
    }

    platform_spi_cs_high(UWB_PORT_CS_PIN);

    return DWT_SUCCESS;
}

STATIC int32_t uwb_spi_write(uint16_t headerLength, const uint8_t* headerBuffer,
                             uint16_t bodyLength, const uint8_t* bodyBuffer)
{
    if (headerBuffer == NULL)
    {
        return DWT_ERROR;
    }

    platform_spi_cs_low(UWB_PORT_CS_PIN);

    if (platform_spi_transmit(headerBuffer, headerLength) != PLATFORM_SPI_SUCCESS)
    {
        platform_spi_cs_high(UWB_PORT_CS_PIN);
        return DWT_ERROR;
    }

    if ((bodyLength > 0U) && (bodyBuffer != NULL))
    {
        if (platform_spi_transmit(bodyBuffer, bodyLength) != PLATFORM_SPI_SUCCESS)
        {
            platform_spi_cs_high(UWB_PORT_CS_PIN);
            return DWT_ERROR;
        }
    }

    platform_spi_cs_high(UWB_PORT_CS_PIN);

    return DWT_SUCCESS;
}

STATIC int32_t uwb_spi_write_crc(uint16_t headerLength, const uint8_t* headerBuffer,
                                 uint16_t bodyLength, const uint8_t* bodyBuffer, uint8_t crc8)
{
    if (headerBuffer == NULL)
    {
        return DWT_ERROR;
    }

    platform_spi_cs_low(UWB_PORT_CS_PIN);

    if (platform_spi_transmit(headerBuffer, headerLength) != PLATFORM_SPI_SUCCESS)
    {
        platform_spi_cs_high(UWB_PORT_CS_PIN);
        return DWT_ERROR;
    }

    if ((bodyLength > 0U) && (bodyBuffer != NULL))
    {
        if (platform_spi_transmit(bodyBuffer, bodyLength) != PLATFORM_SPI_SUCCESS)
        {
            platform_spi_cs_high(UWB_PORT_CS_PIN);
            return DWT_ERROR;
        }
    }

    if (platform_spi_transmit(&crc8, 1) != PLATFORM_SPI_SUCCESS)
    {
        platform_spi_cs_high(UWB_PORT_CS_PIN);
        return DWT_ERROR;
    }

    platform_spi_cs_high(UWB_PORT_CS_PIN);

    return DWT_SUCCESS;
}

STATIC void uwb_spi_set_slow_rate(void)
{
    platform_spi_cs_low(UWB_PORT_CS_PIN);
    platform_spi_set_speed(PLATFORM_SPI_SPEED_SLOW);
    platform_spi_cs_high(UWB_PORT_CS_PIN);
}

STATIC void uwb_spi_set_fast_rate(void)
{
    platform_spi_cs_low(UWB_PORT_CS_PIN);
    platform_spi_set_speed(PLATFORM_SPI_SPEED_FAST);
    platform_spi_cs_high(UWB_PORT_CS_PIN);
}

/*---------------------------------------------------------------------------
 * UWB Configuration and Communication Functions
 *---------------------------------------------------------------------------*/

uwb_port_status_t uwb_port_configure(uwb_dev_t* dev, uwb_channel_t channel)
{
    if (dev == NULL)
    {
        return UWB_PORT_ERROR_NULL_PTR;
    }

    if (!validate_channel(channel))
    {
        return UWB_PORT_ERROR_CONFIG;
    }

    // Stop any active TX/RX operation before reconfiguring
    dwt_forcetrxoff();

    // Clear all RX status flags to start with clean state
    dwt_writesysstatuslo(SYS_STATUS_ALL_RX_GOOD | SYS_STATUS_ALL_RX_ERR);

    dwt_config_t config = {.chan = (uint8_t)channel,
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
                           .pdoaMode = DWT_PDOA_M0};

    if (dwt_configure(&config) != DWT_SUCCESS)
    {
        return UWB_PORT_ERROR_COMM_FAIL;
    }

    dwt_setrxantennadelay(16385);
    dwt_settxantennadelay(16385);

    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

    dwt_configureframefilter(DWT_FF_ENABLE_802_15_4, DWT_FF_DATA_EN);

    dwt_txconfig_t txconfig = {.PGdly = 0x34, .power = 0xFEFEFEFEUL};
    dwt_configuretxrf(&txconfig);

    if (dwt_rxenable(DWT_START_RX_IMMEDIATE) != DWT_SUCCESS)
    {
        return UWB_PORT_ERROR_COMM_FAIL;
    }

    return UWB_PORT_SUCCESS;
}

uwb_port_status_t uwb_port_send_message(uwb_dev_t* dev, const uint8_t* data, uint16_t length)
{
    if (dev == NULL || data == NULL)
    {
        return UWB_PORT_ERROR_NULL_PTR;
    }

    if (length == 0 || length > UWB_MAX_MESSAGE_LENGTH)
    {
        return UWB_PORT_ERROR_CONFIG;
    }

    dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
    dwt_writetxdata(length, (uint8_t*)data, 0);
    dwt_writetxfctrl(length + 2, 0, 0);

    dwt_forcetrxoff();

    if (dwt_starttx(DWT_START_TX_IMMEDIATE) != DWT_SUCCESS)
    {
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        return UWB_PORT_ERROR_TX_FAIL;
    }

    uint32_t status = 0;
    uint32_t timeout = 10000;
    while (timeout--)
    {
        status = dwt_readsysstatuslo();

        if (status & DWT_INT_TXFRS_BIT_MASK)
        {
            dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);

            uint8_t tx_ts_bytes[5];
            dwt_readtxtimestamp(tx_ts_bytes);

            dev->last_tx_ts = 0;
            for (int i = 0; i < 5; i++)
            {
                dev->last_tx_ts |= ((uint64_t)tx_ts_bytes[i]) << (8 * i);
            }

            dwt_rxenable(DWT_START_RX_IMMEDIATE);
            return UWB_PORT_SUCCESS;
        }
        platform_os_delay_us_blocking(10);
    }

    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    return UWB_PORT_ERROR_TIMEOUT;
}

uwb_port_status_t uwb_port_send_message_delayed(uwb_dev_t* dev, const uint8_t* data,
                                                uint16_t length, uint64_t tx_timestamp_dtuh)
{
    if (dev == NULL || data == NULL)
    {
        return UWB_PORT_ERROR_NULL_PTR;
    }

    if (length == 0 || length > UWB_MAX_MESSAGE_LENGTH)
    {
        return UWB_PORT_ERROR_CONFIG;
    }

    dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
    dwt_writetxdata(length, (uint8_t*)data, 0);
    dwt_writetxfctrl(length + 2, 0, 1);

    dwt_forcetrxoff();

    dwt_setdelayedtrxtime(tx_timestamp_dtuh);

    int tx_result = dwt_starttx(DWT_START_TX_DELAYED);

    if (tx_result != DWT_SUCCESS)
    {
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        return UWB_PORT_ERROR_TX_FAIL;
    }

    uint32_t status = 0;
    uint32_t timeout = 100000;

    while (timeout--)
    {
        status = dwt_readsysstatuslo();

        if (status & DWT_INT_TXFRS_BIT_MASK)
        {
            dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
            break;
        }

        for (volatile int i = 0; i < 100; i++)
            ;
    }

    if (!(status & DWT_INT_TXFRS_BIT_MASK))
    {
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        return UWB_PORT_ERROR_TX_FAIL;
    }

    uint8_t tx_ts_bytes[5];
    dwt_readtxtimestamp(tx_ts_bytes);

    dev->last_tx_ts = 0;
    for (int i = 0; i < 5; i++)
    {
        dev->last_tx_ts |= ((uint64_t)tx_ts_bytes[i]) << (8 * i);
    }

    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    return UWB_PORT_SUCCESS;
}

uwb_port_status_t uwb_port_receive_message(uwb_dev_t* dev, uint8_t* data, uint16_t max_length,
                                           uint16_t* received_length)
{
    if (dev == NULL || data == NULL || received_length == NULL)
    {
        return UWB_PORT_ERROR_NULL_PTR;
    }

    uint32_t status = dwt_readsysstatuslo();

    if (status & (DWT_INT_RXPHE_BIT_MASK | DWT_INT_RXFCE_BIT_MASK | DWT_INT_RXFSL_BIT_MASK |
                  DWT_INT_RXFTO_BIT_MASK | DWT_INT_RXOVRR_BIT_MASK | DWT_INT_RXPTO_BIT_MASK))
    {
        dwt_writesysstatuslo(SYS_STATUS_ALL_RX_ERR);
        dwt_forcetrxoff();
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        return UWB_PORT_ERROR_RX_FAIL;
    }

    if ((status & DWT_INT_RXFCG_BIT_MASK) == 0)
    {
        uint8_t sys_state = dwt_checkidlerc();
        if (sys_state != DWT_DW_IDLE_RC)
        {
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
        }
        return UWB_PORT_ERROR_NO_DATA;
    }

    uint8_t rng = 0;
    uint16_t frame_len = dwt_getframelength(&rng);

    if (frame_len == 0 || frame_len > max_length)
    {
        dwt_forcetrxoff();
        dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR);
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        return UWB_PORT_ERROR_RX_FAIL;
    }

    uint16_t data_len = (frame_len > 2) ? (frame_len - 2) : 0;
    dwt_readrxdata(data, data_len, 0);
    *received_length = data_len;

    uint8_t rx_ts_bytes[5];
    dwt_readrxtimestamp(rx_ts_bytes, DWT_COMPAT_NONE);

    dev->last_rx_ts = 0;
    for (int i = 0; i < 5; i++)
    {
        dev->last_rx_ts |= ((uint64_t)rx_ts_bytes[i]) << (8 * i);
    }

    dwt_forcetrxoff();
    dwt_writesysstatuslo(SYS_STATUS_ALL_RX_GOOD | SYS_STATUS_ALL_RX_ERR);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    return UWB_PORT_SUCCESS;
}

uint64_t uwb_port_read_device_time(void)
{
    uint32_t sys_time_hi32 = dwt_readsystimestamphi32();
    return (uint64_t)(sys_time_hi32);
}

uwb_port_status_t uwb_port_read_tx_timestamp(uwb_dev_t* dev, uint8_t timestamp_bytes[5])
{
    if (dev == NULL || timestamp_bytes == NULL)
    {
        return UWB_PORT_ERROR_NULL_PTR;
    }

    dwt_readtxtimestamp(timestamp_bytes);

    return UWB_PORT_SUCCESS;
}

uwb_port_status_t uwb_port_read_rx_timestamp(uwb_dev_t* dev, uint8_t timestamp_bytes[5])
{
    if (dev == NULL || timestamp_bytes == NULL)
    {
        return UWB_PORT_ERROR_NULL_PTR;
    }

    dwt_readrxtimestamp(timestamp_bytes, DWT_COMPAT_NONE);

    return UWB_PORT_SUCCESS;
}

uint64_t uwb_port_get_last_tx_timestamp(uwb_dev_t* dev)
{
    if (dev == NULL)
    {
        return 0;
    }

    return dev->last_tx_ts;
}

uint64_t uwb_port_get_last_rx_timestamp(uwb_dev_t* dev)
{
    if (dev == NULL)
    {
        return 0;
    }

    return dev->last_rx_ts;
}

/*---------------------------------------------------------------------------
 * Platform Compatibility Functions Required by Qorvo Driver
 *---------------------------------------------------------------------------*/

void deca_usleep(unsigned long time_us)
{
    platform_os_delay_us_blocking((uint32_t)time_us);
}

void deca_sleep(unsigned int time_ms)
{
    platform_os_delay_ms((uint32_t)time_ms);
}

decaIrqStatus_t decamutexon(void)
{
    return (decaIrqStatus_t)platform_os_critical_enter();
}

void decamutexoff(decaIrqStatus_t s)
{
    platform_os_critical_exit((platform_os_critical_state_t)s);
}
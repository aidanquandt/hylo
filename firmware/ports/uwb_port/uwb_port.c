/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "uwb_port.h"
#include "FreeRTOS.h"
#include "deca_device_api.h"
#include "deca_interface.h"
#include "error_handler.h"
#include "platform_gpio.h"
#include "platform_os.h"
#include "platform_spi.h"
#include "platform_timer.h"
#include "stopwatch.h"
#include "task.h"
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

typedef struct
{
    uint32_t rx_ok_count;
    uint32_t rx_timeout_count;
    uint32_t rx_error_arfe_count;
    uint32_t rx_error_overrun_count;
    uint32_t rx_error_crc_count;
    uint32_t rx_error_preamble_timeout_count;
    uint32_t rx_error_phy_header_count;
    uint32_t rx_error_sync_loss_count;
    uint32_t rx_error_frame_timeout_count;
    uint32_t rx_error_sfd_timeout_count;
    uint32_t rx_error_other_count;
    uint32_t tx_done_count;
    uint32_t send_message_count;
} uwb_port_statistics_t;

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
/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/
STATIC struct dwt_spi_s uwb_spi_fns = {
    .readfromspi       = uwb_spi_read,
    .writetospi        = uwb_spi_write,
    .writetospiwithcrc = uwb_spi_write_crc,
    .setslowrate       = uwb_spi_set_slow_rate,
    .setfastrate       = uwb_spi_set_fast_rate,
};

STATIC struct uwb_dev_s uwb_device = {.dw_chip = {
                                          .SPI                   = &uwb_spi_fns,
                                          .wakeup_device_with_io = uwb_wakeup_device_impl,
                                      }};

STATIC uwb_dev_t* current_device                    = NULL;
STATIC uwb_port_rx_callback_t rx_callback           = NULL;
STATIC uwb_port_tx_done_callback_t tx_done_callback = NULL;
STATIC uwb_port_statistics_t uwb_port_stats         = {0};

/*---------------------------------------------------------------------------
 * Private Function Implementations
 *---------------------------------------------------------------------------*/
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

STATIC void uwb_port_rx_ok_callback(const dwt_cb_data_t* cb_data)
{
    uwb_port_stats.rx_ok_count++;
    if (cb_data == NULL)
        return;

    uint16_t frame_len = cb_data->datalength;
    if (frame_len <= 2 || frame_len > UWB_MAX_MESSAGE_LENGTH)
    {
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        return;
    }

    uint16_t data_len = frame_len - 2;
    uint8_t rx_buffer[UWB_MAX_MESSAGE_LENGTH];
    dwt_readrxdata(rx_buffer, data_len, 0);

    uint64_t current_ts = 0;
    dwt_readrxtimestamp((uint8_t*)&current_ts, DWT_COMPAT_NONE);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    if (rx_callback != NULL)
    {
        rx_callback(rx_buffer, data_len, current_ts);
    }
}

STATIC void uwb_port_rx_timeout_callback(const dwt_cb_data_t* cb_data)
{
    uwb_port_stats.rx_timeout_count++;
    (void)cb_data;
    error_handler_log(ERROR_SEVERITY_WARNING, "uwb_port", "RX timeout");
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

STATIC void uwb_port_rx_error_callback(const dwt_cb_data_t* cb_data)
{
    if (cb_data->status & DWT_INT_ARFE_BIT_MASK)
    {
        uwb_port_stats.rx_error_arfe_count++;
        error_handler_log(ERROR_SEVERITY_INFO, "uwb_port", "RX Filtered: Address Mismatch (ARFE)");
    }
    else if (cb_data->status & DWT_INT_RXOVRR_BIT_MASK)
    {
        uwb_port_stats.rx_error_overrun_count++;
        error_handler_log(ERROR_SEVERITY_WARNING, "uwb_port", "RX Error: Buffer Overrun (RXOVRR)");
        dwt_forcetrxoff();
    }
    else if (cb_data->status & DWT_INT_RXFCE_BIT_MASK)
    {
        uwb_port_stats.rx_error_crc_count++;
        error_handler_log(ERROR_SEVERITY_WARNING, "uwb_port", "RX Error: CRC Error (RXFCE)");
    }
    else if (cb_data->status & DWT_INT_RXPTO_BIT_MASK)
    {
        uwb_port_stats.rx_error_preamble_timeout_count++;
        error_handler_log(ERROR_SEVERITY_INFO, "uwb_port", "RX Error: Preamble Timeout (RXPTO)");
    }
    else if (cb_data->status & DWT_INT_RXPHE_BIT_MASK)
    {
        uwb_port_stats.rx_error_phy_header_count++;
        error_handler_log(ERROR_SEVERITY_WARNING, "uwb_port", "RX Error: PHY Header Error (RXPHE)");
    }
    else if (cb_data->status & DWT_INT_RXFSL_BIT_MASK)
    {
        uwb_port_stats.rx_error_sync_loss_count++;
        error_handler_log(ERROR_SEVERITY_WARNING, "uwb_port", "RX Error: Sync Loss (RXFSL)");
    }
    else if (cb_data->status & DWT_INT_RXFTO_BIT_MASK)
    {
        uwb_port_stats.rx_error_frame_timeout_count++;
        error_handler_log(ERROR_SEVERITY_INFO, "uwb_port", "RX Error: Frame Wait Timeout (RXFTO)");
    }
    else if (cb_data->status & DWT_INT_RXSTO_BIT_MASK)
    {
        uwb_port_stats.rx_error_sfd_timeout_count++;
        error_handler_log(ERROR_SEVERITY_INFO, "uwb_port", "RX Error: SFD Timeout (RXSTO)");
    }
    else
    {
        uwb_port_stats.rx_error_other_count++;
        error_handler_log(ERROR_SEVERITY_WARNING, "uwb_port",
                          "RX Error: Unknown (Status Lo: 0x%08lX, Hi: 0x%08lX)",
                          (unsigned long)cb_data->status, (unsigned long)cb_data->status_hi);
    }

    dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

STATIC void uwb_port_tx_done_callback(const dwt_cb_data_t* cb_data)
{
    uwb_port_stats.tx_done_count++;
    (void)cb_data;

    if (current_device != NULL)
    {
        current_device->last_tx_ts = 0;
        dwt_readtxtimestamp((uint8_t*)&current_device->last_tx_ts);

        if (tx_done_callback != NULL)
        {
            tx_done_callback(current_device->last_tx_ts);
        }
    }
}

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

    struct dwt_probe_s probe_data = {.dw                    = &dev->dw_chip,
                                     .spi                   = &uwb_spi_fns,
                                     .wakeup_device_with_io = uwb_wakeup_device_impl,
                                     .driver_list           = (struct dwt_driver_s**)driver_list,
                                     .dw_driver_num         = 1};

    int ret = dwt_probe(&probe_data);
    if (ret != DWT_SUCCESS)
    {
        return UWB_PORT_ERROR_COMM_FAIL;
    }

    dwt_softreset(0);
    vTaskDelay(pdMS_TO_TICKS(3));

    uint32_t timeout = 100;
    while (!dwt_checkidlerc() && timeout--)
    {
        vTaskDelay(pdMS_TO_TICKS(10));
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
    vTaskDelay(pdMS_TO_TICKS(3));

    return UWB_PORT_SUCCESS;
}

STATIC void uwb_wakeup_device_impl(void)
{
    platform_spi_cs_low(UWB_PORT_CS_PIN);
    platform_os_delay_us_blocking(600);
    platform_spi_cs_high(UWB_PORT_CS_PIN);
    vTaskDelay(pdMS_TO_TICKS(1));
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

uwb_port_status_t uwb_port_configure(uwb_dev_t* dev)
{
    if (dev == NULL)
    {
        return UWB_PORT_ERROR_NULL_PTR;
    }

    dwt_forcetrxoff();

    dwt_writesysstatuslo(0xFFFFFFFF);
    dwt_writesysstatushi(0xFF);

    dwt_config_t config = {.chan           = (uint8_t)UWB_CHANNEL_5,
                           .txPreambLength = DWT_PLEN_128,
                           .rxPAC          = DWT_PAC8,
                           .txCode         = 9,
                           .rxCode         = 9,
                           .sfdType        = DWT_SFD_DW_8,
                           .dataRate       = DWT_BR_6M8,
                           .phrMode        = DWT_PHRMODE_STD,
                           .phrRate        = DWT_PHRRATE_STD,
                           .sfdTO          = (129 + 8 - 8),
                           .stsMode        = DWT_STS_MODE_OFF,
                           .stsLength      = DWT_STS_LEN_64,
                           .pdoaMode       = DWT_PDOA_M0};

    if (dwt_configure(&config) != DWT_SUCCESS)
    {
        return UWB_PORT_ERROR_COMM_FAIL;
    }

    dwt_setrxantennadelay(16385);
    dwt_settxantennadelay(16385);

    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

    dwt_configureframefilter(DWT_FF_ENABLE_802_15_4, DWT_FF_DATA_EN);

    dwt_txconfig_t txconfig = {.PGdly = 0x34, .power = 0xfdfdfdfdUL};
    dwt_configuretxrf(&txconfig);

    if (dwt_rxenable(DWT_START_RX_IMMEDIATE) != DWT_SUCCESS)
    {
        return UWB_PORT_ERROR_COMM_FAIL;
    }

    return UWB_PORT_SUCCESS;
}

uwb_port_status_t uwb_port_send_message(uwb_dev_t* dev, const uint8_t* data, uint16_t length)
{
    uwb_port_stats.send_message_count++;
    if (dev == NULL || data == NULL)
        return UWB_PORT_ERROR_NULL_PTR;
    if (length == 0 || length > UWB_MAX_MESSAGE_LENGTH)
        return UWB_PORT_ERROR_CONFIG;

    dwt_writetxdata(length, (uint8_t*)data, 0);
    dwt_writetxfctrl(length + 2, 0, 1);
    dwt_forcetrxoff();

    int ret = dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

    if (ret != DWT_SUCCESS)
    {
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        return UWB_PORT_ERROR_TX_FAIL;
    }

    return UWB_PORT_SUCCESS;
}

uwb_port_status_t uwb_port_send_message_delayed(uwb_dev_t* dev, const uint8_t* data,
                                                uint16_t length, uint64_t tx_timestamp_dtuh)
{
    if (dev == NULL || data == NULL)
        return UWB_PORT_ERROR_NULL_PTR;
    if (length == 0 || length > UWB_MAX_MESSAGE_LENGTH)
        return UWB_PORT_ERROR_CONFIG;

    dwt_writetxdata(length, (uint8_t*)data, 0);
    dwt_writetxfctrl(length + 2, 0, 1);
    dwt_forcetrxoff();

    dwt_setdelayedtrxtime(tx_timestamp_dtuh);

    int ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);

    if (ret != DWT_SUCCESS)
    {
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        return UWB_PORT_ERROR_TX_FAIL;
    }

    return UWB_PORT_SUCCESS;
}

uint64_t uwb_port_read_device_time(void)
{
    uint32_t sys_time_hi32 = dwt_readsystimestamphi32();
    return (uint64_t)(sys_time_hi32);
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

void uwb_port_enable_rx_interrupt(void)
{
    dwt_writesysstatuslo(DWT_INT_ALL_LO);
    dwt_writesysstatushi(DWT_INT_ALL_HI);

    dwt_setinterrupt(DWT_INT_TXFRS_BIT_MASK |      // TX Done
                         DWT_INT_RXFCG_BIT_MASK |  // RX Good
                         DWT_INT_RXFCE_BIT_MASK |  // RX Error (CRC)
                         DWT_INT_RXPTO_BIT_MASK |  // Preamble Timeout
                         DWT_INT_RXPHE_BIT_MASK |  // PHY Header Error
                         DWT_INT_RXFSL_BIT_MASK |  // Sync Loss
                         DWT_INT_RXOVRR_BIT_MASK | // RX Overrun
                         DWT_INT_ARFE_BIT_MASK |   // Address Filter
                         DWT_INT_RXFTO_BIT_MASK |  // Frame Wait Timeout
                         DWT_INT_RXSTO_BIT_MASK,   // SFD Timeout
                     0, DWT_ENABLE_INT_ONLY);
}

void uwb_port_handle_irq(void)
{
    dwt_isr();
}

void uwb_port_register_isr_callbacks(uwb_dev_t* dev)
{
    current_device = dev;

    dwt_callbacks_s callbacks = {.cbRxOk      = uwb_port_rx_ok_callback,
                                 .cbRxTo      = uwb_port_rx_timeout_callback,
                                 .cbRxErr     = uwb_port_rx_error_callback,
                                 .cbTxDone    = uwb_port_tx_done_callback,
                                 .cbSPIErr    = NULL,
                                 .cbSPIRDErr  = NULL,
                                 .cbSPIRdy    = NULL,
                                 .cbDualSPIEv = NULL,
                                 .cbFrmRdy    = NULL,
                                 .cbCiaDone   = NULL,
                                 .devErr      = NULL,
                                 .cbSysEvent  = NULL};

    dwt_setcallbacks(&callbacks);
}

void uwb_port_set_rx_callback(uwb_port_rx_callback_t callback)
{
    rx_callback = callback;
}

void uwb_port_set_tx_done_callback(uwb_port_tx_done_callback_t callback)
{
    tx_done_callback = callback;
}

void uwb_port_print_statistics(void)
{
    error_handler_log(ERROR_SEVERITY_INFO, "uwb_port", "rx_ok_count: %lu",
                      (unsigned long)uwb_port_stats.rx_ok_count);
    error_handler_log(ERROR_SEVERITY_INFO, "uwb_port", "rx_timeout_count: %lu",
                      (unsigned long)uwb_port_stats.rx_timeout_count);
    error_handler_log(ERROR_SEVERITY_INFO, "uwb_port", "rx_error_arfe_count: %lu",
                      (unsigned long)uwb_port_stats.rx_error_arfe_count);
    error_handler_log(ERROR_SEVERITY_INFO, "uwb_port", "rx_error_overrun_count: %lu",
                      (unsigned long)uwb_port_stats.rx_error_overrun_count);
    error_handler_log(ERROR_SEVERITY_INFO, "uwb_port", "rx_error_crc_count: %lu",
                      (unsigned long)uwb_port_stats.rx_error_crc_count);
    error_handler_log(ERROR_SEVERITY_INFO, "uwb_port", "rx_error_preamble_timeout_count: %lu",
                      (unsigned long)uwb_port_stats.rx_error_preamble_timeout_count);
    error_handler_log(ERROR_SEVERITY_INFO, "uwb_port", "rx_error_phy_header_count: %lu",
                      (unsigned long)uwb_port_stats.rx_error_phy_header_count);
    error_handler_log(ERROR_SEVERITY_INFO, "uwb_port", "rx_error_sync_loss_count: %lu",
                      (unsigned long)uwb_port_stats.rx_error_sync_loss_count);
    error_handler_log(ERROR_SEVERITY_INFO, "uwb_port", "rx_error_frame_timeout_count: %lu",
                      (unsigned long)uwb_port_stats.rx_error_frame_timeout_count);
    error_handler_log(ERROR_SEVERITY_INFO, "uwb_port", "rx_error_sfd_timeout_count: %lu",
                      (unsigned long)uwb_port_stats.rx_error_sfd_timeout_count);
    error_handler_log(ERROR_SEVERITY_INFO, "uwb_port", "rx_error_other_count: %lu",
                      (unsigned long)uwb_port_stats.rx_error_other_count);
    error_handler_log(ERROR_SEVERITY_INFO, "uwb_port", "tx_done_count: %lu",
                      (unsigned long)uwb_port_stats.tx_done_count);
    error_handler_log(ERROR_SEVERITY_INFO, "uwb_port", "send_message_count: %lu",
                      (unsigned long)uwb_port_stats.send_message_count);

    uint32_t irq_status = uwb_port_read_irq_status();
    uint32_t status_lo  = uwb_port_read_status_register_low();
    uint32_t status_hi  = uwb_port_read_status_register_high();

    error_handler_log(ERROR_SEVERITY_INFO, "uwb_port", "irq_status: 0x%08lX",
                      (unsigned long)irq_status);
    error_handler_log(ERROR_SEVERITY_INFO, "uwb_port", "status_lo: 0x%08lX",
                      (unsigned long)status_lo);
    error_handler_log(ERROR_SEVERITY_INFO, "uwb_port", "status_hi: 0x%08lX",
                      (unsigned long)status_hi);
}

uint32_t uwb_port_read_irq_status(void)
{
    return dwt_checkirq();
}

uint32_t uwb_port_read_status_register_low(void)
{
    decaIrqStatus_t irq_status = decamutexon();
    uint32_t status            = dwt_readsysstatuslo();
    decamutexoff(irq_status);
    return status;
}

uint32_t uwb_port_read_status_register_high(void)
{
    decaIrqStatus_t irq_status = decamutexon();
    uint32_t status            = dwt_readsysstatushi();
    decamutexoff(irq_status);
    return status;
}

void deca_usleep(unsigned long time_us)
{
    platform_os_delay_us_blocking((uint32_t)time_us);
}

void deca_sleep(unsigned int time_ms)
{
    vTaskDelay(pdMS_TO_TICKS(time_ms));
}

decaIrqStatus_t decamutexon(void)
{
    return (decaIrqStatus_t)platform_os_critical_enter();
}

void decamutexoff(decaIrqStatus_t s)
{
    platform_os_critical_exit((platform_os_critical_state_t)s);
}
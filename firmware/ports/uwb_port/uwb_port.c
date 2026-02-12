/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "uwb_port.h"
#include "FreeRTOS.h"
#include "deca_device_api.h"
#include "deca_interface.h"
#include "platform_gpio.h"
#include "platform_os.h"
#include "platform_spi.h"
#include "platform_timer.h"
#include "stopwatch.h"
#include "task.h"

/*---------------------------------------------------------------------------
 * Typedefs
 *---------------------------------------------------------------------------*/
struct uwb_dev_s
{
    dwchip_t dw_chip;
    uint64_t last_tx_ts;
};

/*---------------------------------------------------------------------------
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
extern const struct dwt_driver_s dw3000_driver;

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
 * Defines
 *---------------------------------------------------------------------------*/
#define SPI_READ_SETUP_DELAY_CYCLES (10U)    // Inter-byte delay for DW3000 SPI read setup
#define SOFT_RESET_DELAY_MS (3U)             // Delay after soft reset before polling IDLE_RC
#define IDLE_RC_POLL_INTERVAL_MS (10U)       // Polling interval for IDLE_RC status check
#define IDLE_RC_MAX_RETRIES (100U)           // Maximum retries waiting for IDLE_RC (1 second total)
#define MAC_CRC_LENGTH (2U)                  // IEEE 802.15.4 CRC length in bytes
#define WAKEUP_PULSE_DURATION_US (600U)      // CS low pulse duration to wake DW3000 from sleep
#define WAKEUP_STABILIZATION_DELAY_MS (1U)   // Delay after wakeup for clock stabilization
#define DW3000_ANTENNA_DELAY (16385U)        // Antenna delay for TX/RX (calibrated value)
#define DW3000_PREAMBLE_CODE (9U)            // Preamble code for channel 5  
#define DW3000_SFD_TIMEOUT (129U)            // SFD timeout: preamble length +  SFD length
#define DW3000_TX_PDELAY (0x34U)             // TX preamble delay (Qorvo default)
#define DW3000_TX_POWER (0xFDFDFDFDUL)       // TX power (max on all PRFs)

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

    // Required delay between header and data read per DW3000 datasheet
    for (volatile uint32_t i = 0; i < SPI_READ_SETUP_DELAY_CYCLES; i++)
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
    if (frame_len <= MAC_CRC_LENGTH || frame_len > UWB_MAX_MESSAGE_LENGTH)
    {
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        return;
    }

    // Static buffer: ISR is not reentrant, safe to reuse
    // Avoids allocating 127 bytes on ISR stack (typically only 256-512 bytes total)
    static uint8_t rx_buffer[UWB_MAX_MESSAGE_LENGTH];
    uint16_t data_len = frame_len - MAC_CRC_LENGTH;
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
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

STATIC void uwb_port_rx_error_callback(const dwt_cb_data_t* cb_data)
{
    (void)cb_data;
    // Hardware already tracks all error types in diagnostic counters
    // No need to manually increment - use dwt_readeventcounters() when stats requested
    // Per Qorvo SDK: just re-enable RX after any error
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

    // Explicit RX control: always enable RX after TX completes
    // This is the single, predictable point where RX is re-enabled after transmission
    // Simpler than relying on hardware auto-transitions (DWT_RESPONSE_EXPECTED)
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
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

    // Explicitly wake device before probe - device may be in deep sleep on first power-up
    uwb_wakeup_device_impl();

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
    vTaskDelay(pdMS_TO_TICKS(SOFT_RESET_DELAY_MS));

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
    vTaskDelay(pdMS_TO_TICKS(SOFT_RESET_DELAY_MS));

    return UWB_PORT_SUCCESS;
}

STATIC void uwb_wakeup_device_impl(void)
{
    platform_spi_cs_low(UWB_PORT_CS_PIN);
    platform_os_delay_us_blocking(WAKEUP_PULSE_DURATION_US);
    platform_spi_cs_high(UWB_PORT_CS_PIN);
    vTaskDelay(pdMS_TO_TICKS(WAKEUP_STABILIZATION_DELAY_MS));
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

    // Per DW3000 User Manual: IC must be in idle mode before calling dwt_configure()
    dwt_forcetrxoff();
    
    // Clear status registers before configuration
    dwt_writesysstatuslo(DWT_INT_ALL_LO);
    dwt_writesysstatushi(DWT_INT_ALL_HI);

    dwt_config_t config = {.chan           = (uint8_t)UWB_CHANNEL_5,
                           .txPreambLength = DWT_PLEN_128,
                           .rxPAC          = DWT_PAC8,
                           .txCode         = DW3000_PREAMBLE_CODE,
                           .rxCode         = DW3000_PREAMBLE_CODE,
                           .sfdType        = DWT_SFD_DW_8,
                           .dataRate       = DWT_BR_6M8,
                           .phrMode        = DWT_PHRMODE_STD,
                           .phrRate        = DWT_PHRRATE_STD,
                           .sfdTO          = DW3000_SFD_TIMEOUT,
                           .stsMode        = DWT_STS_MODE_OFF,
                           .stsLength      = DWT_STS_LEN_64,
                           .pdoaMode       = DWT_PDOA_M0};

    if (dwt_configure(&config) != DWT_SUCCESS)
    {
        return UWB_PORT_ERROR_COMM_FAIL;
    }

    dwt_setrxantennadelay(DW3000_ANTENNA_DELAY);
    dwt_settxantennadelay(DW3000_ANTENNA_DELAY);

    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

    dwt_configureframefilter(DWT_FF_ENABLE_802_15_4, DWT_FF_DATA_EN);

    dwt_txconfig_t txconfig = {.PGdly = DW3000_TX_PDELAY, .power = DW3000_TX_POWER};
    dwt_configuretxrf(&txconfig);

    dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

    dwt_configeventcounters(1);

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
    dwt_writetxfctrl(length + MAC_CRC_LENGTH, 0, 1);

    // Stop RX before TX - per DW3000 datasheet, must explicitly transition from RX to TX
    dwt_forcetrxoff();

    int ret = dwt_starttx(DWT_START_TX_IMMEDIATE);

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

void uwb_port_enable_rx_interrupt(void)
{
    // Clear all status bits before enabling interrupts (critical for clean startup)
    dwt_writesysstatuslo(DWT_INT_ALL_LO);
    dwt_writesysstatushi(DWT_INT_ALL_HI);

    dwt_setinterrupt(DWT_INT_RXFCG_BIT_MASK | // RX Good frames
                        SYS_STATUS_ALL_RX_ERR | // All RX error types (PHE, RSL, CRC, etc.)
                        SYS_STATUS_ALL_RX_TO | // All RX timeout types (PTO, SFD, etc.)
                        DWT_INT_TXFRS_BIT_MASK, // TX Done
                        0 , DWT_ENABLE_INT_ONLY);
}

void uwb_port_handle_irq(void)
{
    dwt_isr();
}

// Override the weak callback from platform layer
void platform_gpio_uwb_irq_callback(void)
{
    uwb_port_handle_irq();
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

uwb_port_statistics_t uwb_port_get_statistics(void)
{
    uwb_port_statistics_t stats = uwb_port_stats;
    
    // Read hardware diagnostic counters
    dwt_deviceentcnts_t hw_counters;
    dwt_readeventcounters(&hw_counters);
    
    stats.PHE   = hw_counters.PHE;
    stats.RSL   = hw_counters.RSL;
    stats.CRCG  = hw_counters.CRCG;
    stats.CRCB  = hw_counters.CRCB;
    stats.ARFE  = hw_counters.ARFE;
    stats.OVER  = hw_counters.OVER;
    stats.SFDTO = hw_counters.SFDTO;
    stats.PTO   = hw_counters.PTO;
    stats.RTO   = hw_counters.RTO;
    stats.TXF   = hw_counters.TXF;
    
    // Read live status registers
    stats.irq_status = uwb_port_read_irq_status();
    stats.status_lo  = uwb_port_read_status_register_low();
    stats.status_hi  = uwb_port_read_status_register_high();
    
    return stats;
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

void uwb_port_reset_event_counters(void)
{
    // Re-enabling event counters resets them to zero
    dwt_configeventcounters(1);
}
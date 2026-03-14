/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "uwb_driver.h"
#include "FreeRTOS.h"
#include "deca_device_api.h"
#include "deca_interface.h"
#include "gpio_driver.h"
#include "mac_802154.h"
#include "os_driver.h"
#include "queue.h"
#include "spi_driver.h"
#include "stopwatch.h"
#include "task.h"
#include "timer_driver.h"
#include <string.h>

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
STATIC void uwb_driver_irq_task(void* argument);
STATIC void uwb_driver_rx_task(void* argument);
STATIC void uwb_driver_tx_task(void* argument);

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

#define UWB_DRIVER_RX_QUEUE_LENGTH (20U)
#define UWB_DRIVER_TX_QUEUE_LENGTH (16U)
#define UWB_DRIVER_TX_TIMEOUT_MS (3U)
#define UWB_DRIVER_IRQ_TASK_STACK_SIZE (1024U)
#define UWB_DRIVER_IRQ_TASK_PRIORITY (8U)
#define UWB_DRIVER_RX_TASK_STACK_SIZE (768U)
#define UWB_DRIVER_RX_TASK_PRIORITY (6U)
#define UWB_DRIVER_TX_TASK_STACK_SIZE (512U)
#define UWB_DRIVER_TX_TASK_PRIORITY (5U)

typedef struct
{
    uint8_t data[UWB_MAX_MESSAGE_LENGTH];
    uint16_t length;
    uint64_t timestamp;
} uwb_driver_rx_event_t;

typedef struct
{
    uint8_t payload[MAC_MAX_PAYLOAD_SIZE];
    uint16_t length;
    uint16_t dest_addr;
    uint32_t message_id;
} uwb_driver_tx_item_t;

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
STATIC uwb_driver_rx_callback_t rx_callback           = NULL;
STATIC uwb_driver_tx_done_callback_t tx_done_callback = NULL;
STATIC uwb_driver_statistics_t uwb_driver_stats         = {0};

STATIC QueueHandle_t rx_queue = NULL;
STATIC QueueHandle_t tx_queue = NULL;
STATIC TaskHandle_t irq_task_handle = NULL;
STATIC TaskHandle_t tx_task_handle = NULL;
STATIC uint64_t tx_done_timestamp = 0;
STATIC uint32_t tx_next_message_id = 1;
STATIC uint32_t tx_queue_overflows = 0;
STATIC uint32_t rx_queue_overflows = 0;
STATIC uint16_t tx_pan_id = MAC_DEFAULT_PAN_ID;
STATIC uint16_t tx_my_address = 0;
STATIC uint8_t tx_sequence = 0;

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

    if (spi_driver_acquire(UWB_DRIVER_CS_PIN) != SPI_DRIVER_SUCCESS)
    {
        return DWT_ERROR;
    }

    if (spi_driver_transmit_in_session(UWB_DRIVER_CS_PIN, headerBuffer, headerLength)
        != SPI_DRIVER_SUCCESS)
    {
        spi_driver_release(UWB_DRIVER_CS_PIN);
        return DWT_ERROR;
    }

    // Required delay between header and data read per DW3000 datasheet
    for (volatile uint32_t i = 0; i < SPI_READ_SETUP_DELAY_CYCLES; i++)
        ;

    if (spi_driver_receive_in_session(UWB_DRIVER_CS_PIN, readBuffer, readLength)
        != SPI_DRIVER_SUCCESS)
    {
        spi_driver_release(UWB_DRIVER_CS_PIN);
        return DWT_ERROR;
    }

    spi_driver_release(UWB_DRIVER_CS_PIN);

    return DWT_SUCCESS;
}

STATIC int32_t uwb_spi_write(uint16_t headerLength, const uint8_t* headerBuffer,
                             uint16_t bodyLength, const uint8_t* bodyBuffer)
{
    if (headerBuffer == NULL)
    {
        return DWT_ERROR;
    }

    if (spi_driver_acquire(UWB_DRIVER_CS_PIN) != SPI_DRIVER_SUCCESS)
    {
        return DWT_ERROR;
    }

    if (spi_driver_transmit_in_session(UWB_DRIVER_CS_PIN, headerBuffer, headerLength)
        != SPI_DRIVER_SUCCESS)
    {
        spi_driver_release(UWB_DRIVER_CS_PIN);
        return DWT_ERROR;
    }

    if ((bodyLength > 0U) && (bodyBuffer != NULL))
    {
        if (spi_driver_transmit_in_session(UWB_DRIVER_CS_PIN, bodyBuffer, bodyLength)
            != SPI_DRIVER_SUCCESS)
        {
            spi_driver_release(UWB_DRIVER_CS_PIN);
            return DWT_ERROR;
        }
    }

    spi_driver_release(UWB_DRIVER_CS_PIN);

    return DWT_SUCCESS;
}

STATIC int32_t uwb_spi_write_crc(uint16_t headerLength, const uint8_t* headerBuffer,
                                 uint16_t bodyLength, const uint8_t* bodyBuffer, uint8_t crc8)
{
    if (headerBuffer == NULL)
    {
        return DWT_ERROR;
    }

    if (spi_driver_acquire(UWB_DRIVER_CS_PIN) != SPI_DRIVER_SUCCESS)
    {
        return DWT_ERROR;
    }

    if (spi_driver_transmit_in_session(UWB_DRIVER_CS_PIN, headerBuffer, headerLength)
        != SPI_DRIVER_SUCCESS)
    {
        spi_driver_release(UWB_DRIVER_CS_PIN);
        return DWT_ERROR;
    }

    if ((bodyLength > 0U) && (bodyBuffer != NULL))
    {
        if (spi_driver_transmit_in_session(UWB_DRIVER_CS_PIN, bodyBuffer, bodyLength)
            != SPI_DRIVER_SUCCESS)
        {
            spi_driver_release(UWB_DRIVER_CS_PIN);
            return DWT_ERROR;
        }
    }

    if (spi_driver_transmit_in_session(UWB_DRIVER_CS_PIN, &crc8, 1) != SPI_DRIVER_SUCCESS)
    {
        spi_driver_release(UWB_DRIVER_CS_PIN);
        return DWT_ERROR;
    }

    spi_driver_release(UWB_DRIVER_CS_PIN);

    return DWT_SUCCESS;
}

STATIC void uwb_spi_set_slow_rate(void)
{
    if (spi_driver_acquire(UWB_DRIVER_CS_PIN) == SPI_DRIVER_SUCCESS)
    {
        spi_driver_set_speed(UWB_DRIVER_CS_PIN, SPI_DRIVER_SPEED_SLOW);
        spi_driver_release(UWB_DRIVER_CS_PIN);
    }
}

STATIC void uwb_spi_set_fast_rate(void)
{
    if (spi_driver_acquire(UWB_DRIVER_CS_PIN) == SPI_DRIVER_SUCCESS)
    {
        spi_driver_set_speed(UWB_DRIVER_CS_PIN, SPI_DRIVER_SPEED_FAST);
        spi_driver_release(UWB_DRIVER_CS_PIN);
    }
}

STATIC void uwb_driver_rx_ok_callback(const dwt_cb_data_t* cb_data)
{
    uwb_driver_stats.rx_ok_count++;
    if (cb_data == NULL || rx_queue == NULL)
    {
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        return;
    }

    uint16_t frame_len = cb_data->datalength;
    if (frame_len <= MAC_CRC_LENGTH || frame_len > UWB_MAX_MESSAGE_LENGTH)
    {
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        return;
    }

    uint16_t data_len = frame_len - MAC_CRC_LENGTH;
    uwb_driver_rx_event_t event = {.length = data_len};
    dwt_readrxdata(event.data, data_len, 0);
    dwt_readrxtimestamp((uint8_t*)&event.timestamp, DWT_COMPAT_NONE);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    if (xQueueSend(rx_queue, &event, 0) != pdPASS)
    {
        rx_queue_overflows++;
    }
}

STATIC void uwb_driver_rx_timeout_callback(const dwt_cb_data_t* cb_data)
{
    uwb_driver_stats.rx_timeout_count++;
    (void)cb_data;
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

STATIC void uwb_driver_rx_error_callback(const dwt_cb_data_t* cb_data)
{
    (void)cb_data;
    // Hardware already tracks all error types in diagnostic counters
    // No need to manually increment - use dwt_readeventcounters() when stats requested
    // Per Qorvo SDK: just re-enable RX after any error
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

STATIC void uwb_driver_tx_done_callback(const dwt_cb_data_t* cb_data)
{
    uwb_driver_stats.tx_done_count++;
    (void)cb_data;

    if (current_device != NULL)
    {
        current_device->last_tx_ts = 0;
        dwt_readtxtimestamp((uint8_t*)&current_device->last_tx_ts);
        tx_done_timestamp = current_device->last_tx_ts;
        if (tx_task_handle != NULL)
        {
            xTaskNotifyGive(tx_task_handle);
        }
    }

    dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

STATIC void uwb_driver_irq_task(void* argument)
{
    (void)argument;
    for (;;)
    {
        // Block until signaled by the UWB IRQ pin GPIO callback.
        // dwt_isr() runs here in task context so SPI mutex calls are safe.
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        dwt_isr();
    }
}

STATIC void uwb_driver_rx_task(void* argument)
{
    (void)argument;
    uwb_driver_rx_event_t event;
    for (;;)
    {
        if (xQueueReceive(rx_queue, &event, portMAX_DELAY) == pdPASS && rx_callback != NULL)
        {
            rx_callback(event.data, event.length, event.timestamp);
        }
    }
}

STATIC void uwb_driver_tx_task(void* argument)
{
    (void)argument;
    uwb_driver_tx_item_t item;
    for (;;)
    {
        if (xQueueReceive(tx_queue, &item, portMAX_DELAY) != pdPASS)
        {
            continue;
        }

        uint8_t tx_buffer[MAC_MAX_FRAME_SIZE];
        mac_frame_short_t* frame = (mac_frame_short_t*)tx_buffer;
        frame->frame_control = MAC_FC_TYPE_DATA | MAC_FC_DST_ADDR_SHORT | MAC_FC_SRC_ADDR_SHORT;
        frame->sequence      = tx_sequence++;
        frame->dest_pan_id   = tx_pan_id;
        frame->dest_addr     = item.dest_addr;
        frame->src_addr      = tx_my_address;
        memcpy(frame->payload, item.payload, item.length);
        uint16_t frame_len = MAC_FRAME_SHORT_HEADER_SIZE + item.length;

        uwb_driver_stats.send_message_count++;
        uwb_driver_status_t result =
            uwb_driver_send_message(current_device, tx_buffer, frame_len);
        if (result != UWB_DRIVER_SUCCESS)
        {
            if (tx_done_callback != NULL)
            {
                tx_done_callback(item.message_id, 0);
            }
            continue;
        }

        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(UWB_DRIVER_TX_TIMEOUT_MS)) == 0)
        {
            if (tx_done_callback != NULL)
            {
                tx_done_callback(item.message_id, 0);
            }
            continue;
        }

        if (tx_done_callback != NULL)
        {
            tx_done_callback(item.message_id, tx_done_timestamp);
        }
    }
}

/*---------------------------------------------------------------------------
 * Public Function Implementations
 *---------------------------------------------------------------------------*/
uwb_dev_t* uwb_driver_init(void)
{
    return &uwb_device;
}

uwb_driver_status_t uwb_driver_probe_and_init(uwb_dev_t* dev)
{
    if (dev == NULL)
    {
        return UWB_DRIVER_ERROR_NULL_PTR;
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
        return UWB_DRIVER_ERROR_COMM_FAIL;
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
        return UWB_DRIVER_ERROR_INIT_FAIL;
    }

    ret = dwt_initialise(DWT_READ_OTP_ALL);
    if (ret != DWT_SUCCESS)
    {
        return UWB_DRIVER_ERROR_INIT_FAIL;
    }

    return UWB_DRIVER_SUCCESS;
}

void uwb_driver_wakeup_device(uwb_dev_t* dev)
{
    if (dev == NULL)
    {
        return;
    }
    uwb_wakeup_device_impl();
}

uwb_driver_status_t uwb_driver_soft_reset(uwb_dev_t* dev)
{
    if (dev == NULL)
    {
        return UWB_DRIVER_ERROR_NULL_PTR;
    }

    dwt_softreset(0);
    vTaskDelay(pdMS_TO_TICKS(SOFT_RESET_DELAY_MS));

    return UWB_DRIVER_SUCCESS;
}

STATIC void uwb_wakeup_device_impl(void)
{
    spi_driver_acquire(UWB_DRIVER_CS_PIN);
    os_driver_delay_us_blocking(WAKEUP_PULSE_DURATION_US);
    spi_driver_release(UWB_DRIVER_CS_PIN);
    vTaskDelay(pdMS_TO_TICKS(WAKEUP_STABILIZATION_DELAY_MS));
}

uwb_driver_status_t uwb_driver_check_device_id(uwb_dev_t* dev)
{
    if (dev == NULL)
    {
        return UWB_DRIVER_ERROR_NULL_PTR;
    }

    int ret = dwt_check_dev_id();
    if (ret != DWT_SUCCESS)
    {
        return UWB_DRIVER_ERROR_INVALID_ID;
    }

    return UWB_DRIVER_SUCCESS;
}

uint32_t uwb_driver_read_device_id(uwb_dev_t* dev)
{
    if (dev == NULL)
    {
        return 0;
    }

    return dwt_readdevid();
}

void uwb_driver_set_pan_id(uwb_dev_t* dev, uint16_t pan_id)
{
    if (dev == NULL)
    {
        return;
    }
    tx_pan_id = pan_id;
    dwt_setpanid(pan_id);
}

void uwb_driver_set_address(uwb_dev_t* dev, uint16_t address)
{
    if (dev == NULL)
    {
        return;
    }
    tx_my_address = address;
    dwt_setaddress16(address);
}

uwb_driver_status_t uwb_driver_configure(uwb_dev_t* dev)
{
    if (dev == NULL)
    {
        return UWB_DRIVER_ERROR_NULL_PTR;
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
        return UWB_DRIVER_ERROR_COMM_FAIL;
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
        return UWB_DRIVER_ERROR_COMM_FAIL;
    }

    return UWB_DRIVER_SUCCESS;
}

uwb_driver_status_t uwb_driver_send_message(uwb_dev_t* dev, const uint8_t* data, uint16_t length)
{
    uwb_driver_stats.send_message_count++;
    if (dev == NULL || data == NULL)
        return UWB_DRIVER_ERROR_NULL_PTR;
    if (length == 0 || length > UWB_MAX_MESSAGE_LENGTH)
        return UWB_DRIVER_ERROR_CONFIG;

    dwt_writetxdata(length, (uint8_t*)data, 0);
    dwt_writetxfctrl(length + MAC_CRC_LENGTH, 0, 1);

    // Stop RX before TX - per DW3000 datasheet, must explicitly transition from RX to TX
    dwt_forcetrxoff();

    int ret = dwt_starttx(DWT_START_TX_IMMEDIATE);

    if (ret != DWT_SUCCESS)
    {
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        return UWB_DRIVER_ERROR_TX_FAIL;
    }

    return UWB_DRIVER_SUCCESS;
}

uint64_t uwb_driver_read_device_time(void)
{
    uint32_t sys_time_hi32 = dwt_readsystimestamphi32();
    return (uint64_t)(sys_time_hi32);
}

void uwb_driver_enable_rx_interrupt(void)
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

// Called from GPIO EXTI ISR — only signals the IRQ task; no SPI access in ISR context.
void uwb_driver_handle_irq(void)
{
    if (irq_task_handle == NULL)
    {
        return;
    }
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(irq_task_handle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// Override the weak callback from platform layer
void gpio_driver_uwb_irq_callback(void)
{
    uwb_driver_handle_irq();
}

void uwb_driver_register_isr_callbacks(uwb_dev_t* dev)
{
    current_device = dev;

    if (rx_queue == NULL)
    {
        rx_queue = xQueueCreate(UWB_DRIVER_RX_QUEUE_LENGTH, sizeof(uwb_driver_rx_event_t));
        tx_queue = xQueueCreate(UWB_DRIVER_TX_QUEUE_LENGTH, sizeof(uwb_driver_tx_item_t));
        (void)xTaskCreate(uwb_driver_irq_task, "uwb_irq", UWB_DRIVER_IRQ_TASK_STACK_SIZE, NULL,
                          UWB_DRIVER_IRQ_TASK_PRIORITY, &irq_task_handle);
        (void)xTaskCreate(uwb_driver_rx_task, "uwb_rx", UWB_DRIVER_RX_TASK_STACK_SIZE, NULL,
                          UWB_DRIVER_RX_TASK_PRIORITY, NULL);
        (void)xTaskCreate(uwb_driver_tx_task, "uwb_tx", UWB_DRIVER_TX_TASK_STACK_SIZE, NULL,
                          UWB_DRIVER_TX_TASK_PRIORITY, &tx_task_handle);
    }

    dwt_callbacks_s callbacks = {.cbRxOk      = uwb_driver_rx_ok_callback,
                                 .cbRxTo      = uwb_driver_rx_timeout_callback,
                                 .cbRxErr     = uwb_driver_rx_error_callback,
                                 .cbTxDone    = uwb_driver_tx_done_callback,
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

void uwb_driver_set_rx_callback(uwb_driver_rx_callback_t callback)
{
    rx_callback = callback;
}

void uwb_driver_set_tx_done_callback(uwb_driver_tx_done_callback_t callback)
{
    tx_done_callback = callback;
}

uint32_t uwb_driver_send_async(const uint8_t* payload, uint16_t length, uint16_t dest_addr)
{
    if (tx_queue == NULL || payload == NULL || length == 0 || length > MAC_MAX_PAYLOAD_SIZE)
    {
        return 0;
    }
    uint32_t message_id;
    taskENTER_CRITICAL();
    message_id = tx_next_message_id++;
    if (tx_next_message_id == 0)
    {
        tx_next_message_id = 1;
    }
    taskEXIT_CRITICAL();

    uwb_driver_tx_item_t item = {
        .length     = length,
        .dest_addr  = dest_addr,
        .message_id = message_id,
    };
    memcpy(item.payload, payload, length);

    if (xQueueSend(tx_queue, &item, 0) != pdPASS)
    {
        tx_queue_overflows++;
        return 0;
    }
    return message_id;
}

uint32_t uwb_driver_get_tx_queue_depth(void)
{
    return (tx_queue != NULL) ? (uint32_t)uxQueueMessagesWaiting(tx_queue) : 0;
}

uint32_t uwb_driver_get_tx_queue_overflows(void)
{
    return tx_queue_overflows;
}

uint32_t uwb_driver_get_rx_queue_overflows(void)
{
    return rx_queue_overflows;
}

void uwb_driver_reset_tx_queue(void)
{
    if (tx_queue != NULL)
    {
        xQueueReset(tx_queue);
    }
}

uwb_driver_statistics_t uwb_driver_get_statistics(void)
{
    uwb_driver_statistics_t stats = uwb_driver_stats;
    
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
    stats.irq_status = uwb_driver_read_irq_status();
    stats.status_lo  = uwb_driver_read_status_register_low();
    stats.status_hi  = uwb_driver_read_status_register_high();
    
    return stats;
}

uint32_t uwb_driver_read_irq_status(void)
{
    return dwt_checkirq();
}

uint32_t uwb_driver_read_status_register_low(void)
{
    decaIrqStatus_t irq_status = decamutexon();
    uint32_t status            = dwt_readsysstatuslo();
    decamutexoff(irq_status);
    return status;
}

uint32_t uwb_driver_read_status_register_high(void)
{
    decaIrqStatus_t irq_status = decamutexon();
    uint32_t status            = dwt_readsysstatushi();
    decamutexoff(irq_status);
    return status;
}

void deca_usleep(unsigned long time_us)
{
    os_driver_delay_us_blocking((uint32_t)time_us);
}

void deca_sleep(unsigned int time_ms)
{
    vTaskDelay(pdMS_TO_TICKS(time_ms));
}

decaIrqStatus_t decamutexon(void)
{
    return (decaIrqStatus_t)os_driver_critical_enter();
}

void decamutexoff(decaIrqStatus_t s)
{
    os_driver_critical_exit((os_driver_critical_state_t)s);
}

void uwb_driver_reset_event_counters(void)
{
    // Re-enabling event counters resets them to zero
    dwt_configeventcounters(1);
}
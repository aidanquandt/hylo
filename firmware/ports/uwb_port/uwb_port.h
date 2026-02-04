#pragma once

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "common.h"
#include "deca_interface.h" // Required for decaIrqStatus_t type
#include "platform_spi.h"

/*---------------------------------------------------------------------------
 * Forward Declarations
 *---------------------------------------------------------------------------*/
typedef struct uwb_dev_s uwb_dev_t;

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define UWB_PORT_CS_PIN PLATFORM_SPI_CS_UWB
#define UWB_MAX_MESSAGE_LENGTH (127U) ///< Maximum UWB message length per IEEE 802.15.4

#define UWB_DTUH_TO_MS(dtuh_ticks) (((dtuh_ticks) * 1000ULL) / DW3000_DTU_FREQ)
#define UWB_DTUH_TO_US(dtuh_ticks) (((dtuh_ticks) * 1000000ULL) / DW3000_DTU_FREQ)
#define UWB_DTUH_TO_S(dtuh_ticks) ((dtuh_ticks) / DW3000_DTU_FREQ)
#define UWB_MS_TO_DTUH(milliseconds) (((uint64_t)(milliseconds)) * (DW3000_DTU_FREQ / 1000ULL))
#define UWB_US_TO_DTUH(microseconds) ((((uint64_t)(microseconds)) * DW3000_DTU_FREQ) / 1000000ULL)
#define UWB_DTUH_TO_DTU(dtuh_ticks) (((dtuh_ticks) << 8))
#define UWB_DTU_TO_DTUH(dtu_ticks) ((uint32_t)(((dtu_ticks) >> 8)))
#define UWB_MS_TO_DTU(milliseconds)                                                                \
    ((((uint64_t)(milliseconds)) * ((DW3000_DTU_FREQ * 128ULL) / 1000ULL)))
#define UWB_US_TO_DTU(microseconds)                                                                \
    ((((uint64_t)(microseconds)) * (DW3000_DTU_FREQ * 128ULL)) / 1000000ULL)

/*---------------------------------------------------------------------------
 * Typedefs
 *---------------------------------------------------------------------------*/
typedef enum
{
    UWB_PORT_SUCCESS          = 0,
    UWB_PORT_ERROR_NULL_PTR   = -1,
    UWB_PORT_ERROR_COMM_FAIL  = -2,
    UWB_PORT_ERROR_INVALID_ID = -3,
    UWB_PORT_ERROR_TIMEOUT    = -4,
    UWB_PORT_ERROR_INIT_FAIL  = -5,
    UWB_PORT_ERROR_CONFIG     = -6,
    UWB_PORT_ERROR_TX_FAIL    = -7,
    UWB_PORT_ERROR_RX_FAIL    = -8,
    UWB_PORT_ERROR_NO_DATA    = -9,
    UWB_PORT_ERROR_UNKNOWN    = -99
} uwb_port_status_t;

typedef enum
{
    UWB_CHANNEL_1 = 1,
    UWB_CHANNEL_2 = 2,
    UWB_CHANNEL_3 = 3,
    UWB_CHANNEL_4 = 4,
    UWB_CHANNEL_5 = 5,
    UWB_CHANNEL_7 = 7,
    UWB_CHANNEL_9 = 9
} uwb_channel_t;

/*---------------------------------------------------------------------------
 * Callback Types
 *---------------------------------------------------------------------------*/
typedef void (*uwb_port_rx_callback_t)(const uint8_t* data, uint16_t length, uint64_t timestamp);
typedef void (*uwb_port_tx_done_callback_t)(uint64_t tx_timestamp);

/*---------------------------------------------------------------------------
 * Statistics Structure
 *---------------------------------------------------------------------------*/
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
    uint32_t irq_status;
    uint32_t status_lo;
    uint32_t status_hi;
} uwb_port_statistics_t;

/*---------------------------------------------------------------------------
 * Public Function Prototypes
 *---------------------------------------------------------------------------*/
uwb_dev_t* uwb_port_init(void);
uwb_port_status_t uwb_port_probe_and_init(uwb_dev_t* dev);
void uwb_port_wakeup_device(uwb_dev_t* dev);
uwb_port_status_t uwb_port_soft_reset(uwb_dev_t* dev);
uwb_port_status_t uwb_port_check_device_id(uwb_dev_t* dev);
uint32_t uwb_port_read_device_id(uwb_dev_t* dev);
void uwb_port_set_pan_id(uwb_dev_t* dev, uint16_t pan_id);
void uwb_port_set_address(uwb_dev_t* dev, uint16_t address);
uwb_port_status_t uwb_port_configure(uwb_dev_t* dev);
uwb_port_status_t uwb_port_send_message(uwb_dev_t* dev, const uint8_t* data, uint16_t length);
uwb_port_status_t uwb_port_send_message_delayed(uwb_dev_t* dev, const uint8_t* data,
                                                uint16_t length, uint64_t tx_timestamp_dtuh);
uint64_t uwb_port_read_device_time(void);
uint64_t uwb_port_get_last_tx_timestamp(uwb_dev_t* dev);
uint64_t uwb_port_get_last_rx_timestamp(uwb_dev_t* dev);
void uwb_port_enable_rx_interrupt(void);
void uwb_port_handle_irq(void);
uint32_t uwb_port_read_irq_status(void);
uint32_t uwb_port_read_status_register_low(void);
uint32_t uwb_port_read_status_register_high(void);
void uwb_port_register_isr_callbacks(uwb_dev_t* dev);
void uwb_port_set_rx_callback(uwb_port_rx_callback_t callback);
void uwb_port_set_tx_done_callback(uwb_port_tx_done_callback_t callback);
uwb_port_statistics_t uwb_port_get_statistics(void);

/*---------------------------------------------------------------------------
 * Platform Compatibility Functions (required by Qorvo DW3000 driver)
 *
 * @note These functions are required by the vendor driver library and must
 *       use the exact names specified by Qorvo. They provide OS abstraction.
 *---------------------------------------------------------------------------*/

void deca_usleep(unsigned long time_us);
void deca_sleep(unsigned int time_ms);
decaIrqStatus_t decamutexon(void);
void decamutexoff(decaIrqStatus_t s);

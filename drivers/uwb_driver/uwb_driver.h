#pragma once

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "common.h"

/*---------------------------------------------------------------------------
 * Forward Declarations
 *---------------------------------------------------------------------------*/
typedef struct uwb_dev_s uwb_dev_t;

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define UWB_MAX_MESSAGE_LENGTH (127U)

/*---------------------------------------------------------------------------
 * Typedefs
 *---------------------------------------------------------------------------*/
typedef enum
{
    UWB_DRIVER_SUCCESS          = 0,
    UWB_DRIVER_ERROR_NULL_PTR   = -1,
    UWB_DRIVER_ERROR_COMM_FAIL  = -2,
    UWB_DRIVER_ERROR_INVALID_ID = -3,
    UWB_DRIVER_ERROR_TIMEOUT    = -4,
    UWB_DRIVER_ERROR_INIT_FAIL  = -5,
    UWB_DRIVER_ERROR_CONFIG     = -6,
    UWB_DRIVER_ERROR_TX_FAIL    = -7,
    UWB_DRIVER_ERROR_RX_FAIL    = -8,
    UWB_DRIVER_ERROR_NO_DATA    = -9,
    UWB_DRIVER_ERROR_UNKNOWN    = -99
} uwb_driver_status_t;

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

typedef void (*uwb_driver_rx_callback_t)(const uint8_t* data, uint16_t length, uint64_t timestamp);
typedef void (*uwb_driver_tx_done_callback_t)(uint32_t message_id, uint64_t tx_timestamp);

typedef struct
{
    uint32_t rx_ok_count;
    uint32_t rx_timeout_count;
    uint32_t tx_done_count;
    uint32_t send_message_count;
    uint16_t PHE;
    uint16_t RSL;
    uint16_t CRCG;
    uint16_t CRCB;
    uint8_t ARFE;
    uint8_t OVER;
    uint16_t SFDTO;
    uint16_t PTO;
    uint8_t RTO;
    uint16_t TXF;
    uint32_t irq_status;
    uint32_t status_lo;
    uint32_t status_hi;
} uwb_driver_statistics_t;

/*---------------------------------------------------------------------------
 * Public Function Prototypes
 *---------------------------------------------------------------------------*/
uwb_dev_t* uwb_driver_init(void);
uwb_driver_status_t uwb_driver_probe_and_init(uwb_dev_t* dev);
uwb_driver_status_t uwb_driver_soft_reset(uwb_dev_t* dev);
uwb_driver_status_t uwb_driver_check_device_id(uwb_dev_t* dev);
uint32_t uwb_driver_read_device_id(uwb_dev_t* dev);
void uwb_driver_set_pan_id(uwb_dev_t* dev, uint16_t pan_id);
void uwb_driver_set_address(uwb_dev_t* dev, uint16_t address);
uwb_driver_status_t uwb_driver_configure(uwb_dev_t* dev);
void uwb_driver_enable_rx_interrupt(void);
void uwb_driver_register_isr_callbacks(uwb_dev_t* dev);
void uwb_driver_set_rx_callback(uwb_driver_rx_callback_t callback);
void uwb_driver_set_tx_done_callback(uwb_driver_tx_done_callback_t callback);
uwb_driver_statistics_t uwb_driver_get_statistics(void);
/** Async send: enqueue payload for TX task; returns message_id (0 if queue full). Completion via tx_done_callback. */
uint32_t uwb_driver_send_async(const uint8_t* payload, uint16_t length, uint16_t dest_addr);
uint32_t uwb_driver_get_tx_queue_depth(void);
uint32_t uwb_driver_get_tx_queue_overflows(void);
uint32_t uwb_driver_get_rx_queue_overflows(void);
void uwb_driver_reset_tx_queue(void);

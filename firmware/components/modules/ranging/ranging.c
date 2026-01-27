/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "ranging.h"
#include "anchor/anchor.h"
#include "error_handler.h"
#include "module.h"
#include "tag/tag.h"
#include "uart_manager.h"
#include "uwb.h"
#include "uwb_protocol_messages.h"
#include <string.h>

/*---------------------------------------------------------------------------
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
STATIC void ranging_protocol_handler(const uint8_t* data, uint16_t length, uint16_t src_addr,
                                     uint64_t rx_timestamp);
STATIC void ranging_tx_done_handler(uint32_t message_id, uint64_t tx_timestamp);

/*---------------------------------------------------------------------------
 * Module Functions
 *---------------------------------------------------------------------------*/
STATIC void ranging_init(void);
STATIC void ranging_process_1kHz(void);

extern const module_S ranging_module;

const module_S ranging_module = {
    .module_name         = "ranging",
    .module_init         = ranging_init,
    .module_process_1kHz = ranging_process_1kHz,
};

/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/
STATIC ranging_mode_e current_mode = RANGING_MODE_DISABLED;
STATIC bool module_initialized     = false;

/*---------------------------------------------------------------------------
 * Private Function Implementations
 *---------------------------------------------------------------------------*/
STATIC void ranging_init(void)
{
    // Initialize both tag and anchor (minimal overhead when inactive)
    tag_init();
    anchor_init();

    // Register TWR protocol handler with UWB module
    if (!uwb_register_protocol_handler(PROTOCOL_TYPE_TWR, ranging_protocol_handler))
    {
        error_handler_log(ERROR_SEVERITY_ERROR, "ranging",
                          "Failed to register TWR protocol handler");
    }

    // Register TX done handler
    uwb_register_tx_done_handler(ranging_tx_done_handler);

    current_mode       = RANGING_MODE_DISABLED;
    module_initialized = true;
}

STATIC void ranging_process_1kHz(void)
{
    if (!module_initialized)
    {
        return;
    }

    switch (current_mode)
    {
        case RANGING_MODE_TAG:
            tag_process_1kHz();
            break;

        case RANGING_MODE_ANCHOR:
            anchor_process_1kHz();
            break;

        default:
            break;
    }
}

/*---------------------------------------------------------------------------
 * Public Function Implementations
 *---------------------------------------------------------------------------*/

bool ranging_set_mode(ranging_mode_e mode)
{
    if (!module_initialized)
    {
        return false;
    }

    // Check if already in requested mode
    if (mode == current_mode)
    {
        return true;
    }

    // Stop current mode
    switch (current_mode)
    {
        case RANGING_MODE_TAG:
            tag_stop();
            break;

        case RANGING_MODE_ANCHOR:
            anchor_stop();
            break;

        default:
            break;
    }

    // Update mode
    current_mode = mode;

    // Start new mode
    bool success = true;
    switch (mode)
    {
        case RANGING_MODE_TAG:
            success = tag_start();
            if (!success)
            {
                error_handler_log(ERROR_SEVERITY_ERROR, "ranging", "Failed to start TAG mode");
                current_mode = RANGING_MODE_DISABLED;
            }
            break;

        case RANGING_MODE_ANCHOR:
            success = anchor_start();
            if (!success)
            {
                error_handler_log(ERROR_SEVERITY_ERROR, "ranging", "Failed to start ANCHOR mode");
                current_mode = RANGING_MODE_DISABLED;
            }
            break;

        case RANGING_MODE_DISABLED:
            // Nothing to start
            break;

        default:
            success = false;
            break;
    }

    return success;
}

ranging_mode_e ranging_get_mode(void)
{
    return current_mode;
}

void ranging_get_status(ranging_status_t* status)
{
    if (status == NULL)
    {
        return;
    }

    memset(status, 0, sizeof(ranging_status_t));

    status->mode   = current_mode;
    status->active = (current_mode != RANGING_MODE_DISABLED);

    // Get mode-specific statistics
    switch (current_mode)
    {
        case RANGING_MODE_TAG:
        {
            tag_status_t tag_status;
            tag_get_status(&tag_status);
            status->state              = (int)tag_status.state;
            status->successful_ranges  = tag_status.successful_ranges;
            status->failed_ranges      = tag_status.failed_ranges;
            status->messages_processed = tag_status.successful_ranges + tag_status.failed_ranges;
            break;
        }

        case RANGING_MODE_ANCHOR:
        {
            anchor_status_t anchor_status;
            anchor_get_status(&anchor_status);
            status->state              = (int)anchor_status.state;
            status->successful_ranges  = anchor_status.responses_sent;
            status->failed_ranges      = anchor_status.response_failures;
            status->messages_processed = anchor_status.polls_received;
            break;
        }

        default:
            break;
    }
}

/*---------------------------------------------------------------------------
 * Tag-Specific Function Implementations
 *---------------------------------------------------------------------------*/

bool ranging_tag_start(uint16_t anchor_addr)
{
    if (current_mode != RANGING_MODE_TAG)
    {
        return false;
    }

    return tag_start_ranging(anchor_addr);
}

bool ranging_tag_is_active(void)
{
    if (current_mode != RANGING_MODE_TAG)
    {
        return false;
    }

    return tag_is_ranging();
}

bool ranging_tag_get_result(float* distance_m, float* rssi_dbm)
{
    if (current_mode != RANGING_MODE_TAG || distance_m == NULL)
    {
        return false;
    }

    twr_result_t result;
    if (!tag_get_last_result(&result))
    {
        return false;
    }

    *distance_m = result.distance_m;

    if (rssi_dbm != NULL)
    {
        *rssi_dbm = result.rssi_dbm;
    }

    return true;
}

void ranging_tag_cancel(void)
{
    if (current_mode != RANGING_MODE_TAG)
    {
        return;
    }

    tag_cancel_ranging();
}

STATIC void ranging_protocol_handler(const uint8_t* data, uint16_t length, uint16_t src_addr,
                                     uint64_t rx_timestamp)
{
    if (length < sizeof(protocol_header_t))
    {
        error_handler_log(ERROR_SEVERITY_WARNING, "ranging", "TWR message too short");
        return;
    }

    const protocol_header_t* hdr = (const protocol_header_t*)data;

    switch (hdr->msg_type)
    {
        case TWR_MSG_TYPE_POLL:
            if (current_mode == RANGING_MODE_ANCHOR)
            {
                anchor_rx_callback(data, length, src_addr, rx_timestamp);
            }
            else
            {
                error_handler_log(ERROR_SEVERITY_INFO, "ranging",
                                  "TWR POLL rejected - not anchor mode");
            }
            break;

        case TWR_MSG_TYPE_RESPONSE:
            if (current_mode == RANGING_MODE_TAG)
            {
                tag_rx_callback(data, length, src_addr, rx_timestamp);
            }
            else
            {
                error_handler_log(ERROR_SEVERITY_INFO, "ranging",
                                  "TWR RESPONSE rejected - not tag mode");
            }
            break;

        case TWR_MSG_TYPE_FINAL:
            if (current_mode == RANGING_MODE_ANCHOR)
            {
                anchor_rx_callback(data, length, src_addr, rx_timestamp);
            }
            else
            {
                error_handler_log(ERROR_SEVERITY_INFO, "ranging",
                                  "TWR FINAL rejected - not anchor mode");
            }
            break;

        case TWR_MSG_TYPE_FINAL_ACK:
            if (current_mode == RANGING_MODE_TAG)
            {
                tag_rx_callback(data, length, src_addr, rx_timestamp);
            }
            else
            {
                error_handler_log(ERROR_SEVERITY_INFO, "ranging",
                                  "TWR FINAL_ACK rejected - not tag mode");
            }
            break;

        default:
            error_handler_log(ERROR_SEVERITY_WARNING, "ranging", "TWR unknown message type: 0x%02X",
                              hdr->msg_type);
            break;
    }
}

STATIC void ranging_tx_done_handler(uint32_t message_id, uint64_t tx_timestamp)
{
    // Forward TX done notification to active mode
    switch (current_mode)
    {
        case RANGING_MODE_TAG:
            tag_tx_done_callback(message_id, tx_timestamp);
            break;

        case RANGING_MODE_ANCHOR:
            anchor_tx_done_callback(message_id, tx_timestamp);
            break;

        default:
            break;
    }
}

void ranging_anchor_set_address(uint16_t address)
{
    anchor_set_address(address);
}

uint16_t ranging_anchor_get_address(void)
{
    return anchor_get_address();
}

uint32_t ranging_anchor_get_response_count(void)
{
    if (current_mode != RANGING_MODE_ANCHOR)
    {
        return 0;
    }

    return anchor_get_response_count();
}

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "twr.h"
#include "anchor/anchor.h"
#include "error_handler.h"
#include "feature_config.h"
#include "module.h"
#include "tag/tag.h"
#include "twr/twr_mode.h"          // Simple mode manager
#include "twr/twr_state_machine.h" // For twr_process
#include "uwb.h"
#include "uwb_protocol_messages.h"
#include <string.h>

/*---------------------------------------------------------------------------
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
STATIC void twr_protocol_handler(const uint8_t* data, uint16_t length, uint16_t src_addr,
                                 uint64_t rx_timestamp);
STATIC void twr_tx_done_handler(uint32_t message_id, uint64_t tx_timestamp);
STATIC void twr_module_init(void);
STATIC void twr_process_1kHz(void);

/*---------------------------------------------------------------------------
 * Module Functions
 *---------------------------------------------------------------------------*/
extern const module_S twr_module;

const module_S twr_module = {
    .module_name         = "twr",
    .module_init         = twr_module_init,
    .module_process_1kHz = twr_process_1kHz,
};

/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/
STATIC bool module_initialized = false;
extern twr_context_t anchor_twr_ctx;
extern twr_context_t tag_twr_ctx;

/*---------------------------------------------------------------------------
 * Private Function Implementations
 *---------------------------------------------------------------------------*/
STATIC void twr_module_init(void)
{
    twr_mode_init();

    if (!uwb_register_protocol_handler(PROTOCOL_TYPE_TWR, twr_protocol_handler))
    {
        error_handler_log(ERROR_SEVERITY_ERROR, "twr", "Failed to register TWR protocol handler");
    }

    uwb_register_tx_done_handler(twr_tx_done_handler);
    module_initialized = true;

    // Note: Auto-anchor startup is deferred to twr_process_1kHz() to ensure UWB is ready
}

STATIC void twr_process_1kHz(void)
{
#if FEATURE_AUTO_START_ANCHOR_MODE
    STATIC bool auto_start_completed = false;
    if (!auto_start_completed && uwb_is_ready())
    {
        error_handler_log(ERROR_SEVERITY_INFO, "twr",
                          "Auto-starting anchor mode with address 0x0001");

        // Set UWB address to 0x0001
        uwb_set_address(0x0001, 0xDECA);

        // Set anchor address
        anchor_set_address(0x0001);

        // Request anchor mode
        if (!twr_mode_request(TWR_MODE_ANCHOR, "auto_start"))
        {
            error_handler_log(ERROR_SEVERITY_ERROR, "twr", "Failed to request anchor mode");
        }

        auto_start_completed = true;
    }
#endif

    twr_mode_process();

    twr_mode_e current_mode = twr_mode_get_current();
    switch (current_mode)
    {
        case TWR_MODE_TAG:
            twr_state_machine_process(&tag_twr_ctx);
            break;
        case TWR_MODE_ANCHOR:
            twr_state_machine_process(&anchor_twr_ctx);
            break;
        default:
            // No processing needed for disabled mode
            break;
    }
}

STATIC void twr_protocol_handler(const uint8_t* data, uint16_t length, uint16_t src_addr,
                                 uint64_t rx_timestamp)
{
    if (length < sizeof(protocol_header_t))
    {
        return;
    }

    twr_mode_e current_mode = twr_mode_get_current();

    switch (current_mode)
    {
        case TWR_MODE_ANCHOR:
            twr_rx_callback(&anchor_twr_ctx, data, length, src_addr, rx_timestamp);
            break;

        case TWR_MODE_TAG:
            twr_rx_callback(&tag_twr_ctx, data, length, src_addr, rx_timestamp);
            break;

        default:
            // Ignore if not in an active TWR mode
            break;
    }
}

STATIC void twr_tx_done_handler(uint32_t message_id, uint64_t tx_timestamp)
{
    twr_mode_e current_mode = twr_mode_get_current();

    switch (current_mode)
    {
        case TWR_MODE_ANCHOR:
            twr_tx_done_callback(&anchor_twr_ctx, message_id, tx_timestamp);
            break;

        case TWR_MODE_TAG:
            twr_tx_done_callback(&tag_twr_ctx, message_id, tx_timestamp);
            break;

        default:
            // Ignore if not in an active TWR mode
            break;
    }
}

/*---------------------------------------------------------------------------
 * Public Function Implementations
 *---------------------------------------------------------------------------*/

void twr_rx_callback(twr_context_t* ctx, const uint8_t* data, uint16_t length, uint16_t src_addr,
                     uint64_t rx_timestamp)
{
    if (ctx == NULL)
        return;

    twr_event_t event = {
        .type = TWR_EVENT_RX_MESSAGE,
        .rx = {.data = data, .length = length, .src_addr = src_addr, .rx_timestamp = rx_timestamp}};

    twr_state_machine_handle_event(ctx, &event);
}

void twr_tx_done_callback(twr_context_t* ctx, uint32_t message_id, uint64_t tx_timestamp)
{
    if (ctx == NULL)
        return;

    twr_event_t event = {
        .type = TWR_EVENT_TX_COMPLETE,
        .tx   = {
              .message_id   = message_id,
              .tx_timestamp = tx_timestamp,
              .msg_type     = TWR_MSG_POLL // Placeholder - actual type determined in callback
        }};

    twr_state_machine_handle_event(ctx, &event);
}

bool twr_start(twr_role_e role, uint16_t peer_addr)
{
    if (!module_initialized)
    {
        error_handler_log(ERROR_SEVERITY_ERROR, "twr", "Module not initialized");
        return false;
    }

    twr_mode_e target_mode = (role == TWR_ROLE_TAG) ? TWR_MODE_TAG : TWR_MODE_ANCHOR;

    if (!twr_mode_request(target_mode, "twr_start"))
    {
        error_handler_log(ERROR_SEVERITY_ERROR, "twr", "Failed to request TWR mode");
        return false;
    }

    twr_context_t* ctx = (role == TWR_ROLE_TAG) ? &tag_twr_ctx : &anchor_twr_ctx;
    return twr_state_machine_start(ctx, peer_addr);
}

void twr_stop(void)
{
    twr_state_machine_stop(&tag_twr_ctx);
    twr_state_machine_stop(&anchor_twr_ctx);
    twr_mode_request(TWR_MODE_DISABLED, "twr_stop");
}

bool twr_is_active(const twr_context_t* ctx)
{
    return (ctx != NULL) && (twr_state_machine_get_state(ctx) != TWR_STATE_IDLE);
}

void twr_cancel(twr_context_t* ctx)
{
    if (ctx != NULL)
    {
        twr_state_machine_stop(ctx);
    }
}

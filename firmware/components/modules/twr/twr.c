/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "twr.h"
#include "anchor/anchor.h"
#include "error_handler.h"
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

// External contexts - defined in anchor.c and tag.c
extern twr_context_t anchor_twr_ctx;
extern twr_context_t tag_twr_ctx;

/*---------------------------------------------------------------------------
 * Private Function Implementations
 *---------------------------------------------------------------------------*/
STATIC void twr_module_init(void)
{
    // Initialize TWR mode manager
    twr_mode_init();

    // Register as TWR protocol dispatcher
    if (!uwb_register_protocol_handler(PROTOCOL_TYPE_TWR, twr_protocol_handler))
    {
        error_handler_log(ERROR_SEVERITY_ERROR, "twr", "Failed to register TWR protocol handler");
    }

    uwb_register_tx_done_handler(twr_tx_done_handler);
    module_initialized = true;
}

STATIC void twr_process_1kHz(void)
{
    // Handle mode transitions
    twr_mode_process();

    // Process the active TWR state machine based on current mode
    twr_mode_e current_mode = twr_mode_get_current();
    switch (current_mode)
    {
        case TWR_MODE_TAG:
            twr_process(&tag_twr_ctx);
            break;
        case TWR_MODE_ANCHOR:
            twr_process(&anchor_twr_ctx);
            break;
        default:
            // No processing needed for disabled mode
            break;
    }
}

/*---------------------------------------------------------------------------
 * Public Function Implementations
 *---------------------------------------------------------------------------*/

STATIC void twr_protocol_handler(const uint8_t* data, uint16_t length, uint16_t src_addr,
                                 uint64_t rx_timestamp)
{
    if (length < sizeof(protocol_header_t))
    {
        return; // Silently ignore malformed messages
    }

    twr_mode_e current_mode = twr_mode_get_current();

    // Pure mode-based dispatch - let the state machine validate message appropriateness
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

    // Dispatch to the appropriate context based on current mode
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

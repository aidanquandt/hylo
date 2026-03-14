/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "twr.h"
#include "FreeRTOS.h"
#include "common.h"
#include "feature_config.h"
#include "protocol_tx.h"
#include "protocol.pb.h"
#include "system_halt.h"
#include "task.h"
#include "task_config.h"
#include "twr_state_machine.h" // For twr_process
#include "initiator.h"
#include "responder.h"
#include "uwb.h"
#include "uwb_node.h"
#include "uwb_protocol_messages.h"

/*---------------------------------------------------------------------------
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
STATIC void twr_protocol_handler(const uint8_t* data, uint16_t length, uint16_t src_addr,
                                 uint64_t rx_timestamp);
STATIC void twr_tx_complete_handler(uint32_t message_id, uint64_t tx_timestamp);
STATIC void twr_process_1kHz(void);
STATIC void twr_1kHz_task(void* pvParameters);

/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/
STATIC bool module_initialized = false;
extern twr_context_t responder_twr_ctx;
extern twr_context_t initiator_twr_ctx;

/*---------------------------------------------------------------------------
 * Private Function Implementations
 *---------------------------------------------------------------------------*/
STATIC void twr_1kHz_task(void* pvParameters)
{
    (void)pvParameters;
    TickType_t lastWake = xTaskGetTickCount();
    for (;;)
    {
        twr_process_1kHz();
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(1));
    }
}

void twr_init(void)
{
    // Initialize both initiator and responder - all nodes can do both
    initiator_init();
    responder_init();

    if (!uwb_register_protocol_handler(PROTOCOL_TYPE_TWR, twr_protocol_handler))
    {
        TwrFailedToRegisterProtocolHandlerEvent ev = TwrFailedToRegisterProtocolHandlerEvent_init_zero;
        protocol_tx_TwrFailedToRegisterProtocolHandlerEvent(&ev);
    }

    uwb_register_tx_complete_handler(twr_tx_complete_handler);
    module_initialized = true;

    // Note: Auto-start is deferred to twr_process_1kHz() to ensure UWB is ready

    BaseType_t result = xTaskCreate(twr_1kHz_task, "twr_1khz", TASK_STACK_4KB,
                                    NULL, TASK_PRIORITY_TWR, NULL);
    if (result != pdPASS)
    {
        system_halt("twr", "Failed to create 1kHz task");
    }
}

STATIC void twr_process_1kHz(void)
{
    STATIC bool auto_start_completed = false;
    if (!auto_start_completed && uwb_is_ready())
    {
        // Start responder listening
        if (!responder_start())
        {
            TwrResponderAutoStartFailedEvent ev = TwrResponderAutoStartFailedEvent_init_zero;
            protocol_tx_TwrResponderAutoStartFailedEvent(&ev);
        }
        auto_start_completed = true;
    }

    // Always process both state machines - nodes can do both roles
    twr_state_machine_process(&initiator_twr_ctx);
    twr_state_machine_process(&responder_twr_ctx);
}

STATIC void twr_protocol_handler(const uint8_t* data, uint16_t length, uint16_t src_addr,
                                 uint64_t rx_timestamp)
{
    if (length < sizeof(protocol_header_t))
    {
        return;
    }

    const protocol_header_t* header = (const protocol_header_t*)data;
    twr_msg_type_e msg_type         = (twr_msg_type_e)header->msg_type;

    // Dispatch based on message type:
    // POLL, FINAL -> responder handles them (incoming messages from initiator)
    // RESPONSE, FINAL_ACK -> initiator handles them (incoming messages from responder)
    if (msg_type == TWR_MSG_POLL || msg_type == TWR_MSG_FINAL)
    {
        twr_rx_callback(&responder_twr_ctx, data, length, src_addr, rx_timestamp);
    }
    else // TWR_MSG_RESPONSE or TWR_MSG_FINAL_ACK
    {
        twr_rx_callback(&initiator_twr_ctx, data, length, src_addr, rx_timestamp);
    }
}

STATIC void twr_tx_complete_handler(uint32_t message_id, uint64_t tx_timestamp)
{
    // Check which state machine is expecting this TX completion
    if (responder_twr_ctx.pending_tx_id == message_id)
    {
        twr_tx_complete_callback(&responder_twr_ctx, message_id, tx_timestamp);
    }
    else if (initiator_twr_ctx.pending_tx_id == message_id)
    {
        twr_tx_complete_callback(&initiator_twr_ctx, message_id, tx_timestamp);
    }
    // Else: stale/unexpected TX completion, ignore
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

void twr_tx_complete_callback(twr_context_t* ctx, uint32_t message_id, uint64_t tx_timestamp)
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

bool twr_start_ranging(uint16_t peer_addr)
{
    if (!module_initialized)
    {
        TwrModuleNotInitializedEvent ev = TwrModuleNotInitializedEvent_init_zero;
        protocol_tx_TwrModuleNotInitializedEvent(&ev);
        return false;
    }

    // Manual starts are always initiator (responder auto-starts)
    return twr_state_machine_start(&initiator_twr_ctx, peer_addr);
}

void twr_stop(void)
{
    twr_state_machine_stop(&initiator_twr_ctx);
    twr_state_machine_stop(&responder_twr_ctx);
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

/*---------------------------------------------------------------------------
 * @file    uwb_protocol_router.c
 * @brief   UWB message router/dispatcher implementation
 *---------------------------------------------------------------------------*/

#include "uwb_protocol_router.h"
#include <string.h>

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/

#define MAX_PROTOCOL_HANDLERS 8U

/*---------------------------------------------------------------------------
 * Types
 *---------------------------------------------------------------------------*/

typedef struct
{
    uint8_t protocol_type;
    protocol_handler_t handler;
} protocol_handler_entry_t;

typedef struct
{
    uint32_t total_received;
    uint32_t unhandled;
    uint32_t invalid;
} router_stats_t;

/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/

STATIC protocol_handler_entry_t handlers[MAX_PROTOCOL_HANDLERS];
STATIC uint8_t handler_count = 0;
STATIC router_stats_t stats = {0};
STATIC bool router_initialized = false;

/*---------------------------------------------------------------------------
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/

// None - protocol_router_rx_callback is now public

/*---------------------------------------------------------------------------
 * Public Function Implementations
 *---------------------------------------------------------------------------*/

void uwb_protocol_router_init(void)
{
    if (router_initialized)
    {
        return;
    }

    // Clear handler table
    memset(handlers, 0, sizeof(handlers));
    handler_count = 0;

    // Clear statistics
    memset(&stats, 0, sizeof(stats));

    router_initialized = true;
}

bool uwb_protocol_router_register_handler(uint8_t protocol_type, protocol_handler_t handler)
{
    if (!router_initialized || handler == NULL)
    {
        return false;
    }

    // Check if already registered
    for (uint8_t i = 0; i < handler_count; i++)
    {
        if (handlers[i].protocol_type == protocol_type)
        {
            // Update existing handler
            handlers[i].handler = handler;
            return true;
        }
    }

    // Add new handler
    if (handler_count >= MAX_PROTOCOL_HANDLERS)
    {
        // Handler table full
        return false;
    }

    handlers[handler_count].protocol_type = protocol_type;
    handlers[handler_count].handler = handler;
    handler_count++;

    return true;
}

void uwb_protocol_router_unregister_handler(uint8_t protocol_type)
{
    if (!router_initialized)
    {
        return;
    }

    // Find and remove handler
    for (uint8_t i = 0; i < handler_count; i++)
    {
        if (handlers[i].protocol_type == protocol_type)
        {
            // Shift remaining handlers down
            for (uint8_t j = i; j < handler_count - 1; j++)
            {
                handlers[j] = handlers[j + 1];
            }
            handler_count--;
            handlers[handler_count].protocol_type = 0;
            handlers[handler_count].handler = NULL;
            return;
        }
    }
}

void uwb_protocol_router_get_stats(uint32_t* total_received, uint32_t* unhandled, uint32_t* invalid)
{
    if (total_received != NULL)
    {
        *total_received = stats.total_received;
    }
    if (unhandled != NULL)
    {
        *unhandled = stats.unhandled;
    }
    if (invalid != NULL)
    {
        *invalid = stats.invalid;
    }
}

void uwb_protocol_router_reset_stats(void)
{
    memset(&stats, 0, sizeof(stats));
}

/*---------------------------------------------------------------------------
 * Public Function Implementations - RX Callback
 *---------------------------------------------------------------------------*/

void uwb_protocol_router_rx_callback(const uint8_t* data, uint16_t length, uint16_t src_addr)
{
    stats.total_received++;

    // Validate minimum message size
    if (data == NULL || length < PROTOCOL_MIN_MESSAGE_SIZE)
    {
        stats.invalid++;
        return;
    }

    // Extract protocol type from header
    uint8_t protocol_type = uwb_protocol_get_type(data, length);

    // DEBUG: Print received message info
    extern void uart_manager_print(const char* format, ...);

    // Validate protocol type
    if (!uwb_protocol_is_valid_type(protocol_type))
    {
        uart_manager_print("Router: Invalid proto_type=0x%02X\r\n", protocol_type);
        stats.invalid++;
        return;
    }

    // Find and call handler
    bool handled = false;
    for (uint8_t i = 0; i < handler_count; i++)
    {
        if (handlers[i].protocol_type == protocol_type)
        {
            handlers[i].handler(data, length, src_addr);
            handled = true;
            break;
        }
    }

    if (!handled)
    {
        uart_manager_print("Router: No handler found for proto_type=0x%02X\r\n", protocol_type);
        stats.unhandled++;
    }
}

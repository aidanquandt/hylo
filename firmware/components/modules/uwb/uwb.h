/*---------------------------------------------------------------------------
 * @file    uwb.h
 * @brief   UWB radio hardware communication module
 *
 * @note    All nodes are configured identically - they can both transmit and
 *          receive messages. The receiver is always enabled, and any node can
 *          transmit when needed.
 *---------------------------------------------------------------------------*/
#pragma once

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "common.h"

/*---------------------------------------------------------------------------
 * Typedefs
 *---------------------------------------------------------------------------*/

/**
 * @brief UWB state
 */
typedef enum
{
    UWB_STATE_OFF,            ///< Radio off
    UWB_STATE_INITIALIZATION, ///< Initializing hardware
    UWB_STATE_ACTIVE,         ///< Active and ready
    UWB_STATE_FAULTED         ///< Error state
} uwb_state_e;

/**
 * @brief UWB status information
 */
typedef struct
{
    uwb_state_e state;   ///< Current state
    uint32_t device_id;  ///< Device ID
    float temperature;   ///< Temperature (degrees C)
    float voltage;       ///< Voltage (V)
    uint32_t fault_code; ///< Fault code (0 = no fault)
    uint16_t my_address; ///< Our 802.15.4 address
    uint16_t my_pan_id;  ///< Our PAN ID
} uwb_status_t;

/**
 * @brief UWB RX statistics
 */
typedef struct
{
    uint32_t received;  ///< Valid frames received
    uint32_t rx_errors; ///< RX errors
    uint32_t filtered;  ///< Frames filtered (wrong addr/PAN)
} uwb_rx_stats_t;

/**
 * @brief RX callback function type
 *
 * Called when a valid UWB message is received (at 100Hz polling rate).
 * The callback is invoked from the UWB module's 100Hz processing context.
 *
 * @param data Pointer to received payload data (MAC header already stripped)
 * @param length Length of payload in bytes
 * @param src_addr Source address of the sender (16-bit 802.15.4 address)
 *
 * @note The data pointer is only valid during the callback - copy if needed
 * @note This is called at 100Hz rate - keep processing brief
 */
typedef void (*uwb_rx_callback_t)(const uint8_t* data, uint16_t length, uint16_t src_addr);

/*---------------------------------------------------------------------------
 * Public Function Prototypes
 *---------------------------------------------------------------------------*/

/**
 * @brief Get current UWB status
 * @param status Output: status information
 */
void uwb_get_status(uwb_status_t* status);

/**
 * @brief Get UWB RX statistics
 * @param stats Output: RX statistics
 */
void uwb_get_rx_stats(uwb_rx_stats_t* stats);

/**
 * @brief Reset RX statistics to zero
 */
void uwb_reset_rx_stats(void);

/**
 * @brief Check if hardware is ready
 * @return true if UWB radio is initialized and ready
 */
bool uwb_is_ready(void);

/**
 * @brief Request UWB radio to start
 * @note This is asynchronous - radio will initialize and become active
 * @note Check uwb_is_ready() to verify when radio is active
 */
void uwb_start(void);

/**
 * @brief Request UWB radio to stop
 */
void uwb_stop(void);

/**
 * @brief Set this device's 802.15.4 address and PAN ID
 * @param address 16-bit short address for this device
 * @param pan_id 16-bit PAN identifier (default 0xDECA)
 */
void uwb_set_address(uint16_t address, uint16_t pan_id);

/**
 * @brief Perform a soft reset on the UWB device
 *
 * Performs a software reset of the UWB chip. This should only be called
 * if the device has been initialized and is ready.
 *
 * @return true if reset was successful, false if device is not ready or reset failed
 */
bool uwb_soft_reset(void);

/**
 * @brief Send a message via UWB
 *
 * Sends arbitrary data to a destination node using 802.15.4 framing.
 * This is the main API for applications to transmit messages.
 *
 * @param data Pointer to payload data to send
 * @param length Length of payload in bytes (max MAC_MAX_PAYLOAD_SIZE)
 * @param dest_addr Destination 16-bit address (use MAC_BROADCAST_ADDR for broadcast)
 *
 * @return true if message sent successfully, false if:
 *         - UWB not ready/active
 *         - Invalid parameters (NULL pointer, length too large/small)
 *         - Transmission failed
 *
 * @note This function blocks until transmission completes (~1ms)
 * @note 802.15.4 header is added automatically (source addr, PAN ID, etc.)
 * @note Application modules should call this to send messages
 */
bool uwb_send_message(const uint8_t* data, uint16_t length, uint16_t dest_addr);

/**
 * @brief Register a callback for received UWB messages
 *
 * Applications register a callback to be notified when UWB messages are received.
 * Only one callback can be registered at a time (last registration wins).
 *
 * @param callback Function to call when message received, or NULL to unregister
 *
 * @note Callback is invoked at 100Hz polling rate when messages arrive
 * @note Keep callback processing brief - copy data if needed for later processing
 * @note Payload has MAC header stripped - only application data is passed
 *
 * @example
 * void my_rx_handler(const uint8_t* data, uint16_t len, uint16_t src) {
 *     // Process received message
 *     printf("Got %u bytes from 0x%04X\n", len, src);
 * }
 * uwb_register_rx_callback(my_rx_handler);
 */
void uwb_register_rx_callback(uwb_rx_callback_t callback);

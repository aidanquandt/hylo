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
#include "uwb_port.h"

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
 * @brief Get current UWB address
 * @return Current node address
 */
uint16_t uwb_get_address(void);

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
 * @brief Send message with delayed transmission (for TWR ranging)
 *
 * Used for ranging protocols (DS-TWR) where TX timestamp must be known before sending.
 * Accepts an absolute 40-bit device timestamp for precise timing control.
 *
 * @param data Pointer to payload data (may be modified to update timestamps)
 * @param length Length of payload in bytes (max MAC_MAX_PAYLOAD_SIZE)
 * @param dest_addr Destination 16-bit address
 * @param tx_timestamp_dtuh ABSOLUTE transmission time in Device Time Units (DTU)
 *
 * @return true if message sent successfully, false if transmission failed
 *
 * @pre tx_timestamp_dtuh must be at least 500µs in the future from current device time
 * @note Used for ranging responses where TX time is calculated from RX timestamp
 */
bool uwb_send_message_delayed(const uint8_t* data, uint16_t length, uint16_t dest_addr,
                              uint64_t tx_timestamp_dtuh);

/**
 * @brief Protocol handler callback function type
 *
 * Protocol handlers receive application-layer messages dispatched by protocol type.
 * Each handler processes messages for a specific protocol (TWR, DATA, etc.).
 *
 * @param data Pointer to message data (includes protocol header)
 * @param length Total message length in bytes
 * @param src_addr Source address from MAC layer (16-bit 802.15.4 address)
 *
 * @note Handler is called at 1kHz polling rate - keep processing brief
 * @note Data pointer is only valid during callback - copy if needed
 */
typedef void (*uwb_protocol_handler_t)(const uint8_t* data, uint16_t length, uint16_t src_addr);

/**
 * @brief Register a protocol handler for application-layer protocol dispatch
 *
 * Protocol handlers receive messages matching their protocol type.
 * Multiple protocols can be registered simultaneously (e.g., TWR, DATA).
 * Message routing is based on the first byte (protocol_type field).
 *
 * @param protocol_type Protocol identifier (PROTOCOL_TYPE_* from uwb_protocol_messages.h)
 * @param handler Callback function to process messages of this protocol type
 *
 * @return true if handler registered successfully
 * @return false if registration failed (invalid params, table full)
 *
 * @note Maximum 8 protocol handlers can be registered
 * @note Re-registering same protocol_type updates the handler
 * @note Handler is called from UWB RX processing context (1kHz rate)
 *
 * @example
 * void twr_handler(const uint8_t* data, uint16_t len, uint16_t src) {
 *     // Process TWR ranging messages
 * }
 * uwb_register_protocol_handler(PROTOCOL_TYPE_TWR, twr_handler);
 */
bool uwb_register_protocol_handler(uint8_t protocol_type, uwb_protocol_handler_t handler);

/**
 * @brief Unregister a protocol handler
 *
 * @param protocol_type Protocol identifier to unregister
 */
void uwb_unregister_protocol_handler(uint8_t protocol_type);

/**
 * @brief Get protocol routing statistics
 *
 * @param total_received Output: Total application messages received (can be NULL)
 * @param unhandled Output: Messages with no registered handler (can be NULL)
 * @param invalid Output: Invalid/malformed messages (can be NULL)
 */
void uwb_get_protocol_stats(uint32_t* total_received, uint32_t* unhandled, uint32_t* invalid);

/**
 * @brief Reset protocol routing statistics to zero
 */
void uwb_reset_protocol_stats(void);

/**
 * @brief Register a callback for received UWB messages (low-level)
 *
 * @deprecated Use uwb_register_protocol_handler() for application protocols instead.
 *             This low-level callback is for internal use only.
 *
 * @param callback Function to call when message received, or NULL to unregister
 *
 * @note Callback is invoked at 1kHz polling rate when messages arrive
 * @note Payload has MAC header stripped - only application data is passed
 */
void uwb_register_rx_callback(uwb_rx_callback_t callback);

/**
 * @brief Get the UWB device handle
 * @return Pointer to UWB device, or NULL if not initialized
 */
uwb_dev_t* uwb_get_device(void);

/**
 * @brief Get the last TX timestamp
 *
 * Returns the timestamp of the last transmitted message. This is captured
 * automatically by the hardware after transmission completes.
 *
 * @return 40-bit TX timestamp in Device Time Units (DTU), or 0 if not available
 *
 * @note Only valid after a successful uwb_send_message() or uwb_send_message_delayed()
 * @note Used by ranging protocols (TWR) for precise time-of-flight calculations
 */
uint64_t uwb_get_last_tx_timestamp(void);

/**
 * @brief Get the last RX timestamp
 *
 * Returns the timestamp of the last received message. This is captured
 * automatically by the hardware when a message is received.
 *
 * @return 40-bit RX timestamp in Device Time Units (DTU), or 0 if not available
 *
 * @note Only valid after a message has been received and RX callback invoked
 * @note Used by ranging protocols (TWR) for precise time-of-flight calculations
 */
uint64_t uwb_get_last_rx_timestamp(void);

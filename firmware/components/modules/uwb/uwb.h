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
 * @brief Test UWB radio SPI communication and device ID
 * @return true if device responds correctly, false otherwise
 */
bool uwb_device_id(void);

/**
 * @brief Get the last measured device ID
 * @return 32-bit device ID value, or 0 if not yet read
 */
uint32_t uwb_get_device_id(void);

/**
 * @brief Get the last measured temperature
 * @return Temperature in degrees Celsius, or 0.0f if not yet read
 */
float uwb_get_temperature(void);

/**
 * @brief Get the last measured voltage
 * @return Voltage in volts, or 0.0f if not yet read
 */
float uwb_get_voltage(void);

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

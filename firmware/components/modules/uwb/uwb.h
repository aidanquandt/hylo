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
 * @brief Get last received message
 * @param buffer Buffer to store message
 * @param buffer_size Size of buffer
 * @return Number of bytes received, or 0 if no message
 */
uint16_t uwb_get_received_message(char* buffer, uint16_t buffer_size);

/**
 * @brief Get the parsed numeric value from last received message (watch in debugger)
 * @return Parsed number (e.g., 68)
 */
uint32_t uwb_get_rx_parsed_value(void);

/**
 * @brief Get count of received messages
 * @return Number of messages received
 */
uint32_t uwb_get_rx_count(void);

/**
 * @brief Get count of TX attempts (debug)
 * @return Number of times TX was attempted
 */
uint32_t uwb_get_tx_attempts(void);

/**
 * @brief Get count of RX checks (debug)
 * @return Number of times RX was checked
 */
uint32_t uwb_get_rx_checks(void);

/**
 * @brief Set this device's 802.15.4 address and PAN ID
 * @param address 16-bit short address for this device
 * @param pan_id 16-bit PAN identifier (default 0xDECA)
 */
void uwb_set_address(uint16_t address, uint16_t pan_id);

/**
 * @brief Set destination address for transmission
 * @param dest_addr 16-bit destination address (0xFFFF = broadcast)
 */
void uwb_set_dest_address(uint16_t dest_addr);

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
 * @brief Enable or disable periodic transmission
 * @param enable true to enable TX (node transmits at 1Hz), false to disable (RX only)
 * @note All nodes are always listening. This controls whether they also transmit.
 */
void uwb_set_tx_enable(bool enable);

/**
 * @brief Check if periodic transmission is enabled
 * @return true if TX enabled, false if RX only
 */
bool uwb_get_tx_enabled(void);
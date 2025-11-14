/*---------------------------------------------------------------------------
 * @file    dw3000_test.h
 * @brief   DW3000 hardware connection test module
 * 
 * @note    TX/RX mode is configured at compile time in dw3000_test.c
 *          Define DW3000_TX_MODE for transmitter, or comment it out for receiver
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
 * @brief Test DW3000 SPI communication and device ID
 * @return true if device responds correctly, false otherwise
 */
bool dw3000_test_device_id(void);

/**
 * @brief Get the last measured device ID
 * @return 32-bit device ID value, or 0 if not yet read
 */
uint32_t dw3000_test_get_device_id(void);

/**
 * @brief Get the last measured temperature
 * @return Temperature in degrees Celsius, or 0.0f if not yet read
 */
float dw3000_test_get_temperature(void);

/**
 * @brief Get the last measured voltage
 * @return Voltage in volts, or 0.0f if not yet read
 */
float dw3000_test_get_voltage(void);

/**
 * @brief Check if hardware is ready
 * @return true if DW3000 is initialized and ready
 */
bool dw3000_test_is_ready(void);

/**
 * @brief Enable message transmission mode
 * @param channel UWB channel (5 or 9 recommended)
 * @return true on success, false on failure
 * @note TX/RX mode is now compile-time. This function kept for API compatibility.
 */
bool dw3000_test_enable_tx(uint8_t channel);

/**
 * @brief Enable message reception mode
 * @param channel UWB channel (must match transmitter)
 * @return true on success, false on failure
 * @note TX/RX mode is now compile-time. This function kept for API compatibility.
 */
bool dw3000_test_enable_rx(uint8_t channel);

/**
 * @brief Get last received message
 * @param buffer Buffer to store message
 * @param buffer_size Size of buffer
 * @return Number of bytes received, or 0 if no message
 */
uint16_t dw3000_test_get_received_message(char *buffer, uint16_t buffer_size);

/**
 * @brief Get the parsed numeric value from last received message (watch in debugger)
 * @return Parsed number (e.g., 68)
 */
uint32_t dw3000_test_get_rx_parsed_value(void);

/**
 * @brief Get count of received messages
 * @return Number of messages received
 */
uint32_t dw3000_test_get_rx_count(void);

/**
 * @brief Get count of TX attempts (debug)
 * @return Number of times TX was attempted
 */
uint32_t dw3000_test_get_tx_attempts(void);

/**
 * @brief Get count of RX checks (debug)
 * @return Number of times RX was checked
 */
uint32_t dw3000_test_get_rx_checks(void);

/**
 * @brief Check if in TX mode (for debugging)
 * @return true if transmitter mode enabled
 */
bool dw3000_test_is_tx_mode(void);

/**
 * @brief Check if in RX mode (for debugging)
 * @return true if receiver mode enabled
 */
bool dw3000_test_is_rx_mode(void);

/**
 * @brief Set this device's 802.15.4 address and PAN ID
 * @param address 16-bit short address for this device
 * @param pan_id 16-bit PAN identifier (default 0xDECA)
 */
void dw3000_test_set_address(uint16_t address, uint16_t pan_id);

/**
 * @brief Set destination address for transmission
 * @param dest_addr 16-bit destination address (0xFFFF = broadcast)
 */
void dw3000_test_set_dest_address(uint16_t dest_addr);

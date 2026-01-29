#pragma once

/*---------------------------------------------------------------------------
 * Feature Flags
 *---------------------------------------------------------------------------*/

/**
 * @brief Enable printing of ranging success messages and distance measurements
 *
 * When enabled, successful ranging operations will print debug information
 * including the measured distance and timing information.
 *
 * Set to 1 to enable, 0 to disable.
 */
#define FEATURE_PRINT_RANGING_SUCCESS_AND_DISTANCE (0U)

/**
 * @brief Enable UART logging output
 *
 * When enabled, error_handler_log() will output messages via UART.
 * When disabled, errors are still recorded in history and counters,
 * but no UART output is generated.
 *
 * Set to 1 to enable, 0 to disable.
 */
#define FEATURE_UART_LOGGING (1U)

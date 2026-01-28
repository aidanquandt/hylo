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
#define FEATURE_PRINT_RANGING_SUCCESS_AND_DISTANCE (1U)

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

/**
 * @brief Auto-start device in anchor mode with default address
 *
 * When enabled, the device will automatically start in anchor mode
 * with address 0x0001 after initialization. This is useful for
 * production devices that should always operate as anchors.
 *
 * When disabled, the device starts in an idle state and requires
 * manual configuration via UART commands.
 *
 * Set to 1 to enable, 0 to disable.
 */
#define FEATURE_AUTO_START_ANCHOR_MODE (1U)

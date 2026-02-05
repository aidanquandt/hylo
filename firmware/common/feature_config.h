#pragma once

/*---------------------------------------------------------------------------
 * Feature Flags
 *---------------------------------------------------------------------------*/

#define FEATURE_PRINT_RANGING_SUCCESS_AND_DISTANCE (0U)

#define FEATURE_PRINT_SENSOR_FUSION_LOCATION_ESTIMATE (1U)

#define FEATURE_UART_LOGGING (1U)

/* UART selection: 0 = UART4, 1 = USART3 */
#define FEATURE_USE_USART3 (1U)

/* Watchdog: Set to 0 to disable IWDG for debugging */
#define FEATURE_WATCHDOG_ENABLE_IWDG (1U)

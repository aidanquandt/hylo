#pragma once

/*---------------------------------------------------------------------------
 * Feature Flags
 *---------------------------------------------------------------------------*/

#define FEATURE_PRINT_RANGING_SUCCESS_AND_DISTANCE (1U)

#define FEATURE_PRINT_SENSOR_FUSION_LOCATION_ESTIMATE (0U)

#define FEATURE_UART_LOGGING (1U)

/* UART selection: 0 = UART4, 1 = USART3 */
#define FEATURE_USE_USART3 (1U)

/* Watchdog: Set to 0 to disable IWDG for debugging */
#define FEATURE_WATCHDOG_ENABLE_IWDG (1U)

/* IMU: Set to 1 for 200Hz sampling, 0 for 10Hz (legacy) */
#define FEATURE_IMU_200HZ_SAMPLING (1U)

/* SPI DMA: Set to 1 to use DMA transfers (non-blocking), 0 for blocking transfers
 * Note: Requires hardware DMA configuration via CubeMX before enabling */
#define FEATURE_SPI_USE_DMA (0U)

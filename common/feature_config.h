#pragma once

/*---------------------------------------------------------------------------
 * Feature Flags
 *---------------------------------------------------------------------------*/

#define FEATURE_PRINT_RANGING_SUCCESS_AND_DISTANCE (0U)

#define FEATURE_PRINT_SENSOR_FUSION_LOCATION_ESTIMATE (1U)

#define FEATURE_UART_LOGGING (1U)

/* Watchdog: Set to 0 to disable IWDG for debugging */
#define FEATURE_WATCHDOG_ENABLE_IWDG (1U)

/* Auto-configure UWB address from device UUID mapping table */
#define FEATURE_AUTO_CONFIGURE_ADDRESS_FROM_UUID (1U)

/* Hardware-specific (HWREV 0 / HWREV 1).
 * FEATURE_IMUS_POPULATED: 0 = zero IMUs (HWREV 0), 1 = four IMUs (HWREV 1).
 * The port layer (imu_port.h) maps this to IMU_NUM_DEVICES (0 or 4). */
#if (HWREV == 0)
#define FEATURE_WIFI_MODULE     (0U)
#define FEATURE_IMUS_POPULATED  (0U)
#elif (HWREV == 1)
#define FEATURE_WIFI_MODULE     (1U)
#define FEATURE_IMUS_POPULATED  (1U)
#endif

#define FEATURE_WIFI_PRINT_STATE (0U)
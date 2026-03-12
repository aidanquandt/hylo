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

/* WiFi telemetry module (ESP8266) */
#if (HWREV == 0)
#define FEATURE_WIFI_MODULE (0U)
#elif (HWREV == 1)
#define FEATURE_WIFI_MODULE (1U)
#endif

#define FEATURE_WIFI_PRINT_STATE (0U)
/*---------------------------------------------------------------------------
 * @file    platform_system.h
 * @brief   Platform system utilities (reset, UUID, device identification)
 *---------------------------------------------------------------------------*/
#pragma once

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "common.h"

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define PLATFORM_SYSTEM_UUID_SIZE 12  // 96 bits = 12 bytes

/*---------------------------------------------------------------------------
 * Typedefs
 *---------------------------------------------------------------------------*/
typedef struct
{
    uint8_t uuid[PLATFORM_SYSTEM_UUID_SIZE];  // Full 96-bit UUID
    uint32_t uuid_word0;                      // First 32 bits
    uint32_t uuid_word1;                      // Middle 32 bits
    uint32_t uuid_word2;                      // Last 32 bits (most variable)
    bool is_known_device;                     // true if in mapping table
    uint16_t assigned_address;                // UWB address (0 if unknown)
    const char* device_name;                  // Name from mapping table
} platform_system_device_info_t;

/*---------------------------------------------------------------------------
 * Public Function Prototypes
 *---------------------------------------------------------------------------*/

// System control
void platform_system_reset(void);

// UUID hardware access
void platform_system_get_uuid(uint8_t uuid[PLATFORM_SYSTEM_UUID_SIZE]);
uint32_t platform_system_get_uuid_word(uint8_t word_index);

// Device identification (requires device_mapping from config)
void platform_system_device_init(void);
bool platform_system_device_get_info(platform_system_device_info_t* info);
bool platform_system_device_is_known(void);
uint16_t platform_system_device_get_assigned_address(void);
const char* platform_system_device_get_name(void);
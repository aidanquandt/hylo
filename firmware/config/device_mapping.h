/*---------------------------------------------------------------------------
 * @file    device_mapping.h
 * @brief   Board-specific device UUID to address mappings
 * 
 * This file contains the mapping table for known devices in your deployment.
 * Each device's unique STM32 UUID is mapped to a specific UWB address and
 * human-readable identifier.
 *---------------------------------------------------------------------------*/
#pragma once

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include <stdint.h>

/*---------------------------------------------------------------------------
 * Typedefs
 *---------------------------------------------------------------------------*/
typedef struct
{
    uint32_t uuid_word0;     // First 32 bits of STM32 UUID
    uint32_t uuid_word1;     // Middle 32 bits of STM32 UUID
    uint32_t uuid_word2;     // Last 32 bits of STM32 UUID
    uint16_t uwb_address;    // Assigned UWB address for this device
    const char* device_name; // Human-readable identifier
} device_mapping_entry_t;

/*---------------------------------------------------------------------------
 * Public Data
 *---------------------------------------------------------------------------*/
extern const device_mapping_entry_t device_mapping_table[];
extern const uint32_t device_mapping_table_size;

/*---------------------------------------------------------------------------
 * Public Function Prototypes
 *---------------------------------------------------------------------------*/

/**
 * @brief Find device mapping by full UUID
 * @param uuid_word0 First 32 bits of device UUID
 * @param uuid_word1 Middle 32 bits of device UUID
 * @param uuid_word2 Last 32 bits of device UUID
 * @return Pointer to mapping entry if found, NULL otherwise
 */
const device_mapping_entry_t* device_mapping_find(uint32_t uuid_word0, uint32_t uuid_word1, uint32_t uuid_word2);

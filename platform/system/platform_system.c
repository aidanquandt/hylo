/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "platform_system.h"
#include "device_mapping.h"
#include "stm32h7xx.h"
#include <string.h>

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
// STM32H7 Unique Device ID register base address
#define UID_BASE_ADDRESS 0x1FF1E800UL

/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/
STATIC platform_system_device_info_t device_info = {0};
STATIC bool device_initialized                   = false;

/*---------------------------------------------------------------------------
 * Public Function Implementations
 *---------------------------------------------------------------------------*/

void platform_system_reset(void)
{
    NVIC_SystemReset();
}

void platform_system_get_uuid(uint8_t uuid[PLATFORM_SYSTEM_UUID_SIZE])
{
    if (uuid == NULL)
    {
        return;
    }
    
    const uint32_t* uid = (const uint32_t*)UID_BASE_ADDRESS;
    memcpy(uuid, uid, PLATFORM_SYSTEM_UUID_SIZE);
}

uint32_t platform_system_get_uuid_word(uint8_t word_index)
{
    if (word_index > 2)
    {
        return 0;
    }
    
    const uint32_t* uid = (const uint32_t*)UID_BASE_ADDRESS;
    return uid[word_index];
}

void platform_system_device_init(void)
{
    if (device_initialized)
    {
        return;
    }

    // Read UUID from hardware
    platform_system_get_uuid(device_info.uuid);
    device_info.uuid_word0 = platform_system_get_uuid_word(0);
    device_info.uuid_word1 = platform_system_get_uuid_word(1);
    device_info.uuid_word2 = platform_system_get_uuid_word(2);

    // Check if device is in mapping table
    const device_mapping_entry_t* mapping = device_mapping_find(device_info.uuid_word0, device_info.uuid_word1, device_info.uuid_word2);

    if (mapping != NULL)
    {
        device_info.is_known_device  = true;
        device_info.assigned_address = mapping->uwb_address;
        device_info.device_name      = mapping->device_name;
    }
    else
    {
        device_info.is_known_device  = false;
        device_info.assigned_address = 0;
        device_info.device_name      = "UNKNOWN";
    }

    device_initialized = true;
}

bool platform_system_device_get_info(platform_system_device_info_t* info)
{
    if (info == NULL || !device_initialized)
    {
        return false;
    }

    memcpy(info, &device_info, sizeof(platform_system_device_info_t));
    return true;
}

bool platform_system_device_is_known(void)
{
    return device_initialized && device_info.is_known_device;
}

uint16_t platform_system_device_get_assigned_address(void)
{
    if (!device_initialized || !device_info.is_known_device)
    {
        return 0;
    }

    return device_info.assigned_address;
}

const char* platform_system_device_get_name(void)
{
    if (!device_initialized)
    {
        return "UNINITIALIZED";
    }

    return device_info.device_name;
}

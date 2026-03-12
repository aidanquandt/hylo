#pragma once

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "common.h"

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define SYSTEM_DRIVER_UUID_SIZE 12

/*---------------------------------------------------------------------------
 * Typedefs
 *---------------------------------------------------------------------------*/
typedef struct
{
    uint8_t uuid[SYSTEM_DRIVER_UUID_SIZE];
    uint32_t uuid_word0;
    uint32_t uuid_word1;
    uint32_t uuid_word2;
    bool is_known_device;
    uint16_t assigned_address;
    const char* device_name;
} system_driver_device_info_t;

/*---------------------------------------------------------------------------
 * Public Function Prototypes
 *---------------------------------------------------------------------------*/
void system_driver_reset(void);
void system_driver_get_uuid(uint8_t uuid[SYSTEM_DRIVER_UUID_SIZE]);
uint32_t system_driver_get_uuid_word(uint8_t word_index);
void system_driver_device_init(void);
bool system_driver_device_get_info(system_driver_device_info_t* info);
bool system_driver_device_is_known(void);
uint16_t system_driver_device_get_assigned_address(void);
const char* system_driver_device_get_name(void);

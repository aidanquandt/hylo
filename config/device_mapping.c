/*---------------------------------------------------------------------------
 * @file    device_mapping.c
 * @brief   Device UUID to address mapping table
 * 
 * HOW TO ADD NEW DEVICES:
 * =======================
 * 1. Flash firmware to your device
 * 2. Connect via UART and run command: system.get.uuid
 * 3. Copy the output line directly into the table below
 * 4. Edit the uwb_address and device_name fields as needed
 * 5. Rebuild and reflash - device will auto-configure on boot
 * 
 * EXAMPLE ENTRIES:
 * ================
 * {.uuid_word0 = 0xAABBCCDD, .uuid_word1 = 0x11223344, .uuid_word2 = 0x99887766, .uwb_address = 0x0003, .device_name = "TAG_01"},
 * {.uuid_word0 = 0x12345678, .uuid_word1 = 0xABCDEF00, .uuid_word2 = 0x98765432, .uwb_address = 0x0004, .device_name = "ANCHOR_01"},
 *---------------------------------------------------------------------------*/

#include "device_mapping.h"
#include <stddef.h>

/*---------------------------------------------------------------------------
 * Mapping Table - Add Your Devices Here
 *---------------------------------------------------------------------------*/
const device_mapping_entry_t device_mapping_table[] = {
    {.uuid_word0 = 0x00350034, .uuid_word1 = 0x34335113, .uuid_word2 = 0x33343839, .uwb_address = 0x0003, .device_name = "DEVICE_3"},
    {.uuid_word0 = 0x003C003B, .uuid_word1 = 0x34335113, .uuid_word2 = 0x33343839, .uwb_address = 0x0004, .device_name = "DEVICE_4"},
    {.uuid_word0 = 0x00140044, .uuid_word1 = 0x32335118, .uuid_word2 = 0x39373632, .uwb_address = 0x0005, .device_name = "DEVICE_5"},
    {.uuid_word0 = 0x0008002E, .uuid_word1 = 0x32335114, .uuid_word2 = 0x37333432, .uwb_address = 0x0006, .device_name = "DEVICE_6"},

    // Add more device mappings here
    // Run: system.get.uuid (then copy/paste the line it gives you)
};

const uint32_t device_mapping_table_size = sizeof(device_mapping_table) / sizeof(device_mapping_table[0]);

/*---------------------------------------------------------------------------
 * Public Function Implementations
 *---------------------------------------------------------------------------*/

const device_mapping_entry_t* device_mapping_find(uint32_t uuid_word0, uint32_t uuid_word1, uint32_t uuid_word2)
{
    // Early return if table is empty
    if (device_mapping_table_size == 0)
    {
        return NULL;
    }

    for (size_t i = 0; i < device_mapping_table_size; i++)
    {
        if (device_mapping_table[i].uuid_word0 == uuid_word0 &&
            device_mapping_table[i].uuid_word1 == uuid_word1 &&
            device_mapping_table[i].uuid_word2 == uuid_word2)
        {
            return &device_mapping_table[i];
        }
    }

    return NULL;
}

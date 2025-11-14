/*---------------------------------------------------------------------------
 * @file    counter.c
 * @brief   Utility functions for counter operations
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "counter.h"

/*---------------------------------------------------------------------------
 * Public Function Implementations
 *---------------------------------------------------------------------------*/

bool counter_uint8_t(uint8_t *counter, uint8_t threshold)
{
    if (counter == NULL) {
        return false;
    }
    
    (*counter)++;
    
    if (*counter >= threshold) {
        *counter = 0;
        return true;
    }
    
    return false;
}

bool counter_uint16_t(uint16_t *counter, uint16_t threshold)
{
    if (counter == NULL) {
        return false;
    }
    
    (*counter)++;
    
    if (*counter >= threshold) {
        *counter = 0;
        return true;
    }
    
    return false;
}

bool counter_uint32_t(uint32_t *counter, uint32_t threshold)
{
    if (counter == NULL) {
        return false;
    }
    
    (*counter)++;
    
    if (*counter >= threshold) {
        *counter = 0;
        return true;
    }
    
    return false;
}

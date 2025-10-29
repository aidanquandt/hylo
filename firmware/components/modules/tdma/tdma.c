/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "tdma.h"
#include "module.h"

/*---------------------------------------------------------------------------
 * Module Functions
 *---------------------------------------------------------------------------*/
STATIC void tdma_init(void);
STATIC void tdma_process_100Hz(void);

extern const module_S tdma_module;
const module_S tdma_module = {
        .module_init = tdma_init,
        .module_process_100Hz = tdma_process_100Hz,
};

/*---------------------------------------------------------------------------
 * Public function implementations
 *---------------------------------------------------------------------------*/

STATIC void tdma_init(void) 
{

}

STATIC void tdma_process_100Hz(void) 
{

}

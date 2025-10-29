/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "twr.h"
#include "module.h"

/*---------------------------------------------------------------------------
 * Module Functions
 *---------------------------------------------------------------------------*/
STATIC void twr_init(void);
STATIC void twr_process_100Hz(void);

extern const module_S twr_module;
const module_S twr_module = {
        .module_init = twr_init,
        .module_process_100Hz = twr_process_100Hz,
};

/*---------------------------------------------------------------------------
 * Public function implementations
 *---------------------------------------------------------------------------*/
STATIC void twr_init(void) {

}

STATIC void twr_process_100Hz(void) {

}

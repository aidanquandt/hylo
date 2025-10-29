/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "node.h"
#include "common.h"
#include "module.h"

/*---------------------------------------------------------------------------
 * Module Functions
 *---------------------------------------------------------------------------*/
STATIC void node_init(void);
STATIC void node_process_100Hz(void);

extern const module_S node_module;
const module_S node_module = {
        .module_init = node_init,
        .module_process_100Hz = node_process_100Hz,
};

/*---------------------------------------------------------------------------
 * Public function implementations
 *---------------------------------------------------------------------------*/
STATIC void node_init(void) {

}

STATIC void node_process_100Hz(void) {

}
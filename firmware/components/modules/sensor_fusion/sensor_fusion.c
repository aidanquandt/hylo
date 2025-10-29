/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "sensor_fusion.h"
#include "module.h"

/*---------------------------------------------------------------------------
 * Module Functions
 *---------------------------------------------------------------------------*/
STATIC void sensor_fusion_init(void);
extern const module_S sensor_fusion_module;
const module_S sensor_fusion_module = { 
        .module_init = sensor_fusion_init,
};

/*---------------------------------------------------------------------------
 * Public function implementations
 *---------------------------------------------------------------------------*/
STATIC void sensor_fusion_init(void) {

}

void sensor_fusion_isr(void) {

}

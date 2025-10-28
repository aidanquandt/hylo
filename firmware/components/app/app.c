/*---------------------------------------------------------------------------
 * Includes :) :)
 *---------------------------------------------------------------------------*/
#include "app.h" // hello guys
#include "common.h"
#include "main.h"
#include "platform.h"
#include "platform_gpio.h"
#include "cmsis_os.h"
#include "task.h"
#include "datalogger.h"
#include "node.h"
#include "sensor_fusion.h"
#include "tdma.h"
#include "twr.h"

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define TASK_RATE_1KHZ pdMS_TO_TICKS(1)
#define TASK_RATE_100HZ pdMS_TO_TICKS(10)
#define TASK_RATE_10HZ pdMS_TO_TICKS(100)

#define TASK_PRIORITY_CRITICAL 4
#define TASK_PRIORITY_HIGH 3
#define TASK_PRIORITY_NORMAL 2
#define TASK_PRIORITY_LOW 1

#define TASK_STACK_SMALL 256
#define TASK_STACK_MEDIUM 512
#define TASK_STACK_LARGE 1024

/*---------------------------------------------------------------------------
 * Typedefs
 *---------------------------------------------------------------------------*/
typedef struct
{
    // Other parameters of xTaskCreate left NULL.
    TickType_t task_time_increment;
    void (*task_pointer)(void);
    char * task_name;
    configSTACK_DEPTH_TYPE task_stack_depth;
    UBaseType_t task_priority;
} module_freertos_parameters_S;
typedef struct 
{
    void (*module_init)(void);
    void (*module_process)(void); // note - this is only for freertos tasks so eg sensor fusion ISR will just be NULL for this as we won't have freertos task there
    module_freertos_parameters_S module_freertos_parameters;
} module_parameters_S;

/*---------------------------------------------------------------------------
 * Private function prototypes
 *---------------------------------------------------------------------------*/
STATIC void module_task(void *argument);
STATIC void app_initialize_modules(void);
STATIC void app_create_module_tasks(void);

/*---------------------------------------------------------------------------
 * Data declarations
 *---------------------------------------------------------------------------*/
STATIC const module_parameters_S module_parameters[NUM_MODULES] = {
    [SENSOR_FUSION_MODULE] = {
        .module_init = sensor_fusion_init,
        .module_process = NULL,
        .module_freertos_parameters = { 0 }
    },
    [DATALOGGER_MODULE] = {
        .module_init = datalogger_init,
        .module_process = datalogger_process,
        .module_freertos_parameters = {
            .task_time_increment = TASK_RATE_10HZ,
            .task_pointer = NULL,
            .task_name = TOSTRING(DATALOGGER_MODULE),
            .task_stack_depth = TASK_STACK_MEDIUM,
            .task_priority = TASK_PRIORITY_LOW
        }
    },
    [NODE_MODULE] = {
        .module_init = node_init,
        .module_process = node_process,
        .module_freertos_parameters = {
            .task_time_increment = TASK_RATE_1KHZ,
            .task_pointer = NULL,
            .task_name = TOSTRING(NODE_MODULE),
            .task_stack_depth = TASK_STACK_MEDIUM,
            .task_priority = TASK_PRIORITY_HIGH
        }
    },
    [TDMA_MODULE] = {
        .module_init = tdma_init,
        .module_process = tdma_process,
        .module_freertos_parameters = {
            .task_time_increment = TASK_RATE_1KHZ,
            .task_pointer = NULL,
            .task_name = TOSTRING(TDMA_MODULE),
            .task_stack_depth = TASK_STACK_MEDIUM,
            .task_priority = TASK_PRIORITY_HIGH
        }
    },
    [TWR_MODULE] = {
        .module_init = twr_init,
        .module_process = twr_process,
        .module_freertos_parameters = {
            .task_time_increment = TASK_RATE_1KHZ,
            .task_pointer = NULL,
            .task_name = TOSTRING(TWR_MODULE),
            .task_stack_depth = TASK_STACK_MEDIUM,
            .task_priority = TASK_PRIORITY_HIGH
        }
    }
};

/*---------------------------------------------------------------------------
 * Private function implementations
 *---------------------------------------------------------------------------*/
STATIC void module_task(void *argument) {
    modules_E module = (modules_E)(uintptr_t)argument;
    TickType_t lastWake = xTaskGetTickCount();
    for(;;)
    {
        module_parameters[module].module_process();
        vTaskDelayUntil(&lastWake, module_parameters[module].module_freertos_parameters.task_time_increment);
    }
}

STATIC void app_initialize_modules(void)
{
    for (modules_E module_idx = (modules_E)0U; module_idx < NUM_MODULES; module_idx++) 
    {
        if (module_parameters[module_idx].module_init != NULL)
        {
            module_parameters[module_idx].module_init();
        }
    }
}

STATIC void app_create_module_tasks(void)
{
    for (modules_E module_idx = (modules_E)0U; module_idx < NUM_MODULES; module_idx++) 
    {
        if (module_parameters[module_idx].module_process != NULL)
        {
            xTaskCreate(
                module_task,
                module_parameters[module_idx].module_freertos_parameters.task_name,
                module_parameters[module_idx].module_freertos_parameters.task_stack_depth,
                (void *)(uintptr_t)module_idx,
                module_parameters[module_idx].module_freertos_parameters.task_priority,
                NULL
            );
        }
    }
}

/*---------------------------------------------------------------------------
 * Public function implementations
 *---------------------------------------------------------------------------*/
void app_init(void) {

    app_initialize_modules();
    app_create_module_tasks();

    osThreadExit();
}
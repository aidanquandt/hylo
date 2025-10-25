/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "app.h"
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
 * Private function prototypes
 *---------------------------------------------------------------------------*/
STATIC void datalogger_task(void *argument);
STATIC void node_task(void *argument);
STATIC void sensor_fusion_task(void *argument);
STATIC void tdma_task(void *argument);
STATIC void twr_task(void *argument);

/*---------------------------------------------------------------------------
 * Private function implementations
 *---------------------------------------------------------------------------*/
STATIC void datalogger_task(void *argument) {
    TickType_t lastWake = xTaskGetTickCount();
    for(;;)
    {
        datalogger_process();
        vTaskDelayUntil(&lastWake, TASK_RATE_10HZ);
    }
}

STATIC void node_task(void *argument) {
    TickType_t lastWake = xTaskGetTickCount();
    for(;;)
    {
        node_process();
        vTaskDelayUntil(&lastWake, TASK_RATE_100HZ);
    }
}

STATIC void sensor_fusion_task(void *argument) {
    TickType_t lastWake = xTaskGetTickCount();
    for(;;)
    {
        sensor_fusion_process();
        vTaskDelayUntil(&lastWake, TASK_RATE_1KHZ);
    }
}

STATIC void tdma_task(void *argument) {
    TickType_t lastWake = xTaskGetTickCount();
    for(;;)
    {
        tdma_process();
        vTaskDelayUntil(&lastWake, TASK_RATE_100HZ);
    }
}

STATIC void twr_task(void *argument) {
    TickType_t lastWake = xTaskGetTickCount();
    for(;;)
    {
        twr_process();
        vTaskDelayUntil(&lastWake, TASK_RATE_100HZ);
    }
}

/*---------------------------------------------------------------------------
 * Public function implementations
 *---------------------------------------------------------------------------*/
void app_init(void) {
    datalogger_init();
    node_init();
    sensor_fusion_init();
    tdma_init();
    twr_init();
    
    xTaskCreate(datalogger_task, "Datalogger", TASK_STACK_MEDIUM, NULL, TASK_PRIORITY_NORMAL, NULL);
    xTaskCreate(node_task, "Node", TASK_STACK_MEDIUM, NULL, TASK_PRIORITY_HIGH, NULL);
    xTaskCreate(sensor_fusion_task, "SensorFusion", TASK_STACK_LARGE, NULL, TASK_PRIORITY_CRITICAL, NULL);
    xTaskCreate(tdma_task, "TDMA", TASK_STACK_MEDIUM, NULL, TASK_PRIORITY_HIGH, NULL);
    xTaskCreate(twr_task, "TWR", TASK_STACK_MEDIUM, NULL, TASK_PRIORITY_HIGH, NULL);
}

void app_enter(void *argument) {
    app_init();
    osThreadExit();
}

void configureTimerForRunTimeStats(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

unsigned long getRunTimeCounterValue(void)
{
    return DWT->CYCCNT;
}
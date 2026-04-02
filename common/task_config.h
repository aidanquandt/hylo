#pragma once

/*---------------------------------------------------------------------------
 * System-Wide Task Priority Definitions
 *---------------------------------------------------------------------------
 * FreeRTOS priorities: 0 (lowest/idle) to configMAX_PRIORITIES-1 (highest)
 *
 * Priority Assignment Strategy:
 *   8: UWB IRQ deferred handler (dwt_isr in task context)
 *   7: UWB TX worker (serialized frame transmission)
 *   6: UWB RX worker (deliver received frames to stack)
 *   4: High-rate sensor sampling (IMU at 1kHz)
 *   3: Normal I/O and communication (UART)
 *   2: Background computation (sensor fusion, ranging)
 *   1: Monitoring and logging (datalogger)
 *   0: Watchdog (intentionally lowest - verifies all others can run)
 *
 * Design Note: Watchdog runs at lowest priority as a "canary" - it only
 * executes if higher priority tasks yield, proving they're not starving.
 *---------------------------------------------------------------------------*/

/* Task priorities: highest number first (ties: alphabetical by macro name). */
#define TASK_PRIORITY_UWB_IRQ           8 // GPIO IRQ -> dwt_isr() (highest UWB)
#define TASK_PRIORITY_UWB_TX            7 // TX queue worker
#define TASK_PRIORITY_UWB_RX            6 // RX queue worker
#define TASK_PRIORITY_IMU_PERIODIC      5 // 1kHz sensor sampling
#define TASK_PRIORITY_UART_RX           3 // Console RX: stream drain, framing, protocol dispatch
#define TASK_PRIORITY_UART_TX           3 // UART TX DMA drain; outbound protocol / telemetry
#define TASK_PRIORITY_UWB_STATE_MACHINE 3 // UWB chip state machine (100 Hz)
#define TASK_PRIORITY_SDCARD            2 // SD card logging
#define TASK_PRIORITY_SENSOR_FUSION     2 // Kalman filter updates
#define TASK_PRIORITY_TWR               2 // Ranging algorithm
#define TASK_PRIORITY_TWR_MANAGER       2 // Ranging coordination
#define TASK_PRIORITY_UWB_NODE          2 // Node state management
#define TASK_PRIORITY_WIFI              2 // WiFi state machine (10 Hz)
#define TASK_PRIORITY_DATALOGGER        1 // Statistics logging
#define TASK_PRIORITY_IMU_CALIBRATION   1 // On-demand IMU calibration (same tier as monitoring)
#define TASK_PRIORITY_WATCHDOG          0 // System health monitor (lowest)

/*---------------------------------------------------------------------------
 * Task stack sizes (FreeRTOS: depth in StackType_t words, 32-bit => 4 bytes/word)
 *---------------------------------------------------------------------------
 *  Macro              Words   Bytes   Used by
 *  -----------------  ------  ------  -----------------------------------------
 *  TASK_STACK_1KB     256     1024   watchdog (both tasks), UART TX DMA drain
 *  TASK_STACK_2KB     512     2048   datalogger, UWB 100 Hz, UWB node 1 Hz,
 *                                   sensor_fusion 10 Hz, UWB driver TX worker,
 *                                   IMU stream, IMU calibration
 *  TASK_STACK_3KB     768     3072   UWB driver RX worker
 *  TASK_STACK_4KB     1024    4096   TWR, TWR manager, UART protocol RX task,
 *                                   WiFi task, SD card logger, UWB driver IRQ
 *  TASK_STACK_8KB     2048    8192   IMU 1 kHz, sensor_fusion main task
 *---------------------------------------------------------------------------*/
#define TASK_STACK_1KB 256
#define TASK_STACK_2KB 512
#define TASK_STACK_3KB 768
#define TASK_STACK_4KB 1024
#define TASK_STACK_8KB 2048

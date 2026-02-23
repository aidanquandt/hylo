#pragma once

/*---------------------------------------------------------------------------
 * System-Wide Task Priority Definitions
 *---------------------------------------------------------------------------
 * FreeRTOS priorities: 0 (lowest/idle) to configMAX_PRIORITIES-1 (highest)
 *
 * Priority Assignment Strategy:
 *   6: Critical ISR deferred work (UWB radio timing-sensitive RX)
 *   5: Time-critical transmission (UWB TX serialization)
 *   4: High-rate sensor sampling (IMU at 1kHz)
 *   3: Normal I/O and communication (UART)
 *   2: Background computation (sensor fusion, ranging)
 *   1: Monitoring and logging (datalogger)
 *   0: Watchdog (intentionally lowest - verifies all others can run)
 *
 * Design Note: Watchdog runs at lowest priority as a "canary" - it only
 * executes if higher priority tasks yield, proving they're not starving.
 *---------------------------------------------------------------------------*/

// Critical real-time tasks (timing-sensitive)
#define TASK_PRIORITY_UWB_RX 6 // Deferred interrupt processing
#define TASK_PRIORITY_UWB_TX 5 // Frame transmission timing

// High-rate processing
#define TASK_PRIORITY_IMU_PERIODIC 4 // 1kHz sensor sampling

// Normal processing
#define TASK_PRIORITY_UART_TX 3 // Communication I/O

// Background processing
#define TASK_PRIORITY_SENSOR_FUSION 2 // Kalman filter updates
#define TASK_PRIORITY_TWR 2           // Ranging algorithm
#define TASK_PRIORITY_TWR_MANAGER 2   // Ranging coordination
#define TASK_PRIORITY_UWB_NODE 2      // Node state management

// Low priority / monitoring
#define TASK_PRIORITY_DATALOGGER 1 // Statistics logging
#define TASK_PRIORITY_WATCHDOG 0   // System health monitor (lowest)

/*---------------------------------------------------------------------------
 * Task Stack Sizes (in 32-bit words)
 *---------------------------------------------------------------------------*/
#define TASK_STACK_SMALL 256  // Minimal tasks
#define TASK_STACK_MEDIUM 512 // Standard tasks
#define TASK_STACK_LARGE 1024 // Complex processing

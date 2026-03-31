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
 *   1: Monitoring and logging (datalogger); watchdog 1 kHz tick canary
 *   0: Watchdog supervisor only (IWDG refresh / failure path)
 *
 * Design Note: Watchdog supervisor is lowest so it only runs when higher work
 * yields. The 1 kHz canary is one step higher so UART/reporting in the
 * supervisor cannot stretch tick gaps and false-trip the tick-delta check.
 *---------------------------------------------------------------------------*/

// Critical real-time tasks (timing-sensitive) — see drivers/uwb_driver
#define TASK_PRIORITY_UWB_IRQ 8 // GPIO IRQ -> dwt_isr() (highest UWB)
#define TASK_PRIORITY_UWB_TX 7  // TX queue worker
#define TASK_PRIORITY_UWB_RX 6  // RX queue worker

// High-rate processing
#define TASK_PRIORITY_IMU_PERIODIC 5 // 1kHz sensor sampling

// Normal processing
#define TASK_PRIORITY_UART_TX 3 // Communication I/O

// Background processing
#define TASK_PRIORITY_SENSOR_FUSION 2     // Kalman filter updates
#define TASK_PRIORITY_TWR 2               // Ranging algorithm
#define TASK_PRIORITY_TWR_MANAGER 2       // Ranging coordination
#define TASK_PRIORITY_UWB_NODE 2          // Node state management
#define TASK_PRIORITY_UWB_STATE_MACHINE 3 // UWB chip state machine (100 Hz)
#define TASK_PRIORITY_WIFI 2              // WiFi state machine (10 Hz)

// Low priority / monitoring
#define TASK_PRIORITY_DATALOGGER 1           // Statistics logging
#define TASK_PRIORITY_WATCHDOG_CANARY 9        // 1 kHz tick-gap check (same band as datalogger)
#define TASK_PRIORITY_WATCHDOG 0               // IWDG supervisor (lowest)

/*---------------------------------------------------------------------------
 * Task Stack Sizes
 *---------------------------------------------------------------------------
 * Named in bytes for readability; values are in 32-bit words as required
 * by xTaskCreate (1 KB = 256 words, 2 KB = 512 words, 4 KB = 1024 words).
 *---------------------------------------------------------------------------*/
#define TASK_STACK_1KB 256  // 1024 bytes
#define TASK_STACK_2KB 512  // 2048 bytes
#define TASK_STACK_4KB 1024 // 4096 bytes
#define TASK_STACK_8KB 2048 // 8192 bytes

#pragma once

/* Start the UART protocol RX task. Call once after uart_driver_init(). */
void uart_protocol_task_start(void);

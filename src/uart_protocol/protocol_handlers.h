/*
 * UART protocol handlers: public interface for handler state
 */
#pragma once

#include <stdbool.h>

/**
 * @brief Check if IMU streaming is currently active
 * @return true if IMU stream is running, false otherwise
 */
bool protocol_handler_is_imu_streaming(void);

/**
 * @brief Set response destination to match the source of the incoming request
 * 
 * This enables bidirectional routing: if a request comes from WiFi, 
 * the response goes back to WiFi. If it comes from Console UART, 
 * the response goes to Console.
 * 
 * Call this at the beginning of request handlers that send responses.
 */
void protocol_handler_set_response_destination(void);

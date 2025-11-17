/*---------------------------------------------------------------------------
 * @file    dw3000_port.h
 * @brief   Port layer for DW3000 UWB driver - Adapter between driver and platform
 *---------------------------------------------------------------------------*/
#pragma once

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "common.h"
#include "deca_interface.h"
#include "platform_spi.h"

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define DW3000_CS_PIN  PLATFORM_SPI_CS_DW3000

/** Return codes from port layer functions */
#define DW3000_SUCCESS  (0)   // Operation successful
#define DW3000_ERROR    (-1)  // Operation failed

/*---------------------------------------------------------------------------
 * Public Function Prototypes
 *---------------------------------------------------------------------------*/

/**
 * @brief Initialize the DW3000 port layer
 * @return Pointer to configured dwchip_s structure, or NULL on failure
 */
dwchip_t* dw3000_port_init(void);

/**
 * @brief Probe and initialize the DW3000 device
 * @param chip Pointer to chip structure from dw3000_port_init()
 * @return DW3000_SUCCESS on success, DW3000_ERROR on failure
 * @note This function performs both hardware probe and device initialization
 */
int dw3000_port_probe_and_init(dwchip_t *chip);

/**
 * @brief Wake up the DW3000 device using GPIO
 */
void dw3000_port_wakeup_device(void);

/**
 * @brief Check if DW3000 device ID is valid
 * @return 0 on success, negative error code on failure
 */
int dw3000_port_check_device_id(void);

/**
 * @brief Read DW3000 device ID register
 * @return 32-bit device ID value
 */
uint32_t dw3000_port_read_device_id(void);

/**
 * @brief Read DW3000 IC temperature
 * @return Temperature in degrees Celsius
 */
float dw3000_port_read_temperature(void);

/**
 * @brief Read DW3000 IC voltage
 * @return Voltage in volts
 */
float dw3000_port_read_voltage(void);

/**
 * @brief Read both temperature and voltage in single register access (optimized)
 * @param temperature Pointer to store temperature (can be NULL)
 * @param voltage Pointer to store voltage (can be NULL)
 */
void dw3000_port_read_temp_and_voltage(float *temperature, float *voltage);

/**
 * @brief Set the 802.15.4 PAN ID
 * @param pan_id 16-bit PAN identifier
 */
void dw3000_port_set_pan_id(uint16_t pan_id);

/**
 * @brief Set the 802.15.4 short address (16-bit)
 * @param address 16-bit short address for this device
 */
void dw3000_port_set_address(uint16_t address);

/**
 * @brief Configure DW3000 for basic message transmission
 * @param channel UWB channel (5 or 9 recommended)
 * @return DW3000_SUCCESS on success, DW3000_ERROR on failure
 */
int dw3000_port_configure_tx(uint8_t channel);

/**
 * @brief Configure DW3000 for message reception
 * @param channel UWB channel (must match transmitter)
 * @return DW3000_SUCCESS on success, DW3000_ERROR on failure
 */
int dw3000_port_configure_rx(uint8_t channel);

/**
 * @brief Send a message via UWB
 * @param data Pointer to data buffer to transmit
 * @param length Length of data in bytes (max 127)
 * @return DW3000_SUCCESS on success, DW3000_ERROR on failure
 */
int dw3000_port_send_message(const uint8_t *data, uint16_t length);

/**
 * @brief Check if a message has been received
 * @param data Pointer to buffer to store received data
 * @param max_length Maximum buffer size
 * @param received_length Pointer to store actual received length
 * @return DW3000_SUCCESS if message received, DW3000_ERROR otherwise
 */
int dw3000_port_receive_message(uint8_t *data, uint16_t max_length, uint16_t *received_length);

/*---------------------------------------------------------------------------
 * Platform Compatibility Functions (required by Qorvo driver)
 *---------------------------------------------------------------------------*/

/**
 * @brief Microsecond delay
 * @param time_us Delay time in microseconds
 */
void deca_usleep(unsigned long time_us);

/**
 * @brief Millisecond delay
 * @param time_ms Delay time in milliseconds
 */
void deca_sleep(unsigned int time_ms);

/**
 * @brief Lock mutex for DW3000 access
 * @return IRQ status before disabling interrupts
 */
decaIrqStatus_t decamutexon(void);

/**
 * @brief Unlock mutex for DW3000 access
 * @param s IRQ status to restore
 */
void decamutexoff(decaIrqStatus_t s);

/*---------------------------------------------------------------------------
 * @file    uwb_port.h
 * @brief   Port layer for UWB radio driver - Adapter between driver and platform
 *
 * @note Thread Safety: This implementation is NOT thread-safe. Callers must
 *       ensure exclusive access to UWB operations through external synchronization.
 *
 * @note Initialization: Must call uwb_port_init() before any other functions.
 *       Hardware SPI peripheral must be initialized before calling uwb_port_init().
 *---------------------------------------------------------------------------*/
#pragma once

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "common.h"
#include "deca_interface.h" // Required for decaIrqStatus_t type
#include "platform_spi.h"

/*---------------------------------------------------------------------------
 * Forward Declarations
 *---------------------------------------------------------------------------*/
/**
 * Opaque UWB device handle - application code cannot access internals.
 * Actual structure is defined in uwb_port.c to hide implementation details.
 */
typedef struct uwb_dev_s uwb_dev_t;

/*---------------------------------------------------------------------------
 * Defines - UWB Port Configuration
 *---------------------------------------------------------------------------*/
#define UWB_PORT_CS_PIN PLATFORM_SPI_CS_UWB
#define UWB_MAX_MESSAGE_LENGTH (127U) ///< Maximum UWB message length per IEEE 802.15.4

/** Return codes from port layer functions */
typedef enum
{
    UWB_PORT_SUCCESS = 0,           ///< Operation completed successfully
    UWB_PORT_ERROR_NULL_PTR = -1,   ///< NULL pointer passed to function
    UWB_PORT_ERROR_COMM_FAIL = -2,  ///< SPI communication failure
    UWB_PORT_ERROR_INVALID_ID = -3, ///< Device ID does not match expected value
    UWB_PORT_ERROR_TIMEOUT = -4,    ///< Operation timed out
    UWB_PORT_ERROR_INIT_FAIL = -5,  ///< Device initialization failed
    UWB_PORT_ERROR_CONFIG = -6,     ///< Invalid configuration parameter
    UWB_PORT_ERROR_TX_FAIL = -7,    ///< Transmission failed
    UWB_PORT_ERROR_RX_FAIL = -8,    ///< Reception failed
    UWB_PORT_ERROR_NO_DATA = -9,    ///< No data available
    UWB_PORT_ERROR_UNKNOWN = -99    ///< Unknown/unspecified error
} uwb_port_status_t;

/** UWB channel configurations */
typedef enum
{
    UWB_CHANNEL_1 = 1, ///< 3494.4 MHz center frequency
    UWB_CHANNEL_2 = 2, ///< 3993.6 MHz
    UWB_CHANNEL_3 = 3, ///< 4492.8 MHz
    UWB_CHANNEL_4 = 4, ///< 3993.6 MHz
    UWB_CHANNEL_5 = 5, ///< 6489.6 MHz (recommended)
    UWB_CHANNEL_7 = 7, ///< 6489.6 MHz
    UWB_CHANNEL_9 = 9  ///< 7987.2 MHz (recommended)
} uwb_channel_t;

/*---------------------------------------------------------------------------
 * Public Function Prototypes
 *---------------------------------------------------------------------------*/

/**
 * @brief Initialize the UWB port layer
 *
 * Initializes the static UWB device structure with SPI callbacks and interface settings.
 * Must be called before any other UWB port functions.
 *
 * @pre SPI peripheral hardware must be initialized
 *
 * @return Pointer to UWB device handle on success, NULL on failure
 *
 * @note This function does NOT probe or configure the UWB hardware.
 *       Call uwb_port_probe_and_init() to complete initialization.
 */
uwb_dev_t* uwb_port_init(void);

/**
 * @brief Probe and initialize the UWB device
 *
 * Probes the hardware to detect device type and initializes the radio.
 * Reads calibration data from OTP memory.
 *
 * @param[in] dev Pointer to UWB device handle from uwb_port_init()
 *
 * @pre dev must not be NULL
 * @pre uwb_port_init() must have been called successfully
 *
 * @return UWB_PORT_SUCCESS on success
 * @return UWB_PORT_ERROR_NULL_PTR if dev is NULL
 * @return UWB_PORT_ERROR_COMM_FAIL if SPI communication fails
 * @return UWB_PORT_ERROR_INIT_FAIL if device initialization fails
 */
uwb_port_status_t uwb_port_probe_and_init(uwb_dev_t* dev);

/**
 * @brief Wake up the UWB device from sleep mode
 *
 * @param[in] dev Pointer to UWB device handle
 *
 * @pre dev must not be NULL
 */
void uwb_port_wakeup_device(uwb_dev_t* dev);

/**
 * @brief Perform a soft reset on the UWB device
 *
 * Performs a software reset of the UWB chip. This should be called after
 * the device has been probed (to identify the chip type) but before full
 * initialization.
 *
 * @param[in] dev Pointer to UWB device handle
 *
 * @pre dev must not be NULL
 * @pre Device must have been probed (uwb_port_probe_and_init() must have been called)
 *
 * @return UWB_PORT_SUCCESS on success
 * @return UWB_PORT_ERROR_NULL_PTR if dev is NULL
 */
uwb_port_status_t uwb_port_soft_reset(uwb_dev_t* dev);

/**
 * @brief Check if UWB device ID matches expected value
 *
 * @param[in] dev Pointer to UWB device handle
 *
 * @pre dev must not be NULL
 *
 * @return UWB_PORT_SUCCESS if device ID is valid
 * @return UWB_PORT_ERROR_NULL_PTR if dev is NULL
 * @return UWB_PORT_ERROR_INVALID_ID if device ID doesn't match
 */
uwb_port_status_t uwb_port_check_device_id(uwb_dev_t* dev);

/**
 * @brief Read UWB device ID register
 *
 * @param[in] dev Pointer to UWB device handle
 *
 * @pre dev must not be NULL
 *
 * @return 32-bit device ID value, or 0 on error
 */
uint32_t uwb_port_read_device_id(uwb_dev_t* dev);

/**
 * @brief Read UWB IC temperature
 *
 * @param[in] dev Pointer to UWB device handle
 *
 * @pre dev must not be NULL
 *
 * @return Temperature in degrees Celsius, or 0.0f on error
 */
float uwb_port_read_temperature(uwb_dev_t* dev);

/**
 * @brief Read UWB IC voltage
 *
 * @param[in] dev Pointer to UWB device handle
 *
 * @pre dev must not be NULL
 *
 * @return Voltage in volts, or 0.0f on error
 */
float uwb_port_read_voltage(uwb_dev_t* dev);

/**
 * @brief Read both temperature and voltage in single register access (optimized)
 *
 * @param[in] dev Pointer to UWB device handle
 * @param[out] temperature Pointer to store temperature (can be NULL)
 * @param[out] voltage Pointer to store voltage (can be NULL)
 *
 * @pre dev must not be NULL
 */
void uwb_port_read_temp_and_voltage(uwb_dev_t* dev, float* temperature, float* voltage);

/**
 * @brief Set the 802.15.4 PAN ID
 *
 * @param[in] dev Pointer to UWB device handle
 * @param[in] pan_id 16-bit PAN identifier
 *
 * @pre dev must not be NULL
 */
void uwb_port_set_pan_id(uwb_dev_t* dev, uint16_t pan_id);

/**
 * @brief Set the 802.15.4 short address (16-bit)
 *
 * @param[in] dev Pointer to UWB device handle
 * @param[in] address 16-bit short address for this device
 *
 * @pre dev must not be NULL
 */
void uwb_port_set_address(uwb_dev_t* dev, uint16_t address);

/**
 * @brief Configure UWB radio for bidirectional communication
 *
 * Configures the radio parameters and enables the receiver. After this call,
 * the device is ready to both transmit and receive messages. All nodes should
 * be configured identically.
 *
 * @param[in] dev Pointer to UWB device handle
 * @param[in] channel UWB channel (see uwb_channel_t)
 *
 * @pre dev must not be NULL
 *
 * @return UWB_PORT_SUCCESS on success
 * @return UWB_PORT_ERROR_NULL_PTR if dev is NULL
 * @return UWB_PORT_ERROR_CONFIG if channel is invalid
 * @return UWB_PORT_ERROR_COMM_FAIL if configuration fails
 */
uwb_port_status_t uwb_port_configure(uwb_dev_t* dev, uwb_channel_t channel);

/**
 * @brief Send a message via UWB
 *
 * @param[in] dev Pointer to UWB device handle
 * @param[in] data Pointer to data buffer to transmit
 * @param[in] length Length of data in bytes (max UWB_MAX_MESSAGE_LENGTH)
 *
 * @pre dev must not be NULL
 * @pre data must not be NULL
 * @pre length must be > 0 and <= UWB_MAX_MESSAGE_LENGTH
 *
 * @return UWB_PORT_SUCCESS on success
 * @return UWB_PORT_ERROR_NULL_PTR if dev or data is NULL
 * @return UWB_PORT_ERROR_CONFIG if length is invalid
 * @return UWB_PORT_ERROR_TX_FAIL if transmission fails
 *
 * @note TX timestamp can be read after this call using uwb_port_get_last_tx_timestamp()
 */
uwb_port_status_t uwb_port_send_message(uwb_dev_t* dev, const uint8_t* data, uint16_t length);

/**
 * @brief Send message with delayed transmission at a specific time
 *
 * Sends a message using DW3000 delayed TX mode. The tx_timestamp_dtu parameter
 * specifies the HIGH 32-BITS of the 40-bit system time counter at which to transmit.
 * The DW3000 hardware uses this to schedule the transmission.
 *
 * @param[in] dev Pointer to UWB device handle
 * @param[in] data Pointer to message data (including frame header)
 * @param[in] length Length of data including 802.15.4 header
 * @param[in] tx_timestamp_dtu HIGH 32-BITS of the 40-bit device time (Device Time Units)
 *           The low 8 bits are ignored by hardware. Units: ~8 ns per count.
 *
 * @pre dev must not be NULL
 * @pre data must not be NULL
 * @pre length must be valid (see uwb_port_send_message for constraints)
 * @pre tx_timestamp_dtu must be at least 500µs in the future
 *
 * @return UWB_PORT_SUCCESS if message sent successfully
 * @return UWB_PORT_ERROR_NULL_PTR if dev or data is NULL
 * @return UWB_PORT_ERROR_CONFIG if length is invalid
 * @return UWB_PORT_ERROR_TX_FAIL if transmission fails (scheduled time has passed)
 */
uwb_port_status_t uwb_port_send_message_delayed(uwb_dev_t* dev, const uint8_t* data,
                                                uint16_t length, uint64_t tx_timestamp_dtuh);

/**
 * @brief Check if a message has been received
 *
 * @param[in] dev Pointer to UWB device handle
 * @param[out] data Pointer to buffer to store received data
 * @param[in] max_length Maximum buffer size
 * @param[out] received_length Pointer to store actual received length
 *
 * @pre dev must not be NULL
 * @pre data must not be NULL
 * @pre received_length must not be NULL
 *
 * @return UWB_PORT_SUCCESS if message received
 * @return UWB_PORT_ERROR_NULL_PTR if any pointer is NULL
 * @return UWB_PORT_ERROR_NO_DATA if no message available
 * @return UWB_PORT_ERROR_RX_FAIL if reception failed
 *
 * @note RX timestamp can be read after this call using uwb_port_get_last_rx_timestamp()
 */
uwb_port_status_t uwb_port_receive_message(uwb_dev_t* dev, uint8_t* data, uint16_t max_length,
                                           uint16_t* received_length);

/*---------------------------------------------------------------------------
 * System Time Functions
 *---------------------------------------------------------------------------*/

/**
 * @brief Read the current device system time counter
 * @details Reads the 40-bit system time counter from the DW3000.
 *          The counter runs at 249.6 MHz (DTU frequency) and is used
 *          for precise timing of TX/RX operations and delayed transmissions.
 *
 * @return Current device time in DTU (Device Time Units)
 */
uint64_t uwb_port_read_device_time(void);

/*---------------------------------------------------------------------------
 * Timestamp Functions (for TWR ranging)
 *---------------------------------------------------------------------------*/

/**
 * @brief Read TX timestamp from last transmission
 * @details Reads the 40-bit timestamp of when the last frame was transmitted.
 *          Must be called shortly after uwb_port_send_message() completes.
 *
 * @param[in] dev Pointer to UWB device handle
 * @param[out] timestamp_bytes 5-byte buffer to store timestamp (40-bit)
 *
 * @pre dev must not be NULL
 * @pre timestamp_bytes must not be NULL
 * @pre Must be called after successful TX
 *
 * @return UWB_PORT_SUCCESS on success
 * @return UWB_PORT_ERROR_NULL_PTR if pointers are NULL
 */
uwb_port_status_t uwb_port_read_tx_timestamp(uwb_dev_t* dev, uint8_t timestamp_bytes[5]);

/**
 * @brief Read RX timestamp from last received frame
 * @details Reads the 40-bit timestamp of when the last frame was received.
 *          Must be called shortly after uwb_port_receive_message() returns success.
 *
 * @param[in] dev Pointer to UWB device handle
 * @param[out] timestamp_bytes 5-byte buffer to store timestamp (40-bit)
 *
 * @pre dev must not be NULL
 * @pre timestamp_bytes must not be NULL
 * @pre Must be called after successful RX
 *
 * @return UWB_PORT_SUCCESS on success
 * @return UWB_PORT_ERROR_NULL_PTR if pointers are NULL
 */
uwb_port_status_t uwb_port_read_rx_timestamp(uwb_dev_t* dev, uint8_t timestamp_bytes[5]);

/**
 * @brief Get last TX timestamp as 64-bit value
 * @details Convenience function that returns the last TX timestamp.
 *          Automatically called after successful transmission.
 *
 * @param[in] dev Pointer to UWB device handle
 * @return 64-bit timestamp (40-bit value in lower bits), or 0 on error
 */
uint64_t uwb_port_get_last_tx_timestamp(uwb_dev_t* dev);

/**
 * @brief Get last RX timestamp as 64-bit value
 * @details Convenience function that returns the last RX timestamp.
 *          Automatically captured during reception.
 *
 * @param[in] dev Pointer to UWB device handle
 * @return 64-bit timestamp (40-bit value in lower bits), or 0 on error
 */
uint64_t uwb_port_get_last_rx_timestamp(uwb_dev_t* dev);

/*---------------------------------------------------------------------------
 * Time Conversion Functions
 *
 * @brief Convert between DW3000 Device Time Units (DTU) and human-readable time.
 *        These functions abstract away DTU implementation details from the rest
 *        of the application. All DTU-related knowledge is contained in the port layer.
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
 * Time Unit Conversion Macros
 *
 * Note: These macros use constants from the DW3000 vendor driver:
 *  - DW3000_DTU_FREQ = 249.6 MHz (used for DTUH/DTUC time units)
 *  - Full DTU frequency = 499.2 MHz x 128 = 63.9 GHz
 *---------------------------------------------------------------------------*/

/** @brief Convert DTUH (249.6 MHz) to milliseconds */
#define UWB_DTUH_TO_MS(dtuh_ticks) (((dtuh_ticks) * 1000ULL) / DW3000_DTU_FREQ)

/** @brief Convert DTUH (249.6 MHz) to microseconds */
#define UWB_DTUH_TO_US(dtuh_ticks) (((dtuh_ticks) * 1000000ULL) / DW3000_DTU_FREQ)

/** @brief Convert DTUH (249.6 MHz) to seconds */
#define UWB_DTUH_TO_S(dtuh_ticks) ((dtuh_ticks) / DW3000_DTU_FREQ)

/** @brief Convert milliseconds to DTUH (249.6 MHz) */
#define UWB_MS_TO_DTUH(milliseconds) (((uint64_t)(milliseconds)) * (DW3000_DTU_FREQ / 1000ULL))

/** @brief Convert microseconds to DTUH (249.6 MHz) */
#define UWB_US_TO_DTUH(microseconds) ((((uint64_t)(microseconds)) * DW3000_DTU_FREQ) / 1000000ULL)

/** @brief Convert DTUH to full DTU (DTUH << 8) */
#define UWB_DTUH_TO_DTU(dtuh_ticks) (((dtuh_ticks) << 8))

/** @brief Convert full DTU to DTUH (DTU >> 8) */
#define UWB_DTU_TO_DTUH(dtu_ticks) ((uint32_t)(((dtu_ticks) >> 8)))

/** @brief Convert milliseconds to full DTU (63.9 GHz), preserves sub-DTUH precision */
#define UWB_MS_TO_DTU(milliseconds)                                                                \
    ((((uint64_t)(milliseconds)) * ((DW3000_DTU_FREQ * 128ULL) / 1000ULL)))

/** @brief Convert microseconds to full DTU (63.9 GHz), preserves sub-DTUH precision */
#define UWB_US_TO_DTU(microseconds)                                                                \
    ((((uint64_t)(microseconds)) * (DW3000_DTU_FREQ * 128ULL)) / 1000000ULL)

/*---------------------------------------------------------------------------
 * Platform Compatibility Functions (required by Qorvo DW3000 driver)
 *
 * @note These functions are required by the vendor driver library and must
 *       use the exact names specified by Qorvo. They provide OS abstraction.
 *---------------------------------------------------------------------------*/

/**
 * @brief Microsecond delay (required by Qorvo driver)
 * @param time_us Delay time in microseconds
 */
void deca_usleep(unsigned long time_us);

/**
 * @brief Millisecond delay (required by Qorvo driver)
 * @param time_ms Delay time in milliseconds
 */
void deca_sleep(unsigned int time_ms);

/**
 * @brief Lock mutex for UWB access (required by Qorvo driver)
 * @return IRQ status before disabling interrupts
 */
decaIrqStatus_t decamutexon(void);

/**
 * @brief Unlock mutex for UWB access (required by Qorvo driver)
 * @param s IRQ status to restore
 */
void decamutexoff(decaIrqStatus_t s);

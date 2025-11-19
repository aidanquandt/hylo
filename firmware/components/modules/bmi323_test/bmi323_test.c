/*---------------------------------------------------------------------------
 * @file    bmi323_test.c
 * @brief   BMI323 hardware connection test module
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "bmi323_test.h"
#include "module.h"
#include "platform_gpio.h"
#include "bmi323_port.h"
#include "state_machine.h"

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define BMI323_EXPECTED_CHIP_ID_1  (0x43U)  // BMI323 Chip ID
#define BMI323_EXPECTED_CHIP_ID_2  (0x44U)  // BMI330 Chip ID
#define STARTUP_DELAY_MS           (2000U)  // Wait 2 seconds before probing

// State machine states
typedef enum {
    STATE_STARTUP,              // Waiting for startup delay
    STATE_INITIALIZATION,       // Initializing hardware
    STATE_ACTIVE                // Normal operation - reading sensors
} bmi323_state_E;

/*---------------------------------------------------------------------------
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
STATIC bool verify_chip_id(void);
STATIC void read_sensors(void);

// State machine transition logic
STATIC uint16_t bmi323_test_transition_logic(uint16_t currentState, uint32_t stateTimer);

// State handlers
STATIC void bmi323_test_state_startup_process(void);
STATIC void bmi323_test_state_initialization_on_entry(uint16_t prevState);
STATIC void bmi323_test_state_active_process(void);

/*---------------------------------------------------------------------------
 * Module Functions
 *---------------------------------------------------------------------------*/
STATIC void bmi323_test_init(void);
STATIC void bmi323_test_process_1Hz(void);

extern const module_S bmi323_test_module;
const module_S bmi323_test_module = {
    .module_init = bmi323_test_init,
    .module_process_1Hz = bmi323_test_process_1Hz,
};

/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/
// State machine definition
STATIC const state_s bmi323_states[] = {
    [STATE_STARTUP] = {
        .process = bmi323_test_state_startup_process,
        .onEntry = NULL,
        .onExit = NULL
    },
    [STATE_INITIALIZATION] = {
        .process = NULL,
        .onEntry = bmi323_test_state_initialization_on_entry,
        .onExit = NULL
    },
    [STATE_ACTIVE] = {
        .process = bmi323_test_state_active_process,
        .onEntry = NULL,
        .onExit = NULL
    }
};

STATIC state_machine_s bmi323_state_machine = {
    .prev_state = STATE_STARTUP,
    .curr_state = STATE_STARTUP,
    .next_state = STATE_STARTUP,
    .timer = 0,
    .transitionLogic = bmi323_test_transition_logic,
    .states = bmi323_states
};

// Hardware state
STATIC struct bmi3_dev *bmi_dev = NULL;
STATIC uint8_t chip_id = 0;
STATIC float temperature = 0.0f;
STATIC bool hardware_ready = false;

// Debug variables
STATIC uint8_t raw_read_result[4] = {0};
STATIC int probe_result = 0;  // Store probe_and_init return value
STATIC uint8_t spi_test_tx[4] = {0x80, 0x00, 0x00, 0x00};  // Test SPI transaction
STATIC uint8_t spi_test_rx[4] = {0};
STATIC uint8_t chip_id_raw[4] = {0};  // Raw chip ID read result

// Sensor data (watch these in debugger)
STATIC bmi323_sensor_data_t accel_data = {0};
STATIC bmi323_sensor_data_t gyro_data = {0};
STATIC uint32_t sensor_reads = 0;  // Counter for number of sensor reads

/*---------------------------------------------------------------------------
 * Private Function Implementations
 *---------------------------------------------------------------------------*/

STATIC bool verify_chip_id(void)
{
    if (bmi_dev == NULL) {
        return false;
    }
    
    // Check the chip ID through port layer
    int ret = bmi323_port_check_device_id(bmi_dev);
    if (ret != BMI323_SUCCESS) {
        return false;
    }
    
    // Read and store chip ID
    chip_id = bmi323_port_read_chip_id(bmi_dev);
    
    // Verify it matches expected value (BMI323 or BMI330)
    return (chip_id == BMI323_EXPECTED_CHIP_ID_1 || chip_id == BMI323_EXPECTED_CHIP_ID_2);
}

STATIC void read_sensors(void)
{
    if (bmi_dev == NULL || !hardware_ready) {
        return;
    }
    
    // Read temperature
    temperature = bmi323_port_read_temperature(bmi_dev);
    
    // Read both accelerometer and gyroscope in single optimized call
    int ret = bmi323_port_read_accel_and_gyro(bmi_dev, &accel_data, &gyro_data);
    if (ret == BMI323_SUCCESS) {
        sensor_reads++;
    }
}

STATIC void bmi323_test_init(void)
{
    // State machine is already initialized with static values
}

STATIC void bmi323_test_process_1Hz(void)
{
    // Read sensors if hardware is ready
    if (hardware_ready) {
        read_sensors();
    }
    
    // Run state machine at 1Hz
    state_machine_periodic(&bmi323_state_machine);
}

STATIC uint16_t bmi323_test_transition_logic(uint16_t currentState, uint32_t stateTimer)
{
    uint16_t nextState = currentState;
    
    switch (currentState) {
        case STATE_STARTUP:
            // Transition to INITIALIZATION after startup delay
            if (stateTimer >= MS_TO_S(STARTUP_DELAY_MS)) {
                nextState = STATE_INITIALIZATION;
            }
            break;
            
        case STATE_INITIALIZATION:
            // Transition to ACTIVE if hardware is ready
            if (hardware_ready) {
                nextState = STATE_ACTIVE;
            }
            break;
            
        case STATE_ACTIVE:
            // Stay in ACTIVE state
            nextState = STATE_ACTIVE;
            break;
            
        default:
            nextState = STATE_STARTUP;
            break;
    }
    
    return nextState;
}

STATIC void bmi323_test_state_startup_process(void)
{
    // Nothing to do - timer automatically increments
}

STATIC void bmi323_test_state_initialization_on_entry(uint16_t prevState)
{
    (void)prevState;  // Unused
    
    // Get the port device structure
    bmi_dev = bmi323_port_init();
    if (bmi_dev == NULL) {
        probe_result = -999;  // Debug: port_init failed
        return;
    }
    
    // Debug: Try raw SPI transaction BEFORE anything else
    // Force verify CS pin value
    volatile platform_spi_cs_E test_cs_pin = BMI323_CS_PIN;  // Should be 1
    (void)test_cs_pin;  // Prevent optimization
    
    platform_spi_cs_low(BMI323_CS_PIN);
    platform_spi_transmit(spi_test_tx, 4);
    platform_spi_cs_high(BMI323_CS_PIN);
    // Don't delay here - causes task switch that corrupts current_spi
    
    // Try another raw read
    platform_spi_cs_low(BMI323_CS_PIN);
    uint8_t cmd = 0x80;  // Read chip ID
    platform_spi_transmit(&cmd, 1);
    platform_spi_receive(spi_test_rx, 3);
    platform_spi_cs_high(BMI323_CS_PIN);
    
    // Also try reading with TransmitReceive to see the difference
    platform_spi_cs_low(BMI323_CS_PIN);
    uint8_t tx_buf[3] = {0x80, 0x00, 0x00};
    platform_spi_transfer(tx_buf, chip_id_raw, 3);
    platform_spi_cs_high(BMI323_CS_PIN);
    
    // Probe and initialize the device (this handles the mode switch)
    probe_result = bmi323_port_probe_and_init(bmi_dev);
    if (probe_result != BMI323_SUCCESS) {
        return;
    }
    
    // Debug: Raw SPI read AFTER mode switch
    platform_spi_cs_low(BMI323_CS_PIN);
    uint8_t cmd2 = 0x81;  // Read register 0x01 (error register)
    platform_spi_transmit(&cmd2, 1);
    platform_spi_receive(raw_read_result, 3);
    platform_spi_cs_high(BMI323_CS_PIN);
    
    // Verify chip ID
    if (!verify_chip_id()) {
        return;
    }
    
    hardware_ready = true;
    
    // Configure accelerometer: ±2g range, 100 Hz ODR
    bmi323_port_configure_accel(bmi_dev, BMI3_ACC_RANGE_2G, BMI3_ACC_ODR_100HZ);
    
    // Configure gyroscope: ±2000 deg/s range, 100 Hz ODR
    bmi323_port_configure_gyro(bmi_dev, BMI3_GYR_RANGE_2000DPS, BMI3_GYR_ODR_100HZ);
    
    // Read initial measurements
    read_sensors();
}

STATIC void bmi323_test_state_active_process(void)
{
    // Sensor reading happens in 1Hz periodic callback
    // This state is just to show the system is running
}

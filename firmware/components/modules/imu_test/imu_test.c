/*---------------------------------------------------------------------------
 * @file    imu_test.c
 * @brief   IMU hardware connection test module
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "imu_test.h"
#include "module.h"
#include "platform_gpio.h"
#include "imu_port.h"
#include "state_machine.h"

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define IMU_EXPECTED_CHIP_ID_1  (0x43U)  // BMI323 Chip ID
#define IMU_EXPECTED_CHIP_ID_2  (0x44U)  // BMI330 Chip ID
#define STARTUP_DELAY_MS        (2000U)  // Wait 2 seconds before probing

// State machine states
typedef enum {
    STATE_STARTUP,              // Waiting for startup delay
    STATE_INITIALIZATION,       // Initializing hardware
    STATE_ACTIVE                // Normal operation - reading sensors
} imu_state_E;

/*---------------------------------------------------------------------------
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
STATIC bool verify_chip_id(void);
STATIC void read_sensors(void);

// State machine transition logic
STATIC uint16_t imu_test_transition_logic(uint16_t currentState, uint32_t stateTimer);

// State handlers
STATIC void imu_test_state_startup_process(void);
STATIC void imu_test_state_initialization_on_entry(uint16_t prevState);
STATIC void imu_test_state_active_process(void);

/*---------------------------------------------------------------------------
 * Module Functions
 *---------------------------------------------------------------------------*/
STATIC void imu_test_init(void);
STATIC void imu_test_process_1Hz(void);

extern const module_S imu_test_module;
const module_S imu_test_module = {
    .module_init = imu_test_init,
    .module_process_1Hz = imu_test_process_1Hz,
};

/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/
// State machine definition
STATIC const state_s imu_states[] = {
    [STATE_STARTUP] = {
        .process = imu_test_state_startup_process,
        .onEntry = NULL,
        .onExit = NULL
    },
    [STATE_INITIALIZATION] = {
        .process = NULL,
        .onEntry = imu_test_state_initialization_on_entry,
        .onExit = NULL
    },
    [STATE_ACTIVE] = {
        .process = imu_test_state_active_process,
        .onEntry = NULL,
        .onExit = NULL
    }
};

STATIC state_machine_s imu_state_machine = {
    .prev_state = STATE_STARTUP,
    .curr_state = STATE_STARTUP,
    .next_state = STATE_STARTUP,
    .timer = 0,
    .transitionLogic = imu_test_transition_logic,
    .states = imu_states
};

// Hardware state
STATIC imu_dev_t *imu_dev = NULL;
STATIC uint8_t chip_id = 0;
STATIC float temperature = 0.0f;
STATIC bool hardware_ready = false;

STATIC int probe_result = 0;  // Store probe_and_init return value

STATIC imu_sensor_data_t accel_data = {0};
STATIC imu_sensor_data_t gyro_data = {0};
STATIC uint32_t sensor_reads = 0;  // Counter for number of sensor reads

/*---------------------------------------------------------------------------
 * Private Function Implementations
 *---------------------------------------------------------------------------*/

STATIC bool verify_chip_id(void)
{
    if (imu_dev == NULL) {
        return false;
    }
    
    // Check the chip ID through port layer
    int ret = imu_port_check_device_id(imu_dev);
    if (ret != IMU_PORT_SUCCESS) {
        return false;
    }
    
    // Read and store chip ID
    chip_id = imu_port_read_chip_id(imu_dev);
    
    // Verify it matches expected value (BMI323 or BMI330)
    return (chip_id == IMU_EXPECTED_CHIP_ID_1 || chip_id == IMU_EXPECTED_CHIP_ID_2);
}

STATIC void read_sensors(void)
{
    if (imu_dev == NULL || !hardware_ready) {
        return;
    }
    
    // Read temperature
    temperature = imu_port_read_temperature(imu_dev);
    
    // Read both accelerometer and gyroscope in single optimized call
    int ret = imu_port_read_accel_and_gyro(imu_dev, &accel_data, &gyro_data);
    if (ret == IMU_PORT_SUCCESS) {
        sensor_reads++;
    }
}

STATIC void imu_test_init(void)
{
    // State machine is already initialized with static values
}

STATIC void imu_test_process_1Hz(void)
{
    // Read sensors if hardware is ready
    if (hardware_ready) {
        read_sensors();
    }

    // Run state machine at 1Hz
    state_machine_periodic(&imu_state_machine);
}

STATIC uint16_t imu_test_transition_logic(uint16_t currentState, uint32_t stateTimer)
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

STATIC void imu_test_state_startup_process(void)
{
    // Nothing to do - timer automatically increments
}

STATIC void imu_test_state_initialization_on_entry(uint16_t prevState)
{   
    (void)prevState;  // Unused
    
    // Get the port device structure
    imu_dev = imu_port_init();
    if (imu_dev == NULL) {
        probe_result = -999;  // Debug: port_init failed
        return;
    }
    // Probe and initialize the device (this handles the mode switch and SPI setup)
    probe_result = imu_port_probe_and_init(imu_dev);
    if (probe_result != IMU_PORT_SUCCESS) {
        return;
    }
    
    // Verify chip ID through port layer
    if (!verify_chip_id()) {
        return;
    }
    
    hardware_ready = true;
    
    // Configure accelerometer: ±2g range, 100 Hz ODR
    imu_port_configure_accel(imu_dev, IMU_ACCEL_RANGE_2G, IMU_ODR_100HZ);
    
    // Configure gyroscope: ±2000 deg/s range, 100 Hz ODR
    imu_port_configure_gyro(imu_dev, IMU_GYRO_RANGE_2000DPS, IMU_ODR_100HZ);
    
    // Read initial measurements
    read_sensors();
}

STATIC void imu_test_state_active_process(void)
{
    // Sensor reading happens in 1Hz periodic callback
    // This state is just to show the system is running
}

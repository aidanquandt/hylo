/*---------------------------------------------------------------------------
 * @file    imu.c
 * @brief   IMU hardware connection test module
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "imu.h"
#include "module.h"
#include "platform_gpio.h"
#include "imu_port.h"
#include "state_machine.h"
#include "uart_manager.h"
#include "uart_cmd_router.h"
#include <string.h>
#include <stdlib.h>

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
    STATE_ACTIVE,               // Normal operation - reading sensors
    STATE_FAULTED               // Error state - initialization or communication failed
} imu_state_E;

// Fault codes
typedef enum {
    FAULT_NONE = 0,
    FAULT_INIT_NULL_DEV,        // Device init returned NULL
    FAULT_PROBE_FAILED,         // Probe and init failed
    FAULT_CHIP_ID_INVALID,      // Chip ID verification failed
    FAULT_ACCEL_CONFIG_FAILED,  // Accelerometer config failed
    FAULT_GYRO_CONFIG_FAILED,   // Gyroscope config failed
    FAULT_READ_FAILED           // Sensor read failed in active state
} imu_fault_code_e;

typedef struct {
    bool fault_present;
    bool init_device_completed;
} imu_state_machine_inputs_t;

// IMU measurement data
typedef struct {
    imu_sensor_data_t accel;
    imu_sensor_data_t gyro;
    float temperature;
    uint8_t chip_id;
} imu_measurements_t;

/*---------------------------------------------------------------------------
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
STATIC bool verify_chip_id(void);
STATIC void read_sensors(void);
STATIC void imu_state_machine_sample_inputs(void);
STATIC uint16_t imu_transition_logic(uint16_t currentState, uint32_t stateTimer);
STATIC void imu_state_initialization_on_entry(uint16_t prevState);
STATIC void imu_state_active_process(void);
STATIC void imu_state_faulted_on_entry(uint16_t prevState);

/*---------------------------------------------------------------------------
 * Module Functions
 *---------------------------------------------------------------------------*/
STATIC void imu_init(void);
STATIC void imu_process_100Hz(void);

extern const module_S imu_module;
// Forward declaration of command handler
STATIC bool imu_cmd_handler(const cmd_parsed_t *parsed);

const module_S imu_module = {
    .module_name = "imu",
    .module_init = imu_init,
    .module_process_100Hz = imu_process_100Hz,
    .module_cmd_handler = imu_cmd_handler,
};

/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/
// State machine definition
STATIC const state_s imu_states[] = {
    [STATE_STARTUP] = {
        .process = NULL,
        .onEntry = NULL,
        .onExit = NULL
    },
    [STATE_INITIALIZATION] = {
        .process = NULL,
        .onEntry = imu_state_initialization_on_entry,
        .onExit = NULL
    },
    [STATE_ACTIVE] = {
        .process = imu_state_active_process,
        .onEntry = NULL,
        .onExit = NULL
    },
    [STATE_FAULTED] = {
        .process = NULL,
        .onEntry = imu_state_faulted_on_entry,
        .onExit = NULL
    }
};

STATIC state_machine_s imu_state_machine = {
    .prev_state = STATE_STARTUP,
    .curr_state = STATE_STARTUP,
    .next_state = STATE_STARTUP,
    .timer = 0,
    .transitionLogic = imu_transition_logic,
    .states = imu_states
};

// Hardware state
STATIC imu_dev_t *imu_dev = NULL;
STATIC imu_state_machine_inputs_t imu_state_machine_inputs = {0};
STATIC imu_measurements_t measurements = {0};
STATIC imu_fault_code_e imu_fault_code = FAULT_NONE;

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
    measurements.chip_id = imu_port_read_chip_id(imu_dev);
    
    // Verify it matches expected value (BMI323 or BMI330)
    return (measurements.chip_id == IMU_EXPECTED_CHIP_ID_1 || measurements.chip_id == IMU_EXPECTED_CHIP_ID_2);
}

STATIC void read_sensors(void)
{
    if (imu_dev == NULL) {
        imu_fault_code = FAULT_READ_FAILED;
        return;
    }
    
    // Read temperature
    // maybe we can poll this one at lower rate? - leaving for now
    measurements.temperature = imu_port_read_temperature(imu_dev);
    
    // Read both accelerometer and gyroscope in single optimized call
    int result = imu_port_read_accel_and_gyro(imu_dev, &measurements.accel, &measurements.gyro);
    if (result != IMU_PORT_SUCCESS) {
        imu_fault_code = FAULT_READ_FAILED;
    }
}

STATIC void imu_init(void)
{
    // State machine is already initialized with static values
}

STATIC void imu_process_100Hz(void)
{
    // Run state machine at 100Hz
    imu_state_machine_sample_inputs();
    state_machine_periodic(&imu_state_machine);
}

STATIC void imu_state_machine_sample_inputs(void)
{
    imu_state_machine_inputs.fault_present = (imu_fault_code != FAULT_NONE);
    imu_state_machine_inputs.init_device_completed = (imu_dev != NULL);
}

STATIC uint16_t imu_transition_logic(uint16_t currentState, uint32_t stateTimer)
{
    uint16_t nextState = currentState;
    
    switch (currentState) {
        case STATE_STARTUP:
            // Transition to INITIALIZATION after startup delay (200 ticks at 100Hz = 2000ms)
            if (stateTimer >= MS_TO_100HZ_TICKS(STARTUP_DELAY_MS)) {
                nextState = STATE_INITIALIZATION;
            }
            else 
            {
                // stay in STARTUP state
            }
            break;
            
        case STATE_INITIALIZATION:
            if (imu_state_machine_inputs.fault_present) {
                nextState = STATE_FAULTED;
            } 
            else 
            {
                nextState = STATE_ACTIVE;
            }
            break;
            
        case STATE_ACTIVE:
            if (imu_state_machine_inputs.fault_present) {
                nextState = STATE_FAULTED;
            } 
            else 
            {
                // stay in ACTIVE state
            }
            break;
            
        case STATE_FAULTED:
            // Stay in FAULTED state (would need external reset/recovery command to exit)
            nextState = STATE_FAULTED;
            break;
            
        default:
            nextState = STATE_FAULTED;
            break;
    }
    
    return nextState;
}

STATIC void imu_state_initialization_on_entry(uint16_t prevState)
{   
    (void)prevState;  // Unused
    
    // Clear any previous fault
    imu_fault_code = FAULT_NONE;
    
    // Get the port device structure
    imu_dev = imu_port_init();
    if (imu_dev == NULL) {
        imu_fault_code = FAULT_INIT_NULL_DEV;
        return;
    }
    
    // Probe and initialize the device (this handles the mode switch and SPI setup)
    int probe_result = imu_port_probe_and_init(imu_dev);
    if (probe_result != IMU_PORT_SUCCESS) {
        imu_fault_code = FAULT_PROBE_FAILED;
        return;
    }
    
    // Verify chip ID through port layer
    if (!verify_chip_id()) {
        imu_fault_code = FAULT_CHIP_ID_INVALID;
        return;
    }
    
    // Configure accelerometer: ±2g range, 100 Hz ODR
    int accel_result = imu_port_configure_accel(imu_dev, IMU_ACCEL_RANGE_2G, IMU_ODR_100HZ);
    if (accel_result != IMU_PORT_SUCCESS) {
        imu_fault_code = FAULT_ACCEL_CONFIG_FAILED;
        return;
    }
    
    // Configure gyroscope: ±2000 deg/s range, 100 Hz ODR
    int gyro_result = imu_port_configure_gyro(imu_dev, IMU_GYRO_RANGE_2000DPS, IMU_ODR_100HZ);
    if (gyro_result != IMU_PORT_SUCCESS) {
        imu_fault_code = FAULT_GYRO_CONFIG_FAILED;
        return;
    }
}

STATIC void imu_state_active_process(void)
{
    // Read sensors in active state
    read_sensors();
}

STATIC void imu_state_faulted_on_entry(uint16_t prevState)
{
    (void)prevState;  // Unused
    
    // Log fault information
    const char *fault_str;
    switch (imu_fault_code) {
        case FAULT_INIT_NULL_DEV:       fault_str = "Device init failed (NULL)"; break;
        case FAULT_PROBE_FAILED:        fault_str = "Probe/init failed"; break;
        case FAULT_CHIP_ID_INVALID:     fault_str = "Invalid chip ID"; break;
        case FAULT_ACCEL_CONFIG_FAILED: fault_str = "Accel config failed"; break;
        case FAULT_GYRO_CONFIG_FAILED:  fault_str = "Gyro config failed"; break;
        case FAULT_READ_FAILED:         fault_str = "Sensor read failed"; break;
        default:                        fault_str = "Unknown fault"; break;
    }
    
    uart_manager_print("IMU FAULT: %s (code=%u)\r\n", fault_str, imu_fault_code);
}

/*---------------------------------------------------------------------------
 * Command Handler
 *---------------------------------------------------------------------------*/
STATIC bool imu_cmd_handler(const cmd_parsed_t *parsed)
{
    switch (parsed->action) {
        case CMD_ACTION_GET:
            if (strcmp(parsed->target, "status") == 0) {
                const char *state_str;
                switch (imu_state_machine.curr_state) {
                    case STATE_ACTIVE:         state_str = "active"; break;
                    case STATE_INITIALIZATION: state_str = "init"; break;
                    case STATE_FAULTED:        state_str = "FAULTED"; break;
                    default:                   state_str = "startup"; break;
                }
                uart_manager_print("IMU status: %s, chip_id=0x%02X", state_str, measurements.chip_id);
                if (imu_fault_code != FAULT_NONE) {
                    uart_manager_print(", imu_fault_code=%u", imu_fault_code);
                }
                uart_manager_print("\r\n");
                return true;
            }
            else if (strcmp(parsed->target, "data") == 0) {
                if (imu_state_machine.curr_state != STATE_ACTIVE) {
                    uart_manager_print("IMU not active yet\r\n");
                    return true;
                }
                uart_manager_print("Accel: X=%.3f Y=%.3f Z=%.3f m/s^2\r\n",
                                 measurements.accel.x, measurements.accel.y, measurements.accel.z);
                uart_manager_print("Gyro:  X=%.3f Y=%.3f Z=%.3f deg/s\r\n",
                                 measurements.gyro.x, measurements.gyro.y, measurements.gyro.z);
                uart_manager_print("Temp:  %.2f C\r\n", measurements.temperature);
                return true;
            }
            else if (strcmp(parsed->target, "accel") == 0) {
                if (imu_state_machine.curr_state != STATE_ACTIVE) {
                    uart_manager_print("IMU not active yet\r\n");
                    return true;
                }
                uart_manager_print("Accel: X=%.3f Y=%.3f Z=%.3f m/s^2\r\n",
                                 measurements.accel.x, measurements.accel.y, measurements.accel.z);
                return true;
            }
            else if (strcmp(parsed->target, "gyro") == 0) {
                if (imu_state_machine.curr_state != STATE_ACTIVE) {
                    uart_manager_print("IMU not active yet\r\n");
                    return true;
                }
                uart_manager_print("Gyro: X=%.3f Y=%.3f Z=%.3f deg/s\r\n",
                                 measurements.gyro.x, measurements.gyro.y, measurements.gyro.z);
                return true;
            }
            else if (strcmp(parsed->target, "temp") == 0) {
                if (imu_state_machine.curr_state != STATE_ACTIVE) {
                    uart_manager_print("IMU not active yet\r\n");
                    return true;
                }
                uart_manager_print("Temp: %.2f C\r\n", measurements.temperature);
                return true;
            }
            break;
            
        case CMD_ACTION_SET:
        case CMD_ACTION_REQ:
        case CMD_ACTION_UNKNOWN:
        default:
            break;
    }
    
    return false;
}

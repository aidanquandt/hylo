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
#include "platform_os.h"
#include "imu_port.h"
#include "state_machine.h"
#include "uart_manager.h"

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define IMU_EXPECTED_CHIP_ID_1  (0x43U)  // BMI323 Chip ID
#define IMU_EXPECTED_CHIP_ID_2  (0x44U)  // BMI330 Chip ID
#define STARTUP_DELAY_MS        (2000U)  // Wait 2 seconds before probing
#define PRINT_INTERVAL_MS       (500U)   // Print IMU data every 500ms

// State machine states
typedef enum {
    STATE_STARTUP,              // Waiting for startup delay
    STATE_INITIALIZATION,       // Initializing hardware
    STATE_ACTIVE                // Normal operation - reading sensors
} imu_state_E;

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

// State machine transition logic
STATIC uint16_t imu_test_transition_logic(uint16_t currentState, uint32_t stateTimer);

// State handlers
STATIC void imu_test_state_initialization_on_entry(uint16_t prevState);
STATIC void imu_test_state_active_process(void);

/*---------------------------------------------------------------------------
 * Module Functions
 *---------------------------------------------------------------------------*/
STATIC void imu_test_init(void);
STATIC void imu_test_process_100Hz(void);

extern const module_S imu_test_module;
const module_S imu_test_module = {
    .module_init = imu_test_init,
    .module_process_100Hz = imu_test_process_100Hz,
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
STATIC imu_measurements_t measurements = {0};

// Print throttling
STATIC uint32_t print_counter = 0;

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
        return;
    }
    
    // Read sensors separately instead of combined to avoid SPI sync issues
    // with long jumper wires. Each transaction is independent with its own CS cycle.
    
    // Read accelerometer
    imu_port_read_accel(imu_dev, &measurements.accel);
    
    // Read gyroscope
    imu_port_read_gyro(imu_dev, &measurements.gyro);
    
    // Read temperature (polled less frequently would be better, but keeping for now)
    measurements.temperature = imu_port_read_temperature(imu_dev);
}

STATIC void imu_test_init(void)
{
    // State machine is already initialized with static values
}

STATIC void imu_test_process_100Hz(void)
{
    // Run state machine at 100Hz
    state_machine_periodic(&imu_state_machine);
}

STATIC uint16_t imu_test_transition_logic(uint16_t currentState, uint32_t stateTimer)
{
    uint16_t nextState = currentState;
    
    switch (currentState) {
        case STATE_STARTUP:
            // Transition to INITIALIZATION after startup delay (200 ticks at 100Hz = 2000ms)
            if (stateTimer >= MS_TO_100HZ_TICKS(STARTUP_DELAY_MS)) {
                nextState = STATE_INITIALIZATION;
            }
            break;
            
        case STATE_INITIALIZATION:
            // Transition to ACTIVE immediately (initialization happens in onEntry)
            nextState = STATE_ACTIVE;
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

STATIC void imu_test_state_initialization_on_entry(uint16_t prevState)
{   
    (void)prevState;  // Unused
    
    // Get the port device structure
    imu_dev = imu_port_init();
    if (imu_dev == NULL) {
        return;
    }
    
    // Probe and initialize the device (this handles the mode switch and SPI setup)
    int probe_result = imu_port_probe_and_init(imu_dev);
    if (probe_result != IMU_PORT_SUCCESS) {
        return;
    }
    
    // Verify chip ID through port layer
    if (!verify_chip_id()) {
        return;
    }
    
    // Configure accelerometer: ±2g range, 100 Hz ODR, 4-sample averaging
    imu_port_configure_accel(imu_dev, IMU_ACCEL_RANGE_2G, IMU_ODR_100HZ, IMU_AVG_4);
    
    // Configure gyroscope: ±2000 deg/s range (maximum), 100 Hz ODR, 4-sample averaging
    imu_port_configure_gyro(imu_dev, IMU_GYRO_RANGE_2000DPS, IMU_ODR_100HZ, IMU_AVG_4);
}

STATIC void imu_test_state_active_process(void)
{
    // Read sensors in active state
    read_sensors();
    
    // Print IMU data at reduced rate (every 500ms at 100Hz = 50 ticks)
    print_counter++;
    if (print_counter >= MS_TO_100HZ_TICKS(PRINT_INTERVAL_MS)) {
        print_counter = 0;
        
        // Convert floats to integers for printing (since %f may not be supported)
        // Accelerometer: multiply by 1000 to get millig (mg)
        int32_t acc_x = (int32_t)(measurements.accel.x * 1000.0f);
        int32_t acc_y = (int32_t)(measurements.accel.y * 1000.0f);
        int32_t acc_z = (int32_t)(measurements.accel.z * 1000.0f);
        
        // Gyroscope: multiply by 100 to get centidegrees/s (cdps)
        int32_t gyr_x = (int32_t)(measurements.gyro.x * 100.0f);
        int32_t gyr_y = (int32_t)(measurements.gyro.y * 100.0f);
        int32_t gyr_z = (int32_t)(measurements.gyro.z * 100.0f);
        
        // Temperature: multiply by 10 to get decidegrees
        int32_t temp = (int32_t)(measurements.temperature * 10.0f);
        
        uart_manager_print("IMU [ID:0x%02X] Accel: X=%6d Y=%6d Z=%6d mg | "
                          "Gyro: X=%7d Y=%7d Z=%7d cdps | Temp: %3d.%01d C\r\n",
                          measurements.chip_id,
                          (int)acc_x, (int)acc_y, (int)acc_z,
                          (int)gyr_x, (int)gyr_y, (int)gyr_z,
                          (int)(temp / 10), (int)(temp % 10));
    }
}

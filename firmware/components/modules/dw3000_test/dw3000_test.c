/*---------------------------------------------------------------------------
 * @file    dw3000_test.c
 * @brief   DW3000 hardware connection test module
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "dw3000_test.h"
#include "module.h"
#include "platform_gpio.h"
#include "dw3000_port.h"

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define DW3000_EXPECTED_DEV_ID  (0xDECA0302UL)  // DW3000 Device ID
#define STARTUP_DELAY_SECONDS   (2U)            // Wait 2 seconds before probing
#define TEST_INTERVAL_SECONDS   (5U)            // Test every 5 seconds

/*---------------------------------------------------------------------------
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
STATIC bool verify_device_id(void);
STATIC void read_measurements(void);

/*---------------------------------------------------------------------------
 * Module Functions
 *---------------------------------------------------------------------------*/
STATIC void dw3000_test_init(void);
STATIC void dw3000_test_process_1Hz(void);

extern const module_S dw3000_test_module;
const module_S dw3000_test_module = {
    .module_init = dw3000_test_init,
    .module_process_1Hz = dw3000_test_process_1Hz,
};

/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/
STATIC uint32_t device_id = 0;
STATIC float temperature = 0.0f;
STATIC float voltage = 0.0f;
STATIC uint32_t startup_timer = 0;
STATIC bool hardware_ready = false;

/*---------------------------------------------------------------------------
 * Private Function Implementations
 *---------------------------------------------------------------------------*/

STATIC bool verify_device_id(void)
{
    // Check the device ID through port layer
    int ret = dw3000_port_check_device_id();
    if (ret != DW3000_SUCCESS) {
        return false;
    }
    
    // Read and store device ID
    device_id = dw3000_port_read_device_id();
    
    // Verify it matches expected value
    return (device_id == DW3000_EXPECTED_DEV_ID);
}

STATIC void read_measurements(void)
{
    // Read and store IC temperature and voltage (single optimized call)
    dw3000_port_read_temp_and_voltage(&temperature, &voltage);
}

STATIC void dw3000_test_init(void)
{
    // Nothing to do - initialization happens in 1Hz task after delay
}

STATIC void dw3000_test_process_1Hz(void)
{
    // Toggle LED to show we're running
    platform_gpio_toggle_led_green();
    
    // Increment startup timer if hardware not yet ready
    if (!hardware_ready) {
        startup_timer++;
        
        // After startup delay, attempt to initialize hardware
        if (startup_timer >= STARTUP_DELAY_SECONDS) {
            // Get the port chip structure
            dwchip_t* dw_chip = dw3000_port_init();
            if (dw_chip == NULL) {
                return;
            }
            
            // Probe and initialize the device (single clean call)
            int ret = dw3000_port_probe_and_init(dw_chip);
            if (ret != DW3000_SUCCESS) {
                return;
            }
            
            // Verify device ID (once only)
            if (!verify_device_id()) {
                return;
            }
            
            hardware_ready = true;
            
            // Read initial measurements
            read_measurements();
        }
    } else {
        // Hardware is ready - read measurements every second
        read_measurements();
    }
}

/*---------------------------------------------------------------------------
 * Public Function Implementations
 *---------------------------------------------------------------------------*/

bool dw3000_test_device_id(void)
{
    // For backward compatibility - verify device ID is correct
    return (device_id == DW3000_EXPECTED_DEV_ID);
}

uint32_t dw3000_test_get_device_id(void)
{
    return device_id;
}

float dw3000_test_get_temperature(void)
{
    return temperature;
}

float dw3000_test_get_voltage(void)
{
    return voltage;
}

bool dw3000_test_is_ready(void)
{
    return hardware_ready;
}
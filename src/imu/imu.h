#pragma once

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "common.h"
#include "imu_driver.h"

/*---------------------------------------------------------------------------
 * Typedefs
 *---------------------------------------------------------------------------*/

typedef enum
{
    IMU_STATE_STARTUP,
    IMU_STATE_INITIALIZATION,
    IMU_STATE_ACTIVE,
    IMU_STATE_FAULTED
} imu_state_e;

typedef struct
{
    imu_state_e state;
    uint8_t chip_id;
    uint32_t fault_code;
} imu_status_t;

typedef struct
{
    vec3_t accel;
    vec3_t gyro;
    float temperature;
} imu_data_t;

/*---------------------------------------------------------------------------
 * Public Function Prototypes
 *---------------------------------------------------------------------------*/
void imu_get_status(imu_status_t* status);
imu_state_e imu_get_state(imu_device_e device);
bool imu_get_data(imu_data_t* data);
bool imu_get_accel(vec3_t* accel);
bool imu_get_gyro(vec3_t* gyro);
bool imu_get_temp(imu_device_e device, float* temp);
bool imu_get_individual_data(imu_device_e device, imu_data_t* data);
uint8_t imu_get_device_count(void);
uint8_t imu_get_active_count(void);
bool imu_soft_reset(void);
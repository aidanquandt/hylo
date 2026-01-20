#pragma once

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "common.h"

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
    float x;
    float y;
    float z;
} imu_vector3_t;

typedef struct
{
    imu_state_e state;
    uint8_t chip_id;
    uint32_t fault_code;
} imu_status_t;

typedef struct
{
    imu_vector3_t accel;
    imu_vector3_t gyro;
    float temperature;
} imu_data_t;

/*---------------------------------------------------------------------------
 * Public Function Prototypes
 *---------------------------------------------------------------------------*/
void imu_get_status(imu_status_t* status);
bool imu_get_data(imu_data_t* data);
bool imu_get_accel(imu_vector3_t* accel);
bool imu_get_gyro(imu_vector3_t* gyro);
bool imu_get_temp(float* temp);
bool imu_soft_reset(void);
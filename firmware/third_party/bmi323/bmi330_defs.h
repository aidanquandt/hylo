/**
 * Copyright (c) 2024 Bosch Sensortec GmbH. All rights reserved.
*
* BSD-3-Clause
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* @file       bmi330_defs.h
* @date       2024-10-17
* @version    v2.4.0
*
*/
 #ifndef _BMI330_DEFS_H
 #define _BMI330_DEFS_H

/********************************************************* */
/*!             Header includes                           */
/********************************************************* */
#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/kernel.h>
#else
#include <stdint.h>
#include <stddef.h>
#endif

/********************************************************* */
/*!               Common Macros                           */
/********************************************************* */
#ifdef __KERNEL__
#if !defined(UINT8_C) && !defined(INT8_C)
#define INT8_C(x)    S8_C(x)
#define UINT8_C(x)   U8_C(x)
#endif

#if !defined(UINT16_C) && !defined(INT16_C)
#define INT16_C(x)   S16_C(x)
#define UINT16_C(x)  U16_C(x)
#endif

#if !defined(INT32_C) && !defined(UINT32_C)
#define INT32_C(x)   S32_C(x)
#define UINT32_C(x)  U32_C(x)
#endif

#if !defined(INT64_C) && !defined(UINT64_C)
#define INT64_C(x)   S64_C(x)
#define UINT64_C(x)  U64_C(x)
#endif
#endif

/*! C standard macros */
#ifndef NULL
#ifdef __cplusplus
#define NULL         0
#else
#define NULL         ((void *) 0)
#endif
#endif

/********************************************************* */
/*!             General Macro Definitions                 */
/********************************************************* */
/*! Utility macros */
#define BMI330_SET_BITS(reg_data, bitname, data) \
    ((reg_data & ~(bitname##_MASK)) | \
     ((data << bitname##_POS) & bitname##_MASK))

#define BMI330_GET_BITS(reg_data, bitname) \
    ((reg_data & (bitname##_MASK)) >> \
     (bitname##_POS))

#define BMI330_SET_BIT_POS0(reg_data, bitname, data) \
    ((reg_data & ~(bitname##_MASK)) | \
     (data & bitname##_MASK))

#define BMI330_GET_BIT_POS0(reg_data, bitname)  (reg_data & (bitname##_MASK))
#define BMI330_SET_BIT_VAL0(reg_data, bitname)  (reg_data & ~(bitname##_MASK))

/*! LSB and MSB mask definitions */
#define BMI330_SET_LOW_BYTE                     UINT16_C(0x00FF)
#define BMI330_SET_HIGH_BYTE                    UINT16_C(0xFF00)
#define BMI330_SET_LOW_NIBBLE                   UINT8_C(0x0F)

/*! For getting LSB and MSB */
#define BMI330_GET_LSB(var)                     (uint8_t)(var & BMI330_SET_LOW_BYTE)
#define BMI330_GET_MSB(var)                     (uint8_t)((var & BMI330_SET_HIGH_BYTE) >> 8)

/*! For enable and disable */
#define BMI330_ENABLE                           UINT8_C(1)
#define BMI330_DISABLE                          UINT8_C(0)

/*! To define TRUE or FALSE */
#define BMI330_TRUE                             UINT8_C(1)
#define BMI330_FALSE                            UINT8_C(0)

/*!
 * BMI330_INTF_RET_TYPE is the read/write interface return type which can be overwritten by the build system.
 * The default is set to int8_t.
 */
#ifndef BMI330_INTF_RET_TYPE
#define BMI330_INTF_RET_TYPE                    int8_t
#endif

/*!
 * BMI330_INTF_RET_SUCCESS is the success return value read/write interface return type which can be
 * overwritten by the build system. The default is set to 0.
 */
#ifndef BMI330_INTF_RET_SUCCESS
#define BMI330_INTF_RET_SUCCESS                 INT8_C(0)
#endif

/*! To define the chip id of bmi330 */
#define BMI330_CHIP_ID                          UINT16_C(0x0048)

/*! To define success code */
#define BMI330_OK                               INT8_C(0)

/*! To define error codes */
#define BMI330_E_NULL_PTR                       BMI3_E_NULL_PTR
#define BMI330_E_COM_FAIL                       BMI3_E_COM_FAIL
#define BMI330_E_DEV_NOT_FOUND                  BMI3_E_DEV_NOT_FOUND
#define BMI330_E_INVALID_CONTEXT_SEL            INT8_C(-13)

/***************************************************************************** */
/*!         Sensor Macro Definitions                 */
/***************************************************************************** */
/*! Macros to define BMI330 sensor/feature types */
#define BMI330_ACCEL                            BMI3_ACCEL
#define BMI330_GYRO                             BMI3_GYRO
#define BMI330_SIG_MOTION                       BMI3_SIG_MOTION
#define BMI330_ANY_MOTION                       BMI3_ANY_MOTION
#define BMI330_NO_MOTION                        BMI3_NO_MOTION
#define BMI330_STEP_COUNTER                     BMI3_STEP_COUNTER
#define BMI330_TILT                             BMI3_TILT
#define BMI330_ORIENTATION                      BMI3_ORIENTATION
#define BMI330_FLAT                             BMI3_FLAT
#define BMI330_TAP                              BMI3_TAP
#define BMI330_ALT_ACCEL                        BMI3_ALT_ACCEL
#define BMI330_ALT_GYRO                         BMI3_ALT_GYRO
#define BMI330_ALT_AUTO_CONFIG                  BMI3_ALT_AUTO_CONFIG

/*! Non virtual sensor features */
#define BMI330_TEMP                             BMI3_TEMP
#define BMI330_I3C_SYNC_ACCEL                   BMI3_I3C_SYNC_ACCEL
#define BMI330_I3C_SYNC_GYRO                    BMI3_I3C_SYNC_GYRO
#define BMI330_I3C_SYNC_TEMP                    BMI3_I3C_SYNC_TEMP

/*! Maximum number of features in bmi330 */
#define BMI330_MAX_FEATURE                      UINT8_C(6)

/*! Parameter set limit for bmi330 features */
#define BMI330_PARAM_LIMIT_TILT                 UINT8_C(3)
#define BMI330_PARAM_LIMIT_ANY_MOT              UINT8_C(5)
#define BMI330_PARAM_LIMIT_NO_MOT               UINT8_C(5)
#define BMI330_PARAM_LIMIT_SIG_MOT              UINT8_C(5)
#define BMI330_PARAM_LIMIT_FLAT                 UINT8_C(5)
#define BMI330_PARAM_LIMIT_ORIENT               UINT8_C(7)
#define BMI330_PARAM_LIMIT_WAKEUP               UINT8_C(10)
#define BMI330_PARAM_LIMIT_STEP_COUNT           UINT8_C(22)

#define BMI330_16_BIT_RESOLUTION                BMI3_16_BIT_RESOLUTION

/*! Maximum number of interrupt pins */
#define BMI330_INT_PIN_MAX_NUM                  UINT8_C(2)

/********************************************************* */
/*!               Function Pointers                       */
/********************************************************* */

/*!
 * @brief Bus communication function pointer which should be mapped to
 * the platform specific read functions of the user
 *
 * @param[in]     reg_addr : 8bit register address of the sensor
 * @param[out]    reg_data : Data from the specified address
 * @param[in]     length   : Length of the reg_data array
 * @param[in,out] intf_ptr : Void pointer that can enable the linking of descriptors
 *                           for interface related callbacks
 * @retval 0 for Success
 * @retval Non-zero for Failure
 */
typedef BMI330_INTF_RET_TYPE (*bmi330_read_fptr_t)(uint8_t reg_addr, uint8_t *reg_data, uint32_t length,
                                                   void *intf_ptr);

/*!
 * @brief Bus communication function pointer which should be mapped to
 * the platform specific write functions of the user
 *
 * @param[in]     reg_addr : 8bit register address of the sensor
 * @param[in]     reg_data : Data to the specified address
 * @param[in]     length   : Length of the reg_data array
 * @param[in,out] intf_ptr : Void pointer that can enable the linking of descriptors
 *                           for interface related callbacks
 * @retval 0 for Success
 * @retval Non-zero for Failure
 *
 */
typedef BMI330_INTF_RET_TYPE (*bmi330_write_fptr_t)(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length,
                                                    void *intf_ptr);

/*!
 * @brief Delay function pointer which should be mapped to
 * delay function of the user
 *
 * @param period - The time period in microseconds
 * @param[in,out] intf_ptr : Void pointer that can enable the linking of descriptors
 *                           for interface related callbacks
 */
typedef void (*bmi330_delay_us_fptr_t)(uint32_t period, void *intf_ptr);

#endif /* _BMI330_DEFS_H */

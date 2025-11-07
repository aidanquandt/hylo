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
* @file       bmi330.c
* @date       2024-10-17
* @version    v2.4.0
*
*/

/******************************************************************************/

/*!  @name          Header Files                                  */
/******************************************************************************/

#include "bmi330.h"
#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/kernel.h>
#else
#include "stdio.h"
#endif

/*!              Static Variable
 ****************************************************************************/

/*! Any-motion context parameter set for smart phone */
static uint16_t any_motion_param_set[BMI330_PARAM_LIMIT_ANY_MOT] = { 8, 1, 5, 250, 5 };

/*! No-motion context parameter set for smart phone */
static uint16_t no_motion_param_set[BMI330_PARAM_LIMIT_NO_MOT] = { 30, 1, 5, 250, 5 };

/*! Tap context parameter set for smart phone */
static uint16_t tap_param_set[BMI330_PARAM_LIMIT_WAKEUP] = { 2, 1, 6, 1, 143, 25, 4, 6, 8, 6 };

/*! Step counter context parameter set for smart phone */
static uint16_t step_counter_param_set[BMI330_PARAM_LIMIT_STEP_COUNT] =
{ 0, 0, 306, 61900, 132, 55608, 60104, 64852, 7, 1, 256, 12, 12, 3, 3900, 74, 160, 0, 0, 0, 0, 0 };

/*! Sig-motion context parameter set for smart phone */
static uint16_t sig_motion_param_set[BMI330_PARAM_LIMIT_SIG_MOT] = { 250, 60, 11, 595, 17 };

/*! Orientation context parameter set for smart phone */
static uint16_t orientation_param_set[BMI330_PARAM_LIMIT_ORIENT] = { 0, 0, 3, 38, 10, 50, 32 };

/*! Array to store RAM patch extended code */
static uint8_t bmi330_ram_patch_extn_code[] = {
    0x48, 0x02, 0xca, 0x00, 0x01, 0x2e, 0xa0, 0xf2, 0x09, 0xbc, 0x20, 0x50, 0x0d, 0xb8, 0x03, 0x2e, 0x0a, 0x03, 0x01,
    0x1a, 0xf0, 0x7f, 0xeb, 0x7f, 0x17, 0x2f, 0x20, 0x30, 0x21, 0x2e, 0xd2, 0x01, 0x98, 0x2e, 0x1d, 0xc7, 0x98, 0x2e,
    0x2f, 0xc7, 0x98, 0x2e, 0x8e, 0xc0, 0x03, 0x2e, 0xf6, 0x01, 0x12, 0x24, 0xff, 0xe3, 0x4a, 0x08, 0x00, 0x30, 0x21,
    0x2e, 0xcd, 0x01, 0xf2, 0x6f, 0x21, 0x2e, 0xd1, 0x01, 0x25, 0x2e, 0x0a, 0x03, 0x23, 0x2e, 0xf6, 0x01, 0xeb, 0x6f,
    0xe0, 0x5f, 0xb8, 0x2e, 0x03, 0x2e, 0xac, 0x01, 0x01, 0x2e, 0x21, 0xf1, 0x92, 0xbc, 0x01, 0x0a, 0xc0, 0x2e, 0x21,
    0x2e, 0x21, 0xf1, 0xb8, 0x2e, 0x01, 0x2e, 0xf6, 0x01, 0x86, 0xbc, 0x9f, 0xb8, 0x41, 0x90, 0x06, 0x2f, 0x11, 0x24,
    0xff, 0xfd, 0x01, 0x08, 0x21, 0x2e, 0xf6, 0x01, 0x80, 0x2e, 0x43, 0xeb, 0xb8, 0x2e, 0x10, 0x50, 0xfb, 0x7f, 0x98,
    0x2e, 0x48, 0x02, 0xfb, 0x6f, 0xf0, 0x5f, 0x80, 0x2e, 0xa8, 0x02, 0x80, 0x2e, 0x48, 0x02, 0x01, 0x2e, 0x0b, 0x03,
    0x10, 0x50, 0x00, 0x90, 0x0e, 0x2f, 0x01, 0x2e, 0xa0, 0xf2, 0x09, 0xbc, 0x0d, 0xb8, 0x31, 0x30, 0x08, 0x04, 0xfb,
    0x7f, 0x21, 0x2e, 0xd5, 0x01, 0x98, 0x2e, 0x18, 0xe5, 0x10, 0x30, 0x21, 0x2e, 0x0b, 0x03, 0xfb, 0x6f, 0xf0, 0x5f,
    0xb8, 0x2e, 0xb8, 0x2e, 0xb8, 0x2e, 0x01, 0x2e, 0x12, 0xf1, 0x03, 0x2e, 0xbc, 0x00, 0x96, 0xbc, 0x20, 0x50, 0x96,
    0xb8, 0x00, 0xa6, 0x54, 0xb0, 0xfb, 0x7f, 0x53, 0x2f, 0x03, 0x2e, 0x0d, 0x03, 0x40, 0x90, 0x26, 0x25, 0x01, 0x30,
    0x07, 0x30, 0x13, 0x30, 0x19, 0x2f, 0x09, 0x2e, 0x0f, 0x03, 0x44, 0x0e, 0x02, 0x2f, 0x02, 0x01, 0x29, 0x2e, 0x0f,
    0x03, 0x09, 0x2e, 0x0f, 0x03, 0xc4, 0x0e, 0x0e, 0x2f, 0x27, 0x2e, 0x0d, 0x03, 0x44, 0x31, 0x05, 0x30, 0x41, 0x8b,
    0x20, 0x19, 0x50, 0xa1, 0xfb, 0x2f, 0xf4, 0x3f, 0x20, 0x05, 0x94, 0x04, 0x25, 0x2e, 0x10, 0x03, 0x2d, 0x2e, 0x0e,
    0x03, 0x05, 0x2e, 0x0d, 0x03, 0x81, 0x90, 0xe0, 0x7f, 0x0f, 0x2f, 0x05, 0x2e, 0x0c, 0x03, 0x50, 0x0f, 0x07, 0x2f,
    0x05, 0x2e, 0xbc, 0x00, 0x13, 0x24, 0x00, 0xfc, 0x93, 0x08, 0x25, 0x2e, 0xbc, 0x00, 0x04, 0x2d, 0x27, 0x2e, 0x11,
    0x03, 0x23, 0x2e, 0x0d, 0x03, 0x05, 0x2e, 0x11, 0x03, 0x81, 0x90, 0x15, 0x2f, 0x05, 0x2e, 0x10, 0x03, 0x42, 0x0e,
    0x11, 0x2f, 0x23, 0x2e, 0x11, 0x03, 0x50, 0x30, 0x98, 0x2e, 0x04, 0xeb, 0x01, 0x2e, 0x0e, 0x03, 0x05, 0x2e, 0xbc,
    0x00, 0x11, 0x24, 0xff, 0x03, 0x13, 0x24, 0x00, 0xfc, 0x93, 0x08, 0x01, 0x08, 0x02, 0x0a, 0x21, 0x2e, 0xbc, 0x00,
    0xe0, 0x6f, 0x21, 0x2e, 0x0c, 0x03, 0xfb, 0x6f, 0xe0, 0x5f, 0xb8, 0x2e, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

/*! Array to store RAM patch extended table */
static uint8_t bmi330_ram_patch_extn_table[] = {
    0x05, 0x02, 0x0c, 0x00, 0x90, 0x02, 0x8e, 0x02, 0x86, 0x02, 0x6e, 0x02, 0x00, 0x00, 0x77, 0x02, 0xa6, 0x02, 0xa7,
    0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78, 0x02
};

/*! Array to store RAM version */
static uint8_t bmi330_ram_version[] = {
    0xad, 0x00, 0x01, 0x00, 0x02, 0x0c
};

/******************************************************************************/

/*!         Local Function Prototypes
 ******************************************************************************/

/*!
 * @brief This internal API is used to validate the device pointer for
 * null conditions.
 *
 * @param[in] dev : Structure instance of bmi3_dev.
 *
 * @return Result of API execution status
 *
 * @return 0 -> Success
 * @return < 0 -> Fail
 *
 */
static int8_t null_ptr_check(const struct bmi3_dev *dev);

/*!
 * @brief This internal API writes the command register value to enable cfg res.
 *
 * @param[in,out] dev       : Structure instance of bmi3_dev.
 *
 * @return Result of API execution status
 *
 * @return 0 -> Success
 * @return < 0 -> Fail
 *
 */
static int8_t config_array_set_command(struct bmi3_dev *dev);

/*!
 * @brief This internal API sets the value_one to feature engine in cfg res.
 *
 * @param[in,out] dev       : Structure instance of bmi3_dev.
 *
 * @return Result of API execution status
 *
 * @return 0 -> Success
 * @return < 0 -> Fail
 *
 */
static int8_t config_array_set_value_one_page(struct bmi3_dev *dev);

/*!
 * @brief This internal API writes the config array in cfg res.
 *
 * @param[in,out] dev       : Structure instance of bmi3_dev.
 *
 * @return Result of API execution status
 *
 * @return 0 -> Success
 * @return < 0 -> Fail
 *
 */
static int8_t write_config_array(struct bmi3_dev *dev);

/*!
 * @brief This internal API writes config array.
 *
 * @param[in] config_array  : Pointer variable to store config array.
 * @param[in] config_size   : Variable to store size of config array.
 * @param[in,out] dev       : Structure instance of bmi3_dev.
 *
 * @return Result of API execution status
 *
 * @return 0 -> Success
 * @return < 0 -> Fail
 *
 */
static int8_t load_config_array(const uint8_t *config_array, uint16_t config_size, struct bmi3_dev *dev);

/*!
 * @brief This internal API writes config array.
 *
 * @param[in] config_array  : Pointer variable to store config array.
 * @param[in] config_size   : Variable to store size of config array.
 * @param[in,out] dev       : Structure instance of bmi3_dev.
 *
 * @return Result of API execution status
 *
 * @return 0 -> Success
 * @return < 0 -> Fail
 *
 */
static int8_t upload_file(uint16_t config_base_addr, const uint8_t *config_array, uint16_t indx, struct bmi3_dev *dev);

/*!
 * @brief This internal API writes config version array to feature engine register.
 *
 * @param[in,out] dev       : Structure instance of bmi3_dev.
 *
 * @return Result of API execution status
 *
 * @return 0 -> Success
 * @return < 0 -> Fail
 *
 */
static int8_t write_config_version(struct bmi3_dev *dev);

/******************************************************************************/
/*!  @name      User Interface Definitions                            */
/******************************************************************************/

/*!
 * @brief This API is the entry point for bmi330 sensor. It reads and validates the
 * chip-id of the sensor.
 */
int8_t bmi330_init(struct bmi3_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Null-pointer check */
    rslt = null_ptr_check(dev);

    if (rslt == BMI330_OK)
    {
        rslt = bmi3_init(dev);
    }

    if (rslt == BMI330_OK)
    {
        /* Validate chip-id */
        if (dev->chip_id == BMI330_CHIP_ID)
        {
            /* Assign resolution to the structure */
            dev->resolution = BMI330_16_BIT_RESOLUTION;
        }
        else
        {
            rslt = BMI330_E_DEV_NOT_FOUND;
        }
    }

    if (rslt == BMI330_OK)
    {
        rslt = bmi330_context_switch_selection(dev);
    }

    return rslt;
}

/*!
 * @brief This API reads the data from the given register address of bmi330
 * sensor.
 *
 * @note For most of the registers auto address increment applies, with the
 * exception of a few special registers, which trap the address. For e.g.,
 * Register address - 0x03.
 */
int8_t bmi330_get_regs(uint8_t reg_addr, uint8_t *data, uint16_t len, struct bmi3_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    rslt = bmi3_get_regs(reg_addr, data, len, dev);

    return rslt;
}

/*!
 * @brief This API writes data to the given register address of bmi330 sensor.
 */
int8_t bmi330_set_regs(uint8_t reg_addr, const uint8_t *data, uint16_t len, struct bmi3_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    rslt = bmi3_set_regs(reg_addr, data, len, dev);

    return rslt;
}

/*!
 * @brief This API resets bmi330 sensor. All registers are overwritten with
 * their default values.
 */
int8_t bmi330_soft_reset(struct bmi3_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    rslt = bmi3_soft_reset(dev);

    return rslt;
}

/*!
 * @brief This API writes the available sensor specific commands to the sensor.
 */
int8_t bmi330_set_command_register(uint16_t command, struct bmi3_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Set the command in the command register */
    rslt = bmi3_set_command_register(command, dev);

    return rslt;
}

/*!
 * @brief This API gets the interrupt 1 status of both feature and data
 * interrupts
 */
int8_t bmi330_get_int1_status(uint16_t *int_status, struct bmi3_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Get the interrupt status */
    rslt = bmi3_get_int1_status(int_status, dev);

    return rslt;
}

/*!
 * @brief This API gets the interrupt 2 status of both feature and data
 * interrupts
 */
int8_t bmi330_get_int2_status(uint16_t *int_status, struct bmi3_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Get the interrupt status */
    rslt = bmi3_get_int2_status(int_status, dev);

    return rslt;
}

/*!
 * @brief This API gets the re-mapped x, y and z axes from the sensor and
 * updates the values in the device structure.
 *
 * @note: XYZ axis denotes x = x, y = y, z = z
 * Similarly,
 *    YXZ, x = y, y = x, z = z
 *
 */
int8_t bmi330_get_remap_axes(struct bmi3_axes_remap *remapped_axis, struct bmi3_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Get remapped axes */
    rslt = bmi3_get_remap_axes(remapped_axis, dev);

    return rslt;
}

/*!
 * @brief This API sets the re-mapped x, y and z axes to the sensor and
 * updates them in the device structure.
 */
int8_t bmi330_set_remap_axes(const struct bmi3_axes_remap remapped_axis, struct bmi3_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Set remapped axes */
    rslt = bmi3_set_remap_axes(remapped_axis, dev);

    return rslt;
}

/*!
 * @brief This API sets the sensor/feature configuration.
 */
int8_t bmi330_set_sensor_config(struct bmi3_sens_config *sens_cfg, uint8_t n_sens, struct bmi3_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    rslt = bmi3_set_sensor_config(sens_cfg, n_sens, dev);

    return rslt;
}

/*!
 * @brief This API gets the sensor/feature configuration.
 */
int8_t bmi330_get_sensor_config(struct bmi3_sens_config *sens_cfg, uint8_t n_sens, struct bmi3_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    rslt = bmi3_get_sensor_config(sens_cfg, n_sens, dev);

    return rslt;
}

/*!
 * @brief This API maps/un-maps data interrupts to that of interrupt pins.
 */
int8_t bmi330_map_interrupt(struct bmi3_map_int map_int, struct bmi3_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Read map interrupt data */
    rslt = bmi3_map_interrupt(map_int, dev);

    return rslt;
}

/*!
 * @brief This API selects the sensors/features to be enabled or disabled.
 */
int8_t bmi330_select_sensor(struct bmi3_feature_enable *enable, struct bmi3_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    rslt = bmi3_select_sensor(enable, dev);

    return rslt;
}

/*!
 * @brief This API gets the sensor/feature data for accelerometer, gyroscope,
 * step counter, high-g, gyroscope user-gain update,
 * orientation, gyroscope cross sensitivity and error status for NVM and VFRM.
 */
int8_t bmi330_get_sensor_data(struct bmi3_sensor_data *sensor_data, uint8_t n_sens, struct bmi3_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    rslt = bmi3_get_sensor_data(sensor_data, n_sens, dev);

    return rslt;
}

/*!
 *  @brief This API reads the error status from the sensor.
 */
int8_t bmi330_get_error_status(struct bmi3_err_reg *err_reg, struct bmi3_dev *dev)
{
    /* Variable to hold execution status */
    int8_t rslt;

    rslt = bmi3_get_error_status(err_reg, dev);

    return rslt;
}

/*!
 *  @brief This API reads the feature engine error status from the sensor.
 */
int8_t bmi330_get_feature_engine_error_status(uint8_t *feature_engine_err_reg_lsb,
                                              uint8_t *feature_engine_err_reg_msb,
                                              struct bmi3_dev *dev)
{
    /* Variable to hold execution status */
    int8_t rslt;

    /* Read the feature engine error codes */
    rslt = bmi3_get_feature_engine_error_status(feature_engine_err_reg_lsb, feature_engine_err_reg_msb, dev);

    return rslt;
}

/*!
 * @brief This API sets:
 *        1) The input output configuration of the selected interrupt pin:
 *           INT1 or INT2.
 *        2) The interrupt mode: permanently latched or non-latched.
 */
int8_t bmi330_set_int_pin_config(const struct bmi3_int_pin_config *int_cfg, struct bmi3_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    rslt = bmi3_set_int_pin_config(int_cfg, dev);

    return rslt;
}

/*!
 * @brief This API gets:
 *        1) The input output configuration of the selected interrupt pin:
 *           INT1 or INT2.
 *        2) The interrupt mode: permanently latched or non-latched.
 */
int8_t bmi330_get_int_pin_config(struct bmi3_int_pin_config *int_cfg, struct bmi3_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    rslt = bmi3_get_int_pin_config(int_cfg, dev);

    return rslt;
}

/*!
 * @brief This API is used to get the sensor time.
 */
int8_t bmi330_get_sensor_time(uint32_t *sensor_time, struct bmi3_dev *dev)
{
    int8_t rslt;

    rslt = bmi3_get_sensor_time(sensor_time, dev);

    return rslt;
}

/*!
 * @brief This API reads the FIFO data.
 */
int8_t bmi330_read_fifo_data(struct bmi3_fifo_frame *fifo, struct bmi3_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    rslt = bmi3_read_fifo_data(fifo, dev);

    return rslt;
}

/*!
 * @brief This API parses and extracts the accelerometer frames from FIFO data
 * read by the "bmi330_read_fifo_data" API and stores it in the "accel_data"
 * structure instance.
 */
int8_t bmi330_extract_accel(struct bmi3_fifo_sens_axes_data *accel_data,
                            struct bmi3_fifo_frame *fifo,
                            const struct bmi3_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    rslt = bmi3_extract_accel(accel_data, fifo, dev);

    return rslt;
}

/*!
 * @brief This API parses and extracts the temperature frames from FIFO data
 * read by the "bmi330_read_fifo_data" API and stores it in the "temp_data"
 * structure instance.
 */
int8_t bmi330_extract_temperature(struct bmi3_fifo_temperature_data *temp_data,
                                  struct bmi3_fifo_frame *fifo,
                                  const struct bmi3_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    rslt = bmi3_extract_temperature(temp_data, fifo, dev);

    return rslt;
}

/*!
 * @brief This API parses and extracts the gyro frames from FIFO data
 * read by the "bmi330_read_fifo_data" API and stores it in the "gyro_data"
 * structure instance.
 */
int8_t bmi330_extract_gyro(struct bmi3_fifo_sens_axes_data *gyro_data,
                           struct bmi3_fifo_frame *fifo,
                           const struct bmi3_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    rslt = bmi3_extract_gyro(gyro_data, fifo, dev);

    return rslt;
}

/*!
 * @brief This API sets the FIFO water-mark level in words.
 */
int8_t bmi330_set_fifo_wm(uint16_t fifo_wm, struct bmi3_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    rslt = bmi3_set_fifo_wm(fifo_wm, dev);

    return rslt;
}

/*!
 * @brief This API reads the FIFO water-mark level set in words.
 */
int8_t bmi330_get_fifo_wm(uint16_t *fifo_wm, struct bmi3_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    rslt = bmi3_get_fifo_wm(fifo_wm, dev);

    return rslt;
}

/*!
 * @brief This API sets the FIFO configuration in the sensor.
 */
int8_t bmi330_set_fifo_config(uint16_t config, uint8_t enable, struct bmi3_dev *dev)
{
    int8_t rslt;

    rslt = bmi3_set_fifo_config(config, enable, dev);

    return rslt;
}

/*!
 * @brief This API reads the FIFO configuration from the sensor.
 */
int8_t bmi330_get_fifo_config(uint16_t *fifo_config, struct bmi3_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    rslt = bmi3_get_fifo_config(fifo_config, dev);

    return rslt;
}

/*!
 * @brief This API gets the length of FIFO data available in the sensor in
 * words.
 */
int8_t bmi330_get_fifo_length(uint16_t *fifo_avail_len, struct bmi3_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    rslt = bmi3_get_fifo_length(fifo_avail_len, dev);

    return rslt;
}

/*!
 * @brief This API writes the configurations of context feature for smart phone.
 */
int8_t bmi330_context_switch_selection(struct bmi3_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    struct bmi3_sens_config sens_cfg[BMI330_MAX_FEATURE];

    uint8_t index = 0;

    /* Set any-motion configuration */
    sens_cfg[0].type = BMI330_ANY_MOTION;
    sens_cfg[0].cfg.any_motion.slope_thres = any_motion_param_set[index++];
    sens_cfg[0].cfg.any_motion.acc_ref_up = (uint8_t)(any_motion_param_set[index++]);
    sens_cfg[0].cfg.any_motion.hysteresis = any_motion_param_set[index++];
    sens_cfg[0].cfg.any_motion.duration = any_motion_param_set[index++];
    sens_cfg[0].cfg.any_motion.wait_time = any_motion_param_set[index++];

    /* Set no-motion configuration */
    index = 0;
    sens_cfg[1].type = BMI330_NO_MOTION;
    sens_cfg[1].cfg.no_motion.slope_thres = no_motion_param_set[index++];
    sens_cfg[1].cfg.no_motion.acc_ref_up = (uint8_t)(no_motion_param_set[index++]);
    sens_cfg[1].cfg.no_motion.hysteresis = no_motion_param_set[index++];
    sens_cfg[1].cfg.no_motion.duration = no_motion_param_set[index++];
    sens_cfg[1].cfg.no_motion.wait_time = no_motion_param_set[index++];

    /* Set tap configuration */
    index = 0;
    sens_cfg[2].type = BMI330_TAP;
    sens_cfg[2].cfg.tap.axis_sel = (uint8_t)tap_param_set[index++];
    sens_cfg[2].cfg.tap.wait_for_timeout = (uint8_t)tap_param_set[index++];
    sens_cfg[2].cfg.tap.max_peaks_for_tap = (uint8_t)tap_param_set[index++];
    sens_cfg[2].cfg.tap.mode = (uint8_t)tap_param_set[index++];
    sens_cfg[2].cfg.tap.tap_peak_thres = tap_param_set[index++];
    sens_cfg[2].cfg.tap.max_gest_dur = (uint8_t)tap_param_set[index++];
    sens_cfg[2].cfg.tap.max_dur_between_peaks = (uint8_t)tap_param_set[index++];
    sens_cfg[2].cfg.tap.tap_shock_settling_dur = (uint8_t)tap_param_set[index++];
    sens_cfg[2].cfg.tap.min_quite_dur_between_taps = (uint8_t)tap_param_set[index++];
    sens_cfg[2].cfg.tap.quite_time_after_gest = (uint8_t)tap_param_set[index++];

    /* Set step counter configuration */
    index = 0;
    sens_cfg[3].type = BMI330_STEP_COUNTER;
    sens_cfg[3].cfg.step_counter.watermark_level = step_counter_param_set[index++];
    sens_cfg[3].cfg.step_counter.reset_counter = step_counter_param_set[index++];
    sens_cfg[3].cfg.step_counter.env_min_dist_up = step_counter_param_set[index++];
    sens_cfg[3].cfg.step_counter.env_coef_up = step_counter_param_set[index++];
    sens_cfg[3].cfg.step_counter.env_min_dist_down = step_counter_param_set[index++];
    sens_cfg[3].cfg.step_counter.env_coef_down = step_counter_param_set[index++];
    sens_cfg[3].cfg.step_counter.mean_val_decay = step_counter_param_set[index++];
    sens_cfg[3].cfg.step_counter.mean_step_dur = step_counter_param_set[index++];
    sens_cfg[3].cfg.step_counter.step_buffer_size = step_counter_param_set[index++];
    sens_cfg[3].cfg.step_counter.filter_cascade_enabled = step_counter_param_set[index++];
    sens_cfg[3].cfg.step_counter.step_counter_increment = step_counter_param_set[index++];
    sens_cfg[3].cfg.step_counter.peak_duration_min_walking = step_counter_param_set[index++];
    sens_cfg[3].cfg.step_counter.peak_duration_min_running = step_counter_param_set[index++];
    sens_cfg[3].cfg.step_counter.activity_detection_factor = step_counter_param_set[index++];
    sens_cfg[3].cfg.step_counter.activity_detection_thres = step_counter_param_set[index++];
    sens_cfg[3].cfg.step_counter.step_duration_max = step_counter_param_set[index++];
    sens_cfg[3].cfg.step_counter.step_duration_window = step_counter_param_set[index++];
    sens_cfg[3].cfg.step_counter.step_duration_pp_enabled = step_counter_param_set[index++];
    sens_cfg[3].cfg.step_counter.step_duration_thres = step_counter_param_set[index++];
    sens_cfg[3].cfg.step_counter.mean_crossing_pp_enabled = step_counter_param_set[index++];
    sens_cfg[3].cfg.step_counter.mcr_threshold = step_counter_param_set[index++];

    /* Set significant motion configuration */
    index = 0;
    sens_cfg[4].type = BMI330_SIG_MOTION;
    sens_cfg[4].cfg.sig_motion.block_size = sig_motion_param_set[index++];
    sens_cfg[4].cfg.sig_motion.peak_2_peak_min = sig_motion_param_set[index++];
    sens_cfg[4].cfg.sig_motion.mcr_min = (uint8_t)sig_motion_param_set[index++];
    sens_cfg[4].cfg.sig_motion.peak_2_peak_max = sig_motion_param_set[index++];
    sens_cfg[4].cfg.sig_motion.mcr_max = (uint8_t)sig_motion_param_set[index++];

    /* Set orientation configuration */
    index = 0;
    sens_cfg[5].type = BMI330_ORIENTATION;
    sens_cfg[5].cfg.orientation.ud_en = (uint8_t)orientation_param_set[index++];
    sens_cfg[5].cfg.orientation.mode = (uint8_t)orientation_param_set[index++];
    sens_cfg[5].cfg.orientation.blocking = (uint8_t)orientation_param_set[index++];
    sens_cfg[5].cfg.orientation.theta = (uint8_t)orientation_param_set[index++];
    sens_cfg[5].cfg.orientation.hold_time = (uint8_t)orientation_param_set[index++];
    sens_cfg[5].cfg.orientation.slope_thres = (uint8_t)orientation_param_set[index++];
    sens_cfg[5].cfg.orientation.hysteresis = (uint8_t)orientation_param_set[index++];

    /* Set the context configurations */
    rslt = bmi330_set_sensor_config(sens_cfg, BMI330_MAX_FEATURE, dev);

    return rslt;
}

/*!
 * @brief This API is used to perform the self-test for either accel or gyro or both.
 */
int8_t bmi330_perform_self_test(uint8_t st_selection, struct bmi3_st_result *st_result_status, struct bmi3_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    rslt = bmi3_perform_self_test(st_selection, st_result_status, dev);

    return rslt;
}

/*!
 * @brief This API writes the config array and config version in cfg res.
 */
int8_t bmi330_configure_enhanced_flexibility(struct bmi3_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Null-pointer check */
    rslt = null_ptr_check(dev);

    if (rslt == BMI3_OK)
    {
        /* Bytes written are multiples of 2 */
        if ((dev->read_write_len % 2) != 0)
        {
            dev->read_write_len = dev->read_write_len - 1;
        }

        /* BMI3 has 16 bit address and hence the minimum read write length should be 2 bytes */
        if (dev->read_write_len < 2)
        {
            dev->read_write_len = 2;
        }

        rslt = config_array_set_command(dev);

        if (rslt == BMI3_OK)
        {
            rslt = config_array_set_value_one_page(dev);

            if (rslt == BMI3_OK)
            {
                /* Write the config array */
                rslt = write_config_array(dev);
            }
        }
    }

    return rslt;
}

/*!
 * @brief This API is used to get the config version.
 */
int8_t bmi330_get_config_version(struct bmi3_config_version *config_version, struct bmi3_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    rslt = bmi3_get_config_version(config_version, dev);

    return rslt;
}

/*!
 * @brief This API is used to perform the self-calibration for either sensitivity or offset or both.
 */
int8_t bmi330_perform_gyro_sc(uint8_t sc_selection,
                              uint8_t apply_corr,
                              struct bmi3_self_calib_rslt *sc_rslt,
                              struct bmi3_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    rslt = bmi3_perform_gyro_sc(sc_selection, apply_corr, sc_rslt, dev);

    return rslt;
}

/*!
 * @brief This API is used to set the data sample rate for i3c sync
 */
int8_t bmi330_set_i3c_tc_sync_tph(uint16_t sample_rate, struct bmi3_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    rslt = bmi3_set_i3c_tc_sync_tph(sample_rate, dev);

    return rslt;
}

/*!
 * @brief This API is used to get the data sample rate for i3c sync
 */
int8_t bmi330_get_i3c_tc_sync_tph(uint16_t *sample_rate, struct bmi3_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    rslt = bmi3_get_i3c_tc_sync_tph(sample_rate, dev);

    return rslt;
}

/*!
 * @brief This API is used set the TU(time unit) value is used to scale the delay time payload
 * according to the hosts needs
 */
int8_t bmi330_set_i3c_tc_sync_tu(uint8_t delay_time, struct bmi3_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    rslt = bmi3_set_i3c_tc_sync_tu(delay_time, dev);

    return rslt;
}

/*!
 * @brief This API is used get the TU(time unit) value is used to scale the delay time payload
 * according to the hosts needs
 */
int8_t bmi330_get_i3c_tc_sync_tu(uint8_t *delay_time, struct bmi3_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    rslt = bmi3_get_i3c_tc_sync_tu(delay_time, dev);

    return rslt;
}

/*!
 * @brief This API is used to set the i3c sync ODR.
 */
int8_t bmi330_set_i3c_tc_sync_odr(uint8_t odr, struct bmi3_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    rslt = bmi3_set_i3c_tc_sync_odr(odr, dev);

    return rslt;
}

/*!
 * @brief This API is used to get the i3c sync ODR.
 */
int8_t bmi330_get_i3c_tc_sync_odr(uint8_t *odr, struct bmi3_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    rslt = bmi3_get_i3c_tc_sync_odr(odr, dev);

    return rslt;
}

/*!
 * @brief This internal API gets i3c sync i3c_tc_res
 */
int8_t bmi330_get_i3c_sync_i3c_tc_res(uint8_t *i3c_tc_res, struct bmi3_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    rslt = bmi3_get_i3c_sync_i3c_tc_res(i3c_tc_res, dev);

    return rslt;
}

/*!
 * @brief This internal API sets i3c sync i3c_tc_res
 */
int8_t bmi330_set_i3c_sync_i3c_tc_res(uint8_t i3c_tc_res, struct bmi3_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    rslt = bmi3_set_i3c_sync_i3c_tc_res(i3c_tc_res, dev);

    return rslt;
}

/*!
 * @brief This API is used to enable accel and gyro for alternate configuration
 */
int8_t bmi330_alternate_config_ctrl(uint8_t config_en, uint8_t alt_rst_conf, struct bmi3_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    rslt = bmi3_alternate_config_ctrl(config_en, alt_rst_conf, dev);

    return rslt;
}

/*!
 * @brief This API is used to read the status of alternate configuration
 */
int8_t bmi330_read_alternate_status(struct bmi3_alt_status *alt_status, struct bmi3_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    rslt = bmi3_read_alternate_status(alt_status, dev);

    return rslt;
}

/*!
 * @brief This API gets offset dgain for the sensor which stores self-calibrated values for accel.
 */
int8_t bmi330_get_acc_dp_off_dgain(struct bmi3_acc_dp_gain_offset *acc_dp_gain_offset, struct bmi3_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    rslt = bmi3_get_acc_dp_off_dgain(acc_dp_gain_offset, dev);

    return rslt;
}

/*!
 * @brief This API gets offset dgain for the sensor which stores self-calibrated values for gyro.
 */
int8_t bmi330_get_gyro_dp_off_dgain(struct bmi3_gyr_dp_gain_offset *gyr_dp_gain_offset, struct bmi3_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    rslt = bmi3_get_gyro_dp_off_dgain(gyr_dp_gain_offset, dev);

    return rslt;
}

/*!
 * @brief This API sets Offset dgain for the sensor which stores self-calibrated values for accel.
 */
int8_t bmi330_set_acc_dp_off_dgain(const struct bmi3_acc_dp_gain_offset *acc_dp_gain_offset, struct bmi3_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    rslt = bmi3_set_acc_dp_off_dgain(acc_dp_gain_offset, dev);

    return rslt;
}

/*!
 * @brief This API sets offset dgain for the sensor which stores self-calibrated values for gyro.
 */
int8_t bmi330_set_gyro_dp_off_dgain(const struct bmi3_gyr_dp_gain_offset *gyr_dp_gain_offset, struct bmi3_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    rslt = bmi3_set_gyro_dp_off_dgain(gyr_dp_gain_offset, dev);

    return rslt;
}

/*!
 * @brief This API gets user offset dgain for the sensor which stores self-calibrated values for accel.
 */
int8_t bmi330_get_user_acc_off_dgain(struct bmi3_acc_usr_gain_offset *acc_usr_gain_offset, struct bmi3_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    rslt = bmi3_get_user_acc_off_dgain(acc_usr_gain_offset, dev);

    return rslt;
}

/*!
 * @brief This API sets user offset dgain for the sensor which stores self-calibrated values for accel.
 */
int8_t bmi330_set_user_acc_off_dgain(const struct bmi3_acc_usr_gain_offset *acc_usr_gain_offset, struct bmi3_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    rslt = bmi3_set_user_acc_off_dgain(acc_usr_gain_offset, dev);

    return rslt;
}

/*!
 * @brief This API performs Fast Offset Compensation for accelerometer.
 */
int8_t bmi330_perform_accel_foc(const struct bmi3_accel_foc_g_value *accel_g_value, struct bmi3_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    rslt = bmi3_perform_accel_foc(accel_g_value, dev);

    return rslt;
}

/*!
 * @brief This API gets the data ready status of power on reset, accelerometer, gyroscope
 * and temperature.
 */
int8_t bmi330_get_sensor_status(uint16_t *status, struct bmi3_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Read the status register */
    rslt = bmi3_get_sensor_status(status, dev);

    return rslt;
}

/*!
 * @brief This API gets the I3C IBI status of both feature and data
 * interrupts
 */
int8_t bmi330_get_i3c_ibi_status(uint16_t *int_status, struct bmi3_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    rslt = bmi3_get_i3c_ibi_status(int_status, dev);

    return rslt;
}

/*!
 * @brief This API gets accel gyro offset gain reset values.
 */
int8_t bmi330_get_acc_gyr_off_gain_reset(uint8_t *acc_off_gain_reset, uint8_t *gyr_off_gain_reset, struct bmi3_dev *dev)
{
    /* Variable to store result of API */
    int8_t rslt;

    rslt = bmi3_get_acc_gyr_off_gain_reset(acc_off_gain_reset, gyr_off_gain_reset, dev);

    return rslt;
}

/*!
 * @brief This API sets accel gyro offset gain reset values.
 */
int8_t bmi330_set_acc_gyr_off_gain_reset(uint8_t acc_off_gain_reset, uint8_t gyr_off_gain_reset, struct bmi3_dev *dev)
{
    /* Variable to store result of API */
    int8_t rslt;

    rslt = bmi3_set_acc_gyr_off_gain_reset(acc_off_gain_reset, gyr_off_gain_reset, dev);

    return rslt;
}

/***************************************************************************/

/*!                   Local Function Definitions
 ****************************************************************************/

/*!
 * @brief This internal API is used to validate the device structure pointer for
 * null conditions.
 */
static int8_t null_ptr_check(const struct bmi3_dev *dev)
{
    int8_t rslt;

    if ((dev == NULL) || (dev->read == NULL) || (dev->write == NULL) || (dev->delay_us == NULL))
    {
        rslt = BMI330_E_NULL_PTR;
    }
    else
    {
        /* Device structure is fine */
        rslt = BMI330_OK;
    }

    return rslt;
}

/*!
 * @brief This internal API writes the command register value to enable cfg res.
 */
static int8_t config_array_set_command(struct bmi3_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Array to read feature engine GP1 error status */
    uint8_t feature_engine_gp_1[2] = { 0 };

    /* Array to get default value of cfg res register */
    uint8_t cfg_res[2] = { 0 };

    /* Get error status of feature engine GP1 register */
    rslt = bmi330_get_regs(BMI3_REG_FEATURE_IO1, feature_engine_gp_1, 2, dev);

    /* Check whether feature engine is activated in error status */
    if ((rslt == BMI330_OK) && (feature_engine_gp_1[0] == BMI3_FEAT_ENG_ACT_MASK))
    {
        /* Read default value of cfg res address */
        rslt = bmi330_get_regs(BMI3_REG_CFG_RES, cfg_res, 2, dev);
    }

    /* Burst write the command register value to enable cfg res */
    if (rslt == BMI330_OK)
    {
        rslt = bmi330_set_command_register(BMI3_CMD_2, dev);

        if (rslt == BMI330_OK)
        {
            rslt = bmi330_set_command_register(BMI3_CMD_1, dev);
        }
    }

    return rslt;
}

/*!
 * @brief This internal API sets the value_one to feature engine in cfg res.
 */
static int8_t config_array_set_value_one_page(struct bmi3_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Array to get value of cfg res register after writing the command register */
    uint8_t cfg_res[2] = { 0 };

    /* Array to store value_one */
    uint8_t cfg_res_value[2] = { BMI3_CFG_RES_VALUE_ONE, BMI3_CFG_RES_MASK };

    /* Read cfg res address */
    rslt = bmi330_get_regs(BMI3_REG_CFG_RES, cfg_res, 2, dev);

    if ((rslt == BMI3_OK) && (cfg_res[1] == BMI3_CFG_RES_MASK))
    {
        rslt = bmi330_set_regs(BMI3_REG_CFG_RES, cfg_res_value, 2, dev);
    }

    return rslt;
}

/*!
 * @brief This internal API writes the config array.
 */
static int8_t write_config_array(struct bmi3_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Array to reset the BMI3_REG_CFG_RES register */
    uint8_t reset[2] = { 0 };

    /* Download config code array */
    rslt = load_config_array(bmi330_ram_patch_extn_code, sizeof(bmi330_ram_patch_extn_code), dev);

    if (rslt == BMI3_OK)
    {
        /* Download config array table array */
        rslt = load_config_array(bmi330_ram_patch_extn_table, sizeof(bmi330_ram_patch_extn_table), dev);

        if (rslt == BMI3_OK)
        {
            /* Download config version */
            rslt = write_config_version(dev);
        }
    }

    if (rslt == BMI3_OK)
    {
        /* Reset the BMI3_REG_CFG_RES to return back to user page */
        rslt = bmi330_set_regs(BMI3_REG_CFG_RES, reset, 2, dev);
    }

    return rslt;
}

/*!
 * @brief This internal API writes config array.
 */
static int8_t load_config_array(const uint8_t *config_array, uint16_t config_size, struct bmi3_dev *dev)
{
    /* Variable to define error */
    int8_t rslt = BMI3_OK;

    /* Variable to loop */
    uint16_t indx;

    /* Variable to get the remainder */
    uint8_t remain = (uint8_t)((config_size - BMI3_CONFIG_ARRAY_DATA_START_ADDR) % dev->read_write_len);

    /* Variable to get the balance bytes */
    uint16_t bal_byte = 0;

    /* Variable to define temporary read/write length */
    uint16_t read_write_len = 0;

    /* Variable to store base address of config array */
    uint16_t config_base_addr;

    /* First two bytes of config array denotes the base address */
    config_base_addr = (config_array[0] | (uint16_t)(config_array[1] << 8));

    if (!remain)
    {
        for (indx = BMI3_CONFIG_ARRAY_DATA_START_ADDR; indx < config_size; indx += dev->read_write_len)
        {
            rslt = upload_file(config_base_addr, config_array, indx, dev);

            /* Increment the base address */
            config_base_addr = config_base_addr + (dev->read_write_len / 2);
        }
    }
    else
    {
        /* Get the balance bytes */
        bal_byte = (uint16_t) config_size - (uint16_t) remain;

        /* Write the configuration file for the balance bytes */
        for (indx = BMI3_CONFIG_ARRAY_DATA_START_ADDR; indx < bal_byte; indx += dev->read_write_len)
        {
            rslt = upload_file(config_base_addr, config_array, indx, dev);

            /* Increment the base address */
            config_base_addr = config_base_addr + (dev->read_write_len / 2);
        }

        if (rslt == BMI3_OK)
        {
            /* Update length in a temporary variable */
            read_write_len = dev->read_write_len;

            /* Write the remaining bytes in 2 bytes length */
            dev->read_write_len = 2;

            /* Check if balance byte is zero, then replace with config array data start address. */
            if (bal_byte == 0)
            {
                bal_byte = BMI3_CONFIG_ARRAY_DATA_START_ADDR;
            }

            /* Write the configuration file for the remaining bytes */
            for (indx = bal_byte; indx < config_size; indx += dev->read_write_len)
            {
                rslt = upload_file(config_base_addr, config_array, indx, dev);

                /* Increment the base address */
                config_base_addr = config_base_addr + (dev->read_write_len / 2);
            }

            /* Restore the user set length back from the temporary variable */
            dev->read_write_len = read_write_len;
        }
    }

    return rslt;
}

/*!
 * @brief This internal API writes config array code array and table array to feature engine register.
 */
static int8_t upload_file(uint16_t config_base_addr, const uint8_t *config_array, uint16_t indx, struct bmi3_dev *dev)
{
    int8_t rslt;

    uint8_t base_addr[2];

    base_addr[0] = config_base_addr & BMI3_SET_LOW_BYTE;
    base_addr[1] = (uint8_t)((config_base_addr & BMI3_SET_HIGH_BYTE) >> 8);

    /* Set the config array base address to feature engine transmission address to start DMA transaction */
    rslt = bmi330_set_regs(BMI3_REG_FEATURE_DATA_ADDR, base_addr, 2, dev);

    if (rslt == BMI3_OK)
    {
        /* Set the config array data to feature engine transmission data address to start DMA transaction */
        rslt = bmi330_set_regs(BMI3_REG_FEATURE_DATA_TX, &config_array[indx], dev->read_write_len, dev);
    }

    return rslt;
}

/*!
 * @brief This internal API writes config version array to feature engine register.
 */
static int8_t write_config_version(struct bmi3_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Index variable to store the start address of data */
    uint8_t indx = BMI3_CONFIG_ARRAY_DATA_START_ADDR;

    /* Set the config array base address to feature engine transmission address to start DMA transaction */
    rslt = bmi330_set_regs(BMI3_REG_FEATURE_DATA_ADDR, bmi330_ram_version, 2, dev);

    if (rslt == BMI3_OK)
    {
        /* Set the config version data to feature engine transmission data address to start DMA transaction */
        rslt = bmi330_set_regs(BMI3_REG_FEATURE_DATA_TX, &bmi330_ram_version[indx], 2, dev);
    }

    return rslt;
}

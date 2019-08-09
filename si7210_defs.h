/*
 * MIT License
 * 
 * Copyright (c) 2019 Sean Farrelly
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
 * File        Si7210_defs.h
 * Created by  Sean Farrelly
 * Version     1.0
 * 
 */

/*! @file Si7210_defs.h
 * @brief Definitions used by driver/user for Si7210 Sensor API.
 */

/*!
 * @defgroup SI7210 DEF API
 */
#ifndef _SI7210_DEFS_H_
#define _SI7210_DEFS_H_

/*! CPP guard */
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Possible I2C slave addresses */
#define SI7210_ADDRESS_0    (0x30U << 1)
#define SI7210_ADDRESS_1    (0x31U << 1)
#define SI7210_ADDRESS_2    (0x32U << 1)
#define SI7210_ADDRESS_3    (0x33U << 1)

/* Vdd of SI7210. This is used in device's self-test calculations */
#define SI7210_VDD          (3.3f)

/*!
 * @brief Si7210 API status result code.
 */
typedef enum {
    /* API Success codes */
    SI7210_OK,
    /* API Error codes */
    SI7210_E_NULL_PTR,
    SI7210_E_INVALID_ARG,
    SI7210_E_IO,
    SI7210_E_DEV_NOT_FOUND,
    SI7210_E_SELF_TEST_FAIL,
    /* API Warning codes */
    SI7210_W_THRESHOLD_BOUNDS // Threshold out of bounds
} si7210_status_t;

/*!
 * @brief Measurement scale of reading.
 */
typedef enum {
    SI7210_20mT,
    SI7210_200mT
} si7210_range_t;

/*!
 * @brief State of Si7210's output pin when field is above threshold setting.
 */
typedef enum {
    /* Output pin is LOW when the field is above threshold setting. */
    SI7210_OUTPUT_PIN_LOW,
    /* Ouput pin is HIGH when the field is above threshold setting. */
    SI7210_OUTPUT_PIN_HIGH
} si7210_output_pin_t;

/*!
 * @brief Type definitions
 */
typedef si7210_status_t (*si7210_com_fptr_t)(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);
typedef void (*si7210_delay_fptr_t)(uint32_t period);
typedef void (*si7210_callback_fptr_t)(void *context);

/*!
 * @brief Si7210 calibration data structure
 */
struct si7210_calib_data {

    /* Temperature offset */
    int8_t temp_offset;
    
    /* Temperature gain */
    int8_t temp_gain;
};

/*!
 * @brief Si7210 sensor structure
 */
struct si7210_dev {

    /*! Device ID */
    uint8_t dev_id;
    
    /*! I2C Read function pointer */
    si7210_com_fptr_t read;

    /*! I2C Write function pointer */
    si7210_com_fptr_t write;

    /*! Delay (ms) function pointer */
    si7210_delay_fptr_t delay_ms;

    /*! Threshold callback function pointer */
    si7210_callback_fptr_t callback;

    /*! Calibration data */
    struct si7210_calib_data calib_data;
};

#ifdef __cplusplus
}
#endif /* End of CPP guard */
#endif /* SI7210_H_ */
/** @}*/
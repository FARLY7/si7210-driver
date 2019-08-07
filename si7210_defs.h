#ifndef _SI7210_DEFS_H_
#define _SI7210_DEFS_H_

/* Header includes */
#include <stdint.h>

/* Possible I2C slave addresses */
#define SI7210_ADDRESS_0    (0x30U << 1)
#define SI7210_ADDRESS_1    (0x31U << 1)
#define SI7210_ADDRESS_2    (0x32U << 1)
#define SI7210_ADDRESS_3    (0x33U << 1)

/* Vdd of SI7210. This is used in device's self-test calculations */
#define SI7210_VDD          (3.3f)

/*!
 * @brief Si7210 API status result code
 */
typedef enum {
    /* API Success codes */
    SI7210_OK,
    /* API Error codes */
    SI7210_E_NULL_PTR,
    SI7210_E_IO,
    SI7210_E_DEV_NOT_FOUND,
    SI7210_E_INVALID_RANGE,
    SI7210_E_SELF_TEST_FAIL
    /* API Warning codes */
} si7210_status;

enum si7210_range {
    SI7210_20mT,
    SI7210_200mT
};

/*!
 * @brief Type definitions
 */
typedef si7210_status (*si7210_com_fptr_t)(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);
typedef void (*si7210_delay_fptr_t)(uint32_t period);

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

    /*! Calibration data */
    struct si7210_calib_data calib_data;
};

#endif

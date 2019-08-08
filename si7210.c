/* Header includes */
#include "si7210.h"
#include <stddef.h>
#include <math.h>

/* Si7210 Register addresses */
#define SI72XX_HREVID       0xC0U
#define SI72XX_DSPSIGM      0xC1U
#define SI72XX_DSPSIGL      0xC2U
#define SI72XX_DSPSIGSEL    0xC3U
#define SI72XX_POWER_CTRL   0xC4U
#define SI72XX_ARAUTOINC    0xC5U
#define SI72XX_CTRL1        0xC6U
#define SI72XX_CTRL2        0xC7U
#define SI72XX_SLTIME       0xC8U
#define SI72XX_CTRL3        0xC9U
#define SI72XX_A0           0xCAU
#define SI72XX_A1           0xCBU
#define SI72XX_A2           0xCCU
#define SI72XX_CTRL4        0xCDU
#define SI72XX_A3           0xCEU
#define SI72XX_A4           0xCFU
#define SI72XX_A5           0xD0U
#define SI72XX_OTP_ADDR     0xE1U
#define SI72XX_OTP_DATA     0xE2U
#define SI72XX_OTP_CTRL     0xE3U
#define SI72XX_TM_FG        0xE4U

/* Si7210 Register bit masks */
#define CHIP_ID_MASK        0xF0U
#define REV_ID_MASK         0x0FU
#define DSP_SIGSEL_MASK     0x07U
#define MEAS_MASK           0x80U
#define USESTORE_MASK       0x08U
#define ONEBURST_MASK       0x04U
#define STOP_MASK           0x02U
#define SLEEP_MASK          0x01U
#define ARAUTOINC_MASK      0x01U
#define SW_LOW4FIELD_MASK   0x80U
#define SW_OP_MASK          0x7FU
#define SW_FIELDPOLSEL_MASK 0xC0U
#define SW_HYST_MASK        0x3FU
#define SW_TAMPER_MASK      0xFCU
#define SL_FAST_MASK        0x02U
#define SL_TIMEENA_MASK     0x01U
#define DF_BURSTSIZE_MASK   0xE0U
#define DF_BW_MASK          0x1EU
#define DF_IIR_MASK         0x01U
#define OTP_READ_EN_MASK    0x02U
#define OTP_BUSY_MASK       0x01U
#define TM_FG_MASK          0x03U

#define DSP_SIGM_DATA_FLAG      0x80U
#define DSP_SIGM_DATA_MASK      0x7FU
#define DSP_SIGSEL_TEMP_MASK    0x01U
#define DSP_SIGSEL_FIELD_MASK   0x04U

/* Burst sizes */
#define DF_BW_1             0x0U << 1
#define DF_BW_2             0x1U << 1
#define DF_BW_4             0x2U << 1
#define DF_BW_8             0x3U << 1
#define DF_BW_16            0x4U << 1
#define DF_BW_32            0x5U << 1
#define DF_BW_64            0x6U << 1
#define DF_BW_128           0x7U << 1
#define DF_BW_256           0x8U << 1
#define DF_BW_512           0x9U << 1
#define DF_BW_1024          0xAU << 1
#define DF_BW_2048          0xBU << 1
#define DF_BW_4096          0xCU << 1
#define DF_BURSTSIZE_1      0x0U << 5
#define DF_BURSTSIZE_2      0x1U << 5
#define DF_BURSTSIZE_4      0x2U << 5
#define DF_BURSTSIZE_8      0x3U << 5
#define DF_BURSTSIZE_16     0x4U << 5
#define DF_BURSTSIZE_32     0x5U << 5
#define DF_BURSTSIZE_64     0x6U << 5
#define DF_BURSTSIZE_128    0x7U << 5

/*!
 * @brief This internal API is used to validate the device pointer for
 * null conditions.
 *
 * @param[in] dev : Structure instance of si210_dev.
 *
 * @return Result of API execution status
 * @retval SI7210_OK -> Success / SI7210_E_NULL_PTR -> Error
 */
static si7210_status null_ptr_check(const struct si7210_dev *dev);

/**
  * @brief  Initialise Si7210 device and check if responding
  */
si7210_status si7210_init(struct si7210_dev *dev)
{
    si7210_status rslt;

    uint8_t try_count = 5;

    /* Check for null pointer in device structure */
    if((rslt = null_ptr_check(dev)) != SI7210_OK)
        goto error;

    while(try_count)
    {
        /* Check if device is responding */
        if((rslt = si7210_check(dev)) == SI7210_OK)
            break;

        /* Wait for 10ms and count unsuccessful attempts */
        dev->delay_ms(10);
        try_count--;
    }

error:
    return rslt;
}

/*!
  * @brief This API gets the last measured field strength from the device.
  *        The value is correctly compensated.
  */
si7210_status si7210_get_field_strength(struct si7210_dev *dev, enum si7210_range range, float *field)
{
    /* Temp variable used for register read values */
    uint8_t val = 0;

    si7210_status rslt;

    /* Check for null pointer in device structure */
    if((rslt = null_ptr_check(dev)) != SI7210_OK)
        goto error;
   
    /* Stop the control loop by setting stop bit */
    si7210_write_reg(dev, SI72XX_POWER_CTRL, 0xF0, USESTORE_MASK | STOP_MASK);

    if (range == SI7210_200mT)
    {
        si7210_write_reg(dev, SI72XX_OTP_ADDR, 0, 0x27U);
        si7210_write_reg(dev, SI72XX_OTP_CTRL, 0, OTP_READ_EN_MASK);
        si7210_read_reg (dev, SI72XX_OTP_DATA, &val);
        si7210_write_reg(dev, SI72XX_A0, 0, val);

        si7210_write_reg(dev, SI72XX_OTP_ADDR, 0, 0x28U);
        si7210_write_reg(dev, SI72XX_OTP_CTRL, 0, OTP_READ_EN_MASK);
        si7210_read_reg (dev, SI72XX_OTP_DATA, &val);
        si7210_write_reg(dev, SI72XX_A1, 0, val);
        
        si7210_write_reg(dev, SI72XX_OTP_ADDR, 0, 0x29U);
        si7210_write_reg(dev, SI72XX_OTP_CTRL, 0, OTP_READ_EN_MASK);
        si7210_read_reg (dev, SI72XX_OTP_DATA, &val);
        si7210_write_reg(dev, SI72XX_A2, 0, val);
        
        si7210_write_reg(dev, SI72XX_OTP_ADDR, 0, 0x2AU);
        si7210_write_reg(dev, SI72XX_OTP_CTRL, 0, OTP_READ_EN_MASK);
        si7210_read_reg (dev, SI72XX_OTP_DATA, &val);
        si7210_write_reg(dev, SI72XX_A3, 0, val);
        
        si7210_write_reg(dev, SI72XX_OTP_ADDR, 0, 0x2BU);
        si7210_write_reg(dev, SI72XX_OTP_CTRL, 0, OTP_READ_EN_MASK);
        si7210_read_reg (dev, SI72XX_OTP_DATA, &val);
        si7210_write_reg(dev, SI72XX_A4, 0, val);
        
        si7210_write_reg(dev, SI72XX_OTP_ADDR, 0, 0x2CU);
        si7210_write_reg(dev, SI72XX_OTP_CTRL, 0, OTP_READ_EN_MASK);
        si7210_read_reg (dev, SI72XX_OTP_DATA, &val);
        si7210_write_reg(dev, SI72XX_A5, 0, val);
    }

    /* Use a burst size of 4 samples in FIR and IIR modes */
    //si7210_write_reg(dev, SI72XX_CTRL4, 0, DF_BURSTSIZE_4 | DF_BW_4);
    si7210_write_reg(dev, SI72XX_CTRL4, 0, DF_BURSTSIZE_128 | DF_BW_4096);

    /* Selet field strength measurement */
    si7210_write_reg(dev, SI72XX_DSPSIGSEL, 0, DSP_SIGSEL_FIELD_MASK);    

    /* Start measurement */
    si7210_write_reg(dev, SI72XX_POWER_CTRL, 0xF0, USESTORE_MASK | ONEBURST_MASK);
    
    /* Wait until measurement complete by checking fresh data bit */
    do {
        /* Read most-significant byte */
        si7210_read_reg(dev, SI72XX_DSPSIGM, &val);
    } while( (val & DSP_SIGM_DATA_FLAG) == 0);
    
    /*! ================================================== */
    /*! For calculations below, refer to Si7210 datasheet. */
    /*! ================================================== */

    int32_t value = 256 * (val & DSP_SIGM_DATA_MASK);

    /* Read least-significant byte of data */
    si7210_read_reg(dev, SI72XX_DSPSIGL, &val);
        
    value += val;
    value -= 16384U;

    float raw_field = (float) value;

    if(range == SI7210_20mT)
        *field = raw_field * 0.00125; /* rawField * 1.25 */
    else if(range == SI7210_200mT)
        *field = raw_field * 0.0125; /* rawField * 12.5 */
    else
        rslt = SI7210_E_INVALID_ARG;

error:
    return rslt;
}

/*!
  * @brief This API gets the last measured temperature from the device.
  *        The value is correctly compensated.
  */
si7210_status si7210_get_temperature(struct si7210_dev *dev, float *temperature)
{
    /* Temp variable used for register read values */
    uint8_t val = 0;

    si7210_status rslt;

    /* Check for null pointer in device structure */
    if((rslt = null_ptr_check(dev)) != SI7210_OK)
        goto error;

    /* Stop the control loop by setting stop bit */
    si7210_write_reg(dev, SI72XX_POWER_CTRL, 0xF0, USESTORE_MASK | STOP_MASK);

    /* Do not use burst measurement type */
    si7210_write_reg(dev, SI72XX_CTRL4, 0, 0x0);

    /* Select temperature measurement */
    si7210_write_reg(dev, SI72XX_DSPSIGSEL, 0, DSP_SIGSEL_TEMP_MASK);

    /* Start measurement */
    si7210_write_reg(dev, SI72XX_POWER_CTRL, 0xF0, USESTORE_MASK | ONEBURST_MASK);

    /* Wait until measurement complete by checking fresh data bit */
    do {
        /* Read most-significant byte */
        si7210_read_reg(dev, SI72XX_DSPSIGM, &val);
    } while( (val & DSP_SIGM_DATA_FLAG) == 0);

    /*! ================================================== */
    /*! For calculations below, refer to Si7210 datasheet. */
    /*! ================================================== */

    int32_t value = 32 * (val & DSP_SIGM_DATA_MASK);
    
    /* Read the least-significant byte */
    si7210_read_reg(dev, SI72XX_DSPSIGL, &val);
    value += (val >> 3);

    /* If no offset and gain values exist, read them now. */
    if (dev->calib_data.temp_offset == 0 && dev->calib_data.temp_gain == 0)
    {
        /* Read out compensation values */
        si7210_write_reg(dev, SI72XX_OTP_ADDR, 0, 0x1DU);
        si7210_write_reg(dev, SI72XX_OTP_CTRL, 0, OTP_READ_EN_MASK);
        si7210_read_reg(dev, SI72XX_OTP_DATA, (uint8_t*) &dev->calib_data.temp_offset);

        si7210_write_reg(dev, SI72XX_OTP_ADDR, 0, 0x1EU);
        si7210_write_reg(dev, SI72XX_OTP_CTRL, 0, OTP_READ_EN_MASK);
        si7210_read_reg(dev, SI72XX_OTP_DATA, (uint8_t*) &dev->calib_data.temp_gain);
    }

    float temp_c = (float) value;
    float offset = dev->calib_data.temp_offset / 16.0;
    float gain   = 1 + (dev->calib_data.temp_gain / 2048.0);
    
    temp_c = gain * (-3.83e-6F * temp_c * temp_c + 0.16094F * temp_c - 279.80F - 0.222F * 3.3F) + offset;

    *temperature = temp_c;

error:
    return rslt;
}

si7210_status si7210_set_threshold(struct si7210_dev *dev, float threshold, enum si7210_range range, enum si7210_output_pin pin)
{
    si7210_status rslt;
    uint8_t temp = 0;

    /* Check for null pointer in device structure */
    if((rslt = null_ptr_check(dev)) != SI7210_OK)
        goto error;
    
    if((pin != SI7210_OUTPUT_PIN_HIGH) && (pin != SI7210_OUTPUT_PIN_LOW) &&
       (range != SI7210_200mT) && (range != SI7210_20mT))
    {
        rslt = SI7210_E_INVALID_ARG;
        goto error;
    }

    // /* Save OTP registers */
    // si7210_write_reg(dev, SI72XX_POWER_CTRL, 0xF0, USESTORE_MASK);
    // uint8_t val = 0;
    // si7210_read_reg(dev, SI72XX_CTRL1, &val); // Read SW_OP
    // si7210_read_reg(dev, SI72XX_CTRL2, &val); // Read SW_HYST
    // si7210_read_reg(dev, SI72XX_CTRL3, &val); // Read SW_TAMPER
    
    si7210_write_reg(dev, SI72XX_CTRL2, 0, 63); // Set SW_HYST to 0
    

    /* Set sw_low4field bit if output pin is in LOW configuration.
     * Clear sw_low4field bit if output is in HIGH configuration.*/
    switch(pin)
    {
        case SI7210_OUTPUT_PIN_LOW:  temp |= SW_LOW4FIELD_MASK;  break;
        case SI7210_OUTPUT_PIN_HIGH: temp &= ~SW_LOW4FIELD_MASK; break;
    }

    switch(range)
    {
        case SI7210_200mT: threshold /= 0.05;  break; // 200mT
        case SI7210_20mT:  threshold /= 0.005; break; // 20mT
    }

    /* Keep threshold within supported bounds */
    if(threshold < 16)
        threshold = 16;    
    else if(threshold > 3840)
        threshold = 3840;

    uint8_t divides = 0;

    while((threshold >= 16) && (threshold >= 31))
    {
        threshold = threshold / 2;
        divides++;
    }

    threshold -= 16;
    threshold = roundf(threshold);

    uint8_t sw_op_3_0 = (uint8_t) threshold;
    uint8_t sw_op_6_4 = divides;




    // Set SW_OP and SW_LOW4FIELD
    if((rslt = si7210_write_reg(dev, SI72XX_CTRL1, 0, TEMP)) != SI7210_OK)
        goto error;
    

error:
    return rslt;
}

void si7210_irq_handler(struct si7210_dev *dev)
{

}

/*!
  * @brief This API reads a register from Si7210 device.
  */
si7210_status si7210_read_reg(struct si7210_dev *dev, uint8_t reg, uint8_t *val)
{
    si7210_status rslt;

    /* Check for null pointer in device structure */
    if((rslt = null_ptr_check(dev)) != SI7210_OK)
        goto error;

    /* Call user implemented method for I2C read */
    if((rslt = dev->read(dev->dev_id, reg, val, 1)) != SI7210_OK)
        goto error;

error:
    return rslt;
}

/*!
  * @brief This API writes a value to a register of Si7210 device.
  */
si7210_status si7210_write_reg(struct si7210_dev *dev, uint8_t reg, uint8_t mask, uint8_t val)
{
    si7210_status rslt;

    uint8_t temp_val = 0;

    /* Check for null pointer in device structure */
    if((rslt = null_ptr_check(dev)) != SI7210_OK)
        goto error;

    /* A mask has been used, must read original value from register. */
    if(mask != 0)
    {
        /* Call user implemented method for I2C read */
        if((rslt = dev->read(dev->dev_id, reg, &temp_val, 1)) != SI7210_OK)
        goto error;

        temp_val &= mask;
    }

    temp_val |= val;

    /* Call user implemented method for I2C write */
    if((rslt = dev->write(dev->dev_id, reg, &temp_val, 1)) != SI7210_OK)
        goto error;

error:
    return rslt;
}


/*!
  * @brief This API checks the device is responding to commands.
  */
si7210_status si7210_check(struct si7210_dev *dev)
{
    si7210_status rslt = SI7210_OK;
    uint8_t temp;

    /* Check for null pointer in device structure */
    if((rslt = null_ptr_check(dev)) != SI7210_OK)
        goto error;
    
    /* Read the device ID and revision fields to check comms. */
    if((rslt = si7210_read_reg(dev, SI72XX_HREVID, &temp)) != SI7210_OK)
        goto error;

error:
    return rslt;
}

/*!
  * @brief This API puts the device into SLEEP mode.
 */
si7210_status si7210_sleep(struct si7210_dev *dev)
{
    uint8_t temp;

    si7210_status rslt;
    
    /* Check for null pointer in device structure */
    if((rslt = null_ptr_check(dev)) != SI7210_OK)
        goto error;
    
    /* Read CTRL3 register */
    if((rslt = si7210_read_reg(dev, SI72XX_CTRL3, &temp)) != SI7210_OK)
        goto error;
     
    temp &= 0xFEU; /* Clear SLTIMENA bit of CTRL3 register */
    
    /* Write back new CTRL3 register value  */
    if((rslt = si7210_write_reg(dev, SI72XX_CTRL3, 0, temp)) != SI7210_OK)
        goto error;

    /* Read POWER_CTRL register */
    if((rslt = si7210_read_reg(dev, SI72XX_POWER_CTRL, &temp)) != SI7210_OK)
        goto error;
    
    temp = (temp & 0xF8U) | 0x01; /* Clear STOP and set SLEEP bits */
    
    /* Write back POWER_CTRL register value */
    if((rslt = si7210_write_reg(dev, SI72XX_POWER_CTRL, 0, temp)) != SI7210_OK)
        goto error;
        
error:
    return rslt;
}

/*!
 * @brief This API wakes the device from SLEEP mode.
 */ 
si7210_status si7210_wakeup(struct si7210_dev *dev)
{
    uint8_t temp = 0;
    si7210_status rslt;

    /* Check for null pointer in device structure */
    if((rslt = null_ptr_check(dev)) != SI7210_OK)
        goto error;

    /* Wake the device up by sending a WRITE request. 
     * API call will fail as we are unable to write to memory address 0x00,
     * however it will wake the part up. */
    dev->write(dev->dev_id, 0x00, &temp, 1);
        
error:
    return rslt;
}

/*!
 * @brief This API executes a self-test sequence offered by the device.
 *        It uses an internal coil to generate and test the + and - field.
 */
si7210_status si7210_self_test(struct si7210_dev *dev)
{
    float field_pos, field_neg;
    si7210_status rslt;

    /* Check for null pointer in device structure */
    if((rslt = null_ptr_check(dev)) != SI7210_OK)
        goto error;

    /* Enable test field generator coil in POSITIVE direction. */
    si7210_write_reg(dev, SI72XX_TM_FG, 0, 1);

    /* Measure field strength */
    si7210_get_field_strength(dev, SI7210_200mT, &field_pos);

    /* Enable test field generator coil in POSITIVE direction. */
    si7210_write_reg(dev, SI72XX_TM_FG, 0, 2);

    /* Measure field strength */
    si7210_get_field_strength(dev, SI7210_200mT, &field_neg);

    /* Disable test field generator coil. */
    si7210_write_reg(dev, SI72XX_TM_FG, 0, 0);

    float b_out = 1.16 * SI7210_VDD;
    float b_upper = b_out + (b_out * 0.25); /* +25% */
    float b_lower = b_out - (b_out * 0.25); /* -25% */

    if( (field_pos <= b_upper) &&
        (field_pos >= b_lower) &&
        (field_neg >= (b_upper * -1)) &&
        (field_neg <= (b_lower * -1)))
    {
        rslt = SI7210_OK;
    }
    else
    {
        rslt = SI7210_E_SELF_TEST_FAIL;
    }

error:
    return rslt;
}


/*!
 * @brief This internal API is used to validate the device pointer for
 * null conditions.
 */
static si7210_status null_ptr_check(const struct si7210_dev *dev)
{
    si7210_status rslt;

    if (dev == NULL || dev->read == NULL || dev->write == NULL)
    {
        /* Device structure pointer is not valid */
        rslt = SI7210_E_NULL_PTR;
    }
    else
    {
        /* Device structure is fine */
        rslt = SI7210_OK;
    }

    return rslt;
}

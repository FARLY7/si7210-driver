#ifndef _SI7210_H_
#define _SI7210_H_

/* Header includes */
#include "si7210_defs.h"
#include <stdint.h>


/**
  * @brief  Initialise Si7210 device and check if responding
  * 
  * @param[in] dev : Si7210 device structure
  * 
  * @return result of API execution status
  * @retval si7210_status
  */
si7210_status si7210_init(struct si7210_dev *dev);

/*!
  * @brief This API gets the last measured field strength from the device.
  *        The value is correctly compensated.
  * 
  * @param[in]  dev   : Si7210 device structure.
  * @param[out] field : Pointer to value which is to be written.
  *  
  * @return Success of operation
  * @retval si7210_status
  */
si7210_status si7210_get_field_strength(struct si7210_dev *dev, enum si7210_range range, float *field);

/*!
  * @brief This API gets the last measured temperature from the device.
  *        The value is correctly compensated.
  * 
  * @param[in]  dev         : Si7210 device structure.
  * @param[out] temperature : Pointer to value which is to be written.
  *  
  * @return Success of operation
  * @retval si7210_status
  */
si7210_status si7210_get_temperature(struct si7210_dev *dev, float *temperature);



void si7210_irq_handler(struct si7210_dev *dev);
si7210_status si7210_set_threshold(struct si7210_dev *dev, float threshold, enum si7210_output_pin pin);

/*!
  * @brief This API reads a register from Si7210 device.
  * 
  * @param[in] dev : Si7210 device structure.
  * @param[in] reg : Register number to read from.
  * @param[out] data : Pointer to where register value is to be stored.
  *  
  * @return Success of read operation.
  * @retval si7210_status
  */
si7210_status si7210_read_reg(struct si7210_dev *dev, uint8_t reg, uint8_t *val);

/*!
  * @brief This API writes a value to a register of Si7210 device.
  * 
  * @param[in] dev : Si7210 device structure.
  * @param[in] reg : Register number to write to.
  * @param[in] mask : 1-byte mask used to keep original register bits.
  * @param[in] data : 1-byte data to write to register.
  * 
  * @return Success of write operation.
  * @retval si7210_status
  */
si7210_status si7210_write_reg(struct si7210_dev *dev, uint8_t reg, uint8_t mask, uint8_t val);

/*!
  * @brief This API checks if the device is responding.
  * 
  * @param[in] dev : Si7210 device structure.
  * 
  * @return Success of operation.
  * @retval si7210_status
  */
si7210_status si7210_check(struct si7210_dev *dev);

/*!
  * @brief This API executes a self-test sequence offered by the device.
  *        It uses an internal coil to generate and test the + and - field.
  * 
  * @note Important: There must not be any external magnetic field in the vicinity
  *       for the test to run successfully.
  *       The user must also ensure the correct Vdd value is defined in si7210_defs.h.
  * 
  * @param[in] dev : Si7210 device structure.
  * 
  * @return Success of operation.
  * @retval si7210_status
  */
si7210_status si7210_self_test(struct si7210_dev *dev);

/*!
  * @brief This API puts the device into SLEEP mode.
  * 
  * @param[in] dev : Si7210 device structure.
  * 
  * @return Success of operation.
  * @retval si7210_status
  */
si7210_status si7210_sleep(struct si7210_dev *dev);

/*!
  * @brief This API wakes the device from SLEEP mode.
  * 
  * @param[in] dev : Si7210 device structure.
  * @param[in] reg : Register number to write to.
  * @param[in] data : 1-byte data to write to register.
  * 
  * @return Success of operation.
  * @retval si7210_status
  */
si7210_status si7210_wakeup(struct si7210_dev *dev);

#endif

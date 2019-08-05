# Si7210 Hall-effect sensor driver API
A simple driver for Silicon Lab's Si7210 hall-effect sensor.


## Introduction
This package contains the Si7210 sensor driver.
The driver includes si7210.c, si7210.h, and si7210_defs.h files.

## Integration details
* Integrate si7210.c, si7210.h, and si7210_defs.h files into the project
* Include the si7210.h file in your code like below.

``` c
#include "si7210.h"
```

## File information
* si7210_defs.h : This header file contains definitions used by both the user and driver APIs.
* si7210.h : This header file contains the declarations of the driver APIs.
* lmt01.c : This source file contains the definitions of the driver APIs.

## Supported interfaces
* I2C

## User Guide

### Initialising the device
To intialise the device, the user must create a device structure. The user can do this by creating an instance of the structure si7210_dev. The user must then fill in the various parameters as shown below.

``` c
si7210_status rslt = SI7210_OK;
strut si7210_dev dev;

dev.dev_id = SI7210_ADDRESS_0;
dev.read = usr_i2c_read;
dev.write = usr_i2c_write;
dev.delay_ms = usr_delay_ms;

rslt = si7210_init(&dev);

if(rslt == SI7210_OK)
{
  int32_t field_strength;
  int64_t temperature;
  
  /* Obtain field strength reading from device */
  si7210_get_field_strength(&dev, SI7210_200mT, &field_strength);
  
  /* Obtain a temperature reading from the device */
  si7210_get_temperature(&dev, &temperature);
}
````

### Templates for function pointers
``` c
si7210_status usr_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
  si7210_status rslt = SI7210_OK;
  
  /* User implemented I2C read function */
  
  return rslt;
}

si7210_status usr_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
  si7210_status rslt = SI7210_OK;
  
  /* User implemented I2C write function */
  
  return rslt;
}

void usr_delay_ms(uint32_t period_ms)
{
  /* User implemented delay (ms) function */
}

```

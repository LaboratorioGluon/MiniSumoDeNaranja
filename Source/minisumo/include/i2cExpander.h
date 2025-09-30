#ifndef __I2C_EXPANDER_H__
#define __I2C_EXPANDER_H__

#include "driver/gpio.h"

class i2cExpander
{
public:
    i2cExpander(gpio_num_t SDA, gpio_num_t SCL, uint8_t devAddr = 0x70);

    esp_err_t InitI2cMaster();
    void selectPort(uint8_t p);

private:
    
    gpio_num_t pinSda, pinScl;
    uint8_t i2cAddr;
    int i2c_master_port;

};

#endif //__I2C_EXPANDER_H__
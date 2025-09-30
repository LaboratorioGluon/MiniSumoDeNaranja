#include "i2cExpander.h"

#include "driver/i2c.h"

i2cExpander::i2cExpander(gpio_num_t SDA, gpio_num_t SCL, uint8_t devAddr)
    : pinSda(SDA), pinScl(SCL), i2cAddr(devAddr), i2c_master_port(0)
{
}

esp_err_t i2cExpander::InitI2cMaster()
{

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = pinSda,
        .scl_io_num = pinScl,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = 
            { 
                .clk_speed = 100000
            },      
    };
    esp_err_t err = i2c_param_config(i2c_master_port, &conf);
    if (err != ESP_OK)
    {
        return err;
    }
    return i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
}

void i2cExpander::selectPort(uint8_t p)
{
    uint8_t data[] = { 1 << p };
    i2c_master_write_to_device(i2c_master_port, 0x70, data, 1, portMAX_DELAY);
}
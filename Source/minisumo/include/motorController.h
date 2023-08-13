#ifndef __MOTORCONTROLLER_H__
#define __MOTORCONTROLLER_H__

#include "driver/gpio.h"

class MotorController
{

public:

    enum DIRECTION
    {
        STOP = 0,
        FWD,
        RIGHT,
        LEFT,
        BACK
    };

    MotorController(gpio_num_t motorA1_, gpio_num_t motorA2_, gpio_num_t motorB1_, gpio_num_t motorB2_);

    void Init();

    void setDirection(DIRECTION dir);

private:

    gpio_num_t motorA1, motorA2;
    gpio_num_t motorB1, motorB2;

};


#endif ////__MOTORCONTROLLER_H__
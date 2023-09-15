#ifndef __MOTORCONTROLLER_H__
#define __MOTORCONTROLLER_H__

#include "driver/gpio.h"

/**
 * Driver for TB6612FNG
*/
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

    MotorController( gpio_num_t motorA1_, gpio_num_t motorA2_, gpio_num_t motorAPwm_, 
                     gpio_num_t motorB1_, gpio_num_t motorB2_, gpio_num_t motorBPwm_  );

    void Init();

    void setDirection(DIRECTION dir);

private:

    gpio_num_t motorA1, motorA2, motorAPwm;
    gpio_num_t motorB1, motorB2, motorBPwm;

};


#endif ////__MOTORCONTROLLER_H__
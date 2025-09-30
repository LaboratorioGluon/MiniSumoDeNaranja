#ifndef __INFORMER_H__
#define __INFORMER_H__

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>

class Informer
{
public:

    enum STATUS
    {
        STARTING = 0 ,      
        WAITING_PUSH_BUTTON,
        WAITING_5S,
        RUNNING,
        FAILED,             
        TARGET_ADQUIRED
    };

    Informer(gpio_num_t LedR_, gpio_num_t LedG_ );

    void Init();
    void Update();

    void setStatus(enum STATUS st);



private:
    gpio_num_t LedR, LedG;
    int64_t LedRPeriod, LedGPeriod;
    uint64_t remainingR;
    uint8_t statusR;
    uint64_t remainingG;
    uint8_t statusG;

    TaskHandle_t informerTask;
};



#endif //__INFORMER_H__
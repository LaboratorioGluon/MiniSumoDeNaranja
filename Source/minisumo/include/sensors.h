#ifndef __SENSORS_H__
#define __SENSORS_H__

#define NUM_TOF_SENSORS 3
#define NUM_LINE_SENSORS 2

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "i2cExpander.h"
#include "labVl53l0x.h"

class Sensors{
public:
    Sensors();

    void Init();

    void getTof(uint16_t _tofData[NUM_TOF_SENSORS]);
    void getLine(uint16_t _lineData[NUM_LINE_SENSORS]);


    void updadeTof();
    void updateLine();

private:
    SemaphoreHandle_t semaphore_;
    labVL53L0X rangeSensor;
    
    i2cExpander tca;

    /* Critical sections */
    uint16_t tofData[NUM_TOF_SENSORS];
    uint16_t lineData[NUM_LINE_SENSORS];

};

#endif //__SENSORS_H__
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
    void getLine(uint32_t _lineData[NUM_LINE_SENSORS]);


    void updadeTof();
    void updateLine();

private:
    SemaphoreHandle_t semaphore_;
    labVL53L0X rangeSensor;
    
    i2cExpander tca;

    uint32_t tofRateValue[NUM_TOF_SENSORS] = {70,70,80};
    uint32_t tofThresholdValue[NUM_TOF_SENSORS] = {95,95,98};

    uint32_t lastTofValues[NUM_TOF_SENSORS] = {0,0,0};

    /* Critical sections */
    uint16_t tofData[NUM_TOF_SENSORS];
    uint32_t lineData[NUM_LINE_SENSORS];

};

#endif //__SENSORS_H__
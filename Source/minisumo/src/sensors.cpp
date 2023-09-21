#include "sensors.h"
#include <string.h>
#include <esp_log.h>

static char TAG[] = "SENSORS";
SemaphoreHandle_t semaphore;

Sensors::Sensors(): tca(GPIO_NUM_21, GPIO_NUM_22)
{
    
}

void Sensors::Init()
{
    semaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(semaphore);

    tca.InitI2cMaster();


    for( uint8_t i = 0;i < NUM_TOF_SENSORS; i++)
    {
        tca.selectPort(i);
        if (!rangeSensor.Init(VL53L0X_DEVICEMODE_CONTINUOUS_RANGING))
        {
            ESP_LOGE(TAG, "Failed to initialize VL53L0X :( %d", i);
        }
    }


}

void Sensors::getTof(uint16_t _tofData[NUM_TOF_SENSORS])
{
    if(xSemaphoreTake(semaphore, portMAX_DELAY) == pdTRUE)
    {   
        memcpy(_tofData, tofData, NUM_TOF_SENSORS*sizeof(tofData[0]));
    }
    xSemaphoreGive(semaphore);
}

void Sensors::getLine(uint16_t _lineData[NUM_LINE_SENSORS])
{
    if(xSemaphoreTake(semaphore, portMAX_DELAY) == pdTRUE)
    {
        memcpy(_lineData, lineData, NUM_LINE_SENSORS*sizeof(lineData[0]));
    }
    xSemaphoreGive(semaphore);
}

void Sensors::updadeTof()
{
    uint16_t rangeMeasurement[NUM_TOF_SENSORS];
    if (xSemaphoreTake(semaphore, portMAX_DELAY) == pdTRUE)
    {
        for (uint8_t i = 0; i < NUM_TOF_SENSORS; i++)
        {
            tca.selectPort(i);
            rangeSensor.readContiniousLastData(&rangeMeasurement[i]);
            //ESP_LOGE(TAG, "Sensor %u (%d) : %lu", i, ret, rangeMeasurement[i]);
        }

        memcpy(tofData, rangeMeasurement, NUM_TOF_SENSORS*sizeof(tofData[0]));
    }
    xSemaphoreGive(semaphore);
}

void Sensors::updateLine()
{
    if(xSemaphoreTake(semaphore, portMAX_DELAY) == pdTRUE)
    {

    }
    xSemaphoreGive(semaphore);
}
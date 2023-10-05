#include "sensors.h"
#include <string.h>
#include <esp_log.h>
#include <esp_err.h>
#include <driver/adc.h>

static char TAG[] = "SENSORS";
SemaphoreHandle_t semaphore;

#define ALPHA 0.9f

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

        rangeSensor.setThresholds(tofRateValue[i], tofThresholdValue[i]);
    }

    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_11);

}

void Sensors::getTof(uint16_t _tofData[NUM_TOF_SENSORS])
{
    if(xSemaphoreTake(semaphore, portMAX_DELAY) == pdTRUE)
    {   
        memcpy(_tofData, tofData, NUM_TOF_SENSORS*sizeof(tofData[0]));
    }
    xSemaphoreGive(semaphore);
}

void Sensors::getLine(uint32_t _lineData[NUM_LINE_SENSORS])
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
            //ESP_LOGE(TAG, "Sensor %u (%d) : %lu", i,  rangeMeasurement[i]);
            if ( rangeMeasurement[i] > 8000 )
            {
                rangeMeasurement[i] = 0;
            }
            rangeMeasurement[i] = rangeMeasurement[i]*ALPHA + tofData[i]*(1-ALPHA);
        }

        memcpy(tofData, rangeMeasurement, NUM_TOF_SENSORS*sizeof(tofData[0]));
    }
    xSemaphoreGive(semaphore);
}

void Sensors::updateLine()
{
    uint32_t newLineData[NUM_LINE_SENSORS];
    if(xSemaphoreTake(semaphore, portMAX_DELAY) == pdTRUE)
    {
        // Get configuration
        newLineData[0] = 0;
        newLineData[1] = 0;
        for(uint8_t i =0; i < 5; i++){
            newLineData[0] += adc1_get_raw(ADC1_CHANNEL_0);
            newLineData[1] += adc1_get_raw(ADC1_CHANNEL_3);
        }
        newLineData[0] /= 5;
        newLineData[1] /= 5;

        lineData[0] = newLineData[0];
        lineData[1] = newLineData[1];

        /*ESP_LOGE(TAG, "Line 0 : %lu", lineData[0]);
        ESP_LOGE(TAG, "Line 1 : %lu", lineData[1]);*/

    }
    xSemaphoreGive(semaphore);
}

#include "configLoader.h"
#include <esp_log.h>

#include "driver/adc.h"

const char TAG[]= "CONFIG";

int8_t loadConfig()
{
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);
    
    // Get configuration
    uint32_t adc_raw=0;
    for(uint8_t i =0; i < 10; i++){
        adc_raw += adc1_get_raw(ADC1_CHANNEL_6);
    }
    adc_raw /= 10;
    ESP_LOGE(TAG, "Raw value: %lu", adc_raw);

    int8_t configuration = -1;

    if(adc_raw < 200) // 0000
    {
        configuration = 0;
    }
    else if(adc_raw < 918) // 0001
    {
        configuration = 1;
    }
    else if ( adc_raw < 1381) // 0010
    {
        configuration = 2;
    }
    else if ( adc_raw < 1720) // 0011
    {
        configuration = 3;
    }
    else if (adc_raw < 1981) // 0100
    {
        // Intentionally empty
    }
    else if (adc_raw < 2194) // 0101
    {
        // Intentionally empty    
    }
    else if( adc_raw < 2362) // 0110
    {
        // Intentionally empty
    }
    else if ( adc_raw < 3270) // 0111
    {
        // Intentionally empty
    }
    else //1XXX
    {
        // Intentionally empty
    }

    return configuration;
}
#include "informer.h"
#include "hwInterface.h"

#include <esp_log.h>
#include <esp_timer.h>

// Static thread function
static void informerThread(void *arg);

static char TAG[]= "INFORMER";

Informer::Informer(gpio_num_t LedR_, gpio_num_t LedG_) : LedR(LedR_), LedG(LedG_)
{
    LedRPeriod = -1;
    LedGPeriod = -1;
    remainingG = 0;
    remainingR = 0;
}

void Informer::Init()
{

    gpio_config_t gpioConfig;
    gpioConfig.intr_type = GPIO_INTR_DISABLE;
    gpioConfig.mode = GPIO_MODE_OUTPUT;
    gpioConfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpioConfig.pull_up_en = GPIO_PULLUP_DISABLE;
    gpioConfig.pin_bit_mask = (1ULL << LedR) | (1ULL << LedG);

    gpio_config(&gpioConfig);
    gpio_set_level(LedR, 0);
    gpio_set_level(LedG, 0);

    xTaskCreate(informerThread, "Informer_thread", 1024, this, 1, &informerTask);
}

void Informer::setStatus(enum STATUS st)
{
    switch (st)
    {
    case STARTING:
        LedGPeriod = 1000000;
        break;
    case WAITING_PUSH_BUTTON:
        LedGPeriod = 500000;
        break;
    case WAITING_5S:
        LedGPeriod = 100000;
        break;
    case RUNNING:
        LedRPeriod = -1;
        LedGPeriod = 0;
        break;
    /*case FAILED:
        LedRPeriod = 0;
        break;*/
    case TARGET_ADQUIRED:
        LedRPeriod = 0;
        LedGPeriod = -1;
        break;
    default:
        break;
    }
}


void Informer::Update()
{
    static uint64_t lastCall = esp_timer_get_time();
    uint64_t dt = esp_timer_get_time() - lastCall;
    
    // Control of Red LED
    if (LedRPeriod == -1)
    {
        gpio_set_level(LedR, 0);
    }
    else if(LedRPeriod == 0)
    {
        gpio_set_level(LedR, 1);
    }
    else
    {

        if (dt > remainingR)
        {
            remainingR = 0;
        }
        else
        {
            remainingR -= dt;
        }

        if (remainingR == 0)
        {
            remainingR += LedRPeriod;
            statusR = !statusR;
            gpio_set_level(LedR, statusR);
        }
    }

    // Control of Green LED
    if (LedGPeriod == -1)
    {
        gpio_set_level(LedG, 0);
    }
    else if(LedGPeriod == 0)
    {
        gpio_set_level(LedG, 1);
    }
    else
    {
        if (dt > remainingG)
        {
            remainingG = 0;
        }
        else
        {
            remainingG -= dt;
        }

        if (remainingG == 0)
        {
            remainingG += LedGPeriod;
            statusG = !statusG;
            gpio_set_level(LedG, statusG);
        }
    }

    lastCall = esp_timer_get_time();
}



void informerThread(void *arg)
{
    Informer *pInfo = (Informer*) arg;
    while(1)
    {
        pInfo->Update();
        vTaskDelay(pdMS_TO_TICKS(50));
    }

}
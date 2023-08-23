
#include <freertos/FreeRTOS.h>
#include <freertos/timers.h>
#include <esp_timer.h>
#include <esp_http_client.h>
#include <nvs_flash.h>
#include <esp_https_ota.h>
#include <esp_log.h>
//#include <driver/adc.h>
#include <esp_adc/adc_continuous.h>
#include <stdint.h>

#include "motorController.h"
#include "vl53l0x.h"
#include "labVl53l0x.h"
#include "commander.h"
#include "i2cExpander.h"
#include "sensors.h"
#include "configLoader.h"

#include "secret.h"

static char TAG[] = "MAIN";

#define ENABLE_OTA

#ifdef ENABLE_OTA
    extern "C" void wifi_init_sta();
#endif

#ifndef WIFI_PWD
#error Missing WIFI_PWD Environment variable
#endif

#ifndef WIFI_SSID
#error Missing WIFI_SSID Environment variable
#endif

extern "C" void app_main();

#define MOTOR_PIN_1 GPIO_NUM_5

uint16_t rangeMeasurement = 0;

MotorController motors(GPIO_NUM_5, GPIO_NUM_18, GPIO_NUM_19, GPIO_NUM_21);
labVL53L0X rangeSensor;
Sensors sensors;
Commander commander;
uint16_t tofSensorData[NUM_TOF_SENSORS];

uint8_t isrTriggered = 0;

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    isrTriggered= 1;
}

enum STARTUP_CONFIG{
    START_INVALID_CONFIG = -1,
    START_ENEMY_FWD = 0,
    START_ENEMY_RIGHT,
    START_ENEMY_LEFT,
    START_ENEMY_BACK,
} startupConfig;

void foo(){

    gpio_config_t io_conf = {};

    // Configure pins for MotorA
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1 << GPIO_NUM_23);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    // Configure pins for MotorA
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1 << GPIO_NUM_2);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

}

// Micro FSM Moore
enum STATES{
    SEARCHING = 0,
    ATTACKING,
    EVADING
};

enum STATES currentState = SEARCHING;
int64_t currentStateStart = 0;

void Init()
{

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    foo();

    motors.Init();
    sensors.Init();

    startupConfig = (STARTUP_CONFIG)loadConfig();

    if (startupConfig == START_INVALID_CONFIG)
    {
        // Todo: Inform with LED.
        ESP_LOGE(TAG, "Error invalid config!");
    }
    else
    {
        ESP_LOGE(TAG, "Current config: %d", startupConfig);
    }

    gpio_config_t io_conf = {};

    // Configure pins for MotorA
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << GPIO_NUM_35);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(GPIO_NUM_35, gpio_isr_handler, nullptr);

    while(true){
        if(isrTriggered == 1)
        {
            ESP_LOGE(TAG,"Triggered!");
            isrTriggered = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    /*rangeSensor.i2cMasterInit(GPIO_NUM_4,GPIO_NUM_22);
    if (!rangeSensor.Init(VL53L0X_DEVICEMODE_CONTINUOUS_RANGING))
    {
        ESP_LOGE(TAG, "Failed to initialize VL53L0X :(");
    }*/
}

void Wait()
{
#ifdef ENABLE_OTA
    wifi_init_sta();
#else
    vTaskDelay(pdMS_TO_TICKS(5000));
#endif

}


void loopSearching();
void loopAttack();
void loopEvading();

typedef void (*func_t)();
func_t funcs[] = {
    loopSearching,
    loopAttack,
    loopEvading
};


void changeState(enum STATES newState)
{
    currentState=newState; 
    currentStateStart = esp_timer_get_time();
    funcs[currentState]();
}

void loopEvading()
{
    if(esp_timer_get_time() > currentStateStart + 1000000)
    {
        changeState(SEARCHING);
    }
}


void loopAttack()
{
    ESP_LOGE(TAG," Loop Attack...");
    motors.setDirection(MotorController::FWD);
    rangeMeasurement = tofSensorData[0];
    if (rangeMeasurement > 200)
    {
        changeState(SEARCHING);
    }
}


void loopSearching()
{   
    ESP_LOGE(TAG," Loop Searching...");

    motors.setDirection(MotorController::RIGHT);
    rangeMeasurement = tofSensorData[0];
    if (rangeMeasurement < 200)
    {
        changeState(ATTACKING);
    }

    /*if(esp_timer_get_time() > currentStateStart + 5000000)
    {
        changeState(ATTACKING);
    }*/
}

enum CMD{
    NOTHING = 0,
    FWD,
    BACK,
    RIGHT,
    LEFT,
    CONTINUE, // sigue el proceso normal
    LAUNCH_MISSILE,
    END_CIVILIZATION_WITH_NUKE_RED_BUTTON_ADMIN_123,
    UPDATE_FW
};



void updateOTA()
{
    ESP_LOGI(TAG, "Starting OTA example task");
    esp_http_client_config_t config = {
        .url = "http://192.168.1.133:8000/firmware.bin?", // Cambialo por la URL donde estará tu binario compilado
        .keep_alive_enable = true,
    };

    esp_https_ota_config_t ota_config = {
        .http_config = &config,
    };
    ESP_LOGI(TAG, "Attempting to download update from %s", config.url);
    esp_err_t ret = esp_https_ota(&ota_config);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "OTA Succeed, Rebooting...");
        esp_restart();
    } else {
        ESP_LOGE(TAG, "Firmware upgrade failed");
    }
    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}


uint8_t commandRunning; // If 0 : Do normal execution, otherwise, dont do the main loop
void doCommand()
{

    char lastCommand[50];
    commander.updateCommand();
    commander.getLastCommand(lastCommand, 50);

    if( !strcmp(lastCommand, "UPDATE_FW"))
    {
        ESP_LOGE(TAG,"NEW VERSION!!!");
        commandRunning = 1;
        motors.setDirection(MotorController::STOP);
        updateOTA();
    }
    else if( !strcmp(lastCommand, "FWD"))
    {
        commandRunning = 1;
        ESP_LOGE(TAG,"Setting Direction: FWD");
        motors.setDirection(MotorController::FWD);
    }
    else if( !strcmp(lastCommand, "BACK"))
    {
        commandRunning = 1;
        ESP_LOGE(TAG,"Setting Direction: BACK");
        motors.setDirection(MotorController::BACK);
    }
    else if( !strcmp(lastCommand, "RIGHT"))
    {
        commandRunning = 1;
        ESP_LOGE(TAG,"Setting Direction: RIGHT");
        motors.setDirection(MotorController::RIGHT);
    }
    else if( !strcmp(lastCommand, "LEFT"))
    {
        commandRunning = 1;
        ESP_LOGE(TAG,"Setting Direction: LEFT");
        motors.setDirection(MotorController::LEFT);
    }
    else if( !strcmp(lastCommand, "CONTINUE"))
    {
        commandRunning = 0;
        ESP_LOGE(TAG,"Command continue");
        motors.setDirection(MotorController::STOP);
    }
    
}
// Fin Micro FSM

void coreAThread(void *arg)
{
    ESP_LOGE(TAG, "Iniciando CORE A");
    uint32_t counter = 0;

    while(true)
    {
        sensors.getTof(tofSensorData);
        //ESP_LOGE(TAG,"Sensor: %lu", tofSensorData[0]);

        if(!commandRunning)
        {
            // Check dohyo lines
            // if true: currentState = Evading

            // loop of current state
            funcs[currentState]();
        }

        #ifdef ENABLE_OTA
        // Recibir comandos y autoactualizarnos
        if(counter%10 == 0)
        {
            doCommand();
        }
        counter++;
        #endif
        
        vTaskDelay(pdMS_TO_TICKS(200));
        //taskYIELD();
    }
}

void coreBThread(void *arg)
{
    ESP_LOGE(TAG, "Iniciando CORE B");
    while(true){
        sensors.updadeTof();
        //sensors.updateLine();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

TaskHandle_t mainCoreHandle;
TaskHandle_t sensorCoreHandle;

void app_main() 
{
    ESP_LOGE(TAG, "Iniciando software");
    Init();
    Wait();

    xTaskCreatePinnedToCore(coreAThread, "Main_core",   4096, NULL, 10, &mainCoreHandle, 0);
    xTaskCreatePinnedToCore(coreBThread, "Sensor_Core", 4096, NULL, 10, &sensorCoreHandle, 1);

}
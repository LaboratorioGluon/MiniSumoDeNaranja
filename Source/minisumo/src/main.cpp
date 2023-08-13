
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "esp_timer.h"
#include "esp_http_client.h"
#include "nvs_flash.h"
#include "esp_https_ota.h"

#include <stdint.h>

#include "motorController.h"
#include "vl53l0x.h"
#include "labVl53l0x.h"
#include "esp_log.h"

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

    /*gpio_set_level(GPIO_NUM_2, 0);
    vTaskDelay(pdMS_TO_TICKS(500));
    gpio_set_level(GPIO_NUM_2, 1);
    vTaskDelay(pdMS_TO_TICKS(500));
    gpio_set_level(GPIO_NUM_2, 0);
    vTaskDelay(pdMS_TO_TICKS(500));
    gpio_set_level(GPIO_NUM_2, 1);
    vTaskDelay(pdMS_TO_TICKS(500));
    gpio_set_level(GPIO_NUM_2, 0);
    vTaskDelay(pdMS_TO_TICKS(500));
    gpio_set_level(GPIO_NUM_2, 1);*/

    /*while(true)
    {
        gpio_set_level(GPIO_NUM_2, gpio_get_level(GPIO_NUM_23));
        vTaskDelay(pdMS_TO_TICKS(50));
    }*/
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

    rangeSensor.i2cMasterInit(GPIO_NUM_4,GPIO_NUM_22);
    if (!rangeSensor.Init(VL53L0X_DEVICEMODE_CONTINUOUS_RANGING))
    {
        ESP_LOGE(TAG, "Failed to initialize VL53L0X :(");
    }
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
    //ESP_LOGE(TAG," Loop Evading...");
    if(esp_timer_get_time() > currentStateStart + 1000000)
    {
        changeState(SEARCHING);
    }
}


void loopAttack()
{
    ESP_LOGE(TAG," Loop Attack...");
    motors.setDirection(MotorController::FWD);
    bool ret = rangeSensor.readContiniousLastData(&rangeMeasurement);
    ESP_LOGE(TAG, "Result(%d), %d", ret, rangeMeasurement);
    if (rangeMeasurement > 200)
    {
        changeState(SEARCHING);
    }

    /*if(esp_timer_get_time() > currentStateStart + 3000000)
    {
        changeState(EVADING);
    }*/

}


void loopSearching()
{   
    ESP_LOGE(TAG," Loop Searching...");

    motors.setDirection(MotorController::RIGHT);
    //uint64_t start = esp_timer_get_time();
    bool ret = rangeSensor.readContiniousLastData(&rangeMeasurement);
    ESP_LOGE(TAG, "Result(%d), %d", ret, rangeMeasurement);

    //ESP_LOGE(TAG, "Time: %" PRId64 "\n", esp_timer_get_time()-start);
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

char lastCommand[50] = {0};
uint8_t commandRunning=0; // If 0 : Do normal execution, otherwise, dont do the main loop
void loopCommands()
{
    memset(lastCommand, 0, 50);

    esp_http_client_config_t config = {
        .url = "http://192.168.1.133:8000/commands.html",
        .user_data = lastCommand,        // Pass address of local buffer to get response
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_http_client_open(client,0);
    esp_http_client_fetch_headers(client);
    esp_http_client_read(client, lastCommand, esp_http_client_get_content_length(client));

    ESP_LOGE(TAG,"Received html: %s", lastCommand);

    esp_http_client_close(client);
    esp_http_client_cleanup(client);
}



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

void doCommand()
{
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

void app_main() 
{
    ESP_LOGE(TAG, "Iniciando software");
    Init();

    Wait();

    MotorController::DIRECTION dir = MotorController::DIRECTION::STOP;

    uint32_t counter = 0;
    uint64_t start = esp_timer_get_time();
    while( true )
    {
        
        if(!commandRunning)
        {
            // Check dohyo lines
            // if true: currentState = Evading

            // loop of current state
            funcs[currentState]();
        }
        #ifdef ENABLE_OTA
        
        // Recibir comandos y autoactualizarnos

        #endif

        // Todo: Remove in release
        if(counter%100 == 0)
        {
            loopCommands();
            doCommand();
        }
        counter++;


        vTaskDelay(pdMS_TO_TICKS(20));

    }

#if 0
    while(true)
    {
        if(gpio_get_level(GPIO_NUM_23))
        {
            dir = MotorController::STOP;
            gpio_set_level(GPIO_NUM_2, 0);
        }
        else
        {
            gpio_set_level(GPIO_NUM_2, 1);
            dir = MotorController::FWD;
        }
        /*if(gpio_get_level(GPIO_NUM_23))
        {
            motors.setDirection(MotorController::BACK);
            vTaskDelay(pdMS_TO_TICKS(500));
            motors.setDirection(MotorController::RIGHT);
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        else
            dir = MotorController::FWD;*/
        
        motors.setDirection(dir);
        /*switch(dir)
        {
        case MotorController::STOP: dir = MotorController::FWD; break;
        case MotorController::FWD: dir = MotorController::RIGHT; break;
        case MotorController::RIGHT: dir = MotorController::LEFT; break;
        case MotorController::LEFT: dir = MotorController::BACK; break;
        case MotorController::BACK: dir = MotorController::STOP; break;
        }*/
        rangeSensor.readSingleWithPolling(&rangeMeasurement);
        ESP_LOGE(TAG, "Range sensor: %d", rangeMeasurement);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
#endif 
}




#if 0

void loop()
{
    // Robot es tonto. Es un ente controlado por main.

    sensor X = robot.leerSensores(); // Guarda el sensor que detecta al enemigo más cerca. 
    //-1 : Si no hay ninguno

    if (SEnsores de linea se han activado)
    {
        direction = EVADE;
    }
    else
    {
        switch(X)
        {
            case F: direction = FWD;
            case R: direction = RIGHT;
            case L: direction = LEFT;
        }
    }

    
    robot.move(direction);

    //robot.weapon(Weapon);
}

#endif
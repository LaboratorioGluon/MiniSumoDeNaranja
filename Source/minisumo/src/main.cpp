
#include <freertos/FreeRTOS.h>
#include <freertos/timers.h>
#include <esp_timer.h>
#include <esp_http_client.h>
#include <nvs_flash.h>
#include <esp_https_ota.h>
#include <esp_log.h>
// #include <driver/adc.h>
#include <esp_adc/adc_continuous.h>
#include <driver/adc.h>
#include <stdint.h>

#include "hwInterface.h"
#include "motorController.h"
#include "vl53l0x.h"
#include "labVl53l0x.h"
#include "commander.h"
#include "i2cExpander.h"
#include "sensors.h"
#include "configLoader.h"
#include "informer.h"

#include "secret.h"

static char TAG[] = "MAIN";

// #define ENABLE_OTA

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
#define MIN_TOF_VALUE 20

uint16_t rangeMeasurement = 0;

MotorController motors(PIN_MOTOR_A_IN1, PIN_MOTOR_A_IN2, PIN_MOTOR_A_PWM, PIN_MOTOR_B_IN1, PIN_MOTOR_B_IN2, PIN_MOTOR_B_PWM);
labVL53L0X rangeSensor;
Sensors sensors;
Commander commander;
Informer informer(PIN_LED_1, PIN_LED_2);
uint16_t tofSensorData[NUM_TOF_SENSORS];
uint32_t lineSensorData[NUM_LINE_SENSORS];
uint32_t lineStartValue[NUM_LINE_SENSORS];

uint64_t lastmicros;
uint8_t isRotating = 0;
int64_t remainingRotation = 0;
uint32_t isEnemyDown = 0;

uint8_t isrTriggered = 0;

static TaskHandle_t initTask = NULL;
static TaskHandle_t mainCoreHandle = NULL;
static TaskHandle_t sensorCoreHandle = NULL;

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    isrTriggered = 1;
    if (initTask != NULL)
    {
        vTaskNotifyGiveFromISR(initTask, NULL);
    }
}

enum STARTUP_CONFIG
{
    START_INVALID_CONFIG = -1,
    START_ENEMY_FWD = 0, // 0000
    START_ENEMY_RIGHT,   // 0001
    START_ENEMY_LEFT,    // 0010
    START_ENEMY_BACK,    // 0011
} startupConfig;

// Micro FSM Moore
enum STATES
{
    SEARCHING = 0,
    ATTACKING,
    EVADING
};

void EnableWifi()
{
#ifdef ENABLE_OTA
    wifi_init_sta();
#endif
}

enum STATES currentState = SEARCHING;
int64_t currentStateStart = 0;

void startRotating(uint64_t rotatingTime)
{
    lastmicros = esp_timer_get_time();
    remainingRotation = rotatingTime;
    isRotating = 1;
}

void Init()
{
    initTask = xTaskGetCurrentTaskHandle();

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    informer.Init();
    informer.setStatus(Informer::STARTING);

    motors.Init();
    sensors.Init();

    EnableWifi();

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
    gpio_set_level(GPIO_NUM_35, 1);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(GPIO_NUM_35, gpio_isr_handler, nullptr);

    informer.setStatus(Informer::WAITING_PUSH_BUTTON);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    ESP_LOGE(TAG, "Triggered!!");
    informer.setStatus(Informer::WAITING_5S);
    // Wait 5 seconds mandatory by the rules.
    vTaskDelay(pdMS_TO_TICKS(5000));
    informer.setStatus(Informer::RUNNING);
    ESP_LOGE(TAG, "Robot encendido");

    // Store line value
    sensors.updateLine();
    sensors.getLine(lineSensorData);
    lineStartValue[0] = lineSensorData[0];
    lineStartValue[1] = lineSensorData[1];

    ESP_LOGE(TAG, "Valor linea inicio: %lu, %lu", lineStartValue[0], lineStartValue[1]);

    switch (startupConfig)
    {
    case START_INVALID_CONFIG:
        break;
    case START_ENEMY_FWD:
        break;
    case START_ENEMY_RIGHT:
        motors.setDirection(MotorController::RIGHT);
        startRotating(300000);
        break;
    case START_ENEMY_LEFT:
        motors.setDirection(MotorController::LEFT);
        startRotating(300000);
        break;
    case START_ENEMY_BACK:
        motors.setDirection(MotorController::RIGHT);
        startRotating(600000);
        break;
    }
}

inline uint8_t isTofActivated(uint8_t index)
{
    return ((tofSensorData[index] > 30) && (tofSensorData[index] < 650));
}

inline uint8_t isLineActivated(uint8_t index)
{
    return lineSensorData[index] < lineStartValue[index] * 0.5;
}

void loopSearching();
void loopAttack();
void loopEvading();

typedef void (*func_t)();
func_t funcs[] = {
    loopSearching,
    loopAttack,
    loopEvading};

uint8_t prevLineActivated[NUM_LINE_SENSORS] = {0, 0};

void changeState(enum STATES newState)
{
    currentState = newState;
    currentStateStart = esp_timer_get_time();

    funcs[currentState]();
}

void loopEvading()
{
    if (esp_timer_get_time() > currentStateStart + 1000000)
    {
        changeState(SEARCHING);
    }
}



void attackCheckLine()
{
    if (isLineActivated(0))
    {
        motors.setDirection(MotorController::BACK);
        vTaskDelay(pdMS_TO_TICKS(100));
        //ESP_LOGE(TAG, "ATK LINE SENSOR 0 ACTIVATED");
        motors.setDirection(MotorController::LEFT);
        startRotating(300000);
    }
    else if (isLineActivated(1))
    {
        motors.setDirection(MotorController::BACK);
        vTaskDelay(pdMS_TO_TICKS(100));
        //ESP_LOGE(TAG, "ATK LINE SENSOR 1 ACTIVATED");
        motors.setDirection(MotorController::RIGHT);
        startRotating(300000);
    }
}

void loopAttack()
{

    informer.setStatus(Informer::TARGET_ADQUIRED);

    if (isTofActivated(0))
    {
        motors.setDirection(MotorController::FWD);
        //ESP_LOGE(TAG, "ATTACK: NORMAL FWD");
        attackCheckLine();

        /*if (prevLineActivated[0] == 0 && prevLineActivated[1] == 0)
        {
            // This condition marks the start of the fuck off strategy
            if (isLineActivated(0)) // Right sensor activated for the first time
            {
                ESP_LOGE(TAG, "ATTACK: Activado 0");
                prevLineActivated[0] = 1;
                motors.setDirection(MotorController::BACK);
                vTaskDelay(pdMS_TO_TICKS(50));
            }
            else if (isLineActivated(1)) // Left sensor activated for the first time
            {
                ESP_LOGE(TAG, "ATTACK: Activado 1");
                prevLineActivated[1] = 1;
                motors.setDirection(MotorController::BACK);
                vTaskDelay(pdMS_TO_TICKS(50));
            }
        }
        else
        {
            if (prevLineActivated[0])
            {
                motors.setDirection(MotorController::RIGHT);
                if (isLineActivated(1))
                {
                    ESP_LOGE(TAG, "ATTACK: Fin Activado 1");
                    prevLineActivated[0] = 0;
                    prevLineActivated[1] = 0;
                    motors.setDirection(MotorController::BACK);
                    vTaskDelay(pdMS_TO_TICKS(300));
                    startRotating(300000);
                }
            }
            else
            {
                motors.setDirection(MotorController::LEFT);
                if (isLineActivated(0))
                {
                    ESP_LOGE(TAG, "ATTACK: Fin Activado 0");
                    prevLineActivated[0] = 0;
                    prevLineActivated[1] = 0;
                    motors.setDirection(MotorController::BACK);
                    vTaskDelay(pdMS_TO_TICKS(300));
                    startRotating(300000);
                }
            }
        }*/
    }
    else if (isTofActivated(1))
    {
        //ESP_LOGE(TAG, "ATTACK: NORMAL LEFT");
        motors.setDirection(MotorController::LEFT);
        attackCheckLine();
    }
    else if (isTofActivated(2))
    {
        //ESP_LOGE(TAG, "ATTACK: NORMAL RIGHT");
        motors.setDirection(MotorController::RIGHT);
        attackCheckLine();
    }

    if (
        !isTofActivated(0) &&
        !isTofActivated(1) &&
        !isTofActivated(2))
    {
        /*ESP_LOGE(TAG, "Switching to SEARCH MODE");
        ESP_LOGE(TAG, "%d, %d, %d", isTofActivated(0), isTofActivated(1), isTofActivated(2));
        ESP_LOGE(TAG, "Sensor: %lu, %lu, %lu", tofSensorData[0], tofSensorData[1], tofSensorData[2]);*/
        changeState(SEARCHING);
    }
}

void loopRotating()
{
    if (remainingRotation <= 0)
    {
        // Volvemos a ir recto
        ESP_LOGE(TAG, "End rotating");
        isRotating = 0;
        isEnemyDown = 0;
    }
    remainingRotation -= esp_timer_get_time() - lastmicros;
    lastmicros = esp_timer_get_time();
}

void loopSearching()
{
    informer.setStatus(Informer::RUNNING);
    if (isEnemyDown == 0)
    {
        if (isTofActivated(0) ||
            isTofActivated(1) ||
            isTofActivated(2))
        {
            /*ESP_LOGE(TAG, "Switching to ATTACK MODE");
            ESP_LOGE(TAG, "%d, %d, %d", isTofActivated(0), isTofActivated(1), isTofActivated(2));
            ESP_LOGE(TAG, "Sensor: %lu, %lu, %lu", tofSensorData[0], tofSensorData[1], tofSensorData[2]);*/
            changeState(ATTACKING);
        }
    }

    if (!isRotating)
    {
        motors.setDirection(MotorController::FWD);
        if (isLineActivated(0))
        {
            motors.setDirection(MotorController::BACK);
            vTaskDelay(pdMS_TO_TICKS(100));
            ESP_LOGE(TAG, "LINE SENSOR 0 ACTIVATED");
            motors.setDirection(MotorController::LEFT);
            startRotating(300000);
        }
        else if (isLineActivated(1))
        {
            motors.setDirection(MotorController::BACK);
            vTaskDelay(pdMS_TO_TICKS(100));
            ESP_LOGE(TAG, "LINE SENSOR 1 ACTIVATED");
            motors.setDirection(MotorController::RIGHT);
            startRotating(300000);
        }
    }
    else
    {
        loopRotating();
    }
}

enum CMD
{
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
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "OTA Succeed, Rebooting...");
        esp_restart();
    }
    else
    {
        ESP_LOGE(TAG, "Firmware upgrade failed");
    }
    while (1)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

uint8_t commandRunning; // If 0 : Do normal execution, otherwise, dont do the main loop
void doCommand()
{

    char lastCommand[50];
    commander.updateCommand();
    commander.getLastCommand(lastCommand, 50);

    if (!strcmp(lastCommand, "UPDATE_FW"))
    {
        ESP_LOGE(TAG, "NEW VERSION!!!");
        commandRunning = 1;
        motors.setDirection(MotorController::STOP);
        updateOTA();
    }
    else if (!strcmp(lastCommand, "FWD"))
    {
        commandRunning = 1;
        ESP_LOGE(TAG, "Setting Direction: FWD");
        motors.setDirection(MotorController::FWD);
    }
    else if (!strcmp(lastCommand, "BACK"))
    {
        commandRunning = 1;
        ESP_LOGE(TAG, "Setting Direction: BACK");
        motors.setDirection(MotorController::BACK);
    }
    else if (!strcmp(lastCommand, "RIGHT"))
    {
        commandRunning = 1;
        ESP_LOGE(TAG, "Setting Direction: RIGHT");
        motors.setDirection(MotorController::RIGHT);
    }
    else if (!strcmp(lastCommand, "LEFT"))
    {
        commandRunning = 1;
        ESP_LOGE(TAG, "Setting Direction: LEFT");
        motors.setDirection(MotorController::LEFT);
    }
    else if (!strcmp(lastCommand, "CONTINUE"))
    {
        commandRunning = 0;
        ESP_LOGE(TAG, "Command continue");
        motors.setDirection(MotorController::STOP);
    }
}
// Fin Micro FSM

void coreAThread(void *arg)
{
    static uint64_t lastmicros = esp_timer_get_time();
    ESP_LOGE(TAG, "Iniciando CORE A");

    while (true)
    {
        sensors.getTof(tofSensorData);
        sensors.getLine(lineSensorData);

        // ESP_LOGE(TAG,"Sensor: %lu, %lu, %lu", tofSensorData[0], tofSensorData[1], tofSensorData[2]);
        //ESP_LOGE(TAG, "Lineas :%lu, %lu", lineSensorData[0], lineSensorData[1]);

        if (!commandRunning)
        {
            // loop of current state
            funcs[currentState]();
        }

#ifdef ENABLE_OTA
        // Recibir comandos y autoactualizarnos
        if (esp_timer_get_time() > lastmicros + 500000)
        {
            doCommand();
            lastmicros = esp_timer_get_time();
        }
        counter++;
#endif

        taskYIELD();
    }
}

void coreBThread(void *arg)
{
    ESP_LOGE(TAG, "Iniciando CORE B");
    while (true)
    {
        sensors.updadeTof();
        sensors.updateLine();
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

void app_main()
{
    ESP_LOGE(TAG, "Iniciando software");

    Init();

    xTaskCreatePinnedToCore(coreBThread, "Sensor_Core", 4096, NULL, 9, &sensorCoreHandle, 1);
    xTaskCreatePinnedToCore(coreAThread, "Main_core", 4096, NULL, 10, &mainCoreHandle, 0);
    
    
}
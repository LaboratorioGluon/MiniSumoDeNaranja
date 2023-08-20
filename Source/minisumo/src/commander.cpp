
#include "commander.h"

#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "esp_timer.h"
#include "esp_http_client.h"
#include <esp_log.h>
#include <stdint.h>

const char TAG[] = "COMMS";

void Commander::updateCommand()
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


esp_err_t Commander::getLastCommand(char *cmd, uint8_t max_len)
{
    esp_err_t ret = ESP_FAIL;

    if(cmd == nullptr)
    {
        ret = ESP_ERR_INVALID_ARG;
    }
    else if(strlen(lastCommand) > max_len)
    {
        ret =  ESP_ERR_INVALID_SIZE;
    }
    else
    {
        strcpy(cmd, lastCommand);
        ret = ESP_OK;
    }

    return ret;
}
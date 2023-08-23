#ifndef __COMMANDER_H__
#define __COMMANDER_H__

#define MAX_COMMAND_LEN 50
#include "freertos/FreeRTOS.h"
#include <esp_err.h>
#include <string.h>

class Commander{
public:
    Commander(){
        memset(lastCommand,0, MAX_COMMAND_LEN);
    };

    /**
     * @brief Read the command from the HTTP server
    */
    void updateCommand();

    /** @brief Get the last received command. 'updateCommand' shall be called
     *         before getting the command.
    */
    esp_err_t getLastCommand(char *cmd, uint8_t max_len);

private:
    char lastCommand[MAX_COMMAND_LEN];

};

#endif //__COMMANDER_H__
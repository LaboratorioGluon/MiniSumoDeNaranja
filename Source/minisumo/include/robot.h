#ifndef __ROBOT_H__
#define __ROBOT_H__

#include "motorController.h"
#include "i2cExpander.h"
#include "labVl53l0x.h"


constexpr uint8_t NUM_ENEMY_SENSORS = 3;

class Robot
{
public:
    enum InitialState
    {
        ENEMY_FORWARD,
        ENEMY_RIGHT_SIDE,
        ENEMY_LEFT_SIDE
    };

    enum ENEMY_SENSOR
    {
        SENSOR_NONE = -1,
        SENSOR_LEFT,
        SENSOR_FRONT,
        SENSOR_RIGHT
    };

    enum LINE_SENSOR
    {
        LINE_NONE = -1,
        LINE_LEFT,
        LINE_RIGHT,
    };

    Robot();

    void Init(enum InitialState state);

    enum ENEMY_SENSOR ReadEnemySensors();
    enum LINE_SENSOR ReadLineSensors();

    void Loop();

private:
    uint8_t enemySensor[NUM_ENEMY_SENSORS];
};

#endif //__ROBOT_H__
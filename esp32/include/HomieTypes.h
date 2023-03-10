/**
 * @file HomieTypes.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2020-10-16
 * 
 * @copyright Copyright (c) 2020
 *  All Settings, configurable in Homie
 */
#ifndef HOMIE_PLANT_CFG_CONFIG_H
#define HOMIE_PLANT_CFG_CONFIG_H

#include <Homie.h>

/**
 * @name Sensor types
 * possible sensors:
 * @{
 **/

#define FOREACH_SENSOR(SENSOR) \
        SENSOR(NONE)   \
        SENSOR(FREQUENCY_MOD_RESISTANCE_PROBE)  \
        SENSOR(ANALOG_RESISTANCE_PROBE)   

/**
 * @}
 */

#define GENERATE_ENUM(ENUM) ENUM,
#define GENERATE_STRING(STRING) #STRING,

enum SENSOR_MODE {
    FOREACH_SENSOR(GENERATE_ENUM)
};

static const char *SENSOR_STRING[] = {
    FOREACH_SENSOR(GENERATE_STRING)
};

//plant pump is deactivated, but sensor values are still recorded and published
#define DEACTIVATED_PLANT -1
//special value to indicate a missing sensor when the plant is not deactivated but no valid sensor value was read
#define MISSING_SENSOR -2
//plant uses only cooldown and duration, moisture is measured but ignored, allowedHours is ignored (eg. make a 30min on 30min off cycle)
#define HYDROPONIC_MODE -3
//plant uses cooldown and duration and workhours, moisture is measured but ignored
#define TIMER_ONLY -4

/**
 * @brief State of plants
 * 
 */
#define PLANTSTATE_NUM_DEACTIVATED      -1
#define PLANTSTATE_NUM_NO_SENSOR        -2
#define PLANTSTATE_NUM_WET              0x00
#define PLANTSTATE_NUM_SUNNY_ALARM      0x11
#define PLANTSTATE_NUM_ACTIVE_ALARM     0x41
#define PLANTSTATE_NUM_ACTIVE_SUPESSED  -3
#define PLANTSTATE_NUM_ACTIVE           0x40
#define PLANTSTATE_NUM_SUNNY            0x10
#define PLANTSTATE_NUM_COOLDOWN_ALARM   0x21
#define PLANTSTATE_NUM_COOLDOWN         0x20
#define PLANTSTATE_NUM_AFTERWORK_ALARM  0x31
#define PLANTSTATE_NUM_AFTERWORK        0x30

#define PLANTSTATE_STR_DEACTIVATED      "deactivated"
#define PLANTSTATE_STR_NO_SENSOR        "nosensor"
#define PLANTSTATE_STR_WET              "wet"
#define PLANTSTATE_STR_SUNNY_ALARM      "sunny+alarm"
#define PLANTSTATE_STR_ACTIVE_ALARM     "active+alarm"
#define PLANTSTATE_STR_ACTIVE_SUPESSED  "active+supressed"
#define PLANTSTATE_STR_ACTIVE           "active"
#define PLANTSTATE_STR_SUNNY            "sunny"
#define PLANTSTATE_STR_COOLDOWN_ALARM   "cooldown+alarm"
#define PLANTSTATE_STR_COOLDOWN         "cooldown"
#define PLANTSTATE_STR_AFTERWORK_ALARM  "after-work+alarm"
#define PLANTSTATE_STR_AFTERWORK        "after-work"

typedef struct PlantSettings_t
{
    HomieSetting<double> *pSensorDry;
    HomieSetting<long> *pPumpAllowedHourRangeStart;
    HomieSetting<long> *pPumpAllowedHourRangeEnd;
    HomieSetting<bool> *pPumpOnlyWhenLowLight;
    HomieSetting<long> *pPumpCooldownInSeconds;
    HomieSetting<long> *pPumpDuration;
    HomieSetting<long> *pPumpPowerLevel;
    HomieSetting<long> *pPumpMl;
} PlantSettings_t;

#endif
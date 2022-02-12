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

#define SENSOR_NONE 0
#define SENSOR_CAPACITIVE_FREQUENCY_MOD 1
#define SENSOR_ANALOG_RESISTANCE_PROBE 2

//plant pump is deactivated, but sensor values are still recorded and published
#define DEACTIVATED_PLANT -1
//special value to indicate a missing sensor when the plant is not deactivated but no valid sensor value was read
#define MISSING_SENSOR -2
//plant uses only cooldown and duration, moisture is measured but ignored, allowedHours is ignored (eg. make a 30min on 30min off cycle)
#define HYDROPONIC_MODE -3

typedef struct PlantSettings_t
{
    HomieSetting<double> *pSensorDry;
    HomieSetting<long> *pSensorMode;
    HomieSetting<long> *pPumpAllowedHourRangeStart;
    HomieSetting<long> *pPumpAllowedHourRangeEnd;
    HomieSetting<bool> *pPumpOnlyWhenLowLight;
    HomieSetting<long> *pPumpCooldownInSeconds;
    HomieSetting<long> *pPumpDuration;
    HomieSetting<long> *pPumpPowerLevel;
    HomieSetting<long> *pPumpMl;
} PlantSettings_t;

#endif
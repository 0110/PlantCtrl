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

#define DEACTIVATED_PLANT -1
#define MISSING_SENSOR -2

typedef struct PlantSettings_t
{
    HomieSetting<double> *pSensorDry;
    HomieSetting<long> *pPumpAllowedHourRangeStart;
    HomieSetting<long> *pPumpAllowedHourRangeEnd;
    HomieSetting<bool> *pPumpOnlyWhenLowLight;
    HomieSetting<long> *pPumpCooldownInHours;
    HomieSetting<long> *pPumpDuration;
} PlantSettings_t;

#endif
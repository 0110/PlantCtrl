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

typedef struct PlantSettings_t {
    HomieSetting<long>* pSensorDry;
    HomieSetting<long>* pSensorWet;
    HomieSetting<long>* pPumpAllowedHourRangeStart;
    HomieSetting<long>* pPumpAllowedHourRangeEnd;
    HomieSetting<bool>* pPumpOnlyWhenLowLight;
    HomieSetting<long>* pPumpCooldownInHours;
} PlantSettings_t;

#endif
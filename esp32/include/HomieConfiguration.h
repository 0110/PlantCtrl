/**
 * @file HomieConfiguration.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2020-10-16
 * 
 * @copyright Copyright (c) 2020
 *  All Settings, configurable in Homie
 */
#ifndef HOMIE_PLANT_CONFIG_H
#define HOMIE_PLANT_CONFIG_H

#include "HomieTypes.h"

#define MAX_PLANTS 7

/**
 *********************************** Attributes *******************************
 */

HomieNode plant0("plant0", "Plant 0", "Plant");

HomieNode plant1("plant1", "Plant 1", "Plant");
HomieNode plant2("plant2", "Plant 2", "Plant");
HomieNode plant3("plant3", "Plant 3", "Plant");
HomieNode plant4("plant4", "Plant 4", "Plant");
HomieNode plant5("plant5", "Plant 5", "Plant");
HomieNode plant6("plant6", "Plant 6", "Plant");

HomieNode sensorLipo("lipo", "Battery Status", "Lipo");
HomieNode sensorSolar("solar", "Solar Status", "Solarpanel");
HomieNode sensorWater("water", "WaterSensor", "Water");
HomieNode sensorTemp("temperature", "Temperature", "temperature");
HomieNode stayAlive("stay", "alive", "alive");

/**
 *********************************** Settings *******************************
 */

HomieSetting<long> deepSleepTime("deepsleep", "time in milliseconds to sleep (0 deactivats it)");
HomieSetting<long> deepSleepNightTime("nightsleep", "time in milliseconds to sleep (0 usese same setting: deepsleep at night, too)");
HomieSetting<long> wateringDeepSleep("pumpdeepsleep", "time seconds to sleep, while a pump is running");

HomieSetting<long> waterLevelMax("watermaxlevel", "distance at maximum water level");
HomieSetting<long> waterLevelMin("waterminlevel", "distance at minimum water level (pumps still covered)");
HomieSetting<long> waterLevelWarn("waterlevelwarn", "warn if below this water level %");
HomieSetting<long> waterLevelVol("waterVolume", "ml between minimum and maximum");

/** Plant specific ones */

#define GENERATE_PLANT(plant, strplant)   \
        HomieSetting<long> mSensorDry##plant = HomieSetting<long>("moistdry" strplant, "Plant " strplant "- Moist sensor dry threshold"); \
        HomieSetting<long> mSensorWet##plant = HomieSetting<long>("moistwet" strplant, "Plant" strplant " - Moist sensor wet threshold"); \
        HomieSetting<long> mPumpAllowedHourRangeStart##plant = HomieSetting<long>("rangehourstart" strplant, "Plant" strplant " - Range pump allowed hour start"); \
        HomieSetting<long> mPumpAllowedHourRangeEnd##plant = HomieSetting<long>("rangehourend" strplant, "Plant" strplant " - Range pump allowed hour end"); \
        HomieSetting<bool> mPumpOnlyWhenLowLight##plant = HomieSetting<bool>("onlyWhenLowLightZ" strplant, "Plant" strplant " - Enable the Pump only, when there is light but not enought to charge battery"); \
        HomieSetting<long> mPumpCooldownInHours##plant = HomieSetting<long>("cooldownpump" strplant, "Plant" strplant " - How long to wait until the pump is activated again"); \
        PlantSettings_t mSetting##plant = { &mSensorDry##plant, &mSensorWet##plant, &mPumpAllowedHourRangeStart##plant, &mPumpAllowedHourRangeEnd##plant, &mPumpOnlyWhenLowLight##plant, &mPumpCooldownInHours##plant };
        
GENERATE_PLANT(0, "0");
GENERATE_PLANT(1, "1");
GENERATE_PLANT(2, "2");
GENERATE_PLANT(3, "3");
GENERATE_PLANT(4, "4");
GENERATE_PLANT(5, "5");
GENERATE_PLANT(6, "6");



#endif /* HOMIE_PLANT_CONFIG_H */
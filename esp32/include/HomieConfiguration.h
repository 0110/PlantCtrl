/** \addtogroup Homie
 *  @{
 * 
 * @file HomieConfiguration.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2020-10-16
 * 
 * @copyright Copyright (c) 2020
 *  All Settings, configurable in Homie
 * 
 */
#ifndef HOMIE_PLANT_CONFIG_H
#define HOMIE_PLANT_CONFIG_H

#include "HomieTypes.h"

#define MAX_PLANTS 7

/**
 * @name Homie Attributes
 * generated Information
 * @{
 **/

#define NUMBER_TYPE                     "Float"        /**< numberic information, published or read in Homie */

/**
 * @name Temperatur Node
 * @{
 **/

#define TEMPERATURE_NAME                "Temperature"
#define TEMPERATURE_UNIT                "°C"
#define TEMPERATUR_SENSOR_LIPO          "lipo"          /**< Homie node: temperatur, setting: lipo temperatur (or close to it) */
#define TEMPERATUR_SENSOR_CHIP          "chip"          /**< Homie node: temperatur, setting: battery chip */
#define TEMPERATUR_SENSOR_WATER         "water"         /**< Homie node: temperatur, setting: water temperatur */
/** @} 
 *
 * @name Plant Nodes
 * @{
 */

HomieNode plant0("plant0", "Plant 0", "Plant"); /**< dynamic Homie information for first plant */
HomieNode plant1("plant1", "Plant 1", "Plant"); /**< dynamic Homie information for second plant */
HomieNode plant2("plant2", "Plant 2", "Plant"); /**< dynamic Homie information for third plant */
HomieNode plant3("plant3", "Plant 3", "Plant"); /**< dynamic Homie information for fourth plant */
HomieNode plant4("plant4", "Plant 4", "Plant"); /**< dynamic Homie information for fivth plant */
HomieNode plant5("plant5", "Plant 5", "Plant"); /**< dynamic Homie information for sixth plant */
HomieNode plant6("plant6", "Plant 6", "Plant"); /**< dynamic Homie information for seventh plant */

#if defined(TIMED_LIGHT_PIN)
        HomieNode timedLightNode("timedLight", "TimedLight", "Status");
#endif // TIMED_LIGHT_PIN
 

HomieNode sensorLipo("lipo", "Battery Status", "Lipo");
HomieNode sensorSolar("solar", "Solar Status", "Solarpanel");
HomieNode sensorWater("water", "WaterSensor", "Water");
HomieNode sensorTemp("temperature", "Temperature", "temperature");
HomieNode stayAlive("stay", "alive", "alive");  /**< Necessary for Mqtt Active Command */

/**
 *  @} 
 */

/**
 * @name Settings
 * General settings for the controller
 * @{
 */
HomieSetting<long> deepSleepTime("sleep", "time in seconds to sleep");
HomieSetting<long> deepSleepNightTime("nightsleep", "time in seconds to sleep (0 uses same setting: deepsleep at night, too)");
HomieSetting<long> pumpIneffectiveWarning("pumpConsecutiveWarn", "if the pump was triggered this amount directly after each cooldown, without resolving dryness, warn");
HomieSetting<long> waterLevelMax("tankmax", "distance (mm) at maximum water level");
HomieSetting<long> waterLevelMin("tankmin", "distance (mm) at minimum water level (pumps still covered)");
HomieSetting<long> waterLevelWarn("tankwarn", "warn (mm) if below this water level %");
HomieSetting<long> waterLevelVol("tankVolume", "(ml) between minimum and maximum");
HomieSetting<const char *> lipoSensorAddr("lipoDSAddr", "1wire address for lipo temperature sensor");
HomieSetting<const char *> waterSensorAddr("tankDSAddr", "1wire address for water temperature sensor");
HomieSetting<const char *> ntpServer("ntpServer", "NTP server (pool.ntp.org as default)");

#if defined(TIMED_LIGHT_PIN)
        HomieSetting<double> timedLightVoltageCutoff("LightVoltageCutoff", "voltage at wich to disable light");
        HomieSetting<long> timedLightStart("LightStart", "hour to start light");
        HomieSetting<long> timedLightEnd("LightEnd", "hour to end light");
        HomieSetting<bool> timedLightOnlyWhenDark("LightOnlyDark", "only enable light, if solar is low");
#endif // TIMED_LIGHT_PIN


/**
 * @}
 */

/** 
 * @name Plant specific ones 
 * Setting for one plant
 * @{
 **/

#define GENERATE_PLANT(plant, strplant)                                                                                                                                                                        \
        HomieSetting<double> mSensorDry##plant = HomieSetting<double>("dry" strplant, "Plant " strplant "- Moist sensor dry %");                                                                      \
        HomieSetting<long> pSensorType##plant = HomieSetting<long>("sensoryType" strplant, "sensor" strplant " - sensortype");                      \
        HomieSetting<long> mPumpAllowedHourRangeStart##plant = HomieSetting<long>("hourstart" strplant, "Plant" strplant " - Range pump allowed hour start (0-23)");                                      \
        HomieSetting<long> mPumpAllowedHourRangeEnd##plant = HomieSetting<long>("hourend" strplant, "Plant" strplant " - Range pump allowed hour end (0-23)");                                            \
        HomieSetting<bool> mPumpOnlyWhenLowLight##plant = HomieSetting<bool>("lowLight" strplant, "Plant" strplant " - Enable the Pump only, when there is no sunlight"); \
        HomieSetting<long> mPumpCooldownInMinutes##plant = HomieSetting<long>("delay" strplant, "Plant" strplant " - How long to wait until the pump is activated again (minutes)");                      \
        HomieSetting<long> pPumpDuration##plant = HomieSetting<long>("pumpDuration" strplant, "Plant" strplant " - time seconds to water when pump is active");                      \
        HomieSetting<long> pPumpMl##plant = HomieSetting<long>("pumpAmount" strplant, "Plant" strplant " - ml (if using flowmeter) to water when pump is active");                      \
        HomieSetting<long> pPowerLevel##plant = HomieSetting<long>("powerLevel" strplant, "Plant" strplant " - pwm duty cycle in percent");                      \
        PlantSettings_t mSetting##plant = {&mSensorDry##plant, &pSensorType##plant, &mPumpAllowedHourRangeStart##plant, &mPumpAllowedHourRangeEnd##plant, &mPumpOnlyWhenLowLight##plant, &mPumpCooldownInMinutes##plant, &pPumpDuration##plant, &pPowerLevel##plant, &pPumpMl##plant}; \
        /**< Generate all settings for one plant \
         * \
         * Feature to start pumping only at morning: @link{SOLAR_CHARGE_MIN_VOLTAGE} and @link{SOLAR_CHARGE_MAX_VOLTAGE} \
         */

/**
 * @}
 */

GENERATE_PLANT(0, "0"); /**< Homie settings for first plant */
GENERATE_PLANT(1, "1"); /**< Homie settings for second Plant */
GENERATE_PLANT(2, "2"); /**< Homie settings for third plant */
GENERATE_PLANT(3, "3"); /**< Homie settings for fourth plant */
GENERATE_PLANT(4, "4"); /**< Homie settings for fifth plant */
GENERATE_PLANT(5, "5"); /**< Homie settings for sixth plant */
GENERATE_PLANT(6, "6"); /**< Homie settings for seventh plant */



#endif /* HOMIE_PLANT_CONFIG_H @} */


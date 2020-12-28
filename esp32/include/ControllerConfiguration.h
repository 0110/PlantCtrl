/**
 * @file ControllerConfiguration.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2020-05-30
 * 
 * @copyright Copyright (c) 2020
 * 
 * \mainpage Configuration of the controller
 * @{
 * Describe the used PINs of the controller
 *  
 * @subpage Controller
 * 
 * @subpage Homie
 * 
 * @subpage Configuration
 * 
 * There are several modes in the controller
 * \dot
 *  digraph Operationmode {
 *      ranksep=.75;
 *      poweroff [ label="off" ];
 *      mode1 [ label="Mode 1 - Sensor only", shape=box, width=2 ];
 *      mode2 [ label="Mode 2 - Wifi enabled", shape=box ];
 *      mode3 [ label="Mode 3 - Stay alive", shape=box ];
 *      mode1 -> mode2 [ label="wakeup reason", fontsize=10 ];
 *      mode1 -> mode2 [ label="Time duration", fontsize=10 ];
 *      mode2 -> mode3 [ label="Over the Air Update", fontsize=10 ];
 *      mode3 -> mode2 [ label="Over the Air Finished", fontsize=10 ];
 *      mode3 -> mode2 [ label="Mqtt Command", fontsize=10 ];
 *      mode2 -> mode3 [ label="Mqtt Command", fontsize=10 ];
 *      poweroff -> mode1 [ label="deep sleep wakeup", fontsize=10 ];
 *      mode1 -> poweroff [ label="enter deep sleep", fontsize=10 ];
 *      mode2 -> poweroff [ label="Mqtt queue empty", fontsize=10 ];
 *  }
 *  \enddot
 * 
 * Before entering Deep sleep the controller is configured with an wakeup time.
 * 
 * @}
 */
#ifndef CONTROLLER_CONFIG_H
#define CONTROLLER_CONFIG_H
/** \addtogroup Configuration
 *  @{
 */
#define FIRMWARE_VERSION "1.0.10"

#define ADC_TO_VOLT(adc)                    ((adc) * 3.3 ) / 4095)
#define ADC_TO_VOLT_WITH_MULTI(adc, multi)  (((adc)*3.3 * (multi)) / 4095)
#define MOIST_SENSOR_MAX_ADC                (85 * 4095 / 100)
#define MOIST_SENSOR_MIN_ADC                (25 * 4095 / 100)

#define SOLAR_VOLT(adc) ADC_TO_VOLT_WITH_MULTI(adc, 4.0306) /**< 100k and 33k voltage dividor */
#define ADC_5V_TO_3V3(adc) ADC_TO_VOLT_WITH_MULTI(adc, 1.69) /**< 33k and 47k8 voltage dividor */
#define MS_TO_S 1000

#define SENSOR_LIPO 34   /**< GPIO 34 (ADC1) */
#define SENSOR_SOLAR 35  /**< GPIO 35 (ADC1) */
#define SENSOR_PLANT0 12 /**< GPIO 32 (ADC1) */
#define SENSOR_PLANT1 14/**< GPIO 33 (ADC1) */
#define SENSOR_PLANT2 27 /**< GPIO 25 (ADC2) */
#define SENSOR_PLANT3 26 /**< GPIO 26 (ADC2) */
#define SENSOR_PLANT4 25 /**< GPIO 27 (ADC2) */
#define SENSOR_PLANT5 14 /**< GPIO 14 (ADC2) */
#define SENSOR_PLANT6 12 /**< GPIO 12 (ADC2) */

#define OUTPUT_PUMP0 15 /**< GPIO 23 */
#define OUTPUT_PUMP1 5 /**< GPIO 22 */
#define OUTPUT_PUMP2 18 /**< GPIO 21 */
#define OUTPUT_PUMP3 19 /**< GPIO 19 */
#define OUTPUT_PUMP4 21 /**< GPIO 18 */
#define OUTPUT_PUMP5 22  /**< GPIO 5  */
#define OUTPUT_PUMP6 23 /**< GPIO 15 */

#define OUTPUT_SENSOR 16 /**< GPIO 16 - Enable Sensors  */
#define OUTPUT_PUMP   13 /**< GPIO 13 - Enable Pumps  */

#define SENSOR_DS18B20 2 /**< GPIO 2 - Temperatur sensor */
#define BUTTON         0 /**< GPIO 0 - Fix button of NodeMCU */

#define MIN_TIME_RUNNING    5UL /**< Amount of seconds the controller must stay awoken */
#define MAX_PLANTS          7
#define MINIMUM_LIPO_VOLT   3.6f    /**< Minimum voltage of the Lipo, that must be present */
#define NO_LIPO_VOLT        2.0f    /**< No Lipo connected */
#define MINIMUM_SOLAR_VOLT  4.0f    /**< Minimum voltage of the sun, to detect daylight */
#define SOLAR_CHARGE_MIN_VOLTAGE 7  /**< Sun is rising (morning detected) */
#define SOLAR_CHARGE_MAX_VOLTAGE 9  /**< Sun is shining (noon)  */

#define MAX_CONFIG_SETTING_ITEMS 50 /**< Parameter, that can be configured in Homie */

#define PANIK_MODE_DEEPSLEEP (60 * 60 * 5U) /**< 5 hours in usecond */
#define PANIK_MODE_DEEPSLEEP_US (PANIK_MODE_DEEPSLEEP * 1000 * 1000)

#define TEMPERATURE_DELTA_TRIGGER_IN_C  1.0f
#define MOIST_DELTA_TRIGGER_ADC         10
#define SOLAR_DELTA_VOLT_ADC            3
#define LIPO_DELTA_VOLT_ADC             0.2 /**< trigger for lipo voltage */

#define TEMPERATUR_TIMEOUT              3000    /**< 3 Seconds timeout for the temperatur sensors */
#define TEMP_SENSOR_MEASURE_SERIES      5
#define VOLT_SENSOR_MEASURE_SERIES      5

/* @} */
#endif
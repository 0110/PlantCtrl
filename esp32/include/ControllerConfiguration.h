/**
 * @file ControllerConfiguration.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2020-05-30
 * 
 * @copyright Copyright (c) 2020
 *  Describe the used PINs of the controller
 */
#ifndef CONTROLLER_CONFIG_H
#define CONTROLLER_CONFIG_H

#define FIRMWARE_VERSION        "1.0.1"

#define ADC_TO_VOLT(adc)                      ((adc) * 3.3 ) / 4095)
#define ADC_TO_VOLT_WITH_MULTI(adc, multi)    (((adc) * 3.3 * (multi)) / 4095)

#define SOLAR_VOLT(adc)     ADC_TO_VOLT_WITH_MULTI(adc, 4.0306) /**< 100k and 33k voltage dividor */
#define ADC_5V_TO_3V3(adc)  ADC_TO_VOLT_WITH_MULTI(adc, 1.7)    /**< 33k and 47k8 voltage dividor */
#define MS_TO_S             1000

#define SENSOR_LIPO         34  /**< GPIO 34 (ADC1) */
#define SENSOR_SOLAR        35  /**< GPIO 35 (ADC1) */
#define SENSOR_PLANT0       32  /**< GPIO 32 (ADC1) */
#define SENSOR_PLANT1       33  /**< GPIO 33 (ADC1) */
#define SENSOR_PLANT2       25  /**< GPIO 25 (ADC2) */
#define SENSOR_PLANT3       25  /**< GPIO 26 (ADC2) */
#define SENSOR_PLANT4       27  /**< GPIO 27 (ADC2) */
#define SENSOR_PLANT5       14  /**< GPIO 14 (ADC2) */
#define SENSOR_PLANT6       12  /**< GPIO 12 (ADC2) */

#define OUTPUT_PUMP0        23  /**< GPIO 23  */
#define OUTPUT_PUMP1        22  /**< GPIO 22  */
#define OUTPUT_PUMP2        21  /**< GPIO 21 */
#define OUTPUT_PUMP3        19  /**< GPIO 19 */
#define OUTPUT_PUMP4        18  /**< GPIO 18 */
#define OUTPUT_PUMP5        29  /**< GPIO 29 */
#define OUTPUT_PUMP6        15  /**< GPIO 15 */
    
#define OUTPUT_SENSOR       16  /**< GPIO 16 - Enable Sensors  */
#define OUTPUT_PUMP         13  /**< GPIO 13 - Enable Pumps  */

#define SENSOR_DS18B20      2 /**< GPIO 2 */
#define BUTTON              0  /**< GPIO 0 */

#define MIN_TIME_RUNNING    5UL  /**< Amount of seconds the controller must stay awoken */
#define MAX_PLANTS          7 
#define EMPTY_LIPO_MULTIPL  3    /**< Multiplier to increase time for sleeping when lipo is empty */
#define MINIMUM_LIPO_VOLT   3.3f /**< Minimum voltage of the Lipo, that must be present */
#define MINIMUM_SOLAR_VOLT  4.0f /**< Minimum voltage of the sun, to detect daylight */

#define HC_SR04                  /**< Ultrasonic distance sensor to measure water level */
#define SENSOR_SR04_ECHO    17   /**< GPIO 17 - Echo */
#define SENSOR_SR04_TRIG    23   /**< GPIO 23 - Trigger */

#endif

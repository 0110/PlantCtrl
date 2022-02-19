#ifndef CONTROLLER_CONFIG_H
#define CONTROLLER_CONFIG_H

#define SENSOR_PLANT0 GPIO_NUM_32 /**< GPIO 32 (ADC1) */
#define SENSOR_PLANT1 GPIO_NUM_33 /**< GPIO 33 (ADC1) */
#define SENSOR_PLANT2 GPIO_NUM_25 /**< GPIO 25 (ADC2) */
#define SENSOR_PLANT3 GPIO_NUM_26 /**< GPIO 26 (ADC2) */
#define SENSOR_PLANT4 GPIO_NUM_27 /**< GPIO 27 (ADC2) */
#define SENSOR_PLANT5 GPIO_NUM_39 /**< SENSOR_VIN */
#define SENSOR_PLANT6 GPIO_NUM_36 /**< SENSOR_VP  */

#define OUTPUT_PUMP0 GPIO_NUM_15 /**< GPIO 15 */
#define OUTPUT_PUMP1 GPIO_NUM_5  /**< GPIO 5 */
#define OUTPUT_PUMP2 GPIO_NUM_18 /**< GPIO 18 */
#define OUTPUT_PUMP3 GPIO_NUM_19 /**< GPIO 19 */
#define OUTPUT_PUMP4 GPIO_NUM_21 /**< GPIO 21 */
#define OUTPUT_PUMP5 GPIO_NUM_22 /**< GPIO 22  */
#define OUTPUT_PUMP6 GPIO_NUM_23 /**< GPIO 23 */

#define OUTPUT_ENABLE_SENSOR GPIO_NUM_14 /**< GPIO 14 - Enable Sensors  */
#define OUTPUT_ENABLE_PUMP   GPIO_NUM_13 /**< GPIO 13 - Enable Pumps  */

#define SENSOR_ONEWIRE      GPIO_NUM_4 /**< GPIO 12 - Temperatur sensor, Battery and other cool onewire stuff */
#define SENSOR_TANK_ECHO    GPIO_NUM_16 /**< GPIO 16 - echo feedback of water sensor */ 
#define SENSOR_TANK_TRG     GPIO_NUM_17 /**< GPIO 17 - trigger for water sensor */
#define BUTTON              GPIO_NUM_0  /**< GPIO 0 - Fix button of NodeMCU */

#define CUSTOM1_PIN1        GPIO_NUM_34   /** direct gpio */
#define CUSTOM1_PIN3        GPIO_NUM_35   /** direct gpio */
#define CUSTOM1_PIN5        GPIO_NUM_2   /** mosfet controlled */
#define CUSTOM1_PIN7        GPIO_NUM_12   /** mosfet controlled */

#define I2C1_SDA          GPIO_NUM_34   /**< GPIO 34 - I2C */
#define I2C1_SCL          GPIO_NUM_35   /**< GPIO 35 - I2C */

#define FIRMWARE_VERSION "sw 2.2 hw 0.10b"

#define TIMED_LIGHT_PIN CUSTOM1_PIN5
#define FLOWMETER_PIN CUSTOM1_PIN1
#ifdef FLOWMETER_PIN
    #define FLOWMETER_FLOWFACTOR 23 /** F = 22 * Q;Q = L/min */
#endif

#define MOIST_SENSOR_MAX_FRQ               10000 // 10kHz (as never more then 3000 was measured)
#define MOIST_SENSOR_MIN_FRQ                1000 // 1kHz (500Hz margin)

#define ANALOG_SENSOR_MAX_MV                1000
#define ANALOG_SENSOR_MIN_MV                100

#define SOLAR_VOLT_FACTOR           11
#define BATTSENSOR_INDEX_SOLAR      0
#define BATTSENSOR_INDEX_BATTERY    1

#define MQTT_TIMEOUT                (1000 * 60) /**< After 10 seconds, MQTT is expected to be connected */
#define ESP_STALE_TIMEOUT           (MQTT_TIMEOUT+(700*1000))

#define MAX_PLANTS          7
#define SOLAR_CHARGE_MIN_VOLTAGE 7  /**< Sun is rising (morning detected) */
#define SOLAR_CHARGE_MAX_VOLTAGE 9  /**< Sun is shining (noon)  */
#define VOLT_MAX_BATT               4.2f

#define MAX_CONFIG_SETTING_ITEMS 100 /**< Parameter, that can be configured in Homie */
#define MAX_JSON_CONFIG_FILE_SIZE_CUSTOM 2500

#define TEMPERATUR_TIMEOUT              3000    /**< 3 Seconds timeout for the temperatur sensors */
#define DS18B20_RESOLUTION              9       /**< 9bit temperature resolution -> 0.5°C steps */

#define UTC_OFFSET_DE                   3600    /* UTC offset in seconds for Germany */
#define UTF_OFFSET_DE_DST               3600    /* offset in seconds if daylight saving time is active */

/* @} */

#endif
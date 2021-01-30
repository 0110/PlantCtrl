#include <Arduino.h>
#include "esp_sleep.h"
#include <DS18B20.h>

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  5        /* Time ESP32 will go to sleep (in seconds) */

#define SENSOR_DS18B20      2 /**< GPIO 2 */


#define OUTPUT_PUMP0        17  /**< GPIO 23  */
#define OUTPUT_PUMP1        05  /**< GPIO 22  */
#define OUTPUT_PUMP2        18  /**< GPIO 21 */
#define OUTPUT_PUMP3        19  /**< GPIO 19 */
#define OUTPUT_PUMP4        21  /**< GPIO 18 */
#define OUTPUT_PUMP5        22  /**< GPIO 5  */
#define OUTPUT_PUMP6        23  /**< GPIO 15 */

#define OUTPUT_SENSOR       16  /**< GPIO 16 - Enable Sensors  */
#define OUTPUT_PUMP         13  /**< GPIO 13 - Enable Pumps  */

#define SENSOR_PLANT0       32  /**< GPIO 32 (ADC1) */


#define ADC_TO_VOLT(adc)                      ((adc) * 3.3 ) / 4095)
#define ADC_TO_VOLT_WITH_MULTI(adc, multi)    (((adc) * 3.3 * (multi)) / 4095)


#define SOLAR_VOLT(adc)     ADC_TO_VOLT_WITH_MULTI(adc, 4.0306) /**< 100k and 33k voltage dividor */
#define ADC_5V_TO_3V3(adc)  ADC_TO_VOLT_WITH_MULTI(adc, 1.7)    /**< 33k and 47k8 voltage dividor */

RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR int pumpActive = 0;
int secondBootCount = 0;

Ds18B20 ds(SENSOR_DS18B20);

void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason){
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}

void setAll2Off() {
  digitalWrite(OUTPUT_PUMP0, LOW);
  digitalWrite(OUTPUT_PUMP1, LOW);
  digitalWrite(OUTPUT_PUMP2, LOW);
  digitalWrite(OUTPUT_PUMP3, LOW);
  digitalWrite(OUTPUT_PUMP4, LOW);
  digitalWrite(OUTPUT_PUMP5, LOW);
  digitalWrite(OUTPUT_PUMP6, LOW);
  digitalWrite(OUTPUT_SENSOR, LOW);
  digitalWrite(OUTPUT_PUMP, LOW);
}

void setup() {

  pinMode(OUTPUT_PUMP0, OUTPUT);
  pinMode(OUTPUT_PUMP1, OUTPUT);
  pinMode(OUTPUT_PUMP2, OUTPUT);
  pinMode(OUTPUT_PUMP3, OUTPUT);
  pinMode(OUTPUT_PUMP4, OUTPUT);
  pinMode(OUTPUT_PUMP5, OUTPUT);
  pinMode(OUTPUT_PUMP6, OUTPUT);
  pinMode(OUTPUT_SENSOR, OUTPUT);
  pinMode(OUTPUT_PUMP, OUTPUT);

  pinMode(SENSOR_PLANT0, ANALOG);

  setAll2Off();
  
  Serial.begin(115200);

  //Increment boot number and print it every reboot
  ++bootCount;
  ++secondBootCount;
  Serial.println("Boot number: " + String(bootCount) + " " + String(secondBootCount));

  //Print the wakeup reason for ESP32
  print_wakeup_reason();

  /* activate power pump and pump 0 */
  digitalWrite(OUTPUT_PUMP, HIGH);
  digitalWrite(OUTPUT_SENSOR, HIGH);
}



void loop() {  
  Serial.println("test");
  delay(200);
 
}
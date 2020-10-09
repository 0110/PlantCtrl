#include <Arduino.h>
#include "esp_sleep.h"

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  2        /* Time ESP32 will go to sleep (in seconds) */

#define OUTPUT_PUMP0        23  /**< GPIO 23  */
#define OUTPUT_PUMP1        22  /**< GPIO 22  */
#define OUTPUT_PUMP2        21  /**< GPIO 21 */
#define OUTPUT_PUMP3        19  /**< GPIO 19 */
#define OUTPUT_PUMP4        18  /**< GPIO 18 */
#define OUTPUT_PUMP5        5   /**< GPIO 5  */
#define OUTPUT_PUMP6        15  /**< GPIO 15 */

#define OUTPUT_SENSOR       16  /**< GPIO 16 - Enable Sensors  */
#define OUTPUT_PUMP         13  /**< GPIO 13 - Enable Pumps  */


RTC_DATA_ATTR int bootCount = 0;
int secondBootCount = 0;

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

  setAll2Off();
  
  Serial.begin(115200);

  //Increment boot number and print it every reboot
  ++bootCount;
  ++secondBootCount;
  Serial.println("Boot number: " + String(bootCount) + " " + String(secondBootCount));

  //Print the wakeup reason for ESP32
  print_wakeup_reason();

  /*
  First we configure the wake up source
  We set our ESP32 to wake up every 5 seconds
  */
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) +
  " Seconds");
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_ON);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_XTAL,ESP_PD_OPTION_ON);



  pinMode(GPIO_NUM_32, ANALOG);
  digitalWrite(GPIO_NUM_32, HIGH);
  pinMode(GPIO_NUM_33, INPUT_PULLUP);
  digitalWrite(GPIO_NUM_33, HIGH);
  pinMode(GPIO_NUM_25, INPUT_PULLUP);
  digitalWrite(GPIO_NUM_25, HIGH);
  pinMode(GPIO_NUM_26, INPUT_PULLUP);
  pinMode(GPIO_NUM_27, INPUT_PULLUP);
  pinMode(GPIO_NUM_14, INPUT_PULLUP);
  pinMode(GPIO_NUM_12, INPUT_PULLUP);
}



void loop() {
  Serial.println("------------");
  Serial.flush(); 
  delay(1000);
  digitalWrite(GPIO_NUM_23, analogRead(GPIO_NUM_34) > 3500);
  
  digitalWrite(OUTPUT_PUMP0, HIGH);
  delay(1000);
  setAll2Off();

  digitalWrite(OUTPUT_PUMP1, HIGH);
  delay(1000);
  setAll2Off();

  digitalWrite(OUTPUT_PUMP2, HIGH);
  delay(1000);
  setAll2Off();

  digitalWrite(OUTPUT_PUMP3, HIGH);
  delay(1000);
  setAll2Off();

  digitalWrite(OUTPUT_PUMP4, HIGH);
  delay(1000);
  setAll2Off();

  digitalWrite(OUTPUT_PUMP5, HIGH);
  delay(1000);
  setAll2Off();

  digitalWrite(OUTPUT_PUMP6, HIGH);
  delay(1000);
  setAll2Off();

  digitalWrite(OUTPUT_SENSOR, HIGH);
  delay(1000);
  setAll2Off();

  digitalWrite(OUTPUT_PUMP, HIGH);
  delay(1000);
  setAll2Off();

  delay(1000);
  delay(1000);
  gpio_deep_sleep_hold_en();

  esp_deep_sleep_start();
}
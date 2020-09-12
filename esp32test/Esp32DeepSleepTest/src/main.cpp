#include <Arduino.h>
#include "esp_sleep.h"

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  2        /* Time ESP32 will go to sleep (in seconds) */

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

void setup() {
  pinMode(GPIO_NUM_23, OUTPUT);
  pinMode(GPIO_NUM_22, OUTPUT);
  pinMode(GPIO_NUM_21, OUTPUT);
  pinMode(GPIO_NUM_19, OUTPUT);
  pinMode(GPIO_NUM_18, OUTPUT);
  pinMode(GPIO_NUM_5, OUTPUT);
  pinMode(GPIO_NUM_4, OUTPUT);
  pinMode(GPIO_NUM_15, OUTPUT);
  pinMode(GPIO_NUM_13, OUTPUT);
  digitalWrite(GPIO_NUM_23, HIGH);
  digitalWrite(GPIO_NUM_22, HIGH);
  digitalWrite(GPIO_NUM_21, HIGH);
  digitalWrite(GPIO_NUM_19, HIGH);
  digitalWrite(GPIO_NUM_18, HIGH);
  digitalWrite(GPIO_NUM_5, HIGH);
  digitalWrite(GPIO_NUM_4, HIGH);
  digitalWrite(GPIO_NUM_15, HIGH);
  digitalWrite(GPIO_NUM_13, HIGH);
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
  
  Serial.println(analogRead(GPIO_NUM_34));
  
  Serial.println(analogRead(GPIO_NUM_35));
  
  Serial.println(analogRead(GPIO_NUM_32));
  
  Serial.println(analogRead(GPIO_NUM_33));
  
  Serial.println(analogRead(GPIO_NUM_25));
  
  Serial.println(analogRead(GPIO_NUM_26));
  
  Serial.println(analogRead(GPIO_NUM_27));
  Serial.println(analogRead(GPIO_NUM_14));
  Serial.println(analogRead(GPIO_NUM_12));
   
  gpio_hold_en(GPIO_NUM_4);
  gpio_hold_en(GPIO_NUM_13);
  gpio_hold_en(GPIO_NUM_15);
  gpio_deep_sleep_hold_en();

  esp_deep_sleep_start();
}
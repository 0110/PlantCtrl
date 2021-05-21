#include <Arduino.h>
#include "esp_sleep.h"
#include "DallasTemperature.h"
#include "DS2438.h"

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  5        /* Time ESP32 will go to sleep (in seconds) */

#define SENSOR_DS18B20      2 /**< GPIO 2 */

#define OUTPUT_PUMP0        17  /**< GPIO 23  */
#define OUTPUT_PUMP1        05  /**< GPIO 22  */
#define OUTPUT_PUMP2        18  /**< GPIO 21 */
#define OUTPUT_PUMP3        19  /**< GPIO 19 */
#define OUTPUT_PUMP4        21  /**< GPIO 18 */
#define OUTPUT_PUMP5        15  /**< GPIO 5  */
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

OneWire oneWire(SENSOR_DS18B20);
DallasTemperature temp(&oneWire);
DS2438 battery(&oneWire,0.1f);


void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason){
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\r\n",wakeup_reason); break;
  }
}

bool whatever = true;

void setAll2to(int state) {
  Serial.println("Set GPIO" + String(OUTPUT_PUMP0) + "=" + String(state));
  digitalWrite(OUTPUT_PUMP0, state);
  Serial.println("Set GPIO" + String(OUTPUT_PUMP1) + "=" + String(state));
  digitalWrite(OUTPUT_PUMP1, state);
  Serial.println("Set GPIO" + String(OUTPUT_PUMP2) + "=" + String(state));
  digitalWrite(OUTPUT_PUMP2, state);
  Serial.println("Set GPIO" + String(OUTPUT_PUMP3) + "=" + String(state));
  digitalWrite(OUTPUT_PUMP3, state);
  Serial.println("Set GPIO" + String(OUTPUT_PUMP4) + "=" + String(state));
  digitalWrite(OUTPUT_PUMP4, state);
  Serial.println("Set GPIO" + String(OUTPUT_PUMP5) + "=" + String(state));
  digitalWrite(OUTPUT_PUMP5, state);
  Serial.println("Set GPIO" + String(OUTPUT_PUMP6) + "=" + String(state));
  digitalWrite(OUTPUT_PUMP6, state);
  Serial.println("Set GPIO" + String(OUTPUT_SENSOR) + "=" + String(state));
  digitalWrite(OUTPUT_SENSOR, state);
}

void setup() {

  Serial.begin(115200);
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


  //Increment boot number and print it every reboot
  ++bootCount;
  ++secondBootCount;
  Serial.println("Boot number: " + String(bootCount) + " " + String(secondBootCount));

  //Print the wakeup reason for ESP32
  print_wakeup_reason();
  Serial.println("------- build from " + String(__DATE__) + "=" + String(__TIME__) + " @ " + String(millis()) + "ms");
  Serial.println("Set GPIO" + String(OUTPUT_PUMP) + "=" + String(LOW));
  digitalWrite(OUTPUT_PUMP, LOW);

  setAll2to(HIGH);
  delay(1000);
  Serial.println("--------------------------" + String(" @ ") + String(millis()) + "ms");
  setAll2to(LOW);
  delay(1000);
  Serial.println("--------------------------" + String(" @ ") + String(millis()) + "ms");

  /* activate power pump and pump 0 */

  digitalWrite(OUTPUT_SENSOR, HIGH);

  delay(1);
  
  temp.begin();
  battery.begin();

  Serial.print("Battery");
  Serial.print("\t");
  Serial.print("Solar");
  Serial.print("\t");
  Serial.print("Bat I");
  Serial.print("\t");
  Serial.println("Temp/10");

}

void loop() { 
  static int loop=1;


  DeviceAddress t;
  for(int i=0; i < sizeof(t); i++) {
    t[i] = loop + i*2;
  }
  char buf[sizeof(DeviceAddress)*2];
  snprintf(buf, sizeof(buf), "%.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X", t[0], t[1], t[2], t[3], t[4], t[5], t[6], t[7]);
/*
  %X -> linksbündige hexzahl
  %2X -> 2Stellige hexzahl ... dynamisch erweitert
  %0.2X -> 2stellige hexzahl mit führerder "0"
  */
  printf("Print: %s\n", buf);
  loop++;
  delay(500);
  return;

  whatever = !whatever;
  digitalWrite(OUTPUT_PUMP, HIGH);
  delay(500);
  digitalWrite(OUTPUT_PUMP6, HIGH);

  for(int j=0; j < 5 && temp.getDeviceCount() == 0; j++) {
    delay(10);
   // Serial.println("Reset 1wire temp");
    temp.begin();
  }
  
  for(int j=0; j < 5 && (0 == battery.isFound()); j++) {
    delay(10);
    Serial.println("Reset 1wire bat");
    battery.begin();
    battery.update();
  }
  battery.update();
  Serial.print(battery.getVoltage(0)); //use define here, solar
  Serial.print("\t");
  Serial.print(battery.getVoltage(1)); //use define here, battery
  Serial.print("\t");
  Serial.print(battery.getCurrent());
  Serial.print("\t");
  Serial.println(battery.getTemperature()/10);
}
#include <Arduino.h>
#include "driver/pcnt.h"
#include "ulp-pwm.h"

RTC_SLOW_ATTR uint8_t tick = 0;
RTC_SLOW_ATTR bool dir = true;
void setup()
{
  Serial.begin(115200);
  Serial.println(rtc_io_number_get(GPIO_NUM_12));
  ulp_pwm_init();

  if (dir)  {
    tick += 10;
  }  else  {
    tick -= 10;
  }
  ulp_pwm_set_level(tick);
  if (tick == 250 || tick == 0)  {
    dir = !dir;
  }
  esp_sleep_enable_timer_wakeup(1000000);
  Serial.println(ulp_data_read(ulp_dimm_offset));
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
  esp_deep_sleep_start();
}

void loop(){

}
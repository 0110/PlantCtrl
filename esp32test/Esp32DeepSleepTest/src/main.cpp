#include <Arduino.h>

RTC_SLOW_ATTR uint8_t tick = 0;
RTC_SLOW_ATTR bool dir = true;
void setup()
{
  Serial.begin(115200);
  
}

void loop(){
  Serial.println("alive");
}
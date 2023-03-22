#include <Arduino.h>
#include "driver/pcnt.h"
#include <VL53L0X.h>


#define OUTPUT_SENSOR       14
#define SENSOR_PLANT        17

VL53L0X tankSensor;

void setup()
{
  Serial.begin(115200);
  pinMode(OUTPUT_SENSOR, OUTPUT);
  tankSensor.setTimeout(500);

  digitalWrite(OUTPUT_SENSOR, HIGH);
  Serial.println("Nodemcu ESP32 Start done");

  tankSensor.setTimeout(500);
  long start = millis();
  bool distanceReady = false;
  while (start + 500 > millis())
  {
    if (tankSensor.init())
    {
      distanceReady = true;
      break;
    }
    else
    {
      delay(20);
    }
  }

  if (distanceReady)
  {
    Serial.println("Sensor init done");
    tankSensor.setSignalRateLimit(0.1);
    // increase laser pulse periods (defaults are 14 and 10 PCLKs)
    tankSensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    tankSensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
    tankSensor.setMeasurementTimingBudget(200000);
  } else {
    Serial.println("Sensor init failed");
  }
}

void loop() { 
    
  delay(500);
    
   
  if (!tankSensor.timeoutOccurred())
  {
    uint16_t distance = tankSensor.readRangeSingleMillimeters();
  }
}

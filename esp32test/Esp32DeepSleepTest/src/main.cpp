#include <Arduino.h>
#include "driver/pcnt.h"
#include <VL53L0X.h>

#define SENSOR_TANK_SDA    GPIO_NUM_16 /**< GPIO 16 - echo feedback of water sensor */ 
#define SENSOR_TANK_SCL    GPIO_NUM_17 /**< GPIO 17 - trigger for water sensor */


#define OUTPUT_SENSOR       14
#define SENSOR_PLANT        17

VL53L0X tankSensor;
bool distanceReady = false;

void initializeTanksensor() {
  Wire.begin(SENSOR_TANK_SDA, SENSOR_TANK_SCL, 100000UL /* 100kHz */);
  tankSensor.setBus(&Wire);
  delay(100);
  tankSensor.setTimeout(500);
  long start = millis();
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

  if ((distanceReady) && (!tankSensor.timeoutOccurred()))
  {
    Serial.println("Sensor init done");
    tankSensor.setSignalRateLimit(0.1);
    // increase laser pulse periods (defaults are 14 and 10 PCLKs)
    tankSensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    tankSensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
    tankSensor.setMeasurementTimingBudget(200000);
    tankSensor.startContinuous();
  } else {
    Serial.println("Sensor init failed");
  }
}

void setup()
{
  Serial.begin(115200);
  pinMode(OUTPUT_SENSOR, OUTPUT);

  digitalWrite(OUTPUT_SENSOR, HIGH);
  Serial.println("Nodemcu ESP32 Start done");

  initializeTanksensor();
}

void loop() { 
    
  delay(500);
    
   
  if ((distanceReady) && (!tankSensor.timeoutOccurred()))
  {
    uint16_t distance = tankSensor.readRangeSingleMillimeters();
    if (distance > 8000) {
      Serial.println("Reset due invalid distance: 8 meter");
      Wire.end();
      delay(1000);
      initializeTanksensor();
    } else {
      Serial.print("Distance");
      Serial.println(distance);
    }
  } else {
    Serial.println("Timeout");
    tankSensor.stopContinuous();
    Wire.end();
    delay(100);
    initializeTanksensor();
  }
}

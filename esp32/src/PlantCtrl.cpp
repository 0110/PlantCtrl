/**
 * @file PlantCtrl.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2020-05-27
 * 
 * @copyright Copyright (c) 2020
 * 

 */

#include "PlantCtrl.h"
#include "ControllerConfiguration.h"
#include "TimeUtils.h"
#include "MathUtils.h"

double mapf(double x, double in_min, double in_max, double out_min, double out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

Plant::Plant(int pinSensor, int pinPump, int plantId, HomieNode *plant, PlantSettings_t *setting)
{
    this->mPinSensor = pinSensor;
    this->mPinPump = pinPump;
    this->mPlant = plant;
    this->mSetting = setting;
    this->mPlantId = plantId;
}

void Plant::init(void)
{
    /* Initialize Home Settings validator */
    this->mSetting->pSensorDry->setDefaultValue(DEACTIVATED_PLANT);
    this->mSetting->pSensorDry->setValidator([](double candidate) {
        return (((candidate >= 0.0) && (candidate <= 100.0)) || equalish(candidate,DEACTIVATED_PLANT));
    });
    this->mSetting->pPumpAllowedHourRangeStart->setDefaultValue(8); // start at 8:00
    this->mSetting->pPumpAllowedHourRangeStart->setValidator([](long candidate) {
        return ((candidate >= 0) && (candidate <= 23));
    });
    this->mSetting->pPumpAllowedHourRangeEnd->setDefaultValue(20); // stop pumps at 20:00
    this->mSetting->pPumpAllowedHourRangeEnd->setValidator([](long candidate) {
        return ((candidate >= 0) && (candidate <= 23));
    });
    this->mSetting->pPumpOnlyWhenLowLight->setDefaultValue(true);
    this->mSetting->pPumpCooldownInHours->setDefaultValue(20); // minutes
    this->mSetting->pPumpCooldownInHours->setValidator([](long candidate) {
        return ((candidate >= 0) && (candidate <= 1024));
    });

    /* Initialize Hardware */
    Serial.println("Set GPIO mode " + String(mPinPump) + "=" + String(OUTPUT));
    Serial.flush();
    pinMode(this->mPinPump, OUTPUT);
    Serial.println("Set GPIO mode " + String(mPinSensor) + "=" + String(ANALOG));
    Serial.flush();
    pinMode(this->mPinSensor, ANALOG);
    Serial.println("Set GPIO " + String(mPinPump) + "=" + String(LOW));
    Serial.flush();
    digitalWrite(this->mPinPump, LOW);
}

void Plant::clearMoisture(void){
    this->moistureRaw.clear();
}

void Plant::addSenseValue(void)
{   
    int raw = analogRead(this->mPinSensor);
    Serial << "plant bla " << raw << endl;
    if(raw < MOIST_SENSOR_MAX_ADC && raw > MOIST_SENSOR_MIN_ADC){
        this->moistureRaw.add(raw);
    }
}

void Plant::postMQTTconnection(void)
{
    const String OFF = String("OFF");
    this->mConnected = true;
    this->mPlant->setProperty("switch").send(OFF);

    long raw = getCurrentMoisture();
    double pct = 100 - mapf(raw, MOIST_SENSOR_MIN_ADC, MOIST_SENSOR_MAX_ADC, 0, 100);
    if (equalish(raw, MISSING_SENSOR))
    {
      pct = 0;
    }
    if (pct < 0)
    {
      pct = 0;
    }
    if (pct > 100)
    {
      pct = 100;
    }

    this->mPlant->setProperty("moist").send(String(round(pct*10)/10));
    this->mPlant->setProperty("moistraw").send(String(raw));

}

void Plant::deactivatePump(void)
{
    int plantId = this->mPlantId;
    Serial << "deactivating pump " << plantId << endl;
    digitalWrite(this->mPinPump, LOW);
    if (this->mConnected)
    {
        const String OFF = String("OFF");
        this->mPlant->setProperty("switch").send(OFF);
    }
}

void Plant::publishState(String state) {
    if (this->mConnected)
    {
        this->mPlant->setProperty("state").send(state);
    }
}

void Plant::activatePump(void)
{
    int plantId = this->mPlantId;
    Serial << "activating pump " << plantId << endl;
    digitalWrite(this->mPinPump, HIGH);
    if (this->mConnected)
    {
        const String OFF = String("ON");
        this->mPlant->setProperty("switch").send(OFF);
        this->mPlant->setProperty("lastPump").send(String(getCurrentTime()));
    }
}

void Plant::advertise(void)
{
    // Advertise topics
    this->mPlant->advertise("switch").setName("Pump 1").setDatatype("boolean");
    this->mPlant->advertise("lastPump").setName("lastPump").setDatatype("number").setUnit("unixtime");
    //FIXME add .settable(this->switchHandler)
    this->mPlant->advertise("moist").setName("Percent").setDatatype("number").setUnit("%");
    this->mPlant->advertise("moistraw").setName("adc").setDatatype("number").setUnit("3.3/4096V");
    this->mPlant->advertise("state").setName("state").setDatatype("string");
}

/* FIXME
bool Plant::switchHandler(const HomieRange& range, const String& value) {
  if (range.isRange) return false;  // only one switch is present
  

    if ((value.equals("ON")) || (value.equals("On")) || (value.equals("on")) || (value.equals("true"))) {
      this->activatePump();
      return true;
    } else if ((value.equals("OFF")) || (value.equals("Off")) || (value.equals("off")) || (value.equals("false")) ) {
      this->deactivatePump();
      return true;
    } else {
      return false;
    }
  }
}
*/
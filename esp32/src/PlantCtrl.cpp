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

Plant::Plant(int pinSensor, int pinPump,int plantId, HomieNode* plant, PlantSettings_t* setting) {
    this->mPinSensor = pinSensor;
    this->mPinPump = pinPump;
    this->mPlant = plant;
    this->mSetting = setting;    
}

void Plant::init(void) {
    /* Initialize Home Settings validator */
    this->mSetting->pSensorDry->setDefaultValue(DEACTIVATED_PLANT);
    this->mSetting->pSensorDry->setValidator([] (long candidate) {
        return (((candidate >= 0) && (candidate <= 4095) ) || candidate == DEACTIVATED_PLANT);
    });
    this->mSetting->pPumpAllowedHourRangeStart->setDefaultValue(8); // start at 8:00
    this->mSetting->pPumpAllowedHourRangeStart->setValidator([] (long candidate) {
        return ((candidate >= 0) && (candidate <= 23) );
    });
    this->mSetting->pPumpAllowedHourRangeEnd->setDefaultValue(20); // stop pumps at 20:00
    this->mSetting->pPumpAllowedHourRangeEnd->setValidator([] (long candidate) {
        return ((candidate >= 0) && (candidate <= 23) );
    });
    this->mSetting->pPumpOnlyWhenLowLight->setDefaultValue(true);
    this->mSetting->pPumpCooldownInHours->setDefaultValue(20); // minutes
    this->mSetting->pPumpCooldownInHours->setValidator([] (long candidate) {
        return ((candidate >= 0) && (candidate <= 1024) );
    });

    /* Initialize Hardware */
    pinMode(this->mPinPump, OUTPUT);
    pinMode(this->mPinSensor, ANALOG);
    digitalWrite(this->mPinPump, LOW);   
}

void Plant::addSenseValue(void) {
    this->moistureRaw.add( analogRead(this->mPinSensor) );
}

void Plant::postMQTTconnection(void) {
    const String OFF = String("OFF");
    this->mConnected=true;
    this->mPlant->setProperty("switch").send(OFF);
}

void Plant::deactivatePump(void) {
    digitalWrite(this->mPinPump, LOW);
    if (this->mConnected) {
        const String OFF = String("OFF");
        this->mPlant->setProperty("switch").send(OFF);
    }
}

void Plant::activatePump(void) {
    digitalWrite(this->mPinPump, HIGH);
    if (this->mConnected) {
        const String OFF = String("ON");
        this->mPlant->setProperty("switch").send(OFF);
    }
}

void Plant::advertise(void) {
    // Advertise topics
    this->mPlant->advertise("switch").setName("Pump 1")
                              .setDatatype("boolean");
    //FIXME add .settable(this->switchHandler)
    this->mPlant->advertise("moist").setName("Percent")
                              .setDatatype("number")
                              .setUnit("%");
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
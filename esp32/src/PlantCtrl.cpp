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
    this->mSetting->pSensorDry->setDefaultValue(DEACTIVATED_PLANT);
    this->mSetting->pSensorDry->setValidator([] (long candidate) {
        return (((candidate >= 0) && (candidate <= 4095) ) || candidate == DEACTIVATED_PLANT);
    });
    this->mSetting->pSensorWet->setDefaultValue(0);
    this->mSetting->pSensorWet->setValidator([] (long candidate) {
        return ((candidate >= 0) && (candidate <= 4095) );
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
    
}

void Plant::addSenseValue(int analog) {
    this->mAnalogValue += analog;
}

void Plant::calculateSensorValue(int amountMeasurePoints) {
    this->mValue = this->mAnalogValue / amountMeasurePoints;
    this->mAnalogValue = 0;
}

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

Plant::Plant(int pinSensor, int pinPump,
    HomieNode *plant,
    HomieSetting<long> *sensorTriggerLevel, 
    HomieSetting<long> *wateringTime,
    HomieSetting<long> *wateringIdleTime) {
        this->mPlant=plant;
        this->mPinSensor = pinSensor;
        this->mPinPump = pinPump;
        this->mSensorTriggerLevel=sensorTriggerLevel;
        this->mWateringTime=wateringTime;
        this->mWateringIdleTime=wateringIdleTime;
}

void Plant::addSenseValue(int analog) {
    this->mAnalogValue += analog;
}

void Plant::calculateSensorValue(int amountMeasurePoints) {
    this->mValue = this->mAnalogValue / amountMeasurePoints;
    this->mAnalogValue = 0;
}

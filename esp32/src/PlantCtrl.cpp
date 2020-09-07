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

Plant::Plant(int pinSensor, int pinPump) {
    this->mPinSensor = pinSensor;
    this->mPinPump = pinPump;
}

void Plant::addSenseValue(int analog) {
    this->mAnalogValue += analog;
}

void Plant::calculateSensorValue(int amountMeasurePoints) {
    this->mValue = this->mAnalogValue / amountMeasurePoints;
    this->mAnalogValue = 0;
}

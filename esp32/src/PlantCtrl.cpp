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

Plant::Plant(int pinSensor, int pinPump,int plantId) {
    this->mPinSensor = pinSensor;
    this->mPinPump = pinPump;

    char plantIdChar = plantId+'0';

    /*
    {
        char* name = "moistZdry";
        name[5]= plantIdChar;
        mSensorDry = new HomieSetting<long>(name, "Moist sensor dry threshold");
        mSensorDry->setDefaultValue(4095);
        mSensorDry->setValidator([] (long candidate) {
            return ((candidate >= 0) && (candidate <= 4095) );
        });
    }
    {
        char* name = "moistZwet";
        name[6]= plantIdChar;
        mSensorWet = new HomieSetting<long>(name, "Moist sensor wet threshold");
        mSensorWet->setDefaultValue(0);
        mSensorWet->setValidator([] (long candidate) {
            return ((candidate >= 0) && (candidate <= 4095) );
        });
    }
    {
        char* name = "rangeZhourstart";
        name[6]= plantIdChar;
        mPumpAllowedHourRangeStart = new HomieSetting<long>(name, "Range pump allowed hour start");
        mPumpAllowedHourRangeStart->setDefaultValue(8);
        mPumpAllowedHourRangeStart->setValidator([] (long candidate) {
            return ((candidate >= 0) && (candidate <= 23) );
        });
    }
    {
        char* name = "rangeZhourend";
        name[6]= plantIdChar;
        mPumpAllowedHourRangeEnd = new HomieSetting<long>(name, "Range pump allowed hour end");
        mPumpAllowedHourRangeEnd->setDefaultValue(20);
        mPumpAllowedHourRangeEnd->setValidator([] (long candidate) {
            return ((candidate >= 0) && (candidate <= 23) );
        });
    }
    {
        char* name = "onlyWhenLowLightZ";
        name[16]= plantIdChar;
        mPumpOnlyWhenLowLight = new HomieSetting<bool>(name, "Enable the Pump only, when there is light but not enought to charge battery");
        mPumpOnlyWhenLowLight->setDefaultValue(true);
    }
    {
        char* name = "cooldownpumpZ";
        name[12]= plantIdChar;
        mPumpCooldownInHours = new HomieSetting<long>(name, "How long to wait until the pump is activated again");
        mPumpCooldownInHours->setDefaultValue(20);
        mPumpCooldownInHours->setValidator([] (long candidate) {
            return ((candidate >= 0) && (candidate <= 1024) );
        });
    }
*/
}

void Plant::addSenseValue(int analog) {
    this->mAnalogValue += analog;
}

void Plant::calculateSensorValue(int amountMeasurePoints) {
    this->mValue = this->mAnalogValue / amountMeasurePoints;
    this->mAnalogValue = 0;
}

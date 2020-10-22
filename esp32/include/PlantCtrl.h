/**
 * @file PlantCtrl.h
 * @author your name (you@domain.com)
 * @brief Abstraction to handle the Sensors
 * @version 0.1
 * @date 2020-05-27
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#ifndef PLANT_CTRL_H
#define PLANT_CTRL_H

#include "HomieTypes.h"
#include "RunningMedian.h"

class Plant {

private:
    RunningMedian moistureRaw = RunningMedian(5);
    HomieNode* mPlant = NULL;

public:
    //FIXME visibility
    int mPinSensor=0;   /**< Pin of the moist sensor */
    int mPinPump=0;     /**< Pin of the pump */
    PlantSettings_t* mSetting;
    /**
     * @brief Construct a new Plant object
     * 
     * @param pinSensor Pin of the Sensor to use to measure moist
     * @param pinPump   Pin of the Pump to use
     */
    Plant(int pinSensor, int pinPump,
            int plantId, 
            HomieNode* plant,
            PlantSettings_t* setting);

    /**
     * @brief Add a value, to be measured
     * 
     * @param analogValue 
     */
    void addSenseValue(int analogValue);
    
    /**
     * @brief Get the Sensor Pin of the analog measuring
     * 
     * @return int 
     */
    int  getSensorPin() { return mPinSensor; }

    /**
     * @brief Get the Pump Pin object
     * 
     * @return int 
     */
    int getPumpPin() { return mPinPump; }

    int getSensorValue() { return moistureRaw.getMedian(); }

    /**
     * @brief Check if a plant is too dry and needs some water.
     * 
     * @return true 
     * @return false 
     */
    bool isPumpRequired() {
         return (this->mSetting->pSensorDry != NULL) && (this->moistureRaw.getMedian() < this->mSetting->pSensorDry->get()); 
    }

    HomieInternals::SendingPromise& setProperty(const String& property) const {
        return mPlant->setProperty(property);
    }

    void init(void);
};

#endif

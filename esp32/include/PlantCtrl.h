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
#include <HomieNode.hpp>
#include "ControllerConfiguration.h"
#include "RunningMedian.h"
#include "MathUtils.h"

#define MOISTURE_MEASUREMENT_DURATION  500  /** ms */


class Plant
{

private:
    HomieNode *mPlant = NULL;
    HomieInternals::PropertyInterface mPump;
    int32_t mMoisture_freq = 0;
    int mPinSensor = 0; /**< Pin of the moist sensor */
    int mPinPump = 0;   /**< Pin of the pump */
    bool mConnected = false;
    int mPlantId = -1;

public:
    PlantSettings_t *mSetting;
    /**
     * @brief Construct a new Plant object
     * 
     * @param pinSensor Pin of the Sensor to use to measure moist
     * @param pinPump   Pin of the Pump to use
     */
    Plant(int pinSensor, int pinPump,
          int plantId,
          HomieNode *plant,
          PlantSettings_t *setting);

    void postMQTTconnection(void);

    void advertise(void);

    /**
     * @brief Measure a new analog moister value
     * 
     */
    void startMoistureMeasurement(void);
    void stopMoistureMeasurement(void);

    void deactivatePump(void);

    void activatePump(void);

    /**
     * @brief Check if a plant is too dry and needs some water.
     * 
     * @return true 
     * @return false 
     */
    bool isPumpRequired()
    {
        bool isDry = getCurrentMoisture() > getSetting2Moisture();
        bool isActive = isPumpTriggerActive();
        return isDry && isActive;
    }

    bool isPumpTriggerActive()
    {
        long current = this->mSetting->pSensorDry->get();
        return !equalish(current,DEACTIVATED_PLANT);
    }

    float getCurrentMoisture()
    {   
        if(mMoisture_freq < MOIST_SENSOR_MIN_FRQ){
            return MISSING_SENSOR;
        }
        return mMoisture_freq;
    }

    long getSetting2Moisture()
    {
        if (this->mSetting->pSensorDry != NULL)
        {
            float percent = (this->mSetting->pSensorDry->get());
            return (((MOIST_SENSOR_MAX_FRQ - MOIST_SENSOR_MIN_FRQ) * percent) + MOIST_SENSOR_MIN_FRQ);
        }
        else
        {
            return DEACTIVATED_PLANT;
        }
    }

    HomieInternals::SendingPromise &setProperty(const String &property) const
    {
        return mPlant->setProperty(property);
    }

    void init(void);

    long getCooldownInSeconds() {
        return this->mSetting->pPumpCooldownInHours->get()*60*60;
    }

    /**
     * @brief Get the Hours when pumping should start
     * 
     * @return hour 
     */
    int getHoursStart() {
        return this->mSetting->pPumpAllowedHourRangeStart->get();
    }

    /**
     * @brief Get the Hours when pumping should end
     * 
     * @return hour 
     */
    int getHoursEnd() {
        return this->mSetting->pPumpAllowedHourRangeEnd->get();
    }

    bool isAllowedOnlyAtLowLight(void)
    {
        return this->mSetting->pPumpOnlyWhenLowLight->get();
    }

    void publishState(String state);

    bool switchHandler(const HomieRange& range, const String& value);

    void setSwitchHandler(HomieInternals::PropertyInputHandler f);
};

#endif

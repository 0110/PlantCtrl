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

class Plant
{

private:
    RunningMedian moistureRaw = RunningMedian(5);
    HomieNode *mPlant = NULL;
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
    void addSenseValue(void);

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
        bool isDry = getCurrentMoisture() > getSettingsMoisture();
        bool isActive = isPumpTriggerActive();
        return isDry && isActive;
    }

    bool isPumpTriggerActive()
    {
        return this->mSetting->pSensorDry->get() != DEACTIVATED_PLANT;
    }

    float getCurrentMoisture()
    {   
        if(moistureRaw.getCount()==0){
            return MISSING_SENSOR;
        }
        return this->moistureRaw.getMedian();
    }
    long getSettingsMoisture()
    {
        if (this->mSetting->pSensorDry != NULL)
        {
            return this->mSetting->pSensorDry->get();
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
    bool switchHandler(const HomieRange &range, const String &value);

    void init(void);

    /** @fn  bool isInCooldown(long sinceLastActivation)
     *  @brief determine, if the plant was recently casted
     *  @param sinceLastActivation timestamp of last time
     */
    bool isInCooldown(long sinceLastActivation)
    {
        /* if the time difference is greater than one month, we know these are initial values */
        if (sinceLastActivation > (60 * 60 * 24 * 30))
        {
            return false;
        }

        return (getCooldownInSeconds() > sinceLastActivation);
    }

    long getCooldownInSeconds(){
        return this->mSetting->pPumpCooldownInHours->get()*60*60;
    }

    bool isAllowedOnlyAtLowLight(void)
    {
        return this->mSetting->pPumpOnlyWhenLowLight->get();
    }
};

#endif

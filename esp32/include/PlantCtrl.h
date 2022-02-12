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
#include "MQTTUtils.h"
#include "LogDefines.h"

#define ANALOG_REREADS 5
#define MOISTURE_MEASUREMENT_DURATION 400 /** ms */
#define PWM_FREQ 50000
#define PWM_BITS 8

class Plant
{

private:
    HomieNode *mPlant = NULL;
    HomieInternals::PropertyInterface mPump;
    RunningMedian mMoisture_raw = RunningMedian(ANALOG_REREADS);
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

    //for sensor that might take any time
    void blockingMoistureMeasurement(void);
    //for sensor that need a start and a end in defined timing
    void startMoistureMeasurement(void);
    void stopMoistureMeasurement(void);

    void deactivatePump(void);

    void activatePump(void);

    bool isHydroponic()
    {
        long current = this->mSetting->pSensorDry->get();
        return equalish(current, HYDROPONIC_MODE);
    }

    long isSensorMode(int sensorMode)
    {
        return this->mSetting->pSensorMode->get() == sensorMode;
    }

    /**
     * @brief Check if a plant is too dry and needs some water.
     * 
     * @return true 
     * @return false 
     */
    bool isPumpRequired()
    {
        if (isHydroponic())
        {
            //hydroponic only uses timer based controll
            return true;
        }
        bool isDry = getCurrentMoisturePCT() < getTargetMoisturePCT();
        bool isActive = isPumpTriggerActive();
        return isDry && isActive;
    }

    bool isPumpTriggerActive()
    {
        long current = this->mSetting->pSensorDry->get();
        return !equalish(current, DEACTIVATED_PLANT);
    }

    float getTargetMoisturePCT()
    {
        if (isPumpTriggerActive())
        {
            return this->mSetting->pSensorDry->get();
        }
        else
        {
            return DEACTIVATED_PLANT;
        }
    }

    float getCurrentMoisturePCT()
    {
        if (isSensorMode(SENSOR_NONE))
        {
            return DEACTIVATED_PLANT;
        }
        if (isSensorMode(SENSOR_CAPACITIVE_FREQUENCY_MOD))
        {
              return mapf(mMoisture_raw.getMedian(), MOIST_SENSOR_MAX_FRQ, MOIST_SENSOR_MIN_FRQ, 0, 100);
        }
        else if (isSensorMode(SENSOR_ANALOG_RESISTANCE_PROBE))
        {
            return mapf(mMoisture_raw.getMedian(), ANALOG_SENSOR_MAX_MV, ANALOG_SENSOR_MIN_MV, 0, 100);
        } else {
            log(LOG_LEVEL_ERROR, LOG_SENSORMODE_UNKNOWN, LOG_SENSORMODE_UNKNOWN_CODE);
            return DEACTIVATED_PLANT;
        }
    }

    float getCurrentMoistureRaw()
    {
        if (isSensorMode(SENSOR_CAPACITIVE_FREQUENCY_MOD))
        {
            if (mMoisture_raw.getMedian() < MOIST_SENSOR_MIN_FRQ)
            {
                return MISSING_SENSOR;
            }
        }

        return mMoisture_raw.getMedian();
    }

    HomieInternals::SendingPromise &setProperty(const String &property) const
    {
        return mPlant->setProperty(property);
    }

    void init(void);

    long getCooldownInSeconds()
    {
        return this->mSetting->pPumpCooldownInSeconds->get();
    }

    /**
     * @brief Get the Hours when pumping should start
     * 
     * @return hour 
     */
    int getHoursStart()
    {
        return this->mSetting->pPumpAllowedHourRangeStart->get();
    }

    /**
     * @brief Get the Hours when pumping should end
     * 
     * @return hour 
     */
    int getHoursEnd()
    {
        return this->mSetting->pPumpAllowedHourRangeEnd->get();
    }

    bool isAllowedOnlyAtLowLight(void)
    {
        if (this->isHydroponic())
        {
            return false;
        }
        return this->mSetting->pPumpOnlyWhenLowLight->get();
    }

    void publishState(String state);

    bool switchHandler(const HomieRange &range, const String &value);

    void setSwitchHandler(HomieInternals::PropertyInputHandler f);

    long getPumpDuration()
    {
        return this->mSetting->pPumpDuration->get();
    }
};

#endif

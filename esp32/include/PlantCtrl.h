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

class Plant {

private:
    int mPinSensor=0;   /**< Pin of the moist sensor */
    int mPinPump=0;     /**< Pin of the pump */

    int mValue = 0;     /**< Value of the moist sensor */

    int mAnalogValue=0; /**< moist sensor values, used for a calculation */

public:

    /**
     * @brief Construct a new Plant object
     * 
     * @param pinSensor Pin of the Sensor to use to measure moist
     * @param pinPump   Pin of the Pump to use
     */
    Plant(int pinSensor, int pinPump);

    /**
     * @brief Add a value, to be measured
     * 
     * @param analogValue 
     */
    void addSenseValue(int analogValue);
    
    /**
     * @brief Calculate the value based on the information
     * @see amountMeasurePoints
     * Internal memory, used by addSenseValue will be resetted
     * @return int  analog value
     */
    void calculateSensorValue(int amountMeasurePoints);

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

    int getSensorValue() { return mValue; }

    /**
     * @brief Check if a plant is too dry and needs some water.
     * 
     * @param boundary 
     * @return true 
     * @return false 
     */
    bool isPumpRequired(int boundary) { return (this->mValue < boundary); }
};

#endif

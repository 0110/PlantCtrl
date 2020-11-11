/**
 * @file DS18B20.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2020-06-09
 * 
 * @copyright Copyright (c) 2020
 * Based on the LUA code from the ESP8266
 * --------------------------------------------------------------------------------
 * -- DS18B20 one wire module for NODEMCU
 * -- NODEMCU TEAM
 * -- LICENCE: http://opensource.org/licenses/MIT
 * -- Vowstar <vowstar@nodemcu.com>
 * -- 2015/02/14 sza2 <sza2trash@gmail.com> Fix for negative values
 * --------------------------------------------------------------------------------
 */

#ifndef DS18B20_H
#define DS18B20_H

#include <OneWire.h>

class Ds18B20
{
private:
    OneWire *mDs;
    int foundDevices;

public:
    Ds18B20(int pin)
    {
        this->mDs = new OneWire(pin);
    }

    ~Ds18B20()
    {
        delete this->mDs;
    }
    /**
         * @brief read amount sensots
         * check for available of DS18B20 sensors
         * @return amount of sensors
         */
    int readDevices(void);

    /**
         * @brief Read all temperatures in celsius
         * 
         * @param pTemperatures     array of float valuies
         * @param maxTemperatures  size of the given array
         * @return int amount of read temperature values
         */
    int readAllTemperatures(float *pTemperatures, int maxTemperatures);
};
#endif

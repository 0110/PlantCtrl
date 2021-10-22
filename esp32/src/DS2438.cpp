/*
 *   DS2438.cpp
 *
 *   by Joe Bechter
 *
 *   (C) 2012, bechter.com
 *
 *   All files, software, schematics and designs are provided as-is with no warranty.
 *   All files, software, schematics and designs are for experimental/hobby use.
 *   Under no circumstances should any part be used for critical systems where safety,
 *   life or property depends upon it. You are responsible for all use.
 *   You are free to use, modify, derive or otherwise extend for your own non-commercial purposes provided
 *       1. No part of this software or design may be used to cause injury or death to humans or animals.
 *       2. Use is non-commercial.
 *       3. Credit is given to the author (i.e. portions © bechter.com), and provide a link to the original source.
 *
 */

#include "DS2438.h"

// DSROM FIELDS
#define DSROM_FAMILY    0
#define DSROM_CRC       7

#define DS2438MODEL     0x26

DS2438::DS2438(OneWire *ow, float currentShunt, int retryOnCRCError) {
    _ow = ow;
    _currentShunt = currentShunt;
    _retryOnCRCError = retryOnCRCError;
};

void DS2438::begin(){
    DeviceAddress searchDeviceAddress;

	_ow->reset_search();
    memset(searchDeviceAddress,0, 8);
    _temperature.clear();
    _voltageA.clear();
    _voltageB.clear();
    _error = true;
    _mode = (DS2438_MODE_CHA | DS2438_MODE_CHB | DS2438_MODE_TEMPERATURE);

	deviceFound = false; // Reset the number of devices when we enumerate wire devices

	while (_ow->search(searchDeviceAddress)) {
		if (validAddress(searchDeviceAddress)) {
			if (validFamily(searchDeviceAddress)) {
                memcpy(_address,searchDeviceAddress,8);
                DEFAULT_PAGE0(defaultConfig);
                writePage(0, defaultConfig);
                deviceFound = true;
			}
		}
	}
}

bool DS2438::isFound(){
    return deviceFound;
}

bool DS2438::validAddress(const uint8_t* deviceAddress) {
	return (_ow->crc8(deviceAddress, 7) == deviceAddress[DSROM_CRC]);
}

bool DS2438::validFamily(const uint8_t* deviceAddress) {
	switch (deviceAddress[DSROM_FAMILY]) {
	case DS2438MODEL:
		return true;
	default:
		return false;
	}
}

void DS2438::updateMultiple(){
    for(int i = 0;i< DS2438_MEDIAN_COUNT; i++){
        update(i==0);
        if(_error){
            return;
        }
        delay(DS2438_MEDIAN_DELAY);
    }
}

void DS2438::update(bool firstIteration) {
    uint8_t data[9];
    _error = true;
    
    if(!isFound()){
        return;
    }

    if (_mode & DS2438_MODE_CHA || _mode == DS2438_MODE_TEMPERATURE) {
        boolean doTemperature = _mode & DS2438_MODE_TEMPERATURE;
        if (!startConversion(DS2438_CHA, doTemperature)) {
            Serial.println("Error starting temp conversion ds2438 channel a");
            return;
        }
        if (!readPage(0, data)){
            
            Serial.println("Error reading zero page ds2438 channel a");
            return;
        }
            
        if (doTemperature) {
            _temperature.add((double)(((((int16_t)data[2]) << 8) | (data[1] & 0x0ff)) >> 3) * 0.03125);
        }
        if (_mode & DS2438_MODE_CHA) {
            _voltageA.add((((data[4] << 8) & 0x00300) | (data[3] & 0x0ff)) / 100.0);
        }
    }
    if (_mode & DS2438_MODE_CHB) {
        boolean doTemperature = _mode & DS2438_MODE_TEMPERATURE && !(_mode & DS2438_MODE_CHA);
        if (!startConversion(DS2438_CHB, doTemperature)) {
            Serial.println("Error starting temp conversion channel b ds2438");
            return;
        }
        if (!readPage(0, data)){
            Serial.println("Error reading zero page ds2438 channel b");
            return;
        }
        if (doTemperature) {
            int16_t upperByte = ((int16_t)data[2]) << 8;
            int16_t lowerByte = data[1] >> 3;
            int16_t fullByte = (upperByte | lowerByte);
            _temperature.add(((double)fullByte) * 0.03125);
        }
        _voltageB.add((((data[4] << 8) & 0x00300) | (data[3] & 0x0ff)) / 100.0);
    }
    
    int16_t upperByte = ((int16_t)data[6]) << 8;
    int16_t lowerByte = data[5];
    int16_t fullByte = (int16_t)(upperByte | lowerByte);
    float fullByteb = fullByte;
    _current.add((fullByteb) / ((4096.0f * _currentShunt)));


    if(firstIteration){
        if (readPage(1, data)){
            PageOne_t *pOne = (PageOne_t *) data;
            _ICA = pOne->ICA;
        }

        if (readPage(7, data)){
            PageSeven_t *pSeven = (PageSeven_t *) data;
            _CCA = pSeven->CCA0 | ((int16_t) pSeven->CCA1) << 8;
            _DCA = pSeven->DCA0 | ((int16_t) pSeven->DCA1) << 8;
        }
    }
    _error = false;
}

double DS2438::getTemperature() {
    return _temperature.getMedian();
}

float DS2438::getAh(){
    return _ICA / (2048.0f * _currentShunt);
}

long DS2438::getICA(){
    return _ICA;
}

long DS2438::getDCA(){
    return _DCA;
}

long DS2438::getCCA(){
    return _CCA;
}


float DS2438::getVoltage(int channel) {
    if (channel == DS2438_CHA) {
        return _voltageA.getMedian();
    } else if (channel == DS2438_CHB) {
        return _voltageB.getMedian();
    } else {
        return 0.0;
    }
}

float DS2438::getCurrent() {
    return _current.getMedian();
}

boolean DS2438::isError() {
    return _error;
}

boolean DS2438::startConversion(int channel, boolean doTemperature) {
    if(!isFound()){
        return false;
    }
    if (!selectChannel(channel)){
        return false;
    }
    _ow->reset();
    _ow->select(_address);
    if (doTemperature) {
        _ow->write(DS2438_TEMPERATURE_CONVERSION_COMMAND, 0);
        delay(DS2438_TEMPERATURE_DELAY);
        _ow->reset();
        _ow->select(_address);
    }
    _ow->write(DS2438_VOLTAGE_CONVERSION_COMMAND, 0);
    delay(DS2438_VOLTAGE_CONVERSION_DELAY);
    return true;
}

boolean DS2438::selectChannel(int channel) {
    if(!isFound()){
        return false;
    }
    uint8_t data[9];
    if (readPage(0, data)) {
        if (channel == DS2438_CHB){
            data[0] = data[0] | 0x08;
        }
        else {
            data[0] = data[0] & 0xf7;
        }
        writePage(0, data);
        return true;
    }
    Serial.println("Could not read page zero data");
    return false;
}

void DS2438::writePage(int page, uint8_t *data) {
    _ow->reset();
    _ow->select(_address);
    _ow->write(DS2438_WRITE_SCRATCHPAD_COMMAND, 0);
    if ((page >= PAGE_MIN) && (page <= PAGE_MAX)) {
        _ow->write(page, 0);
    } else {
        return;
    }
    for (int i = 0; i < 8; i++){
        _ow->write(data[i], 0);
    }
    _ow->reset();
    _ow->select(_address);
    _ow->write(DS2438_COPY_SCRATCHPAD_COMMAND, 0);
    _ow->write(page, 0);
}

boolean DS2438::readPage(int page, uint8_t *data) {
    bool valid = false;
    for(int retry = 0;retry < this->_retryOnCRCError && !valid; retry ++){
        //TODO if all data is 0 0 is a valid crc, but most likly not as intended
        _ow->reset();
        _ow->select(_address);
        _ow->write(DS2438_RECALL_MEMORY_COMMAND, 0);
        if ((page >= PAGE_MIN) && (page <= PAGE_MAX)) {
            _ow->write(page, 0);
        } else {
            return false;
        }
        _ow->reset();
        _ow->select(_address);
        _ow->write(DS2438_READ_SCRATCHPAD_COMMAND, 0);
        _ow->write(page, 0);
        for (int i = 0; i < 9; i++){
            data[i] = _ow->read();
        }
        valid = _ow->crc8(data, 8) == data[8];
    }
    return valid;
}


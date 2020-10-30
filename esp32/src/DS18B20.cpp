/**
 * @file DS18B20.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2020-06-09
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#include "DS18B20.h"

#define STARTCONV       0x44
#define READSCRATCH     0xBE  // Read EEPROM
#define TEMP_LSB        0
#define TEMP_MSB        1
#define SCRATCHPADSIZE  9
#define OFFSET_CRC8     8       /**< 9th byte has the CRC of the complete data */

//Printf debugging
//#define DS_DEBUG

int Ds18B20::readDevices() {
    byte addr[8];

    int amount = -1;
    while (this->mDs->search(addr)) {
        amount++;
    }
    this->mDs->reset_search();
    return amount;
}

int Ds18B20::readAllTemperatures(float* pTemperatures, int maxTemperatures) {
    byte addr[8];
    uint8_t scratchPad[SCRATCHPADSIZE];
    int currentTemp = 0;
#ifdef DS_DEBUG
    int i;
#endif

    while (this->mDs->search(addr)) {
#ifdef DS_DEBUG
        Serial.print(" ROM =");
        for (i = 0; i < 8; i++) {
            Serial.write(' ');
            Serial.print(addr[i], HEX);
        }
#endif
        this->mDs->reset();
        this->mDs->select(addr);
        this->mDs->write(STARTCONV);
        this->mDs->reset();
        this->mDs->select(addr);
        this->mDs->write(READSCRATCH);

        // Read all registers in a simple loop
        // byte 0: temperature LSB
        // byte 1: temperature MSB
        // byte 2: high alarm temp
        // byte 3: low alarm temp
        // byte 4: DS18S20: store for crc
        //         DS18B20 & DS1822: configuration register
        // byte 5: internal use & crc
        // byte 6: DS18S20: COUNT_REMAIN
        //         DS18B20 & DS1822: store for crc
        // byte 7: DS18S20: COUNT_PER_C
        //         DS18B20 & DS1822: store for crc
        // byte 8: SCRATCHPAD_CRC
#ifdef DS_DEBUG
        Serial.write("\r\nDATA:");
        for (uint8_t i = 0; i < 9; i++) {
            Serial.print(scratchPad[i], HEX);
        }
#else
        delay(50);
#endif
        for (uint8_t i = 0; i < 9; i++) {
            scratchPad[i] = this->mDs->read();
        }
        uint8_t crc8 = this->mDs->crc8(scratchPad, 8);

        /* Only work an valid data */
        if (crc8 == scratchPad[OFFSET_CRC8]) {
            int16_t fpTemperature = (((int16_t) scratchPad[TEMP_MSB]) << 11)
                | (((int16_t) scratchPad[TEMP_LSB]) << 3);
            float celsius =  (float) fpTemperature * 0.0078125;
#ifdef DS_DEBUG
            Serial.printf("\r\nTemp%d %f °C (Raw: %d, %x =? %x)\r\n", (currentTemp+1), celsius, fpTemperature, crc8, scratchPad[8]);
#endif
            /* check, if the buffer as some space for our data */
            if (currentTemp < maxTemperatures) {
                pTemperatures[currentTemp] = celsius;
            } else {
                return -1;
            }
        }
        currentTemp++;
    } 
    this->mDs->reset();
#ifdef DS_DEBUG
    Serial.println(" No more addresses.");
    Serial.println();
#endif

    return currentTemp;
}
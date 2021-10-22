/**
 *  @file DS2438.h
 *
 *
 */

#ifndef DS2438_h
#define DS2438_h

#include <Arduino.h>
#include <OneWire.h>
#include "RunningMedian.h"

#define DS2438_TEMPERATURE_CONVERSION_COMMAND 0x44
#define DS2438_VOLTAGE_CONVERSION_COMMAND 0xb4
#define DS2438_WRITE_SCRATCHPAD_COMMAND 0x4e
#define DS2438_COPY_SCRATCHPAD_COMMAND 0x48
#define DS2438_READ_SCRATCHPAD_COMMAND 0xbe
#define DS2438_RECALL_MEMORY_COMMAND 0xb8

#define PAGE_MIN 0
#define PAGE_MAX 7

#define DS2438_CHA 0
#define DS2438_CHB 1

#define DS2438_MODE_CHA 0x01
#define DS2438_MODE_CHB 0x02
#define DS2438_MODE_TEMPERATURE 0x04

#define DS2438_TEMPERATURE_DELAY 10
#define DS2438_VOLTAGE_CONVERSION_DELAY 8

#define DS2438_MEDIAN_COUNT 5
#define DS2438_MEDIAN_DELAY 50

#define DEFAULT_PAGE0(var) uint8_t var[8] { \
    0b00001011 /* X, ADB=0, NVB=0, TB=0, AD=1, EE=0, CA=1, IAD=1 */, \
    0, /* Temperatur */ \
    0, /* Temperatur */ \
    0, /* Voltage */ \
    0, /* Voltage */ \
    0, /* Current */ \
    0, /* Current */ \
    0b10000000 /* Threshold to 4LSB */ \
}

typedef struct PageOne {
    uint8_t eleapsedTimerByte0; /**< LSB of timestamp */
    uint8_t eleapsedTimerByte1;
    uint8_t eleapsedTimerByte2;
    uint8_t eleapsedTimerByte3; /**< MSB of timestamp */
    uint8_t ICA; /**< Integrated Current Accumulator (current flowing into and out of the battery) */
    uint8_t offsetRegisterByte0; /**< Offset for ADC calibdation */
    uint8_t offsetRegisterByte1; /**< Offset for ADC calibdation */
    uint8_t reserved;
} PageOne_t;

typedef struct PageSeven {
    uint8_t userByte0;
    uint8_t userByte1;
    uint8_t userByte2;
    uint8_t userByte3;
    uint8_t CCA0;   /**< Charging Current Accumulator (CCA) */
    uint8_t CCA1;   /**< Charging Current Accumulator (CCA) */
    uint8_t DCA0;   /**< Discharge Current Accumulator (DCA) */
    uint8_t DCA1;   /**< Discharge Current Accumulator (DCA) */
} PageSeven_t;

typedef uint8_t DeviceAddress[8];

class DS2438 {
    public:
        DS2438(OneWire *ow, float currentShunt, int retryOnCRCError);
    
        void begin();
        void updateMultiple();
        double getTemperature();
        float getVoltage(int channel=DS2438_CHA);
        float getCurrent();
        long getICA();
        long getCCA();
        long getDCA();
        float getAh();
        boolean isError();
        boolean isFound();
    private:
        bool validAddress(const uint8_t*);
        bool validFamily(const uint8_t* deviceAddress);
        void update(bool firstIteration);
        bool deviceFound = false;
        OneWire *_ow;
        DeviceAddress _address;
        uint8_t _mode;
        RunningMedian _temperature = RunningMedian(DS2438_MEDIAN_COUNT*2);
        RunningMedian _voltageA = RunningMedian(DS2438_MEDIAN_COUNT);
        RunningMedian _voltageB = RunningMedian(DS2438_MEDIAN_COUNT);
        RunningMedian _current = RunningMedian(DS2438_MEDIAN_COUNT);
        float _currentShunt;
        int _retryOnCRCError;
        long _CCA;
        long _DCA;
        long _ICA;
        boolean _error;
        boolean startConversion(int channel, boolean doTemperature);
        boolean selectChannel(int channel);
        void writePage(int page, uint8_t *data);
        boolean readPage(int page, uint8_t *data);
};

#endif
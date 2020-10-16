/**
 * @file main.cpp
 * @author Ollo
 * @brief PlantControl
 * @version 0.1
 * @date 2020-05-01
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#include "PlantCtrl.h"
#include "ControllerConfiguration.h"
#include "HomieConfiguration.h"
#include "DS18B20.h"
#include <Homie.h>
#include "esp_sleep.h"
#include "RunningMedian.h"

const unsigned long TEMPREADCYCLE = 30000; /**< Check temperature all half minutes */

#define AMOUNT_SENOR_QUERYS   8
#define SENSOR_QUERY_SHIFTS   3
#define SOLAR4SENSORS         6.0f
#define TEMP_INIT_VALUE       -999.0f
#define TEMP_MAX_VALUE        85.0f

RTC_DATA_ATTR bool coldBoot = false;

bool mLoopInited = false;
bool mDeepSleep = false;
bool mAlive=false;        /**< Controller must not sleep */

int plantSensor1 = 0;

int lipoSenor = -1;
int lipoSensorValues = 0;
int solarSensor = -1;
int solarSensorValues = 0;

int mWaterGone = -1;  /**< Amount of centimeter, where no water is seen */
int readCounter = 0;
int mButtonClicks = 0;
bool mConfigured = false;

RTC_DATA_ATTR int gBootCount = 0;
RTC_DATA_ATTR int gCurrentPlant = 0; /**< Value Range: 1 ... 7 (0: no plant needs water) */

RunningMedian lipoRawSensor = RunningMedian(5);
RunningMedian solarRawSensor = RunningMedian(5);
RunningMedian waterRawSensor = RunningMedian(5);
RunningMedian temp1 = RunningMedian(5);
RunningMedian temp2 = RunningMedian(5);


Ds18B20 dallas(SENSOR_DS18B20);

Plant mPlants[MAX_PLANTS] = { 
        Plant(SENSOR_PLANT0, OUTPUT_PUMP0, 0, &plant0, &mSetting0), 
        Plant(SENSOR_PLANT1, OUTPUT_PUMP1, 1, &plant1, &mSetting1), 
        Plant(SENSOR_PLANT2, OUTPUT_PUMP2, 2, &plant2, &mSetting2), 
        Plant(SENSOR_PLANT3, OUTPUT_PUMP3, 3, &plant3, &mSetting3), 
        Plant(SENSOR_PLANT4, OUTPUT_PUMP4, 4, &plant4, &mSetting4), 
        Plant(SENSOR_PLANT5, OUTPUT_PUMP5, 5, &plant5, &mSetting5), 
        Plant(SENSOR_PLANT6, OUTPUT_PUMP6, 6, &plant6, &mSetting6) 
      };

void readSystemSensors() {
  lipoRawSensor.add(analogRead(SENSOR_LIPO));
  solarRawSensor.add(analogRead(SENSOR_SOLAR));

  
}

/**
 * @brief Sensors, that are connected to GPIOs, mandatory for WIFI.
 * These sensors (ADC2) can only be read when no Wifi is used.
 */
void readSensors() {
  Serial << "Read sensors..." << endl;

  /* activate all sensors */
  pinMode(OUTPUT_SENSOR, OUTPUT);
  digitalWrite(OUTPUT_SENSOR, HIGH);

  delay(100);
  /* wait before reading something */
  for (int readCnt=0;readCnt < AMOUNT_SENOR_QUERYS; readCnt++) {
    for(int i=0; i < MAX_PLANTS; i++) {
      mPlants[i].addSenseValue(analogRead(mPlants[i].getSensorPin()));
    }
  }

  Serial << "DS18B20 | Initialization " << endl;
  /* Read the temperature sensors once, as first time 85 degree is returned */
  Serial << "DS18B20 | sensors: " << String(dallas.readDevices()) << endl;
  delay(200);


  /* Required to read the temperature once */
  float temp[2] = {0, 0};
  float* pFloat = temp;
  // first read returns crap, ignore result and read twice
  if (dallas.readAllTemperatures(pFloat, 2) > 0) {
      Serial << "DS18B20 | Temperature 1: " << String(temp[0]) << endl;
      Serial << "DS18B20 | Temperature 2: " << String(temp[1]) << endl;
  }
  delay(200);
  if (dallas.readAllTemperatures(pFloat, 2) > 0) {
      Serial << "Temperature 1: " << String(temp[0]) << endl;
      Serial << "Temperature 2: " << String(temp[1]) << endl;
  }

  temp1.add(temp[0]);
  temp2.add(temp[1]);

  /* Use the Ultrasonic sensor to measure waterLevel */
 
  digitalWrite(SENSOR_SR04_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(SENSOR_SR04_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(SENSOR_SR04_TRIG, LOW);
  float duration = pulseIn(SENSOR_SR04_ECHO, HIGH);
  waterRawSensor.add((duration*.343)/2);
  /* deactivate the sensors */
  digitalWrite(OUTPUT_SENSOR, LOW);
}


/**
 * @brief cyclic Homie callback
 * All logic, to be done by the controller cyclically
 */
void loopHandler() {

  /* Move from Setup to this position, because the Settings are only here available */
  if (!mLoopInited) {
    // Configure Deep Sleep:
    if (deepSleepTime.get()) {
      Serial << "HOMIE | Setup sleeping for " << deepSleepTime.get() << " ms" << endl;
    }
    /* Publish default values */
    plant0.setProperty("switch").send(String("OFF"));            
    plant1.setProperty("switch").send(String("OFF"));            
    plant2.setProperty("switch").send(String("OFF"));         
    
#if (MAX_PLANTS >= 4)
    plant3.setProperty("switch").send(String("OFF"));            
    plant4.setProperty("switch").send(String("OFF"));
    plant5.setProperty("switch").send(String("OFF"));
#endif   
#if (MAX_PLANTS >= 7)
    plant6.setProperty("switch").send(String("OFF"));
#endif   

    for(int i=0; i < MAX_PLANTS; i++) {
      mPlants[i].calculateSensorValue(AMOUNT_SENOR_QUERYS);
        mPlants[i].setProperty("moist").send(String(100 * mPlants[i].getSensorValue() / 4095 ));
      /* the last Plant, that was watered is stored in non volatile memory */
      if (gCurrentPlant <= 0 && mPlants[i].isPumpRequired()) {
          /* there was no plant activated -> we can store the first one */
          gCurrentPlant = i + 1;
      } else if (gCurrentPlant > 0 && gCurrentPlant < (i+1) && 
                  mPlants[(gCurrentPlant - 1)].isPumpRequired() == false) {
          /* The last does not need any more some water -> jump to the next one */
          gCurrentPlant = i + 1;
      }
    }

    sensorWater.setProperty("remaining").send(String(waterLevelMax.get() - mWaterGone ));
    Serial << "Water : " << mWaterGone << " cm (" << String(waterLevelMax.get() - mWaterGone ) << "%)" << endl;

    /* Check if a plant needs water */
    if (gCurrentPlant > 0) {
      int plntIdx = (gCurrentPlant-1);
      if (mPlants[plntIdx].isPumpRequired() && 
          (mWaterGone > waterLevelMin.get()) &&
          (digitalRead(mPlants[plntIdx].getPumpPin()) == LOW) ) {
          Serial << "Plant" << plntIdx << " needs water" << endl;
          mPlants[plntIdx].setProperty("switch").send(String("ON"));
        }
        digitalWrite(OUTPUT_PUMP, HIGH);
        digitalWrite(mPlants[plntIdx].getPumpPin(), HIGH);
      }
  }
  mLoopInited = true;

  readSensors();

  if ((millis() % 1500) == 0) {
    sensorLipo.setProperty("percent").send( String(100 * lipoSenor / 4095) );
    sensorLipo.setProperty("volt").send( String(ADC_5V_TO_3V3(lipoSenor)) );
    sensorSolar.setProperty("percent").send(String((100 * solarSensor ) / 4095));
    sensorSolar.setProperty("volt").send( String(SOLAR_VOLT(solarSensor)) );
  } else if ((millis() % 1000) == 0) {
    float temp[2] = { TEMP_INIT_VALUE, TEMP_INIT_VALUE };
    float* pFloat = temp;
    int devices = dallas.readAllTemperatures(pFloat, 2);
    if (devices < 2) {
      if ((pFloat[0] > TEMP_INIT_VALUE) && (pFloat[0] < TEMP_MAX_VALUE) ) {
        sensorTemp.setProperty("control").send( String(pFloat[0]));
      }
    } else if (devices >= 2) {      
      if ((pFloat[0] > TEMP_INIT_VALUE) && (pFloat[0] < TEMP_MAX_VALUE) ) {
        sensorTemp.setProperty("temp").send( String(pFloat[0]));
      }
      if ((pFloat[1] > TEMP_INIT_VALUE) && (pFloat[1] < TEMP_MAX_VALUE) ) {
        sensorTemp.setProperty("control").send( String(pFloat[1]));
      }
    }
  }

  /* Main Loop functionality */
  if (mWaterGone <= waterLevelMin.get()) {
      /* let the ESP sleep qickly, as nothing must be done */
      if ((millis() >= (MIN_TIME_RUNNING * MS_TO_S)) && (deepSleepTime.get() > 0)) {
        mDeepSleep = true;
        Serial << "No Water for pumps" << endl;
      }
  }

  /* Always check, that after 5 minutes the device is sleeping */
  /* Pump is running, go to sleep after defined time */
  if (millis() >= ((MIN_TIME_RUNNING + 5) && 
            (deepSleepTime.get() > 0))) {
    mDeepSleep = true;
  } else if ((millis() >= ((MIN_TIME_RUNNING * MS_TO_S) + 0)) &&
      (deepSleepTime.get() > 0)) {
    Serial << "Maximum time reached: " << endl;
    mDeepSleep = true;
  }
}

bool switchGeneralPumpHandler(const int pump, const HomieRange& range, const String& value) {
  if (range.isRange) return false;  // only one switch is present
  switch (pump)
  {
#if MAX_PLANTS >= 1
  case 0:
#endif
#if MAX_PLANTS >= 2
  case 1:
#endif
  #if MAX_PLANTS >= 3
#endif
  case 2:
#if MAX_PLANTS >= 4
  case 3:
#endif
#if MAX_PLANTS >= 5
  case 4:
#endif
#if MAX_PLANTS >= 6
  case 5:
#endif

    if ((value.equals("ON")) || (value.equals("On")) || (value.equals("on")) || (value.equals("true"))) {
      digitalWrite(mPlants[pump].getPumpPin(), HIGH);
      return true;
    } else if ((value.equals("OFF")) || (value.equals("Off")) || (value.equals("off")) || (value.equals("false")) ) {
      digitalWrite(mPlants[pump].getPumpPin(), LOW);
      return true;
    } else {
      return false;
    }
    break;
  default:
    return false;
  }
}

/**
 * @brief Handle Mqtt commands to keep controller alive
 * 
 * @param range multiple transmitted values (not used for this function)
 * @param value single value
 * @return true when the command was parsed and executed succuessfully
 * @return false on errors when parsing the request
 */
bool aliveHandler(const HomieRange& range, const String& value) {
  if (range.isRange) return false;  // only one controller is present

  if (value.equals("ON") || value.equals("On") || value.equals("1")) {
      mAlive=true;
  } else {
      mAlive=false;
  }
  Serial << "HOMIE  | Controller " << (mAlive ? " has coffee" : " is tired") << endl;
  return true;
}

/**
 * @brief Handle Mqtt commands for the pumpe, responsible for the first plant
 * 
 * @param range multiple transmitted values (not used for this function)
 * @param value single value
 * @return true when the command was parsed and executed succuessfully
 * @return false on errors when parsing the request
 */
bool switch1Handler(const HomieRange& range, const String& value) {
  return switchGeneralPumpHandler(0, range, value);
}


/**
 * @brief Handle Mqtt commands for the pumpe, responsible for the second plant
 * 
 * @param range multiple transmitted values (not used for this function)
 * @param value single value
 * @return true when the command was parsed and executed succuessfully
 * @return false on errors when parsing the request
 */
bool switch2Handler(const HomieRange& range, const String& value) {
  return switchGeneralPumpHandler(1, range, value);
}

/**
 * @brief Handle Mqtt commands for the pumpe, responsible for the third plant
 * 
 * @param range multiple transmitted values (not used for this function)
 * @param value single value
 * @return true when the command was parsed and executed succuessfully
 * @return false on errors when parsing the request
 */
bool switch3Handler(const HomieRange& range, const String& value) {
  return switchGeneralPumpHandler(2, range, value);
}



void systemInit(){
    // Set default values
  deepSleepTime.setDefaultValue(300000);    /* 5 minutes in milliseconds */
  deepSleepNightTime.setDefaultValue(0);
  wateringDeepSleep.setDefaultValue(60000); /* 1 minute in milliseconds */

  waterLevelMax.setDefaultValue(1000);    /* 100cm in mm */
  waterLevelMin.setDefaultValue(50);      /* 5cm in mm */
  waterLevelWarn.setDefaultValue(500);    /* 50cm in mm */
  waterLevelVol.setDefaultValue(5000);    /* 5l in ml */

  if (mConfigured) {
    // Advertise topics
    plant1.advertise("switch").setName("Pump 1")
                              .setDatatype("boolean")
                              .settable(switch1Handler);
    plant1.advertise("moist").setName("Percent")
                              .setDatatype("number")
                              .setUnit("%");
    plant2.advertise("switch").setName("Pump 2")
                              .setDatatype("boolean")
                              .settable(switch2Handler);
    plant2.advertise("moist").setName("Percent")
                              .setDatatype("number")
                              .setUnit("%");
    plant3.advertise("switch").setName("Pump 3")
                              .setDatatype("boolean")
                              .settable(switch3Handler);
    plant3.advertise("moist").setName("Percent")
                              .setDatatype("number")
                              .setUnit("%");
  #if (MAX_PLANTS >= 4)
    plant4.advertise("moist").setName("Percent")
                              .setDatatype("number")
                              .setUnit("%");
    plant5.advertise("moist").setName("Percent")
                              .setDatatype("number")
                              .setUnit("%");
    plant6.advertise("moist").setName("Percent")
                              .setDatatype("number")
                              .setUnit("%");
    plant0.advertise("moist").setName("Percent")
                              .setDatatype("number")
                              .setUnit("%");

  #endif
    sensorTemp.advertise("control")
                .setName("Temperature")
                .setDatatype("number")
                .setUnit("°C");
    sensorTemp.advertise("temp")
                .setName("Temperature")
                .setDatatype("number")
                .setUnit("°C");

    sensorLipo.advertise("percent")
                .setName("Percent")
                .setDatatype("number")
                .setUnit("%");
    sensorLipo.advertise("volt")
                .setName("Volt")
                .setDatatype("number")
                .setUnit("V");

    sensorSolar.advertise("percent")
                .setName("Percent")
                .setDatatype("number")
                .setUnit("%");
    sensorSolar.advertise("volt")
                .setName("Volt")
                .setDatatype("number")
                .setUnit("V");
    sensorWater.advertise("remaining").setDatatype("number").setUnit("%");

    // Mode 3
    stayAlive.advertise("alive").setName("Alive").setDatatype("number").settable(aliveHandler);
  }
}


void mode1(){
  readSensors();
}

void mode2(){
  WiFi.mode(WIFI_STA);
  Homie.setup();
}

void mode3(){
  WiFi.mode(WIFI_STA);
  Homie.setup();
}

/**
 * @brief Startup function
 * Is called once, the controller is started
 */
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(1000); // Set timeout of 1 second
  Serial << endl << endl;
  /* Intialize Plant */
  for(int i=0; i < MAX_PLANTS; i++) {
    mPlants[i].init();
  }

  /* Intialize inputs and outputs */
  pinMode(SENSOR_LIPO, ANALOG);
  pinMode(SENSOR_SOLAR, ANALOG);
  for(int i=0; i < MAX_PLANTS; i++) {
    pinMode(mPlants[i].getPumpPin(), OUTPUT);
    pinMode(mPlants[i].getSensorPin(), ANALOG);
    digitalWrite(mPlants[i].getPumpPin(), LOW);
  }
  /* read button */
  pinMode(BUTTON, INPUT);
  if(!coldBoot){
    digitalWrite(OUTPUT_SENSOR, HIGH);
  }
  
  
  
  /* Disable Wifi and bluetooth */
  WiFi.mode(WIFI_OFF);

  if (HomieInternals::MAX_CONFIG_SETTING_SIZE < MAX_CONFIG_SETTING_ITEMS) {
    Serial << "HOMIE | Settings: " << HomieInternals::MAX_CONFIG_SETTING_SIZE << "/" << MAX_CONFIG_SETTING_ITEMS << endl;
    Serial << "      | Update Limits.hpp : MAX_CONFIG_SETTING_SIZE to " << MAX_CONFIG_SETTING_ITEMS << endl;
    Serial << "      | Update Limits.hpp : MAX_JSON_CONFIG_FILE_SIZE to 5000" << endl;
  }

  Homie_setFirmware("PlantControl", FIRMWARE_VERSION);
  Homie.setLoopFunction(loopHandler);

  mConfigured = Homie.isConfigured();
  systemInit();

  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_ON);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_XTAL,ESP_PD_OPTION_ON);


  mode2();

  // Configure Deep Sleep:
  if (mConfigured && (deepSleepNightTime.get() > 0) &&
      ( SOLAR_VOLT(solarSensor) < MINIMUM_SOLAR_VOLT)) {
    Serial << "HOMIE | Setup sleeping for " << deepSleepNightTime.get() << " ms as sun is at " << SOLAR_VOLT(solarSensor) << "V"  << endl;
    uint64_t usSleepTime = deepSleepNightTime.get() * 1000U;
    esp_sleep_enable_timer_wakeup(usSleepTime);
  }else if (mConfigured && deepSleepTime.get()) {
    Serial << "HOMIE | Setup sleeping for " << deepSleepTime.get() << " ms" << endl;
    uint64_t usSleepTime = deepSleepTime.get() * 1000U;
    esp_sleep_enable_timer_wakeup(usSleepTime);
  }

  if (mConfigured && 
    (ADC_5V_TO_3V3(lipoSenor) < MINIMUM_LIPO_VOLT) && 
    (ADC_5V_TO_3V3(lipoSenor) > NO_LIPO_VOLT) &&
    (deepSleepTime.get()) ) {
    long sleepEmptyLipo = (deepSleepTime.get() * EMPTY_LIPO_MULTIPL);
    Serial << "HOMIE | Change sleeping to " << sleepEmptyLipo << " ms as lipo is at " << ADC_5V_TO_3V3(lipoSenor) << "V" << endl;
    esp_sleep_enable_timer_wakeup(sleepEmptyLipo * 1000U);
    mDeepSleep = true;
  }
}

/**
 * @brief Cyclic call
 * Executs the Homie base functionallity or triggers sleeping, if requested.
 */

void loop() {
  if(!coldBoot){
    coldBoot = true;
    delay(1000);
    digitalWrite(OUTPUT_SENSOR, LOW);
  }
  if (!mDeepSleep || !mConfigured) {
    if ((digitalRead(BUTTON) == LOW) && (mButtonClicks % 2) == 0) {
      mButtonClicks++;

      switch(mButtonClicks) {
        case 1:
        case 3:
        case 5:
          mode3();
          break;
        default:
            Serial << "No further tests! Please reboot" << endl;
      }
      Serial.flush();
    }else if (mButtonClicks > 0 && (digitalRead(BUTTON) == HIGH) && (mButtonClicks % 2) == 1) {
      Serial << "Self Test Ended" << endl;
      mButtonClicks++;
      /* Always reset all outputs */
      digitalWrite(OUTPUT_SENSOR, LOW);
      for(int i=0; i < MAX_PLANTS; i++) {
        digitalWrite(mPlants[i].getPumpPin(), LOW);
      }
      digitalWrite(OUTPUT_PUMP4, LOW);
    } else if (mButtonClicks == 0) {
      Homie.loop();
    }

  } else {
    if (!mAlive) {
      Serial << (millis()/ 1000) << "s running; sleeeping ..." << endl;
      Serial.flush();
      esp_deep_sleep_start();
    } else {
      mDeepSleep = false;

      if (((millis()) % 10000) == 0) {
        /* tell everybody how long we are awoken */
        stayAlive.setProperty("alive").send( String(millis()/ 1000) );
      }
    }
  }
}

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
#include "DS18B20.h"
#include <Homie.h>
#include "esp_sleep.h"


const unsigned long TEMPREADCYCLE = 30000; /**< Check temperature all half minutes */

#define AMOUNT_SENOR_QUERYS   8
#define SENSOR_QUERY_SHIFTS   3
#define SOLAR4SENSORS         6.0f
#define TEMP_INIT_VALUE       -999.0f
#define TEMP_MAX_VALUE        85.0f

bool mLoopInited = false;
bool mDeepSleep = false;

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

#if (MAX_PLANTS >= 1)
HomieNode plant0("plant0", "Plant 0", "Plant");
#endif
#if (MAX_PLANTS >= 2)
HomieNode plant1("plant1", "Plant 1", "Plant");
#endif
#if (MAX_PLANTS >= 3)
HomieNode plant2("plant2", "Plant 2", "Plant");
#endif
#if (MAX_PLANTS >= 4)
HomieNode plant3("plant3", "Plant 3", "Plant");
#endif
#if (MAX_PLANTS >= 5)
HomieNode plant4("plant4", "Plant 4", "Plant");
#endif
#if (MAX_PLANTS >= 6)
HomieNode plant5("plant5", "Plant 5", "Plant");
#endif
#if (MAX_PLANTS >= 7)
HomieNode plant6("plant6", "Plant 6", "Plant");
#endif

HomieNode sensorLipo("lipo", "Battery Status", "Lipo");
HomieNode sensorSolar("solar", "Solar Status", "Solarpanel");
HomieNode sensorWater("water", "WaterSensor", "Water");
HomieNode sensorTemp("temperature", "Temperature", "temperature");

HomieSetting<long> deepSleepTime("deepsleep", "time in milliseconds to sleep (0 deactivats it)");
HomieSetting<long> deepSleepNightTime("nightsleep", "time in milliseconds to sleep (0 usese same setting: deepsleep at night, too)");
HomieSetting<long> wateringDeepSleep("pumpdeepsleep", "time seconds to sleep, while a pump is running");
HomieSetting<long> plantCnt("plants", "amout of plants to control (1 ... 7)");

#ifdef  HC_SR04
HomieSetting<long> waterLevel("watermaxlevel", "Water maximum level in centimeter (50 cm default)");
HomieSetting<long> waterMinPercent("watermin", "Minimum percentage of water, to activate the pumps (default 5%)");
#endif
HomieSetting<long> plant0SensorTrigger("moist0", "Moist0 sensor value, when pump activates");
HomieSetting<long> plant1SensorTrigger("moist1", "Moist1 sensor value, when pump activates");
HomieSetting<long> plant2SensorTrigger("moist2", "Moist2 sensor value, when pump activates");
HomieSetting<long> plant3SensorTrigger("moist3", "Moist3 sensor value, when pump activates");
HomieSetting<long> plant4SensorTrigger("moist4", "Moist4 sensor value, when pump activates");
HomieSetting<long> plant5SensorTrigger("moist5", "Moist5 sensor value, when pump activates");
HomieSetting<long> plant6SensorTrigger("moist6", "Moist6 sensor value, when pump activates");
HomieSetting<long> wateringTime0("plant0MaxPumpTime", "time seconds Pump0 is running (60 is the default)");
HomieSetting<long> wateringTime1("plant1MaxPumpTime", "time seconds Pump1 is running (60 is the default)");
HomieSetting<long> wateringTime2("plant2MaxPumpTime", "time seconds Pump2 is running (60 is the default)");
HomieSetting<long> wateringTime3("plant3MaxPumpTime", "time seconds Pump3 is running (60 is the default)");
HomieSetting<long> wateringTime4("plant4MaxPumpTime", "time seconds Pump4 is running (60 is the default)");
HomieSetting<long> wateringTime5("plant5MaxPumpTime", "time seconds Pump5 is running (60 is the default)");
HomieSetting<long> wateringTime6("plant6MaxPumpTime", "time seconds Pump6 is running (60 is the default)");
HomieSetting<long> wateringIdleTime0("plant0MinPumpIdle", "time in seconds Pump0 will wait (60 is the default)");
HomieSetting<long> wateringIdleTime1("plant1MinPumpIdle", "time in seconds Pump1 will wait (60 is the default)");
HomieSetting<long> wateringIdleTime2("plant2MinPumpIdle", "time in seconds Pump2 will wait (60 is the default)");
HomieSetting<long> wateringIdleTime3("plant3MinPumpIdle", "time in seconds Pump3 will wait (60 is the default)");
HomieSetting<long> wateringIdleTime4("plant4MinPumpIdle", "time in seconds Pump4 will wait (60 is the default)");
HomieSetting<long> wateringIdleTime5("plant5MinPumpIdle", "time in seconds Pump5 will wait (60 is the default)");
HomieSetting<long> wateringIdleTime6("plant6MinPumpIdle", "time in seconds Pump6 will wait (60 is the default)");

Ds18B20 dallas(SENSOR_DS18B20);

Plant mPlants[MAX_PLANTS] = { 
#if (MAX_PLANTS >= 1)
        Plant(SENSOR_PLANT0, OUTPUT_PUMP0, &plant0, &plant0SensorTrigger, &wateringTime0, &wateringIdleTime0), 
#endif
#if (MAX_PLANTS >= 2)
        Plant(SENSOR_PLANT1, OUTPUT_PUMP1, &plant1, &plant1SensorTrigger, &wateringTime1, &wateringIdleTime1), 
#endif
#if (MAX_PLANTS >= 3)
        Plant(SENSOR_PLANT2, OUTPUT_PUMP2, &plant2, &plant2SensorTrigger, &wateringTime2, &wateringIdleTime2), 
#endif
#if (MAX_PLANTS >= 4)
        Plant(SENSOR_PLANT3, OUTPUT_PUMP3, &plant3, &plant3SensorTrigger, &wateringTime3, &wateringIdleTime3),  
#endif
#if (MAX_PLANTS >= 5)
        Plant(SENSOR_PLANT4, OUTPUT_PUMP4, &plant4, &plant4SensorTrigger, &wateringTime4, &wateringIdleTime4),  
#endif
#if (MAX_PLANTS >= 6)
        Plant(SENSOR_PLANT5, OUTPUT_PUMP5, &plant5, &plant5SensorTrigger, &wateringTime5, &wateringIdleTime5),  
#endif
#if (MAX_PLANTS >= 7)
        Plant(SENSOR_PLANT6, OUTPUT_PUMP6, &plant6, &plant6SensorTrigger, &wateringTime6, &wateringIdleTime6) 
#endif
      };

void readAnalogValues() {
  if (readCounter < AMOUNT_SENOR_QUERYS) {
    lipoSensorValues += analogRead(SENSOR_LIPO);
    solarSensorValues += analogRead(SENSOR_SOLAR);
    readCounter++;
  } else {
    lipoSenor = (lipoSensorValues >> SENSOR_QUERY_SHIFTS);
    lipoSensorValues = 0;
    solarSensor = (solarSensorValues >> SENSOR_QUERY_SHIFTS);
    solarSensorValues = 0;
    
    readCounter = 0;
  }
}

/**
 * @brief cyclic Homie callback
 * All logic, to be done by the controller cyclically
 */
void loopHandler() {

  int waterLevelPercent = (100 * mWaterGone) / waterLevel.get();

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

    for(int i=0; i < plantCnt.get() && i < MAX_PLANTS; i++) {
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

    sensorWater.setProperty("remaining").send(String(waterLevelPercent));
    Serial << "Water : " << mWaterGone << " cm (" << waterLevelPercent << "%)" << endl;

    /* Check if a plant needs water */
    if (gCurrentPlant > 0) {
      int plntIdx = (gCurrentPlant-1);
      if (mPlants[plntIdx].isPumpRequired() && 
          (waterLevelPercent > waterMinPercent.get()) &&
          (digitalRead(mPlants[plntIdx].getPumpPin()) == LOW) ) {
          Serial << "Plant" << plntIdx << " needs water" << endl;
          mPlants[plntIdx].setProperty("switch").send(String("ON"));
        }
        digitalWrite(OUTPUT_PUMP, HIGH);
        digitalWrite(mPlants[plntIdx].getPumpPin(), HIGH);
      }
  }
  mLoopInited = true;

  readAnalogValues();

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
  if (waterLevelPercent <= waterMinPercent.get()) {
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
    Serial << "No sleeping activated (maximum)" << endl;
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

/**
 * @brief Sensors, that are connected to GPIOs, mandatory for WIFI.
 * These sensors (ADC2) can only be read when no Wifi is used.
 */
void readSensors() {
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

#ifdef HC_SR04
  /* Use the Ultrasonic sensor to measure waterLevel */
  
  /* deactivate all sensors and measure the pulse */
  digitalWrite(SENSOR_SR04_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(SENSOR_SR04_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(SENSOR_SR04_TRIG, LOW);
  float duration = pulseIn(SENSOR_SR04_ECHO, HIGH);
  float distance = (duration*.0343)/2;
  mWaterGone = (int) distance;
  Serial << "HC_SR04 | Distance : " << String(distance) << " cm" << endl;
#endif
  /* deactivate the sensors */
  digitalWrite(OUTPUT_SENSOR, LOW);
}

/**
 * @brief Startup function
 * Is called once, the controller is started
 */
void setup() {
  /* Required to read the temperature once */
  float temp[2] = {0, 0};
  float* pFloat = temp;

  /* read button */
  pinMode(BUTTON, INPUT);

  Serial.begin(115200);
  Serial.setTimeout(1000); // Set timeout of 1 second
  Serial << endl << endl;
  Serial << "Read analog sensors..." << endl;
  /* Disable Wifi and bluetooth */
  WiFi.mode(WIFI_OFF);
  /* now ADC2 can be used */
  readSensors();
  /* activate Wifi again */
  WiFi.mode(WIFI_STA);


  if (HomieInternals::MAX_CONFIG_SETTING_SIZE < MAX_CONFIG_SETTING_ITEMS) {
    Serial << "HOMIE | Settings: " << HomieInternals::MAX_CONFIG_SETTING_SIZE << "/" << MAX_CONFIG_SETTING_ITEMS << endl;
    Serial << "      | Update Limits.hpp : MAX_CONFIG_SETTING_SIZE to " << MAX_CONFIG_SETTING_ITEMS << endl;
  }

  Homie_setFirmware("PlantControl", FIRMWARE_VERSION);
  Homie.setLoopFunction(loopHandler);

  mConfigured = Homie.isConfigured();
  if (mConfigured) {
    // Load the settings
    deepSleepTime.setDefaultValue(0);
    deepSleepNightTime.setDefaultValue(0);
    wateringTime0.setDefaultValue(60);
    wateringTime1.setDefaultValue(60);
    wateringTime2.setDefaultValue(60);
    wateringTime3.setDefaultValue(60);
    wateringTime4.setDefaultValue(60);
    wateringTime5.setDefaultValue(60);
    wateringTime6.setDefaultValue(60);
    plantCnt.setDefaultValue(0).setValidator([] (long candidate) {
      return ((candidate >= 0) && (candidate <= 6) );
    });
    plant1SensorTrigger.setDefaultValue(0);
    plant2SensorTrigger.setDefaultValue(0);
    plant3SensorTrigger.setDefaultValue(0);
  #if (MAX_PLANTS >= 4)
    plant4SensorTrigger.setDefaultValue(0);
    plant5SensorTrigger.setDefaultValue(0);
    plant6SensorTrigger.setDefaultValue(0);
  #endif

  #ifdef HC_SR04
    waterLevel.setDefaultValue(50);
    waterMinPercent.setDefaultValue(5);
  #endif

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
  }
  
  Homie.setup();

  /* Intialize inputs and outputs */
  for(int i=0; i < plantCnt.get(); i++) {
    pinMode(mPlants[i].getPumpPin(), OUTPUT);
    pinMode(mPlants[i].getSensorPin(), ANALOG);
    digitalWrite(mPlants[i].getPumpPin(), LOW);
  }
  /* Setup Solar and Lipo measurement */
  pinMode(SENSOR_LIPO, ANALOG);
  pinMode(SENSOR_SOLAR, ANALOG);
  /* Read analog values at the start */
  do {
    readAnalogValues();
  } while (readCounter != 0);


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

  if (mConfigured && (ADC_5V_TO_3V3(lipoSenor) < MINIMUM_LIPO_VOLT) && (deepSleepTime.get()) ) {
    long sleepEmptyLipo = (deepSleepTime.get() * EMPTY_LIPO_MULTIPL);
    Serial << "HOMIE | Change sleeping to " << sleepEmptyLipo << " ms as lipo is at " << ADC_5V_TO_3V3(lipoSenor) << "V" << endl;
    esp_sleep_enable_timer_wakeup(sleepEmptyLipo * 1000U);
    mDeepSleep = true;
  }

  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_ON);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_XTAL,ESP_PD_OPTION_ON);

  Serial << "DS18B20 | Initialization " << endl;
  /* Read the temperature sensors once, as first time 85 degree is returned */
  Serial << "DS18B20 | sensors: " << String(dallas.readDevices()) << endl;
  delay(200);
  if (dallas.readAllTemperatures(pFloat, 2) > 0) {
      Serial << "DS18B20 | Temperature 1: " << String(temp[0]) << endl;
      Serial << "DS18B20 | Temperature 2: " << String(temp[1]) << endl;
  }
  delay(200);
  if (dallas.readAllTemperatures(pFloat, 2) > 0) {
      Serial << "Temperature 1: " << String(temp[0]) << endl;
      Serial << "Temperature 2: " << String(temp[1]) << endl;
  }
}

/**
 * @brief Cyclic call
 * Executs the Homie base functionallity or triggers sleeping, if requested.
 */
void loop() {
  if (!mDeepSleep || !mConfigured) {
    if (Serial.available() > 0) {
      // read the incoming byte:
      int incomingByte = Serial.read();

      switch ((char) incomingByte)
      {
      case 'P':
          Serial << "Activate Sensor OUTPUT " << endl;
          pinMode(OUTPUT_SENSOR, OUTPUT);
          digitalWrite(OUTPUT_SENSOR, HIGH);
        break;
      case 'p':
          Serial << "Deactivate Sensor OUTPUT " << endl;
          pinMode(OUTPUT_SENSOR, OUTPUT);
          digitalWrite(OUTPUT_SENSOR, LOW);
        break;
      default:
        break;
      }
    }

    if ((digitalRead(BUTTON) == LOW) && (mButtonClicks % 2) == 0) {
      float temp[2] = {0, 0};
      float* pFloat = temp;
      mButtonClicks++;

      Serial << "SELF TEST (clicks: " << String(mButtonClicks) << ")" << endl;
      
      Serial << "DS18B20 sensors: " << String(dallas.readDevices()) << endl;
      delay(200);
      if (dallas.readAllTemperatures(pFloat, 2) > 0) {
          Serial << "Temperature 1: " << String(temp[0]) << endl;
          Serial << "Temperature 2: " << String(temp[1]) << endl;
      }
      
      switch(mButtonClicks) {
        case 1:
        case 3:
        case 5:
          if (mButtonClicks > 1) {
          Serial << "Read analog sensors..." << endl;
            /* Disable Wifi and bluetooth */
            WiFi.mode(WIFI_OFF);
            delay(50);
            /* now ADC2 can be used */
            readSensors();
          }

          Serial << "Water gone:     " << String(mWaterGone) << " cm" << endl;
          for(int i=0; i < MAX_PLANTS; i++) {
            mPlants[i].calculateSensorValue(AMOUNT_SENOR_QUERYS);

            Serial << "Moist Sensor " << (i+1) << ": " << String(mPlants[i].getSensorValue()) << " Volt: " << String(ADC_5V_TO_3V3(mPlants[i].getSensorValue())) << endl;
          }
          /* Read enough values */
          do {
            readAnalogValues();
            Serial << "Read Analog (" << String(readCounter) << ")" << endl;;
          } while (readCounter != 0);

          Serial << "Lipo Sensor  - Raw: " << String(lipoSenor) << " Volt: " << String(ADC_5V_TO_3V3(lipoSenor)) << endl;
          Serial << "Solar Sensor - Raw: " << String(solarSensor) << " Volt: " << String(SOLAR_VOLT(solarSensor)) << endl;
          break;
          case 7:
            Serial << "Activate Sensor OUTPUT " << endl;
            pinMode(OUTPUT_SENSOR, OUTPUT);
            digitalWrite(OUTPUT_SENSOR, HIGH);
            break;
          case 9:
            Serial << "Activate Pump1 GPIO" << String(mPlants[0].getPumpPin()) << endl;
            digitalWrite(mPlants[0].getPumpPin(), HIGH);
            break;
          case 11:
            Serial << "Activate Pump2 GPIO" << String(mPlants[1].getPumpPin()) << endl;
            digitalWrite(mPlants[1].getPumpPin(), HIGH);
            break;
          case 13:
            Serial << "Activate Pump3 GPIO" << String(mPlants[2].getPumpPin()) << endl;
            digitalWrite(mPlants[2].getPumpPin(), HIGH);
            break;
          case 15:
            Serial << "Activate Pump4/Sensor GPIO" << String(OUTPUT_PUMP4) << endl;
            digitalWrite(OUTPUT_PUMP4, HIGH);
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
    Serial << (millis()/ 1000) << "s running; sleeeping ..." << endl;
    Serial.flush();
    esp_deep_sleep_start();
  }
}

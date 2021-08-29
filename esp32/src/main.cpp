/** \addtogroup Controller
 *  @{
 * 
 * @file main.cpp
 * @author Ollo
 * @brief PlantControl
 * @version 0.1
 * @date 2020-05-01
 * 
 * @copyright Copyright (c) 2020
 */

/******************************************************************************
 *                                     INCLUDES
******************************************************************************/
#include "LogDefines.h"
#include "FileUtils.h"
#include "TimeUtils.h"
#include "PlantCtrl.h"
#include "ControllerConfiguration.h"
#include "HomieConfiguration.h"
#include "DallasTemperature.h"
#include <Homie.h>
#include "time.h"
#include "esp_sleep.h"
#include "RunningMedian.h"
#include "WakeReason.h"
#include <stdint.h>
#include <math.h>
#include <OneWire.h>
#include "DS2438.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <Wire.h>
#include <VL53L0X.h>

/******************************************************************************
 *                                     DEFINES
******************************************************************************/
#define AMOUNT_SENOR_QUERYS 8
#define MAX_TANK_DEPTH 2000
#define TEST_TOPIC "roundtrip\0"
#define LOG_TOPIC "log\0"
#define BACKUP_TOPIC "$implementation/config/backup/set\0"
#define CONFIG_FILE "/homie/config.json"
#define CONFIG_FILE_BACKUP "/homie/config.json.bak"

#define getTopic(test, topic)                                                                                                                 \
  char *topic = new char[strlen(Homie.getConfiguration().mqtt.baseTopic) + strlen(Homie.getConfiguration().deviceId) + 1 + strlen(test) + 1]; \
  strcpy(topic, Homie.getConfiguration().mqtt.baseTopic);                                                                                     \
  strcat(topic, Homie.getConfiguration().deviceId);                                                                                           \
  strcat(topic, "/");                                                                                                                         \
  strcat(topic, test);

/******************************************************************************
 *                            FUNCTION PROTOTYPES
******************************************************************************/

void log(int level, String message, int code);
int determineNextPump(bool lowLight);
void plantcontrol();
void readPowerSwitchedSensors();
bool determineTimedLightState(bool lowLight);

/******************************************************************************
 *                       NON VOLATILE VARIABLES in DEEP SLEEP
******************************************************************************/

RTC_DATA_ATTR int lastPumpRunning = -1; /**< store last successfully waterd plant */
RTC_DATA_ATTR long lastWaterValue = 0;  /**< to calculate the used water per plant */
#if defined(TIMED_LIGHT_PIN)
  RTC_DATA_ATTR bool timedLightOn = false; /**< allow fast recovery after poweron */
#endif // TIMED_LIGHT_PIN


RTC_DATA_ATTR long rtcLastWateringPlant[MAX_PLANTS] = {0};
RTC_DATA_ATTR long consecutiveWateringPlant[MAX_PLANTS] = {0};

/******************************************************************************
 *                            LOCAL VARIABLES
******************************************************************************/
bool volatile mDownloadMode = false; /**< Controller must not sleep */
bool volatile mSensorsRead = false;  /**< Sensors are read without Wifi or MQTT */
bool volatile mAliveWasRead = false;
bool volatile mMQTTReady = false;

bool mConfigured = false;
long nextBlink = 0; /**< Time needed in main loop to support expected blink code */

RunningMedian waterRawSensor = RunningMedian(5);
float mSolarVoltage = 0.0f; /**< Voltage from solar panels */
unsigned long setupFinishedTimestamp;

/*************************** Hardware abstraction *****************************/

OneWire oneWire(SENSOR_ONEWIRE);
DallasTemperature sensors(&oneWire);
DS2438 battery(&oneWire, 0.0333333f, AMOUNT_SENOR_QUERYS);
VL53L0X tankSensor;

Plant mPlants[MAX_PLANTS] = {
    Plant(SENSOR_PLANT0, OUTPUT_PUMP0, 0, &plant0, &mSetting0),
    Plant(SENSOR_PLANT1, OUTPUT_PUMP1, 1, &plant1, &mSetting1),
    Plant(SENSOR_PLANT2, OUTPUT_PUMP2, 2, &plant2, &mSetting2),
    Plant(SENSOR_PLANT3, OUTPUT_PUMP3, 3, &plant3, &mSetting3),
    Plant(SENSOR_PLANT4, OUTPUT_PUMP4, 4, &plant4, &mSetting4),
    Plant(SENSOR_PLANT5, OUTPUT_PUMP5, 5, &plant5, &mSetting5),
    Plant(SENSOR_PLANT6, OUTPUT_PUMP6, 6, &plant6, &mSetting6)};

/******************************************************************************
 *                            LOCAL FUNCTIONS
******************************************************************************/

void espDeepSleepFor(long seconds, bool activatePump)
{
  if (mDownloadMode)
  {
    log(LOG_LEVEL_DEBUG, "abort deepsleep, DownloadMode active", LOG_DEBUG_CODE);
    return;
  }
  if (mAliveWasRead)
  {
    for (int i = 0; i < 10; i++)
    {
      long cTime = getCurrentTime();
      if (cTime < 100000)
      {
        delay(100);
      }
      else
      {
        break;
      }
    }
    if (getCurrentTime() < 100000)
    {
      log(LOG_LEVEL_DEBUG, "NTP timeout before deepsleep", LOG_DEBUG_CODE);
    }
  }
  
  //allo hold for all digital pins
  gpio_deep_sleep_hold_en();

  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_ON);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_ON);
  if (activatePump)
  {
    esp_sleep_pd_config(ESP_PD_DOMAIN_XTAL, ESP_PD_OPTION_ON);
    
    gpio_hold_en(OUTPUT_ENABLE_PUMP); //pump pwr
  }
  else
  {
    gpio_hold_dis(OUTPUT_ENABLE_PUMP); //pump pwr
    digitalWrite(OUTPUT_ENABLE_PUMP, LOW);
    digitalWrite(OUTPUT_ENABLE_SENSOR, LOW);
    for (int i = 0; i < MAX_PLANTS; i++)
    {
      mPlants[i].deactivatePump();
    }
  }
  gpio_hold_en(OUTPUT_PUMP0);
  gpio_hold_en(OUTPUT_PUMP1);
  gpio_hold_en(OUTPUT_PUMP2);
  gpio_hold_en(OUTPUT_PUMP3);
  gpio_hold_en(OUTPUT_PUMP4);
  gpio_hold_en(OUTPUT_PUMP5);
  gpio_hold_en(OUTPUT_PUMP6);
  #if defined(TIMED_LIGHT_PIN)
    gpio_hold_en(TIMED_LIGHT_PIN);
  #endif // TIMED_LIGHT_PIN


  esp_sleep_enable_timer_wakeup((seconds * 1000U * 1000U));
  if (mAliveWasRead)
  {
    delay(1000);
    Homie.prepareToSleep();
  }
  else
  {
    esp_deep_sleep_start();
  }
}

//requires homie being started
void readOneWireSensors()
{
  for (uint8_t i = 0; i < sensors.getDeviceCount(); i++)
  {
    uint8_t ds18b20Address[8];

    bool valid = false;
    float temp = -127;
    for (int retry = 0; retry < AMOUNT_SENOR_QUERYS && !valid; retry++)
    {
      bool validAddress = sensors.getAddress(ds18b20Address, i);
      if (validAddress && sensors.validFamily(ds18b20Address))
      {
        temp = sensors.getTempC(ds18b20Address);
        if (temp != -127)
        {
          valid = true;
        }
        else
        {
          delay(10);
        }
      }
    }

    if (!valid)
    {
      //wrong family or crc errors on each retry
      continue;
    }

    char buf[sizeof(ds18b20Address) * 2];
    snprintf(buf, sizeof(buf), "%.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X",
             ds18b20Address[0],
             ds18b20Address[1],
             ds18b20Address[2],
             ds18b20Address[3],
             ds18b20Address[4],
             ds18b20Address[5],
             ds18b20Address[6],
             ds18b20Address[7]);

    if (valid)
    {
      Serial << "DS18S20 Temperatur " << String(buf) << " : " << temp << " °C " << endl;
      if (strcmp(lipoSensorAddr.get(), buf) == 0)
      {
        if (mAliveWasRead)
        {
          sensorTemp.setProperty(TEMPERATUR_SENSOR_LIPO).send(String(temp));
        }
        Serial << "Lipo Temperatur " << temp << " °C " << endl;
      }
      if (strcmp(waterSensorAddr.get(), buf) == 0)
      {
        if (mAliveWasRead)
        {
          sensorTemp.setProperty(TEMPERATUR_SENSOR_WATER).send(String(temp));
        }
        Serial << "Water Temperatur " << temp << " °C " << endl;
      }
      /* Always send the sensor address with the temperatur value */
      if (mAliveWasRead)
      {
        sensorTemp.setProperty(String(buf)).send(String(temp));
      }
    }
    else
    {
      Serial << "DS18S20 sensor " << String(buf) << " could not be read " << temp << endl;
    }
  }

  battery.update();
  mSolarVoltage = battery.getVoltage(BATTSENSOR_INDEX_SOLAR) * SOLAR_VOLT_FACTOR;

  Serial.flush();
}

/**
 * @brief Sensors, that are connected to GPIOs, mandatory for WIFI.
 * These sensors (ADC2) can only be read when no Wifi is used.
 */
void readPowerSwitchedSensors()
{
  digitalWrite(OUTPUT_ENABLE_SENSOR, HIGH);
  delay(50);
  for (int i = 0; i < MAX_PLANTS; i++)
  {
    mPlants[i].startMoistureMeasurement();
  }

  delay(MOISTURE_MEASUREMENT_DURATION);
  for (int i = 0; i < MAX_PLANTS; i++)
  {
    mPlants[i].stopMoistureMeasurement();
  }

  for (int i = 0; i < MAX_PLANTS; i++)
  {
    Serial << "Plant " << i << " measurement: " << mPlants[i].getCurrentMoisture() << " hz" << endl;
  }

  waterRawSensor.clear();
  Wire.setPins(SENSOR_TANK_TRG, SENSOR_TANK_ECHO);
  Wire.begin();
  tankSensor.setTimeout(500);
  long start = millis();
  bool distanceReady = false;
  while (start + 500 > millis())
  {
    if (tankSensor.init())
    {
      distanceReady = true;
      break;
    }
    else
    {
      delay(20);
    }
  }
  if (distanceReady)
  {
    tankSensor.setSignalRateLimit(0.1);
    // increase laser pulse periods (defaults are 14 and 10 PCLKs)
    tankSensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    tankSensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
    tankSensor.setMeasurementTimingBudget(200000);

    for (int readCnt = 0; readCnt < 5; readCnt++)
    {
      if (!tankSensor.timeoutOccurred())
      {
        uint16_t distance = tankSensor.readRangeSingleMillimeters();
        if (distance < MAX_TANK_DEPTH)
        {
          waterRawSensor.add(distance);
        }
      }
      delay(10);
    }
    Serial << "Distance sensor " << waterRawSensor.getMedian() << " mm" << endl;
  }
  else
  {
    log(LOG_LEVEL_WARN, LOG_TANKSENSOR_FAIL_DETECT, LOG_TANKSENSOR_FAIL_DETECT_CODE);
  }

  /* deactivate the sensors */
  digitalWrite(OUTPUT_ENABLE_SENSOR, LOW);
}

void onMessage(char *incoming, char *payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total)
{
  getTopic(TEST_TOPIC, testTopic);
  if (strcmp(incoming, testTopic) == 0)
  {
    mAliveWasRead = true;
  }
  delete testTopic;
  getTopic(BACKUP_TOPIC, backupTopic);
  if (strcmp(incoming, backupTopic) == 0)
  {
    if (strcmp(payload, "true") == 0)
    {
      bool backupSucessful = copyFile(CONFIG_FILE, CONFIG_FILE_BACKUP);
      printFile(CONFIG_FILE_BACKUP);
      if (backupSucessful)
      {
        log(LOG_LEVEL_INFO, LOG_BACKUP_SUCCESSFUL, LOG_BACKUP_SUCCESSFUL_CODE);
      }
      else
      {
        log(LOG_LEVEL_INFO, LOG_BACKUP_FAILED, LOG_BACKUP_FAILED_CODE);
      }
      Homie.getMqttClient().publish(backupTopic, 2, true, "false");
    }
  }
  delete backupTopic;
}

void onHomieEvent(const HomieEvent &event)
{
  switch (event.type)
  {
  case HomieEventType::READY_TO_SLEEP:
    esp_deep_sleep_start();
    break;
  case HomieEventType::SENDING_STATISTICS:
    break;
  case HomieEventType::MQTT_READY:
    if (mSensorsRead)
    {
      Serial.printf("Timeout occured... too late!\r\n");
      return;
    }
    mSensorsRead = true; // MQTT is working, deactivate timeout logic

    configTime(0, 0, ntpServer.get());

    {
      getTopic(TEST_TOPIC, testopic)
          Homie.getMqttClient()
              .subscribe(testopic, 2);
      Homie.getMqttClient().publish(testopic, 2, false, "ping");
      Homie.getMqttClient().onMessage(onMessage);

      getTopic(BACKUP_TOPIC, backupTopic)
          Homie.getMqttClient()
              .subscribe(backupTopic, 2);
    }
    mMQTTReady = true;

    break;
  case HomieEventType::OTA_STARTED:
    for (int i = 0; i < MAX_PLANTS; i++)
    {
      mPlants[i].deactivatePump();
    }
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
    digitalWrite(OUTPUT_ENABLE_PUMP, HIGH);
    delay(100);
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 1);
    mDownloadMode = true;
    break;
  case HomieEventType::OTA_SUCCESSFUL:
    digitalWrite(OUTPUT_ENABLE_SENSOR, LOW);
    digitalWrite(OUTPUT_ENABLE_PUMP, LOW);
    ESP.restart();
    break;
  default:
    break;
  }
}

int determineNextPump(bool isLowLight)
{
  int pumpToUse = -1;
  for (int i = 0; i < MAX_PLANTS; i++)
  {
    bool wateralarm = consecutiveWateringPlant[i] >= pumpIneffectiveWarning.get();
    if (wateralarm)
    {
      log(LOG_LEVEL_ERROR, String(String(i) + " Plant still dry after " + String(consecutiveWateringPlant[i]) + " watering attempts"), LOG_PUMP_INEFFECTIVE);
    }
    Plant plant = mPlants[i];
    if (!plant.isPumpTriggerActive())
    {
      plant.publishState("deactivated");
      log(LOG_LEVEL_DEBUG, String(String(i) + " Skip deactivated pump"), LOG_DEBUG_CODE);
      continue;
    }
    if ((rtcLastWateringPlant[i] + plant.getCooldownInSeconds()) > getCurrentTime())
    {
      if (wateralarm)
      {
        plant.publishState("cooldown+alarm");
      }
      else
      {
        plant.publishState("cooldown");
      }
      log(LOG_LEVEL_DEBUG, String(String(i) + " Skipping due to cooldown " + String(rtcLastWateringPlant[i] + plant.getCooldownInSeconds())), LOG_DEBUG_CODE);
      continue;
    }
    if (!isLowLight && plant.isAllowedOnlyAtLowLight())
    {
      if (wateralarm)
      {
        plant.publishState("sunny+alarm");
      }
      else
      {
        plant.publishState("sunny");
      }

      log(LOG_LEVEL_DEBUG, String(String(i) + " No pump required: due to light"), LOG_DEBUG_CODE);
      continue;
    }
    if (equalish(plant.getCurrentMoisture(), MISSING_SENSOR))
    {
      plant.publishState("nosensor");
      log(LOG_LEVEL_ERROR, String(String(i) + " No pump possible: missing sensor"), LOG_MISSING_PUMP);
      continue;
    }
    if (plant.isPumpRequired())
    {
      /* Handle e.g. start = 21, end = 8 */
      if (((plant.getHoursStart() > plant.getHoursEnd()) &&
           (getCurrentHour() >= plant.getHoursStart() || getCurrentHour() <= plant.getHoursEnd())) ||
          /* Handle e.g. start = 8, end = 21 */
          ((plant.getHoursStart() < plant.getHoursEnd()) &&
           (getCurrentHour() >= plant.getHoursStart() && getCurrentHour() <= plant.getHoursEnd())) ||
          /* no time from NTP received */
          (getCurrentTime() < 10000))
      {
        if (wateralarm)
        {
          plant.publishState("active+alarm");
        }
        else
        {
          plant.publishState("active");
        }

        consecutiveWateringPlant[i]++;
        log(LOG_LEVEL_DEBUG, String(String(i) + " Requested pumping"), LOG_DEBUG_CODE);
        pumpToUse = i;
      }
      else
      {
        if (wateralarm)
        {
          plant.publishState("after-work+alarm");
        }
        else
        {
          plant.publishState("after-work");
        }
        log(LOG_LEVEL_DEBUG, String(String(i) + " ignored due to time boundary: " + String(plant.getHoursStart()) + " to " + String(plant.getHoursEnd()) + " ( current " + String(getCurrentHour()) + " )"), LOG_DEBUG_CODE);
      }
      continue;
    }
    else
    {
      plant.publishState("wet");
      //plant was detected as wet, remove consecutive count
      consecutiveWateringPlant[i] = 0;
    }
  }
  return pumpToUse;
}

/**
 * @brief Handle Mqtt commands to keep controller alive
 * 
 * @param range multiple transmitted values (not used for this function)
 * @param value single value
 * @return true when the command was parsed and executed succuessfully
 * @return false on errors when parsing the request
 */
bool aliveHandler(const HomieRange &range, const String &value)
{
  if (range.isRange)
  {
    return false; // only one controller is present
  }

  if (value.equals("ON") || value.equals("On") || value.equals("1"))
  {
    mDownloadMode = true;
  }
  else
  {
    if (mDownloadMode)
    {
      esp_restart();
    }
    mDownloadMode = false;
  }

  return true;
}

bool notStarted = true;
void homieLoop()
{
  if (mMQTTReady && mAliveWasRead && notStarted)
  {
    Serial.println("received alive & mqtt is ready");
    notStarted = false;
    plantcontrol();
  }
}

bool switch1(const HomieRange &range, const String &value)
{
  return mPlants[0].switchHandler(range, value);
}

bool switch2(const HomieRange &range, const String &value)
{
  return mPlants[1].switchHandler(range, value);
}

bool switch3(const HomieRange &range, const String &value)
{
  return mPlants[2].switchHandler(range, value);
}

bool switch4(const HomieRange &range, const String &value)
{
  return mPlants[3].switchHandler(range, value);
}

bool switch5(const HomieRange &range, const String &value)
{
  return mPlants[4].switchHandler(range, value);
}

bool switch6(const HomieRange &range, const String &value)
{
  return mPlants[5].switchHandler(range, value);
}

bool switch7(const HomieRange &range, const String &value)
{
  return mPlants[6].switchHandler(range, value);
}

/**
 * @brief Startup function
 * Is called once, the controller is started
 */
void setup()
{
  /* reduce power consumption */
  setCpuFrequencyMhz(80);

  Serial.begin(115200);

  Serial << "Wifi mode set to " << WIFI_OFF << " to allow analog2 useage " << endl;
  WiFi.mode(WIFI_OFF);
  Serial.flush();

  //restore state before releasing pin, to prevent flickering
  #if defined(TIMED_LIGHT_PIN)
    pinMode(TIMED_LIGHT_PIN, OUTPUT);
    digitalWrite(TIMED_LIGHT_PIN, timedLightOn);
    gpio_hold_dis(TIMED_LIGHT_PIN);
  #endif // TIMED_LIGHT_PIN

  gpio_hold_dis(OUTPUT_PUMP0);
  gpio_hold_dis(OUTPUT_PUMP1);
  gpio_hold_dis(OUTPUT_PUMP2);
  gpio_hold_dis(OUTPUT_PUMP3);
  gpio_hold_dis(OUTPUT_PUMP4);
  gpio_hold_dis(OUTPUT_PUMP5);
  gpio_hold_dis(OUTPUT_PUMP6);
  gpio_hold_dis(OUTPUT_ENABLE_PUMP);

  /* Intialize Plant */
  for (int i = 0; i < MAX_PLANTS; i++)
  {
    mPlants[i].init();
  }
  // read button
  pinMode(BUTTON, INPUT);

  // Power pins
  pinMode(OUTPUT_ENABLE_PUMP, OUTPUT);

  digitalWrite(OUTPUT_ENABLE_PUMP, LOW);

  pinMode(OUTPUT_ENABLE_SENSOR, OUTPUT);

  if (HomieInternals::MAX_CONFIG_SETTING_SIZE < MAX_CONFIG_SETTING_ITEMS)
  {
    //increase the config settings
    Serial << "Limits.hpp is not adjusted, please search for this string and increase" << endl;
    return;
  }
  if (HomieInternals::MAX_JSON_CONFIG_FILE_SIZE < MAX_JSON_CONFIG_FILE_SIZE_CUSTOM)
  {
    //increase the config settings
    Serial << "Limits.hpp is not adjusted, please search for this string and increase" << endl;
    return;
  }

  /************************* Start One-Wire bus ***************/
  int tempInitStartTime = millis();
  uint8_t sensorCount = 0U;

  /* Required to read the temperature at least once */
  while ((sensorCount == 0 || !battery.isFound()) && millis() < tempInitStartTime + TEMPERATUR_TIMEOUT)
  {
    sensors.begin();
    battery.begin();
    sensorCount = sensors.getDS18Count();
    delay(50);
  }

  Serial << "DS18S20 count: " << sensorCount << " found in " << (millis() - tempInitStartTime) << " ms" << endl;
  Serial.flush();
  /* Measure temperature TODO idea: move this into setup */
  if (sensorCount > 0)
  {
    //sensors.setResolution(DS18B20_RESOLUTION);
    sensors.requestTemperatures();
  }
  Serial << "Reading sensors start" << endl;
  Serial.flush();
  readPowerSwitchedSensors();
  Serial << "Reading sensors end" << endl;
  Serial.flush();
  /************************* Start Homie Framework ***************/
  Homie_setFirmware("PlantControl", FIRMWARE_VERSION);
  Homie.disableLedFeedback();
  Homie_setBrand("PlantControl");
  // Set default values

  //in seconds
  deepSleepTime.setDefaultValue(600).setValidator([](long candidate)
                                                  { return (candidate > 0) && (candidate < (60 * 60 * 2) /** 2h max sleep */); });
  deepSleepNightTime.setDefaultValue(600);
  wateringDeepSleep.setDefaultValue(5);
  ntpServer.setDefaultValue("pool.ntp.org");

  /* waterLevelMax 1000    */          /* 100cm in mm */
  waterLevelMin.setDefaultValue(50);   /* 5cm in mm */
  waterLevelWarn.setDefaultValue(500); /* 50cm in mm */
  waterLevelVol.setDefaultValue(5000); /* 5l in ml */
  lipoSensorAddr.setDefaultValue("");
  waterSensorAddr.setDefaultValue("");
  pumpIneffectiveWarning.setDefaultValue(5).setValidator([](long candidate)
                                                           { return (candidate > 0) && (candidate < (20)); });

  #if defined(TIMED_LIGHT_PIN)
    timedLightStart.setDefaultValue(18).setValidator([](long candidate)
                                                           { return (candidate > 0) && (candidate < (25)); });
    timedLightEnd.setDefaultValue(23).setValidator([](long candidate)
                                                           { return (candidate > 0) && (candidate < (22)); });
    timedLightOnlyWhenDark.setDefaultValue(true);
    timedLightVoltageCutoff.setDefaultValue(3.8).setValidator([](double candidate)
                                                           { return (candidate > 3.3) && (candidate < (4.2)); });
  #endif // TIMED_LIGHT_PIN
    

  Homie.setLoopFunction(homieLoop);
  Homie.onEvent(onHomieEvent);

  Homie.setup();

  mConfigured = Homie.isConfigured();
  if (mConfigured)
  {
    for (int i = 0; i < MAX_PLANTS; i++)
    {
      mPlants[i].advertise();
    }
    mPlants[0].setSwitchHandler(switch1);
    mPlants[1].setSwitchHandler(switch2);
    mPlants[2].setSwitchHandler(switch3);
    mPlants[3].setSwitchHandler(switch4);
    mPlants[4].setSwitchHandler(switch5);
    mPlants[5].setSwitchHandler(switch6);
    mPlants[6].setSwitchHandler(switch7);

    sensorTemp.advertise(TEMPERATUR_SENSOR_LIPO)
        .setName(TEMPERATURE_NAME)
        .setDatatype(NUMBER_TYPE)
        .setUnit(TEMPERATURE_UNIT);
    sensorTemp.advertise(TEMPERATUR_SENSOR_WATER)
        .setName(TEMPERATURE_NAME)
        .setDatatype(NUMBER_TYPE)
        .setUnit(TEMPERATURE_UNIT);
    sensorTemp.advertise(TEMPERATUR_SENSOR_CHIP)
        .setName(TEMPERATURE_NAME)
        .setDatatype(NUMBER_TYPE)
        .setUnit(TEMPERATURE_UNIT);

    sensorLipo.advertise("percent")
        .setName("Percent")
        .setDatatype(NUMBER_TYPE)
        .setUnit("%");
    sensorLipo.advertise("volt")
        .setName("Volt")
        .setDatatype(NUMBER_TYPE)
        .setUnit("V");

    sensorSolar.advertise("percent")
        .setName("Percent")
        .setDatatype(NUMBER_TYPE)
        .setUnit("%");
    sensorSolar.advertise("volt")
        .setName("Volt")
        .setDatatype(NUMBER_TYPE)
        .setUnit("V");
    sensorWater.advertise("remaining").setDatatype(NUMBER_TYPE).setUnit("%");
  }
  else
  {
    if (doesFileExist(CONFIG_FILE))
    {
      printFile(CONFIG_FILE);
    }
    if (doesFileExist(CONFIG_FILE_BACKUP))
    {
      printFile(CONFIG_FILE_BACKUP);
      bool restoredConfig = copyFile(CONFIG_FILE_BACKUP, CONFIG_FILE);
      if (restoredConfig)
      {
        deleteFile(CONFIG_FILE_BACKUP);
        espDeepSleepFor(1, false);
        return;
      }
    }

    readOneWireSensors();
    //prevent BOD to be paranoid
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
    digitalWrite(OUTPUT_ENABLE_PUMP, HIGH);
    delay(100);
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 1);
    Serial.println("Initial Setup. Start Accesspoint...");
    mDownloadMode = true;
  }
  stayAlive.advertise("alive").setName("Alive").setDatatype(NUMBER_TYPE).settable(aliveHandler);
  setupFinishedTimestamp = millis();
}

/**
 * @brief Cyclic call
 * Executs the Homie base functionallity or triggers sleeping, if requested.
 */
void loop()
{
  Homie.loop();
  /* Toggel Senor LED to visualize mode 3 */
  if (mDownloadMode)
  {
    if (nextBlink < millis())
    {
      digitalWrite(OUTPUT_ENABLE_SENSOR, !digitalRead(OUTPUT_ENABLE_SENSOR));
      if (mConfigured)
      {
        nextBlink = millis() + 500;
      }
      else
      {
        if (lastPumpRunning >= 0 && lastPumpRunning < MAX_PLANTS)
        {
          mPlants[lastPumpRunning].deactivatePump();
        }
        if (lastPumpRunning >= MAX_PLANTS)
        {
          digitalWrite(OUTPUT_ENABLE_PUMP, LOW);
          nextBlink = millis() + 500;
        }
        else
        {
          lastPumpRunning++;
          nextBlink = millis() + 5000;
        }
        if (lastPumpRunning < MAX_PLANTS)
        {
          mPlants[lastPumpRunning].activatePump();
        }
      }
    }
  }
  else
  {
    unsigned long timeSinceSetup = millis() - setupFinishedTimestamp;
    if ((timeSinceSetup > MQTT_TIMEOUT) && (!mSensorsRead))
    {
      mSensorsRead = true;
      /* Disable Wifi and put modem into sleep mode */
      WiFi.mode(WIFI_OFF);
      Serial << "Wifi mode set to " << WIFI_OFF << " mqqt was no reached within " << timeSinceSetup << "ms , fallback to offline mode " << endl;
      Serial.flush();
      plantcontrol();
    }
  }

  /** Timeout always stopping the ESP -> no endless power consumption */
  if (millis() > ESP_STALE_TIMEOUT && !mDownloadMode)
  {
    Serial << (millis() / 1000) << "not terminated watchdog reset" << endl;
    Serial.flush();
    esp_restart();
  }
}

/***
 * @fn plantcontrol
 * Main function, doing the logic
 */
void plantcontrol()
{
  if (lastPumpRunning != -1)
  {
    long waterDiff = waterRawSensor.getAverage() - lastWaterValue;
    mPlants[lastPumpRunning].setProperty("waterusage").send(String(waterDiff));
    /* TODO convert diff into volume (milli liter) */
    Serial << "Plant" << lastPumpRunning << ": Water diff " << waterDiff << " mm" << endl;
  }

  if (mAliveWasRead)
  {
    for (int i = 0; i < MAX_PLANTS; i++)
    {
      mPlants[i].postMQTTconnection();
      mPlants[i].setProperty("consecutivePumps").send(String(consecutiveWateringPlant[i]));
    }
  }

  readOneWireSensors();

  Serial << "W : " << waterRawSensor.getAverage() << " cm (" << String(waterLevelMax.get() - waterRawSensor.getAverage()) << "%)" << endl;
  lastWaterValue = waterRawSensor.getAverage();

  float batteryVoltage = battery.getVoltage(BATTSENSOR_INDEX_BATTERY);
  float chipTemp = battery.getTemperature();
  Serial << "Chip Temperatur " << chipTemp << " °C " << endl;

  if (mAliveWasRead)
  {
    float remaining = waterLevelMax.get() - waterRawSensor.getAverage();
    if (!isnan(remaining))
    {
      sensorWater.setProperty("remaining").send(String(remaining));
    }
    if (!isnan(waterRawSensor.getAverage()))
    {
      sensorWater.setProperty("distance").send(String(waterRawSensor.getAverage()));
    }
    sensorLipo.setProperty("percent").send(String(100 * batteryVoltage / VOLT_MAX_BATT));
    sensorLipo.setProperty("volt").send(String(batteryVoltage));
    sensorLipo.setProperty("current").send(String(battery.getCurrent()));
    sensorLipo.setProperty("Ah").send(String(battery.getAh()));
    sensorLipo.setProperty("ICA").send(String(battery.getICA()));
    sensorLipo.setProperty("DCA").send(String(battery.getDCA()));
    sensorLipo.setProperty("CCA").send(String(battery.getCCA()));
    sensorSolar.setProperty("volt").send(String(mSolarVoltage));
    sensorTemp.setProperty(TEMPERATUR_SENSOR_CHIP).send(String(chipTemp));
  }
  else
  {
    Serial.println("Skipping MQTT, offline mode");
    Serial.flush();
  }

  bool isLowLight = (mSolarVoltage < SOLAR_CHARGE_MIN_VOLTAGE);
  bool hasWater = true; //FIXMEmWaterGone > waterLevelMin.get();
  //FIXME no water warning message
  lastPumpRunning = determineNextPump(isLowLight);
  if (lastPumpRunning != -1 && !hasWater)
  {
    log(LOG_LEVEL_ERROR, LOG_PUMP_BUTNOTANK_MESSAGE, LOG_PUMP_BUTNOTANK_CODE);
  }
  else if (lastPumpRunning != -1 && hasWater)
  {
    if (mDownloadMode)
    {
      log(LOG_LEVEL_INFO, LOG_PUMP_AND_DOWNLOADMODE, LOG_PUMP_AND_DOWNLOADMODE_CODE);
    }
    else
    {
      //prevent BOD to be paranoid
      WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
      digitalWrite(OUTPUT_ENABLE_PUMP, HIGH);
      delay(100);
      WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 1);

      rtcLastWateringPlant[lastPumpRunning] = getCurrentTime();
      mPlants[lastPumpRunning].activatePump();
    }
  }

  #if defined(TIMED_LIGHT_PIN)
    bool shouldLight = determineTimedLightState(isLowLight);
    timedLightOn = shouldLight;
    digitalWrite(TIMED_LIGHT_PIN, shouldLight);
  #endif // TIMED_LIGHT_PIN
  

  /* Always handle one of the deep sleep duration */
  if (lastPumpRunning == -1 || !hasWater)
  {
    if (mSolarVoltage < SOLAR_CHARGE_MIN_VOLTAGE)
    {
      log(LOG_LEVEL_INFO, String(String(mSolarVoltage) + "V! No pumps to activate and low light, deepSleepNight"), LOG_NOPUMP_LOWLIGHT);
      espDeepSleepFor(deepSleepNightTime.get(), false);
    }
    else
    {
      log(LOG_LEVEL_INFO, "No pumps to activate, deepSleep", LOG_NOPUMPS);
      espDeepSleepFor(deepSleepTime.get(), false);
    }
  }
  else
  {
    espDeepSleepFor(wateringDeepSleep.get(), true);
  }
}

/** @}*/


bool determineTimedLightState(bool lowLight){
  bool onlyAllowedWhenDark = timedLightOnlyWhenDark.get();
  long hoursStart = timedLightStart.get();
  long hoursEnd = timedLightEnd.get();

  //ntp missing
  if(getCurrentTime() < 10000){
    timedLightNode.setProperty("state").send(String("Off, missing ntp"));
    return false;
  }
  if(onlyAllowedWhenDark && !lowLight){
    timedLightNode.setProperty("state").send(String("Off, not dark"));
    return false;
  }

  if (((hoursStart > hoursEnd) &&
           (getCurrentHour() >= hoursStart || getCurrentHour() <= hoursEnd)) ||
          /* Handle e.g. start = 8, end = 21 */
          ((hoursStart < hoursEnd) &&
           (getCurrentHour() >= hoursStart && getCurrentHour() <= hoursEnd)))
      {
        if(battery.getVoltage(BATTSENSOR_INDEX_BATTERY) >= timedLightVoltageCutoff.get() ){
          timedLightNode.setProperty("state").send(String("On"));
          return true;
        }else {
          timedLightNode.setProperty("state").send(String("Off, due to missing voltage"));
          return false;
        }

      } else {
        timedLightNode.setProperty("state").send(String("Off, outside worktime"));
        return false;
      }
}

void log(int level, String message, int statusCode)
{
  String buffer;
  StaticJsonDocument<200> doc;
  doc["level"] = level;
  doc["message"] = message;
  doc["statusCode"] = statusCode;
  serializeJson(doc, buffer);
  if (mAliveWasRead)
  {
    getTopic(LOG_TOPIC, logTopic)
        Homie.getMqttClient()
            .subscribe(logTopic, 2);

    Homie.getMqttClient().publish(logTopic, 2, false, buffer.c_str());
    delete logTopic;
  }
  Serial << statusCode << "@" << level << " : " << message << endl;
}

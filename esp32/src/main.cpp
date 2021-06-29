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
#include "FileUtils.h"
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
#define MAX_TANK_DEPTH 1000
#define TEST_TOPIC "roundtrip\0"
#define BACKUP_TOPIC "$implementation/config/backup/set\0"
#define BACKUP_STATUS_TOPIC "$implementation/config/backup\0"
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

int determineNextPump();
void plantcontrol(boolean withHomie);
void readPowerSwitchedSensors();

/******************************************************************************
 *                       NON VOLATILE VARIABLES in DEEP SLEEP
******************************************************************************/

RTC_SLOW_ATTR int lastPumpRunning = -1; /**< store last successfully waterd plant */
RTC_SLOW_ATTR long lastWaterValue = 0;  /**< to calculate the used water per plant */

RTC_SLOW_ATTR long rtcLastWateringPlant[MAX_PLANTS] = {0};

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
long getCurrentTime()
{
  struct timeval tv_now;
  gettimeofday(&tv_now, NULL);
  return tv_now.tv_sec;
}

int getCurrentHour()
{
  struct tm info;
  time_t now;
  time(&now);
  localtime_r(&now, &info);
  return info.tm_hour;
}

void espDeepSleepFor(long seconds, bool activatePump, bool withHomieShutdown)
{
  if (mDownloadMode)
  {
    Serial << "abort deepsleep, DownloadMode active" << endl;
    return;
  }
  if (withHomieShutdown)
  {
    for (int i = 0; i < 10; i++)
    {
      long cTime = getCurrentTime();
      if (cTime < 100000)
      {
        Serial << "Wait for ntp" << endl;
        delay(100);
      }
      else
      {
        break;
      }
    }
  }

  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_ON);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
  if (activatePump)
  {
    esp_sleep_pd_config(ESP_PD_DOMAIN_XTAL, ESP_PD_OPTION_ON);
    gpio_deep_sleep_hold_en();
    gpio_hold_en(OUTPUT_ENABLE_PUMP); //pump pwr
  }
  else
  {
    gpio_hold_dis(OUTPUT_ENABLE_PUMP); //pump pwr
    gpio_deep_sleep_hold_dis();
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
  //FIXME fix for outher outputs

  Serial.print("Trying to sleep for ");
  Serial.print(seconds);
  Serial.println(" seconds");
  esp_sleep_enable_timer_wakeup((seconds * 1000U * 1000U));
  Serial.flush();
  if (withHomieShutdown)
  {
    Homie.prepareToSleep();
  }
  else
  {
    Serial << "Bye offline mode" << endl;
    Serial.flush();
    esp_deep_sleep_start();
  }
}

//requires homie being started
void readOneWireSensors(bool withMQTT)
{

  Serial << "Read OneWire" << endl;
  Serial.flush();

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
        if (withMQTT)
        {
          sensorTemp.setProperty(TEMPERATUR_SENSOR_LIPO).send(String(temp));
        }
        Serial << "Lipo Temperatur " << temp << " °C " << endl;
      }
      if (strcmp(waterSensorAddr.get(), buf) == 0)
      {
        if (withMQTT)
        {
          sensorTemp.setProperty(TEMPERATUR_SENSOR_WATER).send(String(temp));
        }
        Serial << "Water Temperatur " << temp << " °C " << endl;
      }
      /* Always send the sensor address with the temperatur value */
      if (withMQTT)
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
  delay(10);
  for (int readCnt = 0; readCnt < AMOUNT_SENOR_QUERYS; readCnt++)
  {
    for (int i = 0; i < MAX_PLANTS; i++)
    {
      mPlants[i].addSenseValue();
    }
    delay(2);
  }

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
      if(!tankSensor.timeoutOccurred()){
        waterRawSensor.add(tankSensor.readRangeSingleMillimeters());
      }
      delay(10);
    }
    Serial << "Distance sensor " << waterRawSensor.getMedian() << " mm" << endl;
  }
  else
  {
    Serial.println("Failed to detect and initialize distance sensor!");
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
    if(strcmp(payload, "true") == 0){
      bool backupSucessful = copyFile(CONFIG_FILE, CONFIG_FILE_BACKUP);
      printFile(CONFIG_FILE_BACKUP);
      getTopic(BACKUP_STATUS_TOPIC, backupStatusTopic);
      Homie.getMqttClient().publish(backupStatusTopic, 2, true, backupSucessful ? "true" : "false");
      delete backupStatusTopic;
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
    Serial << "Bye homie mode" << endl;
    Serial.flush();
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

    Serial.printf("NTP Setup with server %s\r\n", ntpServer.get());
    configTime(0, 0, ntpServer.get());

    Serial << "publish plants mqtt" << endl;
    for (int i = 0; i < MAX_PLANTS; i++)
    {
      mPlants[i].postMQTTconnection();
    }
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
    Homie.getLogger() << "OTA started" << endl;
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
    Homie.getLogger() << "OTA successful" << endl;
    digitalWrite(OUTPUT_ENABLE_SENSOR, LOW);
    digitalWrite(OUTPUT_ENABLE_PUMP, LOW);
    ESP.restart();
    break;
  default:
    break;
  }
}

int determineNextPump()
{
  bool isLowLight = (mSolarVoltage < SOLAR_CHARGE_MIN_VOLTAGE);
  int pumpToUse = -1;
  for (int i = 0; i < MAX_PLANTS; i++)
  {
    Plant plant = mPlants[i];
    if (!plant.isPumpTriggerActive())
    {
      Serial.printf("%d Skip deactivated pump\r\n", i);
      continue;
    }
    if ((rtcLastWateringPlant[i] > 0) && ((rtcLastWateringPlant[i] + plant.getCooldownInSeconds()) < getCurrentTime()))
    {
      Serial.printf("%d Skipping due to cooldown %ld / %ld \r\n", i, rtcLastWateringPlant[i], plant.getCooldownInSeconds());
      continue;
    }
    if (!isLowLight && plant.isAllowedOnlyAtLowLight())
    {
      Serial.printf("%d No pump required: due to light\r\n", i);
      continue;
    }
    if (plant.getCurrentMoisture() == MISSING_SENSOR)
    {
      Serial.printf("%d No pump possible: missing sensor \r\n", i);
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
        Serial.printf("%d Requested pumping\r\n", i);
        pumpToUse = i;
      }
      else
      {
        Serial.printf("%d ignored due to time boundary: %d to %d (current %d)\r\n", i, plant.getHoursStart(), plant.getHoursEnd(), getCurrentHour());
      }
      continue;
    }
    else
    {
      Serial.printf("%d No pump required: moisture acceptable %f / %ld\r\n", i, plant.getCurrentMoisture(), plant.getSettingsMoisture());
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
    return false; // only one controller is present
  Serial.println("aliuve handler");
  Serial.flush();
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
    plantcontrol(true);
  }
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

  /* Intialize Plant */
  for (int i = 0; i < MAX_PLANTS; i++)
  {
    mPlants[i].init();
  }
  Serial.println("plants init");
  Serial.flush();
  // read button
  pinMode(BUTTON, INPUT);

  // Power pins
  pinMode(OUTPUT_ENABLE_PUMP, OUTPUT);

  digitalWrite(OUTPUT_ENABLE_PUMP, LOW);

  pinMode(OUTPUT_ENABLE_SENSOR, OUTPUT);

  if (HomieInternals::MAX_CONFIG_SETTING_SIZE < MAX_CONFIG_SETTING_ITEMS)
  {
    //increase the config settings to 50 and the json to 3000
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
  deepSleepTime.setDefaultValue(600).setValidator([](long candidate) { return (candidate > 0) && (candidate < (60 * 60 * 2) /** 2h max sleep */); });
  deepSleepNightTime.setDefaultValue(600);
  wateringDeepSleep.setDefaultValue(5);
  ntpServer.setDefaultValue("pool.ntp.org");

  /* waterLevelMax 1000    */          /* 100cm in mm */
  waterLevelMin.setDefaultValue(50);   /* 5cm in mm */
  waterLevelWarn.setDefaultValue(500); /* 50cm in mm */
  waterLevelVol.setDefaultValue(5000); /* 5l in ml */
  lipoSensorAddr.setDefaultValue("");
  waterSensorAddr.setDefaultValue("");
  Homie.setLoopFunction(homieLoop);
  Homie.onEvent(onHomieEvent);
  //Homie.disableLogging();


  Homie.setup();

  mConfigured = Homie.isConfigured();
  if (mConfigured)
  {

    for (int i = 0; i < MAX_PLANTS; i++)
    {
      mPlants[i].advertise();
    }
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
    printFile("/homie/Readme.md");
    if (doesFileExist(CONFIG_FILE)){
      printFile(CONFIG_FILE);
    }
    if (doesFileExist(CONFIG_FILE_BACKUP)){
      printFile(CONFIG_FILE_BACKUP);
      bool restoredConfig = copyFile(CONFIG_FILE_BACKUP, CONFIG_FILE);
      if(restoredConfig){
         deleteFile(CONFIG_FILE_BACKUP);
         espDeepSleepFor(1,false,false);
         return;
      }
    }

    
    readOneWireSensors(false);
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
      plantcontrol(false);
    }
  }

  /** Timeout always stopping the ESP -> no endless power consumption */
  if (millis() > 60000 && !mDownloadMode)
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
void plantcontrol(bool withHomie)
{
  if (lastPumpRunning != -1)
  {
    long waterDiff = waterRawSensor.getAverage() - lastWaterValue;
    mPlants[lastPumpRunning].setProperty("waterusage").send(String(waterDiff));
    /* TODO convert diff into volume (milli liter) */
    Serial << "Plant" << lastPumpRunning << ": Water diff " << waterDiff << " mm" << endl;
  }

  readOneWireSensors(true);

  for (int i = 0; i < MAX_PLANTS; i++)
  {
    long raw = mPlants[i].getCurrentMoisture();
    long pct = 100 - map(raw, MOIST_SENSOR_MIN_ADC, MOIST_SENSOR_MAX_ADC, 0, 100);
    if (raw == MISSING_SENSOR)
    {
      pct = 0;
    }
    if (pct < 0)
    {
      pct = 0;
    }
    if (pct > 100)
    {
      pct = 100;
    }

    mPlants[i].setProperty("moist").send(String(pct));
    mPlants[i].setProperty("moistraw").send(String(raw));
  }

  Serial << "W : " << waterRawSensor.getAverage() << " cm (" << String(waterLevelMax.get() - waterRawSensor.getAverage()) << "%)" << endl;
  lastWaterValue = waterRawSensor.getAverage();

  float batteryVoltage = battery.getVoltage(BATTSENSOR_INDEX_BATTERY);
  float chipTemp = battery.getTemperature();
  Serial << "Chip Temperatur " << chipTemp << " °C " << endl;

  if (withHomie)
  {
    sensorWater.setProperty("remaining").send(String(waterLevelMax.get() - waterRawSensor.getAverage()));
    sensorWater.setProperty("distance").send(String(waterRawSensor.getAverage()));
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

  bool hasWater = true; //FIXMEmWaterGone > waterLevelMin.get();
  //FIXME no water warning message
  lastPumpRunning = determineNextPump();
  if (lastPumpRunning != -1 && !hasWater)
  {
    Serial.println("Want to pump but no water");
  }
  else if (lastPumpRunning != -1 && hasWater)
  {
    if (mDownloadMode)
    {
      Serial.println("Mode 3 active, ignoring pump request");
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

  /* Always handle one of the deep sleep duration */
  if (lastPumpRunning == -1 || !hasWater)
  {
    if (mSolarVoltage < SOLAR_CHARGE_MIN_VOLTAGE)
    {
      Serial.print(mSolarVoltage);
      Serial.println("V! No pumps to activate and low light, deepSleepNight");
      espDeepSleepFor(deepSleepNightTime.get(), false, withHomie);
    }
    else
    {
      Serial.println("No pumps to activate, deepSleep");
      espDeepSleepFor(deepSleepTime.get(), false, withHomie);
    }
  }
  else
  {
    Serial.println("Running pump, watering deepsleep");
    espDeepSleepFor(wateringDeepSleep.get(), true, withHomie);
  }
}

/** @}*/

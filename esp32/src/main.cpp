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
#include "driver/pcnt.h"
#include "MQTTUtils.h"

/******************************************************************************
 *                                     DEFINES
******************************************************************************/
#define AMOUNT_SENOR_QUERYS 8
#define MAX_TANK_DEPTH 2000

/******************************************************************************
 *                            FUNCTION PROTOTYPES
******************************************************************************/

int determineNextPump(bool lowLight);
void plantcontrol();
void readPowerSwitchedSensors();
bool determineTimedLightState(bool lowLight);

/******************************************************************************
 *                       NON VOLATILE VARIABLES in DEEP SLEEP
******************************************************************************/

#if defined(TIMED_LIGHT_PIN)
RTC_DATA_ATTR bool timedLightOn = false;                  /**< allow fast recovery after poweron */
RTC_DATA_ATTR bool timedLightLowVoltageTriggered = false; /**remember if it was shut down due to voltage level */
#endif                                                    // TIMED_LIGHT_PIN

RTC_DATA_ATTR long rtcLastWateringPlant[MAX_PLANTS] = {0};
RTC_DATA_ATTR long consecutiveWateringPlant[MAX_PLANTS] = {0};

/******************************************************************************
 *                            LOCAL VARIABLES
******************************************************************************/
bool volatile mDownloadMode = false; /**< Controller must not sleep */
bool volatile mSensorsRead = false;  /**< Sensors are read without Wifi or MQTT */
int volatile pumpToRun = -1;         /** pump to run  at the end of the cycle */
int volatile selfTestPumpRun = -1;   /** pump to run  at the end of the cycle */

bool mConfigured = false;
long nextBlink = 0; /**< Time needed in main loop to support expected blink code */

RunningMedian waterRawSensor = RunningMedian(5);
float mSolarVoltage = 0.0f; /**< Voltage from solar panels */
unsigned long setupFinishedTimestamp;

bool pumpStarted = false;
long pumpTarget = -1;
long lastSendPumpUpdate = 0;
#ifdef FLOWMETER_PIN
long pumpTargetMl = -1;
#endif

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

void espDeepSleep(bool afterPump = false)
{
  if (mDownloadMode)
  {
    log(LOG_LEVEL_DEBUG, "abort deepsleep, DownloadMode active", LOG_DEBUG_CODE);
    return;
  }
  if (aliveWasRead())
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
  esp_sleep_pd_config(ESP_PD_DOMAIN_XTAL, ESP_PD_OPTION_ON);

#if defined(TIMED_LIGHT_PIN)
  gpio_hold_en(TIMED_LIGHT_PIN);
#endif // TIMED_LIGHT_PIN

  long secondsToSleep = -1;

  if (afterPump)
  {
    log(LOG_LEVEL_INFO, "AfterPump Cycle Resume", LOG_SLEEP_CYCLE);
    secondsToSleep = 1;
  }
  else
  {
    if (mSolarVoltage < SOLAR_CHARGE_MIN_VOLTAGE)
    {
      log(LOG_LEVEL_INFO, String(String(mSolarVoltage) + "V! Low light -> deepSleepNight"), LOG_SLEEP_NIGHT);
      secondsToSleep = deepSleepNightTime.get();
    }
    else
    {
      log(LOG_LEVEL_INFO, "Sunny -> deepSleep", LOG_SLEEP_DAY);
      secondsToSleep = deepSleepTime.get();
    }
  }

  esp_sleep_enable_timer_wakeup((secondsToSleep * 1000U * 1000U));
  if (aliveWasRead())
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

    char buf[(sizeof(ds18b20Address) * 2) + 1]; /* additional byte for trailing terminator */
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
        mqttWrite(&sensorTemp, TEMPERATUR_SENSOR_LIPO, String(temp));
        Serial << "Lipo Temperatur " << temp << " °C " << endl;
      }
      if (strcmp(waterSensorAddr.get(), buf) == 0)
      {
        mqttWrite(&sensorTemp, TEMPERATUR_SENSOR_WATER, String(temp));
        Serial << "Water Temperatur " << temp << " °C " << endl;
      }
      /* Always send the sensor address with the temperatur value */
      mqttWrite(&sensorTemp, String(buf), String(temp));
    }
    else
    {
      Serial << "DS18S20 sensor " << String(buf) << " could not be read " << temp << endl;
    }
  }

  battery.updateMultiple();
  mSolarVoltage = battery.getVoltage(BATTSENSOR_INDEX_SOLAR) * SOLAR_VOLT_FACTOR;

  Serial.flush();
}

/**
 * @brief Sensors, that are connected to GPIOs, mandatory for WIFI.
 * These sensors (ADC2) can only be read when no Wifi is used.
 */
void readPowerSwitchedSensors()
{
  for (int i = 0; i < MAX_PLANTS; i++)
  {
    Serial << "Sensor " << i << " mode:  " << mPlants[i].getSensorModeString() << endl;
  }
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
    mPlants[i].blockingMoistureMeasurement();
  }

  for (int i = 0; i < MAX_PLANTS; i++)
  {
    Plant plant = mPlants[i];
    switch (plant.getSensorMode())
    {
      case CAPACITIVE_FREQUENCY: {
        Serial << "Plant " << i << " measurement: " << mPlants[i].getCurrentMoistureRaw() << " hz " << mPlants[i].getCurrentMoisturePCT() << "%" <<  endl;
        break;
      }
      case ANALOG_RESISTANCE_PROBE : {
        Serial << "Plant " << i << " measurement: " << mPlants[i].getCurrentMoistureRaw() << " mV" << mPlants[i].getCurrentMoisturePCT() << "%" <<  endl;
        break;
      }
      case NONE : {

      }
    }
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

    configTime(UTC_OFFSET_DE, UTF_OFFSET_DE_DST, ntpServer.get());
    startMQTTRoundtripTest();
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
    if (!plant.isHydroponic())
    {
      if (equalish(plant.getCurrentMoistureRaw(), MISSING_SENSOR))
      {
        plant.publishState("nosensor");
        log(LOG_LEVEL_ERROR, String(String(i) + " No pump possible: missing sensor"), LOG_MISSING_PUMP);
        continue;
      }
    }

    if (plant.isPumpRequired())
    {
      /* Handle e.g. start = 21, end = 8 */
      if (plant.isHydroponic() || (((plant.getHoursStart() > plant.getHoursEnd()) &&
                                    (getCurrentHour() >= plant.getHoursStart() || getCurrentHour() <= plant.getHoursEnd())) ||
                                   /* Handle e.g. start = 8, end = 21 */
                                   ((plant.getHoursStart() < plant.getHoursEnd()) &&
                                    (getCurrentHour() >= plant.getHoursStart() && getCurrentHour() <= plant.getHoursEnd())) ||
                                   /* no time from NTP received */
                                   (getCurrentTime() < 10000)))
      {
        if (wateralarm)
        {
          plant.publishState("active+alarm");
        }
        else
        {
          plant.publishState("active");
        }

        if (!plant.isHydroponic())
        {
          consecutiveWateringPlant[i]++;
        }

        log(LOG_LEVEL_DEBUG, String(String(i) + " Requested pumping"), LOG_DEBUG_CODE);
        pumpToUse = i;
        return pumpToUse;
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
  return -1;
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
  if (aliveWasRead() && notStarted)
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

void initPumpLogic()
{
  //set targets

#ifdef FLOWMETER_PIN
  pumpTargetMl = mPlants[pumpToRun].getPumpDuration();

  //0-6 are used for moisture measurment
  pcnt_unit_t unit = (pcnt_unit_t)(PCNT_UNIT_7);
  pcnt_config_t pcnt_config = {};                // Instancia PCNT config
  pcnt_config.pulse_gpio_num = FLOWMETER_PIN;    // Configura GPIO para entrada dos pulsos
  pcnt_config.ctrl_gpio_num = PCNT_PIN_NOT_USED; // Configura GPIO para controle da contagem
  pcnt_config.unit = unit;                       // Unidade de contagem PCNT - 0
  pcnt_config.channel = PCNT_CHANNEL_0;          // Canal de contagem PCNT - 0
  pcnt_config.counter_h_lim = INT16_MAX;         // Limite maximo de contagem - 20000
  pcnt_config.pos_mode = PCNT_COUNT_DIS;         // Incrementa contagem na subida do pulso
  pcnt_config.neg_mode = PCNT_COUNT_INC;         // Incrementa contagem na descida do pulso
  pcnt_config.lctrl_mode = PCNT_MODE_KEEP;       // PCNT - modo lctrl desabilitado
  pcnt_config.hctrl_mode = PCNT_MODE_KEEP;       // PCNT - modo hctrl - se HIGH conta incrementando
  pcnt_unit_config(&pcnt_config);                // Configura o contador PCNT

  pcnt_counter_clear(unit); // Zera o contador PCNT
  pcnt_counter_resume(unit);
#endif
  pumpTarget = millis() + (mPlants[pumpToRun].getPumpDuration() * 1000);
  log(LOG_LEVEL_INFO, "Starting pump " + String(pumpToRun) + " for " + String(mPlants[pumpToRun].getPumpDuration()) + "s", LOG_PUMP_STARTED_CODE);

  //enable power
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  digitalWrite(OUTPUT_ENABLE_PUMP, HIGH);
  delay(100);
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 1);

  mPlants[pumpToRun].activatePump();
}

void pumpActiveLoop()
{
  bool targetReached = false;

  if (!pumpStarted)
  {
    initPumpLogic();
    pumpStarted = true;
    rtcLastWateringPlant[pumpToRun] = getCurrentTime();
  }

  bool mqttUpdateTick = false;
  if (lastSendPumpUpdate + 1000 < millis())
  {
    lastSendPumpUpdate = millis();
    mqttUpdateTick = true;
  }

#ifdef FLOWMETER_PIN

  int16_t pulses;
  pcnt_unit_t unit = (pcnt_unit_t)(PCNT_UNIT_7);
  esp_err_t result = pcnt_get_counter_value(unit, &pulses);
  if (result != ESP_OK)
  {
    log(LOG_LEVEL_ERROR, LOG_HARDWARECOUNTER_ERROR_MESSAGE, LOG_HARDWARECOUNTER_ERROR_CODE);
    targetReached = true;
  }
  else
  {
    /**FLOWMETER_FLOWFACTOR * (L/Min) = F; 
    given 1L/min -> FLOWMETER_FLOWFACTOR*60 pulses per liter 
    -> 1000/result -> ml pro pulse;  
    -> result * pulses ->*/
    long pumped = (FLOWMETER_FLOWFACTOR * 60) * pulses / 1000;
    if (pumped >= pumpTargetMl)
    {
      targetReached = true;
      pcnt_counter_pause(unit);
      mPlants[pumpToRun].setProperty("waterusage").send(String(pumped));
    }
    else if (mqttUpdateTick)
    {
      mPlants[pumpToRun].setProperty("waterusage").send(String(pumped));
    }
  }
#endif

  long pumpStarted = pumpTarget - (mPlants[pumpToRun].getPumpDuration() * 1000);
  long duration = millis() - pumpStarted;
  if (millis() > pumpTarget)
  {
    mPlants[pumpToRun].setProperty("watertime").send(String(duration));
    targetReached = true;
  }
  else if (mqttUpdateTick)
  {
    mPlants[pumpToRun].setProperty("watertime").send(String(duration));
  }

  if (targetReached)
  {
    //disable all
    digitalWrite(OUTPUT_ENABLE_PUMP, LOW);
    for (int i = 0; i < MAX_PLANTS; i++)
    {
      mPlants[i].deactivatePump();
    }
    //disable loop, to prevent multi processing
    pumpStarted = false;
    //if runtime is larger than cooldown, else it would run continously
    rtcLastWateringPlant[pumpToRun] = getCurrentTime();
    espDeepSleep(true);
  }
}

void safeSetup()
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

   /************************* Start Homie Framework ***************/
  Homie_setFirmware("PlantControl", FIRMWARE_VERSION);
  Homie.disableLedFeedback();
  Homie_setBrand("PlantControl");
  // Set default values

  //in seconds
  deepSleepTime.setDefaultValue(600).setValidator([](long candidate)
                                                  { return (candidate > 0) && (candidate < (60 * 60 * 2) /** 2h max sleep */); });
  deepSleepNightTime.setDefaultValue(600);
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
                                                 { return (candidate > 0) && (candidate < (24)); });
  timedLightOnlyWhenDark.setDefaultValue(true);
  timedLightVoltageCutoff.setDefaultValue(3.8).setValidator([](double candidate)
                                                            { return (candidate > 3.3) && (candidate < (4.2)); });
#endif // TIMED_LIGHT_PIN

  Homie.setLoopFunction(homieLoop);
  Homie.onEvent(onHomieEvent);

  Homie.setup();

    /* Intialize Plant */
  for (int i = 0; i < MAX_PLANTS; i++)
  {
    mPlants[i].initSensors();
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

  mConfigured = Homie.isConfigured();
  if (mConfigured)
  {
    Serial << "Reading sensors start" << endl;
    Serial.flush();
    readPowerSwitchedSensors();
    Serial << "Reading sensors end" << endl;
    Serial.flush();
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
        espDeepSleep();
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
 * @brief Startup function
 * Is called once, the controller is started
 */
void setup()
{
  try
  {
    safeSetup();
  }
  catch (const std::exception &e)
  {
    Serial.printf("Exception thrown: \"%s\"", e.what());
  }
  catch (...)
  {
    Serial.println("Other exception thrown.");
  }
}

void selfTest()
{

  if (selfTestPumpRun >= 0 && selfTestPumpRun < MAX_PLANTS)
  {
    Serial << "self test mode pump deactivate " << pumpToRun << endl;
    Serial.flush();
    mPlants[selfTestPumpRun].deactivatePump();
  }
  if (selfTestPumpRun >= MAX_PLANTS)
  {
    Serial << "self test finished all pumps, proceed to initial wait mode " << selfTestPumpRun << endl;
    Serial.flush();
    digitalWrite(OUTPUT_ENABLE_PUMP, LOW);
    nextBlink = millis() + 500;
  }
  else
  {
    selfTestPumpRun++;
    nextBlink = millis() + 5000;
  }
  if (selfTestPumpRun < MAX_PLANTS)
  {
    Serial << "self test activating pump " << selfTestPumpRun << endl;
    Serial.flush();
    mPlants[selfTestPumpRun].activatePump();
  }
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
        selfTest();
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

  if (pumpToRun != -1)
  {
    pumpActiveLoop();
  }
}

/***
 * @fn plantcontrol
 * Main function, doing the logic
 */
void plantcontrol()
{
  if (aliveWasRead())
  {
    for (int i = 0; i < MAX_PLANTS; i++)
    {
      mPlants[i].postMQTTconnection();
      mPlants[i].setProperty("consecutivePumps").send(String(consecutiveWateringPlant[i]));
    }
  }

  readOneWireSensors();

  Serial << "W : " << waterRawSensor.getAverage() << " cm (" << String(waterLevelMax.get() - waterRawSensor.getAverage()) << "%)" << endl;

  float batteryVoltage = battery.getVoltage(BATTSENSOR_INDEX_BATTERY);
  float chipTemp = battery.getTemperature();
  Serial << "Chip Temperatur " << chipTemp << " °C " << endl;

  if (aliveWasRead())
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
  bool hasWater = true; //FIXME remaining > waterLevelMin.get();
  //FIXME no water warning message
  pumpToRun = determineNextPump(isLowLight);
  //early aborts
  if (pumpToRun != -1)
  {
    if (hasWater)
    {
      if (mDownloadMode)
      {
        log(LOG_LEVEL_INFO, LOG_PUMP_AND_DOWNLOADMODE, LOG_PUMP_AND_DOWNLOADMODE_CODE);
        pumpToRun = -1;
      }
    }
    else
    {
      log(LOG_LEVEL_ERROR, LOG_PUMP_BUTNOTANK_MESSAGE, LOG_PUMP_BUTNOTANK_CODE);
      pumpToRun = -1;
    }
  }

  // go directly to sleep, skipping the pump loop
  if (pumpToRun == -1)
  {
    espDeepSleep();
  }

#if defined(TIMED_LIGHT_PIN)
  bool shouldLight = determineTimedLightState(isLowLight);
  timedLightOn = shouldLight;
  digitalWrite(TIMED_LIGHT_PIN, shouldLight);
#endif // TIMED_LIGHT_PIN
}

/** @}*/

#ifdef TIMED_LIGHT_PIN
bool determineTimedLightState(bool lowLight)
{
  bool onlyAllowedWhenDark = timedLightOnlyWhenDark.get();
  long hoursStart = timedLightStart.get();
  long hoursEnd = timedLightEnd.get();

  //ntp missing
  if (getCurrentTime() < 10000)
  {
    timedLightNode.setProperty("state").send(String("Off, missing ntp"));
    return false;
  }
  if (onlyAllowedWhenDark && !lowLight)
  {
    timedLightNode.setProperty("state").send(String("Off, not dark"));
    timedLightLowVoltageTriggered = false;
    return false;
  }

  if (((hoursStart > hoursEnd) &&
       (getCurrentHour() >= hoursStart || getCurrentHour() <= hoursEnd)) ||
      /* Handle e.g. start = 8, end = 21 */
      ((hoursStart < hoursEnd) &&
       (getCurrentHour() >= hoursStart && getCurrentHour() <= hoursEnd)))
  {
    if (!timedLightLowVoltageTriggered && battery.getVoltage(BATTSENSOR_INDEX_BATTERY) >= timedLightVoltageCutoff.get())
    {
      timedLightNode.setProperty("state").send(String("On"));
      return true;
    }
    else
    {
      timedLightNode.setProperty("state").send(String("Off, due to missing voltage"));
      timedLightLowVoltageTriggered = true;
      return false;
    }
  }
  else
  {
    timedLightNode.setProperty("state").send(String("Off, outside worktime"));
    return false;
  }
}

#endif

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

/******************************************************************************
 *                                     DEFINES
******************************************************************************/
#define AMOUNT_SENOR_QUERYS 8
#define MAX_TANK_DEPTH 1000

/******************************************************************************
 *                            FUNCTION PROTOTYPES
******************************************************************************/

int determineNextPump();
//void setLastActivationForPump(int pumpId, long time);
int readTemp();

/******************************************************************************
 *                       NON VOLATILE VARIABLES in DEEP SLEEP
******************************************************************************/

//only relevant if mode2 did start pumping before
RTC_DATA_ATTR int lastPumpRunning = 0;
RTC_DATA_ATTR long lastWaterValue = 0;

RTC_DATA_ATTR int gBootCount = 0;

//FIXME use -1 and configure properly
RTC_DATA_ATTR int rtcLipoTempIndex = 0;
RTC_DATA_ATTR int rtcWaterTempIndex = -1;

/******************************************************************************
 *                            LOCAL VARIABLES
******************************************************************************/
bool volatile mode3Active = false; /**< Controller must not sleep */
bool volatile mDeepsleep = false;

int readCounter = 0;
bool mConfigured = false;
long nextBlink = 0; /**< Time needed in main loop to support expected blink code */

RunningMedian waterRawSensor = RunningMedian(5);
float mTempLipo = 0.0f;
float mTempWater = 0.0f;
float mBatteryVoltage = 0.0f;
float mSolarVoltage = 0.0f;
float mChipTemp = 0.0f;

/*************************** Hardware abstraction *****************************/

OneWire oneWire(SENSOR_ONEWIRE);
DallasTemperature sensors(&oneWire);
DS2438 battery(&oneWire, 0.1f);

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

void espDeepSleepFor(long seconds, bool activatePump = false)
{
  if (mode3Active)
  {
    Serial << "abort deepsleep, mode3Active" << endl;
    return;
  }
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

  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_ON);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_ON);
  if (activatePump)
  {
    esp_sleep_pd_config(ESP_PD_DOMAIN_XTAL, ESP_PD_OPTION_ON);
    gpio_deep_sleep_hold_en();
    gpio_hold_en(GPIO_NUM_13); //pump pwr
  }
  else
  {
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
    gpio_hold_dis(GPIO_NUM_13); //pump pwr
    gpio_deep_sleep_hold_dis();
    digitalWrite(OUTPUT_ENABLE_PUMP, LOW);
    digitalWrite(OUTPUT_ENABLE_SENSOR, LOW);
    for (int i = 0; i < MAX_PLANTS; i++)
    {
      mPlants[i].deactivatePump();
    }
  }
  //gpio_hold_en(GPIO_NUM_23); //p0
  //FIXME fix for outher outputs

  Serial.print("Trying to sleep for ");
  Serial.print(seconds);
  Serial.println(" seconds");
  esp_sleep_enable_timer_wakeup((seconds * 1000U * 1000U));

  mDeepsleep = true;
}

void mode2MQTT()
{
  digitalWrite(OUTPUT_ENABLE_PUMP, LOW);
  for (int i = 0; i < MAX_PLANTS; i++)
  {
    mPlants[i].deactivatePump();
  }

  if (lastPumpRunning != -1)
  {
    //long waterDiff = waterRawSensor.getAverage() - lastWaterValue;
    //TODO attribute used water in ml to plantid
  }
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
  sensorWater.setProperty("remaining").send(String(waterLevelMax.get() - waterRawSensor.getAverage()));
  Serial << "W : " << waterRawSensor.getAverage() << " cm (" << String(waterLevelMax.get() - waterRawSensor.getAverage()) << "%)" << endl;
  lastWaterValue = waterRawSensor.getAverage();

  sensorLipo.setProperty("percent").send(String(100 * mBatteryVoltage / VOLT_MAX_BATT));
  sensorLipo.setProperty("volt").send(String(mBatteryVoltage));
  sensorLipo.setProperty("current").send(String(battery.getCurrent()));
  sensorLipo.setProperty("Ah").send(String(battery.getAh()));
  sensorLipo.setProperty("ICA").send(String(battery.getICA()));
  sensorLipo.setProperty("DCA").send(String(battery.getDCA()));
  sensorLipo.setProperty("CCA").send(String(battery.getCCA()));
  sensorSolar.setProperty("volt").send(String(mSolarVoltage));

  rtcLipoTempIndex = lipoSensorIndex.get();
  rtcWaterTempIndex = waterSensorIndex.get();

  sensorTemp.setProperty(TEMPERATUR_SENSOR_LIPO).send(String(mTempLipo));
  Serial << "Lipo Temperatur " << mTempLipo << " °C " << endl;

  sensorTemp.setProperty(TEMPERATUR_SENSOR_WATER).send(String(mTempWater));
  Serial << "Water Temperatur " << mTempWater << " °C " << endl;

  sensorTemp.setProperty(TEMPERATUR_SENSOR_CHIP).send(String(mChipTemp));
  Serial << "Chip Temperatur " << mChipTemp << " °C " << endl;

  bool hasWater = true; //FIXMEmWaterGone > waterLevelMin.get();
  //FIXME no water warning message
  lastPumpRunning = determineNextPump();
  if (lastPumpRunning != -1 && !hasWater)
  {
    Serial.println("Want to pump but no water");
  }
  if (lastPumpRunning != -1 && hasWater)
  {
    if (mode3Active)
    {
      Serial.println("Mode 3 active, ignoring pump request");
    }
    else
    {
      digitalWrite(OUTPUT_ENABLE_PUMP, HIGH);
      //TODO setLastActivationForPump(lastPumpRunning, getCurrentTime());
      mPlants[lastPumpRunning].activatePump();
    }
  }

  if (lastPumpRunning == -1 || !hasWater)
  {
    if (mSolarVoltage < SOLAR_CHARGE_MIN_VOLTAGE)
    {
      Serial.print(mSolarVoltage);
      Serial.println("V! No pumps to activate and low light, deepSleepNight");
      espDeepSleepFor(deepSleepNightTime.get());
    }
    else
    {
      Serial.println("No pumps to activate, deepSleep");
      espDeepSleepFor(deepSleepTime.get());
    }
  }
  else
  {
    Serial.println("Running pump, watering deepsleep");
    espDeepSleepFor(wateringDeepSleep.get(), true);
  }
}

/**
 * @brief Read ultra sensor JSN-SR04T-2.0
 * Read the distance of the water level.
 */
void readDistance()
{
  for (int i = 0; i < AMOUNT_SENOR_QUERYS; i++)
  {
    unsigned long duration = 0;
  
    digitalWrite(SENSOR_TANK_TRG, HIGH);
    delayMicroseconds(20);
    cli();
    digitalWrite(SENSOR_TANK_TRG, LOW);
    duration = pulseIn(SENSOR_TANK_ECHO, HIGH);
    sei();

    int mmDis = duration * 0.3432 / 2; 
    if(mmDis > MAX_TANK_DEPTH){
      waterRawSensor.add(0);
    } else {
      waterRawSensor.add(mmDis);
    }
  }
}

/**
 * @brief Sensors, that are connected to GPIOs, mandatory for WIFI.
 * These sensors (ADC2) can only be read when no Wifi is used.
 */
void readSensors()
{
  Serial << "Read Sensors" << endl;
    /* activate all sensors */
  digitalWrite(OUTPUT_ENABLE_SENSOR, HIGH);
  /* wait before reading something */
  delay(20);

  int timeoutTemp = millis() + TEMPERATUR_TIMEOUT;
  int sensorCount = 0;

  /* Required to read the temperature at least once */
  while ((sensorCount == 0 || !battery.isFound()) && millis() < timeoutTemp)
  {
    sensors.begin();
    battery.begin();
    sensorCount = sensors.getDS18Count();
    delay(50);
  }

  Serial << "One wire count: " << sensorCount << " found in " << (millis() - timeoutTemp) << "ms" << endl;
  /* Measure temperature */
  if (sensorCount > 0)
  {
    sensors.requestTemperatures();
  }

  for (int i = 0; i < sensorCount; i++) {
    float temp = sensors.getTempCByIndex(i);
    Serial << "OneWire sensor " << i << " has value " << temp << endl;
    if (rtcWaterTempIndex != -1 && rtcWaterTempIndex == i) {
      mTempWater = temp;
    }
    if (rtcLipoTempIndex != -1 && rtcLipoTempIndex == i) {
      mTempLipo = temp;
    }
  }

  // Update battery chip data
  battery.update();
  mSolarVoltage = battery.getVoltage(BATTSENSOR_INDEX_SOLAR) * SOLAR_VOLT_FACTOR;
  mBatteryVoltage = battery.getVoltage(BATTSENSOR_INDEX_BATTERY);
  mChipTemp = battery.getTemperature();
  //  if(mBatteryVoltage < MINIMUM_LIPO_VOLT){
  //    Serial.println("Low lipo voltage, abort high level processing");
  //  }

  for (int readCnt = 0; readCnt < AMOUNT_SENOR_QUERYS; readCnt++)
  {
    for (int i = 0; i < MAX_PLANTS; i++)
    {
      mPlants[i].addSenseValue();
    }
    delay(10);
  }

  /* Read the distance and give the temperature sensors some time */
  readDistance();
  Serial << "Distance sensor " << waterRawSensor.getAverage() << " cm" << endl;

  /* deactivate the sensors */
  digitalWrite(OUTPUT_ENABLE_SENSOR, LOW);
}

void onHomieEvent(const HomieEvent &event)
{
  switch (event.type)
  {
  case HomieEventType::SENDING_STATISTICS:
    Homie.getLogger() << "My statistics" << endl;
    break;
  case HomieEventType::MQTT_READY:
    Serial.printf("NTP Setup with server %s\r\n", ntpServer.get());
    configTime(0, 0, ntpServer.get());

    Serial << "Setup plants" << endl;
    for (int i = 0; i < MAX_PLANTS; i++)
    {
      mPlants[i].postMQTTconnection();
    }

    mode2MQTT();
    break;
  case HomieEventType::OTA_STARTED:
    Homie.getLogger() << "OTA started" << endl;
    digitalWrite(OUTPUT_ENABLE_SENSOR, HIGH);
    digitalWrite(OUTPUT_ENABLE_PUMP, HIGH);
    gpio_hold_dis(GPIO_NUM_13); //pump pwr
    gpio_deep_sleep_hold_dis();
    for (int i = 0; i < MAX_PLANTS; i++)
    {
      mPlants[i].deactivatePump();
    }
    mode3Active = true;
    break;
  case HomieEventType::OTA_SUCCESSFUL:
    Homie.getLogger() << "OTA successfull" << endl;
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
  bool isLowLight = (mSolarVoltage > SOLAR_CHARGE_MIN_VOLTAGE || mSolarVoltage < SOLAR_CHARGE_MAX_VOLTAGE);

  //FIXME instead of for, use sorted by last activation index to ensure equal runtime?

  int pumpToUse = -1;
  for (int i = 0; i < MAX_PLANTS; i++)
  {
    Plant plant = mPlants[i];
    //TODO skip pump last used here!
    //if (plant.isInCooldown(sinceLastActivation))
    //{
    //      Serial.printf("%d Skipping due to cooldown %ld / %ld \r\n", i, sinceLastActivation, plant.getCooldownInSeconds());
    //continue;
    //}
    //skip as it is not low light
    if (!isLowLight && plant.isAllowedOnlyAtLowLight())
    {
      Serial.printf("%d No pump required: due to light\r\n", i);
      continue;
    }
    if (plant.getCurrentMoisture() == MISSING_SENSOR && plant.isPumpTriggerActive())
    {
      Serial.printf("%d No pump possible: missing sensor \r\n", i);
      continue;
    }
    if (plant.isPumpRequired())
    {
      Serial.printf("%d Requested pumping\r\n", i);
      pumpToUse = i;
    }
    else if (plant.isPumpTriggerActive())
    {
      Serial.printf("%d No pump required: moisture acceptable %f / %ld\r\n", i, plant.getCurrentMoisture(), plant.getSettingsMoisture());
    }
    else
    {
      Serial.printf("%d No pump required: disabled pump trigger \r\n", i);
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
  if (value.equals("ON") || value.equals("On") || value.equals("1"))
  {
    mode3Active = true;
  }
  else
  {
    mode3Active = false;
  }

  return true;
}

void homieLoop()
{
}

void systemInit()
{
  WiFi.mode(WIFI_STA);

  Homie_setFirmware("PlantControl", FIRMWARE_VERSION);

  // Set default values

  //in seconds
  maxTimeBetweenMQTTUpdates.setDefaultValue(700);
  deepSleepTime.setDefaultValue(600);
  deepSleepNightTime.setDefaultValue(600);
  wateringDeepSleep.setDefaultValue(5);
  ntpServer.setDefaultValue("pool.ntp.org");

  /* waterLevelMax 1000    */          /* 100cm in mm */
  waterLevelMin.setDefaultValue(50);   /* 5cm in mm */
  waterLevelWarn.setDefaultValue(500); /* 50cm in mm */
  waterLevelVol.setDefaultValue(5000); /* 5l in ml */
  lipoSensorIndex.setDefaultValue(0);
  waterSensorIndex.setDefaultValue(-1);
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
    startupReason.advertise("startupReason").setDatatype(NUMBER_TYPE).setUnit("Enum");
  }
  stayAlive.advertise("alive").setName("Alive").setDatatype(NUMBER_TYPE).settable(aliveHandler);
}

void mode2()
{
  Serial.println("==== Mode 2 ====");
  systemInit();

  /* Jump into Mode 3, if not configured */
  if (!mConfigured)
  {
    Serial.println("==== Mode 3 ====");
    mode3Active = true;
  }
}

/**
 * @brief Startup function
 * Is called once, the controller is started
 */
void setup()
{
  Serial.begin(115200);
  Serial << endl
         << endl;
  /* Intialize Plant */
  for (int i = 0; i < MAX_PLANTS; i++)
  {
    mPlants[i].init();
  }

  // read button
  pinMode(BUTTON, INPUT);

  // Power pins
  pinMode(OUTPUT_ENABLE_PUMP, OUTPUT);
  pinMode(OUTPUT_ENABLE_SENSOR, OUTPUT);

  // Individual Pump pins

  /* Disable Wifi and bluetooth */
  WiFi.mode(WIFI_OFF);

  if (HomieInternals::MAX_CONFIG_SETTING_SIZE < MAX_CONFIG_SETTING_ITEMS)
  {
    //increase the config settings to 50 and the json to 3000
    Serial << "Limits.hpp" << endl;
  }

  readSensors();

  // Big TODO use here the settings in RTC_Memory

  //Panik mode, the Lipo is empty, sleep a long long time:
  //  if ((mBatteryVoltage < MINIMUM_LIPO_VOLT) &&
  //      (mBatteryVoltage > NO_LIPO_VOLT))
  //  {
  //    Serial << PANIK_MODE_DEEPSLEEP << " s lipo " << mBatteryVoltage << "V" << endl;
  //    esp_sleep_enable_timer_wakeup(PANIK_MODE_DEEPSLEEP_US);
  //    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
  //    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
  //    esp_sleep_pd_config(ESP_PD_DOMAIN_XTAL, ESP_PD_OPTION_ON);
  //    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
  //    esp_deep_sleep_start();
  //  }
  mode2();
}

/**
 * @brief Cyclic call
 * Executs the Homie base functionallity or triggers sleeping, if requested.
 */
void loop()
{
  /* Toggel Senor LED to visualize mode 3 */
  if (mode3Active)
  {
    if (nextBlink < millis())
    {
      nextBlink = millis() + 500;
      digitalWrite(OUTPUT_ENABLE_SENSOR, !digitalRead(OUTPUT_ENABLE_SENSOR));
    }
  }
  else if (!mDeepsleep)
  {
    Homie.loop();
  }
  else
  {
    Serial << "Bye" << endl;
    Serial.flush();
    esp_deep_sleep_start();
  }

  if (millis() > 30000 && !mode3Active)
  {
    Serial << (millis() / 1000) << "not terminated watchdog reset" << endl;
    Serial.flush();
    esp_restart();
  }
}

/** @}*/

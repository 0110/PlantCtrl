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
#include "PlantCtrl.h"
#include "ControllerConfiguration.h"
#include "HomieConfiguration.h"
#include "DS18B20.h"
#include <Homie.h>
#include "time.h"
#include "esp_sleep.h"
#include "RunningMedian.h"
#include <stdint.h>
#include <math.h>

const unsigned long TEMPREADCYCLE = 30000; /**< Check temperature all half minutes */

#define AMOUNT_SENOR_QUERYS 8
#define SENSOR_QUERY_SHIFTS 3
#define SOLAR4SENSORS 6.0f
#define TEMP_INIT_VALUE -999.0f
#define TEMP_MAX_VALUE 85.0f
#define HalfHour 60

typedef struct
{
  long lastActive;   /**< Timestamp, a pump was activated */
  long moistTrigger; /**< Trigger value of the moist sensor */
  long moisture;     /**< last measured moist value */

} rtc_plant_t;

/********************* non volatile enable after deepsleep *******************************/
RTC_DATA_ATTR rtc_plant_t rtcPlant[7];
RTC_DATA_ATTR long gotoMode2AfterThisTimestamp = 0;
RTC_DATA_ATTR long rtcDeepSleepTime = 0; /**< Time, when the microcontroller shall be up again */
RTC_DATA_ATTR int lastPumpRunning = 0;
RTC_DATA_ATTR long lastWaterValue = 0;
RTC_DATA_ATTR float rtcLastTemp1 = 0.0f;
RTC_DATA_ATTR float rtcLastTemp2 = 0.0f;
RTC_DATA_ATTR int gBootCount = 0;
RTC_DATA_ATTR int gCurrentPlant = 0; /**< Value Range: 1 ... 7 (0: no plant needs water) */

bool warmBoot = true;
bool volatile mode3Active = false; /**< Controller must not sleep */
bool volatile mDeepsleep = false;

int plantSensor1 = 0;

int mWaterGone = -1; /**< Amount of centimeter, where no water is seen */
int readCounter = 0;
bool mConfigured = false;

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
    Plant(SENSOR_PLANT6, OUTPUT_PUMP6, 6, &plant6, &mSetting6)};

float getBatteryVoltage()
{
  return ADC_5V_TO_3V3(lipoRawSensor.getAverage());
}

float getSolarVoltage()
{
  return SOLAR_VOLT(solarRawSensor.getAverage());
}

void setMoistureTrigger(int plantId, long value)
{
  if ((plantId >= 0) && (plantId < MAX_PLANTS))
  {
    rtcPlant[plantId].moistTrigger = value;
  }
}

void setLastMoisture(int plantId, long value)
{
  if ((plantId >= 0) && (plantId < MAX_PLANTS))
  {
    rtcPlant[plantId].moisture = value;
  }
}

long getLastMoisture(int plantId)
{
  if ((plantId >= 0) && (plantId < MAX_PLANTS))
  {
    return rtcPlant[plantId].moisture;
  }
  else
  {
    return -1;
  }
}

void readSystemSensors()
{
  lipoRawSensor.add(analogRead(SENSOR_LIPO));
  solarRawSensor.add(analogRead(SENSOR_SOLAR));
}

int determineNextPump();
void setLastActivationForPump(int pumpId, long time);

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
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_XTAL, ESP_PD_OPTION_ON);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_ON);
  if (activatePump)
  {
    gpio_deep_sleep_hold_en();
    gpio_hold_en(GPIO_NUM_13); //pump pwr
  }
  else
  {
    gpio_hold_dis(GPIO_NUM_13); //pump pwr
    gpio_deep_sleep_hold_dis();
    digitalWrite(OUTPUT_PUMP, LOW);
    digitalWrite(OUTPUT_SENSOR, LOW);
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
  readSystemSensors();

  digitalWrite(OUTPUT_PUMP, LOW);
  for (int i = 0; i < MAX_PLANTS; i++)
  {
    mPlants[i].deactivatePump();
  }

  if (deepSleepTime.get())
  {
    Serial << "deepsleep time is configured to " << deepSleepTime.get() << endl;
  }
  /* Publish default values */

  if (lastPumpRunning != -1)
  {
    long waterDiff = mWaterGone - lastWaterValue;
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
  sensorWater.setProperty("remaining").send(String(waterLevelMax.get() - mWaterGone));
  Serial << "W : " << mWaterGone << " cm (" << String(waterLevelMax.get() - mWaterGone) << "%)" << endl;
  lastWaterValue = mWaterGone;

  sensorLipo.setProperty("percent").send(String(100 * lipoRawSensor.getAverage() / 4095));
  sensorLipo.setProperty("volt").send(String(getBatteryVoltage()));
  sensorSolar.setProperty("percent").send(String((100 * solarRawSensor.getAverage()) / 4095));
  sensorSolar.setProperty("volt").send(String(getSolarVoltage()));

  float t1 = temp1.getMedian();
  if (t1 != NAN)
  {
    sensorTemp.setProperty("control").send(String(t1));
  }
  float t2 = temp2.getMedian();
  if (t2 != NAN)
  {
    sensorTemp.setProperty("temp").send(String(t2));
  }

  //give mqtt time, use via publish callback instead?
  delay(100);

  bool lipoTempWarning = t1 != 85 && t2 != 85 && abs(t1 - t2) > 10;
  if (lipoTempWarning)
  {
    Serial.println("Lipo temp incorrect, panic mode deepsleep TODO");
    //espDeepSleepFor(PANIK_MODE_DEEPSLEEP);
    //return;
  }

  for (int i = 0; i < MAX_PLANTS; i++)
  {
    setMoistureTrigger(i, mPlants[i].mSetting->pSensorDry->get());
  }

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
      digitalWrite(OUTPUT_PUMP, HIGH);
      setLastActivationForPump(lastPumpRunning, getCurrentTime());
      mPlants[lastPumpRunning].activatePump();
    }
  }
  if (lastPumpRunning == -1 || !hasWater)
  {
    if (getSolarVoltage() < SOLAR_CHARGE_MIN_VOLTAGE)
    {
      gotoMode2AfterThisTimestamp = getCurrentTime() + maxTimeBetweenMQTTUpdates.get();
      Serial.println("No pumps to activate and low light, deepSleepNight");
      espDeepSleepFor(deepSleepNightTime.get());
      rtcDeepSleepTime = deepSleepNightTime.get();
    }
    else
    {
      gotoMode2AfterThisTimestamp = getCurrentTime() + maxTimeBetweenMQTTUpdates.get();
      Serial.println("No pumps to activate, deepSleep");
      espDeepSleepFor(deepSleepTime.get());
      rtcDeepSleepTime = deepSleepTime.get();
    }
  }
  else
  {
    gotoMode2AfterThisTimestamp = 0;
    Serial.println("Running pump, watering deepsleep");
    espDeepSleepFor(wateringDeepSleep.get(), true);
  }
}

long getMoistureTrigger(int plantId)
{
  if ((plantId >= 0) && (plantId < MAX_PLANTS))
  {
    return rtcPlant[plantId].moistTrigger;
  }
  else
  {
    return -1;
  }
}

void setLastActivationForPump(int plantId, long value)
{
  if ((plantId >= 0) && (plantId < MAX_PLANTS))
  {
    rtcPlant[plantId].lastActive = value;
  }
}

long getLastActivationForPump(int plantId)
{
  if ((plantId >= 0) && (plantId < MAX_PLANTS))
  {
    return rtcPlant[plantId].lastActive;
  }
  else
  {
    return -1;
  }
}

/**
 * @brief Sensors, that are connected to GPIOs, mandatory for WIFI.
 * These sensors (ADC2) can only be read when no Wifi is used.
 */
bool readSensors()
{
  float temp[2] = {TEMP_MAX_VALUE, TEMP_MAX_VALUE};
  float *pFloat = temp;
  bool leaveMode1 = false;
  Serial << "Read Sensors" << endl;

  readSystemSensors();

  /* activate all sensors */
  pinMode(OUTPUT_SENSOR, OUTPUT);
  digitalWrite(OUTPUT_SENSOR, HIGH);

  delay(20);
  /* wait before reading something */
  for (int readCnt = 0; readCnt < AMOUNT_SENOR_QUERYS; readCnt++)
  {
    for (int i = 0; i < MAX_PLANTS; i++)
    {
      mPlants[i].addSenseValue();
    }
    delay(10);
  }
  for (int i = 0; i < MAX_PLANTS; i++)
  {
    long current = mPlants[i].getCurrentMoisture();
    long delta = abs(getLastMoisture(i) - current);
    bool tmp = (delta > MOIST_DELTA_TRIGGER_ADC);
    setLastMoisture(i, current);
    if (tmp)
    {
      leaveMode1 = true;
      Serial.printf("Mode2 start due to moist delta in plant %d with %ld \r\n", i, delta);
    }
  }

  Serial << "DS18B20" << endl;
  /* Read the temperature sensors once, as first time 85 degree is returned */
  Serial << "DS18B20" << String(dallas.readDevices()) << endl;
  delay(200);

  /* Required to read the temperature once */
  for (int i = 0; i < 5; i++)
  {
    int sensors = dallas.readAllTemperatures(pFloat, 2);
    if (sensors > 0)
    {
      Serial << "t1: " << String(temp[0]) << endl;
      temp1.add(temp[0]);
    }
    if (sensors > 1)
    {
      Serial << "t2: " << String(temp[1]) << endl;
      temp2.add(temp[1]);
    }
    delay(50);
  }

  if ((temp1.getAverage() - rtcLastTemp1 > TEMPERATURE_DELTA_TRIGGER_IN_C) ||
    (rtcLastTemp1 - temp1.getAverage() > TEMPERATURE_DELTA_TRIGGER_IN_C)) {
    leaveMode1 = true;
  }
  if ((temp2.getAverage() - rtcLastTemp2 > TEMPERATURE_DELTA_TRIGGER_IN_C) ||
    (rtcLastTemp2 - temp2.getAverage() > TEMPERATURE_DELTA_TRIGGER_IN_C)) {
    leaveMode1 = true;
  }

  rtcLastTemp1 = temp1.getAverage();
  rtcLastTemp2 = temp2.getAverage();

  /* Use the Ultrasonic sensor to measure waterLevel */
  digitalWrite(SENSOR_SR04_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(SENSOR_SR04_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(SENSOR_SR04_TRIG, LOW);
  float duration = pulseIn(SENSOR_SR04_ECHO, HIGH);
  waterRawSensor.add((duration * .343) / 2);
  /* deactivate the sensors */
  digitalWrite(OUTPUT_SENSOR, LOW);
  return leaveMode1;
}

//Homie.getMqttClient().disconnect();

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
    //wait for rtc sync?
    rtcDeepSleepTime = deepSleepTime.get();
    Serial << "Setup plants" << endl;
    for (int i = 0; i < MAX_PLANTS; i++)
    {
      mPlants[i].postMQTTconnection();
    }

    mode2MQTT();
    break;
  case HomieEventType::READY_TO_SLEEP:
    Homie.getLogger() << "rtsleep" << endl;
    esp_deep_sleep_start();
    break;
  case HomieEventType::OTA_STARTED:
    Homie.getLogger() << "OTA started" << endl;
    digitalWrite(OUTPUT_SENSOR, HIGH);
    digitalWrite(OUTPUT_PUMP, HIGH);
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
    digitalWrite(OUTPUT_SENSOR, LOW);
    digitalWrite(OUTPUT_PUMP, LOW);
    ESP.restart();
    break;
  default:
    break;
  }
}

int determineNextPump()
{
  float solarValue = getSolarVoltage();
  bool isLowLight = (solarValue > SOLAR_CHARGE_MIN_VOLTAGE || solarValue < SOLAR_CHARGE_MAX_VOLTAGE);

  //FIXME instead of for, use sorted by last activation index to ensure equal runtime?

  int pumpToUse = -1;
  for (int i = 0; i < MAX_PLANTS; i++)
  {
    Plant plant = mPlants[i];
    long lastActivation = getLastActivationForPump(i);
    long sinceLastActivation = getCurrentTime() - lastActivation;
    //this pump is in cooldown skip it and disable low power mode trigger for it
    if (plant.isInCooldown(sinceLastActivation))
    {
      Serial.printf("%d Skipping due to cooldown %ld / %ld \r\n", i, sinceLastActivation, plant.getCooldownInSeconds());
      setMoistureTrigger(i, DEACTIVATED_PLANT);
      continue;
    }
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
  maxTimeBetweenMQTTUpdates.setDefaultValue(120);
  deepSleepTime.setDefaultValue(60);
  deepSleepNightTime.setDefaultValue(600);
  wateringDeepSleep.setDefaultValue(5);
  ntpServer.setDefaultValue("pool.ntp.org");

  /* waterLevelMax 1000    */          /* 100cm in mm */
  waterLevelMin.setDefaultValue(50);   /* 5cm in mm */
  waterLevelWarn.setDefaultValue(500); /* 50cm in mm */
  waterLevelVol.setDefaultValue(5000); /* 5l in ml */

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
  stayAlive.advertise("alive").setName("Alive").setDatatype("number").settable(aliveHandler);
}

bool mode1()
{
  Serial.println("==== Mode 1 ====");
  Serial << getCurrentTime() << " curtime" << endl;

  bool deltaTrigger = readSensors();
  //queue sensor values for

  if (deltaTrigger)
  {
    Serial.println("1 delta triggered, going to mode2");
    return true;
  }
  if (rtcDeepSleepTime == 0)
  {
    Serial.println("1 missing rtc value, going to mode2");
    return true;
  }
  for (int i = 0; i < MAX_PLANTS; i++)
  {
    long trigger = getMoistureTrigger(i);
    if (trigger == 0)
    {
      Serial << "Missing rtc trigger " << i << endl;
      return true;
    }
    if (trigger == DEACTIVATED_PLANT)
    {
      continue;
    }
    long raw = mPlants[i].getCurrentMoisture();
    if (raw == MISSING_SENSOR)
    {
      continue;
    }
    if (raw > trigger)
    {
      Serial << "plant " << i << " dry " << raw << " / " << trigger << " starting mode 2" << endl;
      return true;
    }
  }

  //check how long it was already in mode1 if to long goto mode2

  long cTime = getCurrentTime();
  if (cTime < 100000)
  {
    Serial.println("Starting mode 2 due to missing ntp");
    //missing ntp time boot to mode3
    return true;
  }
  if (gotoMode2AfterThisTimestamp < cTime)
  {
    Serial.println("Starting mode 2 after specified mode1 time");
    return true;
  }
  else
  {
    Serial << "Mode2 Timer " << gotoMode2AfterThisTimestamp << " curtime " << cTime << endl;
  }
  return false;
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
  Serial.setTimeout(1000); // Set timeout of 1 second
  Serial << endl
         << endl;
  /* Intialize Plant */
  for (int i = 0; i < MAX_PLANTS; i++)
  {
    mPlants[i].init();
  }

  /* Intialize inputs and outputs */
  pinMode(SENSOR_LIPO, ANALOG);
  pinMode(SENSOR_SOLAR, ANALOG);
  /* read button */
  pinMode(BUTTON, INPUT);

  /* Power pins */
  pinMode(OUTPUT_PUMP, OUTPUT);

  /* Disable Wifi and bluetooth */
  WiFi.mode(WIFI_OFF);

  if (HomieInternals::MAX_CONFIG_SETTING_SIZE < MAX_CONFIG_SETTING_ITEMS)
  {
    //increase the config settings to 50 and the json to 3000
    Serial << "Limits.hpp" << endl;
  }

  // Big TODO use here the settings in RTC_Memory

  //Panik mode, the Lipo is empty, sleep a long long time:
  if ((getBatteryVoltage() < MINIMUM_LIPO_VOLT) &&
      (getBatteryVoltage() > NO_LIPO_VOLT))
  {
    Serial << PANIK_MODE_DEEPSLEEP << " s lipo " << getBatteryVoltage() << "V" << endl;
    esp_sleep_enable_timer_wakeup(PANIK_MODE_DEEPSLEEP_US);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_XTAL, ESP_PD_OPTION_ON);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
    esp_deep_sleep_start();
  }

  if (mode1())
  {
    mode2();
  }
  else
  {
    Serial.println("nop");
    espDeepSleepFor(rtcDeepSleepTime);
  }
}

/**
 * @brief Cyclic call
 * Executs the Homie base functionallity or triggers sleeping, if requested.
 */
long nextBlink = 0;
void loop()
{
  if (!mDeepsleep || mode3Active)
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
    Serial << (millis() / 1000) << "not terminated watchdog putting to sleep" << endl;
    Serial.flush();
    espDeepSleepFor(rtcDeepSleepTime);
  }

  /* Toggel Senor LED to visualize mode 3 */
  if (mode3Active)
  {
    if (nextBlink < millis())
    {
      nextBlink = millis() + 500;
      digitalWrite(OUTPUT_SENSOR, !digitalRead(OUTPUT_SENSOR));
    }
  }
}

/** @}*/
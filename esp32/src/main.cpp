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

/******************************************************************************
 *                                     DEFINES
******************************************************************************/
#define AMOUNT_SENOR_QUERYS 8
#define SENSOR_QUERY_SHIFTS 3
#define SOLAR4SENSORS 6.0f
#define TEMP_INIT_VALUE -999.0f
#define TEMP_MAX_VALUE 85.0f
#define HalfHour 60

/******************************************************************************
 *                                     TYPE DEFS
******************************************************************************/
typedef struct
{
  long lastActive;   /**< Timestamp, a pump was activated */
  long moistTrigger; /**< Trigger value of the moist sensor */
  long moisture;     /**< last measured moist value */

} rtc_plant_t;

/******************************************************************************
 *                            FUNCTION PROTOTYPES
******************************************************************************/

int determineNextPump();
void setLastActivationForPump(int pumpId, long time);

/******************************************************************************
 *                       NON VOLATILE VARIABLES in DEEP SLEEP
******************************************************************************/

RTC_DATA_ATTR rtc_plant_t rtcPlant[MAX_PLANTS];
RTC_DATA_ATTR long gotoMode2AfterThisTimestamp = 0;
RTC_DATA_ATTR long rtcDeepSleepTime = 0; /**< Time, when the microcontroller shall be up again */
RTC_DATA_ATTR int lastPumpRunning = 0;
RTC_DATA_ATTR long lastWaterValue = 0;
RTC_DATA_ATTR float rtcLastLipoTemp = 0.0f;
RTC_DATA_ATTR float rtcLastWaterTemp = 0.0f;
RTC_DATA_ATTR float rtcLastBatteryVoltage = 0.0f;
RTC_DATA_ATTR float rtcLastSolarVoltage = 0.0f;
RTC_DATA_ATTR int gBootCount = 0;
RTC_DATA_ATTR int gCurrentPlant = 0; /**< Value Range: 1 ... 7 (0: no plant needs water) */
RTC_DATA_ATTR int rtcLipoTempIndex = -1;
RTC_DATA_ATTR int rtcWaterTempIndex = -1;

/******************************************************************************
 *                            LOCAL VARIABLES
******************************************************************************/
const unsigned long TEMPREADCYCLE = 30000; /**< Check temperature all half minutes */

int wakeUpReason = WAKEUP_REASON_UNDEFINED;
bool volatile mode3Active = false; /**< Controller must not sleep */
bool volatile mDeepsleep = false;

int readCounter = 0;
bool mConfigured = false;
long nextBlink = 0;         /**< Time needed in main loop to support expected blink code */

RunningMedian lipoRawSensor = RunningMedian(5);
RunningMedian solarRawSensor = RunningMedian(5);
RunningMedian waterRawSensor = RunningMedian(5);
RunningMedian lipoTempSensor = RunningMedian(TEMP_SENSOR_MEASURE_SERIES);
RunningMedian waterTempSensor = RunningMedian(TEMP_SENSOR_MEASURE_SERIES);

OneWire oneWire(SENSOR_DS18B20);
DallasTemperature sensors(&oneWire);

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

long getDistance()
{
  byte startByte, h_data, l_data, sum;
  byte buf[3];

  startByte = (byte)Serial.read();
  if (startByte == 255)
  {
    unsigned int distance;
    Serial.readBytes(buf, 3);
    h_data = buf[0];
    l_data = buf[1];
    sum = buf[2];
    distance = (h_data << 8) + l_data;
    if (((startByte + h_data + l_data) & 0xFF) != sum)
    {
      return -1;
    }
    else
    {
      return distance;
    }
  }
  else
  {
    return -2;
  }
}

/**
 * @brief Read Voltage
 * Read the battery voltage and the current voltage, provided by the solar panel
 */
void readSystemSensors()
{
  for (int i = 0; i < 5; i++)
  {
    lipoRawSensor.add(analogRead(SENSOR_LIPO));
    solarRawSensor.add(analogRead(SENSOR_SOLAR));
  }
  Serial << "Lipo " << lipoRawSensor.getAverage() << " -> " << getBatteryVoltage() << endl;
}

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
    long waterDiff = waterRawSensor.getAverage() - lastWaterValue;
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

  sensorLipo.setProperty("percent").send(String(100 * lipoRawSensor.getAverage() / 4095));
  sensorLipo.setProperty("volt").send(String(getBatteryVoltage()));
  sensorSolar.setProperty("percent").send(String((100 * solarRawSensor.getAverage()) / 4095));
  sensorSolar.setProperty("volt").send(String(getSolarVoltage()));
  startupReason.setProperty("startupReason").send(String(wakeUpReason));

  rtcLipoTempIndex = lipoSensorIndex.get();
  rtcWaterTempIndex = waterSensorIndex.get();

  float lipoTempCurrent = lipoTempSensor.getMedian();
  
  if (! isnan(lipoTempCurrent))
  {
    sensorTemp.setProperty(TEMPERATUR_SENSOR_LIPO).send(String(lipoTempCurrent));
    Serial << "Lipo Temperatur " << lipoTempCurrent << " °C " << endl;
  }

  float t2 = waterTempSensor.getMedian();
  if (! isnan(t2))
  {
    sensorTemp.setProperty(TEMPERATUR_SENSOR_WATER).send(String(t2));
    Serial << "Water Temperatur " << lipoTempCurrent << " °C " << endl;
  }

  //give mqtt time, use via publish callback instead?
  delay(100);

  bool lipoTempWarning = lipoTempCurrent != 85 && abs(lipoTempCurrent - t2) > 10;
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
 * @brief Read ultra sensor JSN-SR04T-2.0
 * Read the distance of the water level.
 */
void readDistance()
{
  for (int i = 0; i < 5; i++)
  {
    long start = millis();
    while (!Serial.available())
    {
      if ((start + 500) < millis())
      {
        Serial << "Abort reading hall sensor, not measurement after 200ms" << endl;
        waterRawSensor.add(0);
        return;
      }
    }
    unsigned int distance = getDistance();
    if (distance > 0)
    {
      waterRawSensor.add(distance);
    }
  }
}

/**
 * @brief read all temperatur sensors
 * 
 * @return int 
 * <code>0</code> device can sleep, no change in the temperatures
 * <code>1</code> something changed and the temperatures shall be published via MQTT
 */
int readTemp() {
  int readAgain = TEMP_SENSOR_MEASURE_SERIES;
  int sensorCount = 0;
  int leaveMode1 = 0;
  while (readAgain > 0)
  {
    sensors.requestTemperatures();
    if (sensorCount > 0)
    {
      if (rtcLipoTempIndex != -1)
      {
        float temp1Raw = sensors.getTempCByIndex(rtcLipoTempIndex);
        Serial << "lipoTempCurrent: " << temp1Raw << endl;
        lipoTempSensor.add(temp1Raw);
      }
      else
      {
        Serial << "missing lipotemp, proceed to mode2: " << endl;
        leaveMode1 = 1;
        readAgain = 0;
        wakeUpReason = WAKEUP_REASON_RTC_MISSING;
      }
    }
    if (sensorCount > 1 && rtcWaterTempIndex != -1)
    {
      float temp2Raw = sensors.getTempCByIndex(rtcWaterTempIndex);
      Serial << "waterTempCurrent: " << temp2Raw << endl;
      waterTempSensor.add(temp2Raw);
    }
    
    readAgain--;
    delay(50);
  }
  return leaveMode1;
}

/**
 * @brief Sensors, that are connected to GPIOs, mandatory for WIFI.
 * These sensors (ADC2) can only be read when no Wifi is used.
 */
bool readSensors()
{
  bool leaveMode1 = false;
  int timeoutTemp = millis() + TEMPERATUR_TIMEOUT;
  int sensorCount = 0;
  
  Serial << "Read Sensors" << endl;

  readSystemSensors();

  /* activate all sensors */
  pinMode(OUTPUT_SENSOR, OUTPUT);
  digitalWrite(OUTPUT_SENSOR, HIGH);

  delay(20);
  sensors.begin();
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
      wakeUpReason = WAKEUP_REASON_MOIST_CHANGE + i;
      leaveMode1 = true;
      Serial.printf("Mode2 start due to moist delta in plant %d with %ld \r\n", i, delta);
    }
  }

  if (abs(getBatteryVoltage() - rtcLastBatteryVoltage) > LIPO_DELTA_VOLT_ADC)
  {
    wakeUpReason = WAKEUP_REASON_BATTERY_CHANGE;
    leaveMode1 = true;
  }
  if (abs(getSolarVoltage() - rtcLastSolarVoltage) > SOLAR_DELTA_VOLT_ADC)
  {
    wakeUpReason = WAKEUP_REASON_SOLAR_CHANGE;
    leaveMode1 = true;
  }

  rtcLastLipoTemp = lipoTempSensor.getAverage();
  rtcLastWaterTemp = waterTempSensor.getAverage();
  rtcLastBatteryVoltage = getBatteryVoltage();
  rtcLastSolarVoltage = getSolarVoltage();
  
  /* Required to read the temperature at least once */
  while (sensorCount == 0 && millis() < timeoutTemp)
  {
    sensors.begin();
    sensorCount = sensors.getDeviceCount();
    Serial << "Waitloop: One wire count: " << sensorCount << endl;
    delay(200);
  }
  Serial << "One wire count: " << sensorCount << endl;
  /* Measure temperature */
  if (sensorCount > 0)
  {
      sensors.requestTemperatures();
  }

  /* Read the distance and give the temperature sensors some time */
  readDistance();
  Serial << "Distance sensor " << waterRawSensor.getAverage() << " cm" << endl;
  leaveMode1 |= readTemp();
 
  for (int i = 0; i < sensorCount; i++)
  {
    Serial << "OnwWire sensor " << i << " has value " << sensors.getTempCByIndex(i) << endl;
  }

  if (abs(lipoTempSensor.getAverage() - rtcLastLipoTemp) > TEMPERATURE_DELTA_TRIGGER_IN_C)
  {
    leaveMode1 = true;
    wakeUpReason = WAKEUP_REASON_TEMP1_CHANGE;
  }
  if (abs(waterTempSensor.getAverage() - rtcLastWaterTemp) > TEMPERATURE_DELTA_TRIGGER_IN_C)
  {
    wakeUpReason = WAKEUP_REASON_TEMP2_CHANGE;
    leaveMode1 = true;
  }

  /* deactivate the sensors */
  digitalWrite(OUTPUT_SENSOR, LOW);
  return leaveMode1;
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
    wakeUpReason = WAKEUP_REASON_RTC_MISSING;
    Serial.println("1 missing rtc value, going to mode2");
    return true;
  }
  for (int i = 0; i < MAX_PLANTS; i++)
  {
    long trigger = getMoistureTrigger(i);
    if (trigger == 0)
    {
      wakeUpReason = WAKEUP_REASON_RTC_MISSING;
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
      wakeUpReason = WAKEUP_REASON_PLANT_DRY + i;
      return true;
    }
  }

  //check how long it was already in mode1 if to long goto mode2

  long cTime = getCurrentTime();
  if (cTime < 100000)
  {
    Serial.println("Starting mode 2 due to missing ntp");
    //missing ntp time boot to mode3
    wakeUpReason = WAKEUP_REASON_TIME_UNSET;
    return true;
  }
  if (gotoMode2AfterThisTimestamp < cTime)
  {
    wakeUpReason = WAKEUP_REASON_MODE2_WAKEUP_TIMER;
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
  Serial.begin(9600);
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

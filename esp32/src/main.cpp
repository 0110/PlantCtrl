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
#include "time.h"
#include "esp_sleep.h"
#include "RunningMedian.h"
#include <arduino-timer.h>

const unsigned long TEMPREADCYCLE = 30000; /**< Check temperature all half minutes */

#define AMOUNT_SENOR_QUERYS   8
#define SENSOR_QUERY_SHIFTS   3
#define SOLAR4SENSORS         6.0f
#define TEMP_INIT_VALUE       -999.0f
#define TEMP_MAX_VALUE        85.0f

/********************* non volatile enable after deepsleep *******************************/

RTC_DATA_ATTR long rtcDeepSleepTime = 0;      /**< Time, when the microcontroller shall be up again */
RTC_DATA_ATTR long rtcLastActive0 = 0;
RTC_DATA_ATTR long rtcMoistureTrigger0 = 0;   /**<Level for the moisture sensor */
RTC_DATA_ATTR long rtcLastActive1 = 0;
RTC_DATA_ATTR long rtcMoistureTrigger1 = 0;   /**<Level for the moisture sensor */
RTC_DATA_ATTR long rtcLastActive2 = 0;
RTC_DATA_ATTR long rtcMoistureTrigger2 = 0;   /**<Level for the moisture sensor */
RTC_DATA_ATTR long rtcLastActive3 = 0;
RTC_DATA_ATTR long rtcMoistureTrigger3 = 0;   /**<Level for the moisture sensor */
RTC_DATA_ATTR long rtcLastActive4 = 0;
RTC_DATA_ATTR long rtcMoistureTrigger4 = 0;   /**<Level for the moisture sensor */
RTC_DATA_ATTR long rtcLastActive5 = 0;
RTC_DATA_ATTR long rtcMoistureTrigger5 = 0;   /**<Level for the moisture sensor */
RTC_DATA_ATTR long rtcLastActive6 = 0;
RTC_DATA_ATTR long rtcMoistureTrigger6 = 0;   /**<Level for the moisture sensor */
RTC_DATA_ATTR int lastPumpRunning = 0;
RTC_DATA_ATTR long lastWaterValue = 0;


bool warmBoot = true;
bool mode3Active = false;   /**< Controller must not sleep */


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


auto wait4sleep = timer_create_default(); // create a timer with default settings

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

int determineNextPump();
void setLastActivationForPump(int pumpId, long time);


//FIXME real impl
long getCurrentTime(){
  return 1337;
}

//wait till homie flushed mqtt ect.
bool prepareSleep(void *) {
  //FIXME wait till pending mqtt is done, then start sleep via event or whatever
  //Homie.prepareToSleep();
  return true; // repeat? true there is something in the queue to be done
}

void mode2MQTT(){
   if (deepSleepTime.get()) {
      Serial << "HOMIE | Setup sleeping for " << deepSleepTime.get() << " ms" << endl;
    }
    /* Publish default values */

  if(lastPumpRunning != -1){
    long waterDiff = mWaterGone-lastWaterValue;
    //TODO attribute used water in ml to plantid
  }
  sensorWater.setProperty("remaining").send(String(waterLevelMax.get() - mWaterGone ));
  Serial << "Water : " << mWaterGone << " cm (" << String(waterLevelMax.get() - mWaterGone ) << "%)" << endl;
  lastWaterValue = mWaterGone;
  
  if (mWaterGone <= waterLevelMin.get()) {
      /* let the ESP sleep qickly, as nothing must be done */
      if ((millis() >= (MIN_TIME_RUNNING * MS_TO_S)) && (deepSleepTime.get() > 0)) {
        Serial << "No Water for pumps" << endl;
        /* in 500 microseconds */
        wait4sleep.in(500, prepareSleep);
        return;
      }
  }

  sensorLipo.setProperty("percent").send( String(100 * lipoSenor / 4095) );
  sensorLipo.setProperty("volt").send( String(ADC_5V_TO_3V3(lipoSenor)) );
  sensorSolar.setProperty("percent").send(String((100 * solarSensor ) / 4095));
  sensorSolar.setProperty("volt").send( String(SOLAR_VOLT(solarSensor)) );

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

  bool lipoTempWarning = abs(temp[1] - temp[2]) > 5;
  if(lipoTempWarning){
    wait4sleep.in(500, prepareSleep);
    return;
  }

  digitalWrite(OUTPUT_PUMP, LOW);
  for(int i=0; i < MAX_PLANTS; i++) {
    digitalWrite(mPlants[i].mPinPump, LOW); 
  }

  lastPumpRunning = determineNextPump();
  if(lastPumpRunning != -1){
    setLastActivationForPump(lastPumpRunning, getCurrentTime());
    digitalWrite(mPlants[lastPumpRunning].mPinPump, HIGH);  
  }
}

void setMoistureTrigger(int plantId, long value){
  if(plantId == 0){
    rtcMoistureTrigger0 = value;
  }
  if(plantId == 1){
    rtcMoistureTrigger1 = value;
  }
    if(plantId == 2){
    rtcMoistureTrigger2 = value;
  }
    if(plantId == 3){
    rtcMoistureTrigger3 = value;
  }
    if(plantId == 4){
    rtcMoistureTrigger4 = value;
  }
    if(plantId == 5){
    rtcMoistureTrigger5 = value;
  }
  if(plantId == 6){
    rtcMoistureTrigger6 = value;
  } 
}

void setLastActivationForPump(int plantId, long value){
  if(plantId == 0){
    rtcLastActive0 = value;
  }
  if(plantId == 1){
    rtcLastActive1 = value;
  }
    if(plantId == 2){
    rtcLastActive2 = value;
  }
    if(plantId == 3){
    rtcLastActive3 = value;
  }
    if(plantId == 4){
    rtcLastActive4 = value;
  }
    if(plantId == 5){
    rtcLastActive5 = value;
  }
  if(plantId == 6){
    rtcLastActive6 = value;
  } 
}

long getLastActivationForPump(int plantId){
  if(plantId == 0){
    return rtcLastActive0;
  }
  if(plantId == 1){
    return rtcLastActive1;
  }
  if(plantId == 2){
    return rtcLastActive2;
  }
  if(plantId == 3){
    return rtcLastActive3;
  }
  if(plantId == 4){
    return rtcLastActive4;
  }
  if(plantId == 5){
    return rtcLastActive5;
  }
  if(plantId == 6){
    return rtcLastActive6;
  }
  return -1;
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




//Homie.getMqttClient().disconnect();

void onHomieEvent(const HomieEvent& event) {
  switch(event.type) {
    case HomieEventType::MQTT_READY:
      plant0.setProperty("switch").send(String("OFF"));            
      plant1.setProperty("switch").send(String("OFF"));            
      plant2.setProperty("switch").send(String("OFF"));
      plant3.setProperty("switch").send(String("OFF"));            
      plant4.setProperty("switch").send(String("OFF"));
      plant5.setProperty("switch").send(String("OFF"));
      plant6.setProperty("switch").send(String("OFF"));

      //wait for rtc sync?
      rtcDeepSleepTime = deepSleepTime.get();
      if(!mode3Active){
        mode2MQTT();
      }
      Homie.getLogger() << "MQTT connected, preparing for deep sleep after 100ms..." << endl;
      break;
    case HomieEventType::READY_TO_SLEEP:
      Homie.getLogger() << "Ready to sleep" << endl;
      esp_deep_sleep_start();
      break;
  }
}

int determineNextPump(){
  float solarValue = solarRawSensor.getMedian();
  bool isLowLight =(ADC_5V_TO_3V3(solarValue) > SOLAR_CHARGE_MIN_VOLTAGE || ADC_5V_TO_3V3(solarValue)  < SOLAR_CHARGE_MAX_VOLTAGE);

  


  //FIXME instead of for, use sorted by last activation index to ensure equal runtime?
  for(int i=0; i < MAX_PLANTS; i++) {
    mPlants[i].calculateSensorValue(AMOUNT_SENOR_QUERYS);
    mPlants[i].setProperty("moist").send(String(100 * mPlants[i].getSensorValue() / 4095 ));
    long lastActivation = getLastActivationForPump(i);
    long sinceLastActivation = getCurrentTime()-lastActivation;
    //this pump is in cooldown skip it and disable low power mode trigger for it
    if(mPlants[i].mSetting->pPumpCooldownInHours->get() > sinceLastActivation / 3600 / 1000){
      setMoistureTrigger(i, DEACTIVATED_PLANT);
      continue;
    }
    //skip as it is not low light
    if(!isLowLight && mPlants[i].mSetting->pPumpOnlyWhenLowLight->get()){
      continue;
    }

    if(mPlants->isPumpRequired()){
      return i;
    }
  }
  return -1;
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
      mode3Active=true;
  } else {
      mode3Active=false;
      esp_deep_sleep_start();
  }
  Serial << "HOMIE  | Controller " << (mode3Active ? " has coffee" : " is tired") << endl;
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
  WiFi.mode(WIFI_STA);

  Homie_setFirmware("PlantControl", FIRMWARE_VERSION);

  // Set default values
  deepSleepTime.setDefaultValue(300000);    /* 5 minutes in milliseconds */
  deepSleepNightTime.setDefaultValue(0);
  wateringDeepSleep.setDefaultValue(60000); /* 1 minute in milliseconds */

  waterLevelMax.setDefaultValue(1000);    /* 100cm in mm */
  waterLevelMin.setDefaultValue(50);      /* 5cm in mm */
  waterLevelWarn.setDefaultValue(500);    /* 50cm in mm */
  waterLevelVol.setDefaultValue(5000);    /* 5l in ml */

  mConfigured = Homie.isConfigured();
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

  Homie.setup();
}


bool mode1(){
  Serial.println("Init mode 1");
  readSensors();
  //queue sensor values for 

  if ((rtcDeepSleepTime == 0) ||
      (rtcMoistureTrigger0 == 0) ||
      (rtcMoistureTrigger1 == 0) ||
      (rtcMoistureTrigger2 == 0) ||
      (rtcMoistureTrigger3 == 0) ||
      (rtcMoistureTrigger4 == 0) ||
      (rtcMoistureTrigger5 == 0) ||
      (rtcMoistureTrigger6 == 0)
    ) 
  {
      Serial.println("Missing RTC information");
      return true;
  }
 
  if ((rtcMoistureTrigger0 != DEACTIVATED_PLANT) && (mPlants[0].getSensorValue() < rtcMoistureTrigger0) ) {
    Serial.println("Moisture of plant 0");
    return true;
  }
  if ((rtcMoistureTrigger1 != DEACTIVATED_PLANT) && (mPlants[1].getSensorValue() < rtcMoistureTrigger1) ) {
    Serial.println("Moisture of plant 1");
    return true;
  }
  if ((rtcMoistureTrigger2 != DEACTIVATED_PLANT) && (mPlants[2].getSensorValue() < rtcMoistureTrigger2) ) {
    Serial.println("Moisture of plant 2");
    return true;
  }
  if ((rtcMoistureTrigger3 != DEACTIVATED_PLANT) && (mPlants[3].getSensorValue() < rtcMoistureTrigger3) ) {
    Serial.println("Moisture of plant 3");
    return true;
  }
  if ((rtcMoistureTrigger4 != DEACTIVATED_PLANT) && (mPlants[4].getSensorValue() < rtcMoistureTrigger4) ) {
    Serial.println("Moisture of plant 4");
    return true;
  }
  if ((rtcMoistureTrigger5 != DEACTIVATED_PLANT) && (mPlants[5].getSensorValue() < rtcMoistureTrigger5) ) {
    Serial.println("Moisture of plant 5");
    return true;
  }
  if ((rtcMoistureTrigger6 != DEACTIVATED_PLANT) && (mPlants[6].getSensorValue() < rtcMoistureTrigger6) ) {
    Serial.println("Moisture of plant 6");
    return true;
  }
  //check how long it was already in mode1 if to long goto mode2

  //TODO evaluate if something is to do
  return false;
}

void mode2(){
  Serial.println("Init mode 2");

  systemInit();

  /* Jump into Mode 3, if not configured */
  if (!mConfigured) {
    Serial.println("upgrade to mode 3");
    mode3Active = true;
  }
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
 
  /* Disable Wifi and bluetooth */
  WiFi.mode(WIFI_OFF);

  if (HomieInternals::MAX_CONFIG_SETTING_SIZE < MAX_CONFIG_SETTING_ITEMS) {
    Serial << "HOMIE | Settings: " << HomieInternals::MAX_CONFIG_SETTING_SIZE << "/" << MAX_CONFIG_SETTING_ITEMS << endl;
    Serial << "      | Update Limits.hpp : MAX_CONFIG_SETTING_SIZE to " << MAX_CONFIG_SETTING_ITEMS << endl;
    Serial << "      | Update Limits.hpp : MAX_JSON_CONFIG_FILE_SIZE to 5000" << endl;
  }

  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_ON);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_XTAL,ESP_PD_OPTION_ON);

  // Big TODO use here the settings in RTC_Memory

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

  if(mode1()){
    mode2();
  } else {
    Serial.println("Nothing to do back to sleep");
    Serial.flush();
    esp_deep_sleep_start();
  }
}

/**
 * @brief Cyclic call
 * Executs the Homie base functionallity or triggers sleeping, if requested.
 */

void loop() {
  Homie.loop();

  if(millis() > 30000 && !mode3Active){
    Serial << (millis()/ 1000) << "s running; going to suicide ..." << endl;
    Serial.flush();
    esp_deep_sleep_start();
  }
}

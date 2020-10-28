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
#include <stdint.h>

const unsigned long TEMPREADCYCLE = 30000; /**< Check temperature all half minutes */

#define AMOUNT_SENOR_QUERYS   8
#define SENSOR_QUERY_SHIFTS   3
#define SOLAR4SENSORS         6.0f
#define TEMP_INIT_VALUE       -999.0f
#define TEMP_MAX_VALUE        85.0f

/********************* non volatile enable after deepsleep *******************************/

RTC_DATA_ATTR long gotoMode2AfterThisTimestamp = 0;
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

const char* ntpServer = "pool.ntp.org";

bool warmBoot = true;
bool mode3Active = false;   /**< Controller must not sleep */
bool mDeepsleep = false;

int plantSensor1 = 0;

int mWaterGone = -1;  /**< Amount of centimeter, where no water is seen */
int readCounter = 0;
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

float getBatteryVoltage(){
  return ADC_5V_TO_3V3(lipoRawSensor.getAverage());
}

float getSolarVoltage(){
  return SOLAR_VOLT(solarRawSensor.getAverage());
}

void readSystemSensors() {
  lipoRawSensor.add(analogRead(SENSOR_LIPO));
  solarRawSensor.add(analogRead(SENSOR_SOLAR)); 
}

int determineNextPump();
void setLastActivationForPump(int pumpId, long time);


long getCurrentTime(){
  struct timeval tv_now;
  gettimeofday(&tv_now, NULL);
  return tv_now.tv_sec;
}

//wait till homie flushed mqtt ect.
bool prepareSleep(void *) {
  //FIXME wait till pending mqtt is done, then start sleep via event or whatever
  //Homie.disableResetTrigger();
  
  bool queueIsEmpty = true;
  if(queueIsEmpty){
    mDeepsleep = true;
  }
  return false; // repeat? true there is something in the queue to be done
}

void espDeepSleepFor(long seconds, bool activatePump = false){
  delay(1500);

  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_XTAL,ESP_PD_OPTION_ON);
  if (activatePump) {
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_ON);
    gpio_deep_sleep_hold_en();
    gpio_hold_en(GPIO_NUM_13); //pump pwr
  } else {
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
    gpio_hold_dis(GPIO_NUM_13); //pump pwr
    gpio_deep_sleep_hold_dis();
    digitalWrite(OUTPUT_PUMP, LOW);
    for (int i=0; i < MAX_PLANTS; i++) {
      mPlants[i].deactivatePump();
    }
  }
  //gpio_hold_en(GPIO_NUM_23); //p0
  //FIXME fix for outher outputs
  

  Serial.print("Going to sleep for ");
  Serial.print(seconds);
  Serial.println(" seconds");
  esp_sleep_enable_timer_wakeup( (seconds * 1000U * 1000U) );
  wait4sleep.in(500, prepareSleep);
}



void mode2MQTT(){
  readSystemSensors();

  configTime(0, 0, ntpServer);

  digitalWrite(OUTPUT_PUMP, LOW);
  for (int i=0; i < MAX_PLANTS; i++) {
    mPlants[i].deactivatePump();
  }

  if (deepSleepTime.get()) {
    Serial << "sleeping for " << deepSleepTime.get() << endl;
  }
  /* Publish default values */

  if(lastPumpRunning != -1){
    long waterDiff = mWaterGone-lastWaterValue;
    //TODO attribute used water in ml to plantid
  }
  for(int i=0; i < MAX_PLANTS; i++) {
    mPlants[i].setProperty("moist").send(String(100 * mPlants[i].getSensorValue() / 4095 ));
  }
  sensorWater.setProperty("remaining").send(String(waterLevelMax.get() - mWaterGone ));
  Serial << "W : " << mWaterGone << " cm (" << String(waterLevelMax.get() - mWaterGone ) << "%)" << endl;
  lastWaterValue = mWaterGone;

  sensorLipo.setProperty("percent").send( String(100 * lipoRawSensor.getAverage() / 4095) );
  sensorLipo.setProperty("volt").send( String(getBatteryVoltage()) );
  sensorSolar.setProperty("percent").send(String((100 * solarRawSensor.getAverage() ) / 4095));
  sensorSolar.setProperty("volt").send( String(getSolarVoltage()) );

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

  bool lipoTempWarning = abs(temp[0] - temp[1]) > 5;
  if(lipoTempWarning){
    Serial.println("Lipo temp incorrect, panic mode deepsleep");
    espDeepSleepFor(PANIK_MODE_DEEPSLEEP);
    return;
  }

  bool hasWater = true;//FIXMEmWaterGone > waterLevelMin.get();
  //FIXME no water warning message
  lastPumpRunning = determineNextPump();
  if(lastPumpRunning != -1 && !hasWater){
    Serial.println("Want to pump but no water");
  }
  if(lastPumpRunning != -1 && hasWater){
    digitalWrite(OUTPUT_PUMP, HIGH);
    setLastActivationForPump(lastPumpRunning, getCurrentTime());
    mPlants[lastPumpRunning].activatePump();
  }
  if(lastPumpRunning == -1 || !hasWater){
    if(getSolarVoltage() < SOLAR_CHARGE_MIN_VOLTAGE){
      gotoMode2AfterThisTimestamp = getCurrentTime()+deepSleepNightTime.get();
      Serial.println("No pumps to activate and low light, deepSleepNight");
      espDeepSleepFor(deepSleepNightTime.get());
    }else {
      gotoMode2AfterThisTimestamp = getCurrentTime()+deepSleepTime.get();
      Serial.println("No pumps to activate, deepSleep");
      espDeepSleepFor(deepSleepTime.get());
    }
    
  }else {
    gotoMode2AfterThisTimestamp = 0;
    Serial.println("Running pump, watering deepsleep");
    espDeepSleepFor(wateringDeepSleep.get(), true);
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
  Serial << "Read Sensors" << endl;

  readSystemSensors();

  /* activate all sensors */
  pinMode(OUTPUT_SENSOR, OUTPUT);
  digitalWrite(OUTPUT_SENSOR, HIGH);

  delay(100);
  /* wait before reading something */
  for (int readCnt=0;readCnt < AMOUNT_SENOR_QUERYS; readCnt++) {
    for(int i=0; i < MAX_PLANTS; i++) {
      mPlants[i].addSenseValue();
    }
  }

  Serial << "DS18B20" << endl;
  /* Read the temperature sensors once, as first time 85 degree is returned */
  Serial << "DS18B20" << String(dallas.readDevices()) << endl;
  delay(200);


  /* Required to read the temperature once */
  float temp[2] = {0, 0};
  float* pFloat = temp;
  // first read returns crap, ignore result and read twice
  if (dallas.readAllTemperatures(pFloat, 2) > 0) {
      Serial << "t1: " << String(temp[0]) << endl;
      Serial << "t2: " << String(temp[1]) << endl;
  }
  delay(200);
  if (dallas.readAllTemperatures(pFloat, 2) > 0) {
      Serial << "t1: " << String(temp[0]) << endl;
      Serial << "t2: " << String(temp[1]) << endl;
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
    case HomieEventType::SENDING_STATISTICS:
      Homie.getLogger() << "My statistics" << endl;
      break;
    case HomieEventType::MQTT_READY:
      //wait for rtc sync?
      rtcDeepSleepTime = deepSleepTime.get();
      Serial << "MQTT ready " << rtcDeepSleepTime << " ms ds" << endl;
      for(int i=0; i < MAX_PLANTS; i++) {
        mPlants[i].postMQTTconnection();
      }

      mode2MQTT();
      break;
    case HomieEventType::READY_TO_SLEEP:
      Homie.getLogger() << "rtsleep" << endl;
      esp_deep_sleep_start();
      break;
    case HomieEventType::OTA_STARTED:
      digitalWrite(OUTPUT_SENSOR, HIGH);
      digitalWrite(OUTPUT_PUMP, LOW);
      gpio_hold_dis(GPIO_NUM_13); //pump pwr
      gpio_deep_sleep_hold_dis();
      for (int i=0; i < MAX_PLANTS; i++) {
        mPlants[i].deactivatePump();
      }
      mode3Active=true;
      break;
    case HomieEventType::OTA_SUCCESSFUL:
      digitalWrite(OUTPUT_SENSOR, LOW);
      digitalWrite(OUTPUT_PUMP, LOW);
      ESP.restart();
      break;
    default:
      break;
  }
}

int determineNextPump(){
  float solarValue = getSolarVoltage();
  bool isLowLight =(solarValue > SOLAR_CHARGE_MIN_VOLTAGE || solarValue < SOLAR_CHARGE_MAX_VOLTAGE);

  //FIXME instead of for, use sorted by last activation index to ensure equal runtime?
  for(int i=0; i < MAX_PLANTS; i++) {
    long lastActivation = getLastActivationForPump(i);
    long sinceLastActivation = getCurrentTime()-lastActivation;
    //this pump is in cooldown skip it and disable low power mode trigger for it
    if(mPlants[i].isInCooldown(sinceLastActivation) ){
      Serial.printf("%d Skipping due to cooldown\r\n", i);
      setMoistureTrigger(i, DEACTIVATED_PLANT);
      continue;
    }
    //skip as it is not low light
    if(!isLowLight && mPlants[i].isAllowedOnlyAtLowLight()){
      Serial.println("Skipping due to light");
      continue;
    }

    if(mPlants->isPumpRequired()){
      Serial.printf("%d Requested pumping\r\n", i);
      return i;
    }
    Serial.printf("%d No pump required\r\n", i);
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
bool aliveHandler(const HomieRange& range, const String& value) {
  if (range.isRange) return false;  // only one controller is present
  Serial << value  << endl;
  if (value.equals("ON") || value.equals("On") || value.equals("1")) {
      mode3Active=true;
  } else {
      mode3Active=false;
  }
 
  return true;
}

void homieLoop(){

}

void systemInit(){
  WiFi.mode(WIFI_STA);

  Homie_setFirmware("PlantControl", FIRMWARE_VERSION);

  // Set default values
  
  //in seconds
  deepSleepTime.setDefaultValue(10);
  deepSleepNightTime.setDefaultValue(30);
  wateringDeepSleep.setDefaultValue(5);

  /* waterLevelMax 1000    */             /* 100cm in mm */
  waterLevelMin.setDefaultValue(50);      /* 5cm in mm */
  waterLevelWarn.setDefaultValue(500);    /* 50cm in mm */
  waterLevelVol.setDefaultValue(5000);    /* 5l in ml */

  Homie.setLoopFunction(homieLoop);
  Homie.onEvent(onHomieEvent);
  Homie.setup();

  mConfigured = Homie.isConfigured();
  if (mConfigured) {
    for(int i=0; i < MAX_PLANTS; i++) {
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


bool mode1(){
  Serial.println("m1");
  Serial << getCurrentTime() << " curtime" << endl;

  /* Disable all sleeping stuff before reading sensors */
  gpio_deep_sleep_hold_dis();

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
      Serial.println("RTCm2");
      return true;
  }
 
  if ((rtcMoistureTrigger0 != DEACTIVATED_PLANT) && (mPlants[0].getSensorValue() < rtcMoistureTrigger0) ) {
    Serial.println("mt0");
    return true;
  }
  if ((rtcMoistureTrigger1 != DEACTIVATED_PLANT) && (mPlants[1].getSensorValue() < rtcMoistureTrigger1) ) {
    Serial.println("mt1");
    return true;
  }
  if ((rtcMoistureTrigger2 != DEACTIVATED_PLANT) && (mPlants[2].getSensorValue() < rtcMoistureTrigger2) ) {
    Serial.println("mt2");
    return true;
  }
  if ((rtcMoistureTrigger3 != DEACTIVATED_PLANT) && (mPlants[3].getSensorValue() < rtcMoistureTrigger3) ) {
    Serial.println("mt3");
    return true;
  }
  if ((rtcMoistureTrigger4 != DEACTIVATED_PLANT) && (mPlants[4].getSensorValue() < rtcMoistureTrigger4) ) {
    Serial.println("mt4");
    return true;
  }
  if ((rtcMoistureTrigger5 != DEACTIVATED_PLANT) && (mPlants[5].getSensorValue() < rtcMoistureTrigger5) ) {
    Serial.println("mt5");
    return true;
  }
  if ((rtcMoistureTrigger6 != DEACTIVATED_PLANT) && (mPlants[6].getSensorValue() < rtcMoistureTrigger6) ) {
    Serial.println("mt6");
    return true;
  }
  //check how long it was already in mode1 if to long goto mode2

  long cTime = getCurrentTime();
  if(cTime < 100000){
    Serial.println("Starting mode 2 due to missing ntp");
    //missing ntp time boot to mode3
    return true;
  }
  if(gotoMode2AfterThisTimestamp < cTime){
    Serial.println("Starting mode 2 after specified mode1 time");
    return true;
  }
  return false;
}

void mode2(){
  Serial.println("m2");
  systemInit();

  /* Jump into Mode 3, if not configured */
  if (!mConfigured) {
    Serial.println("m3");
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
  /* read button */
  pinMode(BUTTON, INPUT);

  /* Power pins */
  pinMode(OUTPUT_PUMP, OUTPUT);
 
  /* Disable Wifi and bluetooth */
  WiFi.mode(WIFI_OFF);

  if (HomieInternals::MAX_CONFIG_SETTING_SIZE < MAX_CONFIG_SETTING_ITEMS) {
    //increase the config settings to 50 and the json to 3000
    Serial << "Limits.hpp" << endl;
  }

  // Big TODO use here the settings in RTC_Memory

  //Panik mode, the Lipo is empty, sleep a long long time:
  if ((getBatteryVoltage() < MINIMUM_LIPO_VOLT) && 
    (getBatteryVoltage() > NO_LIPO_VOLT)) {
    Serial << PANIK_MODE_DEEPSLEEP << " s lipo " << getBatteryVoltage() << "V" << endl;
    esp_sleep_enable_timer_wakeup(PANIK_MODE_DEEPSLEEP_US);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_XTAL,ESP_PD_OPTION_ON);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
    esp_deep_sleep_start();
  }

  if(mode1()){
    mode2();
  } else {
    Serial.println("nop");
    Serial.flush();
    esp_deep_sleep_start();
  }
}

/**
 * @brief Cyclic call
 * Executs the Homie base functionallity or triggers sleeping, if requested.
 */

void loop() {
  if (!mDeepsleep) {
    Homie.loop();
  } else {
    esp_deep_sleep_start();
  }

  if(millis() > 30000 && !mode3Active){
    Serial << (millis()/ 1000) << "s alive" << endl;
    Serial.flush();
    esp_deep_sleep_start();
  }
}

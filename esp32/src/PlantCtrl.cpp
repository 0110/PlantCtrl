/**
 * @file PlantCtrl.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2020-05-27
 *
 * @copyright Copyright (c) 2020
 *

 */
#include "PlantCtrl.h"
#include "ControllerConfiguration.h"
#include "TimeUtils.h"
#include "driver/pcnt.h"
#include "MQTTUtils.h"

Plant::Plant(int pinSensor, int pinPump, int plantId, HomieNode *plant, PlantSettings_t *setting, SENSOR_MODE mode)
{
    this->mPinSensor = pinSensor;
    this->mPinPump = pinPump;
    this->mPlant = plant;
    this->mSetting = setting;
    this->mPlantId = plantId;
    this->mSensorMode = mode;
}

void Plant::init(void)
{
    /* Initialize Home Settings validator */
    this->mSetting->pSensorDry->setDefaultValue(DEACTIVATED_PLANT);
    this->mSetting->pSensorDry->setValidator([](long candidate)
                                             { return (((candidate >= 0.0) && (candidate <= 100.0)) || equalish(candidate, DEACTIVATED_PLANT) || equalish(candidate, HYDROPONIC_MODE) || equalish(candidate, TIMER_ONLY)); });

    this->mSetting->pPumpAllowedHourRangeStart->setDefaultValue(5); // start at 5:00 UTC or 7:00 ECST
    this->mSetting->pPumpAllowedHourRangeStart->setValidator([](long candidate)
                                                             { return ((candidate >= 0) && (candidate <= 23)); });
    this->mSetting->pPumpAllowedHourRangeEnd->setDefaultValue(18); // stop pumps at 18 UTC or 20:00 ECST
    this->mSetting->pPumpAllowedHourRangeEnd->setValidator([](long candidate)
                                                           { return ((candidate >= 0) && (candidate <= 23)); });
    this->mSetting->pPumpOnlyWhenLowLight->setDefaultValue(false);
    this->mSetting->pPumpCooldownInSeconds->setDefaultValue(60 * 60); // 1 hour
    this->mSetting->pPumpCooldownInSeconds->setValidator([](long candidate)
                                                         { return (candidate >= 0); });

    this->mSetting->pPumpDuration->setDefaultValue(30);
    this->mSetting->pPumpDuration->setValidator([](long candidate)
                                                { return ((candidate >= 0) && (candidate <= 1000)); });
    this->mSetting->pPumpMl->setDefaultValue(1000);
    this->mSetting->pPumpMl->setValidator([](long candidate)
                                          { return ((candidate >= 0) && (candidate <= 5000)); });
    this->mSetting->pPumpPowerLevel->setDefaultValue(100);
    this->mSetting->pPumpPowerLevel->setValidator([](long candidate)
                                                  { return ((candidate >= 0) && (candidate <= 100)); });

    /* Initialize Hardware */
    ledcSetup(this->mPlantId, PWM_FREQ, PWM_BITS);
    ledcAttachPin(mPinPump, this->mPlantId);
    ledcWrite(this->mPlantId, 0);
    pinMode(this->mPinSensor, INPUT);
}

void Plant::initSensors(void)
{
    switch (getSensorMode())
    {
    case FREQUENCY_MOD_RESISTANCE_PROBE:
    {

        pcnt_unit_t unit = (pcnt_unit_t)(PCNT_UNIT_0 + this->mPlantId);
        pcnt_config_t pcnt_config = {}; // Instancia PCNT config

        pcnt_config.pulse_gpio_num = this->mPinSensor; // Configura GPIO para entrada dos pulsos
        pcnt_config.ctrl_gpio_num = PCNT_PIN_NOT_USED; // Configura GPIO para controle da contagem
        pcnt_config.unit = unit;                       // Unidade de contagem PCNT - 0
        pcnt_config.channel = PCNT_CHANNEL_0;          // Canal de contagem PCNT - 0
        pcnt_config.counter_h_lim = INT16_MAX;         // Limite maximo de contagem - 20000
        pcnt_config.pos_mode = PCNT_COUNT_INC;         // Incrementa contagem na subida do pulso
        pcnt_config.neg_mode = PCNT_COUNT_DIS;         // Incrementa contagem na descida do pulso
        pcnt_config.lctrl_mode = PCNT_MODE_KEEP;       // PCNT - modo lctrl desabilitado
        pcnt_config.hctrl_mode = PCNT_MODE_KEEP;       // PCNT - modo hctrl - se HIGH conta incrementando
        pcnt_unit_config(&pcnt_config);                // Configura o contador PCNT

        pcnt_counter_pause(unit); // Pausa o contador PCNT
        pcnt_counter_clear(unit); // Zera o contador PCNT
        break;
    }
    case ANALOG_RESISTANCE_PROBE:
    {
        adcAttachPin(this->mPinSensor);
        break;
    }
    case NONE:
    {
        // do nothing
        break;
    }
    }
}

void Plant::blockingMoistureMeasurement(void)
{
    switch (getSensorMode())
    {
    case ANALOG_RESISTANCE_PROBE:
    {
        for (int i = 0; i < ANALOG_REREADS; i++)
        {
            this->mMoisture_raw.add(analogReadMilliVolts(this->mPinSensor));
            delay(5);
        }
        break;
    }
    case FREQUENCY_MOD_RESISTANCE_PROBE:
    case NONE:
    {
        // nothing to do here
        break;
    }
    }
}

void Plant::startMoistureMeasurement(void)
{
    switch (getSensorMode())
    {
    case FREQUENCY_MOD_RESISTANCE_PROBE:
    {
        pcnt_unit_t unit = (pcnt_unit_t)(PCNT_UNIT_0 + this->mPlantId);
        pcnt_counter_resume(unit);
        break;
    }
    case ANALOG_RESISTANCE_PROBE:
    case NONE:
    {
        // do nothing here
    }
    }
}

void Plant::stopMoistureMeasurement(void)
{
    switch (getSensorMode())
    {
    case FREQUENCY_MOD_RESISTANCE_PROBE:
    {
        int16_t pulses;
        pcnt_unit_t unit = (pcnt_unit_t)(PCNT_UNIT_0 + this->mPlantId);
        pcnt_counter_pause(unit);
        esp_err_t result = pcnt_get_counter_value(unit, &pulses);
        pcnt_counter_clear(unit);
        if (result != ESP_OK)
        {
            log(LOG_LEVEL_ERROR, LOG_HARDWARECOUNTER_ERROR_MESSAGE, LOG_HARDWARECOUNTER_ERROR_CODE);
            this->mMoisture_raw.clear();
            this->mMoisture_raw.add(-1);
        }
        else
        {
            this->mMoisture_raw.add(pulses * (1000 / MOISTURE_MEASUREMENT_DURATION));
        }
        break;
    }
    case ANALOG_RESISTANCE_PROBE:
    case NONE:
    {
        break;
    }
    }
}

void Plant::postMQTTconnection(void)
{
    const String OFF = String("OFF");
    this->mConnected = true;
    this->mPlant->setProperty("switch").send(OFF);

    float pct = getCurrentMoisturePCT();
    float raw = getCurrentMoistureRaw();
    if (equalish(raw, MISSING_SENSOR))
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

    this->mPlant->setProperty("moist").send(String(pct));
    this->mPlant->setProperty("sensormode").send(getSensorModeString());
    this->mPlant->setProperty("moistraw").send(String(raw));
    this->mPlant->setProperty("moisttrigger").send(String(getTargetMoisturePCT()));
}

void Plant::deactivatePump(void)
{
    int plantId = this->mPlantId;
    Serial << "deactivating pump " << plantId << endl;
    ledcWrite(this->mPlantId, 0);
    if (this->mConnected)
    {
        const String OFF = String("OFF");
        this->mPlant->setProperty("switch").send(OFF);
    }
}

void Plant::publishState(int stateNumber, String stateString)
{
    String buffer;
    StaticJsonDocument<200> doc;
    if (this->mConnected)
    {
        doc["number"] = stateNumber;
        doc["message"] = stateString;
        serializeJson(doc, buffer);
        this->mPlant->setProperty("state").send(buffer.c_str());
    }
}

void Plant::activatePump(void)
{
    int plantId = this->mPlantId;

    Serial << "activating pump " << plantId << endl;
    long desiredPowerLevelPercent = this->mSetting->pPumpPowerLevel->get();
    ledcWrite(this->mPlantId, desiredPowerLevelPercent * PWM_BITS);
    if (this->mConnected)
    {
        const String ON = String("ON");
        this->mPlant->setProperty("switch").send(ON);
        this->mPlant->setProperty("lastPump").send(String(getCurrentTime()));
    }
}

bool Plant::switchHandler(const HomieRange &range, const String &value)
{
    if (range.isRange)
    {
        return false; // only one switch is present
    }

    if ((value.equals("ON")) || (value.equals("On")) || (value.equals("on")) || (value.equals("true")))
    {
        this->activatePump();
        return true;
    }
    else if ((value.equals("OFF")) || (value.equals("Off")) || (value.equals("off")) || (value.equals("false")))
    {
        this->deactivatePump();
        return true;
    }
    else
    {
        return false;
    }
}

void Plant::setSwitchHandler(HomieInternals::PropertyInputHandler f)
{
    this->mPump.settable(f);
}

void Plant::advertise(void)
{
    // Advertise topics
    mPump = this->mPlant->advertise("switch").setName("Pump").setDatatype("Boolean");
    this->mPlant->advertise("lastPump").setName("lastPump").setDatatype("Integer").setUnit("unixtime").setRetained(true);
    this->mPlant->advertise("moist").setName("Percent").setDatatype("Float").setUnit("%").setRetained(true);
    this->mPlant->advertise("moistraw").setName("frequency").setDatatype("Float").setUnit("hz").setRetained(true);
    this->mPlant->advertise("state").setName("state").setDatatype("String").setRetained(true);
    
}

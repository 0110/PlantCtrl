#include "MQTTUtils.h"
#include "FileUtils.h"
#include "LogDefines.h"


bool volatile mAliveWasRead = false;

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


void mqttWrite(HomieNode* target,String key, String value){
    if(mAliveWasRead){
        target->setProperty(key).send(value);
    }
}

void mqttWrite(HomieNode* target,const char* key, String value){
    if(aliveWasRead()){
        target->setProperty(key).send(value);
    }
}


void onMQTTMessage(char *incoming, char *payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total)
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

bool aliveWasRead(){
    return mAliveWasRead;
}

void startMQTTRoundtripTest(){
     {
      getTopic(TEST_TOPIC, testopic)
          Homie.getMqttClient()
              .subscribe(testopic, 2);
      Homie.getMqttClient().publish(testopic, 2, false, "ping");
      Homie.getMqttClient().onMessage(onMQTTMessage);

      getTopic(BACKUP_TOPIC, backupTopic)
          Homie.getMqttClient()
              .subscribe(backupTopic, 2);
    }
}
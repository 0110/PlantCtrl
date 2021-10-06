#ifndef MQTTUtils_h
#define MQTTUtils_h

#include <Homie.h>

#define LOG_TOPIC "log\0"
#define TEST_TOPIC "roundtrip\0"
#define BACKUP_TOPIC "$implementation/config/backup/set\0"

#define CONFIG_FILE "/homie/config.json"
#define CONFIG_FILE_BACKUP "/homie/config.json.bak"

#define getTopic(test, topic)                                                                                                                 \
  char *topic = new char[strlen(Homie.getConfiguration().mqtt.baseTopic) + strlen(Homie.getConfiguration().deviceId) + 1 + strlen(test) + 1]; \
  strcpy(topic, Homie.getConfiguration().mqtt.baseTopic);                                                                                     \
  strcat(topic, Homie.getConfiguration().deviceId);                                                                                           \
  strcat(topic, "/");                                                                                                                         \
  strcat(topic, test);

bool aliveWasRead();
bool mqttReady();
void startMQTTRoundtripTest();

void log(int level, String message, int code);
void mqttWrite(HomieNode* target,const char* key, String value);
void mqttWrite(HomieNode* target,String key, String value);

#endif
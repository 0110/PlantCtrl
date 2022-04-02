#include <Arduino.h>
#include "driver/pcnt.h" 

#define PWM_FREQ 50000
#define PWM_BITS 8

#define OUTPUT_SENSOR       14
#define SENSOR_PLANT        17

int16_t         pulses        = 0; 
int16_t         pulses2        = 0; 

int plantId = 0;

void setup() {

  Serial.begin(115200);
  pinMode(OUTPUT_SENSOR, OUTPUT);

  ledcSetup(plantId, PWM_FREQ, PWM_BITS);
  ledcAttachPin(SENSOR_PLANT, plantId);
  ledcWrite(plantId, plantId);
  pinMode(SENSOR_PLANT, INPUT);


  pcnt_config_t pcnt_config = { };                                        // Instancia PCNT config
  pcnt_unit_t unit = (pcnt_unit_t)(PCNT_UNIT_0 + plantId);
  pcnt_config.pulse_gpio_num = SENSOR_PLANT;                         // Configura GPIO para entrada dos pulsos
  pcnt_config.ctrl_gpio_num = PCNT_PIN_NOT_USED;                         // Configura GPIO para controle da contagem
  pcnt_config.unit = unit;                                          // Unidade de contagem PCNT - 0
  pcnt_config.channel = PCNT_CHANNEL_0;                               // Canal de contagem PCNT - 0
  pcnt_config.counter_h_lim = INT16_MAX;                             // Limite maximo de contagem - 20000
  pcnt_config.pos_mode = PCNT_COUNT_INC;                                  // Incrementa contagem na subida do pulso
  pcnt_config.neg_mode = PCNT_COUNT_DIS;                                  // Incrementa contagem na descida do pulso
  pcnt_config.lctrl_mode = PCNT_MODE_KEEP;                             // PCNT - modo lctrl desabilitado
  pcnt_config.hctrl_mode = PCNT_MODE_KEEP;                                // PCNT - modo hctrl - se HIGH conta incrementando
  pcnt_unit_config(&pcnt_config);                                         // Configura o contador PCNT


  pcnt_counter_pause(PCNT_UNIT_0);                                    // Pausa o contador PCNT
  pcnt_counter_clear(PCNT_UNIT_0);                                    // Zera o contador PCNT


  digitalWrite(OUTPUT_SENSOR, HIGH);
  Serial.println("Nodemcu ESP32 Start done");
}

void loop() { 
    pcnt_counter_resume(PCNT_UNIT_0);
    
    delay(500);
    
    pcnt_counter_pause(PCNT_UNIT_0); 
    pcnt_get_counter_value(PCNT_UNIT_0, &pulses);
    pcnt_counter_clear(PCNT_UNIT_0);
    
    Serial.println(pulses*2);
}
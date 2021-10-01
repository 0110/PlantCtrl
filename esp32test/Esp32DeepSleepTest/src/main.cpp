#include <Arduino.h>
#include "driver/pcnt.h" 

#define OUTPUT_SENSOR       14  /**< GPIO 16 - Enable Sensors  */
#define SENSOR_PLANT5       39  /**< SENSOR vn */
#define SENSOR_PLANT6       36  /**< SENSOR VP */

int16_t         pulses        = 0; 
int16_t         pulses2        = 0; 

void setup() {

  Serial.begin(115200);
  pinMode(OUTPUT_SENSOR, OUTPUT);
  pinMode(SENSOR_PLANT5, INPUT);


  pcnt_config_t pcnt_config = { };                                        // Instancia PCNT config

  pcnt_config.pulse_gpio_num = SENSOR_PLANT5;                         // Configura GPIO para entrada dos pulsos
  pcnt_config.ctrl_gpio_num = PCNT_PIN_NOT_USED;                         // Configura GPIO para controle da contagem
  pcnt_config.unit = PCNT_UNIT_0;                                     // Unidade de contagem PCNT - 0
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
  Serial.println("Start done");
}

void loop() { 
    pulses2 = pulseIn(SENSOR_PLANT5,HIGH);
    pcnt_counter_resume(PCNT_UNIT_0);
    
    delay(500);
    
    pcnt_counter_pause(PCNT_UNIT_0); 
    pcnt_get_counter_value(PCNT_UNIT_0, &pulses);
    pcnt_counter_clear(PCNT_UNIT_0);
    
    Serial.println(pulses2*2);
    Serial.println(pulses*2);
}
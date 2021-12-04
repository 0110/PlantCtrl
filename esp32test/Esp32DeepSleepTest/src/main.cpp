#include <Arduino.h>
#include "driver/pcnt.h" 
#include "driver/rtc_io.h"
#include "esp32/ulp.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/rtc.h"

#define ULP_DATA_OFFSET     200

#define ULP_START_OFFSET 0
void ulp_start(void) {
  // Slow memory initialization
  //memset(RTC_SLOW_MEM, ULP_START_OFFSET, (8192-ULP_START_OFFSET));
  // GPIO32 initialization (set to output and initial value is 0)
  rtc_gpio_init(GPIO_NUM_12);
  rtc_gpio_set_direction(GPIO_NUM_12, RTC_GPIO_MODE_OUTPUT_ONLY);
  rtc_gpio_set_level(GPIO_NUM_12, 0);
  // Define ULP program
  const ulp_insn_t  ulp_prog[] = {
    M_LABEL(1),

    I_MOVI(R2, 0),
    I_MOVI(R3, 0),
    I_LD(R2, R2, ULP_DATA_OFFSET),
    I_LD(R3, R3, ULP_DATA_OFFSET+1),


    I_WR_REG(RTC_GPIO_OUT_REG, 29, 29, 1), // on
    
    //wait for 100*r2
    I_MOVR(R0,R2),
    M_LABEL(2),
    I_DELAY(1),
    I_SUBI(R0,R0,1),
    M_BGE(2,1),

    I_WR_REG(RTC_GPIO_OUT_REG, 29, 29, 0), // off

    //wait for 100*R3    
    I_MOVR(R0,R3),
    M_LABEL(3),
    I_DELAY(1),
    I_SUBI(R0,R0,1),
    M_BGE(3,1),

    M_BX(1),
  };
  // Run ULP program
  size_t size = sizeof(ulp_prog) / sizeof(ulp_insn_t);
  ulp_process_macros_and_load(ULP_START_OFFSET, ulp_prog, &size);
  assert(size < ULP_DATA_OFFSET && "ULP_DATA_OFFSET needs to be greater or equal to the program size");
  ulp_run(ULP_START_OFFSET);
}

static inline void ulp_data_write(size_t offset, uint16_t value)
{
    RTC_SLOW_MEM[ULP_DATA_OFFSET + offset] = value;
}

static inline uint16_t ulp_data_read(size_t offset)
{
  return RTC_SLOW_MEM[ULP_DATA_OFFSET + offset] & 0xffff;
}

void setup() {

  Serial.begin(115200);
  ulp_data_write(0,1000);
  ulp_data_write(1,1000);
  ulp_start();
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
  //esp_deep_sleep_start();
}


void loop() { 
  delay(1000);
  Serial.println(ulp_data_read(0));
  ulp_data_write(0, 50);
  ulp_data_write(1, 150);
  //test = 10;
  //test2 = 255-test;
  delay(1000);
  //test = 50;
  //test2 = 255 - test;
  ulp_data_write(0, 150);
  ulp_data_write(1, 50);
  //Serial.print();
  Serial.println("loop");
}
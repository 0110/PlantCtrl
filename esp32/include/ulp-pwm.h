#ifndef ULP_PWM_h
#define ILP_PWM_h

#include <Arduino.h>
#include "driver/rtc_io.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/rtc.h"
#include "esp32/ulp.h"
#include "ControllerConfiguration.h"

#define LBL_START 1
#define LBL_DELAY_ON 2
#define LBL_DELAY_OFF 3
#define LBL_SKIP_ON 4
#define LBL_SKIP_OFF 5
#define REGISTER_DELAY_LOOP_COUNTER R0
#define REGISTER_TICKS_ON R1
#define REGISTER_TICKS_OFF R2
#define TOTAL_TICKS_DELAY 255
#define PIN TIMED_LIGHT_PIN

//support 20 vars
const size_t ulp_var_offset = CONFIG_ULP_COPROC_RESERVE_MEM - 20;
//use the first for dimming
const size_t ulp_dimm_offset = ulp_var_offset + 1;
const size_t ulp_alive_offset = ulp_var_offset + 2;

//see https://github.com/perseus086/ESP32-notes
const uint32_t rtc_bit[40] = {
    25, //gpio0
    0,  //gpio1
    26, //gpio2
    0,  //gpio3
    24, //gpio4
    0,  //gpio5
    0,  //gpio6
    0,  //gpio7
    0,  //gpio8
    0,  //gpio9
    0,  //gpio10
    0,  //gpio11
    29, //gpio12
    28, //gpio13
    30, //gpio14
    27, //gpio15
    0,  //gpio16
    31, //gpio17
    0,  //gpio18
    0,  //gpio19
    0,  //gpio20
    0,  //gpio21
    0,  //gpio22
    0,  //gpio23
    0,  //gpio24
    20, //gpio25
    21, //gpio26
    0,  //gpio27
    0,  //gpio28
    0,  //gpio29
    0,  //gpio30
    0,  //gpio31
    23, //gpio32
    22, //gpio33
    18, //gpio34
    19, //gpio35
    14, //gpio36
    15, //gpio37
    16, //gpio38
    17  //gpio39
};

static inline void ulp_internal_data_write(size_t offset, uint16_t value)
{
    if (offset >= CONFIG_ULP_COPROC_RESERVE_MEM || offset <= ulp_var_offset)
    {
        Serial.print("Invalid ULP offset detected, refusing write!");
        Serial.print(offset);
        Serial.print("-");
        Serial.print(ulp_var_offset);
        Serial.print("-");
        Serial.println(CONFIG_ULP_COPROC_RESERVE_MEM);
        return;
    }
    else
    {
        RTC_SLOW_MEM[offset] = value;
    }
}

static inline uint16_t ulp_internal_data_read(size_t offset)
{
    if (offset >= CONFIG_ULP_COPROC_RESERVE_MEM || offset <= ulp_var_offset)
    {
        Serial.print("Invalid ULP offset detected");
        Serial.print(offset);
        Serial.print("-");
        Serial.print(ulp_var_offset);
        Serial.print("-");
        Serial.println(CONFIG_ULP_COPROC_RESERVE_MEM);
    }
    return RTC_SLOW_MEM[offset] & 0xffff;
}

static inline uint32_t rtc_io_number_get(gpio_num_t gpio_num)
{
    assert(rtc_gpio_is_valid_gpio(gpio_num) && "Invalid GPIO for RTC");
    uint32_t bit = rtc_bit[gpio_num];
    Serial.print("Resolved GPIO ");
    Serial.print(gpio_num);
    Serial.print(" to rtc bit ");
    Serial.println(bit);
    return bit;
}

void ulp_internal_start(void)
{
    rtc_gpio_init(PIN);
    rtc_gpio_set_direction(PIN, RTC_GPIO_MODE_OUTPUT_ONLY);
    rtc_gpio_set_level(PIN, 0);
    const uint32_t rtc_gpio = rtc_io_number_get(PIN);

    // Define ULP program
    const ulp_insn_t ulp_prog[] = {
        M_LABEL(LBL_START),

        I_MOVI(REGISTER_DELAY_LOOP_COUNTER, 1),
        I_MOVI(REGISTER_TICKS_ON, 0),
        I_ST(REGISTER_DELAY_LOOP_COUNTER, REGISTER_TICKS_ON, ulp_alive_offset), //store 1 with 0 offset into alive

        I_LD(REGISTER_TICKS_ON, REGISTER_TICKS_ON, ulp_dimm_offset), //REGISTER_TICKS_ON = RTC_DATA[0+ulp_dimm_offset]
        //in total there is always 255 delay loop iterations, but in different duty cycle
        I_MOVI(REGISTER_TICKS_OFF, TOTAL_TICKS_DELAY),
        I_SUBR(REGISTER_TICKS_OFF, REGISTER_TICKS_OFF, REGISTER_TICKS_ON),

        //on phase
        I_MOVR(REGISTER_DELAY_LOOP_COUNTER, REGISTER_TICKS_ON),
        M_BL(LBL_SKIP_ON, 1),                                 //if never on, skip on phase
        I_WR_REG(RTC_GPIO_OUT_REG, rtc_gpio, rtc_gpio, HIGH), // on
        M_LABEL(LBL_DELAY_ON),
        I_DELAY(1),                                                          //wait 1 clock
        I_SUBI(REGISTER_DELAY_LOOP_COUNTER, REGISTER_DELAY_LOOP_COUNTER, 1), // REGISTER_DELAY_LOOP_COUNTER--
        M_BGE(LBL_DELAY_ON, 1),                                              //if time left, goto start of on loop
        M_LABEL(LBL_SKIP_ON),

        //off phase
        I_MOVR(REGISTER_DELAY_LOOP_COUNTER, REGISTER_TICKS_OFF),

        M_BL(LBL_SKIP_OFF, 1),                               //if never off, skip on phase
        I_WR_REG(RTC_GPIO_OUT_REG, rtc_gpio, rtc_gpio, LOW), // on
        M_LABEL(3),
        I_DELAY(1),                                                          //wait 1 clock
        I_SUBI(REGISTER_DELAY_LOOP_COUNTER, REGISTER_DELAY_LOOP_COUNTER, 1), // REGISTER_DELAY_LOOP_COUNTER--
        M_BGE(3, 1),                                                         //if time left, goto start of on loop
        M_LABEL(LBL_SKIP_OFF),

        M_BX(LBL_START),
    };
    // Run ULP program
    size_t size = sizeof(ulp_prog) / sizeof(ulp_insn_t);
    assert(size < ulp_var_offset && "ULP_DATA_OFFSET needs to be greater or equal to the program size");
    esp_err_t error = ulp_process_macros_and_load(0, ulp_prog, &size);
    Serial.print("ULP bootstrap status ");
    Serial.println(error);

    //allow glitchless start
    ulp_internal_data_write(ulp_alive_offset, 0);

    error = ulp_run(0);
    Serial.print("ULP start status ");
    Serial.println(error);
}

static inline void ulp_pwm_set_level(uint8_t level)
{
    ulp_internal_data_write(ulp_dimm_offset, level);
}

static inline void ulp_pwm_init()
{
    ulp_internal_data_write(ulp_alive_offset, 0);
    delay(10);
    if (ulp_internal_data_read(ulp_alive_offset) == 0)
    {
        ulp_internal_start();
    }
}

#endif
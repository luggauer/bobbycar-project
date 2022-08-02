#include "adc1_read.h"
#include "math_functions.h"
#inlcude "config.h"
volatile int adc_throttle;
volatile float adc_steering;
volatile float desired_steering;

void init_gamepad(void* ignore){
    vTaskDelete(NULL);
}

void init_adc(void* ignore){
    init_adc(STEERING_PIN);
    #if(THROTTLE0_PIN != THROTTLE1_PIN)
        init_adc(THROTTLE1_PIN);
    #endif
    init_adc(THROTTLE0_PIN);
    vTaskDelete(NULL);
}

void init_sbus(void* ignore){
    vTaskDelete(NULL);
}

void gamepad_task(void* ignore){
    vTaskDelete(NULL);
}

void adc_task(void* ignore){
    while(true){
        adc_steering = calc_steering_eagle(clean_adc_steering(value_buffer(read_voltage(STEERING_PIN),0)));
        if(){
            #if(THROTTLE0_PIN != THROTTLE1_PIN)
                adc_throttle = throttle_calc(clean_adc_half(value_buffer(read_voltage(THROTTLE0_PIN),1)) - clean_adc_half(value_buffer(read_voltage(THROTTLE1_PIN),2)));
            #else
                adc_throttle = throttle_calc(clean_adc_full(value_buffer(read_voltage(THROTTLE0_PIN),1)));
            #endif
        }
    }
    vTaskDelay(1);
    vTaskDelete(NULL);
}

void sbus_task(void* ignore){
    vTaskDelete(NULL);
}
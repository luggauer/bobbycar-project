#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

//#include "adc1_read.h"
#include "math_functions.h"
#include "gamepad_manager.h"
#include "config.h"
#include <Arduino.h>
#include <stddef.h>

volatile int adc_throttle;
volatile float adc_steering;
volatile float desired_steering;

int get_throttle(){
    return adc_throttle;
}
float get_steering(){
    return adc_steering;
}
float get_des_steering(){
    return desired_steering;
}

void init_gamepad(void* ignore){
    init_gpm();
    vTaskDelete(NULL);
}

void init_adc_task(void* ignore){
    pinMode(STEERING_PIN, INPUT);
    #if(THROTTLE0_PIN != THROTTLE1_PIN)
        pinMode(THROTTLE1_PIN, INPUT);
    #endif
    pinMode(THROTTLE0_PIN, INPUT);
    vTaskDelete(NULL);
}

void init_sbus(void* ignore){
    vTaskDelete(NULL);
}

void gamepad_task(void* ignore){
    while(true){
        gpm_read();
        vTaskDelay(20);
    }
    vTaskDelete(NULL);
}

void adc_task(void* ignore){
    while(true){
        adc_steering = calc_steering_eagle(clean_adc_steering(value_buffer(analogRead(STEERING_PIN),0)));
        if(true){
            #if(THROTTLE0_PIN != THROTTLE1_PIN)
                adc_throttle = throttle_calc(clean_adc_half(value_buffer(analogRead(THROTTLE0_PIN),1)) - clean_adc_half(value_buffer(analogRead(THROTTLE1_PIN),2)));
            #else
                adc_throttle = throttle_calc(clean_adc_full(value_buffer(analogRead(THROTTLE0_PIN),1)));
            #endif
        }
        vTaskDelay(1);
    }
    vTaskDelete(NULL);
}

void sbus_task(void* ignore){
    vTaskDelete(NULL);
}
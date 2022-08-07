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
volatile int adc_steering;
volatile int desired_steering;
volatile bool active = false;


int get_throttle(){
    return throttle_calc(adc_throttle);
}
float get_steering(){
    return calc_steering_eagle(adc_steering);
}
float get_des_steering(){
    return desired_steering;
}
bool get_contoller_active(){
    return active;
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

void init_crsf(void* ignore){
    vTaskDelete(NULL);
}

void init_auto_reveser(void* ignore){
    vTaskDelete(NULL);
}

void gamepad_task(void* ignore){
    int pad_data[6];
    bool tmp;
    while(true){
        tmp = active;
        gpm_read(&pad_data[0],&pad_data[1],&tmp);
        active = tmp;
        vTaskDelay(20);
    }
    vTaskDelete(NULL);
}

void adc_task(void* ignore){
    while(true){
        adc_steering = clean_adc_steering(value_buffer(analogRead(STEERING_PIN),0));
        if(true){
            #if(THROTTLE0_PIN != THROTTLE1_PIN)
                adc_throttle = clean_adc_half(value_buffer(analogRead(THROTTLE0_PIN),1)) - clean_adc_half(value_buffer(analogRead(THROTTLE1_PIN),2));
            #else
                adc_throttle = clean_adc_full(value_buffer(analogRead(THROTTLE0_PIN),1));
            #endif
        }
        vTaskDelay(1);
    }
    vTaskDelete(NULL);
}

void sbus_task(void* ignore){
    vTaskDelete(NULL);
}

void crsf_task(void* ignore){
    vTaskDelete(NULL);
}


void auto_reverser_task(void* ignore){
    vTaskDelete(NULL);
}
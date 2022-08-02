#include "adc1_read.h"
#include "math_functions.h"

int throttle;
float steering;
float desired_steering;

void init_gamepad(void* ignore){
    vTaskDelete(NULL);
}

void init_adc(void* ignore){
    init_adc(32);
    init_adc(33);
    init_adc(37);
    vTaskDelete(NULL);
}

void init_sbus(void* ignore){
    vTaskDelete(NULL);
}

void gamepad_task(void* ignore){
    vTaskDelete(NULL);
}

void adc_task(void* ignore){
    vTaskDelete(NULL);
}

void sbus_task(void* ignore){
    vTaskDelete(NULL);
}
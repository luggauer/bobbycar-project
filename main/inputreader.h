#pragma once

#ifdef __cplusplus
extern "C" {
#endif
extern volatile float adc_steering;
extern volatile float desired_steering;
int get_throttle();
float get_steering();
float get_des_steering();

void init_gamepad(void* ignore);
void init_adc_task(void* ignore);
void init_sbus(void* ignore);

void gamepad_task(void* ignore);
void adc_task(void* ignore);
void sbus_task(void* ignore);
#ifdef __cplusplus
}
#endif
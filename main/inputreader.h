#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void init_servo(void* pin);
bool get_lock();
void set_lock(bool open);

void admin_set_lock(bool open, int time);
void admin_reset();

#ifdef __cplusplus
}
#endif
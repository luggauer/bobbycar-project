#include <math.h>
#include <stdint.h>

#include "defines.h"
#include "config.h"

uint32_t index_buff_vals[VAL_CNT];
uint32_t buff_vals[VAL_CNT][BUFFERSIZE];
uint32_t cur_buff_val_sum[VAL_CNT];

void init_buffer(){
    for (int i = 0; i < VAL_CNT; i++)
    {
        cur_buff_val_sum[i] = index_buff_vals[i] = 0;
        for (int j = 0; j < BUFFERSIZE; j++)
            cur_buff_val_sum[i] += (buff_vals[i][j] = ADC_MID);
    }
}

uint32_t value_buffer(uint32_t in, int val)
{
  cur_buff_val_sum[val] -= buff_vals[val][index_buff_vals[val]];
  cur_buff_val_sum[val] += (buff_vals[val][index_buff_vals[val]] = in);
  index_buff_vals[val] = (index_buff_vals[val] + 1) % (BUFFERSIZE);
  return (cur_buff_val_sum[val] / (BUFFERSIZE));
}


// bobbycar
int sign(float in){
  if(in > 0.0f)
    return 1;
   else if(in < 0.0f)
    return -1;
   else
    return 0;
}

int clean_adc_full(uint32_t inval)
{
  int outval = (int)(inval) - ADC_MID;
  int abs_outval = abs(outval);
  if (abs_outval < (DEAD_ZONE / 2)) // deadzone
    return 0;
  else
    abs_outval -= (DEAD_ZONE / 2);
  if (abs_outval > (ADC_MAX - ADC_MID - DEAD_ZONE * 3 / 2))
    return THROTTLE_MAX * sign(outval);
  return abs_outval * THROTTLE_MAX / (ADC_MAX - ADC_MID - DEAD_ZONE * 3 / 2) * sign(outval);
}

int clean_adc_half(uint32_t inval)
{
  int outval = (uint32_t)inval;
  if (abs(outval) > (ADC_MAX - ((DEAD_ZONE * 3) / 2)))
    return THROTTLE_MAX;
  else if(abs(outval) < (((DEAD_ZONE * 3) / 2)))
    return 0;
  return outval * (THROTTLE_MAX / 2) / (ADC_MAX - DEAD_ZONE * 3);
}

int throttle_calc(int cleaned_adc)
{
  return cleaned_adc < 0 ?
    ((cleaned_adc * cleaned_adc / THROTTLE_MAX) * (-2) + cleaned_adc) / 3 *THROTTLE_REVERSE_MAX / THROTTLE_MAX
    : ((cleaned_adc * cleaned_adc / THROTTLE_MAX) * 2 + cleaned_adc) / 3;
}

int calc_torque(int throttle, int breaks)
{
  if (breaks == 0)
  { // drive forward
    return throttle;
  }
  else if (breaks == THROTTLE_MAX)
  { // drive backwards
    return -throttle;
  }
  else
  {
    return throttle - breaks;
  }
}

float calc_steering_eagle(int inval)
{
  return (float)inval * STEERING_EAGLE_FACTOR;
}

void calc_torque_per_wheel(int throttle, float steering_eagle, int *torque)
{
    torque[0] = torque[1] = torque[2] = torque[3] = throttle;
}

inline void swp(int *x, int *y)
{
  int tmp = *x;
  *x = *y;
  *y = tmp;
}
inline void sort_array(int x[], int cnt)
{
  for (int y = 0; y < cnt - 1; y++)
    for (int z = y + 1; z < cnt; z++)
      if (x[y] > x[z])
        swp(&x[y], &x[z]);
}
int calc_median(int x[], int cnt)
{
  sort_array(x, cnt);
  if (cnt % 2)
    return x[cnt / 2];
  else
    return (x[cnt / 2] + x[cnt / 2 + 1]) / 2;
}
#pragma once

#include "driver/ledc.h"
#include "soc/ledc_struct.h"
// float base_speed = 0;
typedef struct
{
    ledc_channel_t forward_channel;
    ledc_channel_t reverse_channel;
    int fwd_gpio;
    int rev_gpio;
} motor_t;

void motor_set_speed(motor_t *m, int speed);
void motor_task(void *pvParams);
void motor_init();
extern float base_speed;
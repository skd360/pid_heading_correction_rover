#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include "soc/ledc_struct.h"
#include "soc/gpio_struct.h"
#include "freertos/task.h"
#include "driver/gpio.h"
// #include "esp_freertos_hooks.h"
#include "motor.h"
#include "control.h"
#include "imu.h"
// #include "skd_uart.h"
#include "pid.h"
#include <math.h>
// 1 motor 1 struct
// typedef struct
// {
//     ledc_channel_t forward_channel;
//     ledc_channel_t reverse_channel;
//     int fwd_gpio;
//     int rev_gpio;
// } motor_t;
float base_speed = 0; // pwm
// float target_speed = 50;
float step = 2;
float step_angle = 2;
float left, right, error, correction, right2, diff, delta, dt = 0, omega = 30;
int64_t state_start_time = 0;
// ledc has 8 channels
motor_t right_motors[2] = {{LEDC_CHANNEL_0, LEDC_CHANNEL_1, 25, 26}, {LEDC_CHANNEL_2, LEDC_CHANNEL_3, 32, 33}};
motor_t left_motors[2] = {{LEDC_CHANNEL_4, LEDC_CHANNEL_5, 27, 14}, {LEDC_CHANNEL_6, LEDC_CHANNEL_7, 18, 19}};
// static bool initialized = false;
// motor_t right_motors[2] = {25,26, 32, 33};
// motor_t left_motors[2] = {27, 14, 18, 19};

// void goforward_right(uint8_t a);
// void goreverse_right(uint8_t b);
// void goforward_left(uint8_t c);
// void goreverse_left(uint8_t d);
// #define STEP_INTERVAL 20000 // 20 ms
void motor_right_stop(void);
void motor_left_stop(void);
void set_pwm(ledc_channel_t ch, int duty);
void mo_init(motor_t *ch);
// int go_forward(int target_speed, int time_to_hold, int64_t *now);
// int scan_angle(int8_t first_angle, int8_t second_angle, int64_t *now);
// void motor_set_speed(motor_t *m, int speed);
// int set_speed = 40, state = 2;

void set_pwm(ledc_channel_t ch, int duty)
{
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, ch, duty);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, ch);
}

void motor_set_speed(motor_t *m, int speed)
{
    if (speed > 0)
    {
        set_pwm(m->reverse_channel, 0);
        set_pwm(m->forward_channel, speed);
    }
    else if (speed < 0)
    {
        set_pwm(m->forward_channel, 0);
        set_pwm(m->reverse_channel, -speed);
        // set_pwm(m->reverse_channel, abs(speed));// this is fine but dont do this if u are doing isr or very serious control loop or heavy math loops
    }
    else if (speed == 0)
    {
        set_pwm(m->forward_channel, 0);
        set_pwm(m->reverse_channel, 0);
    }
}
void mo_init(motor_t *ch)
{
    ledc_channel_config_t ledc_chan = {
        .gpio_num = ch->fwd_gpio,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = ch->forward_channel,
        .timer_sel = LEDC_TIMER_0,
        .hpoint = 0, // tells when the signal goes high
        .duty = 0,   // tells how long the signal is high
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_chan));
    ledc_channel_config_t ledc2_chan = {
        .gpio_num = ch->rev_gpio,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = ch->reverse_channel,
        .timer_sel = LEDC_TIMER_0,
        .hpoint = 0, // tells when the signal goes high
        .duty = 0,   // tells how long the signal is high
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc2_chan));
}

void motor_init()
{
    // do this only once, no need to repeat this for every motor
    ledc_timer_config_t ledc_config = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 18000,
        .clk_cfg = LEDC_APB_CLK,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .deconfigure = false};
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_config));

    // initialize the motors
    for (int i = 0; i < 2; i++)
    {
        mo_init(&right_motors[i]); // sends address of the struct at index i
        mo_init(&left_motors[i]);
    }

    motor_right_stop();
    motor_left_stop();
}

void motor_task(void *pvParams)
{
    while (1)
    {
        // ================= ERROR =================
        float error = normalize_angle(target_yaw - current_yaw);
        float correction = control_output;

        // ================= SMALL DEADZONE =================
        if (fabs(correction) < 5)
        {
            correction = 0;
        }

        // ================= SOFTEN NEAR TARGET =================
        if (fabs(error) < 2.0)
        {
            correction *= 0.2;
        }

        // ================= DIFFERENTIAL DRIVE =================
        float left  = base_speed - (correction);
        float right = base_speed + (correction);

        // ================= SAFETY: STOP CASE =================
        // if (base_speed == 0)
        // {
        //     motor_right_stop();
        //     motor_left_stop();
        //     vTaskDelay(pdMS_TO_TICKS(20));
        //     continue;
        // }

        // ================= CLAMP =================
        if (left > 255) left = 255;
        if (left < -255) left = -255;

        if (right > 255) right = 255;
        if (right < -255) right = -255;

        // ================= APPLY =================
        motor_set_speed(&right_motors[0], right*2);
        motor_set_speed(&right_motors[1], right);

        motor_set_speed(&left_motors[0], left);
        motor_set_speed(&left_motors[1], left);

        vTaskDelay(pdMS_TO_TICKS(20)); // 50 Hz
    }
}

// void motor_task(void *pvParams)
// {

//     while (1)
//     {

//         int64_t now = esp_timer_get_time();

//         switch (state)
//         {
//         case 0:
//             // ramping_up_target_ speed, time to hold the speed and automatically ramp down and stop
//             state = go_forward(50, 10000000, &now);
//             break;
//         case 1:
//             // stop position
//             motor_right_stop();
//             motor_left_stop();
//             break;
//         case 2:
//             state = scan_angle(-45, 45, &now);
//             break;
//             // goes to next iteration where state will be still 1 sooo motors are stopped
//             // if u skip continue the remaining pid code will be still working.

//             // case 2:
//             //     //take right or left turn, input:- angle (+ for right, - for left)
//             //     break;
//             // case 3:
//             //    //scan mode (scan for + and - 15 degrees and then correct your heading to < 2 degrees error)
//             //    break;

//         default:
//             break;
//         }
//         error = normalize_angle(target_yaw - current_yaw);
//         correction = control_output;
//         if (fabs(correction) < 40)
//         {
//             if (correction > 0)
//                 correction = 40;
//             else if (correction < 0)
//                 correction = -40;
//         }

//         if (fabs(error) < 2.0) // within 4 degrees
//         {
//             correction *= 0.2;
//             // motor_set_speed(&right_motors[0], 0);
//             // motor_set_speed(&right_motors[1], 0);

//             // motor_set_speed(&left_motors[0], 0);
//             // motor_set_speed(&left_motors[1], 0);

//             // continue;
//         }
//         // for (int i = 0; i <= base_speed; i++)
//         // {

//         // else if (fabs(error) < 8.0)
//         // {
//         //     correction *= 0.4; // soften
//         // }

//         left = base_speed - correction;
//         right = base_speed + correction;
//         right2 = base_speed + correction * 2;
//         // float left2 = base_speed - correction;

//         // clamp
//         if (left > 255)
//             left = 255;
//         if (left < -255)
//             left = -255;

//         if (right > 255)
//             right = 255;
//         if (right < -255)
//             right = -255;

//         if (right2 > 255)
//             right2 = 255;
//         if (right2 < -255)
//             right2 = -255;

//         // if (left2 > 255)
//         //     left2 = 255;
//         // if (left2 < -255)
//         //     left2 = -255;

//         motor_set_speed(&right_motors[0], right2);
//         motor_set_speed(&right_motors[1], right);

//         motor_set_speed(&left_motors[0], left); // this is m3 thats having problem
//         motor_set_speed(&left_motors[1], left);

//         vTaskDelay(pdMS_TO_TICKS(20)); // 50hz loop
//     }
// }
//     vTaskDelay(pdMS_TO_TICKS(10000));
//     for (int speed = base_speed; speed >= 0; speed--)
//     {
//         left = speed - correction;
//         right = speed + correction;
//         right2 = speed + correction * 2;
//         // float left2 = base_speed - correction;

//         // clamp
//         if (left > 255)
//             left = 255;
//         if (left < -255)
//             left = -255;

//         if (right > 255)
//             right = 255;
//         if (right < -255)
//             right = -255;

//         if (right2 > 255)
//             right2 = 255;
//         if (right2 < -255)
//             right2 = -255;
//         motor_set_speed(&right_motors[0], right2);
//         motor_set_speed(&right_motors[1], right);

//         motor_set_speed(&left_motors[0], left);
//         motor_set_speed(&left_motors[1], left);

//         vTaskDelay(pdMS_TO_TICKS(50)); // smooth slowdown
//     }
//     motor_right_stop();
//     motor_left_stop();
//     // vTaskDelay(pdMS_TO_TICKS(500));
//     vTaskDelay(pdMS_TO_TICKS(10000));
// }

//     while (1)
//     {
//         vTaskDelay(pdMS_TO_TICKS(1000));

//         ESP_LOGI("Wheeelzzz", "Rampingggg \n");
//         for (int i = 0; i<= set_speed; i++)
//         {
//             motor_set_speed(&right_motors[0], i+54);
//             motor_set_speed(&right_motors[1], i+12);

//             motor_set_speed(&left_motors[0], i);
//             motor_set_speed(&left_motors[1], i);

//             ESP_LOGI("forwarddd", "PWM: %d \n", i);
//             vTaskDelay(pdMS_TO_TICKS(30));
//         }
//         vTaskDelay(pdMS_TO_TICKS(10000));
//         ESP_LOGI("Wheeelzzz", "slowin_dowwwnnnn \n");
//         for (int i = set_speed; i>= 0; i--)
//         {
//             motor_set_speed(&right_motors[0], i+54);
//             motor_set_speed(&right_motors[1], i+12);

//             motor_set_speed(&left_motors[0], i);
//             motor_set_speed(&left_motors[1], i);
//             ESP_LOGI("forwarddd", "PWM: %d \n", i);
//             vTaskDelay(pdMS_TO_TICKS(30));
//         }
//         motor_right_stop();
//         motor_left_stop();
//         // vTaskDelay(pdMS_TO_TICKS(500));
//         vTaskDelay(pdMS_TO_TICKS(10000));

//         ESP_LOGI("Wheeelzzz", "reverseeee_ramp \n");
//         for (int i = 0; i >= -set_speed; i--)
//         {
//             // motor_set_speed(&right_motors[0], i);
//             motor_set_speed(&right_motors[1], i);

//             motor_set_speed(&left_motors[0], i);
//             motor_set_speed(&left_motors[1], i);
//             ESP_LOGI("reverseee", "PWM: %d \n", i);
//             vTaskDelay(pdMS_TO_TICKS(30));
//         }
//         vTaskDelay(pdMS_TO_TICKS(70000));

//         ESP_LOGI("Wheeelzzz", "reverse_ramp_dowwwnnnn \n");
//         for (int i = -set_speed; i <= 0; i++)
//         {
//             // motor_set_speed(&right_motors[0], i);
//             motor_set_speed(&right_motors[1], i);

//             motor_set_speed(&left_motors[0], i);
//             motor_set_speed(&left_motors[1], i);
//             ESP_LOGI("reverseeee", "PWM: %d \n", i);
//             vTaskDelay(pdMS_TO_TICKS(30));
//         }
//         motor_right_stop();
//         motor_left_stop();
//         vTaskDelay(pdMS_TO_TICKS(10000));
//     }

// }
// // }
// int go_forward(int target_speed, int time_to_hold, int64_t *now)
// {
//     // static bool speed = false; // if this is not static it will get reset everytime
//     static uint8_t phase = 0;
//     if (phase == 0)
//     {
//         target_yaw = current_yaw;
//         initialized = true;
//         phase += 1;
//     }
//     if (phase == 1)
//     {
//         if (base_speed < target_speed)
//         {
//             base_speed += step;
//         }
//         else if (base_speed >= target_speed)
//         {
//             base_speed = target_speed;
//             state_start_time = *now;
//             phase = 2;
//             // target_speed = 0;
//             //  speed = true;
//         }
//     }
//     if (phase == 2)
//     {
//         if ((*now - state_start_time) >= time_to_hold)
//         {
//             if (base_speed > 0)
//             {
//                 base_speed -= step;
//             }
//             else
//             {
//                 base_speed = 0;
//                 return 1;
//                 phase = 0;
//                 // speed = false;
//             }
//         }
//     }
//     return 0;
// }
// int scan_angle(int8_t first_angle, int8_t second_angle, int64_t *now)
// {
//     static int reference_angle = 0;
//     static uint8_t phase = 0;
//     static bool reached = false;
//     // static int64_t last_update_time = 0;
//     float local_error = normalize_angle(target_yaw - current_yaw);
//     if (!initialized)
//     {
//         reference_angle = current_yaw;
//         initialized = true;
//     }
//     // if (last_update_time == 0)
//     // {
//     //     last_update_time = *now;
//     // }
//     if (phase == 0)
//     {
//         diff = normalize_angle(reference_angle + first_angle);
//         delta = normalize_angle(diff - target_yaw);
//         // if ((*now - last_update_time) >= STEP_INTERVAL)
//         // {
//         if (fabs(delta) > step_angle)
//             target_yaw += (delta > 0 ? step_angle : -step_angle);
//         else
//             target_yaw = diff;

//         // last_update_time = *now;
//         // }

//         // omega = 30.0; // deg/sec (slow scan)

//         // dt = (*now - last_update_time) / 1000000.0; // sec

//         // if (fabs(delta) > omega * dt)
//         // {
//         //     target_yaw += (delta > 0 ? omega * dt : -omega * dt);
//         // }
//         // else
//         // {
//         //     target_yaw = diff;
//         // }

//         // last_update_time = *now;

//         if (fabs(local_error) < 2.0)
//         {
//             if (!reached)
//             {
//                 state_start_time = *now;
//                 reached = true;
//             }
//         }
//         if (reached && (*now - state_start_time) >= 1000000)
//         {
//             phase = 1;
//             reached = false;
//         }
//     }
//     else if (phase == 1)
//     {
//         diff = normalize_angle(reference_angle + second_angle);
//         delta = normalize_angle(diff - target_yaw);
//         // if ((*now - last_update_time) >= STEP_INTERVAL)
//         // {
//         if (fabs(delta) > step_angle)
//             target_yaw += (delta > 0 ? step_angle : -step_angle);
//         else
//             target_yaw = diff;

//         // last_update_time = *now;
//         // }
//         // omega = 30.0; // deg/sec (slow scan)

//         // dt = (*now - last_update_time) / 1000000.0; // sec

//         // if (fabs(delta) > omega * dt)
//         // {
//         //     target_yaw += (delta > 0 ? omega * dt : -omega * dt);
//         // }
//         // else
//         // {
//         //     target_yaw = diff;
//         // }
//         // last_update_time = *now;

//         if (fabs(local_error) < 2.0)
//         {
//             if (!reached)
//             {
//                 state_start_time = *now;
//                 reached = true;
//             }
//         }
//         if (reached && (*now - state_start_time) >= 1000000)
//         {
//             phase = 2;
//             reached = false;
//         }
//     }
//     else if (phase == 2)
//     {
//         diff = normalize_angle(reference_angle);
//         delta = normalize_angle(diff - target_yaw);
//         // if ((*now - last_update_time) >= STEP_INTERVAL)
//         // {
//         if (fabs(delta) > step_angle)
//             target_yaw += (delta > 0 ? step_angle : -step_angle);
//         else
//             target_yaw = diff;

//         //     last_update_time = *now;
//         // }
//         // omega = 30.0; // deg/sec (slow scan)

//         // dt = (*now - last_update_time) / 1000000.0; // sec

//         // if (fabs(delta) > omega * dt)
//         // {
//         //     target_yaw += (delta > 0 ? omega * dt : -omega * dt);
//         // }
//         // else
//         // {
//         //     target_yaw = diff;
//         // }
//         // last_update_time = *now;

//         if (fabs(local_error) < 2.0)
//         {
//             if (!reached)
//             {
//                 state_start_time = *now;
//                 reached = true;
//             }
//         }
//         if (reached && (*now - state_start_time) >= 1000000)
//         {
//             phase = 0;
//             reached = false;
//             initialized = false;
//             return 1;
//         }
//     }
//     return 2;
// }
void motor_right_stop()
{
    set_pwm(LEDC_CHANNEL_0, 0);
    set_pwm(LEDC_CHANNEL_1, 0);
    set_pwm(LEDC_CHANNEL_2, 0);
    set_pwm(LEDC_CHANNEL_3, 0);
    // ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 0);
    // ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
    // ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, 0);
    // ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);

    // ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2, 0);
    // ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2);
    // ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_3, 0);
    // ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_3);
}
void motor_left_stop()
{
    set_pwm(LEDC_CHANNEL_5, 0);
    set_pwm(LEDC_CHANNEL_7, 0);
    set_pwm(LEDC_CHANNEL_4, 0);
    set_pwm(LEDC_CHANNEL_6, 0);
    // ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_4, 0);
    // ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_4);
    // ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_5, 0);
    // ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_5);

    // ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_6, 0);
    // ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_6);
    // ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_7, 0);
    // ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_7);
}
// void goforward_right(uint8_t a)
// {
//     set_pwm(LEDC_CHANNEL_1, 0);
//     set_pwm(LEDC_CHANNEL_3, 0);

//     // Apply forward to both right motors
//     set_pwm(LEDC_CHANNEL_0, a + 15);
//     set_pwm(LEDC_CHANNEL_2, a);

//     // ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, 0);
//     // ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);
//     // ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_3, 0);
//     // ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_3);

//     // ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, a + 15);
//     // ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
//     // ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2, a);
//     // ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2);
// }
// void goreverse_right(uint8_t b)
// {
//     // Zero forward channels first (both right motors)
//     set_pwm(LEDC_CHANNEL_0, 0);
//     set_pwm(LEDC_CHANNEL_2, 0);
//     // ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 0);
//     // ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
//     // ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2, 0);
//     // ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2);

//     // Apply reverse to both right motors
//     set_pwm(LEDC_CHANNEL_1, b + 15);
//     set_pwm(LEDC_CHANNEL_3, b);
//     // ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, b + 15);
//     // ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);
//     // ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_3, b);
//     // ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_3);
// }
// void goforward_left(uint8_t c)
// {
//     // Zero reverse channels first (both left motors)
//     set_pwm(LEDC_CHANNEL_5, 0);
//     set_pwm(LEDC_CHANNEL_7, 0);
//     // ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_5, 0);
//     // ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_5);
//     // ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_7, 0);
//     // ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_7);

//     // Apply forward to both left motors
//     set_pwm(LEDC_CHANNEL_4, c);
//     set_pwm(LEDC_CHANNEL_6, c);
//     // ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_4, c);
//     // ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_4);
//     // ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_6, c);
//     // ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_6);
// }
// void goreverse_left(uint8_t d)
// {
//     // Zero forward channels first (both left motors)
//     set_pwm(LEDC_CHANNEL_4, 0);
//     set_pwm(LEDC_CHANNEL_6, 0);
//     // ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_4, 0);
//     // ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_4);
//     // ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_6, 0);
//     // ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_6);

//     // Apply reverse to both left motors
//     set_pwm(LEDC_CHANNEL_5, d);
//     set_pwm(LEDC_CHANNEL_7, d);
//     // ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_5, d);
//     // ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_5);
//     // ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_7, d);
//     // ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_7);

// stuff from previous script that i used for a different project, please ignore
//  ESP_ERROR_CHECK(ledc_fade_func_install(0));
//  // Ramp UP: 0 → 255 over 2000ms
// ledc_set_fade_with_time(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 255, 2000);
// ledc_fade_start(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, LEDC_FADE_WAIT_DONE);
// // Ramp DOWN: 255 → 0 over 2000ms
// ledc_set_fade_with_time(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 0, 2000);
// ledc_fade_start(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, LEDC_FADE_WAIT_DONE);
#include "pid.h"
#include "control.h"
#include "freertos/FreeRTOS.h"
#include "imu.h"
static float kp = 8, ki = 0, kd = 1;
static float prev_error = 0;
static float integral = 0;
// extern volatile target_yaw;
// extern volatile current_yaw;
// volatile float control_output;
void pid_task(void *pvParams)
{
    const float dt = 0.02; // 20ms, 50hz
    while (1)
    {
        float error = normalize_angle(target_yaw - current_yaw);
        integral += error * dt;
        float derivative = (error - prev_error) / dt;
        control_output = kp * error + ki * integral + kd * derivative;
        prev_error = error;

        printf("yaw: %.2f | target: %.2f | error: %.2f | out: %.2f\n",
               current_yaw, target_yaw,
               normalize_angle(target_yaw - current_yaw),
               control_output);

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// printf("yaw: %.2f | target: %.2f | error: %.2f | out: %.2f\n",
//        current_yaw, target_yaw,
//        normalize_angle(target_yaw - current_yaw),
//        control_output);
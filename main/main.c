#include <stdio.h>
#include "imu.h"
#include "pid.h"
#include "motor.h"
#include "control.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
void app_main(void)
{
vTaskDelay(pdMS_TO_TICKS(1000));

bool check = false;
while (!check)
{
    check = bno_init();
    if (!check)
    {
        ESP_LOGE("MAIN", "BNO init failed, retrying in 500ms...");
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

ESP_LOGI("MAIN", "BNO initialized successfully");

motor_init();

target_yaw = current_yaw;

xTaskCreate(pid_task,    "pid",        4096, NULL, 4, NULL);
xTaskCreate(motor_task,  "motor_task", 4096, NULL, 3, NULL);
}

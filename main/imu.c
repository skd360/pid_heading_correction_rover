#include <stdio.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "imu.h"
#include "control.h"

#define I2C_PORT I2C_NUM_0
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define I2C_FREQ_HZ 100000

#define BNO_ADDR 0x28
#define DEFAULT_START_ADDR 0x00

uint8_t reg = 0x00;
uint8_t chip_id = 0;

i2c_master_bus_config_t i2c_mst_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = I2C_PORT,
    .sda_io_num = I2C_SDA_PIN,
    .scl_io_num = I2C_SCL_PIN,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true,
};

i2c_master_bus_handle_t bus_handle;

i2c_device_config_t dev_cnfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = BNO_ADDR,
    .scl_speed_hz = I2C_FREQ_HZ,
};
i2c_master_dev_handle_t dev_handle;

float normalize_angle(float angle_deg)
{
    while (angle_deg > 180.0f)
        angle_deg -= 360.0f;
    while (angle_deg < -180.0f)
        angle_deg += 360.0f;
    return angle_deg;
}

// bno_euler_read task //
void bno_euler_task(void *pvParams)
{
    while (1)
    {
        uint8_t yaw_lsb = 0x1A;
        uint8_t reg6[2];
        // gets yaw lsb at 0x1A and increments till next bytes till 0x1F
        // in one transition, this is called 'Burst Read'
        // 6 bytes(6x8 bits ie 8 bits in one compartment of reg6) we have YAW(heading), ROLL, PITCH:- lsb, msb
        ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, &yaw_lsb, 1, reg6, 2, 50));
        // ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, &yaw_lsb, 1, reg6, 6, 50));

        // refer table 3-28 and 3-29
        // according to this each parameter is a signed 16 bit data (2 bytes)
        // as bno is a 8 bit microcontroller it stores lsb in one memory adress and msb in another
        //  so we have to convert them to int16_t and left shift MSB and bitwise OR with LSB
        // here we have combined the bits to a signed int 16 bit data
        int16_t yaw_raw = (int16_t)((reg6[1] << 8) | reg6[0]);
        //    int16_t roll_raw = (int16_t)((reg6[3] << 8) | reg6[2]);
        //  int16_t pitch_raw = (int16_t)((reg6[5] << 8) | reg6[4]);

        // scale this by dividing by 16.0f (refer table 3-29)
        float yaw_t = yaw_raw / 16.0f; // u can do 16.0 but its a double instead of float which is heavier for microcontrollers
        // float roll_t = roll_raw / 16.0f;
        // float pitch_t = pitch_raw / 16.0f;

        current_yaw = normalize_angle(yaw_t); // u can do 16.0 but its a double instead of float which is heavier for microcontrollers
        // float roll = normalize_angle(roll_t);
        // float pitch = normalize_angle(pitch_t);

        // read and decode calib_stat
        // calib stat is at 0x35
        // uint8_t calib_stat_read = 0x35;
        // uint8_t calib_stat;
        // ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, &calib_stat_read, 1, &calib_stat, 1, 50));

        // uint8_t mag_calib = calib_stat & 0x03;
        // uint8_t acc_calib = ((calib_stat >> 2) & 0x03);
        // uint8_t gyro_calib = ((calib_stat >> 4) & 0x03);
        // uint8_t sys_calib = ((calib_stat >> 6) & 0x03);

        // printf("|EULER| y: %.2f | r: %.2f | p: %.2f | \n |CALIB| sys: %d | gyro: %d | acc: %d  \n ", yaw, roll, pitch, sys_calib, gyro_calib, acc_calib);
        //printf("|EULER| y: %.2f \n ", yaw);

        vTaskDelay(pdMS_TO_TICKS(20)); //20 = 50hz, 50 = 20 hz
    }
}
bool bno_init()
{
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));
    vTaskDelay(pdMS_TO_TICKS(700));
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cnfg, &dev_handle));

    // xTaskCreate(detect_bno, "bno_init", 2048, NULL, 8, NULL);
    //  detect bno task //
    esp_err_t probe_error = i2c_master_probe(bus_handle, BNO_ADDR, 100);

    if (probe_error == ESP_OK)
        printf("BNO55 detecteddd!!! \n");
    else
    {
        printf("BNO055 not found : %s \n", esp_err_to_name(probe_error));
        vTaskDelete(NULL);
    }

    esp_err_t err = i2c_master_transmit_receive(dev_handle, &reg, 1, &chip_id, 1, -1);

    if (err == ESP_OK)
        printf("BNO055 Chip id = 0x%02X \n", chip_id);
    else
        printf("I2C read failed : %s \n", esp_err_to_name(err));

    // page id //
    uint8_t buf[2] = {0x07, 0x00}; // page id = 0
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, buf, 2, 50));

    // check operation mode, OPR_MODE = 0X3D //
    // refer table 3.5 of bno055 datasheet //
    uint8_t opr_mode = 0x3D;
    uint8_t op_m = 0;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, &opr_mode, 1, &op_m, 1, 50));
    printf("operation_mode = 0x%02X \n", op_m);

    // force config mode //
    // just for practice //
    uint8_t conf_m[2] = {0x3D, 0x00};
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, conf_m, 2, 50));
    vTaskDelay(pdMS_TO_TICKS(30));

    // set PWR_MODE = NORMAL //
    uint8_t pwr_m[2] = {0x3E, 0x00};
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, pwr_m, 2, 50));

    // set units (eg degrees, radians etc etc), refer table 3.11 //
    uint8_t unitz_m[2] = {0x3B, 0x00};
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, unitz_m, 2, 50));

    // set to IMU mode, OPR_MODE -> 0x00001000 //
    uint8_t imu_m[2] = {0x3D, 0x08};
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, imu_m, 2, 50));
    vTaskDelay(pdMS_TO_TICKS(30));

    // poll SYS_STATUS 0X39 and SYS_ERR 0X3A //
    uint8_t sys_stat = 0, sys_err = 0;
    uint8_t SYS_ST = 0x39, SYS_ER = 0x3A;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, &SYS_ST, 1, &sys_stat, 1, 50));
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, &SYS_ER, 1, &sys_err, 1, 50));
    printf(" sys_stat = %d sys_err = %d \n", sys_stat, sys_err);
    if (sys_stat == 5 && sys_err == 0)
    {
        xTaskCreate(bno_euler_task, "bno_euler", 4096, NULL, 5, NULL);
        return true;
    }
    else{
        printf("somethings fishy....... \n");
        return false;
    }
}

// void app_main(void)
// {
//     bno_init();
// }
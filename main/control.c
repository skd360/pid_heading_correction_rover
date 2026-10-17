#pragma message("CONTROL.C IS BEING COMPILED")
#include <stdio.h>
#include "control.h"
#include "motor.h"
// #define MAX_SPEED 126
// #define MIN_SPEED 60           // 🔥 critical (tune this)
// #define DIST_STOP_THRESHOLD 10
// #define MAX_DIST 100           // max expected distance (cm)
// control.c
volatile float current_yaw = 0;
volatile float target_yaw = 0;
volatile float control_output = 0;
volatile float heading_error = 0;
extern float base_speed;
// void control_update(int x, int y)
// {
//     float heading_error = (float)x;

//     // ================= HEADING =================
//     if (heading_error > 30) heading_error = 30;
//     if (heading_error < -30) heading_error = -30;

//     target_yaw = current_yaw + heading_error;

//     if (target_yaw > 180) target_yaw -= 360;
//     if (target_yaw < -180) target_yaw += 360;

//     // ================= DISTANCE → SPEED =================
//     float distance = (float)y;

//     if (distance < DIST_STOP_THRESHOLD)
//     {
//         base_speed = 0;
//     }
//     else
//     {
//         // normalize distance (0 → 1)
//         float norm = distance / MAX_DIST;
//         if (norm > 1.0) norm = 1.0;

//         // map to usable speed range
//         base_speed = MIN_SPEED + norm * (MAX_SPEED - MIN_SPEED);
//         //base_speed = MIN_SPEED + (norm * norm) * (MAX_SPEED - MIN_SPEED);
//     }

//     printf("dist: %.2f | speed: %.2f\n", distance, base_speed);

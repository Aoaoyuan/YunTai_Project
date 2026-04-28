#ifndef __IMU_FILTER_H
#define __IMU_FILTER_H

#include "main.h"
#include <math.h>

#define RAD2DEG    57.29578f
#define DEG2RAD    0.0174533f
#define IMU_DT     0.001f   // 1ms 采样

// Mahony 结构体
typedef struct
{
    float q0, q1, q2, q3;
    float exInt, eyInt, ezInt;
    float Kp;
    float Ki;
}Mahony_t;

// 欧拉角
typedef struct
{
    float roll;
    float pitch;
    float yaw;
}Euler_t;

extern Mahony_t mahony;
extern Euler_t euler;

// 函数声明
void Mahony_Init(float Kp, float Ki);
void Mahony_Update(float gx, float gy, float gz, float ax, float ay, float az);
void Quat_To_Euler(void);

#endif

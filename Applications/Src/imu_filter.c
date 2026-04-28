#include "imu_filter.h"

Mahony_t mahony;
Euler_t  euler;

void Mahony_Init(float Kp, float Ki)
{
    mahony.q0 = 1.0f;
    mahony.q1 = 0.0f;
    mahony.q2 = 0.0f;
    mahony.q3 = 0.0f;
    mahony.exInt = 0.0f;
    mahony.eyInt = 0.0f;
    mahony.ezInt = 0.0f;
    mahony.Kp = Kp;
    mahony.Ki = Ki;
}

// Mahony 6轴融合
void Mahony_Update(float gx, float gy, float gz, float ax, float ay, float az)
{
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;

    // 角速度转 rad/s
    gx *= DEG2RAD;
    gy *= DEG2RAD;
    gz *= DEG2RAD;

    // 加速度归一化
    recipNorm = sqrtf(ax*ax + ay*ay + az*az);
    if(recipNorm == 0.0f) return;
    recipNorm = 1.0f / recipNorm;
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // 计算估计重力向量
    halfvx = mahony.q1*mahony.q3 - mahony.q0*mahony.q2;
    halfvy = mahony.q0*mahony.q1 + mahony.q2*mahony.q3;
    halfvz = mahony.q0*mahony.q0 - 0.5f + mahony.q3*mahony.q3;

    // 向量误差
    halfex = ay * halfvz - az * halfvy;
    halfey = az * halfvx - ax * halfvz;
    halfez = ax * halfvy - ay * halfvx;

    // 积分修正
    if(mahony.Ki > 0)
    {
        mahony.exInt += halfex * IMU_DT;
        mahony.eyInt += halfey * IMU_DT;
        mahony.ezInt += halfez * IMU_DT;
        gx += mahony.Ki * mahony.exInt;
        gy += mahony.Ki * mahony.eyInt;
        gz += mahony.Ki * mahony.ezInt;
    }

    // 比例修正
    gx += mahony.Kp * halfex;
    gy += mahony.Kp * halfey;
    gz += mahony.Kp * halfez;

    // 四元数微分更新
    float q0 = mahony.q0;
    float q1 = mahony.q1;
    float q2 = mahony.q2;
    float q3 = mahony.q3;

    mahony.q0 += 0.5f * (-q1*gx - q2*gy - q3*gz) * IMU_DT;
    mahony.q1 += 0.5f * ( q0*gx + q2*gz - q3*gy) * IMU_DT;
    mahony.q2 += 0.5f * ( q0*gy - q1*gz + q3*gx) * IMU_DT;
    mahony.q3 += 0.5f * ( q0*gz + q1*gy - q2*gx) * IMU_DT;

    // 四元数归一化
    recipNorm = 1.0f / sqrtf(mahony.q0*mahony.q0 + mahony.q1*mahony.q1 + mahony.q2*mahony.q2 + mahony.q3*mahony.q3);
    mahony.q0 *= recipNorm;
    mahony.q1 *= recipNorm;
    mahony.q2 *= recipNorm;
    mahony.q3 *= recipNorm;
}

// 四元数转欧拉角(roll/pitch/yaw 角度)
void Quat_To_Euler(void)
{
    euler.roll  = atan2f(2.0f*(mahony.q0*mahony.q1 + mahony.q2*mahony.q3), 1.0f - 2.0f*(mahony.q1*mahony.q1 + mahony.q2*mahony.q2)) * RAD2DEG;
    euler.pitch = asinf(2.0f*(mahony.q0*mahony.q2 - mahony.q1*mahony.q3)) * RAD2DEG;
    euler.yaw   = atan2f(2.0f*(mahony.q0*mahony.q3 + mahony.q1*mahony.q2), 1.0f - 2.0f*(mahony.q2*mahony.q2 + mahony.q3*mahony.q3)) * RAD2DEG;

    // Yaw 限幅 -180 ~ 180 防止爆炸
    if(euler.yaw > 180)  euler.yaw -= 360;
    if(euler.yaw < -180) euler.yaw += 360;
}


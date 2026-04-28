/**
  ******************************************************************************
  * @file    motor_control.c
  * @brief   三轴云台电机限位与控制（仅阻止越界，不限制内部自由运动）
  ******************************************************************************
  */

#include "motor_control.h"
#include "QD4310.h"
#include <math.h>

extern QD4310_t Motor_0;
extern QD4310_t Motor_1;
extern QD4310_t Motor_2;

float motor0_out = 0.0f;
float motor1_out = 0.0f;
float motor2_out = 0.0f;

/* 边界阈值（弧度），小于此距离视为“已到边界” */
#define BOUNDARY_THRESHOLD 0.05f

/* 退出禁区时的固定速度大小 (rad/s) */
#define EXIT_SPEED 50.0f

/* 角度归一化到 [0, 2π) */
static float NormalizeAngle(float ang)
{
    float two_pi = 2.0f * (float)M_PI;
    ang = fmodf(ang, two_pi);
    if (ang < 0) ang += two_pi;
    return ang;
}

/**
  * @brief 电机1限位：安全区 [0,1.11] ∪ [4.73, 2π)
  *        只在紧贴边界且方向向内时阻止；允许内部自由运动。
  */
static float Limit_M1(float speed, float ang)
{
    const float RIGHT = 1.11f;
    const float LEFT  = 4.73f;
    ang = NormalizeAngle(ang);

    // 右侧安全区 [0, RIGHT]
    if (ang <= RIGHT)
    {
        // 当角度接近右边界且速度向增大方向（进入禁区）时阻止
        if (ang >= RIGHT - BOUNDARY_THRESHOLD && speed > 0)
            return 0.0f;
        return speed;
    }
    // 左侧安全区 [LEFT, 2π)
    else if (ang >= LEFT)
    {
        // 当角度接近左边界且速度向减小方向（进入禁区）时阻止
        if (ang <= LEFT + BOUNDARY_THRESHOLD && speed < 0)
            return 0.0f;
        return speed;
    }
    // 禁区 (RIGHT, LEFT) —— 一般不会发生，但若发生则强制向最近边界退出
    else
    {
        float dist_to_right = ang - RIGHT;
        float dist_to_left  = LEFT - ang;
        float exit_speed = (dist_to_right < dist_to_left) ? -EXIT_SPEED : EXIT_SPEED;
        return exit_speed;
    }
}

/**
  * @brief 电机2限位：安全区 [0,1.60] ∪ [5.38, 2π)
  *        只在紧贴边界且方向向内时阻止；允许内部自由运动。
  */
static float Limit_M2(float speed, float ang)
{
    const float RIGHT = 1.60f;
    const float LEFT  = 5.38f;
    ang = NormalizeAngle(ang);

    if (ang <= RIGHT)
    {
        if (ang >= RIGHT - BOUNDARY_THRESHOLD && speed > 0)
            return 0.0f;
        return speed;
    }
    else if (ang >= LEFT)
    {
        if (ang <= LEFT + BOUNDARY_THRESHOLD && speed < 0)
            return 0.0f;
        return speed;
    }
    else
    {
        float dist_to_right = ang - RIGHT;
        float dist_to_left  = LEFT - ang;
        float exit_speed = (dist_to_right < dist_to_left) ? -EXIT_SPEED : EXIT_SPEED;
        return exit_speed;
    }
}

/**
  * @brief 总限位函数（在 CAN 任务中周期性调用）
  */
void Motor_Speed_Limit_All(void)
{
    motor1_out = Limit_M1(motor1_out, Motor_1.angle);
    motor2_out = Limit_M2(motor2_out, Motor_2.angle);
    // motor0 无限位
}

/**
  * @brief 初始化所有电机（使能、零速）
  */
void Motor_Init_All(void)
{
    QD4310_Enable(&Motor_0);
    QD4310_Enable(&Motor_1);
    QD4310_Enable(&Motor_2);
    osDelay(100);

    QD4310_SetSpeed(&Motor_0, 0);
    QD4310_SetSpeed(&Motor_1, 0);
    QD4310_SetSpeed(&Motor_2, 0);
}


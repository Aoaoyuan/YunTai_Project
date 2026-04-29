/**
  ******************************************************************************
  * @file    motor_control.c
  * @brief   三轴云台电机限位与控制（避障版：障碍在5.38）
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

/* === 独立归零速度配置 (rad/s) === */
#define HOMING_SPEED_M0 30.0f 
#define HOMING_SPEED_M1 50.0f 
#define HOMING_SPEED_M2 30.0f 

/* === 零点偏移配置 (rad) === */
#define ZERO_OFFSET_M0 0.0f 
#define ZERO_OFFSET_M1 0.25f 
#define ZERO_OFFSET_M2 0.15f 

/* === 避障分界点 (rad) === */
/* 障碍物位于 5.38 rad。以此值为界决定归零方向 */
#define M2_OBSTACLE_POS 5.3f 

/* 归零完成的误差范围 (rad) */
#define HOMING_TOLERANCE 0.05f

/* 角度归一化到 [0, 2π) */
static float NormalizeAngle(float ang)
{
    float two_pi = 2.0f * (float)M_PI;
    ang = fmodf(ang, two_pi);
    if (ang < 0) ang += two_pi;
    return ang;
}

/**
  * @brief 获取带有偏移补偿的有效角度
  */
static float Get_Effective_Angle(float raw_angle, float offset)
{
    return NormalizeAngle(raw_angle + offset);
}

/**
  * @brief 电机1限位：安全区 [0,1.11] ∪ [4.73, 2π)
  */
static float Limit_M1(float speed, float ang)
{
    const float RIGHT = 1.11f;
    const float LEFT  = 4.73f;
    ang = NormalizeAngle(ang);

    if (ang <= RIGHT)
    {
        if (ang >= (RIGHT - BOUNDARY_THRESHOLD) && speed > 0) return 0.0f;
        return speed;
    }
    else if (ang >= LEFT)
    {
        if (ang <= (LEFT + BOUNDARY_THRESHOLD) && speed < 0) return 0.0f;
        return speed;
    }
    else
    {
        return speed;
    }
}

/**
  * @brief 电机2限位：安全区 [0,1.60] ∪ [5.38, 2π)
  * @note LEFT 设为 5.38f，与障碍物位置一致
  */
static float Limit_M2(float speed, float ang)
{
    const float RIGHT = 1.60f;
    const float LEFT  = 5.38f; // 障碍物位置
    ang = NormalizeAngle(ang);

    if (ang <= RIGHT)
    {
        if (ang >= (RIGHT - BOUNDARY_THRESHOLD) && speed > 0) return 0.0f;
        return speed;
    }
    else if (ang >= LEFT)
    {
        if (ang <= (LEFT + BOUNDARY_THRESHOLD) && speed < 0) return 0.0f;
        return speed;
    }
    else
    {
        // 禁区 (1.60 ~ 5.38)
        return speed;
    }
}

void Motor_Speed_Limit_All(void)
{
    motor1_out = Limit_M1(motor1_out, Motor_1.angle);
    motor2_out = Limit_M2(motor2_out, Motor_2.angle);
}

/**
  * @brief 辅助函数：通过速度环将单个电机归零（带避障逻辑）
  */
static void Home_Motor_With_Offset(QD4310_t *motor, uint8_t id, float offset)
{
    // 【重要优化】等待有效的角度反馈
    osDelay(200); 
    
    float current_ang = NormalizeAngle(motor->angle);
    float speed_cmd = 0.0f;
    float home_speed_abs = 0.0f;
    uint32_t timeout = 0;
    const uint32_t MAX_TIMEOUT = 5000;

    // 目标角度：我们希望归零后， motor->angle 等于 -offset
    float target_raw_angle = NormalizeAngle(-offset);
    
    // 1. 根据电机ID选择归零速度并确定旋转方向
    if (id == 1) 
    {
        home_speed_abs = HOMING_SPEED_M1;
        // 电机1策略：大于2.3时逆时针，否则顺时针
        // 注意：请根据实际测试确认正负号对应方向
        if (current_ang > 2.3f)
        {
            speed_cmd = home_speed_abs; // 假设负是逆时针/减小角度
        }
        else
        {
            speed_cmd = -home_speed_abs;  // 假设正是顺时针/增加角度
        }
    }
    else if (id == 2)
    {
        home_speed_abs = HOMING_SPEED_M2;
        
        // === 电机2 避障归零策略 (障碍在 5.38) ===
        
        if (current_ang > M2_OBSTACLE_POS) 
        {
            // 情况 A: 角度在 5.38 ~ 6.28 之间
            // 障碍在身后(逆时针方向)，不能往回走。
            // 必须 **增加角度** (顺时针)，冲过 2π 回到 0。
            speed_cmd = home_speed_abs; // 给正速度
        }
        else 
        {
            // 情况 B: 角度在 0 ~ 5.38 之间
            // 障碍在前方(顺时针远处)或身后。
            // 直接 **减小角度** (逆时针) 回到 0 是最安全且最短的路径。
            speed_cmd =- home_speed_abs; // 给负速度
        }
    }
    else 
    {
        // 电机0：默认最短路径
        home_speed_abs = HOMING_SPEED_M0;
        float diff = target_raw_angle - current_ang;
        while (diff > (float)M_PI) diff -= 2.0f * (float)M_PI;
        while (diff < -(float)M_PI) diff += 2.0f * (float)M_PI;
        
        speed_cmd = (diff > 0) ? home_speed_abs : -home_speed_abs;
        
        if (fabsf(diff) < HOMING_TOLERANCE) return;
    }

    if (speed_cmd == 0.0f) return;

    // 2. 循环发送速度指令，直到到达目标原始角度
    while (timeout < MAX_TIMEOUT)
    {
        current_ang = NormalizeAngle(motor->angle);
        
        // 计算当前角度与目标原始角度的最短差值（仅用于判断到位）
        float diff = target_raw_angle - current_ang;
        while (diff > (float)M_PI) diff -= 2.0f * (float)M_PI;
        while (diff < -(float)M_PI) diff += 2.0f * (float)M_PI;
        
        // 到位判断
        if (fabsf(diff) < HOMING_TOLERANCE)
        {
            break; 
        }

        QD4310_SetSpeed(motor, speed_cmd);
        osDelay(1); 
        timeout++;
    }

    // 3. 停止电机
    QD4310_SetSpeed(motor, 0);
    osDelay(100); 
}

/**
  * @brief 初始化所有电机（使能、速度环归零）
  */
void Motor_Init_All(void)
{
    // 1. 使能电机
    QD4310_Enable(&Motor_0);
    QD4310_Enable(&Motor_1);
    QD4310_Enable(&Motor_2);
    
    osDelay(100);

    // 2. 依次归零
    Home_Motor_With_Offset(&Motor_0, 0, ZERO_OFFSET_M0);
    Home_Motor_With_Offset(&Motor_1, 1, ZERO_OFFSET_M1);
    Home_Motor_With_Offset(&Motor_2, 2, ZERO_OFFSET_M2);

    // 3. 确保所有输出变量清零
    motor0_out = 0.0f;
    motor1_out = 0.0f;
    motor2_out = 0.0f;
    
    QD4310_SetSpeed(&Motor_0, 0);
    QD4310_SetSpeed(&Motor_1, 0);
    QD4310_SetSpeed(&Motor_2, 0);
}
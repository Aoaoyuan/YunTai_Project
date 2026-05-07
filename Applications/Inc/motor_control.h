#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#include "main.h"
#include "QD4310.h"
#include "cmsis_os.h"
#include "imu_filter.h"

extern QD4310_t Motor_0;
extern QD4310_t Motor_1;
extern QD4310_t Motor_2;

// 电机1 左4.73  右1.11
#define M1_LEFT  4.73f
#define M1_RIGHT 1.11f

// 电机2 左5.38  右1.6
#define M2_LEFT  5.38f
#define M2_RIGHT 1.60f

extern float motor0_out;
extern float motor1_out;
extern float motor2_out;

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float last_error;
    float output_limit; // 输出限幅 (rad/s)
} PID_TypeDef;

extern uint8_t Balance_Mode_Enable; // 平衡模式开关
extern float Target_Roll_Angle;     // 目标 Roll (度)
extern float Target_Pitch_Angle;    // 目标 Pitch (度)
extern float Target_Yaw_Angle;

extern PID_TypeDef Pid_Roll;
extern PID_TypeDef Pid_Pitch;

void Motor_Speed_Limit_All(void);
void Motor_Init_All(void);
void Motor_Balance_Control(void);   

#endif
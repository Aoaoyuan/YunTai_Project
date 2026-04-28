#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#include "main.h"
#include "QD4310.h"
#include "cmsis_os.h"

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

void Motor_Speed_Limit_All(void);
void Motor_Init_All(void);

#endif
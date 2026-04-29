/**
  ******************************************************************************
  * @file    motor_control.c
  * @brief   三轴云台电机限位与控制（避障版：障碍在5.38）
  ******************************************************************************
  */

#include "motor_control.h"
#include "QD4310.h"
#include <math.h>
#include "imu_filter.h"

extern QD4310_t Motor_0;
extern QD4310_t Motor_1;
extern QD4310_t Motor_2;

float motor0_out = 0.0f;
float motor1_out = 0.0f;
float motor2_out = 0.0f;

/* === 全局输出限幅配置 (rad/s) === */
#define MOTOR_OUTPUT_LIMIT 150.0f
/* === 平衡模式配置 === */
uint8_t Balance_Mode_Enable = 0; // 0:关闭, 1:开启自稳

float Target_Roll_Angle = 0.0f;
float Target_Pitch_Angle = 0.0f;


/* === 重力补偿物理参数 (需根据实际硬件填写) === */
// M1 (Roll): 假设负载 0.2kg, 质心距轴 0.05m
#define M1_MASS   0.2f   // kg
#define M1_ARM_L  0.08f  // meters
// M2 (Pitch): 假设负载 0.2kg, 质心距轴 0.05m
#define M2_MASS   0.00f   // kg
#define M2_ARM_L  0.05f  // meters

#define GRAVITY   9.8f   // m/s^2

/* === PID 参数初始化 (需根据实际机械结构调试) === */
// 初始 Kp 给一个小值，防止上电瞬间剧烈抖动
PID_TypeDef Pid_Roll  = { .Kp = 300.0f, .Ki = 10.0f, .Kd = 0.2f, .output_limit = 50.0f };
PID_TypeDef Pid_Pitch = { .Kp = 10.0f, .Ki = 0.0f, .Kd = 0.2f, .output_limit = 30.0f };

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




/**
  * @brief 计算重力补偿力矩 (单位: rad/s, 对应电机速度环指令)
  * @note 这里简化处理，假设电机速度环增益已知，或者直接输出力矩比例值
  *       由于 QD4310 是速度环控制，我们需要将 力矩(N.m) 映射为 速度(rad/s)
  *       这里引入一个经验系数 K_GRAVITY_TO_SPEED
  */
#define K_GRAVITY_TO_SPEED  100.0f  // 经验系数：需要调试，表示 1 N.m 力矩对应多少 rad/s 的速度指令

static float Calculate_Gravity_Compensation_Roll(float roll_rad)
{
    // 重力力矩 T = m * g * L * sin(roll)
    // 当 roll=0 (水平) 时，sin(0)=0，补偿为 0
    // 当 roll=90 (垂直) 时，sin(90)=1，补偿最大
    float torque = M1_MASS * GRAVITY * M1_ARM_L * cosf(roll_rad);
    return torque * K_GRAVITY_TO_SPEED;
}

static float Calculate_Gravity_Compensation_Pitch(float pitch_rad)
{
    // Pitch 轴同理
    float torque = M2_MASS * GRAVITY * M2_ARM_L * cosf(pitch_rad);
    return torque * K_GRAVITY_TO_SPEED;
}








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


//////////////////////////////////////////////////////////////////////////////////////////////////



/* === PID 计算核心函数 === */
static float PID_Compute(PID_TypeDef *pid, float target, float measure, float dt)
{
    float error = target - measure;
    
    // 积分项
    pid->integral += error * dt;
    // 积分限幅 (防止积分饱和)
    if (pid->integral > 100.0f) pid->integral = 100.0f;
    if (pid->integral < -100.0f) pid->integral = -100.0f;
    
    // 微分项
    float derivative = (error - pid->last_error) / dt;
    pid->last_error = error;
    
    // 计算输出
    float output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
    
    // 输出限幅
    if (output > pid->output_limit) output = pid->output_limit;
    if (output < -pid->output_limit) output = -pid->output_limit;
    
    return output;
}

/**
  * @brief 平衡控制主函数
  * @note 建议在 500Hz - 1kHz 频率下调用 (dt = 0.002s - 0.001s)
  */
void Motor_Balance_Control(void)
{
    if (!Balance_Mode_Enable) 
    {
        motor1_out = 0.0f;
        motor2_out = 0.0f;
        return; 
    }

    const float dt = 0.002f; 

    // 1. 获取当前姿态 (弧度)
    float current_roll_rad  = euler.roll * (float)M_PI / 180.0f;
    float current_pitch_rad = euler.pitch * (float)M_PI / 180.0f;
    
    float target_roll_rad  = Target_Roll_Angle * (float)M_PI / 180.0f;
    float target_pitch_rad = Target_Pitch_Angle * (float)M_PI / 180.0f;

    // 2. 计算重力补偿 (前馈)
    float gravity_comp_roll  = Calculate_Gravity_Compensation_Roll(current_roll_rad);
    float gravity_comp_pitch = Calculate_Gravity_Compensation_Pitch(current_pitch_rad);

    // 3. 计算 PID 输出 (反馈)
    // 注意：因为加了前馈，PID 的误差应该很小，所以输出也会很小
    float pid_out_roll  =PID_Compute(&Pid_Roll, target_roll_rad, current_roll_rad, dt);
    float pid_out_pitch =PID_Compute(&Pid_Pitch, target_pitch_rad, current_pitch_rad, dt);
    
    // 4. 最终输出 = 前馈 + 反馈
    // 【重要】：方向判断！
    // 如果重力补偿方向反了，云台会加速倒下。请根据现象调整正负号。
    // 通常：如果云台向右歪 (roll>0)，重力想让它更向右，电机需要向左用力 (负速度/力矩)。
    // sin(roll) 在 roll>0 时为正。如果电机定义正向是向右，那么补偿应该是负的。
    
    motor1_out = -gravity_comp_roll + pid_out_roll;  // 尝试加负号
    motor2_out = gravity_comp_pitch + pid_out_pitch; // 尝试加负号
    
    // 5. 全局限幅 (使用头部定义的宏)
    if (motor1_out > MOTOR_OUTPUT_LIMIT) motor1_out = MOTOR_OUTPUT_LIMIT;
    if (motor1_out < -MOTOR_OUTPUT_LIMIT) motor1_out = -MOTOR_OUTPUT_LIMIT;
    if (motor2_out > MOTOR_OUTPUT_LIMIT) motor2_out = MOTOR_OUTPUT_LIMIT;
    if (motor2_out < -MOTOR_OUTPUT_LIMIT) motor2_out = -MOTOR_OUTPUT_LIMIT;
    
    motor0_out = 0.0f; 
}




/////////////////////////////////////////////////////////////////////////////////////////////////////////////////



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

    Balance_Mode_Enable = 1; // 初始化完成后开启自稳
    Target_Roll_Angle = 0.0f;
    Target_Pitch_Angle = 0.0f;
}
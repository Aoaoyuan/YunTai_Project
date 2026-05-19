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
#define MOTOR_OUTPUT_LIMIT 1000.0f
/* === 平衡模式配置 === */
uint8_t Balance_Mode_Enable = 0; // 0:关闭, 1:开启自稳

float Target_Roll_Angle = -6.0f;
float Target_Pitch_Angle = 0.0f;
float Target_Yaw_Angle = 0.0f;
float yaw_zero_offset = 0.0f;       // 上电自动标定

/* === IMU 零点校准偏移 (单位: 度) === */
// 调试方法：把云台调到肉眼水平，观察 OLED 上的 R: 值。
// 如果显示 R: 3.5，则把 ROLL_OFFSET_DEG 设为 3.5
#define ROLL_OFFSET_DEG   0.0f  
#define PITCH_OFFSET_DEG  0.0f
/* === 重力补偿物理参数 (需根据实际硬件填写) === */
// M1 (Roll): 假设负载 0.2kg, 质心距轴 0.05m
#define M1_MASS   0.5f   // kg
#define M1_ARM_L  0.1f  // meters
// M2 (Pitch): 假设负载 0.2kg, 质心距轴 0.05m
#define M2_MASS   0.00f   // kg
#define M2_ARM_L  0.05f  // meters

#define GRAVITY   9.8f   // m/s^2

/* === PID 参数初始化 (需根据实际机械结构调试) === */
// 初始 Kp 给一个小值，防止上电瞬间剧烈抖动
// PID_TypeDef Pid_Roll  = { .Kp = 300.0f, .Ki = 10.0f, .Kd = 0.2f, .output_limit = 50.0f };
// PID_TypeDef Pid_Pitch = { .Kp = 10.0f, .Ki = 0.0f, .Kd = 0.2f, .output_limit = 30.0f };

// Yaw 轴 (M0)
#define YAW_KP      28.0f
#define YAW_KI      0.0f
#define YAW_KD      00.40f
#define YAW_INTEGRAL_LIMIT 30.0f  // 积分限幅（根据表现再放宽或收紧）
#define YAW_OUTPUT_LIMIT 200.0f   // rad/s，根据实际响应调整
/* === 非对称 PID 参数配置 === */
// Roll 轴 (M1)
#define ROLL_KP_POS   30.0f   // 正角度（例如向右）时的 Kp
#define ROLL_KD_POS   0.50f    // 正角度时的 Kd
#define ROLL_KP_NEG   30.0f   // 负角度（例如向左）时的 Kp
#define ROLL_KD_NEG   0.50f    // 负角度时的 Kd
#define ROLL_KI_POS   0.0f    // 正角度时的 Ki，起始值建议小
#define ROLL_KI_NEG   0.0f    // 负角度时的 Ki，起始值建议更小
#define ROLL_INTEGRAL_LIMIT 30.0f  // 积分限幅（根据表现再放宽或收紧）

// Pitch 轴 (M2) - 同理可设
#define PITCH_KP_POS  40.0f
#define PITCH_KD_POS  0.2f
#define PITCH_KP_NEG  40.0f
#define PITCH_KD_NEG  0.20f
/* 边界阈值（弧度），小于此距离视为“已到边界” */
#define BOUNDARY_THRESHOLD 0.05f

/* === 独立归零速度配置 (rad/s) === */
#define HOMING_SPEED_M0 30.0f 
#define HOMING_SPEED_M1 60.0f 
#define HOMING_SPEED_M2 30.0f 

/* === 零点偏移配置 (rad) === */
#define ZERO_OFFSET_M0 0.0f 
#define ZERO_OFFSET_M1 -0.05f 
#define ZERO_OFFSET_M2 0.15f 

/* === 避障分界点 (rad) === */
/* 障碍物位于 5.38 rad。以此值为界决定归零方向 */
#define M2_OBSTACLE_POS 5.34f 

/* 归零完成的误差范围 (rad) */
#define HOMING_TOLERANCE 0.05f




/**
  * @brief 计算重力补偿力矩 (单位: rad/s, 对应电机速度环指令)
  * @note 这里简化处理，假设电机速度环增益已知，或者直接输出力矩比例值
  *       由于 QD4310 是速度环控制，我们需要将 力矩(N.m) 映射为 速度(rad/s)
  *       这里引入一个经验系数 K_GRAVITY_TO_SPEED
  */
#define K_GRAVITY_TO_SPEED  0.0f  // 经验系数：需要调试，表示 1 N.m 力矩对应多少 rad/s 的速度指令

static float Calculate_Gravity_Compensation_Roll(float roll_rad)
{
    // 重力力矩 T = m * g * L * sin(roll)
    // 当 roll=0 (水平) 时，sin(0)=0，补偿为 0
    // 当 roll=90 (垂直) 时，sin(90)=1，补偿最大
    float torque = M1_MASS * GRAVITY * M1_ARM_L * sinf(roll_rad);
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

// /**
//   * @brief 平衡控制主函数
//   * @note 建议在 500Hz - 1kHz 频率下调用 (dt = 0.002s - 0.001s)
//   */
// void Motor_Balance_Control(void)
// {
//     if (!Balance_Mode_Enable) 
//     {
//         motor1_out = 0.0f;
//         motor2_out = 0.0f;
//         return; 
//     }

//     const float dt = 0.002f; 

//     // 1. 获取当前姿态 (弧度)
//     float current_roll_rad  = euler.roll * (float)M_PI / 180.0f;
//     float current_pitch_rad = euler.pitch * (float)M_PI / 180.0f;
    
//     float target_roll_rad  = Target_Roll_Angle * (float)M_PI / 180.0f;
//     float target_pitch_rad = Target_Pitch_Angle * (float)M_PI / 180.0f;

//     // 2. 计算重力补偿 (前馈)
//     float gravity_comp_roll  = Calculate_Gravity_Compensation_Roll(current_roll_rad);
//     float gravity_comp_pitch = Calculate_Gravity_Compensation_Pitch(current_pitch_rad);

//     // 3. 计算 PID 输出 (反馈)
//     // 注意：因为加了前馈，PID 的误差应该很小，所以输出也会很小
//     float pid_out_roll  =PID_Compute(&Pid_Roll, target_roll_rad, current_roll_rad, dt);
//     float pid_out_pitch =PID_Compute(&Pid_Pitch, target_pitch_rad, current_pitch_rad, dt);
    
//     // 4. 最终输出 = 前馈 + 反馈
//     // 【重要】：方向判断！
//     // 如果重力补偿方向反了，云台会加速倒下。请根据现象调整正负号。
//     // 通常：如果云台向右歪 (roll>0)，重力想让它更向右，电机需要向左用力 (负速度/力矩)。
//     // sin(roll) 在 roll>0 时为正。如果电机定义正向是向右，那么补偿应该是负的。
    
//     motor1_out = -gravity_comp_roll + pid_out_roll;  // 尝试加负号
//     motor2_out = gravity_comp_pitch + pid_out_pitch; // 尝试加负号
    
//     // 5. 全局限幅 (使用头部定义的宏)
//     if (motor1_out > MOTOR_OUTPUT_LIMIT) motor1_out = MOTOR_OUTPUT_LIMIT;
//     if (motor1_out < -MOTOR_OUTPUT_LIMIT) motor1_out = -MOTOR_OUTPUT_LIMIT;
//     if (motor2_out > MOTOR_OUTPUT_LIMIT) motor2_out = MOTOR_OUTPUT_LIMIT;
//     if (motor2_out < -MOTOR_OUTPUT_LIMIT) motor2_out = -MOTOR_OUTPUT_LIMIT;
    
//     motor0_out = 0.0f; 
// }


// /**
//   * @brief 平衡控制主函数
//   */
// void Motor_Balance_Control(void)
// {
//     if (!Balance_Mode_Enable) 
//     {
//         motor1_out = 0.0f;
//         motor2_out = 0.0f;
//         return; 
//     }

//     const float dt = 0.002f; 
// // 1. 获取原始姿态并【减去零点偏移】
//     float raw_roll_deg  = euler.roll - ROLL_OFFSET_DEG;
//     float raw_pitch_deg = euler.pitch - PITCH_OFFSET_DEG;

//     // 转换为弧度
//     float current_roll_rad  = raw_roll_deg * (float)M_PI / 180.0f;
//     float current_pitch_rad = raw_pitch_deg * (float)M_PI / 180.0f;
    
//     float target_roll_rad  = Target_Roll_Angle * (float)M_PI / 180.0f;
//     float target_pitch_rad = Target_Pitch_Angle * (float)M_PI / 180.0f;
//     // 2. 计算重力补偿 (前馈)
//     float gravity_comp_roll  = Calculate_Gravity_Compensation_Roll(current_roll_rad);
//     float gravity_comp_pitch = Calculate_Gravity_Compensation_Pitch(current_pitch_rad);

//     // 3. 计算误差
//     float error_roll  = target_roll_rad - current_roll_rad;
//     float error_pitch = target_pitch_rad - current_pitch_rad;

//     /* 4. 选择非对称 P、D、I */
//     float kp_roll, kd_roll, ki_roll;
//     float kp_pitch, kd_pitch;

//     if (error_roll > 0.0f) {
//         kp_roll = ROLL_KP_POS;
//         kd_roll = ROLL_KD_POS;
//         ki_roll = ROLL_KI_POS;
//     } else {
//         kp_roll = ROLL_KP_NEG;
//         kd_roll = ROLL_KD_NEG;
//         ki_roll = ROLL_KI_NEG;
//     }

//     if (error_pitch > 0.0f) {
//         kp_pitch = PITCH_KP_POS;
//         kd_pitch = PITCH_KD_POS;
//     } else {
//         kp_pitch = PITCH_KP_NEG;
//         kd_pitch = PITCH_KD_NEG;
//     }

//     /* 5. 积分 + 微分 状态（静态变量保留跨次采样） */
//     static float last_error_roll = 0.0f;
//     static float last_error_pitch = 0.0f;
//     static float integral_roll = 0.0f;
//     static int last_sign_roll = 0;

//     int sign_roll = (error_roll > 0.0f) ? 1 : (error_roll < 0.0f) ? -1 : 0;
//     /* 跨零清积分，避免反向大积分 */
//     if (sign_roll != last_sign_roll) {
//         integral_roll = 0.0f;
//         last_sign_roll = sign_roll;
//     }

//     /* 积分累加与限幅（防止积分风暴） */
//     integral_roll += error_roll * dt;
//     if (integral_roll > ROLL_INTEGRAL_LIMIT) integral_roll = ROLL_INTEGRAL_LIMIT;
//     if (integral_roll < -ROLL_INTEGRAL_LIMIT) integral_roll = -ROLL_INTEGRAL_LIMIT;

//     /* 微分项 */
//     float derivative_roll  = (error_roll - last_error_roll) / dt;
//     float derivative_pitch = (error_pitch - last_error_pitch) / dt;

//     last_error_roll  = error_roll;
//     last_error_pitch = error_pitch;

//     /* 6. PID 输出（包含积分） */
//     float pid_out_roll  = (kp_roll * error_roll) + (ki_roll * integral_roll) + (kd_roll * derivative_roll);
//     float pid_out_pitch = (kp_pitch * error_pitch) + (kd_pitch * derivative_pitch);
//     // 6. 最终输出 = 前馈 + 反馈
//     // 注意：这里的正负号需要根据你之前的调试结果确定
//     motor1_out = -gravity_comp_roll +pid_out_roll ;  //
//     motor2_out = -gravity_comp_pitch +pid_out_pitch ; //
    
//     // 7. 全局限幅
//     if (motor1_out > MOTOR_OUTPUT_LIMIT) motor1_out = MOTOR_OUTPUT_LIMIT;
//     if (motor1_out < -MOTOR_OUTPUT_LIMIT) motor1_out = -MOTOR_OUTPUT_LIMIT;
//     if (motor2_out > MOTOR_OUTPUT_LIMIT) motor2_out = MOTOR_OUTPUT_LIMIT;
//     if (motor2_out < -MOTOR_OUTPUT_LIMIT) motor2_out = -MOTOR_OUTPUT_LIMIT;
    
//     motor0_out = 0.0f; 
// }


/**
  * @brief 平衡控制主函数（包含 Roll, Pitch, Yaw）
  * @note 建议 500Hz 调用 (dt = 0.002s)
  */
void Motor_Balance_Control(void)
{
    if (!Balance_Mode_Enable) 
    {
        motor1_out = 0.0f;
        motor2_out = 0.0f;
        motor0_out = 0.0f;
        return; 
    }

    const float dt = 0.002f; 

    // ====================== Roll 轴 (M1) ======================
    float raw_roll_deg  = euler.roll - ROLL_OFFSET_DEG;
    float current_roll_rad  = raw_roll_deg * (float)M_PI / 180.0f;
    float target_roll_rad  = Target_Roll_Angle * (float)M_PI / 180.0f;
    float gravity_comp_roll = Calculate_Gravity_Compensation_Roll(current_roll_rad);

    float error_roll = target_roll_rad - current_roll_rad;
    float kp_roll, kd_roll, ki_roll;
    if (error_roll > 0.0f) {
        kp_roll = ROLL_KP_POS;
        kd_roll = ROLL_KD_POS;
        ki_roll = ROLL_KI_POS;
    } else {
        kp_roll = ROLL_KP_NEG;
        kd_roll = ROLL_KD_NEG;
        ki_roll = ROLL_KI_NEG;
    }

    static float last_error_roll = 0.0f;
    static float integral_roll = 0.0f;
    static int last_sign_roll = 0;
    int sign_roll = (error_roll > 0.0f) ? 1 : (error_roll < 0.0f) ? -1 : 0;
    if (sign_roll != last_sign_roll) {
        integral_roll = 0.0f;
        last_sign_roll = sign_roll;
    }
    integral_roll += error_roll * dt;
    if (integral_roll > ROLL_INTEGRAL_LIMIT) integral_roll = ROLL_INTEGRAL_LIMIT;
    if (integral_roll < -ROLL_INTEGRAL_LIMIT) integral_roll = -ROLL_INTEGRAL_LIMIT;

    float derivative_roll = (error_roll - last_error_roll) / dt;
    last_error_roll = error_roll;

    float pid_out_roll = kp_roll * error_roll + ki_roll * integral_roll + kd_roll * derivative_roll;
    motor1_out = -gravity_comp_roll + pid_out_roll;

    // ====================== Pitch 轴 (M2) ======================
    float raw_pitch_deg = euler.pitch - PITCH_OFFSET_DEG;
    float current_pitch_rad = raw_pitch_deg * (float)M_PI / 180.0f;
    float target_pitch_rad = Target_Pitch_Angle * (float)M_PI / 180.0f;
    float gravity_comp_pitch = Calculate_Gravity_Compensation_Pitch(current_pitch_rad);

    float error_pitch = target_pitch_rad - current_pitch_rad;
    float kp_pitch, kd_pitch;
    if (error_pitch > 0.0f) {
        kp_pitch = PITCH_KP_POS;
        kd_pitch = PITCH_KD_POS;
    } else {
        kp_pitch = PITCH_KP_NEG;
        kd_pitch = PITCH_KD_NEG;
    }

    static float last_error_pitch = 0.0f;
    static float integral_pitch = 0.0f;      // Pitch 可选用积分，此处保留但未用
    static int last_sign_pitch = 0;
    int sign_pitch = (error_pitch > 0.0f) ? 1 : (error_pitch < 0.0f) ? -1 : 0;
    if (sign_pitch != last_sign_pitch) {
        integral_pitch = 0.0f;
        last_sign_pitch = sign_pitch;
    }
    integral_pitch += error_pitch * dt;
    // 若需要积分限幅，可自行添加宏，此处暂不启用积分
    float derivative_pitch = (error_pitch - last_error_pitch) / dt;
    last_error_pitch = error_pitch;

    float pid_out_pitch = kp_pitch * error_pitch + kd_pitch * derivative_pitch;   // 未使用积分
    motor2_out = -gravity_comp_pitch + pid_out_pitch;

    // // ====================== Yaw 轴 (M0) ======================
    // // 注：需要全局变量 Target_Yaw_Angle (度)
    // float target_yaw_rad = Target_Yaw_Angle * (float)M_PI / 180.0f;
    // float current_yaw_rad = euler.yaw * (float)M_PI / 180.0f;   // euler.yaw 范围假设为 ±180°

    // // 误差归一化到 [-pi, pi] 区间，避免 360° 跳变
    // float error_yaw = target_yaw_rad - current_yaw_rad;
    // error_yaw = fmodf(error_yaw, 2.0f * (float)M_PI);
    // if (error_yaw > (float)M_PI) error_yaw -= 2.0f * (float)M_PI;
    // if (error_yaw < -(float)M_PI) error_yaw += 2.0f * (float)M_PI;

    // // 静态 PID 变量 (Yaw 通常不需要积分，但预留)
    // static float last_error_yaw = 0.0f;
    // static float integral_yaw = 0.0f;
    // static int last_sign_yaw = 0;
    // int sign_yaw = (error_yaw > 0.0f) ? 1 : (error_yaw < 0.0f) ? -1 : 0;
    // if (sign_yaw != last_sign_yaw) {
    //     integral_yaw = 0.0f;
    //     last_sign_yaw = sign_yaw;
    // }
    // integral_yaw += error_yaw * dt;
    // if (integral_yaw > YAW_INTEGRAL_LIMIT) integral_yaw = YAW_INTEGRAL_LIMIT;
    // if (integral_yaw < -YAW_INTEGRAL_LIMIT) integral_yaw = -YAW_INTEGRAL_LIMIT;

    // float derivative_yaw = (error_yaw - last_error_yaw) / dt;
    // last_error_yaw = error_yaw;

    // float pid_out_yaw = YAW_KP * error_yaw + YAW_KI * integral_yaw + YAW_KD * derivative_yaw;
    // motor0_out = pid_out_yaw;   // Yaw 轴无重力补偿
    // ====================== Yaw 轴 (M0) ======================
    float target_yaw_rad = Target_Yaw_Angle * (float)M_PI / 180.0f;

    // 计算相对于上电零点的实际 yaw 角度
    float current_yaw_relative = euler.yaw - yaw_zero_offset;
    float current_yaw_rad = current_yaw_relative * (float)M_PI / 180.0f;

    // 误差归一化到 [-pi, pi] 区间，避免 360° 跳变
    float error_yaw = target_yaw_rad - current_yaw_rad;
    error_yaw = fmodf(error_yaw, 2.0f * (float)M_PI);
    if (error_yaw > (float)M_PI) error_yaw -= 2.0f * (float)M_PI;
    if (error_yaw < -(float)M_PI) error_yaw += 2.0f * (float)M_PI;

    // 静态 PID 变量
    static float last_error_yaw = 0.0f;
    static float integral_yaw = 0.0f;
    static int last_sign_yaw = 0;
    int sign_yaw = (error_yaw > 0.0f) ? 1 : (error_yaw < 0.0f) ? -1 : 0;
    if (sign_yaw != last_sign_yaw) {
        integral_yaw = 0.0f;
        last_sign_yaw = sign_yaw;
    }
    integral_yaw += error_yaw * dt;
    if (integral_yaw > YAW_INTEGRAL_LIMIT) integral_yaw = YAW_INTEGRAL_LIMIT;
    if (integral_yaw < -YAW_INTEGRAL_LIMIT) integral_yaw = -YAW_INTEGRAL_LIMIT;

    float derivative_yaw = (error_yaw - last_error_yaw) / dt;
    last_error_yaw = error_yaw;

    float pid_out_yaw = YAW_KP * error_yaw + YAW_KI * integral_yaw + YAW_KD * derivative_yaw;
    motor0_out = pid_out_yaw;

    // ====================== 全轴输出限幅 ======================
    if (motor0_out > MOTOR_OUTPUT_LIMIT) motor0_out = MOTOR_OUTPUT_LIMIT;
    if (motor0_out < -MOTOR_OUTPUT_LIMIT) motor0_out = -MOTOR_OUTPUT_LIMIT;
    if (motor1_out > MOTOR_OUTPUT_LIMIT) motor1_out = MOTOR_OUTPUT_LIMIT;
    if (motor1_out < -MOTOR_OUTPUT_LIMIT) motor1_out = -MOTOR_OUTPUT_LIMIT;
    if (motor2_out > MOTOR_OUTPUT_LIMIT) motor2_out = MOTOR_OUTPUT_LIMIT;
    if (motor2_out < -MOTOR_OUTPUT_LIMIT) motor2_out = -MOTOR_OUTPUT_LIMIT;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/**
  * @brief 设定三轴目标跟踪角度（度）
  * @param roll_deg  目标 Roll  角度，限幅 ±45°
  * @param pitch_deg 目标 Pitch 角度，限幅 ±45°
  * @param yaw_deg   目标 Yaw   角度（相对上电零位），无硬限幅（可按需添加 ±180° 限制）
  */
void Set_Target_Angles(float roll_deg, float pitch_deg, float yaw_deg)
{
    if (roll_deg > 90.0f)  roll_deg = 90.0f;
    if (roll_deg < -90.0f) roll_deg = -90.0f;
    if (pitch_deg > 45.0f)  pitch_deg = 45.0f;
    if (pitch_deg < -45.0f) pitch_deg = -45.0f;

    Target_Roll_Angle  = roll_deg;
    Target_Pitch_Angle = pitch_deg;
    Target_Yaw_Angle   = yaw_deg;
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

     // ---- 自动 Yaw 归零 ----
    osDelay(200);                            // 等待姿态解算稳定
    yaw_zero_offset = euler.yaw;            // 记录当前朝向作为零度
    // Target_Yaw_Angle = 0.0f;                // 默认希望保持当前朝向
    Set_Target_Angles(-6.0f, 0.0f, 0.0f);   // 内部会设置 Target_Roll/Pitch/Yaw_Angle
    Balance_Mode_Enable = 1; // 初始化完成后开启自稳

}
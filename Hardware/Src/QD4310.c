//
// Created by jason on 25-10-16.
//
#include "main.h"
#include "QD4310.h"

// 限制函数，用于替代C++的std::clamp
static float QD4310_Clamp(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

// 发送命令到电机
void QD4310_SendCommand(QD4310_t *motor, QD4310_Command_t cmd, int16_t value) {
    static uint8_t TxBuffer[3];
    TxBuffer[0] = (uint8_t)cmd;
    // 将int16_t值拆分为两个字节
    TxBuffer[1] = (uint8_t)(value & 0xFF);
    TxBuffer[2] = (uint8_t)((value >> 8) & 0xFF);

    FDCAN_TxHeaderTypeDef TxHeader;

    TxHeader.Identifier = 0x400 + motor->id;      // 标准 ID
    TxHeader.IdType = FDCAN_STANDARD_ID;           // 指定为标准 ID (对应旧的 CAN_ID_STD)
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_3;       // 数据长度 3 字节 (对应旧的 DLC=3)
    
    // 以下配置保持默认或根据需求调整
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;        // 经典 CAN 模式，不使用比特率切换
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;         // 设置为经典 CAN 格式
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;
    // 3. 发送到 TX FIFO
    // 注意：这里使用 motor->hfdcan，请确保 QD4310.h 中结构体成员已改为 FDCAN_HandleTypeDef *hfdcan;
    if (HAL_FDCAN_AddMessageToTxFifoQ(motor->hfdcan, &TxHeader, TxBuffer) != HAL_OK)
    {
        // 可选：处理发送失败，例如 FIFO 已满
        // 通常可以在这里加一个简单的重试或错误标志
    }

    // 4. 等待 FIFO 有空闲位置 (防止连续快速发送导致丢包)
    // HAL_FDCAN_GetTxFifoFreeLevel 返回剩余空闲空间数量
    while (HAL_FDCAN_GetTxFifoFreeLevel(motor->hfdcan) == 0)
    {
        // 可以添加看门狗喂狗或短暂延时，避免死循环看门狗复位
        // __NOP(); 
    }
}


// 更新电机状态
void QD4310_Update(QD4310_t *motor, const uint8_t feedback[8]) {
    motor->enabled = feedback[0] & 0x01;

    // 重构int16_t值从字节数组
    int16_t current_raw = (int16_t)((feedback[3] << 8) | feedback[2]);
    motor->current = (float)current_raw * 10.0f / INT16_MAX;

    int16_t speed_raw = (int16_t)((feedback[5] << 8) | feedback[4]);
    motor->speed = (float)speed_raw * 1000.0f / 32767.0f;

    uint16_t angle_raw = (uint16_t)((feedback[7] << 8) | feedback[6]);
    motor->angle = (float)angle_raw * QD4310_TWO_PI / UINT16_MAX;
}


// 使能电机
void QD4310_Enable(QD4310_t *motor) {
    QD4310_SendCommand(motor, QD4310_CMD_ENABLE, 0x0000);
}

// 失能电机
void QD4310_Disable(QD4310_t *motor) {
    QD4310_SendCommand(motor, QD4310_CMD_DISABLE, 0x0000);
}

// 设置电机角度
void QD4310_SetAngle(QD4310_t *motor, float angle) {
    angle = QD4310_Clamp(angle, 0.0f, QD4310_TWO_PI);
    int16_t angle_value = (int16_t)(angle / QD4310_TWO_PI * UINT16_MAX);
    QD4310_SendCommand(motor, QD4310_CMD_ANGLE, angle_value);
}

// 设置电机步进角度
void QD4310_SetStepAngle(QD4310_t *motor, float step_angle) {
    step_angle = QD4310_Clamp(step_angle, QD4310_MIN_STEPANGLE, QD4310_MAX_STEPANGLE);
    int16_t step_angle_value = (int16_t)(step_angle / QD4310_MAX_STEPANGLE * INT16_MAX);
    QD4310_SendCommand(motor, QD4310_CMD_STEP_ANGLE, step_angle_value);
}

// 设置电机转速
void QD4310_SetSpeed(QD4310_t *motor, float speed) {
    speed = QD4310_Clamp(speed, QD4310_MIN_SPEED, QD4310_MAX_SPEED);
    int16_t speed_value = (int16_t)(speed / QD4310_MAX_SPEED * INT16_MAX);
    QD4310_SendCommand(motor, QD4310_CMD_SPEED, speed_value);
}

// 设置电机低速
void QD4310_SetLowSpeed(QD4310_t *motor, float speed) {
    speed = QD4310_Clamp(speed, QD4310_MIN_SPEED, QD4310_MAX_SPEED);
    int16_t speed_value = (int16_t)(speed / QD4310_MAX_SPEED * INT16_MAX);
    QD4310_SendCommand(motor, QD4310_CMD_LOW_SPEED, speed_value);
}

// 设置电机电流
void QD4310_SetCurrent(QD4310_t *motor, float current) {
    current = QD4310_Clamp(current, QD4310_MIN_CURRENT, QD4310_MAX_CURRENT);
    int16_t current_value = (int16_t)(current / QD4310_MAX_CURRENT * INT16_MAX);
    QD4310_SendCommand(motor, QD4310_CMD_CURRENT, current_value);
}

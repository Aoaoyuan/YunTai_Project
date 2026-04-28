#include "icm42688.h"
#include "spi.h"
#include "cmsis_os2.h"
#include <math.h>
#include <string.h>
#define CS_PIN    GPIO_PIN_2
#define CS_PORT   GPIOD
#define SPI_HANDLE hspi3

// 灵敏度常量 (根据 datasheet)
// Accel 16g: 2048 LSB/g -> 1/2048 g/LSB
// Gyro 2000dps: 16.4 LSB/dps -> 1/16.4 dps/LSB
#define ACCEL_SENSITIVITY  (1.0f / 2048.0f)
#define GYRO_SENSITIVITY   (1.0f / 16.4f)

static void ICM_CSB(int level)
{
    HAL_GPIO_WritePin(CS_PORT, CS_PIN, level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

// 底层：写单个寄存器
static void ICM_WriteReg(uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = {reg & 0x7F, val}; // 写操作最高位为0
    ICM_CSB(0);
    HAL_SPI_Transmit(&SPI_HANDLE, tx, 2, 10);
    ICM_CSB(1);
    osDelay(1); // 某些寄存器写入后需要短暂延时
}

// 底层：读单个寄存器
static uint8_t ICM_ReadReg(uint8_t reg)
{
    uint8_t tx[2] = {reg | 0x80, 0xFF}; // 读操作最高位为1
    uint8_t rx[2] = {0};
    ICM_CSB(0);
    HAL_SPI_TransmitReceive(&SPI_HANDLE, tx, rx, 2, 10);
    ICM_CSB(1);
    return rx[1];
}

// 底层：连续读取多个寄存器 (关键优化)
// reg: 起始寄存器地址 (会自动设置读位)
// buf: 数据缓冲区
// len: 长度
static void ICM_ReadRegs(uint8_t reg, uint8_t *buf, uint16_t len)
{
    uint8_t tx_cmd = reg | 0x80; // 设置读位
    
    ICM_CSB(0);
    // 发送命令字节
    HAL_SPI_Transmit(&SPI_HANDLE, &tx_cmd, 1, 10);
    // 接收数据 (主机发送Dummy Byte 0x00 以产生时钟)
    memset(buf, 0x00, len); // 确保缓冲区初始化为0，防止残留
    HAL_SPI_Receive(&SPI_HANDLE, buf, len, 10);
    ICM_CSB(1);
}

void ICM42688_Init(void)
{
    ICM_CSB(1);
    osDelay(50);

    // 1. 软复位
    ICM_WriteReg(ICM42688_DEVICE_CONFIG, 0x01);
    osDelay(50);

    // 2. 唤醒传感器 (配置 PWR_MGMT0)
    // Bit[7:6]: Reserved
    // Bit[5]: TEMP_DIS (0=Enable)
    // Bit[4:2]: GYRO_MODE (011=Low Noise)
    // Bit[1:0]: ACCEL_MODE (011=Low Noise)
    ICM_WriteReg(ICM42688_PWR_MGMT0, 0x0F); 
    osDelay(10);

    // 3. 配置量程和ODR
    // ACCEL_CONFIG0: Bit[7:5] Range (011=16g), Bit[4:0] ODR (01111=1kHz UI output)
    ICM_WriteReg(ICM42688_ACCEL_CONFIG0, (ICM_ACCEL_16G << 5) | ICM_ODR_1KHZ);
    
    // GYRO_CONFIG0: Bit[7:5] Range (011=2000dps), Bit[4:0] ODR (01111=1kHz UI output)
    ICM_WriteReg(ICM42688_GYRO_CONFIG0, (ICM_GYRO_2000DPS << 5) | ICM_ODR_1KHZ);

    osDelay(50);
}

/**
 * @brief 一次性读取加速度和陀螺仪数据
 * @note 使用连续读取保证数据的时间同步性，避免抖动
 * @param ax, ay, az: 加速度指针 (单位: g)
 * @param gx, gy, gz: 陀螺仪指针 (单位: dps)
 * @retval 0: 成功, 1: 数据无效
 */
uint8_t ICM42688_Read_All(float *ax, float *ay, float *az, float *gx, float *gy, float *gz)
{
    uint8_t buffer[12];
    
    // 从 0x1F (ACCEL_DATA_X1) 开始连续读取 12 个字节
    // 顺序: Ax_H, Ax_L, Ay_H, Ay_L, Az_H, Az_L, Gx_H, Gx_L, Gy_H, Gy_L, Gz_H, Gz_L
    ICM_ReadRegs(ICM42688_ACCEL_DATA_X1, buffer, 12);

    // 组合数据并转换为有符号整数
    int16_t raw_ax = (int16_t)((buffer[0] << 8) | buffer[1]);
    int16_t raw_ay = (int16_t)((buffer[2] << 8) | buffer[3]);
    int16_t raw_az = (int16_t)((buffer[4] << 8) | buffer[5]);
    
    int16_t raw_gx = (int16_t)((buffer[6] << 8) | buffer[7]);
    int16_t raw_gy = (int16_t)((buffer[8] << 8) | buffer[9]);
    int16_t raw_gz = (int16_t)((buffer[10] << 8) | buffer[11]);

    // 检查数据是否无效 (全0或全FF通常表示传感器未就绪或错误)
    if (raw_ax == -1 && raw_ay == -1 && raw_az == -1) {
        return 1; 
    }

    // 转换为物理量
    *ax = (float)raw_ax * ACCEL_SENSITIVITY;
    *ay = (float)raw_ay * ACCEL_SENSITIVITY;
    *az = (float)raw_az * ACCEL_SENSITIVITY;

    *gx = (float)raw_gx * GYRO_SENSITIVITY;
    *gy = (float)raw_gy * GYRO_SENSITIVITY;
    *gz = (float)raw_gz * GYRO_SENSITIVITY;

    return 0;
}
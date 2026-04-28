#ifndef ICM42688_H
#define ICM42688_H

#include "main.h"

// 寄存器地址定义 (Bank 0)
#define ICM42688_REG_BANK_SEL       0x76
#define ICM42688_DEVICE_CONFIG      0x11
#define ICM42688_PWR_MGMT0          0x4E
#define ICM42688_GYRO_CONFIG0       0x4F
#define ICM42688_ACCEL_CONFIG0      0x50
#define ICM42688_TEMP_DATA1         0x1D
#define ICM42688_ACCEL_DATA_X1      0x1F
#define ICM42688_GYRO_DATA_X1       0x25
#define ICM42688_WHO_AM_I           0x75
#define ICM42688_ID                 0x47 // 或者 0x67, 取决于具体子型号，通常检查是否为非0即可

// 量程定义
#define ICM_ACCEL_16G               0x03
#define ICM_GYRO_2000DPS            0x03
#define ICM_GYRO_1000DPS            0x02
#define ICM_ODR_1KHZ                0x0F // 1kHz

void ICM42688_Init(void);
// 推荐一次性读取所有数据，保证时间同步
uint8_t ICM42688_Read_All(float *ax, float *ay, float *az, float *gx, float *gy, float *gz);

#endif /* ICM42688_H */
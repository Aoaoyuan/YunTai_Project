#include "niming.h"

/*
 * 为匿名协议封包发送3个浮点数（data1,data2,data3）
 * 打包格式：
 * 头(1) 源地址(1) 目标地址(1) 帧ID(1) 长度低字节(1) 长度高字节(1) 数据(...) 校验1(1) 校验2(1)
 */

#define BYTE0(dwTemp)       (*(unsigned char*)(&dwTemp))
#define BYTE1(dwTemp)       (*((unsigned char*)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((unsigned char*)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((unsigned char*)(&dwTemp) + 3))

uint8_t BUFF[20];

void Get_NMdata(float data1, float data2, float data3)
{
    int i;
    uint8_t sumcheck = 0;
    uint8_t addcheck = 0;
    uint8_t _cnt = 0;

    BUFF[_cnt++] = 0xAB;    // 帧头
    BUFF[_cnt++] = 0x01;    // 源地址
    BUFF[_cnt++] = 0xFF;    // 目标地址
    BUFF[_cnt++] = 0xF1;    // 帧ID
    BUFF[_cnt++] = 0x0C;    // 数据长度低字节 (3 float = 12 bytes)
    BUFF[_cnt++] = 0x00;    // 数据长度高字节

    BUFF[_cnt++] = BYTE0(data1);
    BUFF[_cnt++] = BYTE1(data1);
    BUFF[_cnt++] = BYTE2(data1);
    BUFF[_cnt++] = BYTE3(data1);

    BUFF[_cnt++] = BYTE0(data2);
    BUFF[_cnt++] = BYTE1(data2);
    BUFF[_cnt++] = BYTE2(data2);
    BUFF[_cnt++] = BYTE3(data2);

    BUFF[_cnt++] = BYTE0(data3);
    BUFF[_cnt++] = BYTE1(data3);
    BUFF[_cnt++] = BYTE2(data3);
    BUFF[_cnt++] = BYTE3(data3);

    // 计算校验（覆盖到数据长度+6字节范围）
    for (i = 0; i < (BUFF[4] + BUFF[5] * 256 + 6); i++) {
        sumcheck += BUFF[i];
        addcheck += sumcheck;
    }
    BUFF[_cnt++] = sumcheck;
    BUFF[_cnt++] = addcheck;
}

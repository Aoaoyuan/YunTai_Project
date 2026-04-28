#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "icm42688.h"
/************************************************
 ALIENTEK 探索者STM32F407开发板 实验25
 SPI实验-HAL库函数版
 技术支持：www.openedv.com
 淘宝店铺：http://eboard.taobao.com 
 关注微信公众平台微信号："正点原子"，免费获取STM32资料。
 广州市星翼电子科技有限公司  
 作者：正点原子 @ALIENTEK
************************************************/

// VCC--------5V或者3.3V都可以
// SPI 模式接线
// PA4 ------------------------CS
// PA5 ------------------------SCLK
// PA6------------------------MISO
// PA7------------------------MOSI

// IIC 接线
// PC1 ------------------------SCL
// PC2 ------------------------SDA

icm42688RealData_t accval;
icm42688RealData_t gyroval;
int main(void)
{
	
    HAL_Init();                   	//初始化HAL库    
    Stm32_Clock_Init(336,8,2,7);  	//设置时钟,168Mhz
	delay_init(168);               	//初始化延时函数
	uart_init(115200);             	//初始化USART
	bsp_Icm42688Init();
		

	while(1)
	{
	//读取加速度和陀螺仪的当前ADC
		bsp_IcmGetRawData(&accval, &gyroval);
		printf("%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\r\n", accval.x, accval.y, accval.z, gyroval.x, gyroval.y, gyroval.z);

		delay_ms(50);	   
	}	
}


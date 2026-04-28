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
// PA2------------------------INT1
// PA4------------------------CS
// PA5------------------------SCLK
// PA6------------------------MISO
// PA7------------------------MOSI

icm42688RawData_t accval;
icm42688RawData_t gyroval;
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

int main(void)
{
	
    HAL_Init();                   	//初始化HAL库    
    Stm32_Clock_Init(336,8,2,7);  	//设置时钟,168Mhz
	delay_init(168);               	//初始化延时函数
	uart_init(115200);             	//初始化USART
	bsp_Icm42688Init();
	MX_GPIO_Init();
		

	while(1)
	{
	//读取加速度和陀螺仪的当前ADC

		delay_ms(100);	   
	}	
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	bsp_IcmGetRawData(&accval, &gyroval);
	printf("%d\t%d\t%d\t%d\t%d\t%d\r\n", accval.x, accval.y, accval.z, gyroval.x, gyroval.y, gyroval.z);
}

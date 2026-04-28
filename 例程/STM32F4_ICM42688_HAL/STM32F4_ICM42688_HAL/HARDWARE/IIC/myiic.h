#ifndef _MYIIC_H
#define _MYIIC_H
#include "sys.h"
	
//IO方向设置
#define SDA_IN()  {GPIOC->MODER&=~(3<<(2*2));GPIOC->MODER|=0<<2*2;}	//PC2输入模式
#define SDA_OUT() {GPIOC->MODER&=~(3<<(2*2));GPIOC->MODER|=1<<2*2;} //PC2输出模式
//IO操作
#define IIC_SCL    PCout(1) //SCL
#define IIC_SDA    PCout(2) //SDA	 
#define READ_SDA   PCin(2)  //输入SDA 

//IIC所有操作函数
void IIC_Init(void);                //初始化IIC的IO口				 
void IIC_Start(void);				//发送IIC开始信号
void IIC_Stop(void);	  			//发送IIC停止信号
void IIC_Send_Byte(u8 txd);			//IIC发送一个字节
u8 IIC_Read_Byte(unsigned char ack);//IIC读取一个字节
u8 IIC_Wait_Ack(void); 				//IIC等待ACK信号
void IIC_Ack(void);					//IIC发送ACK信号
void IIC_NAck(void);				//IIC不发送ACK信号



u8 IIC_Write_1Byte(u8 SlaveAddress,u8 REG_Address,u8 REG_data);
u8 IIC_Read_1Byte(u8 SlaveAddress,u8 REG_Address);
u8 IIC_Write_nByte(u8 SlaveAddress, u8 REG_Address, u16 len, const u8 *buf);
u8 IIC_Read_nByte(u8 SlaveAddress, u8 REG_Address, u16 len, u8 *buf);

#endif


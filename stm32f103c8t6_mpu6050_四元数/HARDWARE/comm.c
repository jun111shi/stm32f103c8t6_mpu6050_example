#include "comm.h"
#include "alldata.h"
#include "usart.h"

/* ANOTC匿名站通讯机制（串口版本） */
void Send_ANOTC()
{
	uint8_t i;
	uint8_t len=0;							
	int16_t Anto[12];
	int8_t *pt = (int8_t*)(Anto);		
	
	Anto[2] = (int16_t)(Angle.roll*100);
	Anto[3] = (int16_t)(Angle.pitch*100);
	Anto[4] = -(int16_t)(Angle.yaw*100);
	Anto[5] = 0;//没有高度数据
	Anto[6] = 0;
	Anto[7] = ALL_flag.unlock<<8;//解锁信息
	len = 12;						//数据长度
	
	Anto[0] = 0XAAAA;
	Anto[1] = len | 0x01<<8;
	pt[len+4] = (int8_t)(0xAA+0xAA);
	
	pt[len+4] = (int8_t)(0xAA+0xAA);
	for(i=2;i<len+4;i+=2)    //a swap with b;
	{
		pt[i] ^= pt[i+1];
		pt[i+1] ^= pt[i];
		pt[i] ^= pt[i+1];
		pt[len+4] += pt[i] + pt[i+1];
	}
	
//	USART1_SendByte(pt,len+5);//采用串口发送到匿名上位机
//	HAL_UART_Transmit(&huart2,(uint8_t *)pt,len+5,0xFFFF);
}

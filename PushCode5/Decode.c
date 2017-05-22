
#include "nomal.h"
#include "config.h"



#if CONFIG_433SG

#include "soft.h"
#include "main.h"
#include "Hd_ElecPush5.H"
#include "Decode.h"



DatTypeX	DecodeLastTime;	//上次脉冲时刻
unsigned char DecodeTimeout;	//脉冲超时定时器
unsigned char FlagDecodeStart;	//前导已接收
unsigned char DecodeStartCount;	//前导脉冲接收计数器
unsigned char DecodeBitCount;	//位计数器
unsigned char DecodeByteCount;	//字节计数器
unsigned char DecodeByte;
unsigned char DecodeString[MAX_DECODE_BUF_LEN+10];
unsigned char DecodeCompleteFlag;	//指令接收完成

//unsigned char DecodeActBitCount;	//合法脉冲计数器
//unsigned char DecodeLastProcessCount;	//上次合法脉冲比较计数器


void InitDecode(void);
void _GetPulseProcess(unsigned char flag);	//接收到脉冲处理
void DecodePulse(DatTypeX cap_time);

////
void InitDecode(void)
{
unsigned char i;
	DecodeLastTime=0;	//上次脉冲时刻
	DecodeTimeout=0;	//脉冲超时定时器

	DecodeByte=0;
	
	DecodeBitCount=0;	//位计数器
	DecodeByteCount=0;	//字节计数器
	FlagDecodeStart=0;	//前导以接收
	DecodeStartCount=0;	//前导脉冲接收计数器
	DecodeCompleteFlag=0;
	
	for(i=0;i<MAX_DECODE_BUF_LEN;i++)
		{
			DecodeString[i]=0;
		}
}
//////
void _GetPulseProcess(unsigned char flag)	//接收到脉冲处理
{
	if(flag==1)	//接收到脉冲1
		{
		if(DecodeStartCount<100)DecodeStartCount++;	//前导脉冲接收计数器
		if(FlagDecodeStart)	//前导已接收
			{
			DecodeByte<<=1;	
			DecodeByte|=1;	//移入1
			DecodeBitCount++;	//位计数器
			if(DecodeBitCount>=8)
				{
				if(DecodeByteCount<MAX_DECODE_BUF_LEN)
					{	
					DecodeString[DecodeByteCount]=DecodeByte;	
					DecodeByteCount++;
					
					if(DecodeByteCount>=LENGTH_REMOT_PARK)
						{//指令接收完成标记
						DecodeString[DecodeByteCount]=0x0;
						DecodeCompleteFlag=1;
						DecodeByteCount=0;
						DecodeTimeout=0;
						//TestProc_LED3_OFF;	//完成
						}
					}
				DecodeBitCount=0;	
				}	
				
			}
		}
	else{	//接收到脉冲0

		if(DecodeStartCount>10)	//16
			{
			if(FlagDecodeStart==0)	//前导未接收
				{
				DecodeBitCount=0;	//位计数器
				DecodeByteCount=0;	//字节计数器	
				FlagDecodeStart=1;
				}	
			//TestProc_LED3_ON;
			}
			
		if(FlagDecodeStart)	//前导已接收
			{
			DecodeByte<<=1;	
			DecodeBitCount++;	//位计数器
			if(DecodeBitCount>=8)
				{
					
				if(DecodeByteCount<MAX_DECODE_BUF_LEN)
					{	
					DecodeString[DecodeByteCount]=DecodeByte;	
					DecodeByteCount++;
					
					
					
					if(DecodeByteCount>=LENGTH_REMOT_PARK)
						{//指令接收完成标记
						
						DecodeString[DecodeByteCount]=0x0;
						DecodeCompleteFlag=1;
						DecodeByteCount=0;
						DecodeTimeout=0;
						//TestProc_LED3_OFF;	//完成
						}
					}
				DecodeBitCount=0;	
				}
			}	
		DecodeStartCount=0;	//前导脉冲接收计数器
		}	
}

//////////////////////////
void DecodePulse(DatTypeX cap_time)
{
//遥控器接收中断  timer0 int1
//unsigned int temp2;	//周期
DatTypeX temp2;	//周期
	if(DecodeTimeout<100)
		{
		temp2=cap_time-DecodeLastTime;	//周期

		if(temp2>DECODE_TIME1)
			{
			if(temp2<DECODE_TIME2)
				{//DECODE_TIME1~DECODE_TIME2  的码
					//FlagDecodeStart=1;	//前导以接收
				//	DecodeActBitCount++;
					if(DecodeCompleteFlag==0)_GetPulseProcess(1);
					TestProc_LED2_ON;	//收到0
					Time_TestProc_LED2=50;
				}
			else if	(temp2>DECODE_TIME3)
				{
				if(temp2<DECODE_TIME4)
					{//DECODE_TIME3~DECODE_TIME4  的码
						//FlagDecodeStart=1;	//前导以接收
//						DecodeActBitCount++;
						if(DecodeCompleteFlag==0)_GetPulseProcess(0);
						TestProc_LED1_ON;
						Time_TestProc_LED1=50;
					}
				else{
					//错误，重新开始
					
					DecodeBitCount=0;
					DecodeByteCount=0;
					FlagDecodeStart=0;	//前导以接收
					DecodeStartCount=0;	//前导脉冲接收计数器
					//DecodeCompleteFlag=0;
					/* */
					}	
				}
			else{
				//错误，重新开始
				DecodeBitCount=0;
				DecodeByteCount=0;
				FlagDecodeStart=0;	//前导以接收
				DecodeStartCount=0;	//前导脉冲接收计数器
				//DecodeCompleteFlag=0;
				
				}
			}
		else{/*
				//错误，重新开始
				DecodeBitCount=0;
				DecodeByteCount=0;
				FlagDecodeStart=0;	//前导以接收
				DecodeStartCount=0;	//前导脉冲接收计数器
				//DecodeCompleteFlag=0;
				
				*/
				}
		//高位在前	

		}
	else{//超时，重新开始
		DecodeBitCount=0;
		DecodeByteCount=0;
		FlagDecodeStart=0;	//前导以接收
		DecodeStartCount=0;	//前导脉冲接收计数器
		//DecodeCompleteFlag=0;
		}
DecodeLastTime=cap_time;	//上次脉冲时刻
DecodeTimeout=0;

}

#endif
#ifndef _DECODE_H
#define _DECODE_H

#define MAX_DECODE_BUF_LEN	6
#define LENGTH_REMOT_PARK	4





#define TestProc_LED1_ON	;//SetLed1
#define TestProc_LED1_OFF	;//ClrLed1
#define TestProc_LED2_ON	;//SetLed2
#define TestProc_LED2_OFF	;//ClrLed2

#define LED_RECV_ON		;//SetLed2
#define LED_RECV_OFF	;//ClrLed2


#define DatTypeX	unsigned char
//#define DatTypeX	unsigned int 

//#define Fosc  8000000
//#define Fosc	  (8000000/2)

	//改进的高速编码
	#define DECODE_TIME1	(Fosc*2/10000/256)
	#define DECODE_TIME2	(Fosc*5/10000/256)
	#define DECODE_TIME3	(Fosc*6/10000/256)
	#define DECODE_TIME4	(Fosc*12/10000/256)


extern DatTypeX DecodeLastTime;	//上次脉冲时刻
extern unsigned char DecodeTimeout;	//脉冲超时定时器
extern unsigned char FlagDecodeStart;	//前导已接收
extern unsigned char DecodeStartCount;	//前导脉冲接收计数器
extern unsigned char DecodeBitCount;	//位计数器
extern unsigned char DecodeByteCount;	//字节计数器
extern unsigned char DecodeByte;
extern unsigned char DecodeString[];
extern unsigned char DecodeCompleteFlag;	//指令接收完成

//extern unsigned char DecodeActBitCount;	//合法脉冲计数器
//extern unsigned char DecodeLastProcessCount;	//上次合法脉冲比较计数器

void InitDecode(void);
void _GetPulseProcess(unsigned char flag);	//接收到脉冲处理
void DecodePulse(DatTypeX cap_time);

#endif


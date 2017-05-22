#ifndef _SOFT_H
#define _SOFT_H


/**************** 选择软件版本号*************/
/*  软件版本号  */
//#define Soft_VER  0x05
//#define FIRST_PASS 0x20



#define DefaultUserBaud			BAUD9600
#define UserBaud 						BAUD9600

extern uchar  InputBuf;
extern uchar  KeybyteBuf;
extern uchar FlagSetCurrent1;

/* 数据类型 */
typedef unsigned char        BOOLEN;
typedef unsigned char        INT8U;
typedef signed char          INT8S;
typedef unsigned short       INT16U;
typedef signed short         INT16S;
typedef unsigned int         INT32U;
typedef signed int           INT32S;
typedef float                FP32;
typedef double               FP64;


#define BYTE                 INT8S
#define UBYTE                INT8U    
#define WORD                 INT16S
#define UWORD                INT16U
#define LONG                 INT32S
#define ULONG                INT32U

//点动
#define COMMAND_C1	0X11		//按一下第一路正转，任何一路转时按则停止
#define COMMAND_CC1	0X21		//按一下第一路反转，任何一路转时按则停止
#define COMMAND_C2	0X12		//按一下第二路正转，任何一路转时按则停止
#define COMMAND_CC2	0X22		//按一下第二路反转，任何一路转时按则停止
#define COMMAND_C3	0X13
#define COMMAND_CC3	0X23

//自动命令
#define COMMAND_A1	0X31			//按一下第一路正转
#define COMMAND_AA1	0X41			//按一下第一路反转
#define COMMAND_A2	0X51			//按一下第二路正转
#define COMMAND_AA2	0X61			//按一下第二路反转	
#define COMMAND_A3	0X71			//按一下第三路正转
#define COMMAND_AA3	0X81			//按一下第三路反转		


/**************************************************************************************************/
extern char bTimeBase;
#define Setb_bTimeBase	bTimeBase=1;
#define Clr_bTimeBase	bTimeBase=0;

extern uchar FlagRuning;
#define LED_RUN_ON			SetLed1
#define LED_RUN_OFF			ClrLed1
//#define LED_RECV_ON		;//SetLed2
//#define LED_RECV_OFF	;//ClrLed2



#if CONFIG_WIRELESS_DECODE
			//LED自动熄灭
			extern unsigned char Time_TestProc_LED1;
			extern unsigned char Time_TestProc_LED2;
			extern unsigned char Time_LedRecv_LED;
			extern uchar LastCommand;	//上次命令
			extern uint SetIdTime;			//设置ID时间
			extern uint PulseCount;
			extern uchar SamCommandTime;	//重复的命令时间
#endif




extern uchar bExCurrentForwardMax;
extern uchar bExCurrentBackwardMax;

struct	struct_save
{

		uchar bCurrentForward;			//工作电流1(uint=0.2A)
		uchar bCurrentBackward;			//工作电流2(uint=0.2A)
		uchar bCurrentRate;			//电流测量比例200-206-220(uint=0.2A)		
		
		#if CONFIG_WIRELESS_DECODE
		uchar RemotName[2];
		uchar xxx[2];
		#endif
		
		
		uchar flag;				//参数已经初始化标记
};

extern struct	struct_save *gpParam;
#define Max_Param_Len		8
//参数已经初始化标记
#define FlagParamInitnized	0xA7
extern uchar gbParamBuf[Max_Param_Len+2];

//参数的EEPROM区首地址
#define EEPROM_BASE_ADR	0X20



#if CONFIG_UART
void ParamSend(void);
void AutoSend(void);
void SendText_UART0(INT8U *StrData);
	#if CONFIG_WIRELESS_DECODE
			void RemotCodeSend(uchar *command1,uchar comlen1);
	#endif
#endif
void CheckKey(void);		//周期10ms
void CheckInput(void);
void Work(void);
void Default_ParamInit(void);

#if CONFIG_SAVE5
void Load_Param(void);
void Write_Param(void);
#endif





#endif











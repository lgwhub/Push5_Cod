#ifndef _SOFT_H
#define _SOFT_H


/**************** ѡ������汾��*************/
/*  ����汾��  */
//#define Soft_VER  0x05
//#define FIRST_PASS 0x20



#define DefaultUserBaud			BAUD9600
#define UserBaud 						BAUD9600

extern uchar  InputBuf;
extern uchar  KeybyteBuf;
extern uchar FlagSetCurrent1;

/* �������� */
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

//�㶯
#define COMMAND_C1	0X11		//��һ�µ�һ·��ת���κ�һ·תʱ����ֹͣ
#define COMMAND_CC1	0X21		//��һ�µ�һ·��ת���κ�һ·תʱ����ֹͣ
#define COMMAND_C2	0X12		//��һ�µڶ�·��ת���κ�һ·תʱ����ֹͣ
#define COMMAND_CC2	0X22		//��һ�µڶ�·��ת���κ�һ·תʱ����ֹͣ
#define COMMAND_C3	0X13
#define COMMAND_CC3	0X23

//�Զ�����
#define COMMAND_A1	0X31			//��һ�µ�һ·��ת
#define COMMAND_AA1	0X41			//��һ�µ�һ·��ת
#define COMMAND_A2	0X51			//��һ�µڶ�·��ת
#define COMMAND_AA2	0X61			//��һ�µڶ�·��ת	
#define COMMAND_A3	0X71			//��һ�µ���·��ת
#define COMMAND_AA3	0X81			//��һ�µ���·��ת		

#define REMOT_COMMAND_MOT1_MOT2_CW	0X34
#define REMOT_COMMAND_MOT1_MOT2_CCW	0X44


//�綯����
#define REMOT_COMMAND_MOT3_CW		0X51
#define REMOT_COMMAND_MOT3_CCW		0X55
//#define REMOT_COMMAND_MOT3_CW		REMOT_COMMAND_MOT1_MOT2_CW
//#define REMOT_COMMAND_MOT3_CCW		REMOT_COMMAND_MOT1_MOT2_CCW

#define REMOT_COMMAND_MOT4_CW		0X52
#define REMOT_COMMAND_MOT4_CCW		0X56
#define REMOT_COMMAND_MOT3_MOT4_CW		0X53
#define REMOT_COMMAND_MOT3_MOT4_CCW		0X57
////////////////
#define REMOT_COMMAND_POWER_ON		0X39
#define REMOT_COMMAND_POWER_OFF		0X49

#define RESPONES_RUNING			0X68
#define RESPONES_COMMPLETE		0X6A
//�綯��������
#define REMOT_COMMAND_SHIFT_SET_ID		0X6C
//�����綯��������
#define REMOT_COMMAND_SET_RATE3		0X6D
//�綯�����ܿ��ƿ�
#define REMOT_COMMAND_PUSH_POWER_ON		0X6F
/**************************************************************************************************/
extern char bTimeBase;
#define Setb_bTimeBase	bTimeBase=1;
#define Clr_bTimeBase	bTimeBase=0;

#define LED_RUN_ON			SetLed1
#define LED_RUN_OFF			ClrLed1
#define LED_RECV_ON			SetLed2
#define LED_RECV_OFF		ClrLed2


#define LED_Al_Box		;//SetLed2
#define LED_Fe_Box	;//ClrLed2

extern unsigned char Time_LedRecv_LED;

#if CONFIG_433SG
			//LED�Զ�Ϩ��
			extern unsigned char Time_TestProc_LED1;
			extern unsigned char Time_TestProc_LED2;
			
			extern uint PulseCount;
#endif			
			extern uchar LastCommand;	//�ϴ�����
			
//			extern uchar SamCommandTime;	//�ظ�������ʱ��	
			

 			
			extern uchar SetIdTime;			//����IDʱ��





extern uchar bExCurrentForwardMax;
extern uchar bExCurrentBackwardMax;

struct	struct_save
{

		uchar bCurrentForward;			//��������1(uint=0.2A)
		uchar bCurrentBackward;			//��������2(uint=0.2A)
		uchar bCurrentRate;			//������������200-206-220(uint=0.2A)		
		
		#if CONFIG_433SG 
				uchar RemotName[2];
				uchar xxx[2];
		#elif CONFIG_CC1100
				uchar RemotName[3];
				uchar xxx[2];
		#endif
		
		
		uchar flag;				//�����Ѿ���ʼ�����
};

extern struct	struct_save *gpParam;
//#define Max_Param_Len		8
#define Max_Param_Len		10
//�����Ѿ���ʼ�����
#define FlagParamInitnized	0xA9
extern uchar gbParamBuf[Max_Param_Len+2];

//������EEPROM���׵�ַ
#define EEPROM_BASE_ADR	0X10


void SendToRemot(uchar command); //������ң����



#if CONFIG_UART
void ParamSend(void);
void AutoSend(void);
void SendText_UART0(INT8U *StrData);
	#if CONFIG_433SG
			void RemotCodeSend(uchar *command1,uchar comlen1);
	#endif
#endif
void CheckKey(void);		//����10ms
void CheckInput(void);
void Work(void);
void Default_ParamInit(void);

#if CONFIG_SAVE5
void Load_Param(void);
void Write_Param(void);
#endif





#endif











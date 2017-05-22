
#include "nomal.h"
#include "config.h"
#include "soft.h"
#include "main.h"
#include "Hd_ElecPush3.H"
#include "Mega48_Adc.h"

#include "CC1100.h"

#if CONFIG_433SG
  #include "Decode.h"
#endif

	#if IAR_SYSTEM
		#include "iom88v.h"
        	#include "IAR_VECT.h"
	#else
		#include <iom88v.h>
	#endif
#if CONFIG_UART
	#include "LoopBuf.h"
#endif

/**************************************************************************************************/
uchar bTimeBase;

ReInitCC1100Time=0;
unsigned char Time_LedRecv_LED;

#if CONFIG_433SG
		//LED自动熄灭
		unsigned char Time_TestProc_LED1;
		unsigned char Time_TestProc_LED2;
		
		uint PulseCount=0;
		
#endif		
//		uchar SamCommandTime;	//重复的命令时间
		uchar LastCommand;	//上次命令

		


		uchar SetIdTime;			//设置ID时间


uchar FlagSetCurrent1=0;
uchar CommandLostTimeOut;	//没有收到命令的时间,超时清除命令和连续键的时间


uchar FlagAuto;	//自动或点动方式标记
uchar DropOffTime;//死区时间
#if 0
uchar TestStepCount;
uchar FlagTestStep;
#endif


struct	struct_save *gpParam;
uchar gbParamBuf[Max_Param_Len+2];

uchar FlagInputZero=0;
uchar InputBuf=0x0f;

uchar bExCurrentForwardMax;
uchar bExCurrentForwardPer85;
//333	uchar bExCurrentForwardPer94;

uchar bExCurrentBackwardMax;
uchar bExCurrentBackwardPer85;
//333	uchar bExCurrentBackwardPer94;

uchar KeybyteOld_Buf;
uchar KeybyteBuf;
uchar KeybyteOld;
uchar KeybyteCur;
////
struct _motor_struct
{
//第一路正转向前信号标记
//第二路反转向后信号标记
//第三路反转向后信号标记

uchar CommandType	;//方向,转向
uchar CurrentType	;//当前对方向,转向
uchar OverType1	;//过流的方向,转向
//uchar OverType2	;//过流的方向,转向
//uchar OverType3	;//过流的方向,转向
uchar FlagPower;
uchar FlagRuning;
uchar Pwm1;		//第一路正转向前的脉冲宽度
uchar timer_sec100;
//uchar RunTime;	//运行时间定时器,单位S
uint16	RunTime;	//运行时间定时器,单位0.1S
uchar unPushTime;
//333	uchar TimePer94;		//94%最大电流的运行时间定时器,单位10mS
uint16 iTimePer85;		//75%最大电流的运行时间定时器,单位10mS
};
struct _motor_struct  Motor;
///

//void ProcessTestStep(void);

void SendToRemot(uchar command) //反馈到遥控器
{
#if CONFIG_CC1100
uchar buf[36+14];
uchar i,len;	

//static uchar id;
//id++;
//buf[0]=0x31+(0x07&id);	//1~8
buf[0]=0x31;	//1~8
buf[1]=gpParam->RemotName[0];
buf[2]=gpParam->RemotName[1];
buf[3]=gpParam->RemotName[2];
buf[4]=command;
buf[5]='3';
buf[6]='3';
buf[7]=0x77;
for(i=8;i<30;i++)
{
buf[i]='4';
}
//		if(1)	//if(WirelessDebugTime>0)
//						{
//						len=8;
//						len+=MakeValAsc8("%A,T",Motor.ActiveTime,",",&buf[len]);
//						len+=MakeValAsc8("L",chCurrent1,",",&buf[len]);
//						len+=MakeValAsc8("R",chCurrent2,",",&buf[len]);
//						//场强
//						len+=MakeValAsc8("E",VolRssi,",",&buf[len]);
//						len		+=	PutString("*****",&buf[len],5);
//						}

buf[30]=0x0d;
buf[31]=0x0a;
buf[32]=0x00;
		
								cc1100Printf(buf);


		#endif	
}



#if CONFIG_UART
void SendText_UART0(INT8U *StrData)
{
	uchar i;
	for(i=0;i<255;i++)
	  {
	  if(*(StrData+i)==0x00)break;
	   Uart0CharSend(*(StrData+i));
	  }	
	
}
#endif
///////

void Default_ParamInit(void)
{	//gpParam参数结构的指针

	//gpParam->bCurrentForward=90;			//电机正转的 18A   max=256*75%=192
	//gpParam->bCurrentBackward=80;			//电机反转的 16A
	//gpParam->bCurrentForward=124;			//电机正转的 18A   max=256*75%=192
	//gpParam->bCurrentBackward=110;			//电机反转的 16A
	gpParam->bCurrentForward=CURRENT_FORWARD;			//电机正转的 18A   max=256*75%=192
	gpParam->bCurrentBackward=CURRENT_BACKWARD;			//电机反转的 16A
	//	gpParam->bCurrentBackward=3;			//电机反转的 0.6A
	gpParam->bCurrentRate=CURRENT_RATE;			//电机3的 5A
	
	

	
#if CONFIG_CC1100
	gpParam->RemotName[0]=0x35;
	gpParam->RemotName[1]=0x35;
	gpParam->RemotName[2]=0x36;	
	//gpParam->xxx[0]=0x0;
	//gpParam->xxx[1]=0x0;
#elif CONFIG_433SG
	gpParam->RemotName[0]=0x0;
	gpParam->RemotName[1]=0x0;
	//gpParam->xxx[0]=0x0;
	//gpParam->xxx[1]=0x0;
#endif	
	gpParam->flag=FlagParamInitnized;		//参数已经初始化标记
	
	/////////
	
	CommandLostTimeOut=0;	//没有收到命令的时间,超时清除命令和连续键的时间


	
	FlagAuto=0;	//自动或点动方式标记
	Motor.FlagRuning=0;
	Motor.FlagPower=0;		//无线电电源打开标记
//333		Motor.TimePer94=0;
	Motor.iTimePer85=0;
	Motor.timer_sec100=0;
	Motor.RunTime=0;	//60s自动关掉
	Motor.unPushTime=0;
	Motor.CommandType=0;
	Motor.CurrentType=0	;		//当前对方向,转向
	Motor.OverType1	=0;		//过流的方向,转向
	//Motor.OverType2	=0;		//过流的方向,转向
	//Motor.OverType3	=0;		//过流的方向,转向
	
	Motor.Pwm1=0;		//第一路正转向前的脉冲宽度

	DropOffTime=0;//死区时间
	

	#if 0
	TestStepCount=0;
	FlagTestStep=0;
	#endif
	
//		#if CONFIG_433SG ||CONFIG_CC1100
//		SamCommandTime=0;	//连续键的时间
//		#endif	
		LastCommand=0;	//上次命令

	
	SetIdTime=70;  //7S
	
	
}
	
///////////

//////////////////////////////////////////////////

/////////////
void AllJspOff(void)
{
JspOut1_OFF;
JspOut2_OFF;	

}
/////
#if 0
void ProcessTestStep(void)
{
	static uchar tim6;
	if(FlagTestStep==0)return;
		if(tim6<100)	//1s
			{
				tim6++;
			}
	else{
			tim6=0;
			if(TestStepCount<8)
					{
						TestStepCount++;
					}
			else{
				FlagTestStep=0;
					}
			
			switch(TestStepCount)
					{
						case 1:
						Motor.CommandType=COMMAND_C1;		//按住第一路正转
						FlagAuto=1;	//自动或点动方式标记
						break;
						
						case 4:
						FlagAuto=0;	//自动或点动方式标记
						break;
						
						case 5:
						Motor.CommandType=COMMAND_CC1;		//按住第一路反转
						FlagAuto=1;	//自动或点动方式标记
						break;
						
						case 8:
						FlagAuto=0;	//自动或点动方式标记
						break;	
						

						
						
						default:
						break;
			
					}
			}
}
#endif
/////
void ProcessJspSwitch(void)		//旋转方向切换 极性
{
		//if(Motor.Pwm1==0)
			if(DropOffTime>120)//死区时间
						{//反转已经停止
						switch(Motor.CommandType)	//
								{
								case COMMAND_C1:		//按住第一路正转
								case COMMAND_A1:			//按一下第一路正转
							//	if(Motor.OverType1!=1)		//过流的方向,转向
										{
											Motor.OverType1=0;
											JspOut1_ON;
											JspOut2_OFF;
											Motor.FlagRuning=1;
											Motor.CurrentType=Motor.CommandType;		//当前对方向,转向
											Motor.CommandType=0;	//执行后清除
										}
								break;
								
								case COMMAND_CC1:		//按住第一路反转
								case COMMAND_AA1:			//按一下第一路反转
							//		if(Motor.OverType1!=2)		//过流的方向,转向
										{
											Motor.OverType1=0;
											JspOut1_OFF;
											JspOut2_ON;
											Motor.FlagRuning=2;
											Motor.CurrentType=Motor.CommandType;		//当前对方向,转向
											Motor.CommandType=0;	//执行后清除
										}
								break;								

			
								
								default:
								
								break;
								
							}
							
							
						}
	
}
/////

/////////
#if CONFIG_433SG || CONFIG_CC1100
void ProcessCommand(uchar command)
{
	
	switch(command)
		{	//点动操作键
			case COMMAND_C1:		//按住第一路正转
			case COMMAND_CC1:		//按住第一路反转
			case COMMAND_C2:
			case COMMAND_CC2:
			case COMMAND_C3:
			case COMMAND_CC3:
				
				//333	Motor.TimePer94=0;
				Motor.iTimePer85=0;
				Motor.timer_sec100=0;
				Motor.RunTime=0;	//60s自动关掉
				Motor.unPushTime=0;
				
				#if CONFIG_FUNCTION_AUTO
						
//				//长按成为自动功能，放开继续，再按放开成为手动功能
//				if(SamCommandTime>150)
//					{
//					FlagAuto=1;//非点动方式
//					}
//				else FlagAuto=0;//点动方式
					
				FlagAuto=1;//非点动方式
				
				
				#else	
					FlagAuto=0;//点动方式	
				#endif
			break;

			
			default:
				
			break;
		}
}
#endif
///////
#if CONFIG_CC1100

void ProcessRemotCommand(uchar *p)		//for CC1100
{

uchar i;

uchar *nm;
uchar cmd;


/*
			buf[4]=	command;	
			buf[5]=	'6';	
			buf[6]=	'6';
			buf[7]=	0x70;
	*/
		if(*p<0x40)return;		//包号，1字节, 遥控器发送为"4xH"
		if((*(p+7)!=0x70)&&(*(p+7)!=0x71))return;
			
		nm=p+1;
		cmd=*(p+4);
		//if(('6'==*(p+5))&&('6'==*(p+6))&&(*(p+7)==0x70))
//		if(*(p+7)==0x70)	
//			{//老式遥控器兼容
//		}

	if(SetIdTime>0)
		{
	

			//setid
			if(cmd==REMOT_COMMAND_SHIFT_SET_ID)//REMOT_COMMAND_POWER_ON)
					{
					gpParam->RemotName[0]=*(nm);
					gpParam->RemotName[1]=*(nm+1);
					gpParam->RemotName[2]=*(nm+2);


				#if CONFIG_SAVE5

						Write_Param();				
				#endif
				Motor.FlagPower=90; //3分钟
				SetIdTime=0;
		
				}
		}	
	else{	
			//非对码时间
			if((CompareCharChar(nm,"555",3)!=0)||(CompareCharChar(nm,&gpParam->RemotName[0],3)!=0))	//默认地址
				{
			//遥控器序号ok

			
					//	SendUartCommand(cmd);
					if(LastCommand!=cmd)
								{
								switch(cmd)
														{
															case REMOT_COMMAND_MOT3_CW:
																if(Motor.FlagPower>0)
																			{
																			Motor.FlagPower=90; //3分钟
																			Motor.FlagRuning=0;
																			Motor.CommandType=COMMAND_C1;		//按第一路正转
																			FlagAuto=1;//非点动方式
																			ProcessCommand(Motor.CommandType);		//for CC1100
																			}
															break;
															case REMOT_COMMAND_MOT3_CCW:
																if(Motor.FlagPower>0)
																			{
																			Motor.FlagPower=90; //3分钟	
																			Motor.FlagRuning=0;
																			Motor.CommandType=COMMAND_CC1;		//按第一路反转
																			FlagAuto=1;//非点动方式
																			ProcessCommand(Motor.CommandType);		//for CC1100
																			}
															break;								
															case REMOT_COMMAND_POWER_ON:
																//Motor.FlagPower=240; //8分钟
																Motor.FlagPower=90; //3分钟	
															break;
															case REMOT_COMMAND_POWER_OFF:
																Motor.FlagPower=0;
																Motor.FlagRuning=0;
																Motor.CommandType=0;
																Motor.CurrentType=0;
																			
															break;
															case REMOT_COMMAND_SET_RATE3:	//设置电流
																Motor.FlagPower=90; //4分钟
																
															if(	FlagSetCurrent1==0)
																		{
															if((Motor.FlagRuning)&&(InputBuf==0x0f)&&(FlagInputZero))
																							{
																								FlagSetCurrent1=1;
																							}
																			}
															break;
															
															//控制器在操作
															case REMOT_COMMAND_MOT1_CW:
															case REMOT_COMMAND_MOT1_CCW:	
															case REMOT_COMMAND_MOT2_CW:
															case REMOT_COMMAND_MOT2_CCW:
															case REMOT_COMMAND_MOT1_CW_MOT2_CCW:
															case REMOT_COMMAND_MOT2_CW_MOT1_CCW:
															case REMOT_COMMAND_MOT1_MOT2_CW:
															case REMOT_COMMAND_MOT1_MOT2_CCW:

															//遥控器电源开着
															case REMOT_COMMAND_REMOT_HAND_ON:
															
															Motor.FlagPower=90; //4分钟
															
															break;
															
															
														}
								}
				LastCommand=cmd;								
				}
		}

	
}
///////////////////

void ProcessRfRecv(void)
{
	if(cc1100Scanf(rx_buf))
									{
									//RecvAny_LED1_ON;
									ReInitCC1100Time=0;
									
									LED_RECV_ON;
									Time_LedRecv_LED=20;
				
									ProcessRemotCommand(rx_buf);		//for CC1100
//									ResponseTime=0;
//									#if CONFIG_TEST_COMMAND_ENDCODE
//									for(i=0;i<CC1100_PKT_LEN;i++)
//							        	{
//							        	Uart0CharSend(rx_buf[i]);	
//							        	}
//									#endif
				        	}
}

#elif CONFIG_433SG
void ProcessRfRecv(void)
{
	uchar i;
	if(DecodeCompleteFlag==0)return;	//指令接收完成
	
	/*
	sum=0;
	for(i=0;i<4;i++)
			{
				sum^=DecodeString[i];
			}
	if(sum==0)
		*/
	if(DecodeString[2]==DecodeString[3])	
		{
		LED_RECV_ON;
		Time_LedRecv_LED=20;
				
				if((SetIdTime>1)||((DecodeString[0]==gpParam->RemotName[0])&&(DecodeString[1]==gpParam->RemotName[1])))
					{//对码时间
					CommandLostTimeOut=0;	//没有收到命令的时间
					if(LastCommand!=DecodeString[2])	//新指令
							{//判断时间
					//			SamCommandTime=0;
								LastCommand=DecodeString[2];
								Motor.CommandType=DecodeString[2];
								Motor.FlagRuning=0;
							}
			
						
					ProcessCommand(DecodeString[2]);
					if(SetIdTime>1)
											{
												//if(DecodeString[2]==0x50)
													gpParam->RemotName[0]=DecodeString[0];
													gpParam->RemotName[1]=DecodeString[1];

													#if CONFIG_SAVE5	
													Write_Param();
													#endif
												SetIdTime=0;
											}
					}
					
		}
	#if CONFIG_UART
	RemotCodeSend(DecodeString,LENGTH_REMOT_PARK);	
	#endif
	DecodeCompleteFlag=0;	//允许再次接收指令
}
#endif
///////

void CheckInput(void)
{
static uchar old;
uchar temp;									

PORTB|=0xf0;
temp=(0XF0&PINB);		//结果
		if(old==temp)
					{
						//InputBuf=temp>>4;
						InputBuf=(~temp)>>4;	//取反
						if(InputBuf==0)FlagInputZero=1;
					}
		else{ 	
				old=temp;	
				}
				
	//			InputBuf=0~15;   //18-30A
				
		/*
		Max=(18+0.8*InputBuf)
		Max=18*(1+InputBuf/15*2/3)
		Max=18*(1+InputBuf*2/45)=18*(45+2*InputBuf)/45=18+18*2*InputBuf/45		
		
		InputBuf=0  18A
		InputBuf=1  18.8A
		InputBuf=2  19.6A		
		
		
		InputBuf=13  28.4A
		InputBuf=14  29.2A
		InputBuf=15  30A			
		*/
		if(Motor.FlagRuning==0)
			{
			//正转	(1+adj*2/45)*100%   adj=0-15
			bExCurrentForwardMax=(gpParam->bCurrentForward<<1)*InputBuf/30+gpParam->bCurrentForward;

			bExCurrentForwardPer85=(uchar)((uint16)(bExCurrentForwardMax)*7>>3);
			//15/16=93.75%
			
			//反转
			bExCurrentBackwardMax=(gpParam->bCurrentBackward<<1)*InputBuf/30+gpParam->bCurrentBackward;			

			//3/4=75%
			bExCurrentBackwardPer85=(uchar)((uint16)(bExCurrentBackwardMax*7)>>3);
			//15/16=93.75%
		}
		
}
/////
void ProcessKey(uchar in,uchar old)
{
		#if CONFIG_FUNCTION_AUTO
		
		if( (in&BIT0)!=(old&BIT0) )	//是边沿
			{
				if((in&BIT0)==0)
					{	//k1
							if(Motor.FlagRuning!=1)
							{
								Motor.FlagRuning=0;
							Motor.CommandType=COMMAND_C1;		//按第一路正转
							FlagAuto=1;//非点动方式
							
							//333	Motor.TimePer94=0;
							Motor.iTimePer85=0;
							Motor.timer_sec100=0;
							Motor.RunTime=0;	//60s自动关掉
							Motor.unPushTime=0;
							}
					}
				}
	
		if( (in&BIT1)!=(old&BIT1) )	//是边沿
			{
				if((in&BIT1)==0)
					{	//k2
						//if(Motor.FlagRuning==0)
						if(Motor.FlagRuning!=2)	
							{
								Motor.FlagRuning=0;
							Motor.CommandType=COMMAND_CC1;		//反转
							FlagAuto=1;//非点动方式

							//333	Motor.TimePer94=0;
							Motor.iTimePer85=0;	
							Motor.timer_sec100=0;						
							Motor.RunTime=0;	//60s自动关掉
							Motor.unPushTime=0;
							}
					}
				}		
////				

////				
			#else	
			if( in !=old)
				{
					//333	Motor.TimePer94=0;
					Motor.iTimePer85=0;
					Motor.timer_sec100=0;
					Motor.RunTime=0;	//60s自动关掉
					Motor.unPushTime=0;
				}
			//k1
			if((in&BIT0)==0)
				{
					Motor.CommandType=COMMAND_C1;		//按第一路正转
					FlagAuto=0;//点动方式	
					CommandLostTimeOut=0;
				}
			else if((in&BIT1)==0)
				{//k2
					Motor.CommandType=COMMAND_CC1;		//反转
					FlagAuto=0;//点动方式	
					CommandLostTimeOut=0;
				}
				
			#endif
			
			if( (in&BIT2)!=(old&BIT2) )	//是边沿
						{
							if((in&BIT2)==0)
										{
											if(FlagSetCurrent1==0)	
												{
												
													//if((Motor.FlagRuning)&&(InputBuf==0x0f))
														if((Motor.FlagRuning)&&(InputBuf==0x0f)&&(FlagInputZero))
																{
																	FlagSetCurrent1=1;
																}
													/*
													Motor.FlagRuning=0;
													Motor.CommandType=COMMAND_C1;		//按第一路正转
													FlagAuto=1;//非点动方式
													Motor.TimePer94=0;
													Motor.iTimePer85=0;
													Motor.RunTime=0;	//60s自动关掉
													*/
												}
										}
						}			
			//ProcessCommand(Motor.CommandType);
}

/////
void CheckKey(void)		//周期10ms
{
		
				if(K1_LVL)
						KeybyteCur	|=	BIT0;
				else	KeybyteCur	&=	(~BIT0);
				if(K2_LVL)
						KeybyteCur	|=	BIT1;
				else	KeybyteCur	&=	(~BIT1);
				
				if(K3_LVL)
						KeybyteCur	|=	BIT2;
				else	KeybyteCur	&=	(~BIT2);
	/*
	#if Config_Al_Box				
				if(K4_LVL)
						KeybyteCur	|=	BIT3;
				else	KeybyteCur	&=	(~BIT3);					
	#endif
	*/

		
	
	//键盘噪声控制,两次相同有效
	
					if(KeybyteOld==KeybyteCur)		//上次读的输入
								KeybyteBuf=KeybyteCur;		//结果
					else  KeybyteOld=KeybyteCur;
		ProcessKey(KeybyteBuf,KeybyteOld_Buf);
			/*
			if((KeybyteOld_Buf!=KeybyteBuf)&&(0==(KeybyteBuf&BIT0)))
				{
				TestStepCount=0;
				FlagTestStep=1;
			}*/
		KeybyteOld_Buf=KeybyteBuf;
}
//////

void ProcessTimeOut(void)	//10MS命令保持
{
//#define COMMAND_CONTINU_TIME_300  40
#define COMMAND_CONTINU_TIME_300  30

static uchar tim1000ms;


//#if CONFIG_433SG
//	if(SamCommandTime<253)SamCommandTime++;
//#endif

//按钮松开自动关闭
	if(CommandLostTimeOut<253)CommandLostTimeOut++;	//没有收到命令的时间
	if(CommandLostTimeOut>COMMAND_CONTINU_TIME_300)	//没有收到命令的时间	//400ms
		{
			if(FlagAuto==0)
				{//点动方式
					Motor.FlagRuning=0;
					Motor.CurrentType=0;
					}
					
//		#if CONFIG_433SG ||CONFIG_CC1100
//			SamCommandTime=0;	//age
//		#endif	
		//	LastCommand=0;		
		

		}
		
////////////////////////////////////////////	
	if(tim1000ms<200)
		{
			tim1000ms++;
		}
	else{//到了2秒
		tim1000ms=0;
		LastCommand=0;

					
				
		/////////////////////////////
			if(ReInitCC1100Time>20)//if(ReInitCC1100Time>30)
										{
										
										ReInitCC1100Time=0;
							
												#if CONFIG_CC1100
															cc1100Initializtion();
												#endif		
										}
				else{
						ReInitCC1100Time++;
						}
			
			if(Motor.FlagPower>0)	//
						{
							SendToRemot(REMOT_COMMAND_PUSH_POWER_ON);
							Motor.FlagPower--;//每2秒减1
						}			
						
			}
///////////////////////////////////////////////////////////	
//超时自动关闭		
	if(Motor.timer_sec100<10)	
			{
				Motor.timer_sec100++;
			}
	else{//100MS
			Motor.timer_sec100=0;
			if(SetIdTime>0)SetIdTime--;			//设置ID时间	
			//if(Motor.RunTime<253)	//60s自动关掉	
			if(Motor.RunTime<65530)	//60s自动关掉		
				{
					Motor.RunTime++;
				}

			}
			
		
			if(Motor.RunTime>MAX_RUN_TIME_sec)	//80s自动关掉	
			//if((Motor.RunTime+Motor.unPushTime)>MAX_RUN_TIME_sec)	//80s自动关掉		
					{
					//Motor.RunTime=0;
					Motor.FlagRuning=0;
					Motor.CommandType=0;
					Motor.CurrentType=0;
					
					}	
		
			if(FlagSetCurrent1==1)
				{
				//if(Motor.RunTime>5)
				if(Motor.RunTime>50)	
						{
						FlagSetCurrent1=2;
						}
				}
		
			if(FlagSetCurrent1==3)
				{
					FlagSetCurrent1=0;
					Motor.FlagRuning=0;
					Motor.CommandType=0;
					Motor.CurrentType=0;
					#if CONFIG_SAVE5	
					Write_Param();
					#endif	
				}
					
			if(Motor.unPushTime!=0)
				{
					Motor.unPushTime--;
				}
		else{
		//if(Motor.RunTime>3)	//过电流处理，跳过启动时间？
		//if(Motor.RunTime>1)	//过电流处理，跳过启动时间？	
			{	
			switch(Motor.CurrentType)		//当前对方向,转向
				{//电流=(DIGETAL-5)*20	(mA)
											case COMMAND_C1:		//按住第一路正转
										case COMMAND_A1:			//按一下第一路正转
											if(chAdc_Resoult7>bExCurrentForwardPer85)
												{
												Motor.iTimePer85++;
												}
											else{
													if(Motor.iTimePer85>0)Motor.iTimePer85--;
													}
											
											//if((chAdc_Resoult7>bExCurrentForwardMax)||(0*Motor.iTimePer85>Time10000Ms))
											if((chAdc_Resoult7>bExCurrentForwardMax)||(Motor.iTimePer85>Time10000Ms))
														{
															Motor.OverType1=1;		//过流的方向,转向
															Motor.FlagRuning=0;
															Motor.CommandType=0;
															Motor.CurrentType=0;
															//Motor.unPushTime=0;
															SendToRemot(RESPONES_COMMPLETE);
															SendToRemot(RESPONES_COMMPLETE);
															SendToRemot(RESPONES_COMMPLETE);
															SendToRemot(RESPONES_COMMPLETE);

														}
										break;
										
										case COMMAND_CC1:		//按住第一路反转
										case COMMAND_AA1:			//按一下第一路反转
											if(chAdc_Resoult7>bExCurrentBackwardPer85)
												{
												Motor.iTimePer85++;
												}
											else{
													//333	Motor.TimePer94=0;
													if(Motor.iTimePer85>0)Motor.iTimePer85--;
													}
											
											//if((chAdc_Resoult7>bExCurrentBackwardMax)||(0*Motor.iTimePer85>Time10000Ms))
											if((chAdc_Resoult7>bExCurrentBackwardMax)||(Motor.iTimePer85>Time10000Ms))
												{
													Motor.OverType1=2;		//过流的方向,转向
													Motor.FlagRuning=0;
													Motor.CommandType=0;		
													Motor.CurrentType=0;
													//SendToRemot(RESPONES_COMMPLETE);
													
													#if CONFIG_UNPUSH
													//防止橡皮圈压得太紧，反转退到底后自动正转一点，放松
														//if(Motor.RunTime>10)	//运行10秒后有效
														if(Motor.RunTime>100)	//运行10秒后有效	
																		{
																			Motor.CommandType=COMMAND_C1;		//正转
																			FlagAuto=1;//非点动方式
																			//333	Motor.TimePer94=0;
																			Motor.iTimePer85=0;							
																			//Motor.RunTime=MAX_RUN_TIME_sec-3;	//反转退到底后自动正转3秒
																			Motor.timer_sec100=0;
																			#if Config_Al_Box
																			//if(KeybyteBuf&BIT3)	//K4_LVL
																				if(K4_LVL)
																				{//SetLed2
																					Motor.RunTime=MAX_RUN_TIME_sec-15;	//反转退到底后自动正转0.8秒,准对新款铝合金箱体
																					}
																			else{//闭合
																					//Motor.RunTime=MAX_RUN_TIME_sec-4;	//反转退到底后自动正转3秒
																					Motor.RunTime=MAX_RUN_TIME_sec-40;	//反转退到底后自动正转3秒
																					}	
																			#else			
																				//Motor.RunTime=MAX_RUN_TIME_sec-4;	//反转退到底后自动正转3秒
																				Motor.RunTime=MAX_RUN_TIME_sec-40;	//反转退到底后自动正转3秒
																			#endif
																		Motor.unPushTime=20;
																		}
																else{
																		SendToRemot(RESPONES_COMMPLETE);
																		SendToRemot(RESPONES_COMMPLETE);
																		SendToRemot(RESPONES_COMMPLETE);
																		SendToRemot(RESPONES_COMMPLETE);
																	
																		}
													#endif
													
																						
												}
										break;								
										
										default:
										
										break;
										
									}
				}
			}
}
////
void ProcessTimeSoftStart(void)	//10MS软启动
{	
//#define SoftStep	10
//#define SoftStep	20
#define SoftStep	15
		if(Motor.FlagRuning)
		{
			//软启动
			//if(Motor.Pwm1<(PWM_MAX_VAL-1))Motor.Pwm1+=1;
			if(Motor.Pwm1<(PWM_MAX_VAL-SoftStep))Motor.Pwm1+=SoftStep;
			else Motor.Pwm1=PWM_MAX_VAL;
			DropOffTime=0;	
		}
	else{

			#define CONFIG_SOFT_STOP		0
			
			#if CONFIG_SOFT_STOP
						if(Motor.Pwm1>(SoftStep*4))
							{
								Motor.Pwm1-=(SoftStep*4);	
							}
						else{
							  Motor.Pwm1=0;	//直接停止
							}
				#else
				Motor.Pwm1=0;	//直接停止
				#endif			
				
		if(Motor.Pwm1==0)
				{
				if(DropOffTime<253)DropOffTime++;//死区时间
				//if(DropOffTime>10)AllJspOff();//继电器关
				if(DropOffTime>30)AllJspOff();//继电器关	
				}
		 else{
		 			DropOffTime=0;
					}
		}
PwmContral1(Motor.Pwm1);	//OC1A PWM1 控制

}

/////
#if CONFIG_UART
void ParamSend(void)
{
	uchar len;
	//uchar i;
	uchar  buf[60];

	len		=	PutString("&ED,",buf,5);								//head 4
	len		+=	MakeValAsc8("I1=",gpParam->bCurrentForward,",",&buf[len]);	//
	len		+=	MakeValAsc8("I2=",gpParam->bCurrentBackward,",",&buf[len]);	//
	len		+=	MakeValAsc8("k=",gpParam->bCurrentRate,",",&buf[len]);	//
	
	#if CONFIG_433SG || CONFIG_CC1100
	len		+=	PutString("ID=",&buf[len],5);
	buf[len]=HexToAsc(gpParam->RemotName[0]>>4);
	len++;
	buf[len]=HexToAsc(gpParam->RemotName[0]);
	len++;
	buf[len]=HexToAsc(gpParam->RemotName[1]>>4);
	len++;
	buf[len]=HexToAsc(gpParam->RemotName[1]);
	len++;
			#if CONFIG_CC1100
			buf[len]=HexToAsc(gpParam->RemotName[2]>>4);
			len++;
			buf[len]=HexToAsc(gpParam->RemotName[2]);
			len++;
			#endif
	buf[len]=',';
	len++;
	#endif
	
	
	len		+=	PutString("\r\n",&buf[len],5);

SendText_UART0(buf);	
}
/////
void AutoSend(void)
{
	uchar len;
	//uchar i;
	uchar temp;
	
	uchar buf[60];
	len		=	PutString("&EA,",buf,5);								//head 4
	buf[len]=HexToAsc(Motor.FlagRuning);		//foreward向前正转1,backward向后2
	//buf[len]=HexToAsc(FlagInputZero);
	//buf[len]=HexToAsc(FlagSetCurrent1);
	len++;
	buf[len]=',';
	len++;
	
	len		+=	MakeValAsc8("I=",chAdc_Resoult7,",",&buf[len]);	//电流100ma
	len		+=	MakeValAsc8("F=",bExCurrentForwardMax,",",&buf[len]);	//adj
	len		+=	MakeValAsc8("B=",bExCurrentBackwardMax,",",&buf[len]);	//比较电流100ma
	
	//len		+=	MakeValAsc8("9=",bExCurrentBackwardPer94,",",&buf[len]);	//比较电流100ma
	//len		+=	MakeValAsc8("7=",bExCurrentBackwardPer85,",",&buf[len]);	//比较电流100ma
	
	//gpParam->bCurrentForward
	len		+=	PutString("In=",&buf[len],5);
	/*
	buf[len]=HexToAsc(InputBuf>>4);
	len++;
	*/
	//InputBuf低4位
	buf[len]=HexToAsc(InputBuf);
	len++;
	
	///
	#if	CONFIG_OSSCAL
			len		+=	PutString(",OS=",&buf[len],5);
			temp=OSSCAL_AT_MEGA48_FLASH;
			buf[len]=HexToAsc(temp>>4);
			len++;
			buf[len]=HexToAsc(temp);
			len++;
	#endif
	///
	#if CONFIG_433SG
			buf[len]=',';
			len++;
			len		+=	PutString("Pu=",&buf[len],5);
			buf[len]=HexToAsc(PulseCount>>12);
			len++;
			buf[len]=HexToAsc(PulseCount>>8);
			len++;
			buf[len]=HexToAsc(PulseCount>>4);
			len++;
			buf[len]=HexToAsc(PulseCount);
			len++;	
	#endif		

	/*
	len		+=	PutString(",Key=",&buf[len],5);
	for(i=0;i<4;i++)
		{if(Key_Buf[i])
			{
			buf[len]='1';	
			}
		else{
			buf[len]='0';
			}
		len++;
		}		
	*/	


len		+=	PutString("\r\n",&buf[len],5);

SendText_UART0(buf);
		
}
#if CONFIG_433SG
void RemotCodeSend(uchar *command1,uchar comlen1)
{
	uchar len;
	uchar i;
	
	uchar buf[60];
	
	if(comlen1>16)comlen1=16;
	
	len		=	PutString("&ER,",buf,5);								//head 4


	for(i=0;i<comlen1;i++)
					{
				
					buf[len]=HexToAsc(*(command1+i)>>4);
					len++;
					buf[len]=HexToAsc(*(command1+i));
					len++;
				}
	

	/*
	len		+=	PutString(",Key=",&buf[len],5);
	for(i=0;i<4;i++)
		{if(Key_Buf[i])
			{
			buf[len]='1';	
			}
		else{
			buf[len]='0';
			}
		len++;
		}		
	*/	


len		+=	PutString("\r\n",&buf[len],5);

SendText_UART0(buf);
		
}
#endif
#endif
/////
void Work(void)
{
	static uchar tim;
	uchar tim4;
		if(bTimeBase)	//程序时间到标记	10ms
				{	
				bTimeBase=0;

				//#if CONFIG_TEST_FREQ_PIN
				//Not_TEST_FREQ_PIN;
				//#endif	
					
				AdcProcess();
				CheckKey();		//周期10ms
				CheckInput();
#if CONFIG_433SG || CONFIG_CC1100
				ProcessRfRecv();
#endif				

				
				
				ProcessTimeSoftStart();	//10MS软启动
//				ProcessTestStep();
				ProcessJspSwitch();		//旋转方向切换 极性
				ProcessTimeOut();	//10MS命令保持
				


				LedContral();
	
				//chAdc_Resoult7
				if(Motor.FlagRuning==2)
							tim4=150;		//放松 1.5s闪烁
				else  tim4=30;		//顶进 300ms闪烁
				
				
				tim++;
				if(tim>tim4)
					{tim=0;
						#if CONFIG_UART
						AutoSend();
						#endif
					if((Motor.FlagRuning!=0)||(SetIdTime>0)||(Motor.FlagPower>0))
									{
									LED_RUN_ON;	//SetLed1
									}
					//if(Motor.FlagRuning!=0)
						if(Motor.CurrentType!=0)
							{
							SendToRemot(RESPONES_RUNING);	
							}
						
						
					}
				else if(tim>15)
					{
						if((Motor.FlagRuning!=0)||(SetIdTime>0)||(Motor.FlagPower==0))
										{
											LED_RUN_OFF;
										}
					}
#if Config_Al_Box					
			//	if(KeybyteBuf&BIT3)	//K4_LVL
					if(K4_LVL)
								{
									//SetLed2;	//跳线不连接表示铝箱体
									LED_Al_Box;
								}
				else{
								//ClrLed2;	//跳线连接表示铁箱体
								LED_Fe_Box;
						}
#endif				
				
				}
}
#include "config.h"
#include "nomal.h"
#include "Soft.h"
#include "Mega48_Adc.h"

#include "CC1100.h"




#include "Hd_ElecPush5.H"

	#if IAR_SYSTEM
		#include "iom88v.h"
        	#include "IAR_VECT.h"
	#else
		#include <iom88v.h>
	#endif



	#include "LoopBuf.h"

#if CONFIG_433SG
  #include "Decode.h"
		void Init_ext_Int1(void);
	#endif

void Init_Hd(void);

void InitOutLevelDefault(void);
//////////////////////////////




 
#if old_adc  
void InitAdc(void);
#define MAX_ADC_NUMBER	16

struct adc_struct 
{
	

	//unsigned short int sum6;
	unsigned short int sum7;	
	
	
	unsigned char ch;
	unsigned char number;
	unsigned char complete;
	
	unsigned char Time;
}Adc;
#endif


/////////////
	
//
void DelayMs(unsigned char x)
{
unsigned char i;
unsigned int j;
for(i=0;i<x;i++)
				{
				for(j=0;j<570;j++)	//4mhz
								{}
				//asm("wdr");
				}
				
}
///
void LedStart(void)   /* 启动时指示灯闪烁  */
{

	uchar i;
	ClrLed1;
	ClrLed2;
	for(i=0;i<3;i++)
	{
		DelayMs(100);
		SetLed1;
		ClrLed2;
		DelayMs(50);
		ClrLed1;
		SetLed2;
		#if EnWdog
		asm("wdr");
		#endif
	}

	/*
	for(i=0;i<3;i++)
	{
		DelayMs(100);
		SetLed2;
		DelayMs(50);
		ClrLed2;
		#if EnWdog
		asm("wdr");
		#endif
	}
	*/
ClrLed1;
ClrLed2;
}

////////////////////////
void SendTestCC1100M(void) //上电时按住键则发送测试包
{

uchar buf[36+14];
uchar len;
uint16	iTimeout;;	

for(len=0;len<30;len++)
		{
		buf[len]='*';
		}


buf[0]=0x31;	//1~8
buf[1]=gpParam->RemotName[0];
buf[2]=gpParam->RemotName[1];
buf[3]=gpParam->RemotName[2];
buf[4]=REMOT_COMMAND_POWER_OFF;
buf[5]=giCodeChecksum>>8;
buf[6]=giCodeChecksum&0xff;
buf[7]=0x70;



						len=8;
						len		+=	MakeValAsc8("K",gpParam->bCurrentRate,",",&buf[len]);	//
						len+=MakeValAsc8("I",gpParam->bCurrentForward,",",&buf[len]);
						len+=MakeValAsc8("",gpParam->bCurrentBackward,",",&buf[len]);
						len+=MakeValAsc8("F",cc1100regcfg[2],",",&buf[len]);
						



buf[30]=0x0d;
buf[31]=0x0a;
buf[32]=0x00;

if(K5_LVL==0)
		{
			cc1100Initializtion();
			DelayMs(10);
		}
		

		iTimeout=0;
while((K5_LVL==0)&&(iTimeout<2000))
				{
					iTimeout++;
					cc1100Printf(buf);
					asm("wdr");
				}
}
////////////////////////
uint16 GetCodeCheckSum(void)
{
	uint16 sum;
	uint16 i;
	const uchar *p;
	p=0;
	sum=0;
	for(i=0;i<8192;i++)  //mega88
			{
				sum+=*(p+i);
				
			}		

return sum;
}
//////////////////////

void main(void)
{
	uchar tim88;
	uchar ReSendCount;
	
InitOutLevelDefault();
gpParam=(struct	struct_save	*)&gbParamBuf[0];		//参数结构的指针


#if EnWdog
asm("wdr");
//WDTCR=0X1F;  //mage16
//WDTCR=0X0F;

WDTCSR=0X1F;
WDTCSR=0X0F;
asm("wdr");
#endif

//DefaultSet();
Default_ParamInit();	//Init_Param();
#if CONFIG_SAVE5
Load_Param();		//取设定值
#endif

Init_Hd();
InitAdc();
LedStart();

#if EnWdog
asm("wdr");
#endif
DelayMs(200);

#if EnWdog
asm("wdr");
#endif

if(K5_LVL==0)
		{
			DelayMs(5);
			giCodeChecksum=GetCodeCheckSum();
			
			if(K5_LVL==0)
						{
						/*
						&ED,I1=048,I2=049,I3=050,ID=0000,
						&EC,I=000,In=00,Pu=0208
						&ER,3132333435
						*/
						SendText_UART0("***Push5**\r\n");
						ParamSend();
						AutoSend();
								#if CONFIG_433SG
										RemotCodeSend("12345",5);
								#endif							
						SendTestCC1100M();//上电时按住键则发送测试包	
						}

		}


///////////////////
//LedStart(); 

#if CONFIG_CC1100
cc1100Initializtion();

#elif CONFIG_433SG
InitDecode();
#endif		

//////////////////////
//DelayMs(200);
#if EnWdog
asm("wdr");
#endif


	while(1)
			{
			#if EnWdog
			asm("wdr");
			#endif
				Work();

			}
}

/////////////////////
void InitOutLevelDefault(void)
{



}
//////


/////////////////////
void Init_GPIO(void)
{
JspOut1_OFF;
JspOut2_OFF;
JspOut1_Enable;
JspOut2_Enable;

SW_PWM1_OFF;
SW_PWM1_ENABLE; 	


Led1_Enable;

K1_PULLUP;
K2_PULLUP;
K3_PULLUP;
K4_PULLUP;
K5_PULLUP;



Led2_Enable;	//选择铝箱体指示灯SetLed2

K4_PULLUP;		//选择铁箱体跳线
K5_PULLUP;

		#if CONFIG_433SG == 0
				#if CONFIG_TEST_FREQ_PIN
				Enable_TEST_FREQ_PIN;	//单片机频率测试Not_TEST_FREQ_PIN;
				#endif
	#endif				
}
////
#if CONFIG_SAVE5
void EEPROM_Write(unsigned int iAdr,unsigned char bDat)
{
	uint16 i;
	i=0;
	while( (i<50000) && (EECR & (1<<EEWE)) )
		{//等待前一次操作完成
			i++;
		}
	while( (i<50000) && (EECR & (1<<EEWE)) )
		{//等待前一次操作完成
			i++;
		}		
EEAR = iAdr;
EEDR	=	bDat;
EECR	|=	(1<<EEMWE);		//写EEMWE位，允许EEPROM操作
EECR	|=	(1<<EEWE);		//写EEWE位，开始EEPROM操作
	
}
////////////


unsigned char EEPROM_Read(unsigned int iAdr)
{
	uint16 i;
	i=0;
	while( (i<50000) && (EECR & (1<<EEWE)) )
		{//等待前一次操作完成
			i++;
		}
EEAR = iAdr;
EECR	|=	(1<<EERE);		//读EEWE位，开始EEPROM操作
return EEDR;	
}
////
void Read_Param(uchar *buf)
{
uchar i;
		for(i=0;i<Max_Param_Len;i++)
				{
				buf[i]=EEPROM_Read(EEPROM_BASE_ADR+i);
				}
}
void Write_Param(void)
{
uchar i,Sum;
///
Sum=0;
for(i=0;i<Max_Param_Len-1;i++)
		{
		Sum+=gbParamBuf[i];
		}
gbParamBuf[Max_Param_Len-1]=~Sum;	//校验码求反
///
for(i=0;i<Max_Param_Len;i++)
										{
										EEPROM_Write(EEPROM_BASE_ADR+i,gbParamBuf[i]);
										}
										
										
}
////
void Load_Param(void)
{
	struct	struct_save	*p;				//15869127174
	uchar buf[Max_Param_Len+2];
	uchar i,j,Sum;
	uchar FlagOk;
	
	p=(struct	struct_save	*)&buf[0];
	
	for(j=0;j<3;j++)
		{
	Read_Param(buf);
	FlagOk=0;
	if(p->flag==FlagParamInitnized)
			{//参数已经初始化标记正确
			
			Sum=0;
			for(i=0;i<Max_Param_Len;i++)
				{
				Sum+=buf[i];//校验码
				}
			if(Sum==0xff)//校验码
				{
					FlagOk=1;
					break;
				}
			}
		}
		
	if(p->flag!=FlagParamInitnized)
		{//EEPROM参数初始化
			//if((p->flag==0xff)||(p->flag==0))
							{
							Write_Param();
							}
			}
					
if(FlagOk)	
			{		
			for(i=0;i<Max_Param_Len;i++)
				{//拷贝参数
				gbParamBuf[i]=buf[i];
				}
			}
}
#endif
////

void Init_Uart(unsigned int baud,unsigned char parity)
{


	DDRD	&=	(~BIT0);	//RXD
	
	PORTD	|=	BIT0;			//上拉

	
	DDRD	|=	BIT1;			//TXD
	
	
	
UCSR0B	=	0X0;				//禁止发送和接收
UCSR0A	|=	(1<<U2X0);	//0X02;				//倍速 U2X0


		switch	(parity)
			{
				case 1:
					UCSR0C	=	0X36;	//0XB6;				//8数据位,奇校验	
				break;
				case 2:
					UCSR0C	=	0X26;	//0XA6;				//8数据位,偶校验	
				break;
				case 0:
				default:
					UCSR0C	=	0X06;	//0X86;				//8数据位,无校验
				break;
				}
UBRR0L	=	baud&0XFF;
UBRR0H	=	(baud>>8)&0XFF;


//中断
	UCSR0B	|=	(1<<RXEN0)+(1<<TXEN0);				//允许接收和接收
	UCSR0B	|=	(1<<RXCIE0);				//允许接收中断
	//UCSR0B	|=	(1<<UDRIE0);				//允许发送缓冲器空中断
	UCSR0B	|=	(1<<TXCIE0);				//允许发送完成中断
	
	

}


/////

#if CONFIG_433SG
void Init_ext_Int1(void)//遥控器接收中断
{
PORTD	|=	BIT3;		//EX_INT1	PORTD3
DDRD	&=(~BIT3);	//输入
EICRA	|=	(1<<ISC11);	//下降沿
EIMSK	|=	BIT1;	//MEGA88
}
#endif
/////////////////
void Init_Timer0(void)
{
	TCNT0	=	0;		//计数器
	//TCCR0B	=	0X03;	//ICNC1捕捉噪声抑制  /64
	//8bit 定时器
	//TCCR0B	=	0X05;	//ICNC1捕捉噪声抑制  /1024
	TCCR0B	=	0X04;	//ICNC1捕捉噪声抑制  /256  //遥控器编码需要
	//TIMSK0	|=	(1<<OCIE0A)+(1<<OCIE0B)+(1<<TOIE0);
	TIMSK0	|=	(1<<OCIE0B)+(1<<TOIE0);
}




/////

void PwmContral1(uchar x)	//OC1A PWM1 控制
{
	
	#if CONFIG_PWM != 1
			if(x>=5)x=PWM_MAX_VAL;
	#endif
	
	if(x<5)
		{
		TCCR1A	&=	( ~ (1<<COM1A1 ));
		TCCR1A	&=	( ~ (1<<COM1A0 ));
		SW_PWM1_OFF;		
		}
	else if(x>(PWM_MAX_VAL-10))
		{
		TCCR1A	&=	( ~ (1<<COM1A1 ));
		TCCR1A	&=	( ~ (1<<COM1A0 ));
		SW_PWM1_ON;			
			
		}
	else{//比较匹配时清零
		TCCR1A	|=	(1<<COM1A1);
		TCCR1A	&=	( ~ (1<<COM1A0 ));
		//SW_PWM1_OFF;	
		
		}
	
	#if FREQ_PWM_RATE	==	2
		//if(x>PWM_MAX_VAL)x=PWM_MAX_VAL;
		//OCR1A=(PWM_MAX_VAL-x)<<2;	//*4
		OCR1A=(x)<<2;	//*4
		
	#elif FREQ_PWM_RATE	==	4
		//if(x>PWM_MAX_VAL)x=PWM_MAX_VAL;
		//OCR1A=(PWM_MAX_VAL-x)<<1;	//*2
		OCR1A=(x)<<1;	//*2
	#else 
	//x8  x1
		//if(x>PWM_MAX_VAL)x=PWM_MAX_VAL;
		//OCR1A=PWM_MAX_VAL-x;
		OCR1A=x;	
	#endif



}

////
/*
void PwmContral2(uchar x)	//OC1B PWM2 控制
{
	#if CONFIG_PWM != 1
			if(x>=5)x=PWM_MAX_VAL;
	#endif
	
	if(x<5)
		{
		TCCR1A	&=	( ~ (1<<COM1B1 ));
		TCCR1A	&=	( ~ (1<<COM1B0 ));	
		//SW_PWM2_OFF;			
		}
	else if(x>(PWM_MAX_VAL-10))
		{
		TCCR1A	&=	( ~ (1<<COM1B1 ));
		TCCR1A	&=	( ~ (1<<COM1B0 ));	
		//SW_PWM2_ON;			
		}
	else{
		TCCR1A	|=	(1<<COM1B1);
		TCCR1A	&=	( ~ (1<<COM1B0 ));	
		//SW_PWM2_OFF;
		}
		
	//ICR1=PWM_MAX_VAL;
	
		#if FREQ_PWM_RATE	==	2
		//ICR1=PWM_MAX_VAL;
		if(x>PWM_MAX_VAL)x=PWM_MAX_VAL;
		OCR1B=(PWM_MAX_VAL-x)<<2;	//*4
	#elif FREQ_PWM_RATE	==	4
		if(x>PWM_MAX_VAL)x=PWM_MAX_VAL;
		OCR1B=(PWM_MAX_VAL-x)<<1;	//*2
	#else 
	//x8  x1
		//ICR1=PWM_MAX_VAL;
		if(x>PWM_MAX_VAL)x=PWM_MAX_VAL;
		OCR1B=PWM_MAX_VAL-x;	
	#endif
	
	
		
}
*/
/////////////////////
void Init_Timer1_PWM(void)
{

#if CONFIG_PWM



	TCNT1	=	0;		//计数器
	//TCCR1B	|=	(1<<ICNC1)+0X03;	//ICNC1捕捉噪声抑制  /64
	//PWM模式 1000
	TCCR1B	|=	(1<<WGM13);
	//TCCR1B	|=	(1<<WGM12);
	//TCCR1A	|=	(1<<WGM11);
	//TCCR1A	|=	(1<<WGM10);
	
	//OC1A,OC1B模式
	//TCCR1A	|=	(1<<COM1A1);
	//TCCR1A	&=	( ~ (1<<COM1A0 ));
	//TCCR1A	|=	(1<<COM1B1);
	//TCCR1A	&=	( ~ (1<<COM1B0 ));
	
	//值
	#if FREQ_PWM_RATE	==	2
		TCCR1B	|=	0X02;	//timer1 start/8  f=4000 000 /2 /8 /1024  =244hz
		ICR1=PWM_MAX_VAL<<2;
		//OCR1A=PWM_MAX_VAL<<2;
		//OCR1B=PWM_MAX_VAL<<2;
		OCR1A=0;
		OCR1B=0;		
	#elif FREQ_PWM_RATE	==	4
		TCCR1B	|=	0X02;	//timer1 start/8  f=4000 000 /2 /8 /512  =488hz
		ICR1=PWM_MAX_VAL<<1;
		//OCR1A=PWM_MAX_VAL<<1;
		//OCR1B=PWM_MAX_VAL<<1;
		OCR1A=0;
		OCR1B=0;		
	#elif FREQ_PWM_RATE	==	8
		TCCR1B	|=	0X02;	//timer1 start/8  f=4000 000 /2 /8 /256  =976hz
		ICR1=PWM_MAX_VAL;
		//OCR1A=PWM_MAX_VAL;
		//OCR1B=PWM_MAX_VAL;
		OCR1A=0;
		OCR1B=0;				
	#else
		TCCR1B	|=	0X03;	//timer1 start/64  f=4000 000 /2 /64 /256  =122hz
		ICR1=PWM_MAX_VAL;
		//OCR1A=PWM_MAX_VAL;
		//OCR1B=PWM_MAX_VAL;
		OCR1A=0;
		OCR1B=0;		
	#endif
	

	//TCCR1C	&=(~FOC1A);
	//TCCR1C	&=(~FOC1B);
	//TIMSK1	|=	(1<<OCIE1A)+(1<<TICIE1)+(1<<TOIE1);
	TIMSK1	|=	(1<<OCIE1A)+(1<<OCIE1B)+(1<<TOIE1);

	#endif
}
/////
void OSCCAL_calibration(void)
{
	/*
		unsigned char step=0;
    unsigned char calibrate = FALSE;
    int temp;
    unsigned char tempL;
    
    int TestCount1,TestCount2;	//测试用
    TestCount1=0;
    TestCount2=0;

    CLKPR = (1<<CLKPCE);        // set Clock Prescaler Change Enable
    // set prescaler = 8, Inter RC 8Mhz / 8 = 1Mhz
    CLKPR = (1<<CLKPS1) | (1<<CLKPS0);
    
    TIMSK2 = 0;             //disable OCIE2A and TOIE2

    ASSR = (1<<AS2);        //select asynchronous operation of timer2 (32,768kHz)
    
    OCR2A = 200;            // set timer2 compare value 

    TIMSK0 = 0;             // delete any interrupt sources

        
    TCCR1B = (1<<CS10);     // start timer1 with no prescaling
    TCCR2A = (1<<CS20);     // start timer2 with no prescaling

    //while((ASSR & 0x01) | (ASSR & 0x04));       //wait for TCN2UB and TCR2UB to be cleared
tempL=(ASSR & 0x01) | (ASSR & 0x04);


    Delay(1000);    // wait for external crystal to stabilise
    
    while(!calibrate)
    {
        __disable_interrupt();  // disable global interrupt
        
        TIFR1 = 0xFF;   // delete TIFR1 flags
        TIFR2 = 0xFF;   // delete TIFR2 flags
        
        TCNT1H = 0;     // clear timer1 counter
        TCNT1L = 0;
        TCNT2 = 0;      // clear timer2 counter
           
        while ( !(TIFR2 & (1<<OCF2A)) );   // wait for timer2 compareflag
    
        TCCR1B = 0; // stop timer1

        __enable_interrupt();  // enable global interrupt
    
        if ( (TIFR1 & (1<<TOV1)) )
        {
            temp = 0xFFFF;      // if timer1 overflows, set the temp to 0xFFFF
        }
        else
        {   // read out the timer1 counter value
            tempL = TCNT1L;
            temp = TCNT1H;
            temp = (temp << 8);
            temp += tempL;
        }
    
//        if (temp > 6250)
//        {
//            OSCCAL--;   // the internRC oscillator runs to fast, decrease the OSCCAL
//        }
//        else if (temp < 6120)
//        {
//            OSCCAL++;   // the internRC oscillator runs to slow, increase the OSCCAL
//        }
//        else
//            calibrate = TRUE;   // the interRC is correct
    if (step==0)
		    	{//第一步
		    	if (temp < 6750)		//先调高20%
		    			{
		    			if(OSCCAL<0X7A)	//最大0X7F
		    					OSCCAL+=5;
		    			else step=1;
		    			}
		    	 else step=1;
		    	 
		    	 TestCount1++;	
		    	}
    else  //(step!=0)
    	{	//第二步
		    if (temp > 6160)		//	200/32768=6103us
		        {
		           if( OSCCAL>0)
		            		OSCCAL--;   // the internRC oscillator runs to fast, decrease the OSCCAL
		        		else
		    						calibrate = TRUE;   // the interRC is correct
		        }
		    else
		    		calibrate = TRUE;   // the interRC is correct
		    TestCount2++;
    		}
    		
        TCCR1B = (1<<CS10); // start timer1
    }
*/
}

/////
void Init_Hd(void)
{

#if CONFIG_EXT_4MHZ_OSC
//外部晶振
	CLKPR=0X80;	//1000 0000不分频
//	CLKPR=0X10;	//1000 0000
	CLKPR=0X00;	//0000 0000
	
#else
//内部晶振	
	CLKPR=0X80;	//1000 0000	二分频
	//CLKPR=0X01;	//1000 0001
	CLKPR=0X01;	//0000 0001
#endif	
	
	
	Init_GPIO();

	//内部RC振荡器频率校准
	//OSCCAL = EEPROM_Read(EEPROM_OSCCAL_ADR);

	#if	CONFIG_OSSCAL
	if(OSSCAL_AT_MEGA48_FLASH!=0xff)
			{
			OSCCAL = OSSCAL_AT_MEGA48_FLASH;
			}
#endif
		
	/////
	ClearLoopBuf(&Uart0SendStruct,UART0_SEND_BUF_SIZE);	//清除环型缓冲区
Init_Uart(BAUD_DEFINE(9600),0);
//	Init_Uart(BAUD_DEFINE(9400),0);
	//Init_Uart(BAUD_DEFINE(9200),0);
	
	
	//InitAdc();
	Init_Timer0();
	Init_Timer1_PWM();
	#if CONFIG_433SG
	Init_ext_Int1();	//遥控器接收中断
	#endif
SREG	|=	0X80;					//使能总中断
}
///////////////





/////

uchar Uart0CharSend(uchar x)
{
	

	uchar fail=0;
	uchar t;
	
    if(Uart0SendStruct.len>3)
	    	{//full
	    	t=5;
	    	while((Uart0SendStruct.addTail == Uart0SendStruct.outTail)&&(t>0))
	    		{
	    		//OSTimeDly(OS_TICKS_PER_SEC / 200);	//延时5毫秒	
	    		DelayMs(5);
	    		t--;
	    		}
	    	}
	    if(t==0)
	    	fail=1;
				
	//OS_ENTER_CRITICAL();	//ucos2   
	AddLoopBuf(&Uart0SendStruct,Uart0SendBuf,UART0_SEND_BUF_SIZE,x);	//加入到环型缓冲区
	//OS_EXIT_CRITICAL();          //ucos2  	
	

	
	if(UCSR0A&(1<<UDRE0))	//缓冲器空
	             {
	              // Set_En485; 	//485允许发送
	              //OS_ENTER_CRITICAL();	//ucos2
	        				UCSR0B	|=	(1<<UDRIE0);				//允许发送缓冲器空中断
									//UCSR0B	|=	(1<<TXCIE0);				//允许发送完成中断
	        			//OS_EXIT_CRITICAL();          //ucos2  	
	             }
	             
	          
	             
	return (!fail);;
}

/////////////////////////

//////////////////


////////////////////////

/////////////////////////////


#if IAR_SYSTEM
#pragma vector=INT0_vect	
__interrupt void ext_int0_isr(void)
#else
#pragma interrupt_handler ext_int0_isr:iv_INT0	//2
void ext_int0_isr(void)	
#endif
{
//EIMSK	&=	(~BIT0);	//MEGA88	
	
}
///
#if IAR_SYSTEM
#pragma vector=INT1_vect	
__interrupt void ext_int1_isr(void)
#else
#pragma interrupt_handler ext_int1_isr:iv_INT1	//2
void ext_int1_isr(void)	
#endif
{
//EIMSK	&=	(~BIT1);	//MEGA88	
#if CONFIG_433SG
		uchar temp;	//当前时刻
		temp=TCNT0;	//timer0
		DecodePulse(temp);
		PulseCount++;	
#endif		

}
///
#if IAR_SYSTEM
#pragma vector=USART_RX_vect	
__interrupt void uart0_rxd_isr(void)
#else
#pragma interrupt_handler uart0_rxd_isr:iv_USART0_RX	//19
void uart0_rxd_isr(void)	
#endif
{


	unsigned char temp;	
	
	temp	=	UDR0;
	
	
	AddLoopBuf(&Uart0RecvStruct,Uart0RecvBuf,UART0_RECV_BUF_SIZE,temp);	//加入到环型缓冲区




}
//////////////////

#if IAR_SYSTEM
#pragma vector=USART_UDRE_vect	
__interrupt void uart0_udre_isr(void)
#else
#pragma interrupt_handler uart0_udre_isr:iv_USART0_UDRE		//20发送数据寄存器空
void uart0_udre_isr(void)
#endif

{//20发送数据寄存器空
	

	unsigned char temp;
	

	
	if(GetLoopbuf(&Uart0SendStruct,Uart0SendBuf,UART0_SEND_BUF_SIZE,&temp))	//从环型缓冲区提取	
			{
				//Set_En485; 	//485允许发送
				UDR0	=	temp;
			}
		else{
				UCSR0B	&=	~(1<<UDRIE0);				//防止发送缓冲器空中断
				}
	

	

}
//////////////////
#if IAR_SYSTEM
#pragma vector=USART_TX_vect	
__interrupt void uart0_txc_isr(void)	
#else
#pragma interrupt_handler uart0_txc_isr:iv_USART0_TXC		//21发送数据完成
void uart0_txc_isr(void)	
#endif
{

	//Clr_En485; 	//485禁止发送


}

//////////
#if IAR_SYSTEM
#pragma vector=TIMER0_COMPA_vect	
__interrupt void timer0_compa_isr(void)
#else
#pragma interrupt_handler timer0_compa_isr:iv_TIMER0_COMPA //12
void timer0_compa_isr(void)
#endif
{

}
//////////

#if IAR_SYSTEM
#pragma vector=TIMER0_COMPB_vect	
__interrupt void timer0_compb_isr(void)
#else
#pragma interrupt_handler timer0_compb_isr:iv_TIMER0_COMPB //12
void timer0_compb_isr(void)
#endif

{
	OCR0B	+=		TIMER_10MS_8BIT;
	bTimeBase	=	1;	//程序时间到标记
				
		#if CONFIG_433SG == 0
				#if CONFIG_TEST_FREQ_PIN
				Not_TEST_FREQ_PIN;	//单片机频率测试Enable_TEST_FREQ_PIN;
				#endif
		#endif
}

//////////
#if IAR_SYSTEM
#pragma vector=TIMER0_OVF_vect
__interrupt void time0_ovf_isr(void)
#else
#pragma interrupt_handler time0_ovf_isr:iv_TIMER0_OVF	//14
void time0_ovf_isr(void)
#endif
{
#if CONFIG_433SG
		DecodeTimeout++;
#endif		

}
//////////



#if IAR_SYSTEM
#pragma vector=TIMER1_CAPT_vect
__interrupt void time1_capt_isr(void)
#else
#pragma interrupt_handler time1_capt_isr:iv_TIMER1_CAPT	//11
void time1_capt_isr(void)
#endif
{
	/*
  
      uchar temp;
	
	temp	=	ICR1L;	//先读低位
	//Measure.ulFreeTimer	+=	temp;
	temp	=	ICR1H;
	//Measure.ulFreeTimer	+=	(uint16)temp*0x100;
	
	*/		
}
///
#if IAR_SYSTEM
#pragma vector=TIMER1_COMPA_vect	
__interrupt void timer1_compa_isr(void)
#else
#pragma interrupt_handler timer1_compa_isr:iv_TIMER1_COMPA //12
void timer1_compa_isr(void)
#endif
{


}
///

#if IAR_SYSTEM
#pragma vector=TIMER1_OVF_vect
__interrupt void time1_ovf_isr(void)
#else
#pragma interrupt_handler time1_ovf_isr:iv_TIMER1_OVF	//14
void time1_ovf_isr(void)
#endif
{

}

/////////
#if IAR_SYSTEM
#pragma vector=ADC_vect
__interrupt void adc_isr(void)
#else
#pragma interrupt_handler adc_isr:iv_ADC	//22
void adc_isr(void)	
#endif
{
#if old_adc
if(1)
	{
unsigned short int temp16;
//CPU_MAGE16 MAGE48 都可以
//左对齐...
//ADMUX	=	(1<<ADLAR)+Adc.ch;
temp16 =ADCH;
//右对齐...
//temp16	=	ADCL;
//temp16	+=	ADCH<<8;


/*
switch(Adc.ch)
	{	
		/////////////////
		case 0:
		Adc.sum0	+=	temp16;	
		Adc.ch	=	1;
		break;	
				
		case 1:
		Adc.sum1	+=	temp16;	
		Adc.ch	=	2;
		break;
	
		case 2:
		Adc.sum2	+=	temp16;	
		Adc.ch	=	3;
		break;
		
		case 3:
		Adc.sum3	+=	temp16;	
		Adc.ch	=	4;
		break;
		

		case 4:
		Adc.sum4	+=	temp16;	
		Adc.ch	=	5;
		break;
		
		case 5:
		Adc.sum5	+=	temp16;	
		Adc.ch	=	6;
		break;
		//////////////
		
		case 6:
		Adc.sum6	+=	temp16;	
	
		Adc.ch	=	7;	//PA7作为单向无线电的RSSI输入
		break;

		case 7:				//
		Adc.sum7	+=	temp16;	
		Adc.ch	=	6;
		//Adc.ch	=	0;	//
		break;
		
		default:
			//Adc.ch	=	0;
			Adc.ch	=	7;
		break;
	}
	*/

Adc.sum7	+=	temp16;	


	//ADMUX	=	(1<<REFS0)+(1<<REFS1)+Adc.ch;
	//ADMUX	=	(1<<ADLAR)+Adc.ch;	//左对齐
	ADMUX	=	(1<<ADLAR)+7;	//只有第7路ADC
	//ADMUX	=	Adc.ch;
		//if(Adc.number<2*MAX_ADC_NUMBER)
		if(Adc.number<MAX_ADC_NUMBER)			
			{
			Adc.number	++;
			ADCSRA	|=	(1<<ADSC);		//启动转换
			}
	else{
			//ADCSRA	&=	(~(1<<ADEN));
			Adc.complete	=	1;
			//TestAdc_LED1_OFF;
			}
		}
#else			
		unsigned short int temp16;
//CPU_MAGE16 MAGE48 都可以
//左对齐...
//ADMUX	=	(1<<ADLAR)+Adc.ch;
temp16 =ADCH;
//右对齐...
//temp16	=	ADCL;
//temp16	+=	ADCH<<8;

if(Adc.ch<=LAST_ADC_CH)	//Big Error   Adc.sum范围为6-7
	{
		//Adc.sum[Adc.ch]	+=	temp16;	   //Big Error   Adc.sum范围为6-7
		Adc.sum[Adc.ch - FIRST_ADC_CH]	+=	temp16;			//Big Error  clred
		}
if(Adc.ch<LAST_ADC_CH)
	{
		Adc.ch++;
	}
else{
		Adc.ch=FIRST_ADC_CH;
		}

	//ADMUX	=	(1<<REFS0)+(1<<REFS1)+Adc.ch;
	//ADMUX	=	(1<<ADLAR)+Adc.ch;	//左对齐

	ADMUX	=	(1<<ADLAR)+Adc.ch;	//左对齐
		//if(Adc.number<2*MAX_ADC_NUMBER)
		if(Adc.number<(LAST_ADC_CH+1-FIRST_ADC_CH)*MAX_ADC_NUMBER)			
			{
			Adc.number	++;
			ADCSRA	|=	(1<<ADSC);		//启动转换
			}
	else{
			//ADCSRA	&=	(~(1<<ADEN));
			Adc.complete	=	1;
			//TestAdc_LED1_OFF;
			}	
			
#endif			
}

//////
#if old_adc
void InitAdc(void)
{
	
	ADCSRA	&=	~(1<<ADEN);		//关闭ADC
	
	
	ADCSRA	|=	(1<<ADPS2);		//32分频
	ADCSRA	&=	(~(1<<ADPS1));
	ADCSRA	|=	(1<<ADPS0);	
/*
	ADCSRA	|=	(1<<ADPS2);		//128分频
	ADCSRA	|=	(1<<ADPS1);
	ADCSRA	|=	(1<<ADPS0);
		*/	
	
	//ADMUX		|=	(1<<REFS0);	//内部参考源
	//ADMUX		|=	(1<<REFS1);	//内部参考源
	
	
	


		
	
	//Adc.ch	=	0;
	//Adc.ch	=	6;
	//ADMUX	=	(1<<REFS0)+(1<<REFS1)+Adc.ch;
	//ADMUX	=	(1<<ADLAR)+Adc.ch;	//左对齐
	ADMUX	=	(1<<ADLAR)+7;	//只有第7路ADC
	//ADMUX	=	Adc.ch;
	//	|=	(1<<REFS0);	//参考源VCC
	
	//DIDR0	|=	0x03;
	//DIDR0	|=	0x3f;	//Digital Input Disable Register
	//DIDR0	|=	0x3e;	//Digital Input Disable Register
	//DIDR0	|=	0xc0;	//Digital Input Disable Register  ADC6 ADC7
		DIDR0	|=	0x80;	//Digital Input Disable Register  ADC7	

	
	ADCSRA	|=	(1<<ADEN);	//开ADC
	
}
//////
void StartAdc(void)
{
//CPU_MAGE16 MAGE48 都可以
	//ADCSRA	&=	(~(1<<ADEN));		//关闭ADC

	//Adc.max1	=	0;
	//Adc.min1	=	65535;
	

	//Adc.sum6	=	0;
	Adc.sum7	=	0;		
	Adc.complete	=	0;
	Adc.number	=	1;
	ADMUX	=	(1<<ADLAR)+7;	//只有第7路ADC
	
	
	ADCSRA	|=	(1<<ADIE);		//中断允许
	//ADCSRA	|=	(1<<ADEN);		//
	ADCSRA	|=	(1<<ADSC);		//启动转换
	
//	TestAdc_LED1_ON;		//指示AD周期
}
////////

////////
void StopAdc(void)
{//MAX_ADC_NUMBER
uint16 temp16;
uchar xxx;

	if(Adc.complete)
			{
	

		//	temp16=(Adc.sum6>>4);	
			//if(temp16>128)temp16=128;		//32/128=25%
		//	chAdc_Resoult6=(uchar)(temp16);	
			

		
			
			//temp16=((Adc.sum7>>4)*33)>>8;	//(UINT=100mA)
			//temp16=((Adc.sum7>>4)*330)>>8;	//(UINT=10mA) max=1980mA
			temp16=(Adc.sum7>>4);	
			if(FlagSetCurrent1==2)
					{//10A,显示50
				if(temp16>25)
							{
							xxx=10*5*128/(temp16);
							if (xxx>60)
										{
										gpParam->bCurrentRate=(uchar)xxx;
										FlagSetCurrent1=3;
										}
							}
						}
			
			
			/*
			if (temp16>(65535/330))temp16=(65535/330);
			chAdc_Resoult7 = (uchar)((temp16*330)>>8);
		*/
				if (temp16>(32767/gpParam->bCurrentRate))temp16=(32767/gpParam->bCurrentRate);
			chAdc_Resoult7 = (uchar)((temp16*gpParam->bCurrentRate)>>7);		//uint 200ma
			
			
			
			
			}
}
//////
/*
void CloseAdc(void)
{

	ADCSRA	&=	~(1<<ADEN);
		ADMUX=0;
}
*/
///////
void AdcProcess(void)	//周期10mS
{	
	if(Adc.Time<20)Adc.Time++;
		else {
			Adc.Time=0;
		}
						switch(Adc.Time)
							{
							case 1:	//测量时间100MS
							StopAdc();
							//CloseAdc();
							break;
							
							case 2:
							/*
							InitAdc();
							
							*/
							StartAdc();
							break;
							
							
							
								
							default:
							break;	
							}
}
#endif //old_adc
/////////

void LedContral(void)  /* 亮灭转换  */
{
	
				if(Time_LedRecv_LED>0)
						{
							Time_LedRecv_LED--;
							#if Config_Al_Box					
								if(K4_LVL)
										{
											//SetLed2;	//跳线不连接表示铝箱体
											LED_Fe_Box;  //信号时候暗
										}
							else{
											//ClrLed2;	//跳线连接表示铁箱体
											LED_Al_Box;  //信号时候亮
									}
							#endif
										
						}
				else{
							#if Config_Al_Box					
								if(K4_LVL)
										{
											//SetLed2;	//跳线不连接表示铝箱体
											LED_Al_Box;  //平常时候亮
										}
							else{
											//ClrLed2;	//跳线连接表示铁箱体
											LED_Fe_Box;  //平常时候暗
									}
							#endif					
							//LED_RECV_OFF;	
							}		
						
#if CONFIG_433SG						
				if(Time_TestProc_LED1>0)
						{
							Time_TestProc_LED1--;
							if(Time_TestProc_LED1==0)
									{
										TestProc_LED1_OFF;	
									}
						}
				if(Time_TestProc_LED2>0)
						{
							Time_TestProc_LED2--;
							if(Time_TestProc_LED2==0)
										{
											TestProc_LED2_OFF;	
										}
						}
		
#endif
}



/////////


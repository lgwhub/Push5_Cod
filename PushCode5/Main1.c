#include "config.h"
#include "nomal.h"
#include "Soft.h"
#include "Mega48_Adc.h"

#include "CC1100.h"




#include "Hd_ElecPush3.H"

	#if IAR_SYSTEM
		#include "iom88v.h"
        	#include "IAR_VECT.h"
	#else
		#include <iom88v.h>
	#endif


#if CONFIG_UART
	#include "LoopBuf.h"
#endif
#if CONFIG_433SG
  #include "Decode.h"
		void Init_ext_Int1(void);
	#endif

void Init_Hd(void);

void InitOutLevelDefault(void);
//////////////////////////////
//void TestUart2(void);



 
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
void LedStart(void)   /* ����ʱָʾ����˸  */
{
	/*
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
	*/
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
//ClrLed2;
}


void main(void)
{
	uchar tim88;
	uchar ReSendCount;
	
InitOutLevelDefault();
gpParam=(struct	struct_save	*)&gbParamBuf[0];		//�����ṹ��ָ��


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
Load_Param();		//ȡ�趨ֵ
#endif

Init_Hd();
InitAdc();
LedStart();

#if CONFIG_UART
/*
***Takeup6**
&ED,I1=048,I2=049,I3=050,ID=0000,
&EC,I=000,In=00,Pu=0208
&ER,3132333435
*/
SendText_UART0("***Push3**\r\n");
ParamSend();
AutoSend();
		#if CONFIG_433SG
				RemotCodeSend("12345",5);
		#endif
#endif
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
#if Config_Al_Box	
Led2_Enable;	//ѡ��������ָʾ��SetLed2

//ѡ������������
K4_PULLUP;
Select_Enable;
Select_Low;
#endif
//
		#if CONFIG_433SG == 0
				#if CONFIG_TEST_FREQ_PIN
				Enable_TEST_FREQ_PIN;	//��Ƭ��Ƶ�ʲ���Not_TEST_FREQ_PIN;
				#endif
	#endif				
}
////
#if CONFIG_SAVE5
void EEPROM_Write(unsigned int iAdr,unsigned char bDat)
{
	uint16 i;
	i=0;
	while( (i<30000) && (EECR & (1<<EEWE)) )
		{//�ȴ�ǰһ�β������
			i++;
		}
EEAR = iAdr;
EEDR	=	bDat;
EECR	|=	(1<<EEMWE);		//дEEMWEλ������EEPROM����
EECR	|=	(1<<EEWE);		//дEEWEλ����ʼEEPROM����
	
}
////////////


unsigned char EEPROM_Read(unsigned int iAdr)
{
	uint16 i;
	i=0;
	while( (i<30000) && (EECR & (1<<EEWE)) )
		{//�ȴ�ǰһ�β������
			i++;
		}
EEAR = iAdr;
EECR	|=	(1<<EERE);		//��EEWEλ����ʼEEPROM����
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
gbParamBuf[Max_Param_Len-1]=~Sum;	//У������
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
			{//�����Ѿ���ʼ�������ȷ
			
			Sum=0;
			for(i=0;i<Max_Param_Len;i++)
				{
				Sum+=buf[i];//У����
				}
			if(Sum==0xff)//У����
				{
					FlagOk=1;
					break;
				}
			}
		}
		
	if(p->flag!=FlagParamInitnized)
		{//EEPROM������ʼ��
			if((p->flag==0xff)||(p->flag==0))
							{
							Write_Param();
							}
			}
					
if(FlagOk)	
			{		
			for(i=0;i<Max_Param_Len;i++)
				{//��������
				gbParamBuf[i]=buf[i];
				}
			}
}
#endif
////
#if CONFIG_UART
void Init_Uart(unsigned int baud,unsigned char parity)
{


	DDRD	&=	(~BIT0);	//RXD
	
	PORTD	|=	BIT0;			//����

	
	DDRD	|=	BIT1;			//TXD
	
	
	
UCSR0B	=	0X0;				//��ֹ���ͺͽ���
UCSR0A	|=	(1<<U2X0);	//0X02;				//���� U2X0


		switch	(parity)
			{
				case 1:
					UCSR0C	=	0X36;	//0XB6;				//8����λ,��У��	
				break;
				case 2:
					UCSR0C	=	0X26;	//0XA6;				//8����λ,żУ��	
				break;
				case 0:
				default:
					UCSR0C	=	0X06;	//0X86;				//8����λ,��У��
				break;
				}
UBRR0L	=	baud&0XFF;
UBRR0H	=	(baud>>8)&0XFF;


//�ж�
	UCSR0B	|=	(1<<RXEN0)+(1<<TXEN0);				//������պͽ���
	UCSR0B	|=	(1<<RXCIE0);				//��������ж�
	//UCSR0B	|=	(1<<UDRIE0);				//�����ͻ��������ж�
	UCSR0B	|=	(1<<TXCIE0);				//����������ж�
	
	

}
#endif

/////

#if CONFIG_433SG
void Init_ext_Int1(void)//ң���������ж�
{
PORTD	|=	BIT3;		//EX_INT1	PORTD3
DDRD	&=(~BIT3);	//����
EICRA	|=	(1<<ISC11);	//�½���
EIMSK	|=	BIT1;	//MEGA88
}
#endif
/////////////////
void Init_Timer0(void)
{
	TCNT0	=	0;		//������
	//TCCR0B	=	0X03;	//ICNC1��׽��������  /64
	//8bit ��ʱ��
	//TCCR0B	=	0X05;	//ICNC1��׽��������  /1024
	TCCR0B	=	0X04;	//ICNC1��׽��������  /256  //ң����������Ҫ
	//TIMSK0	|=	(1<<OCIE0A)+(1<<OCIE0B)+(1<<TOIE0);
	TIMSK0	|=	(1<<OCIE0B)+(1<<TOIE0);
}




/////

void PwmContral1(uchar x)	//OC1A PWM1 ����
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
	else{//�Ƚ�ƥ��ʱ����
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
void PwmContral2(uchar x)	//OC1B PWM2 ����
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



	TCNT1	=	0;		//������
	//TCCR1B	|=	(1<<ICNC1)+0X03;	//ICNC1��׽��������  /64
	//PWMģʽ 1000
	TCCR1B	|=	(1<<WGM13);
	//TCCR1B	|=	(1<<WGM12);
	//TCCR1A	|=	(1<<WGM11);
	//TCCR1A	|=	(1<<WGM10);
	
	//OC1A,OC1Bģʽ
	//TCCR1A	|=	(1<<COM1A1);
	//TCCR1A	&=	( ~ (1<<COM1A0 ));
	//TCCR1A	|=	(1<<COM1B1);
	//TCCR1A	&=	( ~ (1<<COM1B0 ));
	
	//ֵ
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
void Init_Hd(void)
{

	/*
	CLKPR=0X80;	//1000 0000����Ƶ
//	CLKPR=0X10;	//1000 0000
	CLKPR=0X00;	//0000 0000
	*/
	
	CLKPR=0X80;	//1000 0000	����Ƶ
	//CLKPR=0X01;	//1000 0001
	CLKPR=0X01;	//0000 0001
	
	
	
	Init_GPIO();
	#if CONFIG_UART
	//�ڲ�RC����Ƶ��У׼
	//OSCCAL = EEPROM_Read(EEPROM_OSCCAL_ADR);

	#if	CONFIG_OSSCAL
	if(OSSCAL_AT_MEGA48_FLASH!=0xff)
			{
			OSCCAL = OSSCAL_AT_MEGA48_FLASH;
			}
	#endif
		
	/////
	ClearLoopBuf(&Uart0SendStruct,UART0_SEND_BUF_SIZE);	//������ͻ�����
	//Init_Uart(BAUD_DEFINE(9600),0);
//	Init_Uart(BAUD_DEFINE(9400),0);
Init_Uart(BAUD_DEFINE(9200),0);
	#endif
	
	//InitAdc();
	Init_Timer0();
	Init_Timer1_PWM();
	#if CONFIG_433SG
	Init_ext_Int1();	//ң���������ж�
	#endif
SREG	|=	0X80;					//ʹ�����ж�
}
///////////////





/////
#if CONFIG_UART
uchar Uart0CharSend(uchar x)
{
	

	uchar fail=0;
	uchar t;
	
    if(Uart0SendStruct.len>3)
	    	{//full
	    	t=5;
	    	while((Uart0SendStruct.addTail == Uart0SendStruct.outTail)&&(t>0))
	    		{
	    		//OSTimeDly(OS_TICKS_PER_SEC / 200);	//��ʱ5����	
	    		DelayMs(5);
	    		t--;
	    		}
	    	}
	    if(t==0)
	    	fail=1;
				
	//OS_ENTER_CRITICAL();	//ucos2   
	AddLoopBuf(&Uart0SendStruct,Uart0SendBuf,UART0_SEND_BUF_SIZE,x);	//���뵽���ͻ�����
	//OS_EXIT_CRITICAL();          //ucos2  	
	

	
	if(UCSR0A&(1<<UDRE0))	//��������
	             {
	              // Set_En485; 	//485������
	              //OS_ENTER_CRITICAL();	//ucos2
	        				UCSR0B	|=	(1<<UDRIE0);				//�����ͻ��������ж�
									//UCSR0B	|=	(1<<TXCIE0);				//����������ж�
	        			//OS_EXIT_CRITICAL();          //ucos2  	
	             }
	             
	          
	             
	return (!fail);;
}
#endif 
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
		uchar temp;	//��ǰʱ��
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
#if CONFIG_UART
/*
	unsigned char temp;	
	
	temp	=	UDR0;
	
	
	AddLoopBuf(&Uart0RecvStruct,Uart0RecvBuf,UART0_RECV_BUF_SIZE,temp);	//���뵽���ͻ�����
*/
#endif	


}
//////////////////

#if IAR_SYSTEM
#pragma vector=USART_UDRE_vect	
__interrupt void uart0_udre_isr(void)
#else
#pragma interrupt_handler uart0_udre_isr:iv_USART0_UDRE		//20�������ݼĴ�����
void uart0_udre_isr(void)
#endif

{//20�������ݼĴ�����
	
#if CONFIG_UART
	unsigned char temp;
	

	
	if(GetLoopbuf(&Uart0SendStruct,Uart0SendBuf,UART0_SEND_BUF_SIZE,&temp))	//�ӻ��ͻ�������ȡ	
			{
				//Set_En485; 	//485������
				UDR0	=	temp;
			}
		else{
				UCSR0B	&=	~(1<<UDRIE0);				//��ֹ���ͻ��������ж�
				}
	
#endif	
	

}
//////////////////
#if IAR_SYSTEM
#pragma vector=USART_TX_vect	
__interrupt void uart0_txc_isr(void)	
#else
#pragma interrupt_handler uart0_txc_isr:iv_USART0_TXC		//21�����������
void uart0_txc_isr(void)	
#endif
{
#if CONFIG_UART
	//Clr_En485; 	//485��ֹ����
#endif

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
	bTimeBase	=	1;	//����ʱ�䵽���
				
		#if CONFIG_433SG == 0
				#if CONFIG_TEST_FREQ_PIN
				Not_TEST_FREQ_PIN;	//��Ƭ��Ƶ�ʲ���Enable_TEST_FREQ_PIN;
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
	
	temp	=	ICR1L;	//�ȶ���λ
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
//CPU_MAGE16 MAGE48 ������
//�����...
//ADMUX	=	(1<<ADLAR)+Adc.ch;
temp16 =ADCH;
//�Ҷ���...
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
	
		Adc.ch	=	7;	//PA7��Ϊ�������ߵ��RSSI����
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
	//ADMUX	=	(1<<ADLAR)+Adc.ch;	//�����
	ADMUX	=	(1<<ADLAR)+7;	//ֻ�е�7·ADC
	//ADMUX	=	Adc.ch;
		//if(Adc.number<2*MAX_ADC_NUMBER)
		if(Adc.number<MAX_ADC_NUMBER)			
			{
			Adc.number	++;
			ADCSRA	|=	(1<<ADSC);		//����ת��
			}
	else{
			//ADCSRA	&=	(~(1<<ADEN));
			Adc.complete	=	1;
			//TestAdc_LED1_OFF;
			}
		}
#else			
		unsigned short int temp16;
//CPU_MAGE16 MAGE48 ������
//�����...
//ADMUX	=	(1<<ADLAR)+Adc.ch;
temp16 =ADCH;
//�Ҷ���...
//temp16	=	ADCL;
//temp16	+=	ADCH<<8;

if(Adc.ch<=LAST_ADC_CH)	//Big Error   Adc.sum��ΧΪ6-7
	{
		//Adc.sum[Adc.ch]	+=	temp16;	   //Big Error   Adc.sum��ΧΪ6-7
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
	//ADMUX	=	(1<<ADLAR)+Adc.ch;	//�����

	ADMUX	=	(1<<ADLAR)+Adc.ch;	//�����
		//if(Adc.number<2*MAX_ADC_NUMBER)
		if(Adc.number<(LAST_ADC_CH+1-FIRST_ADC_CH)*MAX_ADC_NUMBER)			
			{
			Adc.number	++;
			ADCSRA	|=	(1<<ADSC);		//����ת��
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
	
	ADCSRA	&=	~(1<<ADEN);		//�ر�ADC
	
	
	ADCSRA	|=	(1<<ADPS2);		//32��Ƶ
	ADCSRA	&=	(~(1<<ADPS1));
	ADCSRA	|=	(1<<ADPS0);	
/*
	ADCSRA	|=	(1<<ADPS2);		//128��Ƶ
	ADCSRA	|=	(1<<ADPS1);
	ADCSRA	|=	(1<<ADPS0);
		*/	
	
	//ADMUX		|=	(1<<REFS0);	//�ڲ��ο�Դ
	//ADMUX		|=	(1<<REFS1);	//�ڲ��ο�Դ
	
	
	


		
	
	//Adc.ch	=	0;
	//Adc.ch	=	6;
	//ADMUX	=	(1<<REFS0)+(1<<REFS1)+Adc.ch;
	//ADMUX	=	(1<<ADLAR)+Adc.ch;	//�����
	ADMUX	=	(1<<ADLAR)+7;	//ֻ�е�7·ADC
	//ADMUX	=	Adc.ch;
	//	|=	(1<<REFS0);	//�ο�ԴVCC
	
	//DIDR0	|=	0x03;
	//DIDR0	|=	0x3f;	//Digital Input Disable Register
	//DIDR0	|=	0x3e;	//Digital Input Disable Register
	//DIDR0	|=	0xc0;	//Digital Input Disable Register  ADC6 ADC7
		DIDR0	|=	0x80;	//Digital Input Disable Register  ADC7	

	
	ADCSRA	|=	(1<<ADEN);	//��ADC
	
}
//////
void StartAdc(void)
{
//CPU_MAGE16 MAGE48 ������
	//ADCSRA	&=	(~(1<<ADEN));		//�ر�ADC

	//Adc.max1	=	0;
	//Adc.min1	=	65535;
	

	//Adc.sum6	=	0;
	Adc.sum7	=	0;		
	Adc.complete	=	0;
	Adc.number	=	1;
	ADMUX	=	(1<<ADLAR)+7;	//ֻ�е�7·ADC
	
	
	ADCSRA	|=	(1<<ADIE);		//�ж�����
	//ADCSRA	|=	(1<<ADEN);		//
	ADCSRA	|=	(1<<ADSC);		//����ת��
	
//	TestAdc_LED1_ON;		//ָʾAD����
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
					{//10A,��ʾ50
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
void AdcProcess(void)	//����10mS
{	
	if(Adc.Time<20)Adc.Time++;
		else {
			Adc.Time=0;
		}
						switch(Adc.Time)
							{
							case 1:	//����ʱ��100MS
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
#if CONFIG_433SG
void LedContral(void)  /* ����ת��  */
{
	
				if(Time_LedRecv_LED>0)
						{
							Time_LedRecv_LED--;
							if(Time_LedRecv_LED==0)
										{
											LED_RECV_OFF;	
										}
						}
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
		

}
#endif

#if CONFIG_UART
void TestUart2(void)
{
//uchar MakeValAsc16(uchar *StrHead,uint16 Val,uchar *StrEnd,uchar *out)	//16λ����ת��Ϊʮ���Ƶ�ASC�룬��ǰ��׺
/*
uchar buf[50];
uchar l;
l=MakeValAsc16("At=",ActiveTime,"S",buf,0);//�ʱ��,�������ֵ����
l+=MakeValAsc16("Nt=",NoChangTime,"S,",&buf[l],0);
l+=MakeValAsc16("V=",iVolt,"x10mV",&buf[l],0);

l		+=	PutString("\r\n",&buf[l],5);	
SendText_UART0(buf);
*/
}
#endif
/////////


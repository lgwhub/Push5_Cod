
//Mega48_Adc.c

#include "config.h"
#include "nomal.h"
#include "Soft.h"
#include "Mega48_Adc.h"



#include "Hd_ElecPush3.H"

	#if IAR_SYSTEM
		#include "iom88v.h"
        	#include "IAR_VECT.h"
	#else
		#include <iom88v.h>
	#endif

//uchar chAdc_Resoult1;
uchar chAdc_Resoult6;
uchar chAdc_Resoult7;

#if old_adc == 0

const uchar BIT_TAB8[8]={BIT0,BIT1,BIT2,BIT3,BIT4,BIT5,BIT6,BIT7};

struct adc_struct Adc;


void InitAdc(void)
{
	uchar i;
	
	ADCSRA	&=	~(1<<ADEN);		//关闭ADC
	
	
	ADCSRA	|=	(1<<ADPS2);		//32分频
	ADCSRA	&=	(~(1<<ADPS1));
	ADCSRA	|=	(1<<ADPS0);	
/*
	ADCSRA	|=	(1<<ADPS2);		//128分频
	ADCSRA	|=	(1<<ADPS1);
	ADCSRA	|=	(1<<ADPS0);
		*/	
	
	ADMUX		=	FIRST_ADC_CH;		//开始的ADC通道0-7
	//ADMUX		|=	(1<<REFS0);	//内部参考源
	//ADMUX		|=	(1<<REFS1);	//内部参考源
	ADMUX	|=	(1<<ADLAR);	//左对齐
	
	
	for(i=FIRST_ADC_CH;i<=LAST_ADC_CH;i++)
			{
			DIDR0	|=BIT_TAB8[i];
			}
	
	ADCSRA	|=	(1<<ADEN);	//开ADC
	
}
//////
void StartAdc(void)
{
uchar i;
//CPU_MAGE16 MAGE48 都可以
	//ADCSRA	&=	(~(1<<ADEN));		//关闭ADC

	//Adc.max1	=	0;
	//Adc.min1	=	65535;
	
	
		//Big Error   Adc.sum范围为6-7
//for(i=FIRST_ADC_CH;i<=LAST_ADC_CH;i++)
//		{
//			Adc.sum[i]	=	0;		
//		}
//		

		//Big Error  clred
//for(i=0; i <= FIRST_ADC_CH - LAST_ADC_CH; i++)
for(i=0; i <= LAST_ADC_CH - FIRST_ADC_CH; i++)
		{
			Adc.sum[i]	=	0;		
		}
//////////////////////////////////

	Adc.complete	=	0;
	Adc.number	=	1;
	Adc.ch=FIRST_ADC_CH;
	
	ADMUX		=	FIRST_ADC_CH;		//开始的ADC通道0-7
	//ADMUX		|=	(1<<REFS0);	//内部参考源
	//ADMUX		|=	(1<<REFS1);	//内部参考源
	ADMUX	|=	(1<<ADLAR);	//左对齐
	
	
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
	

		//	temp16=(Adc.sum[6]>>4);	
			//if(temp16>128)temp16=128;		//32/128=25%
		//	chAdc_Resoult6=(uchar)(temp16);	
			

		
			
			//temp16=((Adc.sum7>>4)*33)>>8;	//(UINT=100mA)
			//temp16=((Adc.sum7>>4)*330)>>8;	//(UINT=10mA) max=1980mA
			//temp16=(Adc.sum[7]>>4);				//Big Error   Adc.sum范围为6-7
			temp16=(Adc.sum[LAST_ADC_CH - FIRST_ADC_CH]>>4);		//Big Error  clred
			if(FlagSetCurrent1==2)
					{//10A,显示50
				if(temp16>25)		//相当于RATE<256
							{
							xxx=10*5*128/(temp16);
							if  (xxx>60)	//相当于temp16<
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
				if (temp16>(32768/gpParam->bCurrentRate))temp16=(32768/gpParam->bCurrentRate);
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

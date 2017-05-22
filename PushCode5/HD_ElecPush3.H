#ifndef _HD_ELECPUSH3_H
#define _HD_ELECPUSH3_H

#define OSSCAL_AT_MEGA48_FLASH	(*(const unsigned char *)(0xfff))
//#define OSSCAL_AT_MEGA48_FLASH2	(*(const unsigned int *)(0xfff))
#define Enable_TEST_FREQ_PIN	DDRD|=BIT3
#define Not_TEST_FREQ_PIN		PORTD^=BIT3

//out
#define JspOut1_Enable 		DDRB|=BIT2
#define JspOut1_OFF			PORTB&=(~BIT2)
#define JspOut1_ON			PORTB|=BIT2
#define JspOut2_Enable 		DDRD|=BIT5
#define JspOut2_OFF			PORTD&=(~BIT5)
#define JspOut2_ON			PORTD|=BIT5


//PWM_OUT
#define SW_PWM1_ENABLE 		DDRB|=BIT1
#define SW_PWM1_ON			PORTB|=BIT1
#define SW_PWM1_OFF			PORTB&=(~BIT1)


//input 接近开关
//#define InputPort P2


//OC1A(PB1)	PWM输出
//adc7:		保护电流输入
//int1(PD3):	脉冲输入中断

#define K1_LVL	(PINC&BIT0)
#define K1_PULLUP	(PORTC|=BIT0)
#define K2_LVL	(PINC&BIT1)
#define K2_PULLUP	(PORTC|=BIT1)

//CSN
#define K3_LVL	1//(PIND&BIT4)
#define K3_PULLUP	;//(PORTD|=BIT4)

//select
#define K4_LVL	1//(PIND&BIT7)
#define K4_PULLUP	;//(PORTD|=BIT7)

#define Select_Enable 		;//DDRC|=BIT3
#define Select_Low			;//PORTC&=(~BIT3)

/*
//CSN
#define K3_LVL	(PIND&BIT4)
#define K3_PULLUP	(PORTD|=BIT4)

//select
#define K4_LVL	(PIND&BIT7)
#define K4_PULLUP	(PORTD|=BIT7)

#define Select_Enable 		DDRC|=BIT3
#define Select_Low			PORTC&=(~BIT3)
*/


#define Led1_Enable 		DDRB|=BIT0
#define ClrLed1			PORTB&=(~BIT0)
#define SetLed1			PORTB|=BIT0

#define Led2_Enable 		DDRC|=BIT5
#define ClrLed2			PORTC&=(~BIT5)
#define SetLed2			PORTC|=BIT5
//#define NotLed2			PORTC^=BIT5


///////////////////////////
//CC1100 IO
/*******************IO定义(根据具体情况重新定义)*******************/
//输出IO方向寄存器
#define CC1100_SCLK_ENABLE  DDRC|=BIT3
#define CC1100_CSN_ENABLE   DDRD|=BIT4
#define CC1100_SI_ENABLE    DDRD|=BIT7

//IO数据输出寄存器
#define SET_CC1100_SCLK  PORTC|=BIT3
#define SET_CC1100_SO    PORTC|=BIT4
#define SET_CC1100_GDO2  PORTD|=BIT3
#define SET_CC1100_GDO0  PORTD|=BIT6
#define SET_CC1100_CSN   PORTD|=BIT4
#define SET_CC1100_SI    PORTD|=BIT7

#define CLR_CC1100_SCLK  PORTC&=(~BIT3)
#define CLR_CC1100_SO    PORTC&=(~BIT4)
#define CLR_CC1100_GDO2  PORTD&=(~BIT3)
#define CLR_CC1100_GDO0  PORTD&=(~BIT6)
#define CLR_CC1100_CSN   PORTD&=(~BIT4)
#define CLR_CC1100_SI    PORTD&=(~BIT7)

//IO读管脚寄存器
#define CC1100_SO_LVL    (PINC&BIT4)
#define CC1100_GDO2_LVL  (PIND&BIT3)
#define CC1100_GDO0_LVL  (PIND&BIT6)
/*******************IO初始化(根据具体情况重新定义)********************/
#define cc1100_IO_INITIALIZTION()  \
CC1100_SCLK_ENABLE;\
CC1100_CSN_ENABLE;\
CC1100_SI_ENABLE;\
CLR_CC1100_SCLK;\
SET_CC1100_SO;\
SET_CC1100_GDO2;\
SET_CC1100_GDO0;\
SET_CC1100_CSN;\
CLR_CC1100_SI

#endif

    



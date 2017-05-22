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
#define K3_LVL	(PIND&BIT4)
#define K3_PULLUP	(PORTD|=BIT4)
//select
#define K4_LVL	(PIND&BIT7)
#define K4_PULLUP	(PORTD|=BIT7)
#define K5_LVL	1
#define K6_LVL	1
#define K7_LVL	1
#define K8_LVL	1

#define Select_Enable 		DDRC|=BIT3
#define Select_Low			PORTC&=(~BIT3)

#define Led1_Enable 		DDRB|=BIT0
#define ClrLed1			PORTB&=(~BIT0)
#define SetLed1			PORTB|=BIT0

#define Led2_Enable 		DDRC|=BIT5
#define ClrLed2			PORTC&=(~BIT5)
#define SetLed2			PORTC|=BIT5
//#define NotLed2			PORTC^=BIT5




#endif

    



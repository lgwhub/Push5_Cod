#ifndef _HD_TAKEUP6_H
#define _HD_TAKEUP6_H



//out
#define JspOut1_Enable 		DDRD|=BIT2
#define JspOut1_OFF			PORTD&=(~BIT2)
#define JspOut1_ON			PORTD|=BIT2
#define JspOut2_Enable 		DDRD|=BIT4
#define JspOut2_OFF			PORTD&=(~BIT4)
#define JspOut2_ON			PORTD|=BIT4
#define JspOut3_Enable 		DDRD|=BIT5
#define JspOut3_OFF			PORTD&=(~BIT5)
#define JspOut3_ON			PORTD|=BIT5
#define JspOut4_Enable 		DDRD|=BIT6
#define JspOut4_OFF			PORTD&=(~BIT6)
#define JspOut4_ON			PORTD|=BIT6
#define JspOut5_Enable 		DDRB|=BIT2
#define JspOut5_OFF			PORTB&=(~BIT2)
#define JspOut5_ON			PORTB|=BIT2
#define JspOut6_Enable 		DDRD|=BIT7
#define JspOut6_OFF			PORTD&=(~BIT7)
#define JspOut6_ON			PORTD|=BIT7

/*
//j9  led
sbit P10=P1^0;
sbit P11=P1^1;

//j10
sbit P12=P1^2;
sbit P13=P1^3;

//8
sbit P14=P1^4;
sbit P15=P1^5;

//j4
sbit P30=P3^0;
sbit P31=P3^1;
*/

//input 接近开关
//#define InputPort P2
#define Sen1	(PINC&BIT1)
#define Sen2	(PINC&BIT2)
#define Sen3	(PINC&BIT3)
#define Sen4	(PINC&BIT4)
#define Sen5	(PINC&BIT5)
#define Sen6	(PINC&BIT6)


//OC1A(PB1)	PWM输出
//adc7:		保护电流输入
//int1(PD3):	脉冲输入中断

//#define K1_LVL	(PINC&BIT0)
//#define K1_PULLUP	(PORTC|=BIT0)
#define K1_LVL	(PINB&BIT3)
#define K1_PULLUP	(PORTB|=BIT3)
#define K2_LVL	1
#define K3_LVL	1
#define K4_LVL	1
#define K5_LVL	1
#define K6_LVL	1
#define K7_LVL	1
#define K8_LVL	1


#define Led1_Enable 		DDRB|=BIT4
#define ClrLed1			PORTB&=(~BIT4)
#define SetLed1			PORTB|=BIT4

#define Led2_Enable 		;//DDRC|=BIT0
#define ClrLed2			;//PORTC&=(~BIT0)
#define SetLed2			;//PORTC|=BIT0


#endif

    


